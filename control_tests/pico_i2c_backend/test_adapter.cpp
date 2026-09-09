#include "pico_i2c_esp_backend.h"
#include "pico_i2c_native.h"
#include <algorithm>
#include <array>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <vector>

using namespace pico_i2c;
#define CHECK(value) do { if (!(value)) { std::cerr << __func__ << ':' << __LINE__ << ": " #value "\n"; std::abort(); } } while (false)

namespace {
constexpr uint32_t expectedRevision = 0x05050401u;
constexpr BusConfig validBus{1, 2, 100000};
constexpr uint64_t futureDeadline = uint64_t{1} << 40;

// This fake only controls the synchronous C API's terminal result and context
// ownership. It does NOT simulate commands, ACKs, time, bus recovery or silicon.
struct Native {
    uint32_t revision = expectedRevision;
    unsigned revisions = 0, opens = 0, transfers = 0, closes = 0;
    wbb_i2c_result openResult{WBB_I2C_OK, 0};
    wbb_i2c_result transferResult{WBB_I2C_OK, 0};
    wbb_i2c_result closeResult{WBB_I2C_OK, 0};
    bool allocateOnOpen = true, allocated = false;
    int token = 0;
    BusConfig openedConfig{};
    int64_t openedDeadline = 0, transferredDeadline = 0;
    uint8_t address = 0;
    uintptr_t txPointer = 0, rxPointer = 0, closedContext = 0;
    std::vector<uint8_t> tx;
    std::size_t rxSize = 0, errorRxBytes = 0;
};
Native* active = nullptr;

void unknown(const Result& result, Code code, int sdkError = 0) {
    CHECK(result.code == code);
    CHECK(result.evidence == CountEvidence::Unknown);
    // Numeric placeholders remain zero, but Unknown explicitly disclaims any
    // measurement that zero bytes reached the target or caller buffer.
    CHECK(result.transmitted == 0 && result.received == 0);
    CHECK(result.backendError == sdkError);
}
void complete(const Result& result, std::size_t tx, std::size_t rx) {
    CHECK(result.code == Code::Complete);
    CHECK(result.evidence == CountEvidence::BackendCompletion);
    CHECK(result.transmitted == tx && result.received == rx);
}
struct Fixture {
    Native native;
    EspIdfBackend backend;
    Fixture() { CHECK(active == nullptr); active = &native; }
    ~Fixture() {
        // Explicit successful teardown after each test's lifecycle assertions.
        native.closeResult = {WBB_I2C_OK, 0};
        backend.shutdown();
        CHECK(!native.allocated);
        active = nullptr;
    }
    void configure(uint64_t deadline = futureDeadline) {
        unknown(backend.configure(validBus, deadline), Code::Complete);
        CHECK(native.allocated);
    }
};
} // namespace

extern "C" uint32_t wbb_i2c_backend_revision(void) {
    CHECK(active); ++active->revisions; return active->revision;
}
extern "C" wbb_i2c_result wbb_i2c_open(void** context, int sda, int scl,
                                       uint32_t hz, int64_t deadline) {
    CHECK(active && context && *context == nullptr && !active->allocated);
    ++active->opens;
    active->openedConfig = {sda, scl, hz};
    active->openedDeadline = deadline;
    if (active->allocateOnOpen) { *context = &active->token; active->allocated = true; }
    return active->openResult;
}
extern "C" wbb_i2c_result wbb_i2c_transfer(void* context, uint8_t address,
        const uint8_t* tx, size_t txSize, uint8_t* rx, size_t rxSize, int64_t deadline) {
    CHECK(active && active->allocated && context == &active->token);
    CHECK(tx && txSize && (rx || !rxSize));
    ++active->transfers;
    active->address = address;
    active->txPointer = reinterpret_cast<uintptr_t>(tx);
    active->rxPointer = reinterpret_cast<uintptr_t>(rx);
    active->tx.assign(tx, tx + txSize);
    active->rxSize = rxSize;
    active->transferredDeadline = deadline;
    const auto written = active->transferResult.code == WBB_I2C_OK
        ? rxSize : std::min(rxSize, active->errorRxBytes);
    for (std::size_t i = 0; i < written; ++i) rx[i] = uint8_t(0x80u + i);
    return active->transferResult;
}
extern "C" wbb_i2c_result wbb_i2c_close(void** context) {
    CHECK(active && context);
    CHECK(*context == (active->allocated ? &active->token : nullptr));
    ++active->closes;
    active->closedContext = reinterpret_cast<uintptr_t>(*context);
    if (active->closeResult.code == WBB_I2C_OK) {
        *context = nullptr;
        active->allocated = false;
    }
    return active->closeResult;
}

namespace {
void configure_guards_and_exact_bus() {
    Fixture f;
    uint8_t tx = 0;
    unknown(f.backend.transfer({0x08, &tx, 1, nullptr, 0}, futureDeadline), Code::NotReady);
    for (BusConfig bad : {BusConfig{2, 1, 100000}, {0, 2, 100000}, {1, 0, 100000},
                          {-1, 2, 100000}, {1, -1, 100000}, {1, 2, 0},
                          {1, 2, 99999}, {1, 2, 100001}, {1, 2, 400000}}) {
        unknown(f.backend.configure(bad, futureDeadline), Code::InvalidArgument);
    }
    CHECK(f.native.revisions == 0 && f.native.opens == 0 && f.native.transfers == 0);
    f.configure();
    CHECK(f.native.openedConfig.sda == 1 && f.native.openedConfig.scl == 2);
    CHECK(f.native.openedConfig.frequencyHz == 100000);
    CHECK(f.native.openedDeadline == int64_t(futureDeadline));
    unknown(f.backend.configure(validBus, futureDeadline), Code::InvalidArgument);
    CHECK(f.native.opens == 1 && f.native.revisions == 1 && f.native.closes == 0);
    complete(f.backend.transfer({0x08, &tx, 1, nullptr, 0}, futureDeadline), 1, 0);
}

void allowed_addresses_and_complete_frame_counts() {
    Fixture f; f.configure();
    std::array<uint8_t, 69> tx;
    std::array<uint8_t, 70> rx;
    for (unsigned i = 0; i < tx.size(); ++i) tx[i] = uint8_t(i ^ 0x5A);
    for (unsigned address = 0; address <= 255; ++address) {
        const auto before = f.native.transfers;
        rx.fill(0xCC);
        Result result = f.backend.transfer({uint8_t(address), tx.data(), tx.size(), rx.data() + 1, 68}, futureDeadline);
        if (address == 0x08 || address == 0x40) {
            complete(result, 69, 68);
            CHECK(f.native.transfers == before + 1 && f.native.address == address);
            CHECK(f.native.tx == std::vector<uint8_t>(tx.begin(), tx.end()));
            CHECK(f.native.txPointer == reinterpret_cast<uintptr_t>(tx.data()));
            CHECK(f.native.rxPointer == reinterpret_cast<uintptr_t>(rx.data() + 1));
            CHECK(f.native.rxSize == 68 && f.native.transferredDeadline == int64_t(futureDeadline));
            for (unsigned i = 0; i < 68; ++i) CHECK(rx[i + 1] == uint8_t(0x80u + i));
        } else {
            unknown(result, Code::InvalidArgument);
            CHECK(f.native.transfers == before);
            CHECK(rx[1] == 0xCC);
        }
        CHECK(rx.front() == 0xCC && rx.back() == 0xCC);
    }
    complete(f.backend.transfer({0x40, tx.data(), 3, nullptr, 0}, futureDeadline), 3, 0);
    CHECK(f.native.opens == 1 && f.native.closes == 0);
}

void transfer_bounds_do_not_call_c_or_latch_fault() {
    Fixture f; f.configure(); uint8_t tx[70]{}, rx[69]{};
    const Transfer invalid[] = {
        {0x08, nullptr, 1, nullptr, 0}, {0x08, tx, 0, nullptr, 0},
        {0x08, tx, 70, nullptr, 0}, {0x08, tx, std::numeric_limits<std::size_t>::max(), nullptr, 0},
        {0x40, tx, 1, nullptr, 2}, {0x40, tx, 1, rx, 69},
        {0x40, tx, 1, rx, std::numeric_limits<std::size_t>::max()}
    };
    for (const auto& request : invalid) {
        unknown(f.backend.transfer(request, futureDeadline), Code::InvalidArgument);
    }
    CHECK(f.native.transfers == 0 && f.native.closes == 0);
    complete(f.backend.transfer({0x40, tx, 1, rx, 2}, futureDeadline), 1, 2);
}

void deadline_representation_and_delegation() {
    const uint64_t signedMax = uint64_t(std::numeric_limits<int64_t>::max());
    for (uint64_t invalid : {uint64_t{0}, signedMax + 1, std::numeric_limits<uint64_t>::max()}) {
        Fixture f;
        unknown(f.backend.configure(validBus, invalid), Code::InvalidArgument);
        CHECK(f.native.opens == 0 && f.native.revisions == 0);
        f.configure(); uint8_t tx = 0;
        unknown(f.backend.transfer({0x08, &tx, 1, nullptr, 0}, invalid), Code::InvalidArgument);
        CHECK(f.native.transfers == 0);
        complete(f.backend.transfer({0x08, &tx, 1, nullptr, 0}, futureDeadline), 1, 0);
    }
    // The adapter has no clock: representable absolute deadlines, including 1,
    // pass unchanged to C. These tests do not pretend to enforce elapsed time.
    for (uint64_t deadline : {uint64_t{1}, (uint64_t{1} << 32) + 7, signedMax}) {
        Fixture f; f.configure(deadline); uint8_t tx = 0;
        complete(f.backend.transfer({0x08, &tx, 1, nullptr, 0}, deadline), 1, 0);
        CHECK(f.native.openedDeadline == int64_t(deadline));
        CHECK(f.native.transferredDeadline == int64_t(deadline));
    }
}

void nack_has_unknown_counts_no_retry_and_allows_new_transfer() {
    Fixture f; f.configure(); uint8_t tx[3]{}, rx[2]{0xCC, 0xCC};
    f.native.transferResult = {WBB_I2C_NAK, -71};
    f.native.errorRxBytes = 1;
    for (unsigned i = 1; i <= 12; ++i) {
        unknown(f.backend.transfer({0x40, tx, 3, rx, 2}, futureDeadline), Code::Nak, -71);
        CHECK(f.native.transfers == i && f.native.opens == 1 && f.native.closes == 0);
        CHECK(rx[0] == 0x80 && rx[1] == 0xCC); // Simulated partial side effect, counts still Unknown.
    }
    f.native.transferResult = {WBB_I2C_OK, 0};
    complete(f.backend.transfer({0x40, tx, 3, rx, 2}, futureDeadline), 3, 2);
    CHECK(f.native.transfers == 13 && rx[1] == 0x81);
}

void fatal_transfer_requires_explicit_shutdown_and_configure() {
    for (auto failure : {wbb_i2c_result{WBB_I2C_TIMEOUT, -72}, {WBB_I2C_ERROR, -73}}) {
        Fixture f; f.configure(); uint8_t tx = 0, rx[2]{0xCC, 0xCC};
        f.native.transferResult = failure; f.native.errorRxBytes = 1;
        unknown(f.backend.transfer({0x08, &tx, 1, rx, 2}, futureDeadline),
                failure.code == WBB_I2C_TIMEOUT ? Code::Timeout : Code::BusError, failure.sdk_error);
        CHECK(rx[0] == 0x80 && rx[1] == 0xCC);
        f.native.transferResult = {WBB_I2C_OK, 0};
        for (unsigned i = 0; i < 3; ++i)
            unknown(f.backend.transfer({0x40, &tx, 1, rx, 2}, futureDeadline), Code::NotReady);
        CHECK(f.native.transfers == 1 && f.native.opens == 1 && f.native.closes == 0);
        unknown(f.backend.configure(validBus, futureDeadline), Code::InvalidArgument);
        CHECK(f.native.opens == 1 && f.native.allocated);
        f.backend.shutdown(); unknown(f.backend.shutdownResult(), Code::Complete);
        CHECK(!f.native.allocated && f.native.closes == 1);
        unknown(f.backend.transfer({0x08, &tx, 1, rx, 2}, futureDeadline), Code::NotReady);
        f.configure();
        complete(f.backend.transfer({0x08, &tx, 1, rx, 2}, futureDeadline), 1, 2);
        CHECK(f.native.transfers == 2 && f.native.opens == 2);
    }
}

void native_invalid_argument_does_not_fabricate_completion() {
    Fixture f; f.configure(); uint8_t tx = 0;
    f.native.transferResult = {WBB_I2C_INVALID, -74};
    unknown(f.backend.transfer({0x08, &tx, 1, nullptr, 0}, futureDeadline), Code::InvalidArgument, -74);
    CHECK(f.native.transfers == 1 && f.native.closes == 0);
    f.native.transferResult = {WBB_I2C_OK, 0};
    complete(f.backend.transfer({0x08, &tx, 1, nullptr, 0}, futureDeadline), 1, 0);
    CHECK(f.native.transfers == 2);
}

void partial_config_keeps_context_until_explicit_shutdown() {
    for (auto failure : {wbb_i2c_result{WBB_I2C_ERROR, -81}, {WBB_I2C_TIMEOUT, -82},
                         {WBB_I2C_NAK, -83}, {WBB_I2C_INVALID, -84}}) {
        Fixture f; f.native.openResult = failure;
        const Code expected = failure.code == WBB_I2C_ERROR ? Code::BusError :
            failure.code == WBB_I2C_TIMEOUT ? Code::Timeout :
            failure.code == WBB_I2C_NAK ? Code::Nak : Code::InvalidArgument;
        unknown(f.backend.configure(validBus, futureDeadline), expected, failure.sdk_error);
        CHECK(f.native.allocated && f.native.opens == 1 && f.native.closes == 0);
        uint8_t tx = 0;
        unknown(f.backend.transfer({0x08, &tx, 1, nullptr, 0}, futureDeadline), Code::NotReady);
        unknown(f.backend.configure(validBus, futureDeadline), Code::InvalidArgument);
        CHECK(f.native.opens == 1 && f.native.transfers == 0);
        f.backend.shutdown();
        CHECK(f.native.closedContext == reinterpret_cast<uintptr_t>(&f.native.token));
        CHECK(!f.native.allocated && f.native.closes == 1);
        f.native.openResult = {WBB_I2C_OK, 0}; f.configure();
        complete(f.backend.transfer({0x08, &tx, 1, nullptr, 0}, futureDeadline), 1, 0);
    }
    // Failure before allocation leaves nothing to leak or block a new configure.
    Fixture f; f.native.openResult = {WBB_I2C_ERROR, -85}; f.native.allocateOnOpen = false;
    unknown(f.backend.configure(validBus, futureDeadline), Code::BusError, -85);
    CHECK(!f.native.allocated && f.native.closes == 0);
    f.native.openResult = {WBB_I2C_OK, 0}; f.native.allocateOnOpen = true; f.configure();
    CHECK(f.native.opens == 2);
}

void shutdown_failure_keeps_context_and_visible_error() {
    for (bool partial : {false, true}) {
        for (auto failure : {wbb_i2c_result{WBB_I2C_ERROR, -91}, {WBB_I2C_TIMEOUT, -92}}) {
            Fixture f;
            if (partial) {
                f.native.openResult = {WBB_I2C_ERROR, -93};
                unknown(f.backend.configure(validBus, futureDeadline), Code::BusError, -93);
            } else f.configure();
            f.native.closeResult = failure; f.backend.shutdown();
            const Code expected = failure.code == WBB_I2C_TIMEOUT ? Code::Timeout : Code::BusError;
            unknown(f.backend.shutdownResult(), expected, failure.sdk_error);
            CHECK(f.native.allocated && f.native.closes == 1);
            CHECK(f.native.closedContext == reinterpret_cast<uintptr_t>(&f.native.token));
            uint8_t tx = 0;
            unknown(f.backend.transfer({0x08, &tx, 1, nullptr, 0}, futureDeadline), Code::NotReady);
            unknown(f.backend.configure(validBus, futureDeadline), Code::InvalidArgument);
            unknown(f.backend.shutdownResult(), expected, failure.sdk_error);
            CHECK(f.native.opens == 1 && f.native.transfers == 0 && f.native.closes == 1);
            f.native.closeResult = {WBB_I2C_OK, 0}; f.backend.shutdown();
            unknown(f.backend.shutdownResult(), Code::Complete);
            CHECK(!f.native.allocated && f.native.closes == 2);
            f.native.openResult = {WBB_I2C_OK, 0}; f.configure();
            complete(f.backend.transfer({0x08, &tx, 1, nullptr, 0}, futureDeadline), 1, 0);
        }
    }
    Fixture f;
    CHECK(f.backend.shutdownResult().code == Code::None);
    f.backend.shutdown(); f.backend.shutdown();
    unknown(f.backend.shutdownResult(), Code::Complete);
    CHECK(f.native.opens == 0 && !f.native.allocated);
}

void revision_mismatch_prevents_native_initialization() {
    for (uint32_t revision : {0u, expectedRevision - 1, expectedRevision + 1, 0x05050301u}) {
        Fixture f; f.native.revision = revision;
        unknown(f.backend.configure(validBus, futureDeadline), Code::NotReady);
        CHECK(f.native.revisions == 1 && f.native.opens == 0 && f.native.closes == 0);
        uint8_t tx = 0;
        unknown(f.backend.transfer({0x08, &tx, 1, nullptr, 0}, futureDeadline), Code::NotReady);
        CHECK(f.native.transfers == 0 && !f.native.allocated);
        f.native.revision = expectedRevision; f.configure();
        CHECK(f.native.revisions == 2 && f.native.opens == 1);
    }
}
} // namespace

int main() {
    const std::pair<const char*, void(*)()> tests[] = {
        {"GPIO1/2 100kHz configuration and lifecycle guards", configure_guards_and_exact_bus},
        {"only addresses 0x08/0x40, full 69TX/68RX and guarded buffers", allowed_addresses_and_complete_frame_counts},
        {"invalid sizes/pointers rejected without C calls or fault latch", transfer_bounds_do_not_call_c_or_latch_fault},
        {"deadline bounds and exact signed 64-bit delegation", deadline_representation_and_delegation},
        {"NACK unknown counts, no implicit retry, explicit next request", nack_has_unknown_counts_no_retry_and_allows_new_transfer},
        {"timeout/bus error stays unready until shutdown plus configure", fatal_transfer_requires_explicit_shutdown_and_configure},
        {"native invalid argument keeps counts unknown", native_invalid_argument_does_not_fabricate_completion},
        {"partial configuration context retained for explicit shutdown", partial_config_keeps_context_until_explicit_shutdown},
        {"shutdown failure retains context and visible error", shutdown_failure_keeps_context_and_visible_error},
        {"revision mismatch prevents initialization", revision_mismatch_prevents_native_initialization},
    };
    for (const auto& test : tests) { test.second(); std::cout << "PASS " << test.first << '\n'; }
    std::cout << "PASS " << std::size(tests) << " real C++ adapter groups; fake C API, no SDK timing/ACK validation\n";
}
