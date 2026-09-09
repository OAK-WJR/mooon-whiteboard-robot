#if defined(WBB_TOP_BOARD_PICO) || defined(WBB_I2C_BACKEND_NATIVE_TEST)
#include "pico_i2c_esp_backend.h"
#include "pico_i2c_native.h"
#include <limits>

namespace pico_i2c {
namespace {
Result mapped(wbb_i2c_result value) {
    const Code code = value.code == WBB_I2C_OK ? Code::Complete :
        value.code == WBB_I2C_NAK ? Code::Nak : value.code == WBB_I2C_TIMEOUT ? Code::Timeout :
        value.code == WBB_I2C_INVALID ? Code::InvalidArgument : Code::BusError;
    return {code, 0, 0, CountEvidence::Unknown, value.sdk_error};
}
bool validDeadline(uint64_t deadline) {
    return deadline > 0 && deadline <= uint64_t(std::numeric_limits<int64_t>::max());
}
}

EspIdfBackend::~EspIdfBackend() { configASSERT(context_ == nullptr); }

Result EspIdfBackend::configure(const BusConfig& config, uint64_t deadline) {
    if (context_ || config.sda != sdaGpio || config.scl != sclGpio ||
        config.frequencyHz != busFrequencyHz || !validDeadline(deadline)) return {Code::InvalidArgument};
    if (wbb_i2c_backend_revision() != 0x05050401u) return {Code::NotReady};
    Result result = mapped(wbb_i2c_open(&context_, config.sda, config.scl,
                                       config.frequencyHz, int64_t(deadline)));
    ready_ = result.code == Code::Complete;
    return result;
}

Result EspIdfBackend::transfer(const Transfer& request, uint64_t deadline) {
    if (!ready_) return {Code::NotReady};
    if ((request.address != 0x08 && request.address != 0x40) || !request.tx ||
        request.txSize == 0 || request.txSize > maxTx || request.rxSize > maxRx ||
        (request.rxSize && !request.rx) || !validDeadline(deadline)) return {Code::InvalidArgument};
    Result result = mapped(wbb_i2c_transfer(context_, request.address, request.tx, request.txSize,
                                          request.rx, request.rxSize, int64_t(deadline)));
    if (result.code == Code::Complete) {
        result.transmitted = request.txSize;
        result.received = request.rxSize;
        result.evidence = CountEvidence::BackendCompletion;
    } else if (result.code != Code::Nak && result.code != Code::InvalidArgument) ready_ = false;
    return result;
}

void EspIdfBackend::shutdown() {
    ready_ = false;
    shutdown_ = mapped(wbb_i2c_close(&context_));
}
} // namespace pico_i2c
#endif
