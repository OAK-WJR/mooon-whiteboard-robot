#ifdef WBB_TOP_BOARD_PICO
#include "pico_rail_service.h"
#include "pico_i2c_esp_backend.h"
#include "board_profile_pico.h"

static_assert(pico_i2c::sdaGpio == pico::i2cSda && pico_i2c::sclGpio == pico::i2cScl,
              "Worker pins must match the schematic-reviewed Pico profile");

namespace pico_rails {
namespace {
class EspBus final : public Bus {
public:
    bool open() override { return transport_.begin(); }
    void close() override { transport_.requestStop(); }
    BusState state() const override {
        using S = pico_i2c::Transport::State;
        switch (transport_.state()) {
        case S::Starting: return BusState::Starting;
        case S::Ready: return BusState::Ready;
        case S::Fault: return BusState::Fault;
        case S::Stopping: return BusState::Stopping;
        case S::Stopped: {
            // Stopped is an acquire observation of worker completion. Do not
            // read the non-atomic backend cleanup result while it is running.
            const auto code = backend_.shutdownResult().code;
            return code == pico_i2c::Code::None || code == pico_i2c::Code::Complete ?
                BusState::Stopped : BusState::CleanupFault;
        }
        }
        return BusState::Fault;
    }
    uint64_t nowMicros() const override { return clock_.nowUs(); }
    bool start(uint8_t address, const uint8_t* tx, std::size_t txSize,
               uint8_t* rx, std::size_t rxSize) override {
        return transport_.ina().start(address, tx, txSize, rx, rxSize);
    }
    ina3221::IoResult poll() override { return transport_.ina().poll(); }
    void cancel() override { transport_.ina().cancel(); }
    bq76905::I2c& cells() override { return transport_.bq(); }
    bool cellsIdle() const override { return transport_.bq().idle(); }
private:
    pico_i2c::EspClock clock_;
    pico_i2c::EspIdfBackend backend_;
    pico_i2c::Transport transport_{backend_, clock_};
};
}
Service& diagnostics() {
    // No hardware calls during construction. setup() starts the worker only
    // after safe GPIO, UART/radio initialization and task watchdog enrollment.
    static EspBus bus;
    static Service service(bus);
    return service;
}
} // namespace pico_rails
#endif
