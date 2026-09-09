#pragma once
#include "pico_i2c_transport.h"

namespace pico_i2c {
// ESP-IDF5.5.4 on S3, using the project-local namespaced master driver.
// Error/timeout requires explicit shutdown/configure; NACK alone is terminal
// for this request and does not secretly retry or clear the bus.
class EspIdfBackend final : public Backend {
public:
    EspIdfBackend() = default;
    EspIdfBackend(const EspIdfBackend&) = delete;
    EspIdfBackend& operator=(const EspIdfBackend&) = delete;
    ~EspIdfBackend() override;
    Result configure(const BusConfig& config, uint64_t deadlineUs) override;
    Result transfer(const Transfer& transfer, uint64_t deadlineUs) override;
    void shutdown() override;
    Result shutdownResult() const { return shutdown_; }
private:
    void* context_ = nullptr;
    bool ready_ = false;
    Result shutdown_;
};
} // namespace pico_i2c
