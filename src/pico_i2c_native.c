#if defined(WBB_TOP_BOARD_PICO)
#include "sdkconfig.h"
#include "pico_i2c_native.h"
#include "../third_party/esp_idf_i2c_no_clear/i2c_master.c"

// Shared base ABI checked against the installed SDK's DWARF, not guessed from
// a version string. The project-private master extension is never shared.
_Static_assert(sizeof(i2c_bus_t) == 52, "Unreviewed common I2C bus ABI");
_Static_assert(offsetof(i2c_bus_t, spinlock) == 8 && offsetof(i2c_bus_t, hal) == 16 &&
    offsetof(i2c_bus_t, clk_src) == 20 && offsetof(i2c_bus_t, clk_src_freq_hz) == 24 &&
    offsetof(i2c_bus_t, sda_num) == 28 && offsetof(i2c_bus_t, scl_num) == 32 &&
    offsetof(i2c_bus_t, pull_up_enable) == 36 && offsetof(i2c_bus_t, intr_handle) == 40 &&
    offsetof(i2c_bus_t, pm_lock) == 44 && offsetof(i2c_bus_t, bus_mode) == 48,
    "Unreviewed common I2C bus member offsets");

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t bq, ina;
    BaseType_t core;
} wbb_context;

static struct wbb_i2c_result wbb_result(esp_err_t error)
{
    return (struct wbb_i2c_result){error == ESP_OK ? WBB_I2C_OK :
        error == ESP_ERR_NOT_FOUND ? WBB_I2C_NAK : error == ESP_ERR_TIMEOUT ? WBB_I2C_TIMEOUT :
        error == ESP_ERR_INVALID_ARG ? WBB_I2C_INVALID : WBB_I2C_ERROR, error};
}

uint32_t wbb_i2c_backend_revision(void) { return 0x05050401u; }

struct wbb_i2c_result wbb_i2c_open(void **context, int sda, int scl, uint32_t hz, int64_t deadline)
{
    if (!context || *context || sda != 1 || scl != 2 || hz != 100000 || deadline <= 0)
        return wbb_result(ESP_ERR_INVALID_ARG);
    if (esp_timer_get_time() >= deadline) return wbb_result(ESP_ERR_TIMEOUT);
    wbb_context *owner = calloc(1, sizeof(*owner));
    if (!owner) return wbb_result(ESP_ERR_NO_MEM);
    *context = owner; // Even a partial setup remains owned until explicit close.
    owner->core = xPortGetCoreID();
    const i2c_master_bus_config_t bus = {.i2c_port=I2C_NUM_0, .sda_io_num=sda, .scl_io_num=scl,
        .clk_source=I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt=7, .trans_queue_depth=0,
        .flags.enable_internal_pullup=false};
    esp_err_t error = i2c_new_master_bus(&bus, &owner->bus);
    if (error != ESP_OK) return wbb_result(error);
    if (esp_timer_get_time() >= deadline) return wbb_result(ESP_ERR_TIMEOUT);
    i2c_device_config_t device = {.dev_addr_length=I2C_ADDR_BIT_LEN_7, .device_address=0x08,
        .scl_speed_hz=hz, .scl_wait_us=2000, .flags.disable_ack_check=false};
    error = i2c_master_bus_add_device(owner->bus, &device, &owner->bq);
    if (error != ESP_OK) return wbb_result(error);
    if (esp_timer_get_time() >= deadline) return wbb_result(ESP_ERR_TIMEOUT);
    device.device_address = 0x40;
    error = i2c_master_bus_add_device(owner->bus, &device, &owner->ina);
    return wbb_result(error == ESP_OK && esp_timer_get_time() >= deadline ? ESP_ERR_TIMEOUT : error);
}

struct wbb_i2c_result wbb_i2c_transfer(void *context, uint8_t address,
    const uint8_t *tx, size_t tx_size, uint8_t *rx, size_t rx_size, int64_t deadline)
{
    wbb_context *owner = context;
    if (!owner || !owner->bus || !tx || !tx_size || tx_size > 69 || rx_size > 68 ||
        (rx_size && !rx) || (address != 0x08 && address != 0x40) || deadline <= 0)
        return wbb_result(ESP_ERR_INVALID_ARG);
    if (owner->core != xPortGetCoreID() || owner->bus->wbb_faulted)
        return wbb_result(ESP_ERR_INVALID_STATE);
    i2c_master_dev_handle_t device = address == 0x08 ? owner->bq : owner->ina;
    if (!device) return wbb_result(ESP_ERR_INVALID_STATE);
    owner->bus->wbb_deadline_us = deadline;
    // These names resolve only to the adapted translation unit, not stock APIs.
    esp_err_t result = rx_size ? i2c_master_transmit_receive(device, tx, tx_size, rx, rx_size, 0) :
                                i2c_master_transmit(device, tx, tx_size, 0);
    return wbb_result(result);
}

struct wbb_i2c_result wbb_i2c_close(void **context)
{
    if (!context) return wbb_result(ESP_ERR_INVALID_ARG);
    wbb_context *owner = *context;
    if (!owner) return wbb_result(ESP_OK);
    if (owner->core != xPortGetCoreID()) return wbb_result(ESP_ERR_INVALID_STATE);
    if (owner->bus) {
        wbb_quiesce(owner->bus, true);
        atomic_store(&owner->bus->status, I2C_STATUS_IDLE);
        esp_err_t error;
        if (owner->bq) {
            error = i2c_master_bus_rm_device(owner->bq);
            if (error != ESP_OK) return wbb_result(error);
            owner->bq = NULL;
        }
        if (owner->ina) {
            error = i2c_master_bus_rm_device(owner->ina);
            if (error != ESP_OK) return wbb_result(error);
            owner->ina = NULL;
        }
        error = i2c_del_master_bus(owner->bus);
        if (error != ESP_OK) return wbb_result(error);
    }
    free(owner);
    *context = NULL;
    return wbb_result(ESP_OK);
}
#endif
