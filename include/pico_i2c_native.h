#pragma once
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
// Only the pinned I2C worker may call these. No call retains caller buffers.
enum wbb_i2c_code { WBB_I2C_OK, WBB_I2C_NAK, WBB_I2C_TIMEOUT, WBB_I2C_ERROR, WBB_I2C_INVALID };
struct wbb_i2c_result { enum wbb_i2c_code code; int sdk_error; };
struct wbb_i2c_result wbb_i2c_open(void **context, int sda, int scl, uint32_t hz, int64_t deadline_us);
struct wbb_i2c_result wbb_i2c_transfer(void *context, uint8_t address,
    const uint8_t *tx, size_t tx_size, uint8_t *rx, size_t rx_size, int64_t deadline_us);
struct wbb_i2c_result wbb_i2c_close(void **context);
// Link-audit marker; returned only by the project-local adaptation.
uint32_t wbb_i2c_backend_revision(void);
#ifdef __cplusplus
}
#endif
