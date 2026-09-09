"""Reproducible project-local IDF5.5.4 adaptation. Never edits installed SDK."""
from pathlib import Path
import difflib
import hashlib
import json
import re

HERE = Path(__file__).resolve().parent
source = (HERE/'upstream/i2c_master.c').read_text()
header = (HERE/'upstream/i2c_private.h').read_text()


def replace_function(text, name, replacement):
    match = re.search(r'^(?:static |inline |I2C_MASTER_ISR_ATTR )*\w+[ *]+'+name+r'\([^;]*?\)\s*\{', text, re.M)
    assert match, name
    start = text.index('{', match.start()); depth = 1; end = start+1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}'); end += 1
    return text[:match.start()]+replacement.strip()+text[end:]


# Independent symbols, sharing only IDF's common peripheral allocation layer.
names = re.findall(r'^esp_err_t (i2c_\w+)\(', source, re.M)
(HERE/'namespace.h').write_text('// Project-only names; no override of installed public APIs.\n'+
    '\n'.join('#define '+n+' wbb_idf554_'+n for n in names)+'\n')
source = source.replace('#include <string.h>', '''// Modified by WhiteBoardBot: see PATCH.diff and README.md.
#include "namespace.h"
#include "esp_idf_version.h"
#include "esp_timer.h"
#if !CONFIG_IDF_TARGET_ESP32S3 || ESP_IDF_VERSION != ESP_IDF_VERSION_VAL(5, 5, 4)
#error "This isolated I2C adaptation requires ESP32-S3 and ESP-IDF5.5.4"
#endif
#include <string.h>''', 1)
# Eliminate both the software and hardware clear-clock implementations entirely.
source = replace_function(source, 's_i2c_master_clear_bus', '''static esp_err_t s_i2c_master_clear_bus(i2c_bus_handle_t handle)
{
    (void)handle;
    return ESP_ERR_NOT_SUPPORTED; // No clear clocks, including explicit reset API.
}''')
source = source.replace('s_i2c_hw_fsm_reset(i2c_master, true)', 's_i2c_hw_fsm_reset(i2c_master, false)')
source = replace_function(source, 'i2c_master_bus_reset', '''esp_err_t i2c_master_bus_reset(i2c_master_bus_handle_t bus_handle)
{
    (void)bus_handle;
    return ESP_ERR_NOT_SUPPORTED;
}''')
header = header.replace('    i2c_transaction_t i2c_trans_pool[];', '''    // Task-owned deadline/core; ISR only writes the bus-error flag.
    int64_t wbb_deadline_us;
    BaseType_t wbb_owner_core;
    _Atomic bool wbb_bus_error;
    bool wbb_faulted;
    i2c_transaction_t i2c_trans_pool[];''')

helpers = '''
// Deadline includes every wait, FIFO chunk and final STOP. Flooring tick waits
// avoids rounding beyond it; the final sub-tick interval yields and rechecks.
static bool wbb_expired(i2c_master_bus_handle_t master)
{
    return esp_timer_get_time() >= master->wbb_deadline_us;
}
static TickType_t wbb_ticks_left(i2c_master_bus_handle_t master)
{
    int64_t left = master->wbb_deadline_us - esp_timer_get_time();
    return left > 0 ? (TickType_t)(left / (1000000 / configTICK_RATE_HZ)) : 0;
}
static bool wbb_take(i2c_master_bus_handle_t master, SemaphoreHandle_t semaphore)
{
    while (!wbb_expired(master)) {
        if (xSemaphoreTake(semaphore, wbb_ticks_left(master)) == pdTRUE) return true;
        taskYIELD();
    }
    return false;
}
static bool wbb_event(i2c_master_bus_handle_t master, i2c_master_event_t *event)
{
    while (!wbb_expired(master)) {
        if (xQueueReceive(master->event_queue, event, wbb_ticks_left(master)) == pdTRUE) return true;
        taskYIELD();
    }
    return false;
}
// Worker stays on the interrupt-allocation core. Masking on that core prevents
// an already-entered ISR from accessing buffers concurrently with this cleanup.
// The FSM abort can release bus lines; it does NOT run a bus-clear pulse train.
static void wbb_quiesce(i2c_master_bus_handle_t master, bool abort_transfer)
{
    configASSERT(xPortGetCoreID() == master->wbb_owner_core);
    portENTER_CRITICAL(&master->base->spinlock);
    i2c_ll_disable_intr_mask(master->base->hal.dev, I2C_LL_INTR_MASK);
    i2c_ll_clear_intr_mask(master->base->hal.dev, I2C_LL_INTR_MASK);
    master->contains_read = false;
    master->i2c_trans.cmd_count = 0;
    memset(master->i2c_ops, 0, sizeof(master->i2c_ops));
    portEXIT_CRITICAL(&master->base->spinlock);
    if (abort_transfer) s_i2c_hw_fsm_reset(master, false);
}
'''
position = source.index('static void s_i2c_send_commands(')
source = source[:position]+helpers+'\n'+source[position:]
source = replace_function(source, 's_i2c_send_commands', '''static void s_i2c_send_commands(i2c_master_bus_handle_t master, TickType_t unused_ticks)
{
    (void)unused_ticks;
    i2c_hal_context_t *hal = &master->base->hal;
    uint8_t fifo_fill = 0, address_fill = 0;
    xQueueReset(master->event_queue);
    master->event = I2C_EVENT_ALIVE;
    while (master->i2c_trans.cmd_count) {
        if (!wbb_take(master, master->cmd_semphr) || wbb_expired(master)) goto timeout;
        i2c_master_status_t status = atomic_load(&master->status);
        if (status == I2C_STATUS_TIMEOUT) goto timeout;
        if (status == I2C_STATUS_ACK_ERROR) {
            const i2c_ll_hw_cmd_t stop = {.op_code = I2C_LL_CMD_STOP};
            i2c_ll_master_write_cmd_reg(hal->dev, stop, 0);
            break; // The single start below sends STOP, not a repeated retry.
        }
        i2c_operation_t *operation = &master->i2c_trans.ops[master->trans_idx];
        if (operation->hw_cmd.op_code == I2C_LL_CMD_WRITE)
            s_i2c_write_command(master, operation, &fifo_fill, &address_fill, NULL);
        else if (operation->hw_cmd.op_code == I2C_LL_CMD_READ)
            s_i2c_read_command(master, operation, &fifo_fill, NULL);
        else s_i2c_start_end_command(master, operation, &address_fill, NULL);
    }
    if (wbb_expired(master)) goto timeout;
    i2c_hal_master_trans_start(hal);
    i2c_master_event_t event;
    if (!wbb_event(master, &event) || wbb_expired(master)) goto timeout;
    if (event == I2C_EVENT_DONE) atomic_store(&master->status, I2C_STATUS_DONE);
    else if (event == I2C_EVENT_NACK) {
        while (i2c_ll_is_bus_busy(hal->dev)) {
            if (wbb_expired(master)) goto timeout;
            taskYIELD();
        }
        atomic_store(&master->status, I2C_STATUS_ACK_ERROR);
    } else goto timeout;
    xSemaphoreGive(master->cmd_semphr);
    return;
timeout:
    atomic_store(&master->status, I2C_STATUS_TIMEOUT);
    xSemaphoreGive(master->cmd_semphr);
}''')
# Refuse starting on a busy/previously failed bus; no recovery/retry is hidden.
old = '''if (atomic_load(&i2c_master->status) == I2C_STATUS_TIMEOUT || i2c_ll_is_bus_busy(hal->dev)) {
        ESP_RETURN_ON_ERROR(s_i2c_hw_fsm_reset(i2c_master, false), TAG, "reset hardware failed");
    }'''
assert old in source
source = source.replace(old, '''if (i2c_master->wbb_faulted || wbb_expired(i2c_master) || i2c_ll_is_bus_busy(hal->dev)) {
        return ESP_ERR_TIMEOUT;
    }''', 1)
old = '''if (atomic_load(&i2c_master->status) != I2C_STATUS_DONE) {
            ret = ESP_ERR_INVALID_STATE;
        }'''
assert old in source
source = source.replace(old, '''if (atomic_load(&i2c_master->wbb_bus_error)) ret = ESP_FAIL;
        else if (atomic_load(&i2c_master->status) == I2C_STATUS_ACK_ERROR) ret = ESP_ERR_NOT_FOUND;
        else if (atomic_load(&i2c_master->status) != I2C_STATUS_DONE) ret = ESP_ERR_TIMEOUT;''')
source = source.replace('''if (int_mask & I2C_LL_INTR_NACK) {''', '''if (int_mask & I2C_LL_INTR_ARBITRATION) atomic_store(&i2c_master->wbb_bus_error, true);
    if (int_mask & I2C_LL_INTR_NACK) {''', 1)
source = replace_function(source, 's_i2c_synchronous_transaction', '''static esp_err_t s_i2c_synchronous_transaction(i2c_master_dev_handle_t device, i2c_operation_t *ops, size_t count, int unused_ms)
{
    (void)unused_ms;
    i2c_master_bus_handle_t master = device->master_bus;
    if (xPortGetCoreID() != master->wbb_owner_core || master->async_trans || master->wbb_faulted)
        return ESP_ERR_INVALID_STATE;
    if (count > I2C_STATIC_OPERATION_ARRAY_MAX) return ESP_ERR_INVALID_ARG;
    if (!wbb_take(master, master->bus_lock_mux)) return ESP_ERR_TIMEOUT;
    master->trans_done = false;
    atomic_store(&master->wbb_bus_error, false);
    memcpy(master->i2c_ops, ops, sizeof(i2c_operation_t)*count);
    master->addr_10bits_bus = device->addr_10bits;
    master->i2c_trans = (i2c_transaction_t){.device_address=device->device_address, .ops=master->i2c_ops, .cmd_count=count};
    master->sent_all = master->trans_finish = master->queue_trans = false;
    master->ack_check_disable = false;
    esp_err_t result = s_i2c_transaction_start(device, 0);
    if (wbb_expired(master)) result = ESP_ERR_TIMEOUT;
    wbb_quiesce(master, result != ESP_OK);
    if (result != ESP_OK && result != ESP_ERR_NOT_FOUND) master->wbb_faulted = true;
    xSemaphoreGive(master->bus_lock_mux);
    return result;
}''')
# Reject async setup and callbacks so the unbounded upstream async path is unreachable.
needle = '    i2c_port_num_t i2c_port_num = bus_config->i2c_port;'
source = source.replace(needle, '''    ESP_RETURN_ON_FALSE(bus_config && bus_config->trans_queue_depth == 0, ESP_ERR_INVALID_ARG, TAG, "synchronous bus only");
'''+needle, 1)
needle = '    ESP_RETURN_ON_FALSE(i2c_master, ESP_ERR_NO_MEM, TAG, "no memory for i2c master bus");'
source = source.replace(needle, needle+'\n    i2c_master->wbb_owner_core = xPortGetCoreID();', 1)
source = replace_function(source, 'i2c_master_register_event_callbacks', '''esp_err_t i2c_master_register_event_callbacks(i2c_master_dev_handle_t handle, const i2c_master_event_callbacks_t *cbs, void *user_data)
{
    (void)handle; (void)cbs; (void)user_data;
    return ESP_ERR_NOT_SUPPORTED;
}''')
(HERE/'i2c_master.c').write_text(source)
(HERE/'i2c_private.h').write_text(header)
patch=''
for name in ('i2c_master.c','i2c_private.h'):
    patch+=''.join(difflib.unified_diff((HERE/'upstream'/name).read_text().splitlines(True),
                                       (HERE/name).read_text().splitlines(True),fromfile='upstream/'+name,tofile=name))
(HERE/'PATCH.diff').write_text(patch)
records={}
for name in ('i2c_master.c','i2c_private.h','i2c_common.c'):
    f=HERE/'upstream'/name
    records[name]={'url':'https://raw.githubusercontent.com/espressif/esp-idf/v5.5.4/components/esp_driver_i2c/'+name,
                   'sha256':hashlib.sha256(f.read_bytes()).hexdigest()}
(HERE/'source_manifest.json').write_text(json.dumps({'version':'ESP-IDF5.5.4','license':'Apache-2.0',
    'sources':records,'adapted_sha256':{n:hashlib.sha256((HERE/n).read_bytes()).hexdigest() for n in ('i2c_master.c','i2c_private.h','namespace.h')}},indent=2)+'\n')
print('Generated namespaced no-clear/deadline driver; hardware qualification remains open')
