// This harness includes the ACTUAL adapted wait/dispatch/quiesce functions.
// RTOS and FIFO boundaries below are controlled fakes, not a silicon model.
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdatomic.h>

typedef unsigned TickType_t;
typedef int BaseType_t;
typedef int SemaphoreHandle_t;
typedef int esp_err_t;
typedef enum { I2C_STATUS_WRITE, I2C_STATUS_READ, I2C_STATUS_ACK_ERROR,
               I2C_STATUS_TIMEOUT, I2C_STATUS_DONE } i2c_master_status_t;
typedef enum { I2C_EVENT_ALIVE, I2C_EVENT_DONE, I2C_EVENT_NACK, I2C_EVENT_TIMEOUT } i2c_master_event_t;
enum { pdTRUE=1, I2C_LL_CMD_WRITE=1, I2C_LL_CMD_READ=2, I2C_LL_CMD_STOP=3, I2C_LL_INTR_MASK=255 };
typedef struct { int op_code; } i2c_ll_hw_cmd_t;
typedef struct { i2c_ll_hw_cmd_t hw_cmd; uint8_t* data; unsigned remaining; } i2c_operation_t;
typedef struct { int dev; } i2c_hal_context_t;
typedef struct { i2c_hal_context_t hal; int spinlock; } bus_base;
typedef struct master {
    bus_base* base;
    int64_t wbb_deadline_us;
    BaseType_t wbb_owner_core;
    bool contains_read;
    SemaphoreHandle_t cmd_semphr;
    int event_queue;
    _Atomic i2c_master_status_t status;
    i2c_master_event_t event;
    struct { unsigned cmd_count; i2c_operation_t* ops; } i2c_trans;
    unsigned trans_idx;
    i2c_operation_t i2c_ops[8];
} *i2c_master_bus_handle_t;

static int64_t now;
static unsigned chunks, starts, stops, gives, resets, clear_clocks, masks, calls_after_deadline;
static bool sem_available, event_available, busy, masked;
static int64_t sem_latency, event_latency;
static unsigned timeout_on_chunk;
static i2c_master_event_t final_event;
static i2c_master_bus_handle_t current;
#define configTICK_RATE_HZ 1000
#define configASSERT assert
#define taskYIELD() (now += 50)
#define portENTER_CRITICAL(p) ((void)(p))
#define portEXIT_CRITICAL(p) ((void)(p))
static int xPortGetCoreID(void) { return 1; }
static int64_t esp_timer_get_time(void) { return now; }
static int xSemaphoreTake(int semaphore, unsigned ticks) {
    (void)semaphore;
    if (sem_available) { sem_available=false; now+=sem_latency; return 1; }
    now += ticks*1000;
    return 0;
}
static void xSemaphoreGive(int semaphore) { (void)semaphore; sem_available=true; ++gives; }
static void xQueueReset(int queue) { (void)queue; }
static int xQueueReceive(int queue, i2c_master_event_t* event, unsigned ticks) {
    (void)queue;
    if (event_available) { event_available=false; now+=event_latency; *event=final_event; return 1; }
    now += ticks*1000;
    return 0;
}
static void i2c_ll_disable_intr_mask(int dev, unsigned mask) { (void)dev; assert(mask==255); masked=true; ++masks; }
static void i2c_ll_clear_intr_mask(int dev, unsigned mask) { (void)dev; assert(mask==255); }
static int s_i2c_hw_fsm_reset(i2c_master_bus_handle_t m, bool clear) {
    assert(masked && !m->contains_read && m->i2c_trans.cmd_count==0);
    for (unsigned i=0;i<8;++i) assert(m->i2c_ops[i].data==NULL);
    ++resets; clear_clocks+=clear; return 0;
}
static bool i2c_ll_is_bus_busy(int dev) { (void)dev; return busy; }
static void i2c_ll_master_write_cmd_reg(int dev, i2c_ll_hw_cmd_t command, int index) {
    (void)dev; assert(command.op_code==I2C_LL_CMD_STOP && index==0); ++stops;
}
static void i2c_hal_master_trans_start(i2c_hal_context_t* hal) {
    (void)hal; ++starts; if (now>=current->wbb_deadline_us) ++calls_after_deadline;
}
static void consume(i2c_master_bus_handle_t m, i2c_operation_t* op) {
    if (now>=m->wbb_deadline_us) ++calls_after_deadline;
    unsigned bytes=op->remaining>31?31:op->remaining;
    op->remaining-=bytes; now+=bytes*90; ++chunks;
    if (!op->remaining) { ++m->trans_idx; --m->i2c_trans.cmd_count; }
    if (chunks==timeout_on_chunk) atomic_store(&m->status,I2C_STATUS_TIMEOUT);
    sem_available=true;
}
static void s_i2c_write_command(i2c_master_bus_handle_t m, i2c_operation_t* op,
    uint8_t* fifo, uint8_t* address, void* yield) { (void)fifo;(void)address;(void)yield;consume(m,op); }
static void s_i2c_read_command(i2c_master_bus_handle_t m, i2c_operation_t* op,
    uint8_t* fifo, void* yield) { (void)fifo;(void)yield;consume(m,op); }
static void s_i2c_start_end_command(i2c_master_bus_handle_t m, i2c_operation_t* op,
    uint8_t* address, void* yield) { (void)address;(void)yield;consume(m,op); }

#include "actual_driver_functions.inc"

static void setup(struct master* m, bus_base* base, unsigned length, int64_t budget) {
    memset(m,0,sizeof(*m));memset(base,0,sizeof(*base));
    now=1000; chunks=starts=stops=gives=resets=clear_clocks=masks=calls_after_deadline=0;
    sem_latency=event_latency=0;timeout_on_chunk=0;
    sem_available=event_available=true;busy=masked=false;final_event=I2C_EVENT_DONE;
    m->base=base;m->wbb_deadline_us=now+budget;m->wbb_owner_core=1;
    m->cmd_semphr=1;m->event_queue=2;m->contains_read=true;
    m->i2c_ops[0]=(i2c_operation_t){.hw_cmd={I2C_LL_CMD_WRITE},.remaining=length,.data=(uint8_t*)m};
    m->i2c_trans.ops=m->i2c_ops;m->i2c_trans.cmd_count=1;
    atomic_store(&m->status,I2C_STATUS_WRITE);current=m;
}

int main(void) {
    struct master m;bus_base b;unsigned cases=0;
    setup(&m,&b,69,25000);s_i2c_send_commands(&m,0);
    assert(m.status==I2C_STATUS_DONE && chunks==3 && starts==1 && !calls_after_deadline);++cases;
    wbb_quiesce(&m,false);assert(masks==1 && resets==0 && !m.i2c_ops[0].data);++cases;

    setup(&m,&b,69,4000);s_i2c_send_commands(&m,0);
    assert(m.status==I2C_STATUS_TIMEOUT && chunks==2 && starts==0 && !calls_after_deadline);
    wbb_quiesce(&m,true);assert(resets==1 && clear_clocks==0);++cases;

    setup(&m,&b,3,4000);sem_available=false;s_i2c_send_commands(&m,0);
    assert(m.status==I2C_STATUS_TIMEOUT && chunks==0 && starts==0);++cases;
    setup(&m,&b,3,4000);sem_latency=4000;s_i2c_send_commands(&m,0);
    assert(m.status==I2C_STATUS_TIMEOUT && chunks==0 && starts==0);++cases;
    setup(&m,&b,69,25000);timeout_on_chunk=1;s_i2c_send_commands(&m,0);
    assert(m.status==I2C_STATUS_TIMEOUT && chunks==1 && starts==0);++cases;

    setup(&m,&b,3,4000);m.status=I2C_STATUS_ACK_ERROR;final_event=I2C_EVENT_NACK;
    s_i2c_send_commands(&m,0);assert(m.status==I2C_STATUS_ACK_ERROR && chunks==0 && starts==1 && stops==1 && clear_clocks==0);++cases;
    setup(&m,&b,3,4000);m.status=I2C_STATUS_ACK_ERROR;final_event=I2C_EVENT_NACK;busy=true;
    s_i2c_send_commands(&m,0);assert(m.status==I2C_STATUS_TIMEOUT && chunks==0 && starts==1 && stops==1 && clear_clocks==0);++cases;
    setup(&m,&b,3,4000);event_latency=4000;s_i2c_send_commands(&m,0);
    assert(m.status==I2C_STATUS_TIMEOUT && !calls_after_deadline);++cases;
    setup(&m,&b,3,4000);event_available=false;s_i2c_send_commands(&m,0);
    assert(m.status==I2C_STATUS_TIMEOUT);++cases;
    setup(&m,&b,3,4000);final_event=I2C_EVENT_ALIVE;s_i2c_send_commands(&m,0);
    assert(m.status==I2C_STATUS_TIMEOUT);++cases;
    setup(&m,&b,1,4000);m.wbb_deadline_us=now;s_i2c_send_commands(&m,0);
    assert(m.status==I2C_STATUS_TIMEOUT && chunks==0 && starts==0);++cases;
    setup(&m,&b,3,4000);m.i2c_ops[0].hw_cmd.op_code=I2C_LL_CMD_READ;s_i2c_send_commands(&m,0);
    assert(m.status==I2C_STATUS_DONE && chunks==1);++cases;
    printf("%u actual-driver dispatch/deadline/quiesce cases PASS; fake RTOS/FIFO only\n",cases);
    return 0;
}
