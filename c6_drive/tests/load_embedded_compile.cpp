// Compile-only target for ESP32-C6. No setup/loop, GPIO or device transport.
#include "encoder_servo.h"
#include "encoder_load_servo.h"
#define EncoderServo EncoderLoadServo
#include "control.h"
#undef EncoderServo
static_assert(sizeof(drive::Control)<4096,"Unexpected controller memory growth");
extern "C" void wbb_load_servo_tick(drive::Control* control,uint32_t now,float dt){
    control->tick(now,dt);
}
