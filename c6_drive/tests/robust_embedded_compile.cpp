#include "robust_control.h"
// Compile/link-shape probe for the ESP32-C6 toolchain. No setup(), GPIO or motor code.
static wbb_robust::Controller control;
extern "C" void robust_arm(float now){control.arm(now);}
extern "C" const wbb_robust::Output* robust_tick(const wbb_robust::Input* input){return &control.update(*input);}
extern "C" void robust_stop(){control.stop();}
