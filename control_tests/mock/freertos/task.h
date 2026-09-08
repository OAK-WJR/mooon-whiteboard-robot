#pragma once
#include "FreeRTOS.h"
inline void (*criticalSectionHook)() = nullptr;
inline void taskENTER_CRITICAL(portMUX_TYPE* mux) {
    if (criticalSectionHook) criticalSectionHook();
    mux->mutex.lock();
}
inline void taskEXIT_CRITICAL(portMUX_TYPE* mux) { mux->mutex.unlock(); }
inline void taskENTER_CRITICAL_ISR(portMUX_TYPE* mux) { mux->mutex.lock(); }
inline void taskEXIT_CRITICAL_ISR(portMUX_TYPE* mux) { mux->mutex.unlock(); }
