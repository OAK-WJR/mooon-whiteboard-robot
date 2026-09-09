#pragma once
#if defined(CONFIG_IDF_TARGET_ESP32S3) && !defined(WBB_TOP_BOARD_PICO)
#error "The S3 integrated runtime requires explicit WBB_TOP_BOARD_PICO selection"
#endif
// Substitute hardware dependencies only; native tests compile the real command
// processor and loop below, including the permanent board qualification gate.
#ifdef WBB_CONTROL_NATIVE_TEST
#include "../control_tests/runtime_stubs.h"
#ifdef WBB_TOP_BOARD_PICO
#include "../control_tests/pico_rail_mock.h"
#endif
#else
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <mbedtls/md.h>
#include "robot_controller.h"
#include "control_transport.h"
#endif
