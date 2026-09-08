#pragma once
#include <cassert>
#include <cstdint>
using BaseType_t = int;
using UBaseType_t = unsigned;
using TickType_t = uint32_t;
constexpr BaseType_t pdPASS = 1, pdTRUE = 1, pdFALSE = 0;
#define configASSERT(value) assert(value)
