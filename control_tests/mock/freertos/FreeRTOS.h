#pragma once
#include <mutex>

// Host mutual exclusion only; does not emulate ESP interrupt masking/scheduling.
struct portMUX_TYPE { std::recursive_mutex mutex; };
#define portMUX_INITIALIZER_UNLOCKED {}
