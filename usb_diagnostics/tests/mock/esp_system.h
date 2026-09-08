#pragma once
#include <cstdint>
inline int esp_reset_reason(){return 1;}
inline uint32_t esp_random(){return 1234567;}
inline unsigned restartCalls=0;
inline void esp_restart(){++restartCalls;}
