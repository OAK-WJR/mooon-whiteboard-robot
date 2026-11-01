#pragma once
// Native-test stand-in for the ESP-IDF register header that c6_selftest/src/main.cpp includes unconditionally.
// Only its legacy adapter self-test branch reads GPIO_IN_REG (REG_READ); the R3 read-only diagnostic that
// control_tests/test_r3_diagnostic.cpp compiles uses no register, so nothing is modelled here.
