#define WBB_TOP_BOARD_V2 1
#define WBB_CONTROL_NATIVE_TEST 1
#include <cassert>
#include <cmath>
#include <iostream>
#include "../src/top_v2_main.cpp"
int main(){
    setup();
    fakeNow=55;loop();
    // Actual elapsed time must reach the velocity estimator, not a fixed 20 ms.
    assert(std::abs(robot.lastDt-.055f)<.00001f);
    std::cout<<"Production runtime elapsed-time regression PASS\n";
}
