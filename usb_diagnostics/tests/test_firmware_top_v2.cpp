// Exercise the actual new-board build: neither voltage may bypass the power hold.
#define DIAG_TOP_V2 1
#include <cassert>
#include <iostream>
#include "../src/main.cpp"
static void submit(const std::string& s) {
    Serial.feed(s);while(Serial.available())loop();
}
static void stopped() {
    assert(!safety.running && !safety.armed && levels[21]==LOW);
    for(const auto& p:pins)assert(duties[p.fi]==0 && duties[p.bi]==0);
    assert(modes[7]==INPUT_PULLUP && modes[16]!=OUTPUT);
}
int main() {
    setup();stopped();fakeNow=1000;
    for(int mv:{6000,12000}) {
        submit("1 armbench "+std::to_string(mv)+" "+std::to_string(mv)+" 64\n");
        assert(!strcmp(safety.reason,"BOARD_POWER_UNREVIEWED"));stopped();
        submit("2 pulse 3 1 64 500\n");stopped();
    }
    submit("3 reboot\n4 armbench 6000 6000 64\n");
    stopped();assert(rebootPending && restartCalls==0);
    fakeNow+=100;loop();assert(restartCalls==1);stopped();
    setup();fakeNow=0xfffffff0;
    submit("5 reboot\n");fakeNow+=100;loop();
    assert(restartCalls==2);stopped();
    status();std::cout<<Serial.tx;
}
