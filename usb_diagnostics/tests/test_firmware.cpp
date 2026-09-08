// Runs the actual firmware parser/loop against mocked GPIO, LEDC and USB.
#include <cassert>
#include <iostream>
#include "../src/main.cpp"
static void submit(const char* text) {
    Serial.feed(text);
    while(Serial.available()) loop();
}
static void assertStopped() {
    assert(!safety.running);
    for(const auto& p:pins){assert(duties[p.fi]==0 && duties[p.bi]==0);}
    for(const auto& p:pins){assert(modes[p.a]==INPUT_PULLUP && modes[p.b]==INPUT_PULLUP);}
}
int main() {
    setup(); assertStopped();
    fakeNow=1000;
    submit("1 pulse 0 1 64 500\n"); assertStopped();
    submit("2 armbench 6000 12000 64\n"); assertStopped();
    fakeNow+=600;
    submit("3 armbench 6000 6000 64\n");
    submit("4 pulse 0 1 64 1000\n");
    assert(duties[0]==64 && duties[1]==0);
    fakeNow+=350;loop();assertStopped();
    assert(!strcmp(safety.reason,"HEARTBEAT_TIMEOUT"));
    fakeNow+=1000;
    submit("5 armbench 12000 12000 64\n6 pulse 3 -1 64 1000\n");
    for(const auto& p:pins)assert(duties[p.bi]==64 && duties[p.fi]==0);
    Serial.connected=false;loop();assertStopped();
    Serial.connected=true;fakeNow+=1000;
    submit("70 armbench 6000 6000 64\n71 pulse 0 1 64 1000\n");
    submit("72 pulse 1 1 64 1000\n");
    assertStopped(); // A rejected command must not leave the previous pulse running.
    fakeNow+=1000;
    submit("7 armbench 6000 6000 64\n8 pulse 0 1 64 1000\n");
    Serial.feed(std::string(500,'9')); // No newline: stop as soon as overflow is seen.
    while(Serial.available())loop();
    assertStopped();assert(!strcmp(safety.reason,"RX_OVERFLOW"));
    submit("\n");
    // Excess fields used to be capable of overrunning the numeric array.
    submit("9 bogus 1 2 3 4 5 6\n");assertStopped();
    fakeNow+=1000;
    submit("10 armbench 6000 6000 64\n");
    failWrite=true;
    submit("11 pulse 2 1 64 500\n");assertStopped();
    assert(!pwmReady);
    failWrite=false;failAttach=5;setup();
    submit("12 armbench 6000 6000 64\n13 pulse 1 1 64 500\n");assertStopped();
    assert(!pwmReady);
    failAttach=-1;setup();fakeNow+=1000;
    submit("14 armbench 6000 6000 64\n15 pulse 0 1 64 1000\n16 reboot\n");
    assertStopped();assert(rebootPending && restartCalls==0);
    submit("17 armbench 6000 6000 64\n18 pulse 2 -1 64 500\n");
    assertStopped();assert(!safety.armed);
    fakeNow+=100;loop();assert(restartCalls==1);assertStopped();
    status();
    // Last status is read by the Python protocol tests as a real generated frame.
    std::cout << Serial.tx;
}
