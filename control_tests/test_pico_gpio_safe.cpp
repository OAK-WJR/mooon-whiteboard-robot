// Actual SafeDefaults, with only SDK/clock boundaries mocked. No runtime or IO.
#include "pico_gpio_safe.h"
#include <iostream>
#include <stdexcept>

using namespace pico_gpio_mock;
static void require(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
static void reset() {
    levels.fill(HIGH); modes.fill(-99); driverModes.fill(0);
    // Retained push-pull LOW on wake must be disabled before preloading HIGH.
    levels[39]=LOW; driverModes[39]=GPIO_MODE_OUTPUT;
    events.clear(); pinOperations.clear(); failOp.clear();
    failPin=releaseFailPin=failValue=-1;
}
static void safeEvents() {
    for(const auto& e:events) {
        require(e.pin!=40 && !pico::reserved(e.pin),"reserved/NC pin touched");
        const std::string op=e.op;
        if(e.pin==39 && op=="level")
            require(e.value==HIGH && (e.mode==GPIO_MODE_INPUT || e.mode==GPIO_MODE_OUTPUT_OD),
                    "wake HIGH before output disable, or wake LOW request");
        if(e.pin==39 && op=="config")
            require(e.mode==GPIO_MODE_OUTPUT_OD && e.value==HIGH,"wake OD enabled with LOW latch");
        if(e.pin==41 && (op=="level" || op=="config"))
            require(e.value==LOW,"charge HIGH requested/enabled");
        if(op=="config" && e.mode==GPIO_MODE_OUTPUT)
            require(e.value==LOW,"retained HIGH latch enabled");
    }
}
static void released(int failedPin=-1) {
    // Input mode means the external R101/R113 biases may act, not measured levels.
    for(int pin:{16,42,41,17,39}) if(pin!=failedPin)
        require(driverModes[pin]==GPIO_MODE_INPUT,"failure did not release every owned output");
}
int main() {
    unsigned scenarios=0;
    reset();
    pico::SafeDefaults baseline;
    require(baseline.begin(),"normal GPIO startup failed");
    const auto startup=events;
    require(driverModes[39]==GPIO_MODE_OUTPUT_OD && levels[39]==HIGH,"wake not released OD");
    require(driverModes[41]==GPIO_MODE_OUTPUT && levels[41]==LOW,"charge not driven LOW");
    std::vector<std::string> wakeOps;
    for(const auto& e:events) if(e.pin==39)wakeOps.push_back(e.op);
    require(wakeOps==std::vector<std::string>{"direction","level","config"},"wake startup order changed");
    for(int i=0;i<10;++i) require(baseline.holdLow(),"holdLow failed");
    require(baseline.pulseHeartbeat(),"existing checked heartbeat failed");
    require(levels[39]==HIGH && levels[41]==LOW,"heartbeat changed wake/charge default");
    safeEvents();++scenarios;

    // Fail each actual startup SDK call, including partial combined configs.
    for(const auto& event:startup) {
        reset(); failOp=event.op; failPin=event.pin;
        pico::SafeDefaults pins;
        require(!pins.begin() && pins.error()==-42,"startup SDK failure was hidden");
        require(pins.releaseError()==ESP_OK,"clean release reported an error");
        released();
        require(!pins.begin() && !pins.holdLow() && !pins.pulseHeartbeat(),"latched failure cleared");
        safeEvents();++scenarios;
    }
    for(int pin:{16,42,41,39}) {
        reset();pico::SafeDefaults pins;require(pins.begin(),"setup failed");
        const auto before=events.size();failOp="level";failPin=pin;
        const bool held=pin==39?pins.holdLow():pins.pulseHeartbeat();
        require(!held && pins.error()==-42,"hold failure allowed safe operation");
        for(std::size_t i=before;i<events.size();++i)
            require(!(events[i].pin==17 && std::string(events[i].op)=="level" && events[i].value==LOW),
                    "failed safe default emitted WDI pulse");
        released();safeEvents();++scenarios;
    }
    for(int pin:{16,42,41,17,39}) {
        reset();pico::SafeDefaults pins;require(pins.begin(),"setup failed");
        failOp="level";failPin=41;releaseFailPin=pin;
        require(!pins.holdLow() && pins.error()==-42 && pins.releaseError()==-42,
                "initiating or release SDK error was hidden");
        released(pin);
        require(!pins.begin() && !pins.holdLow() && !pins.ready(),"release failure self-cleared");
        safeEvents();++scenarios;
    }
    reset();pico::SafeDefaults waking;require(waking.begin(),"wake setup failed");
    require(waking.setWakeRequest(true) && waking.wakeRequested(),"wake assert failed");
    require(driverModes[39]==GPIO_MODE_OUTPUT_OD && levels[39]==LOW,"wake is not an OD sink");
    for(int i=0;i<10;++i) {
        require(waking.holdPowerOff() && waking.pulseHeartbeat(),"progress during wake failed");
        require(levels[39]==LOW && waking.wakeRequested(),"heartbeat truncated wake");
        for(int pin:{16,42,41})require(levels[pin]==LOW,"wake enabled a power request");
    }
    require(waking.holdLow() && !waking.wakeRequested() && levels[39]==HIGH,"stop did not release wake");
    ++scenarios;
    for(int value:{LOW,HIGH}) {
        reset();pico::SafeDefaults pins;require(pins.begin(),"wake fault setup failed");
        if(value==HIGH)require(pins.setWakeRequest(true),"wake assert failed");
        failOp="level";failPin=39;failValue=value;
        require(!pins.setWakeRequest(value==LOW) && pins.error()==-42 && !pins.wakeRequested(),"wake SDK failure hidden");
        released();require(!pins.begin() && !pins.setWakeRequest(true),"wake fault re-enabled output");
        ++scenarios;
    }
    std::cout<<scenarios<<" Pico safe-default scenarios PASS\n";
}
