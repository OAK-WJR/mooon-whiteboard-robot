#define WBB_CONTROL_NATIVE_TEST 1
#include "runtime_stubs.h"
#include "kiss_c6_voltage.h"
#include <cassert>
#include <algorithm>
#include <cstring>
#include <iostream>
using kiss_c6::VoltageMonitor;
static void resetMock(){
    assert(adc_mock::units==0&&adc_mock::calibrations==0);
    fakeNow=0;adc_mock::initError=adc_mock::mapError=adc_mock::configError=0;
    adc_mock::calError=adc_mock::readError=adc_mock::convertError=0;
    adc_mock::readCalls=adc_mock::readDelayMs=0;adc_mock::failureChannel=5;
    adc_mock::wrongMapping=false;adc_mock::raw[0]=2000;adc_mock::raw[1]=1500;
    adc_mock::mv[0]=600;adc_mock::mv[1]=460;
    adc_mock::deleteCalibrationError=adc_mock::deleteUnitError=0;
    adc_mock::mapped.clear();adc_mock::configured.clear();adc_mock::readChannels.clear();
    adc_mock::rawDrivesMv=false;adc_mock::rawQueue[0].clear();adc_mock::rawQueue[1].clear();
}
// Same rounding as the stub's linear 12 dB/0 dB calibration model.
static int modelMv(int raw,int fullScale){return (raw*fullScale+2047)/4095;}
static void acquire(VoltageMonitor& m){fakeNow+=10;m.poll();}
int main(){
    resetMock();{
        VoltageMonitor m;assert(!m.reading().valid);assert(m.begin());m.poll();
        assert(!adc_mock::readCalls&&!m.reading().valid);
        acquire(m);auto r=m.reading();
        assert(r.valid&&r.railMv[0]==7800&&r.railMv[1]==460&&!r.currentSaturated&&r.sequence==1);  // IO5: INA180 pin mV, gain 1
        assert(adc_mock::mapped==std::vector<int>({4,5}));
        assert(adc_mock::configured==std::vector<int>({4,5}));
        for(unsigned i=0;i<16;++i)assert(adc_mock::readChannels[i]==4+int(i%2));
        m.poll();assert(adc_mock::readCalls==16); // rate bound
        fakeNow+=251;assert(!m.reading().valid&&!strcmp(m.state(),"stale"));
        m.poll();assert(m.reading().valid&&m.reading().sequence==2);
        m.stop();m.poll();assert(!m.reading().valid&&!strcmp(m.state(),"stopped"));
    }std::cout<<"paired calibration, pin mapping, settling, rate, stale, stop PASS\n";
    resetMock();{
        VoltageMonitor m;assert(m.begin());acquire(m);
        adc_mock::readError=ESP_ERR_TIMEOUT;fakeNow+=100;m.poll();
        assert(!m.reading().valid&&!strcmp(m.state(),"adc_read"));
        assert(m.error()==ESP_ERR_TIMEOUT&&m.channel()==1);
        adc_mock::readError=0;adc_mock::mv[0]=500;fakeNow+=100;m.poll();
        assert(m.reading().valid&&m.reading().railMv[0]==6500&&m.reading().sequence==2);
    }std::cout<<"second-channel timeout invalidates entire old pair; bounded retry PASS\n";
    for(int target:{0,1}){
        resetMock();VoltageMonitor m;adc_mock::failureChannel=4+target;
        adc_mock::calError=ESP_ERR_NOT_SUPPORTED;
        assert(!m.begin()&&!m.reading().valid&&!strcmp(m.state(),"adc_calibration"));
        assert(m.error()==ESP_ERR_NOT_SUPPORTED&&m.channel()==target);
        fakeNow+=1000;m.poll();assert(!adc_mock::readCalls); // no nominal fallback
    }std::cout<<"missing factory calibration on either channel fails without fallback PASS\n";
    for(int issue=0;issue<4;++issue){
        resetMock();VoltageMonitor m;
        if(issue==0)adc_mock::wrongMapping=true;
        if(issue==1)adc_mock::mapError=55;
        if(issue==2)adc_mock::initError=66;
        if(issue==3)adc_mock::configError=77;
        assert(!m.begin());fakeNow+=100;m.poll();assert(!adc_mock::readCalls);
    }std::cout<<"mapping, unit allocation and configuration failure PASS\n";
    for(int issue=0;issue<5;++issue){
        resetMock();VoltageMonitor m;assert(m.begin());
        if(issue==0)adc_mock::raw[0]=4095;
        if(issue==1)adc_mock::raw[1]=-1;
        if(issue==2)adc_mock::mv[0]=1001;
        if(issue==3)adc_mock::mv[1]=-1;
        if(issue==4)adc_mock::convertError=88;
        acquire(m);assert(!m.reading().valid);
        assert(strcmp(m.state(),"ready"));
    }std::cout<<"saturation, malformed conversion and calibration failure PASS\n";
    resetMock();{
        VoltageMonitor m;assert(m.begin());adc_mock::readDelayMs=2;acquire(m);
        assert(!m.reading().valid&&!strcmp(m.state(),"adc_deadline"));
        assert(adc_mock::readCalls<=16);
    }std::cout<<"acquisition time bound prevents slow pair publication PASS\n";
    resetMock();{
        VoltageMonitor m;fakeNow=UINT32_MAX-5;assert(m.begin());acquire(m);
        assert(m.reading().valid);fakeNow+=251;assert(!m.reading().valid);
        m.poll();assert(m.reading().valid);
    }std::cout<<"millis wrap preserves settling and sample expiry PASS\n";
    resetMock();{
        VoltageMonitor m;assert(m.begin());adc_mock::mv[0]=adc_mock::mv[1]=0;
        adc_mock::raw[0]=adc_mock::raw[1]=0;acquire(m);
        assert(m.reading().valid&&m.reading().railMv[0]==0&&m.reading().railMv[1]==0);
        adc_mock::mv[0]=adc_mock::mv[1]=1000;
        adc_mock::raw[0]=adc_mock::raw[1]=4094;fakeNow+=100;m.poll();
        assert(m.reading().valid&&m.reading().railMv[0]==13000);
    }std::cout<<"zero and upper calibrated range are measured, not clamped to nominal rails PASS\n";
    resetMock();{
        // INA180 at its rail (stalled motor): IO5 reports saturated full scale, VBAT stays valid.
        VoltageMonitor m;assert(m.begin());adc_mock::raw[1]=4095;adc_mock::mv[1]=3300;acquire(m);
        auto r=m.reading();assert(r.valid&&r.currentSaturated&&r.railMv[1]==3300&&r.railMv[0]==7800);
        adc_mock::raw[0]=4095;fakeNow+=100;m.poll();
        assert(!m.reading().valid&&!strcmp(m.state(),"adc_saturated")&&m.channel()==0);
    }std::cout<<"motor-current saturation reported, VBAT saturation still fails PASS\n";
    resetMock();{
        // INA180 rail threshold (VoltageMonitor::currentRailMv): 3149 mV is a measurement, 3150 mV is the rail.
        static_assert(VoltageMonitor::currentRailMv==3150&&VoltageMonitor::maxPinMv[1]==3300,"rail constants changed");
        VoltageMonitor m;assert(m.begin());adc_mock::raw[1]=3790;adc_mock::mv[1]=3149;acquire(m);
        auto r=m.reading();assert(r.valid&&!r.currentSaturated&&r.pinMv[1]==3149&&r.railMv[1]==3149);
        adc_mock::mv[1]=3150;fakeNow+=100;m.poll();r=m.reading();
        assert(r.valid&&r.currentSaturated&&r.pinMv[1]==3150&&r.railMv[1]==3150);
        // Calibrated values above the 12 dB range (raw < 4095) are clamped to 3300 mV and flagged as the rail.
        adc_mock::raw[1]=4094;adc_mock::mv[1]=3350;fakeNow+=100;m.poll();r=m.reading();
        assert(r.valid&&r.currentSaturated&&r.pinMv[1]==3300&&r.railMv[1]==3300&&r.raw[1]==4094);
        assert(r.railMv[0]==7800&&!strcmp(m.state(),"ready"));
        adc_mock::mv[1]=460;adc_mock::raw[1]=1500;fakeNow+=100;m.poll();
        assert(m.reading().valid&&!m.reading().currentSaturated); // the flag is per sample set, not latched
    }std::cout<<"current rail threshold 3150 mV and >3300 mV clamp PASS\n";
    resetMock();{
        // Raw codes through a linear 12 dB calibration (full scale 3400 mV in the stub) on IO5, 0 dB on IO4.
        adc_mock::rawDrivesMv=true;
        VoltageMonitor m;assert(m.begin());
        adc_mock::raw[0]=2457;adc_mock::raw[1]=2409;acquire(m);auto r=m.reading();
        assert(r.valid&&r.pinMv[0]==modelMv(2457,1000)&&r.railMv[0]==13*modelMv(2457,1000));
        assert(r.pinMv[1]==modelMv(2409,3400)&&r.pinMv[1]==2000&&!r.currentSaturated);
        for(int raw:{3793,3794,4000,4094,4095}){
            adc_mock::raw[1]=raw;fakeNow+=100;m.poll();r=m.reading();
            const int expected=raw==4095?3300:std::min(modelMv(raw,3400),3300);
            assert(r.valid&&r.raw[1]==raw&&r.pinMv[1]==expected&&r.railMv[1]==expected);
            assert(r.currentSaturated==(raw!=3793));
        }
        assert(modelMv(3793,3400)==3149&&modelMv(3794,3400)==3150&&modelMv(4000,3400)>3300);
        // One sample at the rail among eight sets the flag although the 8-sample mean stays far below it.
        adc_mock::raw[1]=2409;adc_mock::rawQueue[1]={3794};fakeNow+=100;m.poll();r=m.reading();
        assert(r.valid&&r.currentSaturated&&r.pinMv[1]==(3150+7*2000+4)/8&&r.pinMv[1]<VoltageMonitor::currentRailMv);
        fakeNow+=100;m.poll();assert(m.reading().valid&&!m.reading().currentSaturated);
        // The 0 dB VBAT channel keeps its own range: top code 4094 is 1000 mV, never clamped.
        adc_mock::raw[0]=4094;fakeNow+=100;m.poll();
        assert(m.reading().valid&&m.reading().pinMv[0]==1000&&m.reading().railMv[0]==13000);
    }std::cout<<"12 dB raw-to-mV model: threshold, clamp and any-sample saturation PASS\n";
    for(bool unitFailure:{false,true}){
        resetMock();VoltageMonitor m;assert(m.begin());acquire(m);
        if(unitFailure)adc_mock::deleteUnitError=99;else adc_mock::deleteCalibrationError=99;
        assert(!m.begin()&&m.cleanupError()==99&&m.error()==99&&!m.reading().valid);
        adc_mock::deleteCalibrationError=adc_mock::deleteUnitError=0;
        assert(m.begin());acquire(m);assert(m.reading().valid);
    }std::cout<<"cleanup failures retained and recovery does not leak handles PASS\n";
    resetMock();assert(!adc_mock::units&&!adc_mock::calibrations);
}
