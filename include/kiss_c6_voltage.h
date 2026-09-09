#pragma once
#ifdef WBB_TOP_BOARD_KISS_C6
#include <stdint.h>
#include "board_profile_kiss_c6.h"
#ifdef WBB_CONTROL_NATIVE_TEST
#include "../control_tests/kiss_adc_stubs.h"
#else
#include <Arduino.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali_scheme.h>
#endif

namespace kiss_c6 {
// Nominal board-divider gain. Factory ADC calibration does not calibrate the
// two external resistors or qualify their off-state behavior.
class VoltageMonitor {
public:
    static constexpr uint32_t gain=13, periodMs=100, settleMs=10;
    static constexpr uint32_t maxSpanMs=25, maxAgeMs=250;
    static constexpr unsigned samples=8;
    struct Reading {
        bool valid=false;
        uint32_t sequence=0, at=0, span=0;
        int raw[2]{}, pinMv[2]{}, railMv[2]{};
    };
    ~VoltageMonitor(){release();}
    VoltageMonitor()=default;
    VoltageMonitor(const VoltageMonitor&)=delete;
    VoltageMonitor& operator=(const VoltageMonitor&)=delete;
    bool begin(){
        enabled=false; data={}; problem="init"; sdkError=0; failingChannel=-1;
        if(!release())return fail("adc_release",releaseError,-1);
        for(int i=0;i<2;++i){
            adc_unit_t unit{};adc_channel_t channel{};
            int rc=adc_oneshot_io_to_channel(i==0?batterySense:motorSense,&unit,&channel);
            if(rc!=ESP_OK)return fail("adc_mapping",rc,i);
            if(unit!=ADC_UNIT_1 || channel!=(i==0?ADC_CHANNEL_4:ADC_CHANNEL_5))
                return fail("adc_mapping",ESP_ERR_INVALID_ARG,i);
            channels[i]=channel;
        }
        adc_oneshot_unit_init_cfg_t cfg{};
        cfg.unit_id=ADC_UNIT_1;cfg.ulp_mode=ADC_ULP_MODE_DISABLE;
        int rc=adc_oneshot_new_unit(&cfg,&unit);
        if(rc!=ESP_OK)return fail("adc_unit",rc,-1);
        adc_oneshot_chan_cfg_t channelCfg{};
        channelCfg.atten=ADC_ATTEN_DB_0;channelCfg.bitwidth=ADC_BITWIDTH_12;
        for(int i=0;i<2;++i){
            rc=adc_oneshot_config_channel(unit,channels[i],&channelCfg);
            if(rc!=ESP_OK)return fail("adc_config",rc,i);
            adc_cali_curve_fitting_config_t calCfg{};
            calCfg.unit_id=ADC_UNIT_1;calCfg.chan=channels[i];
            calCfg.atten=ADC_ATTEN_DB_0;calCfg.bitwidth=ADC_BITWIDTH_12;
            rc=adc_cali_create_scheme_curve_fitting(&calCfg,&cal[i]);
            if(rc!=ESP_OK)return fail("adc_calibration",rc,i);
        }
        enabled=true;problem="settling";began=millis();lastAttempt=began-periodMs;
        return true;
    }
    void stop(){enabled=false;data.valid=false;problem="stopped";}
    void poll(){
        const uint32_t now=millis();
        if(!enabled || uint32_t(now-began)<settleMs || uint32_t(now-lastAttempt)<periodMs)return;
        lastAttempt=now;
        int rawSum[2]{},mvSum[2]{};
        for(unsigned n=0;n<samples;++n)for(int i=0;i<2;++i){
            int raw=-1,mv=-1;
            int rc=adc_oneshot_read(unit,channels[i],&raw);
            if(rc!=ESP_OK){fail("adc_read",rc,i);return;}
            if(raw<0 || raw>=4095){fail("adc_saturated",0,i);return;}
            rc=adc_cali_raw_to_voltage(cal[i],raw,&mv);
            if(rc!=ESP_OK){fail("adc_convert",rc,i);return;}
            // C6 ATTEN0 documented effective range: 0..1000 mV.
            if(mv<0 || mv>1000){fail("adc_range",0,i);return;}
            if(uint32_t(millis()-now)>maxSpanMs){fail("adc_deadline",0,i);return;}
            rawSum[i]+=raw;mvSum[i]+=mv;
        }
        Reading next{};next.valid=true;next.sequence=data.sequence+1;
        if(!next.sequence)next.sequence=1;
        next.at=now;next.span=uint32_t(millis()-now);
        for(int i=0;i<2;++i){
            next.raw[i]=(rawSum[i]+samples/2)/samples;
            next.pinMv[i]=(mvSum[i]+samples/2)/samples;
            next.railMv[i]=(mvSum[i]*gain+samples/2)/samples;
        }
        data=next;problem="none";sdkError=0;failingChannel=-1;
    }
    Reading reading()const{
        Reading result=data;
        result.valid=enabled&&data.valid&&uint32_t(millis()-data.at)<=maxAgeMs;
        return result;
    }
    const char* state()const{
        if(enabled&&data.valid)return reading().valid?"ready":"stale";
        return problem;
    }
    int error()const{return sdkError;}
    int channel()const{return failingChannel;}
    int cleanupError()const{return releaseError;}
private:
    adc_oneshot_unit_handle_t unit=nullptr;
    adc_cali_handle_t cal[2]{};
    adc_channel_t channels[2]{};
    Reading data{};
    uint32_t began=0,lastAttempt=0;
    bool enabled=false;
    const char* problem="not_started";
    int sdkError=0,failingChannel=-1,releaseError=0;
    bool fail(const char* stage,int rc,int channel){
        data.valid=false;problem=stage;sdkError=rc;failingChannel=channel;
        return false;
    }
    bool release(){
        releaseError=0;
        for(auto& handle:cal)if(handle){
            int rc=adc_cali_delete_scheme_curve_fitting(handle);
            if(rc==ESP_OK)handle=nullptr;else if(!releaseError)releaseError=rc;
        }
        if(unit){
            int rc=adc_oneshot_del_unit(unit);
            if(rc==ESP_OK)unit=nullptr;else if(!releaseError)releaseError=rc;
        }
        return !releaseError;
    }
};
} // namespace kiss_c6
#endif
