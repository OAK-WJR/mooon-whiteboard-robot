#pragma once
// SDK boundary double only. Production monitor/state/conversion code is shared.
#include <vector>
constexpr int ESP_ERR_INVALID_ARG=0x102,ESP_ERR_TIMEOUT=0x107,ESP_ERR_NOT_SUPPORTED=0x106;
enum adc_unit_t{ADC_UNIT_1,ADC_UNIT_2};
enum adc_channel_t{ADC_CHANNEL_4=4,ADC_CHANNEL_5=5};
enum adc_atten_t{ADC_ATTEN_DB_0};
enum adc_bitwidth_t{ADC_BITWIDTH_12=12};
enum adc_ulp_mode_t{ADC_ULP_MODE_DISABLE};
struct adc_oneshot_unit_init_cfg_t{adc_unit_t unit_id;int clk_src;adc_ulp_mode_t ulp_mode;};
struct adc_oneshot_chan_cfg_t{adc_atten_t atten;adc_bitwidth_t bitwidth;};
struct adc_cali_curve_fitting_config_t{adc_unit_t unit_id;adc_channel_t chan;adc_atten_t atten;adc_bitwidth_t bitwidth;};
struct FakeADCUnit{};
struct FakeADCCal{int channel;};
using adc_oneshot_unit_handle_t=FakeADCUnit*;
using adc_cali_handle_t=FakeADCCal*;
namespace adc_mock {
inline int initError=0,mapError=0,configError=0,calError=0,readError=0,convertError=0;
inline int failureChannel=5,raw[2]={2000,1500},mv[2]={600,460};
inline int deleteCalibrationError=0,deleteUnitError=0;
inline unsigned readCalls=0,readDelayMs=0;
inline int units=0,calibrations=0;
inline bool wrongMapping=false;
inline std::vector<int> mapped,configured,readChannels;
}
inline int adc_oneshot_io_to_channel(int gpio,adc_unit_t* unit,adc_channel_t* ch){
    adc_mock::mapped.push_back(gpio);if(adc_mock::mapError)return adc_mock::mapError;
    *unit=ADC_UNIT_1;*ch=adc_channel_t(adc_mock::wrongMapping?5:gpio);return 0;
}
inline int adc_oneshot_new_unit(const adc_oneshot_unit_init_cfg_t* cfg,adc_oneshot_unit_handle_t* h){
    if(adc_mock::initError)return adc_mock::initError;
    if(cfg->unit_id!=ADC_UNIT_1)return ESP_ERR_INVALID_ARG;
    *h=new FakeADCUnit;++adc_mock::units;return 0;
}
inline int adc_oneshot_config_channel(adc_oneshot_unit_handle_t,adc_channel_t ch,const adc_oneshot_chan_cfg_t* cfg){
    adc_mock::configured.push_back(ch);
    if(ch==adc_mock::failureChannel&&adc_mock::configError)return adc_mock::configError;
    return cfg->atten==ADC_ATTEN_DB_0&&cfg->bitwidth==ADC_BITWIDTH_12?0:ESP_ERR_INVALID_ARG;
}
inline int adc_cali_create_scheme_curve_fitting(const adc_cali_curve_fitting_config_t* cfg,adc_cali_handle_t* h){
    if(cfg->chan==adc_mock::failureChannel&&adc_mock::calError)return adc_mock::calError;
    if(cfg->unit_id!=ADC_UNIT_1||cfg->atten!=ADC_ATTEN_DB_0||cfg->bitwidth!=ADC_BITWIDTH_12)return ESP_ERR_INVALID_ARG;
    *h=new FakeADCCal{cfg->chan};++adc_mock::calibrations;return 0;
}
inline int adc_oneshot_read(adc_oneshot_unit_handle_t,adc_channel_t ch,int* raw){
    ++adc_mock::readCalls;adc_mock::readChannels.push_back(ch);fakeNow+=adc_mock::readDelayMs;
    if(ch==adc_mock::failureChannel&&adc_mock::readError)return adc_mock::readError;
    *raw=adc_mock::raw[ch-4];return 0;
}
inline int adc_cali_raw_to_voltage(adc_cali_handle_t h,int,int* mv){
    if(h->channel==adc_mock::failureChannel&&adc_mock::convertError)return adc_mock::convertError;
    *mv=adc_mock::mv[h->channel-4];return 0;
}
inline int adc_cali_delete_scheme_curve_fitting(adc_cali_handle_t h){
    if(h->channel==adc_mock::failureChannel&&adc_mock::deleteCalibrationError)return adc_mock::deleteCalibrationError;
    delete h;--adc_mock::calibrations;return 0;
}
inline int adc_oneshot_del_unit(adc_oneshot_unit_handle_t h){
    if(adc_mock::deleteUnitError)return adc_mock::deleteUnitError;
    delete h;--adc_mock::units;return 0;
}
