#ifdef WBB_TOP_BOARD_V2
// Integrated-board runtime. All robot access is serialized in this task.
// The electrical review has not passed: motion stays locked in this build.
#include "top_v2_platform.h"
#if defined(WBB_TOP_BOARD_PICO)
#include "pico_watchdog.h"
#include "pico_rail_service.h"
namespace selectedBoard = pico;
static pico::SafeDefaults boardSafe;
static pico::ExternalWatchdog externalWatchdog(boardSafe);
static constexpr const char* safeDefaultsFailure="PICO_SAFE_DEFAULTS_FAILED";
#elif defined(WBB_TOP_BOARD_KISS_C6)
#include "kiss_c6_gpio_safe.h"
#include "kiss_c6_voltage.h"
#include "kiss_c6_radio_power.h"
namespace selectedBoard = kiss_c6;
static kiss_c6::MotorArmSafeLow boardSafe;
static kiss_c6::VoltageMonitor voltageMonitor;
static kiss_c6::RadioPower radioPower;
static String radioSSID,radioPassword;
#ifndef WBB_CONTROL_NATIVE_TEST
#include <esp_attr.h>
static RTC_NOINIT_ATTR volatile uint32_t radioRestartCookie[2];
#else
static volatile uint32_t radioRestartCookie[2]{};
#endif
static constexpr uint32_t radioResetMagic=0x52504631;
static bool previousRadioPowerReset=false;
static constexpr const char* safeDefaultsFailure="GPIO14_SAFE_LOW_FAILED";
#elif defined(WBB_TOP_BOARD_R3)
#include "r3_gpio_safe.h"
namespace selectedBoard = r3;
static r3::MotorArmSafeLow boardSafe;
static constexpr const char* safeDefaultsFailure="GPIO21_SAFE_LOW_FAILED";
#endif
#if defined(WBB_TOP_BOARD_R3) || defined(WBB_TOP_BOARD_PICO) || defined(WBB_TOP_BOARD_KISS_C6)
static bool robotInitialized=false;
#endif

static RobotController robot;
static ControlTransport transport;
static control::Session owner;
static control::Framer<ControlTransport::maxLine> usbFrame,bleFrame;
static uint32_t framedBLEPeer=0;
static char boot[17],deviceName[32];
static uint8_t key[32]{};
static bool keySet=false,watchdogReady=false,rebootPending=false;
static uint32_t rebootAt=0,rebootQuietAt=0;
static control::Link rebootLink=control::Link::None;
static uint64_t rebootPeer=0;
static uint32_t lastControlAt=0,controlGapMs=0,maxControlGapMs=0,controlOverruns=0;
static bool controlTimingFault=false;
static constexpr uint32_t maxControlIntervalMs=100;
static const char* reason="BOARD_POWER_UNREVIEWED";
static constexpr bool motionQualified=false;

#ifdef WBB_TOP_BOARD_PICO
static bool cellWakeRequest(bool active){return boardSafe.setWakeRequest(active);}
static bool configureTaskWatchdog(uint32_t timeoutMs){
 esp_task_wdt_config_t wdt{timeoutMs,1,true};
 esp_err_t rc=esp_task_wdt_reconfigure(&wdt);
 if(rc==ESP_ERR_INVALID_STATE)rc=esp_task_wdt_init(&wdt);
 return rc==ESP_OK&&(watchdogReady||esp_task_wdt_add(nullptr)==ESP_OK);
}
#endif

static void halt(const char* why){
#if defined(WBB_TOP_BOARD_R3) || defined(WBB_TOP_BOARD_PICO) || defined(WBB_TOP_BOARD_KISS_C6)
 if(robotInitialized){JsonDocument d;d["cmd"]="stop";robot.handleCommand(d);}
#else
 JsonDocument d;d["cmd"]="stop";robot.handleCommand(d);
#endif
#if defined(WBB_TOP_BOARD_R3) || defined(WBB_TOP_BOARD_PICO) || defined(WBB_TOP_BOARD_KISS_C6)
 if(!boardSafe.holdLow())why=safeDefaultsFailure;
#ifdef WBB_TOP_BOARD_PICO
 pico_rails::diagnostics().wakeAborted();
#endif
#else
 digitalWrite(21,LOW);
#endif
 owner.clear();reason=why;
}
#ifdef WBB_TOP_BOARD_KISS_C6
static void restartForRadioPower(){
 halt("RADIO_POWER_LOST");voltageMonitor.stop();
 // RTC no-init survives a software reset. Commit the tag last; never write NVS
 // while supply is failing. The next boot consumes it only for ESP_RST_SW.
 radioRestartCookie[0]=0;radioRestartCookie[1]=~radioResetMagic;
 radioRestartCookie[0]=radioResetMagic;
 esp_restart();
}
#endif
static int hexDigit(char c){
 if(c>='0'&&c<='9')return c-'0';
 if(c>='a'&&c<='f')return c-'a'+10;
 return -1;
}
static bool plainString(JsonVariantConst value){
 if(!value.is<const char*>())return false;
 const JsonString text=value.as<JsonString>();
 return text.c_str()&&text.size()==strlen(text.c_str());
}
// ArduinoJson stops at an object's closing brace. Keep the reader position so
// a valid prefix cannot hide a second command or other trailing bytes.
struct CommandReader {
 const char* next;
 int read(){return *next?static_cast<unsigned char>(*next++):-1;}
 size_t readBytes(char* buffer,size_t length){
  size_t n=0;while(n<length&&*next)buffer[n++]=*next++;return n;
 }
};
static bool parseObject(JsonDocument& document,const char* text){
 CommandReader reader{text};
 if(deserializeJson(document,reader,DeserializationOption::NestingLimit(8))||!document.is<JsonObject>())return false;
 while(*reader.next==' '||*reader.next=='\t'||*reader.next=='\r'||*reader.next=='\n')++reader.next;
 return !*reader.next;
}
static bool unhex(const char* value,uint8_t* result,size_t n){
 if(!value||strlen(value)!=2*n)return false;
 for(size_t i=0;i<n;++i){int a=hexDigit(value[2*i]),b=hexDigit(value[2*i+1]);if(a<0||b<0)return false;result[i]=(a<<4)|b;}
 return true;
}
static bool mac(const char* payload,uint8_t* result){
 return keySet&&mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),key,sizeof(key),
  reinterpret_cast<const uint8_t*>(payload),strlen(payload),result)==0;
}
#ifdef WBB_TOP_BOARD_PICO
static void fillPicoStatus(JsonDocument& d){
 // These are sampled digital levels, not evidence of USB sustain, a battery
 // condition, a voltage window, or motor isolation.
 if(boardSafe.ready()){
  d["logic_flt_n"]=gpio_get_level(gpio_num_t(pico::logicFaultN));
  d["logic_pgood"]=gpio_get_level(gpio_num_t(pico::logicPgood));
 }else{d["logic_flt_n"]=nullptr;d["logic_pgood"]=nullptr;}
 auto& rails=pico_rails::diagnostics();
 if(!boardSafe.ready()||rebootPending||externalWatchdog.failed())rails.stop();
 const auto measurement=rails.status();
 const auto cells=rails.cellStatus();
 d["cell_monitor"]=cells.state;d["cell_error"]=cells.error;
 auto rail=d["rail_monitor"].to<JsonObject>();
 rail["state"]=measurement.state;rail["error"]=measurement.error;rail["bus"]=measurement.bus;
 if(measurement.sample){
  d["raw_logic_5v_mv"]=measurement.sample->millivolts[0];
  d["usb_vbus_mv"]=measurement.sample->millivolts[1];
  d["motor_voltage_mv"]=measurement.sample->millivolts[2];
  rail["seq"]=measurement.sample->sequence;rail["age_us"]=measurement.ageUs;rail["span_us"]=measurement.spanUs;
 }else{
  d["raw_logic_5v_mv"]=nullptr;d["usb_vbus_mv"]=nullptr;d["motor_voltage_mv"]=nullptr;
  rail["seq"]=nullptr;rail["age_us"]=nullptr;rail["span_us"]=nullptr;
 }
 d["source_cycle_enabled"]=false;
 const uint32_t watchdogStatusAt=millis();
 d["external_watchdog_ready"]=externalWatchdog.ready(watchdogStatusAt);
 d["external_watchdog_state"]=externalWatchdog.stateName(watchdogStatusAt);
 d["motor_arm_requested"]=false;d["odometry_qualified"]=false;
 // Encoder counts remain raw diagnostics. The new harness's polarity, CPR
 // and wheel calibration have not been qualified for pose or speed telemetry.
 d["x"]=nullptr;d["y"]=nullptr;d["theta"]=nullptr;
 auto speed=d["wheel_speed_mps"].to<JsonArray>();
 for(int i=0;i<3;++i)speed.add(nullptr);
}
#endif
// Read-only cell report is separate from the high-rate status envelope, keeping
// normal USB/radio packets bounded while retaining detailed failure evidence.
#ifdef WBB_TOP_BOARD_PICO
static void fillCellReport(JsonDocument& d){
 auto& service=pico_rails::diagnostics();
 if(!boardSafe.ready()||rebootPending||externalWatchdog.failed())service.stop();
 const auto c=service.cellStatus();const auto& s=c.sample;const auto& diag=c.diagnostics;
 d["protocol"]=2;d["firmware"]="wbb-control-2";d["profile"]=pico::profile;
 d["kind"]="cells";d["device"]=deviceName;d["uptime_ms"]=millis();
 d["cell_monitor"]=c.state;d["cell_error"]=c.error;d["wake_requested"]=c.wakeRequested;
 d["read_only"]=true;d["motion_enabled"]=false;d["charge_enabled"]=false;
 d["system_protection_qualified"]=false;
 d["configuration_verified"]=diag.configuration_verified;
 d["voltage_profile_verified"]=diag.voltage_profile_verified;
 d["generation"]=diag.generation;d["fresh"]=s.fresh;
 if(diag.response_confirmed)d["device_number"]=diag.device_number;else d["device_number"]=nullptr;
 if(diag.error!=bq76905::Error::None)d["failing_address"]=diag.failing_address;else d["failing_address"]=nullptr;
 d["cell_pair_valid"]=s.cell_pair_valid;d["stack_consistent"]=s.stack_consistent;
 auto cells=d["cell_mv"].to<JsonArray>();
 for(const auto* reading:{&s.lower_cell_mv,&s.upper_cell_mv}){
  if(s.fresh&&reading->available)cells.add(reading->value);else cells.add(nullptr);
 }
 if(s.fresh&&s.stack_mv.available)d["stack_mv"]=s.stack_mv.value;else d["stack_mv"]=nullptr;
 if(s.fresh&&s.ts_adc_counts.available)d["ts_counts"]=s.ts_adc_counts.value;else d["ts_counts"]=nullptr;
 if(s.fresh&&s.ts_nominal_uv.available)d["ts_nominal_uv"]=s.ts_nominal_uv.value;else d["ts_nominal_uv"]=nullptr;
 d["battery_current_ma"]=nullptr;d["pack_temperature_mc"]=nullptr;
 auto alerts=d["safety_alert"].to<JsonArray>();auto faults=d["safety_status"].to<JsonArray>();
 if(s.fresh&&s.status_available){
  d["seq"]=s.sequence;d["age_ms"]=c.ageMs;d["read_span_ms"]=s.read_span_ms;
  d["battery_status"]=s.battery_status;d["alarm_status"]=s.alarm_status;d["alarm_raw"]=s.alarm_raw;
  d["balance_mask"]=s.balancing_mask;d["fets_off_observed"]=diag.fets_off_observed;
  alerts.add(s.safety_alert_a);alerts.add(s.safety_alert_b);faults.add(s.safety_status_a);faults.add(s.safety_status_b);
 }else{
  for(const char* field:{"seq","age_ms","read_span_ms","battery_status","alarm_status","alarm_raw","balance_mask","fets_off_observed"})d[field]=nullptr;
  for(int i=0;i<2;++i){alerts.add(nullptr);faults.add(nullptr);}
 }
}
#endif
#ifdef WBB_TOP_BOARD_KISS_C6
static void fillVoltageReport(JsonDocument& d){
 if(!boardSafe.ready())voltageMonitor.stop();
 const auto v=voltageMonitor.reading();
 d["protocol"]=2;d["firmware"]="wbb-control-2";d["profile"]=kiss_c6::profile;
 d["kind"]="rails";d["device"]=deviceName;d["uptime_ms"]=millis();
 d["state"]=voltageMonitor.state();d["adc_error"]=voltageMonitor.error();
 d["adc_cleanup_error"]=voltageMonitor.cleanupError();
 d["failing_channel"]=voltageMonitor.channel();
 d["fresh"]=v.valid;d["read_only"]=true;d["motion_enabled"]=false;
 d["hardware_qualified"]=false;d["divider_calibrated"]=false;
 d["divider_gain"]=kiss_c6::VoltageMonitor::gain;
 d["wireless_power_state"]=radioPower.state();
 d["wireless_start_attempted"]=radioPower.hasStarted();
 d["previous_radio_power_reset"]=previousRadioPowerReset;
 auto raw=d["adc_raw"].to<JsonArray>();auto pins=d["adc_pin_mv"].to<JsonArray>();
 if(v.valid){
  d["battery_voltage_mv"]=v.railMv[0];d["motor_voltage_mv"]=v.railMv[1];
  d["seq"]=v.sequence;d["age_ms"]=uint32_t(millis()-v.at);d["span_ms"]=v.span;
  for(int i=0;i<2;++i){raw.add(v.raw[i]);pins.add(v.pinMv[i]);}
 }else{
  for(const char* field:{"battery_voltage_mv","motor_voltage_mv","seq","age_ms","span_ms"})d[field]=nullptr;
  for(int i=0;i<2;++i){raw.add(nullptr);pins.add(nullptr);}
 }
}
static void fillKissCalibrationStatus(JsonDocument& d){
 d["odometry_qualified"]=false;
 d["x"]=nullptr;d["y"]=nullptr;d["theta"]=nullptr;
 auto speed=d["wheel_speed_mps"].to<JsonArray>();
 for(int i=0;i<3;++i)speed.add(nullptr);
}
#endif
static void fillStatus(JsonDocument& d){
#if defined(WBB_TOP_BOARD_R3) || defined(WBB_TOP_BOARD_PICO) || defined(WBB_TOP_BOARD_KISS_C6)
 if(!boardSafe.ready()){
  // No robot, encoder, or radio status access in the latched fault path.
  d["protocol"]=2;d["firmware"]="wbb-control-2";d["profile"]=selectedBoard::profile;
  d["boot"]=boot;d["device"]=deviceName;d["uptime_ms"]=millis();d["reset_reason"]=int(esp_reset_reason());
  d["state"]="gpio_fault";d["reason"]=safeDefaultsFailure;
  d["gpio_fault_latched"]=true;d["gpio_error"]=boardSafe.error();d["gpio_release_error"]=boardSafe.releaseError();
  d["read_only"]=true;d["motion_enabled"]=false;d["charge_enabled"]=false;d["power_disconnect"]=false;
  d["pwm_ready"]=false;d["watchdog_ready"]=watchdogReady;d["wireless_control_enabled"]=false;
  d["telemetry_available"]=false;d["x"]=nullptr;d["y"]=nullptr;d["theta"]=nullptr;
  d["motor_voltage_mv"]=nullptr;d["motor_current_ma"]=nullptr;d["rail_3v3_mv"]=nullptr;
  auto counts=d["encoder_counts"].to<JsonArray>();auto speed=d["wheel_speed_mps"].to<JsonArray>();
  auto pwm=d["pwm_command"].to<JsonArray>();auto physical=d["physical_motors"].to<JsonArray>();
  auto headers=d["header_order"].to<JsonArray>();
  for(int i=0;i<3;++i){counts.add(nullptr);speed.add(nullptr);pwm.add(0);physical.add(i+1);}
  for(const char* h:{"M0","M2","M1"})headers.add(h);
  // PWM above is the inhibited command, not a measurement of physical outputs.
  d["owner"]=int(control::Link::None);d["lease_ms"]=control::Session::timeoutMs;
  d["usb_reboot"]=true;d["reboot_pending"]=rebootPending;d["tx_dropped"]=transport.dropped;
#ifdef WBB_TOP_BOARD_PICO
  fillPicoStatus(d);
#endif
#ifdef WBB_TOP_BOARD_KISS_C6
  fillKissCalibrationStatus(d);
#endif
  return;
 }
#endif
 robot.fillStatus(d);
 d["protocol"]=2;d["firmware"]="wbb-control-2";d["profile"]="top_board_v2";
#if defined(WBB_TOP_BOARD_R3) || defined(WBB_TOP_BOARD_PICO) || defined(WBB_TOP_BOARD_KISS_C6)
 d["profile"]=selectedBoard::profile;d["charge_enabled"]=false;
#endif
 d["boot"]=boot;d["device"]=deviceName;d["uptime_ms"]=millis();d["reset_reason"]=int(esp_reset_reason());
 d["motion_enabled"]=motionQualified;d["reason"]=reason;
 d["lease_ms"]=control::Session::timeoutMs;d["owner"]=int(owner.link);
 d["wifi_connected"]=transport.wifiUp;d["wifi_ip"]=WiFi.localIP().toString();
 if(transport.wifiUp)d["wifi_rssi_dbm"]=WiFi.RSSI();else d["wifi_rssi_dbm"]=nullptr;
 d["wifi_reconnects"]=transport.reconnects;d["ble_ready"]=transport.bleReady;d["ble_connected"]=transport.bleUp.load();
 d["authenticated_wireless"]=keySet;d["watchdog_ready"]=watchdogReady;
 d["tx_dropped"]=transport.dropped;d["usb_reboot"]=true;d["reboot_pending"]=rebootPending;
 d["ble_tx_failures"]=transport.notifyFailures.load();
 d["motor_voltage_mv"]=nullptr;d["motor_current_ma"]=nullptr;d["rail_3v3_mv"]=nullptr;
 d["power_disconnect"]=false;
 d["control_gap_ms"]=controlGapMs;d["max_control_gap_ms"]=maxControlGapMs;
 d["control_overruns"]=controlOverruns;d["control_timing_fault"]=controlTimingFault;
#ifdef WBB_TOP_BOARD_PICO
 fillPicoStatus(d);
#endif
#ifdef WBB_TOP_BOARD_KISS_C6
 fillKissCalibrationStatus(d);
#endif
}
static void reply(control::Link link,uint64_t peer,uint32_t id,const char* error=nullptr,bool details=false,bool cells=false,bool rails=false){
 JsonDocument d;d["id"]=id;d["ok"]=error==nullptr;d["error"]=error?error:"";
 d["boot"]=boot;d["lease"]=owner.owns(link,peer)?owner.lease:0;
#ifdef WBB_TOP_BOARD_PICO
 if(cells)fillCellReport(d);else
#endif
#ifdef WBB_TOP_BOARD_KISS_C6
 if(rails)fillVoltageReport(d);else
#endif
 if(details)fillStatus(d);
#ifdef WBB_TOP_BOARD_PICO
 char payload[1792];
#else
 char payload[1200];
#endif
 size_t n=measureJson(d);
 if(n>=sizeof(payload)){++transport.dropped;return;}
 serializeJson(d,payload,sizeof(payload));
 JsonDocument envelope;envelope["payload"]=payload;
 uint8_t digest[32];char signature[65]{};
 if(keySet){
  if(!mac(payload,digest)){++transport.dropped;return;}
  for(int i=0;i<32;++i)snprintf(signature+2*i,3,"%02x",digest[i]);
 }
 envelope["mac"]=signature;
 char output[ControlTransport::maxLine];
 if(measureJson(envelope)+2>=sizeof(output)){++transport.dropped;return;}
 size_t count=serializeJson(envelope,output,sizeof(output));output[count++]='\n';output[count]=0;
#if defined(WBB_TOP_BOARD_R3) || defined(WBB_TOP_BOARD_PICO) || defined(WBB_TOP_BOARD_KISS_C6)
 if(!boardSafe.ready()){
  if(link!=control::Link::USB)return;
  if(!Serial||Serial.availableForWrite()<int(count)||Serial.write(reinterpret_cast<const uint8_t*>(output),count)!=count)
   ++transport.dropped;
  return;
 }
#endif
 transport.send(link,peer,output);
}
static void badInput(control::Link link,uint64_t peer,const char* why){
 if(link==control::Link::USB||owner.owns(link,peer))halt(why);
}
static void scheduleReboot(control::Link link,uint64_t peer,uint32_t id){
 halt("REBOOT_REQUESTED");rebootPending=true;rebootAt=rebootQuietAt=millis();
#ifdef WBB_TOP_BOARD_PICO
 pico_rails::diagnostics().stop();
#endif
 rebootLink=link;rebootPeer=peer;reply(link,peer,id);
}
static void command(control::Link link,uint64_t peer,const char* line){
#if defined(WBB_TOP_BOARD_R3) || defined(WBB_TOP_BOARD_PICO) || defined(WBB_TOP_BOARD_KISS_C6)
 if(!boardSafe.ready()&&link!=control::Link::USB)return;
#endif
 if(owner.expired(millis()))halt("HEARTBEAT_TIMEOUT");
 if(!strcmp(line,"hello")){reply(link,peer,0,nullptr,true);return;}
 JsonDocument envelope,d;
 if(!parseObject(envelope,line)){badInput(link,peer,"BAD_FRAME");return;}
 if(link==control::Link::USB && !envelope["payload"].is<const char*>()){
  d=envelope;
 }else{
  if(!plainString(envelope["payload"])||!plainString(envelope["mac"])){
   badInput(link,peer,"BAD_ENVELOPE");return;
  }
  const char* payload=envelope["payload"]|"";
  const char* signature=envelope["mac"]|"";
  uint8_t expected[32],received[32];
  if(!*payload||strlen(payload)>800||!unhex(signature,received,32)||!mac(payload,expected))return;
  unsigned diff=0;for(int i=0;i<32;++i)diff|=expected[i]^received[i];if(diff)return;
  if(!parseObject(d,payload)){badInput(link,peer,"BAD_JSON");return;}
 }
 uint32_t id=d["id"]|0U;
 if(!plainString(d["cmd"])||!plainString(d["boot"])){
  badInput(link,peer,"BAD_STRING");reply(link,peer,id,"BAD_STRING");return;
 }
 if(!d["id"].is<uint32_t>()||!id||!d["cmd"].is<const char*>()){badInput(link,peer,"BAD_COMMAND");reply(link,peer,id,"BAD_COMMAND");return;}
 const char* op=d["cmd"];
 if(strcmp(d["boot"]|"",boot)){reply(link,peer,id,"BOOT_CHANGED");return;}
 if(!strcmp(op,"status")){reply(link,peer,id,nullptr,true);return;}
#ifdef WBB_TOP_BOARD_KISS_C6
 if(!strcmp(op,"rail_status")){reply(link,peer,id,nullptr,false,false,true);return;}
#endif
#ifdef WBB_TOP_BOARD_PICO
 if(!strcmp(op,"cell_status")){reply(link,peer,id,nullptr,false,true);return;}
#endif
 if(!strcmp(op,"stop")){halt("USER_STOP");reply(link,peer,id,nullptr,true);return;}
 if(rebootPending){reply(link,peer,id,"REBOOT_PENDING");return;}
#if defined(WBB_TOP_BOARD_R3) || defined(WBB_TOP_BOARD_PICO) || defined(WBB_TOP_BOARD_KISS_C6)
 if(!boardSafe.ready()){
  if(!strcmp(op,"reboot")){scheduleReboot(link,peer,id);return;}
  reply(link,peer,id,safeDefaultsFailure,true);return;
 }
#endif
 if(!strcmp(op,"provision")){
  if(link!=control::Link::USB){reply(link,peer,id,"USB_ONLY");return;}
  halt("PROVISIONING");
#if defined(WBB_TOP_BOARD_R3) || defined(WBB_TOP_BOARD_PICO) || defined(WBB_TOP_BOARD_KISS_C6)
  if(!boardSafe.ready()){reply(link,peer,id,safeDefaultsFailure,true);return;}
#endif
  if(!plainString(d["ssid"])||!plainString(d["password"])||!plainString(d["key"])){
   reply(link,peer,id,"INVALID_CONFIG");return;
  }
  const char* ssid=d["ssid"]|"";const char* password=d["password"]|"";const char* secret=d["key"]|"";
  uint8_t parsed[32];
  if(!*ssid||strlen(ssid)>32||strlen(password)>63||(*password&&strlen(password)<8)||!unhex(secret,parsed,32)){
   reply(link,peer,id,"INVALID_CONFIG");return;
  }
  JsonDocument config;config["ssid"]=ssid;config["password"]=password;config["key"]=secret;
  String serialized;serializeJson(config,serialized);Preferences prefs;
  bool ok=prefs.begin("wbb-control",false);
  if(ok){ok=prefs.putString("config",serialized)==serialized.length();prefs.end();}
  // One NVS record makes credentials + control key update together. Apply on reboot.
  reply(link,peer,id,ok?nullptr:"CONFIG_WRITE_FAILED");return;
 }
 if(!strcmp(op,"claim")){
  uint32_t token=esp_random();if(!token)token=1;
  bool ok=owner.claim(link,peer,token,id,millis());
  reply(link,peer,id,ok?nullptr:"OWNER_BUSY");return;
 }
 if(!strcmp(op,"reboot") && link==control::Link::USB){
  scheduleReboot(link,peer,id);return;
 }
 if(!d["lease"].is<uint32_t>()){reply(link,peer,id,"NOT_OWNER");return;}
 const char* err=owner.accept(link,peer,d["lease"],id,millis());
 if(err){
  // Parsing/authentication may cross the deadline after the entry check.
  // accept() clears an expired owner; stop now, before that loss of ownership
  // makes the loop's disconnect/expiry checks unable to find it.
  if(!strcmp(err,"LEASE_EXPIRED"))halt("HEARTBEAT_TIMEOUT");
  else if(!strcmp(err,"REPLAY_OR_REORDER"))halt(err);
  reply(link,peer,id,err);return;
 }
 if(!strcmp(op,"heartbeat")){reply(link,peer,id);return;}
 if(!strcmp(op,"release")){halt("RELEASED");reply(link,peer,id);return;}
 if(!strcmp(op,"reboot")){
  scheduleReboot(link,peer,id);return;
 }
 // No software override is exposed while voltage/current feedback and power
 // gating remain missing. A hardware-qualified revision needs its own review.
 if(!strcmp(op,"start")||!strcmp(op,"goto")||!strcmp(op,"wheel")||!strcmp(op,"arm")||!strcmp(op,"pulse")){
  halt("BOARD_POWER_UNREVIEWED");reply(link,peer,id,"BOARD_POWER_UNREVIEWED");return;
 }
#ifdef WBB_TOP_BOARD_PICO
 if(!strcmp(op,"source_cycle")||!strcmp(op,"logic_cycle")){
  // Report unavailable even if the requester supplies plausible measurements.
  // Battery-only self-reset and esp_restart() cannot qualify a source cycle.
  halt("SOURCE_CYCLE_UNAVAILABLE");reply(link,peer,id,"SOURCE_CYCLE_UNAVAILABLE");return;
 }
#endif
 if(!strcmp(op,"area")||!strcmp(op,"pos")||!strcmp(op,"set_speed")){
  const char* fields[]={"x","y","theta","w","h","spacing","v"};
  for(const char* field:fields)if(!d[field].isNull()&&(!d[field].is<float>()||!isfinite(d[field].as<float>()))){
   halt("BAD_NUMBER");reply(link,peer,id,"BAD_NUMBER");return;
  }
  if((!strcmp(op,"area") && (!(d["w"]|0.0f)||!(d["h"]|0.0f)||(d["w"]|0.0f)<0||(d["h"]|0.0f)<0||
     (d["w"]|0.0f)>5||(d["h"]|0.0f)>5||(d["spacing"]|0.0f)<.005f))||
     (!strcmp(op,"set_speed")&&((d["v"]|0.0f)<=0||(d["v"]|0.0f)>.2f))){reply(link,peer,id,"OUT_OF_RANGE");return;}
  robot.handleCommand(d);reply(link,peer,id,nullptr,true);return;
 }
 halt("BAD_COMMAND");reply(link,peer,id,"BAD_COMMAND");
}
static void receive(control::Link link,uint64_t peer,const uint8_t* bytes,size_t n){
 if(link==control::Link::WiFi){
  // Datagram boundaries prevent cross-client fragment injection.
  control::Framer<ControlTransport::maxLine> frame;
  for(size_t i=0;i<n;++i){auto r=frame.feed(bytes[i]);
   if(r==frame.Overflow){badInput(link,peer,"RX_OVERFLOW");return;}
   if(r==frame.Line){if(i!=n-1){badInput(link,peer,"MULTIPLE_COMMANDS");return;}command(link,peer,frame.data());}
  }
  return;
 }
 auto& frame=link==control::Link::USB?usbFrame:bleFrame;
 if(link==control::Link::BLE && framedBLEPeer!=peer){frame.reset();framedBLEPeer=peer;}
 for(size_t i=0;i<n;++i){auto r=frame.feed(bytes[i]);
  if(r==frame.Overflow)badInput(link,peer,"RX_OVERFLOW");
  if(r==frame.Line)command(link,peer,frame.data());
 }
}
void setup(){
#ifdef WBB_TOP_BOARD_KISS_C6
 previousRadioPowerReset=esp_reset_reason()==ESP_RST_SW&&
     radioRestartCookie[0]==radioResetMagic&&radioRestartCookie[1]==~radioResetMagic;
 radioRestartCookie[0]=0;radioRestartCookie[1]=0;
#endif
#if defined(WBB_TOP_BOARD_R3) || defined(WBB_TOP_BOARD_PICO) || defined(WBB_TOP_BOARD_KISS_C6)
 if(!boardSafe.begin())reason=safeDefaultsFailure;
#else
 digitalWrite(21,LOW);pinMode(21,OUTPUT);
#endif
#ifdef WBB_TOP_BOARD_PICO
 externalWatchdog.begin(millis());
 // Bound NVS/radio initialization, including SDK semaphore waits. No external
 // pulses during startup: safety may clear, but U20 does not drive MCU RUN.
 if(boardSafe.ready())watchdogReady=configureTaskWatchdog(pico::ExternalWatchdog::startupLimitMs);
#endif
#if !defined(WBB_TOP_BOARD_R3) && !defined(WBB_TOP_BOARD_PICO) && !defined(WBB_TOP_BOARD_KISS_C6)
 for(int pin:{0,1,2,3,4,5}){digitalWrite(pin,LOW);pinMode(pin,OUTPUT);}
#endif
 // The selected MotorDriver owns FI/BI after the checked ARM-low step.
 // An earlier Arduino pinMode would enable a retained HIGH latch before that.
 Serial.setTxBufferSize(8192);Serial.setRxBufferSize(2048);
#if defined(WBB_TOP_BOARD_R3) || defined(WBB_TOP_BOARD_PICO) || defined(WBB_TOP_BOARD_KISS_C6)
 // HardwareSerial has no CDC TX timeout or host/cable-present signal.
 // Writes remain capacity-checked in ControlTransport; leases still expire.
 Serial.begin(115200,SERIAL_8N1,selectedBoard::uartRx,selectedBoard::uartTx);
 // The pinned HardwareSerial begin() installs UART_HW_FLOWCTRL_DISABLE.
 // No RTS/CTS pins are assigned; bool(Serial) is not USB power/host detection.
#else
 Serial.begin(115200);Serial.setTxTimeoutMs(2);
#endif
 snprintf(boot,sizeof(boot),"%08lx%08lx",(unsigned long)esp_random(),(unsigned long)esp_random());
 snprintf(deviceName,sizeof(deviceName),"WhiteBoardBot-%06lx",(unsigned long)(ESP.getEfuseMac()&0xffffff));
#if defined(WBB_TOP_BOARD_R3) || defined(WBB_TOP_BOARD_PICO) || defined(WBB_TOP_BOARD_KISS_C6)
 if(!boardSafe.ready()){reply(control::Link::USB,0,0,nullptr,true);return;}
#endif
 robot.begin();
#if defined(WBB_TOP_BOARD_R3) || defined(WBB_TOP_BOARD_PICO) || defined(WBB_TOP_BOARD_KISS_C6)
 robotInitialized=true;
#endif
 halt("BOARD_POWER_UNREVIEWED");
#if defined(WBB_TOP_BOARD_R3) || defined(WBB_TOP_BOARD_PICO) || defined(WBB_TOP_BOARD_KISS_C6)
 if(!boardSafe.ready())return;
#endif
 Preferences prefs;JsonDocument config;
 if(prefs.begin("wbb-control",true)){deserializeJson(config,prefs.getString("config","{}"));prefs.end();}
 keySet=unhex(config["key"]|"",key,sizeof(key));
#ifdef WBB_TOP_BOARD_KISS_C6
 voltageMonitor.begin();
 radioSSID=keySet?(config["ssid"]|""):"";radioPassword=config["password"]|"";
 // ControlTransport::poll services USB without initializing either radio.
 const bool transportReady=true;
#else
 const bool transportReady=transport.begin(deviceName,keySet?(config["ssid"]|""):"",config["password"]|"");
#endif
 if(!transportReady)reason="TRANSPORT_INIT_FAILED";
#ifdef WBB_TOP_BOARD_PICO
 // The task was already enrolled before any blocking initialization. Do not
 // subscribe twice when changing its startup budget to the normal loop budget.
 if(externalWatchdog.startupExpired(millis())){
  externalWatchdog.finishStartup(millis(),false);
  halt(externalWatchdog.failureReason());
  // The TWDT normally fires first; do not renew its deadline if a late SDK
  // call returns before the pending panic/reset is handled.
  esp_restart();return;
 }
 const bool startupWatchdogReady=watchdogReady;
 watchdogReady=configureTaskWatchdog(3000);
 if(watchdogReady)watchdogReady=esp_task_wdt_reset()==ESP_OK;
 externalWatchdog.finishStartup(millis(),startupWatchdogReady&&watchdogReady&&transportReady&&bool(Serial));
 if(externalWatchdog.failed())reason=externalWatchdog.failureReason();
 if(boardSafe.ready()&&!externalWatchdog.failed())pico_rails::diagnostics().begin(cellWakeRequest);
#else
 esp_task_wdt_config_t wdt{3000,1,true};
 esp_err_t rc=esp_task_wdt_reconfigure(&wdt);
 if(rc==ESP_ERR_INVALID_STATE)rc=esp_task_wdt_init(&wdt);
 watchdogReady=rc==ESP_OK&&esp_task_wdt_add(nullptr)==ESP_OK;
 if(!watchdogReady)reason="WATCHDOG_INIT_FAILED";
#endif
 lastControlAt=millis();
}
void loop(){
#ifdef WBB_TOP_BOARD_KISS_C6
 if(!boardSafe.ready())voltageMonitor.stop();
 // Inspect old evidence before replacing it. A stalled loop cannot conceal an
 // expired supply sample by taking a new one before deciding whether to reset.
 if(radioPower.hasStarted()&&radioPower.observe(voltageMonitor.reading(),millis(),false)==kiss_c6::RadioPower::Action::Restart){
  restartForRadioPower();return;
 }
#endif
#ifdef WBB_TOP_BOARD_PICO
 // Inspect before RX/status and again after all work; a resumed late loop
 // cannot hide a stall by replacing its progress timestamp first.
 externalWatchdog.check(millis());
 if(externalWatchdog.failed())halt(externalWatchdog.failureReason());
 if(!boardSafe.ready()||rebootPending||externalWatchdog.failed())pico_rails::diagnostics().stop();
 bool controlCompleted=false;
#endif
#if defined(WBB_TOP_BOARD_R3) || defined(WBB_TOP_BOARD_PICO) || defined(WBB_TOP_BOARD_KISS_C6)
 if(!boardSafe.ready()){
  boardSafe.holdLow();owner.clear();reason=safeDefaultsFailure;
  // An earlier wireless reboot must not block intentional USB recovery.
  if(rebootPending&&rebootLink!=control::Link::USB)rebootPending=false;
  if(rebootPending&&uint32_t(millis()-rebootAt)>=100){esp_restart();return;}
  uint8_t bytes[64];size_t count=0;
  while(count<sizeof(bytes)&&Serial.available())bytes[count++]=Serial.read();
  if(count)receive(control::Link::USB,0,bytes,count);
  // Keep the read-only task alive instead of resetting automatically on its TWDT.
#ifdef WBB_TOP_BOARD_PICO
  if(watchdogReady)watchdogReady=esp_task_wdt_reset()==ESP_OK;
#else
  if(watchdogReady)esp_task_wdt_reset();
#endif
  delay(1);return;
 }
#endif
 static bool usbWasConnected=false;
 bool usbConnected=bool(Serial);
 if(!usbConnected&&usbWasConnected)usbFrame.reset();
 usbWasConnected=usbConnected;
 if(owner.expired(millis()))halt("HEARTBEAT_TIMEOUT");
 if(owner.link!=control::Link::None&&!transport.alive(owner.link,owner.peer))halt("LINK_LOST");
#ifdef WBB_TOP_BOARD_KISS_C6
 // Keep supply monitoring through the bounded application-reboot reply drain.
 if(boardSafe.ready())voltageMonitor.poll();
 const auto radioAction=radioPower.observe(voltageMonitor.reading(),millis(),
                                         boardSafe.ready()&&watchdogReady&&!rebootPending);
 if(radioAction==kiss_c6::RadioPower::Action::Restart){
  restartForRadioPower();return;
 }
 if(radioAction==kiss_c6::RadioPower::Action::Start){
  // Initialization may block in the SDK. Drop any existing control lease first;
  // the already enrolled task watchdog remains active throughout startup.
  halt("RADIO_STARTING");
  if(!boardSafe.ready()){restartForRadioPower();return;}
  const bool ready=transport.begin(deviceName,radioSSID.c_str(),radioPassword.c_str());
  radioPower.started(ready);reason=ready?"BOARD_POWER_UNREVIEWED":"TRANSPORT_INIT_FAILED";
  voltageMonitor.poll();
  if(radioPower.observe(voltageMonitor.reading(),millis(),false)==kiss_c6::RadioPower::Action::Restart){
   restartForRadioPower();return;
  }
 }
 // ADC conversion can consume time; do not retain an expired control owner.
 if(owner.expired(millis()))halt("HEARTBEAT_TIMEOUT");
#endif
 if(transport.takeOverflow()){bleFrame.feed(0);if(owner.link==control::Link::BLE)halt("RX_OVERFLOW");}
#if defined(WBB_TOP_BOARD_R3) || defined(WBB_TOP_BOARD_PICO) || defined(WBB_TOP_BOARD_KISS_C6)
 if(!boardSafe.ready()){delay(1);return;}
#endif
 if(rebootPending){
  if(transport.txPending(rebootLink,rebootPeer))rebootQuietAt=millis();
  if(uint32_t(millis()-rebootQuietAt)>=100||uint32_t(millis()-rebootAt)>=2500){
   halt("REBOOT_REQUESTED");
#if defined(WBB_TOP_BOARD_R3) || defined(WBB_TOP_BOARD_PICO) || defined(WBB_TOP_BOARD_KISS_C6)
   if(!boardSafe.ready()){delay(1);return;}
#endif
   esp_restart();return;
  }
 }
 transport.poll(receive);
#if defined(WBB_TOP_BOARD_R3) || defined(WBB_TOP_BOARD_PICO) || defined(WBB_TOP_BOARD_KISS_C6)
 if(!boardSafe.ready()){delay(1);return;}
#endif
 uint32_t now=millis();
 uint32_t elapsed=now-lastControlAt;
 if(elapsed>=CONTROL_PERIOD_MS){
  lastControlAt=now;controlGapMs=elapsed;
  if(elapsed>maxControlGapMs)maxControlGapMs=elapsed;
  if(elapsed>maxControlIntervalMs){++controlOverruns;controlTimingFault=true;halt("CONTROL_OVERRUN");}
#if defined(WBB_TOP_BOARD_R3) || defined(WBB_TOP_BOARD_PICO) || defined(WBB_TOP_BOARD_KISS_C6)
  if(!boardSafe.ready()){delay(1);return;}
#endif
  robot.update(elapsed/1000.0f);
#ifdef WBB_TOP_BOARD_PICO
  controlCompleted=true;
#endif
 }
 if(!motionQualified){
#if defined(WBB_TOP_BOARD_PICO)
  if(!boardSafe.holdPowerOff())halt(safeDefaultsFailure);
#elif defined(WBB_TOP_BOARD_R3) || defined(WBB_TOP_BOARD_KISS_C6)
  if(!boardSafe.holdLow())halt(safeDefaultsFailure);
#else
  digitalWrite(21,LOW);
#endif
 }
#ifdef WBB_TOP_BOARD_PICO
 if(!boardSafe.ready()||rebootPending||externalWatchdog.failed())pico_rails::diagnostics().stop();
 else pico_rails::diagnostics().tick();
 if(watchdogReady)watchdogReady=esp_task_wdt_reset()==ESP_OK;
 if(!boardSafe.ready())externalWatchdog.stop(pico::ExternalWatchdog::State::GpioFault);
 else if(controlTimingFault)externalWatchdog.stop(pico::ExternalWatchdog::State::Stale);
 else if(!watchdogReady)externalWatchdog.stop(pico::ExternalWatchdog::State::InitFailed);
 else {
  externalWatchdog.check(millis());
  // The transport poll, robot update, rail service, ARM/cycle LOW writes and task-watchdog
  // reset have all returned. RX traffic/host leases alone never earn an edge.
  if(controlCompleted)externalWatchdog.completedControl(millis());
 }
 if(externalWatchdog.failed())halt(externalWatchdog.failureReason());
 if(!boardSafe.ready()||rebootPending||externalWatchdog.failed())pico_rails::diagnostics().stop();
#else
 if(watchdogReady)esp_task_wdt_reset();
#endif
 delay(1);
}
#endif
