// Dedicated runtime for the measured Waveshare C6 + adapter + bottom board.
// Physical wheel order 1,2,3 (NOT header order M0,M1,M2). No other PCB profile.
#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <esp_eap_client.h>
#include <esp_wifi.h>
#include <mbedtls/x509_crt.h>
#include <memory>
#include <SPI.h>
#include <Adafruit_ST7789.h>
#include <soc/gpio_reg.h>
#include <driver/gpio.h>
#include <esp_app_desc.h>
#include "control.h"
#include "web_ui.h"
#include "network_config.h"
#include "network_request.h"

struct Pins {uint8_t fi,bi,a,b;int sign;};
static constexpr Pins pins[]={{0,1,16,17,1},{4,5,20,23,1},{2,3,18,19,-1}};
static volatile uint32_t counts[3]={},bad[3]={},edges[3]={};
static volatile uint32_t aEdges[3]={},bEdges[3]={};
static volatile uint8_t previous[3]={};
static portMUX_TYPE encoderMux=portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t controlMutex;
static drive::Control control;
static bool ready=false,calibrated=false;
// Candidate selection is USB-only and session-local until measured validation.
static bool robustValidated=false;
static constexpr const char* robustRevision="robust-wheels-19-drive-ux";
static constexpr const char* holdRevision="encoder-hold-6";
static bool holdValidated=false;
// Persist validation only for this exact firmware image. A new build relocks it.
struct SavedController {uint8_t sha[32],mode,robust,hold,version;};
static bool controllerSaved=false;
struct ControlSample {uint32_t ms,commandMs;float speed[3],reference[3];int pwm[3];};
static ControlSample controlSamples[8];
static unsigned sampleHead=0,sampleCount=0;
static bool traceEnabled=false;
static bool pwmAttached[6]={};
static const char* outputFault=nullptr;
static uint8_t gainMask=0;
static uint32_t session=0;
static char apName[32],apPass[16],webToken[24];
static WebServer server(80);
static bool httpStarted=false;
static uint32_t httpSocketGeneration=0,httpRestarts=0;
static Adafruit_ST7789 lcd(&SPI,14,15,21);
static bool lcdReady=false,apReady=false;
static Preferences prefs;
static drive::NetworkCredentials homeNetwork;
static std::unique_ptr<drive::NetworkCredentials> pendingNetwork;
static drive::NetworkPolicy networkPolicy;
static char networkHost[24];
static bool mdnsReady=false;
static const char* networkError=nullptr;
// Wi-Fi callbacks run on a different task. Copy only non-secret diagnostic
// fields under their own lock; never touch motor state or credentials here.
struct NetworkObservation {
 bool associated=false;
 uint8_t channel=0,reason=0;
 uint32_t failures=0,reasonMs=0;
 uint32_t socketGeneration=0;
};
static NetworkObservation networkObservation;
static portMUX_TYPE networkObservationMux=portMUX_INITIALIZER_UNLOCKED;
static bool networkScanPending=false,networkScanRunning=false;
static int networkScanResult=-2,networkScanMatches=0;
static int networkScanRssi=-127,networkScanChannel=0,networkScanAuth=0;
static uint32_t networkScanStarted=0;
static bool networkBusy(){return networkPolicy.blocksArm()||networkScanPending||networkScanRunning;}
static NetworkObservation networkSnapshot(){
 portENTER_CRITICAL(&networkObservationMux);
 auto observation=networkObservation;
 portEXIT_CRITICAL(&networkObservationMux);
 return observation;
}
static void observeNetwork(WiFiEvent_t event,WiFiEventInfo_t info){
 portENTER_CRITICAL(&networkObservationMux);
 if(event==ARDUINO_EVENT_WIFI_STA_CONNECTED){
  networkObservation.associated=true;networkObservation.channel=info.wifi_sta_connected.channel;
 }else if(event==ARDUINO_EVENT_WIFI_STA_DISCONNECTED){
  networkObservation.associated=false;
  ++networkObservation.socketGeneration;
  // A local disconnect at the attempt deadline must not overwrite the useful
  // radio/authentication failure that preceded it.
  if(info.wifi_sta_disconnected.reason!=WIFI_REASON_ASSOC_LEAVE&&info.wifi_sta_disconnected.reason!=WIFI_REASON_STA_LEAVING){
   networkObservation.reason=info.wifi_sta_disconnected.reason;
   networkObservation.reasonMs=millis();++networkObservation.failures;
  }
 }else if(event==ARDUINO_EVENT_WIFI_STA_STOP){
  networkObservation.associated=false;
  ++networkObservation.socketGeneration;
 }else if(event==ARDUINO_EVENT_WIFI_STA_GOT_IP){
  ++networkObservation.socketGeneration;
 }
 portEXIT_CRITICAL(&networkObservationMux);
}
static void recoverHttpServer(){
 // ESP-IDF invalidates TCP sockets on STA disconnect/IP changes. Event callbacks
 // only publish a generation; recreate WebServer sockets in the main loop.
 auto observation=networkSnapshot();
 if(!httpStarted||observation.socketGeneration==httpSocketGeneration)return;
 xSemaphoreTake(controlMutex,portMAX_DELAY);
 if(control.armed)control.faultStop("NETWORK_LOST",millis(),0);
 xSemaphoreGive(controlMutex);
 server.client().stop();server.stop();server.begin();
 httpSocketGeneration=observation.socketGeneration;++httpRestarts;
}
static const char* authName(uint8_t auth){
 switch(auth){case drive::NetworkCredentials::Peap:return "peap";case drive::NetworkCredentials::TtlsMschapv2:return "ttls_mschapv2";case drive::NetworkCredentials::TtlsPap:return "ttls_pap";default:return "personal";}
}
static uint32_t lastControl=0;
static const int8_t QT[16]={0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};

static bool webAccess(){
 // Direct access on the robot AP and the owner's explicitly selected networks.
 if(server.client().localIP()==WiFi.softAPIP())return true;
 if(WiFi.status()==WL_CONNECTED&&server.client().localIP()==WiFi.localIP()&&
    drive::passwordlessLocalNetwork(homeNetwork))return true;
 if(server.authenticate("wbb",apPass))return true;
 server.requestAuthentication(DIGEST_AUTH,"WhiteBoardBot","Use wbb and the robot hotspot password.");
 return false;
}

static void networkStatus(JsonObject n){
 bool connected=WiFi.status()==WL_CONNECTED;
 auto observation=networkSnapshot();
 const auto& profile=pendingNetwork?*pendingNetwork:homeNetwork;
 n["configured"]=networkPolicy.configured;n["ssid"]=networkPolicy.configured?profile.ssid:"";
 n["auth"]=authName(profile.auth);n["server_name"]=profile.server;n["error"]=networkError?networkError:"";
 n["connected"]=connected;n["busy"]=networkBusy();
 n["configure_over_lan"]=true;n["password_saved"]=networkPolicy.configured&&profile.password[0]!=0;
 n["associated"]=observation.associated;n["channel"]=observation.channel;
 n["attempts"]=networkPolicy.attempts;n["attempt_limit"]=networkPolicy.attemptLimit;
 n["disconnect_reason"]=observation.reason;
 n["disconnect_name"]=observation.reason?WiFi.STA.disconnectReasonName(wifi_err_reason_t(observation.reason)):"";
 n["disconnect_ms"]=observation.reasonMs;n["failure_events"]=observation.failures;
 n["http_recovery_revision"]=1;n["http_restarts"]=httpRestarts;
 n["scan_state"]=networkScanPending||networkScanRunning?"scanning":networkScanResult>=0?"complete":networkScanResult==-2?"not_run":"failed";
 n["scan_ap_count"]=networkScanResult>=0?networkScanResult:0;n["scan_matches"]=networkScanMatches;
 n["scan_channel"]=networkScanChannel;n["scan_auth"]=networkScanAuth;n["scan_rssi_dbm"]=networkScanRssi;
 n["state"]=connected?"connected":networkPolicy.pending?"applying":networkPolicy.connecting?"connecting":networkPolicy.exhausted()?"check_settings":networkPolicy.configured?"retry_wait":"ap_only";
 n["station_ip"]=connected?WiFi.localIP().toString():String("");
 n["station_url"]=connected?String("http://")+WiFi.localIP().toString()+"/":String("");
 n["hostname"]=networkHost;n["mdns_url"]=mdnsReady?String("http://")+networkHost+".local/":String("");
 n["ap_ssid"]=apName;n["ap_url"]=String("http://")+WiFi.softAPIP().toString()+"/";
 if(connected)n["rssi_dbm"]=WiFi.RSSI();else n["rssi_dbm"]=nullptr;
 // Never expose the home password or return credentials to a browser.
}
static void networkTick(){
 bool armed; xSemaphoreTake(controlMutex,portMAX_DELAY);armed=control.armed;xSemaphoreGive(controlMutex);
 // Explicit USB diagnostic only. Run the radio operation outside the control
 // lock and defer connection retries until it finishes. No credentials sent.
 if(networkScanPending||networkScanRunning){
  if(armed)return;
  if(networkScanPending){
   networkScanPending=false;networkScanStarted=millis();
   networkScanResult=WiFi.scanNetworks(true,true);
   networkScanRunning=networkScanResult==WIFI_SCAN_RUNNING;
   if(!networkScanRunning){networkScanResult=-3;WiFi.scanDelete();}
   return;
  }
  int found=WiFi.scanComplete();
  if(found==WIFI_SCAN_RUNNING&&uint32_t(millis()-networkScanStarted)<15000)return;
  if(found==WIFI_SCAN_RUNNING)esp_wifi_scan_stop();
  networkScanResult=found>=0?found:-3;
  for(int i=0;i<found&&i<256;i++)if(WiFi.SSID(i)==homeNetwork.ssid){
   ++networkScanMatches;
   if(WiFi.RSSI(i)>networkScanRssi){networkScanRssi=WiFi.RSSI(i);networkScanChannel=WiFi.channel(i);networkScanAuth=WiFi.encryptionType(i);}
  }
  WiFi.scanDelete();networkScanRunning=false;return;
 }
 auto action=networkPolicy.tick(millis(),WiFi.status()==WL_CONNECTED,armed);
 if(networkPolicy.linkLost){
  xSemaphoreTake(controlMutex,portMAX_DELAY);
  if(control.armed)control.faultStop("NETWORK_LOST",millis(),0);
  xSemaphoreGive(controlMutex);
 }
 if(action!=drive::NetworkPolicy::None){
  // Radio and flash calls stay outside the motor-control mutex.
  if(!WiFi.disconnect(false,false)){networkError="WIFI_DISCONNECT_FAILED";return;}
  // Disabling EAP clears all SDK-held CA/identity pointers before replacing the profile.
  esp_err_t disabled=esp_wifi_sta_enterprise_disable();
  if(disabled!=ESP_OK&&disabled!=ESP_ERR_INVALID_STATE){networkError="EAP_RESET_FAILED";return;}
  if(pendingNetwork){homeNetwork=*pendingNetwork;pendingNetwork.reset();}
  if(action==drive::NetworkPolicy::Connect){
   networkError=nullptr;
   if(homeNetwork.auth==drive::NetworkCredentials::Personal){
    WiFi.setMinSecurity(homeNetwork.password[0]?WIFI_AUTH_WPA2_PSK:WIFI_AUTH_OPEN);
    WiFi.begin(homeNetwork.ssid,homeNetwork.password);
   }else{
    // Configure without associating; every verification/credential API must
    // succeed before the driver is allowed to send an EAP request.
    if(WiFi.begin(homeNetwork.ssid,nullptr,0,nullptr,false)==WL_CONNECT_FAILED){networkError="EAP_CONFIG_FAILED";return;}
    const char* identity=homeNetwork.identity[0]?homeNetwork.identity:homeNetwork.username;
    bool peap=homeNetwork.auth==drive::NetworkCredentials::Peap;
    wifi_config_t conf{};
    esp_err_t result=esp_wifi_get_config(WIFI_IF_STA,&conf);
    conf.sta.threshold.authmode=WIFI_AUTH_WPA2_ENTERPRISE;
    if(result==ESP_OK)result=esp_wifi_set_config(WIFI_IF_STA,&conf);
    if(result==ESP_OK)result=esp_eap_client_set_eap_methods(peap?ESP_EAP_TYPE_PEAP:ESP_EAP_TYPE_TTLS);
    if(result==ESP_OK)result=esp_eap_client_set_domain_name(homeNetwork.server);
    if(result==ESP_OK)result=esp_eap_client_use_default_cert_bundle(false);
    if(result==ESP_OK)result=esp_eap_client_set_ca_cert(reinterpret_cast<const unsigned char*>(homeNetwork.ca),strlen(homeNetwork.ca)+1);
    if(result==ESP_OK)result=esp_eap_client_set_disable_time_check(false);
    if(result==ESP_OK)result=esp_eap_client_set_identity(reinterpret_cast<const unsigned char*>(identity),strlen(identity));
    if(result==ESP_OK)result=esp_eap_client_set_username(reinterpret_cast<const unsigned char*>(homeNetwork.username),strlen(homeNetwork.username));
    if(result==ESP_OK)result=esp_eap_client_set_password(reinterpret_cast<const unsigned char*>(homeNetwork.password),strlen(homeNetwork.password));
    if(result==ESP_OK&&!peap)result=esp_eap_client_set_ttls_phase2_method(homeNetwork.auth==drive::NetworkCredentials::TtlsPap?ESP_EAP_TTLS_PHASE2_PAP:ESP_EAP_TTLS_PHASE2_MSCHAPV2);
    if(result==ESP_OK)result=esp_wifi_sta_enterprise_enable();
    if(result==ESP_OK)result=esp_wifi_connect();
    if(result!=ESP_OK){esp_wifi_sta_enterprise_disable();networkError="EAP_CONFIG_FAILED";}
   }
  }else if(networkPolicy.configured&&!networkError){
   networkError="WIFI_CONNECT_TIMEOUT";
  }else if(!networkPolicy.configured){networkError=nullptr;}
 }
}
// Main-loop only, outside controlMutex: flash/radio operations must not block
// the control task. Commands and the HTTP server execute on this same thread.
static const char* saveNetworkConfiguration(JsonVariantConst d,bool usb,bool* unchanged=nullptr){
 if(unchanged)*unchanged=false;
 bool forget=false;
 auto next=std::unique_ptr<drive::NetworkCredentials>(new(std::nothrow) drive::NetworkCredentials{});
 if(!next)return "NO_MEMORY";
 if(const char* error=drive::parseNetworkRequest(d,usb,*next,forget,&homeNetwork))return error;
 if(!forget&&next->auth!=drive::NetworkCredentials::Personal){
   mbedtls_x509_crt ca;mbedtls_x509_crt_init(&ca);
   int result=mbedtls_x509_crt_parse(&ca,reinterpret_cast<const unsigned char*>(next->ca),strlen(next->ca)+1);
   bool valid=result==0&&mbedtls_x509_crt_get_ca_istrue(&ca)==1;mbedtls_x509_crt_free(&ca);
   if(!valid)return "INVALID_CA";
 }
 xSemaphoreTake(controlMutex,portMAX_DELAY);
 float speed[3];for(int i=0;i<3;i++)speed[i]=control.wheel[i].measured;
 bool safe=!networkBusy()&&drive::canConfigureNetwork(control.armed,ready,uint32_t(millis()-lastControl),speed);
 xSemaphoreGive(controlMutex);
 if(!safe)return "STOP_BEFORE_WIFI_CHANGE";
 // Re-saving the connected profile must not interrupt an already working link.
 if(!forget&&WiFi.status()==WL_CONNECTED&&drive::sameNetwork(*next,homeNetwork)){
  if(unchanged)*unchanged=true;
  return nullptr;
 }
 // One versioned blob prevents a partially updated SSID/password pair.
 bool saved=forget?(!prefs.isKey("home-wifi")||prefs.remove("home-wifi")):
                  prefs.putBytes("home-wifi",next.get(),sizeof(*next))==sizeof(*next);
 if(!saved)return "SAVE_FAILED";
 pendingNetwork=std::move(next);
 networkScanResult=-2;networkScanMatches=0;networkScanRssi=-127;networkScanChannel=networkScanAuth=0;
 networkPolicy.configure(millis(),!forget,pendingNetwork->auth!=drive::NetworkCredentials::Personal);
 return nullptr;
}
static void configureNetwork(){
 if(!webAccess())return;
 if(server.header("X-Control-Token")!=webToken){server.send(403,"application/json","{\"ok\":false,\"error\":\"BAD_TOKEN\"}");return;}
 // Use the same local-access and request-token checks as the drive page.
 // The stopped-wheel check below applies on both STA and AP interfaces.
 if(server.arg("plain").length()>8192){server.send(413);return;}
 JsonDocument d;auto parseError=deserializeJson(d,server.arg("plain"));
 bool unchanged=false;
 const char* error=parseError?"BAD_JSON":saveNetworkConfiguration(d.as<JsonVariantConst>(),false,&unchanged);
 if(error){
  int status=!strcmp(error,"STOP_BEFORE_WIFI_CHANGE")?409:!strcmp(error,"SAVE_FAILED")?500:!strcmp(error,"NO_MEMORY")?503:400;
  JsonDocument reply;reply["ok"]=false;reply["error"]=error;String text;serializeJson(reply,text);
  server.send(status,"application/json",text);return;
 }
 server.sendHeader("Cache-Control","no-store");
 server.send(200,"application/json",unchanged?"{\"ok\":true,\"saved\":true,\"unchanged\":true}":"{\"ok\":true,\"saved\":true,\"unchanged\":false}");
}

static void IRAM_ATTR encoder(void* arg){
 const unsigned i=uintptr_t(arg);uint32_t in=REG_READ(GPIO_IN_REG);
 uint8_t ab=(((in>>pins[i].a)&1)<<1)|((in>>pins[i].b)&1);
 portENTER_CRITICAL_ISR(&encoderMux);
 uint8_t old=previous[i];
 if(ab!=old){edges[i]++;if((ab^old)&2)aEdges[i]++;if((ab^old)&1)bEdges[i]++;
  if((ab^old)==3)bad[i]++;
  else counts[i]+=QT[(old<<2)|ab];
  previous[i]=ab;}
 portEXIT_CRITICAL_ISR(&encoderMux);
}
static void snapshot(uint32_t c[3],uint32_t b[3],uint32_t e[3],uint32_t* a=nullptr,uint32_t* z=nullptr){
 portENTER_CRITICAL(&encoderMux);
 for(int i=0;i<3;i++){c[i]=counts[i];b[i]=bad[i];e[i]=edges[i];if(a)a[i]=aEdges[i];if(z)z[i]=bEdges[i];}
 portEXIT_CRITICAL(&encoderMux);
}
static void outputLow(int index){
 const uint8_t pin=index%2?pins[index/2].bi:pins[index/2].fi;
 // Preload GPIO latch before detaching PWM, then select GPIO explicitly.
 gpio_set_level(gpio_num_t(pin),0);
 if(pwmAttached[index]){
  ledcWrite(pin,0);
  if(!ledcDetach(pin)){ready=false;outputFault="PWM_DETACH_FAULT";return;}
  pwmAttached[index]=false;
  gpio_reset_pin(gpio_num_t(pin));
 }
 bool ok=gpio_set_level(gpio_num_t(pin),0)==ESP_OK;
 ok=(gpio_set_direction(gpio_num_t(pin),GPIO_MODE_INPUT_OUTPUT)==ESP_OK)&&ok;
 if(!ok){ready=false;outputFault="GPIO_LOW_FAULT";}
}
static void low(){for(int i=0;i<6;i++)outputLow(i);}
static bool attachOutput(int index){
 if(pwmAttached[index])return true;
 uint8_t p=index%2?pins[index/2].bi:pins[index/2].fi;
 if(!(pwmAttached[index]=ledcAttachChannel(p,20000,8,index)))return false;
 bool ok=ledcWrite(p,0);gpio_input_enable(gpio_num_t(p));return ok;
}
static void motorWrite(int i,int value){
 if(!ready||value==0){outputLow(2*i);outputLow(2*i+1);return;}
 if(!attachOutput(2*i)||!attachOutput(2*i+1)){ready=false;outputFault="PWM_ATTACH_FAULT";low();return;}
 bool ok;
 if(value>0)ok=ledcWrite(pins[i].bi,0)&&ledcWrite(pins[i].fi,value);
 else ok=ledcWrite(pins[i].fi,0)&&ledcWrite(pins[i].bi,-value);
 if(!ok){ready=false;outputFault="PWM_WRITE_FAULT";control.faultStop(outputFault,millis(),0,i);low();}
}
static void controlTask(void*){
 drive::FeedbackWatch feedback[3];
 uint32_t prev[3]={},b[3],e[3];snapshot(prev,b,e);
 TickType_t wake=xTaskGetTickCount();uint32_t before=millis();
 for(;;){
  vTaskDelayUntil(&wake,pdMS_TO_TICKS(10));uint32_t now=millis();float dt=uint32_t(now-before)/1000.f;before=now;
  uint32_t c[3];snapshot(c,b,e);
  xSemaphoreTake(controlMutex,portMAX_DELAY);
  // Capture the actual previous-interval actuator duty before any fault clears it.
  for(int i=0;i<3;i++)control.applied[i]=
   int(pwmAttached[2*i]?ledcRead(pins[i].fi):0)-int(pwmAttached[2*i+1]?ledcRead(pins[i].bi):0);
  float signedDelta[3];
  for(int i=0;i<3;i++){
   int32_t delta=int32_t(c[i]-prev[i]);prev[i]=c[i];
   signedDelta[i]=float(delta)*pins[i].sign;
   float measured=dt>0?signedDelta[i]/dt:0;
   control.wheel[i].measured+=.4f*(measured-control.wheel[i].measured);
  }
  control.observe(now,signedDelta,dt);
  if(digitalRead(9)==LOW)control.stop("BUTTON_STOP");
  for(int i=0;i<3;i++)if(!feedback[i].update(now,c[i],e[i],control.armed?control.wheel[i].output:0)){
   control.faultStop("ENCODER_FEEDBACK",now,dt,i);break;
  }
  control.tick(now,dt);
  if(!ready)control.faultStop(outputFault?outputFault:"PWM_FAULT",now,dt);
  for(int i=0;i<3;i++)motorWrite(i,int(lroundf(control.wheel[i].output)));
  for(int i=0;i<3;i++)if(control.wheel[i].output==0){
   if(gpio_get_level(gpio_num_t(pins[i].fi))||gpio_get_level(gpio_num_t(pins[i].bi))){
    ready=false;outputFault="STOP_PIN_HIGH";control.faultStop(outputFault,now,dt,i);low();break;
   }
  }
  lastControl=now;
  if(traceEnabled){
   auto& sample=controlSamples[sampleHead];sample.ms=now;sample.commandMs=control.last;
   for(int i=0;i<3;i++){
    sample.speed[i]=control.wheel[i].measured;sample.reference[i]=control.wheel[i].ramp;
    sample.pwm[i]=pwmAttached[2*i]?int(ledcRead(pins[i].fi)):0;
    sample.pwm[i]-=pwmAttached[2*i+1]?int(ledcRead(pins[i].bi)):0;
   }
   sampleHead=(sampleHead+1)%8;if(sampleCount<8)sampleCount++;
  }
  xSemaphoreGive(controlMutex);
 }
}
static String status(){
 // Network calls may yield. Complete them before taking the control snapshot,
 // then timestamp every control/odometry field against that one snapshot.
 JsonDocument d;d["type"]="status";d["firmware"]="wbb-c6-drive-1";d["session"]=session;
 d["ip"]=WiFi.status()==WL_CONNECTED?WiFi.localIP().toString():WiFi.softAPIP().toString();
 networkStatus(d["network"].to<JsonObject>());
 d["clients"]=WiFi.softAPgetStationNum();
 drive::Control s;bool cal,pwmReady,robustCal,holdCal;uint32_t tick,capturedMs,duties[3][2];int levels[3][2];
 ControlSample samples[8];unsigned nSamples=0;
 xSemaphoreTake(controlMutex,portMAX_DELAY);s=control;robustCal=robustValidated;holdCal=holdValidated;cal=s.holdMode?holdCal:s.robustMode?robustCal:calibrated;pwmReady=ready;tick=lastControl;
 for(int i=0;i<3;i++){
  duties[i][0]=pwmAttached[2*i]?ledcRead(pins[i].fi):0;duties[i][1]=pwmAttached[2*i+1]?ledcRead(pins[i].bi):0;
  levels[i][0]=gpio_get_level(gpio_num_t(pins[i].fi));levels[i][1]=gpio_get_level(gpio_num_t(pins[i].bi));
 }
 if(traceEnabled){nSamples=sampleCount;for(unsigned n=0;n<nSamples;n++)samples[n]=controlSamples[(sampleHead+8-sampleCount+n)%8];}
 capturedMs=millis();
 xSemaphoreGive(controlMutex);
 uint32_t c[3],b[3],e[3],ae[3],be[3];snapshot(c,b,e,ae,be);
 d["ms"]=capturedMs;d["armed"]=s.armed;d["lease"]=s.lease;d["reason"]=s.reason;
 d["telemetry_revision"]=2;
 d["control_run"]=s.run;d["diagnostics_revision"]=1;
 d["direction_guard_revision"]=2; // actual-duty qualification + 250ms continuous dwell in every mode
 d["usb_wifi_revision"]=1;
 d["last_fault"]=nullptr;
 if(s.lastFault.code){
  const auto& f=s.lastFault;auto fault=d["last_fault"].to<JsonObject>();
  fault["code"]=f.code;fault["motor"]=f.motor;fault["ms"]=f.ms;fault["run"]=f.run;
  fault["mode"]=f.mode;fault["dt_s"]=f.dt;
  auto targets=fault["target_cps"].to<JsonArray>();auto refs=fault["reference_cps"].to<JsonArray>();
  auto speeds=fault["speed_cps"].to<JsonArray>();auto duties=fault["applied_pwm"].to<JsonArray>();
  for(int i=0;i<3;i++){targets.add(f.target[i]);refs.add(f.reference[i]);speeds.add(f.speed[i]);duties.add(f.applied[i]);}
  auto pose=fault["encoder_pose"].to<JsonArray>();pose.add(f.pose.x);pose.add(f.pose.y);pose.add(f.pose.yaw);
  fault["error_m"]=f.positionError;fault["heading_error_rad"]=f.headingError;
 }
 d["ready"]=pwmReady;d["control_age_ms"]=uint32_t(capturedMs-tick);d["calibrated"]=cal;d["ap"]=apName;
 d["lcd"]=lcdReady;d["voltage"]=nullptr;
 d["max_wheel_cps"]=drive::maxWheelSpeed;d["speed_ui_revision"]=2;
 d["floor_trial"]=s.floorTrial;
 d["manual_control_available"]=cal||s.floorTrial;
 d["max_body_speed_mps"]=s.floorTrial?drive::Control::floorSpeed:s.holdMode?drive::EncoderHold::maxSpeed:.4f;
 d["max_yaw_rps"]=s.floorTrial?drive::Control::floorYaw:s.holdMode?drive::EncoderHold::maxYaw:1.2f;
 d["integral_limit_pwm"]=drive::integralLimit;
 d["controller"]=s.mode();
 d["controller_revision"]=robustRevision;d["robust_validated"]=robustCal;
 JsonObject heading=d["heading_assist"].to<JsonObject>();
 heading["enabled"]=s.headingAssistEnabled;heading["active"]=s.headingAssist.active;
 heading["goal_rad"]=s.headingAssist.goal;heading["error_rad"]=s.headingAssist.error;
 heading["yaw_rate_rps"]=s.headingAssist.yawRate;heading["wheel_correction_cps"]=s.headingAssist.correctionCps;
 heading["braking"]=s.coordinatedBraking;heading["brake_seconds"]=s.brakeTime;
 d["hold_revision"]=holdRevision;d["hold_validated"]=holdCal;
 d["controller_saved_for_build"]=controllerSaved;
 d["body_feedback"]=false;d["wall_hold"]=false;
 d["validation_scope"]=s.floorTrial?"limited_floor_trial_not_full_validation":"suspended_wheels_only";d["continuous_hold_torque"]=false;
 auto od=d["odometry"].to<JsonObject>();od["source"]="wheel_encoders";
 od["valid"]=s.odometry.valid;od["age_ms"]=uint32_t(capturedMs-s.odometry.stamp);
 od["x_m"]=s.odometry.pose.x;od["y_m"]=s.odometry.pose.y;od["yaw_rad"]=s.odometry.pose.yaw;
 od["goal_x_m"]=s.odometry.goal.x;od["goal_y_m"]=s.odometry.goal.y;od["goal_yaw_rad"]=s.odometry.goal.yaw;
 od["error_m"]=s.odometry.error;od["heading_error_rad"]=s.odometry.headingError;
 od["holding"]=s.armed&&s.holdMode&&s.odometry.active&&s.odometry.holding;
 od["within_hold_band"]=s.odometry.quiet;
 d["demand_scale"]=s.robustMode?s.robust.scale:1;
 d["output_limited"]=s.robustMode&&s.robust.limited;
 if(s.robustMode)d["integral_limit_pwm"]=120;
 if(nSamples){
  // [control_ms, accepted_command_ms, speed x3, reference x3, applied PWM x3]
  auto rows=d["control_samples"].to<JsonArray>();
  for(unsigned n=0;n<nSamples;n++){
   const auto& sample=samples[n];auto row=rows.add<JsonArray>();
   row.add(sample.ms);row.add(sample.commandMs);
   for(float x:sample.speed)row.add(x);
   for(float x:sample.reference)row.add(x);
   for(int x:sample.pwm)row.add(x);
  }
 }
 auto a=d["motors"].to<JsonArray>();
 for(int i=0;i<3;i++){
  auto w=a.add<JsonObject>();w["physical"]=i+1;w["count"]=int32_t(c[i]);w["bad"]=b[i];w["edges"]=e[i];
  w["a_edges"]=ae[i];w["b_edges"]=be[i];
  w["a_level"]=digitalRead(pins[i].a);w["b_level"]=digitalRead(pins[i].b);
  w["target"]=s.wheel[i].target;w["speed"]=s.wheel[i].measured;w["pwm"]=s.wheel[i].output;
  w["integral"]=s.wheel[i].integral;
  w["reference"]=s.wheel[i].ramp;
  w["compensation"]=s.robustMode?s.robust.compensation[i]:0;
  // LEDC register readback, not a claim about measured voltage at the driver.
  w["fi_duty"]=duties[i][0];w["bi_duty"]=duties[i][1];
  w["fi_level"]=levels[i][0];w["bi_level"]=levels[i][1];
  auto gs=w["gains"].to<JsonArray>();
  for(auto g:s.wheel[i].gains){auto z=gs.add<JsonArray>();z.add(g.kp);z.add(g.ki);z.add(g.kv);z.add(g.ks);}
 }
 d["telemetry_build_ms"]=uint32_t(millis()-capturedMs);
 String out;serializeJson(d,out);return out;
}
static bool number(JsonVariantConst v,float& f){
 if(!v.is<float>()&&!v.is<int>())return false;f=v.as<float>();return isfinite(f);
}
static String command(const String& text,bool usb){
 JsonDocument d,r;r["type"]="ack";r["ok"]=false;
 auto err=deserializeJson(d,text);r["id"]=d["id"]|0;
 const char* op=d["op"]|"";const char* error="BAD_COMMAND";
 if(err)return "{\"type\":\"ack\",\"ok\":false,\"error\":\"BAD_JSON\"}";
 if(!strcmp(op,"wifi_config")){
  const char* wifiError=usb?saveNetworkConfiguration(d.as<JsonVariantConst>(),true):"USB_ONLY";
  // Never serialize the request or echo credentials, including on failure.
  r["ok"]=wifiError==nullptr;r["error"]=wifiError?wifiError:"";r["session"]=session;
  r["saved"]=wifiError==nullptr;
  String out;serializeJson(r,out);return out;
 }
 xSemaphoreTake(controlMutex,portMAX_DELAY);
 if(!strcmp(op,"stop")){control.stop("USER_STOP");error=nullptr;}
 else if(!strcmp(op,"hello")||!strcmp(op,"status")){error=nullptr;}
 else if(usb&&!strcmp(op,"info")){r["ap"]=apName;r["password"]=apPass;r["url"]="http://192.168.4.1";error=nullptr;}
 else if(usb&&!strcmp(op,"wifi_scan")){
  float speed[3];for(int i=0;i<3;i++)speed[i]=control.wheel[i].measured;
  if(networkBusy()||WiFi.status()==WL_CONNECTED||!drive::canConfigureNetwork(control.armed,ready,uint32_t(millis()-lastControl),speed))error="STOP_BEFORE_WIFI_SCAN";
  else {networkScanPending=true;networkScanMatches=0;networkScanRssi=-127;networkScanChannel=networkScanAuth=0;error=nullptr;}
 }
 else if(!strcmp(op,"arm")){
  if(!ready)error=outputFault?outputFault:"PWM_FAULT";
  else if(networkBusy())error="WIFI_CONNECTING";
  else if(control.armed)error="ALREADY_ARMED";
  else if(!usb&&control.headingAssistEnabled&&!control.floorTrial)error="HEADING_ASSIST_USB_TRIAL";
  else if((control.holdMode||control.floorTrial)&&!control.odometry.valid)error="ODOMETRY_INVALID";
  else if(!usb&&!control.floorTrial&&!(control.holdMode?holdValidated:control.robustMode?robustValidated:calibrated))error="NEEDS_CALIBRATION";
  else {control.arm(millis(),esp_random()|1);r["lease"]=control.lease;error=nullptr;}
 }else if(!strcmp(op,"drive")){
  float values[3];auto a=d["values"].as<JsonArrayConst>();bool valid=a.size()==3;
  for(int i=0;i<3&&valid;i++)valid=number(a[i],values[i]);
  if(!usb&&!holdValidated)error="NEEDS_HOLD_VALIDATION";
  else if(valid&&d["lease"].is<uint32_t>()&&d["seq"].is<uint32_t>()&&
    control.acceptBody(millis(),d["lease"],d["seq"],values))error=nullptr;
  else error="EXPIRED_OR_INVALID";
 }else if(!strcmp(op,"pwm")||!strcmp(op,"wheels")){
  float values[3];auto a=d["values"].as<JsonArrayConst>();bool valid=a.size()==3;
  for(int i=0;i<3&&valid;i++)valid=number(a[i],values[i]);
  bool pwm=!strcmp(op,"pwm");
  if(pwm&&!usb)error="USB_BENCH_ONLY";
  else if(!pwm&&!control.robustMode&&(gainMask!=63||(!usb&&!calibrated)))error="NEEDS_CALIBRATION";
  else if(!pwm&&control.robustMode&&!usb&&!robustValidated&&!control.floorTrial)error="NEEDS_CALIBRATION";
  else if(valid&&d["lease"].is<uint32_t>()&&d["seq"].is<uint32_t>()&&
    control.accept(millis(),d["lease"],d["seq"],values,pwm))error=nullptr;
  else error="EXPIRED_OR_INVALID";
 }else if(!strcmp(op,"controller")&&!control.armed){
  const char* mode=d["mode"]|"";
  if(!strcmp(mode,"floor_trial")){
   control.selectFloorTrial();error=nullptr;r["controller"]=control.mode();
  }else if(!strcmp(mode,"encoder_hold")&&(usb||holdValidated)){
   control.selectHold();error=nullptr;r["controller"]=control.mode();
  }else if((!strcmp(mode,"legacy")&&(usb||calibrated))||(!strcmp(mode,"robust_wheels")&&(usb||robustValidated))){
   control.selectRobust(!strcmp(mode,"robust_wheels"));error=nullptr;
   r["controller"]=control.mode();
  }
 }else if(!strcmp(op,"zero_odometry")&&!control.armed){
  bool still=true;for(auto& w:control.wheel)still=still&&fabsf(w.measured)<30;
  if(still){control.odometry.pose={};control.odometry.resetGoal();error=nullptr;}
  else error="WHEELS_MOVING";
 }else if(!strcmp(op,"trace")&&!control.armed&&d["enabled"].is<bool>()){
  traceEnabled=d["enabled"].as<bool>();sampleHead=sampleCount=0;error=nullptr;
 }else if(usb&&!strcmp(op,"heading_assist")&&!control.armed&&!control.floorTrial&&d["enabled"].is<bool>()){
  control.headingAssistEnabled=d["enabled"].as<bool>();control.headingAssist.reset();
  robustValidated=holdValidated=controllerSaved=false;error=nullptr;
 }else if(usb&&!strcmp(op,"validate_robust")&&!control.armed&&control.robustMode&&!control.headingAssistEnabled){
  // Same trust boundary as measured legacy gain validation: USB host only.
  // Validation is deliberately not persisted across reboot or a new firmware.
  robustValidated=true;error=nullptr;
 }else if(usb&&!strcmp(op,"validate_hold")&&!control.armed&&control.holdMode&&robustValidated){
  holdValidated=true;error=nullptr;
 }else if(usb&&!strcmp(op,"save_controller")&&!control.armed&&control.robustMode&&
          !control.headingAssistEnabled&&robustValidated&&(!control.holdMode||holdValidated)){
  SavedController saved{};memcpy(saved.sha,esp_app_get_description()->app_elf_sha256,32);
  saved.mode=control.holdMode?2:1;saved.robust=robustValidated;saved.hold=holdValidated;saved.version=1;
  if(prefs.putBytes("controller",&saved,sizeof(saved))==sizeof(saved)){controllerSaved=true;error=nullptr;}
  else error="SAVE_FAILED";
 }else if(usb&&!strcmp(op,"gains")&&!control.armed){
  int i=d["motor"]|-1,dir=d["dir"]|-1;float g[4];auto a=d["values"].as<JsonArrayConst>();bool ok=a.size()==4;
  for(int k=0;k<4&&ok;k++)ok=number(a[k],g[k]);
  if(ok&&i>=0&&i<3&&dir>=0&&dir<2&&drive::valid({g[0],g[1],g[2],g[3]})){
   control.wheel[i].gains[dir]={g[0],g[1],g[2],g[3]};gainMask|=1U<<(2*i+dir);calibrated=false;error=nullptr;
  }
 }else if(usb&&!strcmp(op,"save")&&!control.armed&&gainMask==63){
  drive::Gains g[6];for(int i=0;i<3;i++)for(int k=0;k<2;k++)g[2*i+k]=control.wheel[i].gains[k];
  error=prefs.putBytes("gains",g,sizeof(g))==sizeof(g)?nullptr:"SAVE_FAILED";
  if(!error)calibrated=true;
 }
 // Any rejected control frame revokes the lease, preventing stale retries from moving.
 if(error)control.reject(error);
 r["ok"]=error==nullptr;r["error"]=error?error:"";r["session"]=session;
 r["stop_reason"]=control.reason;
 if(!error)r["command_ms"]=control.last;
 xSemaphoreGive(controlMutex);
 String out;serializeJson(r,out);return out;
}
static void drawRow(unsigned index,const String& text,uint16_t color){
 static String cached[8];static uint16_t colors[8]={};
 if(cached[index]==text&&colors[index]==color)return;
 cached[index]=text;colors[index]=color;
 // Fixed 24-character cells, opaque built-in ASCII font. No bitmap upload,
 // no clear-then-redraw, and no leftover characters when a value shrinks.
 char padded[25];snprintf(padded,sizeof(padded),"%-24.24s",text.c_str());
 lcd.setTextWrap(false);lcd.setTextSize(2);lcd.setTextColor(color,0x0841);
 lcd.setCursor(12,6+20*index);lcd.print(padded);
}
static void draw(){
 drive::Control s;bool cal,pwmReady;
 xSemaphoreTake(controlMutex,portMAX_DELAY);s=control;cal=s.holdMode?holdValidated:s.robustMode?robustValidated:calibrated;pwmReady=ready;xSemaphoreGive(controlMutex);
 drawRow(0,s.robustMode?String(s.floorTrial?"FLOOR ":s.holdMode?"ENC ":"RW ")+s.reason:((!pwmReady||cal)?String(s.reason):String("WBB: PID NOT TUNED")),!pwmReady?ST77XX_RED:(s.armed?ST77XX_GREEN:ST77XX_CYAN));
 if(WiFi.status()==WL_CONNECTED){
  drawRow(1,String("IP ")+WiFi.localIP().toString(),ST77XX_GREEN);
  drawRow(2,"OPEN IP IN BROWSER",ST77XX_WHITE);
  drawRow(3,String(networkHost)+".local",ST77XX_CYAN);
 }else{
  drawRow(1,String("AP ")+apName,ST77XX_WHITE);
  drawRow(2,String("PW ")+apPass,ST77XX_WHITE);
  drawRow(3,"http://192.168.4.1",ST77XX_CYAN);
 }
 drawRow(4,"M   SET   CPS   PWM",0x9cf3);
 char line[64];
 for(int i=0;i<3;i++){
  snprintf(line,sizeof(line),"%d %5.0f %5.0f %5.0f",i+1,s.wheel[i].target,s.wheel[i].measured,s.wheel[i].output);
  drawRow(5+i,line,ST77XX_WHITE);
 }
}
static void displayTask(void*){for(;;){draw();vTaskDelay(pdMS_TO_TICKS(200));}}
void setup(){
 for(auto p:pins){gpio_reset_pin(gpio_num_t(p.fi));gpio_reset_pin(gpio_num_t(p.bi));pinMode(p.a,INPUT_PULLUP);pinMode(p.b,INPUT_PULLUP);}
 ready=true;low();
 pinMode(9,INPUT_PULLUP);pinMode(22,OUTPUT);digitalWrite(22,LOW);
 controlMutex=xSemaphoreCreateMutex();if(!controlMutex)return;
 // Fault snapshots and the optional eight-sample trace must fit in one status frame.
 Serial.setTxBufferSize(8192);Serial.setRxBufferSize(1024);Serial.begin(115200);Serial.setTxTimeoutMs(2);
 session=esp_random();
 for(int i=0;i<3;i++){
  uint32_t in=REG_READ(GPIO_IN_REG);previous[i]=(((in>>pins[i].a)&1)<<1)|((in>>pins[i].b)&1);
  attachInterruptArg(pins[i].a,encoder,reinterpret_cast<void*>(i),CHANGE);attachInterruptArg(pins[i].b,encoder,reinterpret_cast<void*>(i),CHANGE);
 }
 prefs.begin("c6-drive",false);
 drive::Gains g[6];
 if(prefs.getBytesLength("gains")==sizeof(g)&&prefs.getBytes("gains",g,sizeof(g))==sizeof(g)){
  calibrated=true;for(auto z:g)calibrated=calibrated&&drive::valid(z);
  if(calibrated){gainMask=63;for(int i=0;i<3;i++)for(int k=0;k<2;k++)control.wheel[i].gains[k]=g[2*i+k];}
 }
 SavedController saved{};
 if(prefs.getBytesLength("controller")==sizeof(saved)&&prefs.getBytes("controller",&saved,sizeof(saved))==sizeof(saved)&&
    saved.version==1&&saved.robust==1&&saved.hold<=1&&saved.mode>=1&&saved.mode<=2&&
    (saved.mode!=2||saved.hold==1)&&!memcmp(saved.sha,esp_app_get_description()->app_elf_sha256,32)){
  robustValidated=true;holdValidated=saved.hold;controllerSaved=true;
  if(saved.mode==2)control.selectHold();else control.selectRobust(true);
 }
 // Manual trial is the usable default, always disarmed, never full validation.
 if(!controllerSaved)control.selectFloorTrial();
 snprintf(apName,sizeof(apName),"WBB-C6-%04X",unsigned(ESP.getEfuseMac()>>32)&65535);
 String pass=prefs.getString("ap-pass","");
 if(pass.length()!=12){snprintf(apPass,sizeof(apPass),"%08lX%04lX",(unsigned long)esp_random(),(unsigned long)(esp_random()&65535));prefs.putString("ap-pass",apPass);}else strlcpy(apPass,pass.c_str(),sizeof(apPass));
 snprintf(webToken,sizeof(webToken),"%08lX%08lX",(unsigned long)esp_random(),(unsigned long)esp_random());
 if(xTaskCreate(controlTask,"wheel-control",4096,nullptr,4,nullptr)!=pdPASS){ready=false;low();return;}
 SPI.begin(7,-1,6,14);lcd.init(172,320);lcd.setRotation(1);lcd.setSPISpeed(8000000);
 lcd.fillScreen(0x0841);
 // Independent sigma-delta peripheral: all six LEDC channels belong to motors.
 lcdReady=sigmaDeltaAttach(22,1000000)&&sigmaDeltaWrite(22,100);
 snprintf(networkHost,sizeof(networkHost),"wbb-c6-%04x",unsigned(ESP.getEfuseMac()>>32)&65535);
 WiFi.persistent(false);WiFi.setHostname(networkHost);
 WiFi.onEvent(observeNetwork);
 WiFi.mode(WIFI_AP_STA);WiFi.setAutoReconnect(false);
 apReady=WiFi.softAP(apName,apPass,6,0,1);WiFi.setSleep(false);
 if(prefs.getBytesLength("home-wifi")==sizeof(homeNetwork)&&
    prefs.getBytes("home-wifi",&homeNetwork,sizeof(homeNetwork))==sizeof(homeNetwork)&&drive::validNetwork(homeNetwork))
  networkPolicy.configure(millis(),true,homeNetwork.auth!=drive::NetworkCredentials::Personal);
 else homeNetwork={};
 mdnsReady=MDNS.begin(networkHost);if(mdnsReady)MDNS.addService("http","tcp",80);
 const char* headers[]={"X-Control-Token"};server.collectHeaders(headers,1);
 server.on("/",HTTP_GET,[]{if(!webAccess())return;String page=WEB_UI;page.replace("__TOKEN__",webToken);server.sendHeader("Cache-Control","no-store");server.send(200,"text/html; charset=utf-8",page);});
 server.on("/status",HTTP_GET,[]{if(!webAccess())return;server.sendHeader("Cache-Control","no-store");server.send(200,"application/json",status());});
 server.on("/network",HTTP_POST,configureNetwork);
 server.on("/cmd",HTTP_POST,[]{
  if(!webAccess())return;
  if(server.header("X-Control-Token")!=webToken){server.send(403,"application/json","{\"ok\":false}");return;}
  if(server.arg("plain").length()>512){server.send(413);return;}
  server.send(200,"application/json",command(server.arg("plain"),false));
 });
 server.begin();httpStarted=true;httpSocketGeneration=networkSnapshot().socketGeneration;
 xTaskCreate(displayTask,"display",4096,nullptr,1,nullptr);
 Serial.printf("\nWBB WiFi %s password %s URL http://192.168.4.1\n",apName,apPass);
}
void loop(){
 if(!controlMutex){delay(100);return;}
 static String input;static bool overflow=false;static uint32_t tele=0;
 for(int n=0;n<128&&Serial.available();n++){
  int c=Serial.read();
  if(c=='\n'){
   if(!overflow&&input.length())Serial.println(command(input,true));
   input="";overflow=false;
  }else if(c!='\r'){
   if(input.length()>=512||c<32||c>126){overflow=true;xSemaphoreTake(controlMutex,portMAX_DELAY);control.stop("RX_OVERFLOW");xSemaphoreGive(controlMutex);}
   else if(!overflow)input+=char(c);
  }
 }
 if(millis()-tele>=50){tele=millis();String s=status();if(Serial&&Serial.availableForWrite()>int(s.length()+1))Serial.println(s);}
 networkTick();recoverHttpServer();server.handleClient();delay(1);
}
