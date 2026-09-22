#ifdef WBB_PINCHECK
#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <Adafruit_ST7789.h>
#include <driver/gpio.h>
#include <soc/gpio_reg.h>
static const uint8_t out[]={0,1,4,5,2,3};
static Adafruit_ST7789 screen(&SPI,14,15,21);
static void report(const char* phase){
 Serial.printf("\nPHASE %s\n",phase);
 for(int i=0;i<6;i++){
  gpio_input_enable(gpio_num_t(out[i]));
  uint32_t cfg=REG_READ(GPIO_FUNC0_OUT_SEL_CFG_REG+4*out[i]);
  Serial.printf("GPIO%u level=%d mux=0x%lx output_en=%d\n",out[i],gpio_get_level(gpio_num_t(out[i])),(unsigned long)cfg,!!(REG_READ(GPIO_ENABLE_REG)&(1U<<out[i])));
 }
}
void setup(){
 for(auto p:out){gpio_reset_pin(gpio_num_t(p));gpio_set_level(gpio_num_t(p),0);gpio_set_direction(gpio_num_t(p),GPIO_MODE_INPUT_OUTPUT);}
 Serial.setTxBufferSize(4096);Serial.begin(115200);delay(2000);report("GPIO_LOW");
 for(int i=0;i<6;i++){bool ok=ledcAttachChannel(out[i],20000,8,i);ledcWrite(out[i],0);Serial.printf("attach %u=%d\n",out[i],ok);}
 delay(10);report("LEDC_ZERO");
 SPI.begin(7,-1,6,14);report("SPI_BEGIN");
 screen.init(172,320);screen.setRotation(1);screen.fillScreen(0x0841);screen.setTextColor(ST77XX_WHITE);screen.setTextSize(2);screen.setCursor(12,12);screen.print("PIN CHECK / NO DRIVE");report("LCD_INIT");
 bool sd=sigmaDeltaAttach(22,1000000)&&sigmaDeltaWrite(22,100);Serial.printf("backlight=%d\n",sd);report("BACKLIGHT");
 WiFi.mode(WIFI_AP);WiFi.softAP("WBB-PINCHECK","diagnose-only");report("WIFI");
}
void loop(){
 for(auto p:out)ledcWrite(p,0);
 if(Serial.available()){
  char c=Serial.read();
  if(c=='p')report("LIVE");
  if(c=='x'){
   for(auto p:out){ledcDetach(p);gpio_reset_pin(gpio_num_t(p));gpio_set_level(gpio_num_t(p),0);gpio_set_direction(gpio_num_t(p),GPIO_MODE_INPUT_OUTPUT);}
   report("DETACHED_GPIO_LOW");
   while(true){for(auto p:out)gpio_set_level(gpio_num_t(p),0);delay(10);}
  }
 }
 delay(10);
}
#endif
