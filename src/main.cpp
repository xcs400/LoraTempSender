#include <LoRa.h>
#include <SPI.h>

#include <Wire.h>
#include <stdio.h>
 
#include <OneWire.h>
#include <DallasTemperature.h>

#include "SSD1306.h"
#include "rom/ets_sys.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"


#include <esp_adc_cal.h>
#include <driver/adc.h>
//#include "heltec.h"
#include <WiFi.h>


#define  ALLWAYSCREEN           0       // enable screen and serial output  ;  drawbattery more heavily   use for test only
#define  USDELAY                120000000  //120 s

#define MAXBATT                 4200    // The default Lipo is 4200mv when the battery is fully charged.
#define LIGHT_SLEEP_VOLTAGE     3750    // Point where start light sleep
#define MINBATT                 3200    // The default Lipo is 3200mv when the battery is empty...this WILL be low on the 3.3v rail specs!!!

#define VOLTAGE_DIVIDER         3.20    // Lora has 220k/100k voltage divider so need to reverse that reduction via (220k+100k)/100k on vbat GPIO37 or ADC1_1 (early revs were GPIO13 or ADC2_4 but do NOT use with WiFi.begin())
#define DEFAULT_VREF            1100    // Default VREF use if no e-fuse calibration
#define VBATT_SAMPLE            500     // Battery sample rate in ms
#define VBATT_SMOOTH            5      // Number of averages in sample
#define ADC_READ_STABILIZE      5       // in ms (delay from GPIO control and ADC connections times)
#define LO_BATT_SLEEP_TIME      10*60*1000*1000     // How long when low batt to stay in sleep (us)
#define HELTEC_V2_1             1       // Set this to switch between GPIO13(V2.0) and GPIO37(V2.1) for VBatt ADC.
#define VBATT_GPIO              21      // Heltec GPIO to toggle VBatt read connection ... WARNING!!! This also connects VEXT to VCC=3.3v so be careful what is on header.  Also, take care NOT to have ADC read connection in OPEN DRAIN when GPIO goes HIGH
#define __DEBUG                 0       // DEBUG Serial output



// Heltec ESP32 wifi    v2.1

#define SCK  5 // GPIO5  -- SX1278's SCK
#define MISO 19 // GPIO19 -- SX1278's MISnO
#define MOSI 27 // GPIO27 -- SX1278's MOSI
#define SS   18 // GPIO18 -- SX1278's CS
#define RST  14 // GPIO14 -- SX1278's RESET
#define DI0  26 // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND 868E6


RTC_DATA_ATTR int bootCount = 0;  // deep sleep saved 

// GPIO where the DS18B20 is connected to
const int oneWireBus =23;     

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

SSD1306 display(0x3c, 4, 15);



uint16_t Sample();
void drawBattery(uint16_t, bool = false);

esp_adc_cal_characteristics_t *adc_chars;


uint16_t ReadVBatt() {
  int reading = 666;

  digitalWrite(VBATT_GPIO, LOW);              // ESP32 Lora v2.1 reads on GPIO37 when GPIO21 is low
  delay(ADC_READ_STABILIZE);                  // let GPIO stabilize
  #if (defined(HELTEC_V2_1))
  pinMode(ADC1_CHANNEL_1, OPEN_DRAIN);        // ADC GPIO37
  reading = adc1_get_raw(ADC1_CHANNEL_1);
  pinMode(ADC1_CHANNEL_1, INPUT);             // Disconnect ADC before GPIO goes back high so we protect ADC from direct connect to VBATT (i.e. no divider)
  #else
  //pinMode(ADC2_CHANNEL_4, OPEN_DRAIN);        // ADC GPIO13
  adc2_get_raw(ADC2_CHANNEL_4,ADC_WIDTH_BIT_12,&reading);
  //pinMode(ADC2_CHANNEL_4, INPUT);             // Disconnect ADC before GPIO goes back high so we protect ADC from direct connect to VBATT (i.e. no divider
  #endif

  uint16_t voltage = esp_adc_cal_raw_to_voltage(reading, adc_chars);  
  voltage*=VOLTAGE_DIVIDER;

  return voltage;
}


//  Use a buffer to average/sample ADC
uint16_t Sample() {
  static uint8_t i = 0;
  static uint16_t samp[VBATT_SMOOTH];
  static int32_t t = 0;
  static bool f = true;
  if(f){ for(uint8_t c=0;c<VBATT_SMOOTH;c++){ samp[c]=0; } f=false; }   // Initialize the sample array first time
  t -= samp[i];   // doing a rolling recording, so remove the old rolled around value out of total and get ready to put new one in.
  if (t<0) {t = 0;}

  // ADC read
  uint16_t voltage = ReadVBatt();

  samp[i]=voltage;
  #if defined(__DEBUG) && __DEBUG > 0
  Serial.printf("ADC Raw Reading[%d]: %d", i, voltage);
  #endif
  t += samp[i];

  if(++i >= VBATT_SMOOTH) {i=0;}
  uint16_t s = round(((float)t / (float)VBATT_SMOOTH));
  #if defined(__DEBUG) && __DEBUG > 0
  Serial.printf("   Smoothed of %d/%d = %d\n",t,VBATT_SMOOTH,s); 
  #endif

  return s;
}



void setup() {

  unsigned long StartTime = millis();

  pinMode(16, OUTPUT);  // reset OLED
  pinMode(25, OUTPUT);

                                           //pour allumer l ecran     
  pinMode(VBATT_GPIO,OUTPUT);
  digitalWrite(VBATT_GPIO, LOW);              // ESP32 Lora v2.1 reads on GPIO37 when GPIO21 is low
  //delay(ADC_READ_STABILIZE);                  // let GPIO stabilize
 
  // Characterize ADC at particular atten
  #if (defined(HELTEC_V2_1))
  adc_chars = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_6, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_1,ADC_ATTEN_DB_6);
  #else
  // Use this for older V2.0 with VBatt reading wired to GPIO13
  adc_chars = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTEN_DB_6, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
  adc2_config_channel_atten(ADC2_CHANNEL_4,ADC_ATTEN_DB_6);
  #endif

  // Prime the Sample register
  for (uint8_t i = 0;i < VBATT_SMOOTH;i++) {
    Sample();
  }

  uint16_t voltage =0; // ; Sample();
  voltage = Sample();
  double pct = map(voltage,MINBATT,MAXBATT,0,100);
  uint8_t bars = round(pct );

  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DI0);
  if (!LoRa.begin(BAND)) {
  //  Serial.println("Starting LoRa failed!");
  //  while (1)
   //   ;
    }


  String NodeId = "Yaourt1" ;  // WiFi.macAddress();
  sensors.requestTemperatures(); 
  float temperatureC = sensors.getTempCByIndex(0);

  String msg = "{\"model\":\"ESP32TEMP\",\"id\":\"" + NodeId + "\",\"TempCelsius\":" + String(temperatureC) +  ",\"Elapsed\":" + String((int)round((double)bootCount*USDELAY/1000000)) + ",\"Vbatt\":" + String(voltage) + ",\"Charge%\":" + String(bars) + "}";
 

  if (bootCount==0  || ALLWAYSCREEN)
    {
    digitalWrite(16, LOW); // set GPIO16 low to reset OLED
    delay(200);
    digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high
  
    Serial.begin(115200);
    while (!Serial)
      ; 
    Serial.println();
    Serial.println("LoRa Sender Test");
  
    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);

    delay(1500);

    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);

    display.drawString(0, 0, "Module:");
    display.drawString(0, 30, "voltage: " + String(voltage) + "mv");
  

    Serial.println(String(msg));
    
    display.drawString(40, 0, String(NodeId));
    display.drawString(0, 15, "tempc: " + String(temperatureC) + " C");
    display.display();
    delay(2000); // wait for  2  second

    digitalWrite(21, HIGH); //por etteindre l ecran 
    pinMode(21, INPUT);
    }


 
 
  // send packet
  LoRa.beginPacket();
   // Send json string
  LoRa.print(msg);
  LoRa.endPacket();
  LoRa.end();

 

  digitalWrite(25, HIGH); // turn the LED on (HIGH is the voltage level)
  delay(2); // wait for a  ms second
  digitalWrite(25, LOW); // turn the LED off by making the voltage LOW


  bootCount++;

// essai mais toujour conso 2.7mA
//    WiFi.mode(WIFI_OFF);    // Switch WiFi off
//   pinMode(16, INPUT);
//  pinMode(25, INPUT);
 //   pinMode(21, INPUT);

 adc_power_off();

  unsigned long CurrentTime = millis();
 unsigned long ElapsedTime = CurrentTime - StartTime;

 esp_sleep_enable_timer_wakeup(USDELAY- (ElapsedTime *1000) );  
 esp_deep_sleep_start();  
}

void loop(void){}