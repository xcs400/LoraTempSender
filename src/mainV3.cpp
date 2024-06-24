 #ifdef HELTEC_V3_YAOURT

#include <SPI.h>

#include <Wire.h>
#include <stdio.h>
 
#include <RadioLib.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#include "SSD1306.h"
#include "rom/ets_sys.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"


#include <esp_adc_cal.h>
#include <driver/adc.h>
#include <WiFi.h>

#define V3 1



#define  ALLWAYSCREEN          1       // enable screen and serial output  ;  drawbattery more heavily   use for test only
#define  USDELAY                120000000  //120 s

#define MAXBATT                 4200    // The default Lipo is 4200mv when the battery is fully charged.
#define LIGHT_SLEEP_VOLTAGE     3750    // Point where start light sleep
#define MINBATT                 3200    // The default Lipo is 3200mv when the battery is empty...this WILL be low on the 3.3v rail specs!!!

#define DEFAULT_VREF            1100    // Default VREF use if no e-fuse calibration
#define VBATT_SAMPLE            500     // Battery sample rate in ms
#define VBATT_SMOOTH            5      // Number of averages in sample
#define ADC_READ_STABILIZE      5       // in ms (delay from GPIO control and ADC connections times)
#define LO_BATT_SLEEP_TIME      10*60*1000*1000     // How long when low batt to stay in sleep (us)
#define HELTEC_V2_1             1       // Set this to switch between GPIO13(V2.0) and GPIO37(V2.1) for VBatt ADC.

#ifdef  V3
   #define VBATT_GPIO                37      //  390/100DIV
   #define VBATT_VEXTGPIO                36      // 3
  #define VOLTAGE_DIVIDER         3.90    // Lora has 390k/100k voltage divider 
  #define PIN_VBAT   ADC1_CHANNEL_1
#endif

#define __DEBUG                 0       // DEBUG Serial output


#define BAND 868E6

// NSS pin:   10
// DIO0 pin:  2
// RESET pin: 9
// DIO1 pin:  3
//SX1278 radio = new Module(10, 2, 9, 3);
SX1262 radio = new Module(SS, DIO0, RST_LoRa, BUSY_LoRa);


RTC_DATA_ATTR int bootCount = 0;  // deep sleep saved 

// GPIO where the DS18B20 is connected to
#ifdef V3
const int oneWireBus =34;     
#endif
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

/*
 * @param _address I2C Display address
     * @param _sda I2C SDA pin number, default to -1 to skip Wire begin call
     * @param _scl I2C SCL pin number, default to -1 (only SDA = -1 is considered to skip Wire begin call)
     * @param g display geometry dafault to generic GEOMETRY_128_64, see OLEDDISPLAY_GEOMETRY definition for other options
     * @param _i2cBus on ESP32 with 2 I2C HW buses, I2C_ONE for 1st Bus, I2C_TWO fot 2nd bus, default I2C_ONE
     * @param _frequency for Frequency by default Let's use ~700khz if ESP8266 is in 160Mhz mode, this will be limited to ~400khz if the ESP8266 in 80Mhz mode
   */

#ifdef V3
SSD1306 display(0x3c, SDA_OLED, SCL_OLED );
#endif

uint16_t Sample();
void drawBattery(uint16_t, bool = false);

esp_adc_cal_characteristics_t *adc_chars;
 String msg;

uint16_t ReadVBatt() {
  int reading = 666;

  digitalWrite(VBATT_GPIO, LOW);              // ESP32 Lora v2.1 reads on GPIO37 when GPIO21 is low
  delay(ADC_READ_STABILIZE);                  // let GPIO stabilize
  #if (defined(HELTEC_V2_1))
  pinMode(PIN_VBAT, OPEN_DRAIN);        // ADC GPIO37
  reading = adc1_get_raw(PIN_VBAT);
  pinMode(PIN_VBAT, INPUT);             // Disconnect ADC before GPIO goes back high so we protect ADC from direct connect to VBATT (i.e. no divider)
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

#ifdef V3
  #define OLEDRST RST_OLED 
#endif

  pinMode(VBATT_GPIO, OUTPUT);  // v33 sur ADCIN
 digitalWrite(VBATT_GPIO, LOW);              // ESP32 Lora v2.1 reads on GPIO37 when GPIO21 is low

  pinMode(VBATT_VEXTGPIO, OUTPUT);  // reset OLED
 digitalWrite(VBATT_VEXTGPIO, LOW);              // ESP32 Lora v2.1 reads on GPIO37 when GPIO21 is low
 

  pinMode(OLEDRST, OUTPUT);  // reset OLED
  pinMode(LED, OUTPUT);
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

 Serial.begin(115200);
    while (!Serial)
      ; 

  //SPI.begin(SCK, MISO, MOSI, SS);
/*
Parameters:
freq – Carrier frequency in MHz. Defaults to 434.0 MHz.
bw – LoRa bandwidth in kHz. Defaults to 125.0 kHz.
sf – LoRa spreading factor. Defaults to 9.
cr – LoRa coding rate denominator. Defaults to 7 (coding rate 4/7).
syncWord – 1-byte LoRa sync word. Defaults to RADIOLIB_SX126X_SYNC_WORD_PRIVATE (0x12).
power – Output power in dBm. Defaults to 10 dBm.
preambleLength – LoRa preamble length in symbols. Defaults to 8 symbols.
tcxoVoltage – TCXO reference voltage to be set on DIO3. Defaults to 1.6 V. If you are seeing -706/-707 error codes, it likely means you are using non-0 value for module with XTAL. To use XTAL, either set this value to 0, or set SX126x::XTAL to true.
useRegulatorLDO – Whether to use only LDO regulator (true) or DC-DC regulator (false). Defaults to false.
*/
//                       freq,  bw,   sf,cr,syncWord,power,preambleLength,tcxoVoltage,useRegulatorLDO
    int state = radio.begin(868.0, 125.0, 7, 5, 0x12,    14,       8);

   Serial.printf("\r\n  init Radio Lora . state %u",state);



  String NodeId = "Yaourt3" ;  // WiFi.macAddress();
  sensors.requestTemperatures(); 
  float temperatureC = sensors.getTempCByIndex(0);

   msg = "{\"model\":\"ESP32TEMP\",\"id\":\"" + NodeId + "\",\"TempCelsius\":" + String(temperatureC) +  ",\"Elapsed\":" + String((int)round((double)bootCount*USDELAY/1000000)) + ",\"Vbatt\":" + String(voltage) + ",\"Charge%\":" + String(bars) + "}";
 

  if (bootCount==0  || ALLWAYSCREEN)
    {
    digitalWrite(OLEDRST, LOW); // set GPIO16 low to reset OLED
    delay(200);
    digitalWrite(OLEDRST, HIGH); // while OLED is running, must set GPIO16 in high
  
   
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
  

 //   Serial.println(String(msg));
    
    display.drawString(40, 0, String(NodeId));
    display.drawString(0, 15, "tempc: " + String(temperatureC) + " C");
    display.display();
    delay(60000); // wait for  2  second

    digitalWrite(VBATT_GPIO, HIGH); //por etteindre l ecran 
    pinMode(VBATT_GPIO, INPUT);
    }

 

  digitalWrite(LED, HIGH); // turn the LED on (HIGH is the voltage level)
  delay(2); // wait for a  ms second
  digitalWrite(LED, LOW); // turn the LED off by making the voltage LOW


  bootCount++;

// essai mais toujour conso 2.7mA
//    WiFi.mode(WIFI_OFF);    // Switch WiFi off
//   pinMode(16, INPUT);
//  pinMode(25, INPUT);
 //   pinMode(21, INPUT);

// adc_power_off();



  unsigned long CurrentTime = millis();
 unsigned long ElapsedTime = CurrentTime - StartTime;

 //esp_sleep_enable_timer_wakeup(USDELAY- (ElapsedTime *1000) );  
 //esp_deep_sleep_start();  
}

void loop(void){

  digitalWrite(LED, HIGH); // turn the LED on (HIGH is the voltage level)
  delay(100); // wait for a  ms second
  digitalWrite(LED, LOW); // turn the LED off by making the voltage LOW



    display.drawString(0, 0, "Module:");
    display.drawString(0, 30, "voltage: " + String(voltage) + "mv");
  

int transmissionState = RADIOLIB_ERR_NONE;
  transmissionState = radio.startTransmit(msg);

    Serial.println(String(msg));
     Serial.printf(" state= %u",transmissionState);
   delay(5000); // wait for a  ms second
  
}


#endif