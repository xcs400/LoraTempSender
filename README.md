# LoRa Node Example for Heltec ESP32 wifi Lora module  (V2 or V2.1  accordind to the setting)
The program reads the  temperature of a DS18B20 , packages the data into a JSON format, and sends it over LoRa then goes to deep sleep for 2mn.
the vBatt voltage is also reported.

The SSD1306 OLED display is only use once to save battery.
The serial output (at 115200 baud rate) will report the JSON formatted data being sent ( only once to save battery)

On my Heltec 2,7ma are drawn from the battery during the deep sleep mode  (and 80-100ma during the wake up and ADC read and lora sending) 

OPENMQTTgateway is used on anoter modul to send the json to a MQTTbroker

## Hardware :
* ESP32  wifi Lora module , include:
    * SX12XX LoRa module.       (adapt region  #define BAND 868E6)
    * SSD1306 OLED display.
* DS18B20      is attached on pin 23 on the ESP (onewire)

## Pin Configuration (used on Heltec Module):
* SCK  - GPIO5
* MISO - GPIO19
* MOSI - GPIO27
* SS   - GPIO18
* RST  - GPIO14
* DI0  - GPIO26

### Software Setup:

* Clone this repository.
* Open the provided node program with PlatformIO  (use of Ardwino env)
* Upload the program to your ESP32.


## Data Format:
The data is sent in the following JSON format:

```json
{"model":"ESP32TEMP","id":"B0:B2:1C:F8:73:94","TempCelsius":14.31,"Elapsed":0,"Vbatt":4112,"Charge%":91}
```
