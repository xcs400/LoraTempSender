#ifdef CHAUFFEAU

#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SSD1306.h>
#include <esp_adc_cal.h>
#include <driver/adc.h>
#include <WiFi.h>
#include <ArduinoJson.h>

#include <ArduinoOTA.h>


#include <ESPAsyncWebServer.h>

#include <Preferences.h>



const char* ssid = "Freebox-7BF0EF";
const char* password = "kangourou";
String NodeId = "Chauffeau";



// Nom de l'espace de stockage dans la mémoire flash
Preferences preferences;


// Définition des constantes et variables pour la gestion de la pompe

float seuilON = 40;
float seuilOFF = 35.0;
unsigned long intervalpompe = 2 * 60 * 1000;  // 2 minutes on
unsigned long dutyoff = 1 * 60 * 1000;  // 1 minutes

   
float temperature1=0 ;
float temperature2 =0;
float temperatureRemote=0;

unsigned long    dureeTotalON=0;
unsigned long    dureeTotalOFF=0;
unsigned long    dureeTotalDuty=0;
unsigned long   DelaiLastLorain=0;

 unsigned long currentMillis =0;
 unsigned long lorain = 0;
 unsigned long dureEtat=0;
 unsigned long countChangeetat=0;

 String msg;
 String command;

int contlora=0;
// États de la state machine pour la pompe
enum StatePompe {
    POMPEOFF,
    POMPEOFF_DUTY,
    POMPEOFF_DELAI,
    POMPEON_DELAI,
    WAITREMOTE
};

StatePompe CurrentStatePompe = WAITREMOTE;

void saveParams() {
    preferences.begin("params", false); // Ouverture de l'espace de stockage "params", false indique que nous ne l'effaçons pas
    preferences.putFloat("seuilON", seuilON); // Sauvegarde de variables
    preferences.putFloat("seuilOFF", seuilOFF);
    preferences.putUInt("intervalpompe", intervalpompe);
    preferences.putUInt("dutyoff", dutyoff);
    preferences.end(); // Fin de l'accès aux préférences
}
void loadParams() {
    preferences.begin("params", true); // Ouverture de l'espace de stockage "params", true indique que nous le créons s'il n'existe pas
    seuilON = preferences.getFloat("seuilON", 25.0); // Restauration des variables avec des valeurs par défaut
    seuilOFF = preferences.getFloat("seuilOFF", 20.0);
    intervalpompe = preferences.getUInt("intervalpompe", 2 * 60 * 1000);
    dutyoff = preferences.getUInt("dutyoff", 1 * 60 * 1000);
    preferences.end(); // Fin de l'accès aux préférences
}


// Définition du type pour la fonction traductor
typedef String (*TraductorFunc)(void*);

// Structure pour représenter une variable
struct WebVar {
    const char* name;
    void* var;
    const char* type;
    const char* access;
    float scale;
    TraductorFunc traductor;
    const char* unit;
};

String getPompeStateName(void* state) ;

// Tableau de structures représentant les variables
WebVar varWeb[] = {
     {"currentMillis", &currentMillis, "int", "RO",1.0/1000,NULL,"sec"},
    {"seuilON", &seuilON, "float", "RW",1,NULL,"°C"},
    {"seuilOFF", &seuilOFF, "float", "RW",1,NULL,"°C"},
    {"intervalpompe", &intervalpompe, "int", "RW",1.0/60000,NULL,"min"},
    {"dutyoff", &dutyoff, "int", "RW",1.0/60000,NULL,"min"},
    {"CurrentStatePompe", &CurrentStatePompe, "int", "RO",1,getPompeStateName ,"STATE"},
    {"dureEtat", &dureEtat, "int", "RO",1.0/1000,NULL,"sec"}  ,
     {"countChangeetat", &countChangeetat, "int", "RO",1,NULL,"count"}  ,
    {"temperature In", &temperature1, "float", "RO",1,NULL,"°C"},
    {"temperature Out", &temperature2, "float", "RO",1,NULL,"°C"},
    {"temperature Echangeur", &temperatureRemote, "float", "RO",1,NULL,"°C"},
    {"LastReflesh Echangeur", &DelaiLastLorain, "int", "RO",1.0/1000,NULL,"sec"},
    {"PompeTotalON", &dureeTotalON, "int", "RO",1.0/1000,NULL,"sec"}  ,
    {"PompeTotalOFF", &dureeTotalOFF, "int", "RO",1.0/1000,NULL,"sec"}  ,
    {"PompeTotalDuty", &dureeTotalDuty, "int", "RO",1.0/1000,NULL,"sec"}  ,
    {"contlora", &contlora, "int", "RO",1.0,NULL,"cnt"}  ,


};



const int varCount = sizeof(varWeb) / sizeof(varWeb[0]);


AsyncWebServer  server(80);


String generateJSONString() {
    String json = "{";
    for (int i = 0; i < sizeof(varWeb) / sizeof(varWeb[0]); i++) {
        json += "\"" + String(varWeb[i].name) + "\":{\"value\":";
        
        if (varWeb[i].traductor) {
            json += "\"" + varWeb[i].traductor(varWeb[i].var) + "\"";
        } else if (strcmp(varWeb[i].type, "float") == 0) {
            json += String(*((float*)varWeb[i].var) * varWeb[i].scale);
        } else if (strcmp(varWeb[i].type, "int") == 0) {
            json += String(*((int*)varWeb[i].var) * varWeb[i].scale);
        }
        
        json += ",\"type\":\"" + String(varWeb[i].type) +  "\"}";

        if (i < sizeof(varWeb) / sizeof(varWeb[0]) - 1) {
            json += ",";
        }
    }
    json += "}";
    return json;
}


// Page HTML à servir
 // Fonction pour générer le tableau HTML
String generateHTMLTable() {
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Variables</title></head><body><h1>Variables</h1><table border='1'><tr><th>Nom</th><th>Valeur lue</th><th>Valeur à écrire</th><th>Unité</th><th>Action</th></tr>";
    
    for (int i = 0; i < sizeof(varWeb) / sizeof(varWeb[0]); i++) {
        html += "<tr><td>" + String(varWeb[i].name) + "</td><td id='value_" + String(varWeb[i].name) + "'>";
        
        if (varWeb[i].traductor) {
            html += varWeb[i].traductor(varWeb[i].var);
        } else if (strcmp(varWeb[i].type, "float") == 0) {
            html += String(*((float*)varWeb[i].var) * varWeb[i].scale);
        } else if (strcmp(varWeb[i].type, "int") == 0) {
            html += String(*((int*)varWeb[i].var) * varWeb[i].scale);
        }
        
        html += "</td>";
        
        if (strcmp(varWeb[i].access, "RW") == 0) {
            html += "<td><input type='text' id='" + String(varWeb[i].name) + "'></td><td>" + String(varWeb[i].unit) + "</td><td><button onclick=\"updateVar('" + String(varWeb[i].name) + "')\">Appliquer</button></td>";
        } else {
            html += "<td></td><td>" + String(varWeb[i].unit) + "</td><td></td>";
        }
        
        html += "</tr>";
    }
    
    // Ajout du bouton Reset ESP32
    html += "</table><button onclick=\"resetESP32()\">Reset ESP32</button>";
 //   html += "<div>lastSent" + msg+ "</div>" ;
 //   html += "<div>command:" + command+ "</div>" ;
  //   html += "<div>json:" + generateJSONString()+ "</div>" ;
   

     html += "<script>";
  

    // Définition de la fonction resetESP32()
    html += "function resetESP32() { fetch('/reset').then(response => { if (response.ok) { console.log('ESP32 restarted successfully.'); } else { console.error('Failed to restart ESP32.'); } }).catch(error => { console.error('Error restarting ESP32:', error); }); };";
    
    // Script pour mise à jour automatique des valeurs
    html += "function updateVar(name) { var value = document.getElementById(name).value; fetch('/update?name=' + name + '&value=' + value); };";
    html += "setInterval(refreshValues, 2000); function refreshValues() {";
    
    // Instructions JavaScript pour mettre à jour les valeurs
    for (int i = 0; i < sizeof(varWeb) / sizeof(varWeb[0]); i++) {
        html += "fetch('/value?name=" + String(varWeb[i].name) + "').then(response => response.text()).then(data => document.getElementById('value_" + String(varWeb[i].name) + "').textContent = data);";
    }
    
    html += "}</script></body></html>";
    
    return html;
}

/*
void handleRoot(AsyncWebServerRequest *request) {
    request->send(200, "text/html", "<h1>Hello kkkk from ESP32!</h1>");
}
*/



// Configuration des broches et des constantes
const int oneWireBus = 33;
const int VBATT_GPIO = 37;
const int VBATT_VEXTGPIO = 36;
const int OLEDRST = RST_OLED;  // Ajout de la définition de la broche OLEDRST
const int POMPE = 48;


#define LED_PIN LED
const long interval = 120000; // Intervalle de 120 secondes pour l'émission

#define DEFAULT_VREF 1100
#define VBATT_SMOOTH 5
#define MINBATT 3200
#define MAXBATT 4200
#define VOLTAGE_DIVIDER 3.90

RTC_DATA_ATTR int bootCount = 0; // Compteur de démarrage pour le mode deep sleep

SSD1306 display(0x3c, SDA_OLED, SCL_OLED);
SX1262 radio = new Module(SS, DIO0, RST_LoRa, BUSY_LoRa);
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);
esp_adc_cal_characteristics_t *adc_chars;

unsigned long previousMillis = 0;

void setFlag(void);



float processReceivedMessage(const String &receivedMsg)
 {
  
  contlora++;
    // Vérifier si la chaîne est un JSON valide
    StaticJsonDocument<1024> doc;
    DeserializationError error = deserializeJson(doc, receivedMsg);
    command=receivedMsg;
    if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());

        command="["+command+"]deserializeJsonfailed "; 
        return temperatureRemote;  // Retourner une valeur non définie si le JSON n'est pas valide
    }

 
// {  "message": "{\"id\":\"Chauffeau\",\"command\":\"noreset\" } "}
   const char* idm = doc["id"];
    if (idm &&  strcmp(idm, "Chauffeau") == 0) {
          contlora+=10;
        String scommand = doc["command"];
         command=scommand;
        if (command=="reset")
              ESP.restart(); // Redémarrage de l'ESP32 
        if (command=="incseuil")
                {
                 seuilON=seuilON+1;  
                 seuilOFF= seuilON-3;
                }
         if (command=="decseuil")
                {
                 seuilON=seuilON-1;  
                 seuilOFF= seuilON-3;
                }   

               if (command=="incintervalpompe")
                {
                 intervalpompe=intervalpompe+ 1*60*1000;  
                }   
                if (command=="decintervalpompe")
                {
                 intervalpompe=intervalpompe- 1*60*1000;  
                } 

                    if (command=="incdutyoff")
                {
                 dutyoff=dutyoff+ 1*60*1000;  
                }   
                if (command=="decdutyoff")
                {
                 dutyoff=dutyoff- 1*60*1000;  
                }               

 
       if (command=="getparametre")
              { 
            // Construire le message JSON  " + generateJSONString() + "
                 msg = "{\"model\":\"Chauffepara\",\"id\":\"Chauffeaupara\",\"param\":\"dddd\" }                 ";
                        // Démarrer la transmission
                 int transmissionState = radio.startTransmit(msg);
                    delay(500);  // Attendre suffisamment longtemps pour que la transmission ait lieu
                //  receivedFlag = 0;

             // Commencer à recevoir les messages
                int startReceiveState = radio.startReceive();
                if (startReceiveState != RADIOLIB_ERR_NONE) {
                    Serial.printf("Failed to start receive, code: %d\n", startReceiveState);
                }

              }

    }


    // Vérifier si l'ID est "Yaourt1"
    const char*  id   = doc["id"];
    if (id && strcmp(id, "Yaourt1") == 0) {
        // Récupérer la valeur de TempCelsius
        float tempCelsius = doc["TempCelsius"];
         lorain= millis(); 
        return tempCelsius;
    }

    return temperatureRemote;  // Retourner une valeur non définie si l'ID ne correspond pas
}
 String ipAd="0.0.0.0";
void setup() {

    pinMode(LED, OUTPUT);
    pinMode(VBATT_GPIO, OUTPUT);
    pinMode(VBATT_VEXTGPIO, OUTPUT);
    pinMode(OLEDRST, OUTPUT);
    pinMode(POMPE, OUTPUT);


    digitalWrite(VBATT_GPIO, LOW);
    digitalWrite(VBATT_VEXTGPIO, LOW);
    digitalWrite(OLEDRST, LOW);
    delay(200);
    digitalWrite(OLEDRST, HIGH);

    adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_6, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_6);

    loadParams() ;


    Serial.begin(115200);

    sensors.begin();
    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_16);
    display.clear();
    display.drawString(0, 0, "Initializing IP...");
    display.display();

    int state = radio.begin(868.0, 125.0, 7, 5, 0x12, 10, 8);
    radio.setDio1Action(setFlag);
    state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println(F("LoRa initialization success!"));
    } else {
        Serial.print(F("LoRa initialization failed, code "));
        Serial.println(state);
        while (true);
    }



 WiFi.begin(ssid, password);
  
   // Limiter le nombre de tentatives de connexion à 15
  for (int count = 0; count < 17; count++) {
    if (WiFi.status() == WL_CONNECTED) {
      break;
    }
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

   ipAd = WiFi.localIP().toString();
  Serial.println("Connected to WiFi");
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(ipAd);

    display.clear();
    display.drawString(0, 0, "IP "+ipAd);
    display.display();
    delay(4000);

   // Configuration du serveur web
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/html", generateHTMLTable());
    });

    server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("name") && request->hasParam("value")) {
            String name = request->getParam("name")->value();
            String value = request->getParam("value")->value();
            
            for (int i = 0; i < sizeof(varWeb) / sizeof(varWeb[0]); i++) {
                if (name == varWeb[i].name && strcmp(varWeb[i].access, "RW") == 0) {
                    if (strcmp(varWeb[i].type, "float") == 0) {
                        *((float*)varWeb[i].var) = value.toFloat() / varWeb[i].scale;
                    } else if (strcmp(varWeb[i].type, "int") == 0) {
                        *((int*)varWeb[i].var) = value.toInt() / varWeb[i].scale;
                    }
                }
            }
            saveParams();
        }
        request->send(200, "text/plain", "OK");
    });

server.on("/value", HTTP_GET, [](AsyncWebServerRequest *request){
    String paramName = request->getParam("name")->value();
    for (int i = 0; i < sizeof(varWeb) / sizeof(varWeb[0]); i++) {
        if (strcmp(varWeb[i].name, paramName.c_str()) == 0) {
            if (varWeb[i].traductor) {
                request->send(200, "text/plain", varWeb[i].traductor(varWeb[i].var));
            } else if (strcmp(varWeb[i].type, "float") == 0) {
                request->send(200, "text/plain", String(*((float*)varWeb[i].var) * varWeb[i].scale));
            } else if (strcmp(varWeb[i].type, "int") == 0) {
                request->send(200, "text/plain", String(*((int*)varWeb[i].var) * varWeb[i].scale));
            } else {
                request->send(200, "text/plain", "Unsupported type");
            }
            return;
        }
    }
    request->send(404); // Variable non trouvée
});


server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "Resetting ESP32...");
    delay(1000); // Ajouter un délai si nécessaire avant le redémarrage
    ESP.restart(); // Redémarrage de l'ESP32
});

    server.begin();
    Serial.println("HTTP server started");


  
  ArduinoOTA.setHostname("esp32-chauffeau");
  ArduinoOTA.setPassword("expresso");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
    Serial.println("Start updating " + type);
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });

  ArduinoOTA.begin();



    bootCount++;
}

int receivedFlag = 0;

void setFlag(void) {
    // On a reçu un paquet, mettre le flag
    receivedFlag = true;
}

uint16_t ReadVBatt() {
    digitalWrite(VBATT_GPIO, LOW);
    delay(5);
    pinMode(ADC1_CHANNEL_1, OPEN_DRAIN);
    int reading = adc1_get_raw(ADC1_CHANNEL_1);
    pinMode(ADC1_CHANNEL_1, INPUT);
    uint16_t voltage = esp_adc_cal_raw_to_voltage(reading, adc_chars);
    return voltage * VOLTAGE_DIVIDER;
}

uint16_t Sample() {
    static uint8_t i = 0;
    static uint16_t samp[VBATT_SMOOTH];
    static int32_t t = 0;
    static bool f = true;
    if (f) {
        for (uint8_t c = 0; c < VBATT_SMOOTH; c++) {
            samp[c] = 0;
        }
        f = false;
    }
    t -= samp[i];
    if (t < 0) {
        t = 0;
    }
    uint16_t voltage = ReadVBatt();
    samp[i] = voltage;
    t += samp[i];
    if (++i >= VBATT_SMOOTH) {
        i = 0;
    }
    return round(((float)t / (float)VBATT_SMOOTH));
}



unsigned long previousPompeMillis = 0;


// Fonction pour obtenir le nom de l'état de la pompe en clair
String getPompeStateName(void* state) {
    switch (*(int*)state) {
        case POMPEOFF:
            return "OFF";
        case POMPEOFF_DUTY:
            return "DUTY";
        case POMPEOFF_DELAI:
            return "OFF_D";
        case POMPEON_DELAI:
            return "ON";
        case WAITREMOTE:
            return "?R?";
        default:
            return "UNKNOWN";
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
//loop

void loop() {
    // Lire la température

   
     ArduinoOTA.handle();

    sensors.requestTemperatures();
    temperature1 = sensors.getTempCByIndex(0);
    temperature2 = sensors.getTempCByIndex(1);
    currentMillis = millis();
    
    // Construire le message JSON
  /*  String msg = "{\"model\":\"ESP32TEMP\",\"id\":\"" + NodeId + "\",\"TempCelsius\":" + String(temperature1) +
                 ",\"Elapsed\":" + String((int)round((double)currentMillis/1000)) +
                 "}";
*/

    // Émettre le message toutes les 120 secondes
 /*   if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        
        // Démarrer la transmission
        int transmissionState = radio.startTransmit(msg);
        Serial.print("Transmission state: ");
        Serial.println(transmissionState == RADIOLIB_ERR_NONE ? "success" : "failed");
        Serial.println(msg);

        delay(500);  // Attendre suffisamment longtemps pour que la transmission ait lieu
      //  receivedFlag = 0;

        // Commencer à recevoir les messages
        int startReceiveState = radio.startReceive();
        if (startReceiveState != RADIOLIB_ERR_NONE) {
            Serial.printf("Failed to start receive, code: %d\n", startReceiveState);
        }
    }
   */ 
    // Vérifier si un message a été reçu
    String receivedMsg = "";
    if (receivedFlag) {
        receivedFlag = 0;
        
        // Lire les données reçues
        int receiveState = radio.readData(receivedMsg);
        if (receiveState == RADIOLIB_ERR_NONE && radio.getPacketLength() != 0) {
    
            Serial.print("Received message: ");
            Serial.println(receivedMsg);

            // Traitement du message reçu
            temperatureRemote = processReceivedMessage(receivedMsg);

            // Pulse LED
            digitalWrite(LED, HIGH);
            delay(100);
            digitalWrite(LED, LOW);
        } else if (receiveState == RADIOLIB_ERR_CRC_MISMATCH) {
            Serial.println("CRC error");
        } else if (receiveState == RADIOLIB_ERR_RX_TIMEOUT) {
            Serial.print("..");
        } else {
            Serial.printf("Receive failed, code: %d\n", receiveState);
        }
    }

 ////////////////////////////////////////pompe////////////////////////////////
 //
    // Implémentation de la state machine pour la pompe
    dureEtat= currentMillis - previousPompeMillis;
    StatePompe Nextrequested=CurrentStatePompe;   // le meme par default

    DelaiLastLorain= currentMillis -  lorain ;
    switch (CurrentStatePompe) {
        case WAITREMOTE:
                digitalWrite(POMPE, LOW);
                if  ( currentMillis -  lorain  <   250 * 60 *1000)  // 250 secondes sans temperature remote  ?
                    {Nextrequested = POMPEOFF;
                    }
                 break;

        case POMPEOFF:
            digitalWrite(POMPE, LOW);

            if ( currentMillis > 15* 60 *60 * 1000  && ipAd=="0.0.0.0")   // essaye de de reconnecté 15mn
                    {ESP.restart(); // Redémarrage de l'ESP32
                    }

            if (temperatureRemote > seuilON   ) {
                Nextrequested = POMPEON_DELAI;
                digitalWrite(POMPE, HIGH);
            }

            if  ( currentMillis -  lorain  >  250 * 60 *1000)  // 250 secondes sans temperature remote  ?
                {Nextrequested = WAITREMOTE;
                }
            break;

        case POMPEOFF_DUTY:
            digitalWrite(POMPE, LOW);
            if (currentMillis - previousPompeMillis >= dutyoff) {
                Nextrequested = POMPEOFF;
            }
            break;

        case POMPEON_DELAI:
            digitalWrite(POMPE, HIGH);
            if (temperatureRemote < seuilOFF     ||    
               currentMillis - previousPompeMillis >= intervalpompe  )
                {
                Nextrequested = POMPEOFF_DUTY;
             }
          
            break;

        default:
            break;
    }
    if  (Nextrequested !=CurrentStatePompe)
            {
            countChangeetat++;
             if (CurrentStatePompe ==POMPEON_DELAI )  
                dureeTotalON+=dureEtat;

             if (CurrentStatePompe ==POMPEOFF_DELAI )  
                dureeTotalOFF+=dureEtat;
                     
                     
            if (CurrentStatePompe ==POMPEOFF_DUTY )  
                dureeTotalDuty+=dureEtat;


            previousPompeMillis = currentMillis;
            CurrentStatePompe=Nextrequested;


            // Construire le message JSON
            msg = "{\"model\":\"ESP32TEMP\",\"id\":\"" + NodeId + "\",\"OState\":\"" + getPompeStateName((void*)&CurrentStatePompe) + 
                        "\",\"NState\":\"" + getPompeStateName((void*)&Nextrequested) + 
                        "\",\"D\":" + String((int)round((double)dureEtat/1000)) + 
                        ",\"T1\":" + String(temperature1) + 
                        ",\"T2\":" + String(temperature2) + 
                        ",\"TR\":" + String(temperatureRemote) + 
                        ",\"Elapsed\":" + String((int)round((double)currentMillis/1000)) + 
                        "}";
                        // Démarrer la transmission
                    int transmissionState = radio.startTransmit(msg);
                    delay(500);  // Attendre suffisamment longtemps pour que la transmission ait lieu
                //  receivedFlag = 0;

        // Commencer à recevoir les messages
                int startReceiveState = radio.startReceive();
                if (startReceiveState != RADIOLIB_ERR_NONE) {
                    Serial.printf("Failed to start receive, code: %d\n", startReceiveState);
                }



            }

///////////////////////////////////////////////affichage//////////////////////////////


    // Mettre à jour l'affichage OLED
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, String(currentMillis/60000) +  "/" + String(lorain/60000)  +"  "  +  getPompeStateName((void*)&CurrentStatePompe)  +"  " + String(dureEtat/1000) );
    display.drawString(0, 14, "Temp1: " + String(temperature1) + " C");
    display.drawString(0, 28, "Temp2: " + String(temperature2) + " C");  // Affichage de la température reçue
    display.drawString(0, 44, "TempR: " + String(temperatureRemote) + " C");  // Affichage de la température reçue
    //display.drawString(0, 44, "lora: " + String(lorain));
    display.display();
    
    Serial.print("..");
}



#endif
