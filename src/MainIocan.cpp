#ifdef IOCAN

// ============================================================
// MainIocan.cpp — Contrôleur d'arrosage professionnel 8 vannes
// PlatformIO / ESP32 — RadioLib SX1262 / AsyncWebServer / OTA
// ÉVOLUTION DE : projet IOCAN chauffe-eau 4 vannes
//
// ARCHITECTURE INTERNE (tout dans ce fichier) :
//   - ConfigManager   : NVS Preferences
//   - LoggerManager   : journal circulaire 1000 entrées
//   - TimeManager     : NTP + sync LoRa
//   - ValveManager    : états, priorités, sécurités
//   - ScheduleManager : programmes + calendrier avancé
//   - ManualOverrideManager : entrées physiques courtes/longues
//   - LoRaManager     : trames STATUS / CMD / TIME_SYNC
//   - WebManager      : AsyncWebServer + WebSocket REST
// ============================================================

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <SSD1306.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <time.h>
#include <esp_task_wdt.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DNSServer.h>

#include "WebContent.h"   // SPA HTML — seul fichier séparé

// ============================================================
// SECTION 1 — CONSTANTES & PINS
// ============================================================
#define NODE_ID_DEFAULT      "IRRIGATION01"
#define SOFT_REV             "2.1"
#define OTA_HOSTNAME         "esp32-irrigation"
#define OTA_PASSWORD         "irrigation2024"



// Capteurs de température
const int oneWireBus = 6; // GPIO6 (J3-17) pour le bus OneWire
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);


// ENTREE SORTIE
static const int VANNE_COUNT       = 4;
static const int VANNE_PINS[VANNE_COUNT]     = {3,     //PD0
                                      2,     //PD1
                                      1,     //PD2
                                      38,    //PD3
                                        };

static const int OUT_PINS[4]= { 
                                      39,    //PD4
                                      40,    //PD5
                                      41,    //PD6
                                      42     //PD7
                                       };

static const int LEDVISU_PINS[VANNE_COUNT]   = {48, //PA0-LED 
                                      46, //PA1-LED
                                      45, //PA2-LED
                                      37}; //PA3-LED


// Entrées forçage manuel (INPUT_PULLUP actif bas)
#define INPUTCOUNT 8
static const int FORCE_INPUT_PINS[INPUTCOUNT] = { 47 ,  //PB0
                                                33,     //PB1
                                                34,     //PB2
                                                35,     //PB3
                                                36,     //PB4
                                                5,      //PB5
                                                20      //PB6
                                                ,26 };  //PB7


/******************************************************************************
 * Wi-Fi LoRa 32 (ESP32) - PINOUT
 * Source : schéma "Wi-Fi LoRa 32 Pin Map"
 * {}  input
 * [] output
 * !! Temp Sensor
 * Header J3 (gauche)                     Header J2 (droite)
 * ===================                    ===================
 *
 * J3-18: GPIO7 ADC1_CH6 T7                       J2-18 : GPIO19 | U1RTS 
 * J3-17: GPIO6 ADC1_CH5 TOUCH6  !oneWireBusTEMP! J2-17 : GPIO20 | U1CTS              {PB6-BP}
 * J3-16: GPIO5 ADC1_CH4 TOUCH5  {PB5-BP}         J2-16 : GPIO21 | OLED_RST
 * J3-15: GPIO4 ADC1_CH3 TOUCH4                   J2-15 : GPIO26 | SPI_CS1            {PB7-BP}
 * J3-14: GPIO3 ADC1_CH2 TOUCH3  [PD0-VANNE]    J2-14 : GPIO48                        [PA0-LED]
 * J3-13: GPIO2 ADC1_CH1 TOUCH2  [PD1-VANNE]    J2-13 : GPIO47                        {PB0-BP}
 * J3-12: GPIO1 ADC1_CH0 VBAT_RD [PD2-VANNE]    J2-12 : GPIO33 | SPIIO4               {PB1-BP}
 * J3-11: GPIO38 FSPIWP          [PD3-VANNE]    J2-11 : GPIO34 | SPIIO5               {PB2-BP}
 * J3-10: GPIO39 MTCK            [PD4-O1]       J2-10 : GPIO35 | FSPID  | LEDBlanche  {PB3-BP}
 * J3-09: GPIO40 MTDO            [PD5-O2]       J2-09 : GPIO36 | SPIIO7 | VEXT_CTL    {PB4-BP}
 * J3-08: GPIO41 MTDI            [PD6-O3]       J2-08 : GPIO0  | BOOT_SW                
 * J3-07: GPIO42 MTMS            [PD7-O4]       J2-07 : RST
 * J3-06: GPIO45                 [PA2-LED]      J2-06 : U0TXD  | GPIO43
 * J3-05: GPIO46                 [PA1-LED]      J2-05 : U0RXD  | GPIO44
 * J3-04: GPIO37                 [PA3-LED]      J2-04 : VE
 * J3-03: 3V3                                   J2-03 : VE
 * J3-02: 3V3                                   J2-02 : 5V
 * J3-01: GND                                   J2-01 : GND
 *                         USB
 * ---------------------------------------------------------------------------
 * OLED intégré
 * ---------------------------------------------------------------------------
 *
 * OLED_SDA -> GPIO17
 * OLED_SCL -> GPIO18
 * OLED_RST -> GPIO21
 *
 * ---------------------------------------------------------------------------
 * Module LoRa SX127x intégré
 * ---------------------------------------------------------------------------
 *
 * LoRa_NSS   -> GPIO8
 * LoRa_SCK   -> GPIO9
 * LoRa_MOSI  -> GPIO10
 * LoRa_MISO  -> GPIO11
 * LoRa_RST   -> GPIO12
 * LoRa_BUSY  -> GPIO13
 * LoRa_DIO1  -> GPIO14
 *
 * ---------------------------------------------------------------------------
 * SPI Flash / FSPI
 * ---------------------------------------------------------------------------
 *
 * FSPIWP     -> GPIO38
 * FSPID      -> GPIO35
 * FSPIIO4    -> GPIO33
 * FSPIIO5    -> GPIO34
 * FSPIIO7    -> GPIO36
 * FSPICS0    -> GPIO34
 * FSPICS1    -> GPIO26
 * FSPICLK    -> GPIO37
 *
 * ---------------------------------------------------------------------------
 * UART
 * ---------------------------------------------------------------------------
 *
 * UART0_TX   -> GPIO43
 * UART0_RX   -> GPIO44
 *
 * UART1_RTS  -> GPIO19
 * UART1_CTS  -> GPIO20
 *
 * ---------------------------------------------------------------------------
 * Alimentation
 * ---------------------------------------------------------------------------
 *
 * 5V         -> J2-02
 * 3.3V       -> J3-02, J3-03
 * GND        -> J2-01, J3-01
 *
 *****************************************************************************/




float temperature1 = 0;
float temperature2 = 0;
float temperatureRemote = 0;

// Bouton pour l'affichage (BOOT = GPIO 0)
const int BUTTON_PIN = 0;
int oledPage = 2; // 0=IP, 1=Temp, 2=Vanne (default)
unsigned long lastButtonPress = 0;

// LoRa SX1262 (broches selon board HELTEC / TTGO — adapter)
#define LORA_NSS        SS
#define LORA_DIO1       DIO0
#define LORA_RST        RST_LoRa
#define LORA_BUSY       BUSY_LoRa

#define LORA_FREQ_DEF   868.0f
#define LORA_BW         125.0f
#define LORA_SF         7
#define LORA_CR         5
#define LORA_SYNCWORD   0x12
#define LORA_POWER_DEF  10
#define LORA_PREAMBLE   8
#define LORA_TX_INTERVAL_MS  30000UL   // STATUS broadcast toutes les 30s

// Durées & limites
#define MAX_VALVE_OPEN_MS    (3600000UL)  // 1h sécurité absolue (ms)
#define FORCE_MANUAL_DUR_S   1800         // 30 min forçage input par défaut
#define LONG_PRESS_MS        1500         // appui long pour fermeture
#define MAX_PROGRAMS         10           // programmes par vanne
#define LOG_MAX              1000         // journal circulaire

// Modes irrigation
#define MODE_PARALLEL    0
#define MODE_SEQUENTIAL  1

// Priorités commandes (plus bas = plus prioritaire)
#define PRIO_WEB    1
#define PRIO_INPUT  2
#define PRIO_LORA   3
#define PRIO_AUTO   4
#define PRIO_NONE   99

// ============================================================
// SECTION 2 — TYPES & STRUCTURES
// ============================================================

enum class CmdSource : uint8_t { NONE=0, AUTO=1, WEB=2, PHYS_INPUT=3, LORA=4 };
static const char* srcStr(CmdSource s){
    switch(s){
        case CmdSource::AUTO:      return "AUTO";
        case CmdSource::WEB:       return "WEB";
        case CmdSource::PHYS_INPUT:return "INPUT";
        case CmdSource::LORA:      return "LORA";
        default:                   return "NONE";
    }
}
static int srcPrio(CmdSource s){
    switch(s){
        case CmdSource::WEB:       return PRIO_WEB;
        case CmdSource::PHYS_INPUT:return PRIO_INPUT;
        case CmdSource::LORA:      return PRIO_LORA;
        case CmdSource::AUTO:      return PRIO_AUTO;
        default:                   return PRIO_NONE;
    }
}

// --- Programme calendrier ---
struct Schedule {
    bool    active        = false;
    uint8_t hour          = 6;
    uint8_t minute        = 0;
    uint16_t durationSec  = 900;
    uint8_t weekDays      = 0b0111111;  // Lun-Ven bit0..bit6
    uint8_t calMode       = 0;          // 0=hebdo 1=intervalle 2=saison
    uint8_t intervalDays  = 2;
    uint8_t intervalStartMonth = 1;
    uint8_t intervalStartDay   = 1;
    uint8_t seasonStartMonth = 4;
    uint8_t seasonStartDay   = 1;
    uint8_t seasonEndMonth   = 10;
    uint8_t seasonEndDay     = 31;
    char    name[24]      = "";        // nom optionnel du programme
};

// --- Vanne ---
struct Valve {
    char      name[24];
    bool      isOpen        = false;
    CmdSource source        = CmdSource::NONE;
    int       priority      = PRIO_NONE;
    uint32_t  remainingSec  = 0;
    uint32_t  totalOpenSec  = 0;
    time_t    openedAt      = 0;
    time_t    closedAt      = 0;
    unsigned long openEndMs = 0;         // millis() deadline fermeture auto
    unsigned long lastUpdateMs = 0;
    Schedule  schedules[MAX_PROGRAMS];

    Valve(){ snprintf(name,24,"Vanne X"); }
};

// --- Journal ---
struct LogEntry {
    unsigned long tsMs;
    time_t        epoch;
    uint8_t       valveIdx; // 0xFF = système
    char          msg[80];
};



// --- Config système ---
struct SysConfig {
    char    ssid[32]      = "Freebox-7BF0EF";
    char    wifiPass[64]  = "kangourou";
    char    nodeId[24]    = NODE_ID_DEFAULT;
    float   loraFreq      = LORA_FREQ_DEF;
    int8_t  loraPower     = LORA_POWER_DEF;
    int32_t tzOffset      = 3600;         // UTC+1
    char    ntpServer[48] = "pool.ntp.org";
    uint8_t irrigMode     = MODE_PARALLEL;
    uint32_t maxOpenSec   = 3600;
    uint16_t manualForceSec = FORCE_MANUAL_DUR_S;
};

// ============================================================
// SECTION 3 — VARIABLES GLOBALES
// ============================================================

// Hardware
SSD1306  display(0x3c, SDA_OLED, SCL_OLED);
SX1262   radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
Preferences prefs;

// Serveur web & WebSocket
AsyncWebServer  server(80);
AsyncWebSocket  ws("/ws");

// Vannes
Valve  valves[VANNE_COUNT];
SysConfig sysConfig;

// Journal circulaire
LogEntry  logBuf[LOG_MAX];
uint16_t  logHead  = 0;
uint16_t  logCount = 0;

// LoRa
volatile bool loraRxFlag = false;
unsigned long lastLoraTx = 0;
int           loraRxCount = 0;
float         loraRssi = 0;

// Temps
unsigned long bootMs       = 0;
bool          timeIsSynced = false;

// Entrées manuelles
unsigned long inputPressMs[VANNE_COUNT]  = {0};
bool          inputActive[VANNE_COUNT]   = {false};

// OLED refresh
unsigned long lastOledMs = 0;


// WDT reset loop counter
unsigned long lastWdtMs = 0;

// Portail captif
bool          captivePortalActive = false;
DNSServer     dnsServer;
AsyncWebServer captiveServer(80);
// Captive portal timers/settings
unsigned long captivePortalStartMs = 0;
const unsigned long CAPTIVE_PORTAL_TIMEOUT_MS = 120000UL; // 2*60 seconds
unsigned long lastWifiReconnectMs = 0;
const unsigned long WIFI_RECONNECT_INTERVAL_MS = 30000UL; // 30 seconds
bool          pendingRestart = false;   // demande de restart après sauvegarde portail
unsigned long pendingRestartMs = 0;     // timestamp de la demande

// ============================================================
// SECTION 4 — LOGGERMANAGER (interne)
// ============================================================

time_t nowEpoch(){
    struct tm ti;
    if(!getLocalTime(&ti,5)) return 0;
    return mktime(&ti);
}

void logAdd(uint8_t vIdx, const char* msg){
    LogEntry& e = logBuf[logHead];
    e.tsMs     = millis();
    e.epoch    = nowEpoch();
    e.valveIdx = vIdx;
    strlcpy(e.msg, msg, 80);
    logHead = (logHead+1) % LOG_MAX;
    if(logCount < LOG_MAX) logCount++;
    Serial.printf("[LOG] V%u: %s\n", vIdx, msg);
}
void logSys(const char* msg){ logAdd(0xFF, msg); }

String logToJson(uint16_t last=200){
    if(last>logCount) last=logCount;
    int start = ((int)logHead - (int)last + LOG_MAX) % LOG_MAX;
    String out="[";
    for(uint16_t i=0;i<last;i++){
        const LogEntry& e = logBuf[(start+i)%LOG_MAX];
        if(i) out+=',';
        out+="{\"ts\":"+String(e.tsMs);
        out+=",\"epoch\":"+String((long)e.epoch);
        if(e.valveIdx==0xFF) out+=",\"valve\":\"SYS\"";
        else out+=",\"valve\":"+String(e.valveIdx+1);
        String m=String(e.msg);
        m.replace("\"","\\\"");
        out+=",\"msg\":\""+m+"\"}";
    }
    out+="]";
    return out;
}

// ============================================================
// SECTION 5 — CONFIGMANAGER (NVS Preferences)
// ============================================================

void configLoad(){
    prefs.begin("irrigcfg", false);
    prefs.getString("ssid",    sysConfig.ssid,    32);
    prefs.getString("wpass",   sysConfig.wifiPass, 64);
    prefs.getString("nodeId",  sysConfig.nodeId,   24);
    prefs.getString("ntpSrv",  sysConfig.ntpServer,48);
    sysConfig.loraFreq     = prefs.getFloat("lFreq",  LORA_FREQ_DEF);
    sysConfig.loraPower    = prefs.getChar("lPow",    LORA_POWER_DEF);
    sysConfig.tzOffset     = prefs.getInt("tzOff",    3600);
    sysConfig.irrigMode    = prefs.getUChar("iMode",  MODE_PARALLEL);
    sysConfig.maxOpenSec   = prefs.getUInt("maxOpen", 3600);
    sysConfig.manualForceSec = prefs.getUShort("mForce", FORCE_MANUAL_DUR_S);
    // Noms vannes
    for(int i=0;i<VANNE_COUNT;i++){
        char key[12]; snprintf(key,12,"vname%d",i);
        char def[24]; snprintf(def,24,"Vanne %d",i+1);
        prefs.getString(key, valves[i].name, 24);
        if(strlen(valves[i].name)==0) strlcpy(valves[i].name,def,24);
    }
    prefs.end();
}

void configSave(){
    prefs.begin("irrigcfg", false);
    prefs.putString("ssid",   sysConfig.ssid);
    prefs.putString("wpass",  sysConfig.wifiPass);
    prefs.putString("nodeId", sysConfig.nodeId);
    prefs.putString("ntpSrv", sysConfig.ntpServer);
    prefs.putFloat("lFreq",   sysConfig.loraFreq);
    prefs.putChar("lPow",     sysConfig.loraPower);
    prefs.putInt("tzOff",     sysConfig.tzOffset);
    prefs.putUChar("iMode",   sysConfig.irrigMode);
    prefs.putUInt("maxOpen",  sysConfig.maxOpenSec);
    prefs.putUShort("mForce", sysConfig.manualForceSec);
    for(int i=0;i<VANNE_COUNT;i++){
        char key[12]; snprintf(key,12,"vname%d",i);
        prefs.putString(key, valves[i].name);
    }
    prefs.end();
}

// Sauvegarde/chargement des programmes
void schedSave(){
    prefs.begin("schedcfg", false);
    for(int v=0;v<VANNE_COUNT;v++){
        for(int p=0;p<MAX_PROGRAMS;p++){
            char key[16]; snprintf(key,16,"s%d_%d",v,p);
            // Sérialise le programme en blob binaire
            prefs.putBytes(key, &valves[v].schedules[p], sizeof(Schedule));
        }
    }
    prefs.end();
}

void schedLoad(){
    prefs.begin("schedcfg", true);
    for(int v=0;v<VANNE_COUNT;v++){
        for(int p=0;p<MAX_PROGRAMS;p++){
            char key[16]; snprintf(key,16,"s%d_%d",v,p);
            if(prefs.isKey(key))
                prefs.getBytes(key, &valves[v].schedules[p], sizeof(Schedule));
        }
    }
    prefs.end();
}

// ============================================================
// SECTION 6 — TIMEMANAGER
// ============================================================

void timeInit(){
    configTime(sysConfig.tzOffset, 0, sysConfig.ntpServer);
    struct tm ti;
    if(getLocalTime(&ti,5000)){
        timeIsSynced = true;
        logSys("NTP synchronisé");
    } else if (oledPage == 2) {
        logSys("NTP échec (sera retenté)");
    }
}

// Convertit (année,mois,jour) en day-of-year (0-based)
static int monthDayToYday(int year, int month, int day){
    const int mdaysNorm[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    bool leap = ((year%4==0) && (year%100!=0 || year%400==0));
    int yday = 0;
    for(int m=1;m<month;m++){
        if(m==2) yday += mdaysNorm[1] + (leap?1:0);
        else yday += mdaysNorm[m-1];
    }
    yday += (day-1);
    return yday;
}

// Retourne true si on est dans la saison active
bool inSeason(const Schedule& s){
    struct tm ti;
    if(!getLocalTime(&ti,5)) return true;
    int md   = (ti.tm_mon+1)*100 + ti.tm_mday;
    int mds  = s.seasonStartMonth*100 + s.seasonStartDay;
    int mde  = s.seasonEndMonth  *100 + s.seasonEndDay;
    return md>=mds && md<=mde;
}

// ============================================================
// SECTION 7 — VALVEMANAGER
// ============================================================

// Ferme physiquement une vanne, met à jour état
void valveHardClose(int idx){
    if(idx<0||idx>=VANNE_COUNT) return;
    digitalWrite(VANNE_PINS[idx], LOW);
    // Mirror to visualization LED pin if available
    if(idx>=0 && idx<VANNE_COUNT) digitalWrite(LEDVISU_PINS[idx], LOW);
    Valve& v = valves[idx];
    if(v.isOpen){
        unsigned long openDur = (millis() - v.lastUpdateMs)/1000;
        v.totalOpenSec += openDur;
        v.closedAt = nowEpoch();
        char msg[60];
        snprintf(msg,60,"Fermée — source=%s dur=%lus", srcStr(v.source),(unsigned long)openDur);
        logAdd(idx, msg);
    }
    v.isOpen       = false;
    v.source       = CmdSource::NONE;
    v.priority     = PRIO_NONE;
    v.remainingSec = 0;
    v.openEndMs    = 0;
}

// Ouvre physiquement une vanne avec priorité et durée
// Retourne false si refusé (priorité inférieure)
bool valveHardOpen(int idx, CmdSource src, uint32_t durationSec){
    if(idx<0||idx>=VANNE_COUNT) return false;
    int prio = srcPrio(src);
    Valve& v = valves[idx];
    // Refuser si déjà ouverte avec priorité supérieure ou égale (sauf même source)
    if(v.isOpen && v.priority < prio && v.source != src) return false;
    // Mode séquentiel : fermer toutes les autres
    if(sysConfig.irrigMode == MODE_SEQUENTIAL){
        for(int i=0;i<VANNE_COUNT;i++){
            if(i!=idx && valves[i].isOpen) valveHardClose(i);
        }
    }
    // Cap de sécurité absolue
    if(durationSec > sysConfig.maxOpenSec) durationSec = sysConfig.maxOpenSec;
    if((unsigned long)durationSec * 1000UL > MAX_VALVE_OPEN_MS)
        durationSec = MAX_VALVE_OPEN_MS/1000;

    digitalWrite(VANNE_PINS[idx], HIGH);
    // Mirror to visualization LED pin
    if(idx>=0 && idx<VANNE_COUNT) digitalWrite(LEDVISU_PINS[idx], HIGH);
    v.isOpen       = true;
    v.source       = src;
    v.priority     = prio;
    v.remainingSec = durationSec;
    v.openEndMs    = millis() + (unsigned long)durationSec*1000UL;
    v.openedAt     = nowEpoch();
    v.lastUpdateMs = millis();
    char msg[60];
    snprintf(msg,60,"Ouverte — source=%s dur=%us", srcStr(src), durationSec);
    logAdd(idx, msg);
    return true;
}

// Ferme une vanne si la source a la priorité suffisante
bool valveClose(int idx, CmdSource src){
    if(idx<0||idx>=VANNE_COUNT) return false;
    int prio = srcPrio(src);
    if(valves[idx].isOpen && valves[idx].priority < prio) return false;
    valveHardClose(idx);
    return true;
}

// Mise à jour des timers vannes (appelé dans loop, sans delay)
void valveUpdate(){
    unsigned long now = millis();
    for(int i=0;i<VANNE_COUNT;i++){
        Valve& v = valves[i];
        if(!v.isOpen) continue;
        // Sécurité absolue max ouverture
        if(now - v.lastUpdateMs > MAX_VALVE_OPEN_MS){
            logAdd(i,"SÉCURITÉ: fermeture max durée atteinte");
            valveHardClose(i);
            continue;
        }
        // Deadline programmée
        if(v.openEndMs && now >= v.openEndMs){
            valveHardClose(i);
            continue;
        }
        // Mise à jour temps restant (arrondi à la seconde)
        if(v.openEndMs > now)
            v.remainingSec = (v.openEndMs - now + 500) / 1000;
        else
            v.remainingSec = 0;
    }
}

// Ferme toutes les vannes (sécurité boot / reset)
void valveCloseAll(CmdSource src=CmdSource::WEB){
    for(int i=0;i<VANNE_COUNT;i++) valveHardClose(i);
    // ensure visualization LEDs are also cleared
    for(int i=0;i<VANNE_COUNT;i++) digitalWrite(LEDVISU_PINS[i], LOW);
    logSys("Toutes vannes fermées");
}

// ============================================================
// SECTION 8 — SCHEDULEMANAGER
// ============================================================

// Vérifie si un programme doit se déclencher maintenant
// Appelé 1×/min dans loop
unsigned long lastSchedCheckMs = 0;

void schedCheck(){
    if(!timeIsSynced) return;
    unsigned long now = millis();
    if(now - lastSchedCheckMs < 60000UL) return;
    lastSchedCheckMs = now;

    struct tm ti;
    if(!getLocalTime(&ti,5)) return;
    int curH = ti.tm_hour;
    int curM = ti.tm_min;
    int dow  = (ti.tm_wday + 6) % 7;  // 0=Lun
    int yday = ti.tm_yday;

    for(int v=0;v<VANNE_COUNT;v++){
        for(int p=0;p<MAX_PROGRAMS;p++){
            Schedule& s = valves[v].schedules[p];
            if(!s.active) continue;
            if(s.hour!=curH || s.minute!=curM) continue;

            bool trigger = false;
            switch(s.calMode){
                case 0: // hebdomadaire
                    trigger = !!(s.weekDays & (1<<dow));
                    break;
                case 1: // intervalle
                    if(s.intervalDays>0){
                        int startY = monthDayToYday(ti.tm_year+1900, s.intervalStartMonth, s.intervalStartDay);
                        int daysInYear = 365 + (( (ti.tm_year+1900)%4==0 && ((ti.tm_year+1900)%100!=0 || (ti.tm_year+1900)%400==0))?1:0);
                        int diff = (yday - startY + daysInYear) % s.intervalDays;
                        trigger = (diff == 0);
                    } else trigger = false;
                    break;
                case 2: // saison
                    trigger = inSeason(s);
                    break;
            }
            if(!trigger) continue;

            // N'ouvrir que si non commandée par priorité supérieure
            Valve& vv = valves[v];
            if(vv.isOpen && vv.priority < PRIO_AUTO){
                logAdd(v,"Programme ignoré (priorité supérieure active)");
                continue;
            }
            valveHardOpen(v, CmdSource::AUTO, s.durationSec);
        }
    }
}

// ============================================================
// SECTION 9 — MANUALOVERRIDEMANAGER (entrées physiques)
// ============================================================

void inputUpdate(){
    unsigned long now = millis();
    for(int i=0;i<VANNE_COUNT;i++){
        bool pressed = (digitalRead(FORCE_INPUT_PINS[i]) == LOW);
        if(pressed && !inputActive[i]){
            // Début appui (mémoriser timestamp)
            inputPressMs[i] = now;
            inputActive[i]  = true;
        }
        else if(!pressed && inputActive[i]){
            // Relâchement -> vérifier debounce puis toggle l'état de la vanne
            unsigned long dur = now - inputPressMs[i];
            inputActive[i] = false;
            const unsigned long DEBOUNCE_MS = 50;
            if(dur < DEBOUNCE_MS) {
                // bruit, ignorer
                continue;
            }
            // Toggle: si ouverte -> fermer, sinon ouvrir pour la durée configurée
            if(valves[i].isOpen){
                valveClose(i, CmdSource::PHYS_INPUT);
            } else {
                valveHardOpen(i, CmdSource::PHYS_INPUT, sysConfig.manualForceSec);
            }
        }
    }
}

// ============================================================
// SECTION 10 — LORAMANAGER
// ============================================================

void IRAM_ATTR loraSetFlag(){ loraRxFlag = true; }

// Construit la trame STATUS JSON
String loraBuildStatus(){
    StaticJsonDocument<1024> doc;
    doc["id"]   = sysConfig.nodeId;
    doc["type"] = "STATUS";
    doc["heap"]   = ESP.getFreeHeap();
    doc["uptime"] = millis()/1000;
    doc["rssi"]   = loraRssi;
    doc["alarm"]  = 0;
    JsonArray vannesArr   = doc.createNestedArray("vannes");
    JsonArray remArr      = doc.createNestedArray("remaining");
    JsonArray srcArr      = doc.createNestedArray("source");
    for(int i=0;i<VANNE_COUNT;i++){
        vannesArr.add(valves[i].isOpen ? 1 : 0);
        remArr.add(valves[i].remainingSec);
        srcArr.add(srcStr(valves[i].source));
    }
    String out; serializeJson(doc,out);
    return out;
}

// Traitement commande LoRa reçue
void loraProcessCmd(JsonDocument& doc){
    const char* cmd = doc["cmd"];
    if(!cmd) return;

    if(strcmp(cmd,"OPEN")==0){
        int valve = doc["valve"] | -1;
        uint32_t dur = doc["duration"] | (uint32_t)sysConfig.maxOpenSec;
        if(valve>=0 && valve<VANNE_COUNT)
            valveHardOpen(valve, CmdSource::LORA, dur);
    }
    else if(strcmp(cmd,"CLOSE")==0){
        int valve = doc["valve"] | -1;
        if(valve>=0 && valve<VANNE_COUNT)
            valveClose(valve, CmdSource::LORA);
        else if(valve==-1)
            valveCloseAll(CmdSource::LORA);
    }
    else if(strcmp(cmd,"FORCE")==0){
        int valve = doc["valve"] | -1;
        uint32_t dur = doc["duration"] | (uint32_t)sysConfig.manualForceSec;
        if(valve>=0 && valve<VANNE_COUNT)
            valveHardOpen(valve, CmdSource::LORA, dur);
    }
    else if(strcmp(cmd,"GET_STATUS")==0){
        // Répondre immédiatement
        String msg = loraBuildStatus();
        int st = radio.startTransmit(msg);
        if(st != RADIOLIB_ERR_NONE) Serial.printf("LoRa TX err %d\n",st);
    }
    else if(strcmp(cmd,"CLOSE_ALL")==0){
        valveCloseAll(CmdSource::LORA);
    }
}

// Réception LoRa (non-bloquant)
void loraRxProcess(){
    if(!loraRxFlag) return;
    loraRxFlag = false;
    String msg;
    int st = radio.readData(msg);
    if(st == RADIOLIB_ERR_NONE && radio.getPacketLength()>0){
        loraRssi = radio.getRSSI();
        loraRxCount++;
        Serial.print("[LoRa RX] "); Serial.println(msg);
        logSys(("LoRa reçu: "+msg.substring(0,40)).c_str());

        StaticJsonDocument<512> doc;
        if(deserializeJson(doc,msg) == DeserializationError::Ok){
            const char* id = doc["id"];
            if (id && strcmp(id, "Yaourt1") == 0) {
                temperatureRemote = doc["TempCelsius"] | temperatureRemote;
            }

            const char* type = doc["type"];
            if(type){
                if(strcmp(type,"CMD")==0)       loraProcessCmd(doc);
                else if(strcmp(type,"TIME_SYNC")==0){
                    long epoch = doc["epoch"] | 0L;
                    if(epoch>0){
                        struct timeval tv = {epoch,0};
                        settimeofday(&tv,nullptr);
                        timeIsSynced = true;
                        logSys("Heure synchronisée via LoRa");
                    }
                }
            }
        }
    } else if(st != RADIOLIB_ERR_RX_TIMEOUT){
        Serial.printf("[LoRa] Err lecture %d\n",st);
    }
    // Reprendre la réception
    radio.startReceive();
}

// Envoi STATUS périodique
void loraTxUpdate(){
    unsigned long now = millis();
    if(now - lastLoraTx < LORA_TX_INTERVAL_MS) return;
    lastLoraTx = now;
    String msg = loraBuildStatus();
    int st = radio.startTransmit(msg);
    Serial.print("[LoRa TX STATUS] "); Serial.println(st==RADIOLIB_ERR_NONE?"OK":"FAIL");
    // Reprendre RX après TX
    delayMicroseconds(2000);  // mini pause hardware (non bloquant pour la logique)
    radio.startReceive();
}

// ============================================================
// SECTION 11 — WEBSOCKET BROADCAST
// ============================================================

unsigned long lastWsBroadcastMs = 0;

String buildStatusJson(){
    StaticJsonDocument<2048> doc;
    doc["type"]   = "STATUS";
    doc["uptime"] = millis()/1000;
    doc["heap"]   = ESP.getFreeHeap();
    doc["temp1"]  = temperature1;
    doc["temp2"]  = temperature2;
    doc["tempR"]  = temperatureRemote;
    JsonArray arr = doc.createNestedArray("valves");
    for(int i=0;i<VANNE_COUNT;i++){
        Valve& v = valves[i];
        JsonObject o = arr.createNestedObject();
        o["name"]        = v.name;
        o["state"]       = v.isOpen ? 1 : 0;
        o["source"]      = srcStr(v.source);
        o["remainingSec"]= v.remainingSec;
        o["openedAt"]    = (long)v.openedAt;
        o["totalOpenSec"]= v.totalOpenSec;
    }
    String out; serializeJson(doc,out);
    return out;
}

void wsBroadcastStatus(){
    if(ws.count()==0) return;
    unsigned long now = millis();
    if(now - lastWsBroadcastMs < 1000) return;   // max 1×/s
    lastWsBroadcastMs = now;
    String json = buildStatusJson();
    ws.textAll(json);
}

void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len)
{
    if(type == WS_EVT_CONNECT){
        // Envoyer l'état complet à la connexion
        client->text(buildStatusJson());
    }
}

// ============================================================
// SECTION 12 — WEBMANAGER (routes REST)
// ============================================================

// Helper: réponse JSON 200
static void jsonResp(AsyncWebServerRequest* req, const String& body, int code=200){
    AsyncWebServerResponse* r = req->beginResponse(code,"application/json",body);
    r->addHeader("Access-Control-Allow-Origin","*");
    req->send(r);
}

// Construit le JSON config pour /api/config GET
String configToJson(){
    StaticJsonDocument<1024> doc;
    doc["ssid"]         = sysConfig.ssid;
    doc["ntpServer"]    = sysConfig.ntpServer;
    doc["tzOffset"]     = sysConfig.tzOffset;
    doc["loraFreq"]     = sysConfig.loraFreq;
    doc["loraPower"]    = sysConfig.loraPower;
    doc["nodeId"]       = sysConfig.nodeId;
    doc["irrigMode"]    = sysConfig.irrigMode;
    doc["maxOpenSec"]   = sysConfig.maxOpenSec;
    doc["manualForceSec"]=sysConfig.manualForceSec;
    JsonArray names = doc.createNestedArray("valveNames");
    for(int i=0;i<VANNE_COUNT;i++) names.add(valves[i].name);
    String out; serializeJson(doc,out);
    return out;
}

// Construit le JSON des programmes (flat list)
//
// IMPORTANT (correctif bug "V3/V4 invisibles dans les programmes") :
// Avec VANNE_COUNT vannes × MAX_PROGRAMS slots × ~16 champs par programme,
// l'ancien buffer StaticJsonDocument<4096> était sous-dimensionné (besoin
// réel de l'ordre de 8-10 Ko avec VANNE_COUNT=4, et bien plus avec 8 vannes).
// ArduinoJson ne signale pas d'erreur quand le pool mémoire est plein : les
// objets ajoutés après saturation sont silencieusement vides ou tronqués.
// Comme la boucle remplit V1 puis V2 puis V3 puis V4 dans cet ordre, ce sont
// les DERNIÈRES vannes traitées (V3, V4, ...) qui se retrouvaient amputées —
// exactement le symptôme observé : leurs programmes "n'apparaissaient pas".
// On utilise donc un DynamicJsonDocument dimensionné dynamiquement selon
// VANNE_COUNT et MAX_PROGRAMS, avec une marge de sécurité, et on vérifie
// explicitement le résultat de overflowed() pour journaliser le problème
// au lieu de le laisser passer silencieusement si jamais la taille venait
// à manquer de nouveau (ex: noms de programmes plus longs).
String schedulesToJson(){
    // ~220 octets par programme (slot JsonObject + 16 champs + chaîne name)
    // est une estimation large pour ArduinoJson v6 sur ESP32 (32-bit).
    const size_t perSchedule = 220;
    const size_t capacity = JSON_ARRAY_SIZE(VANNE_COUNT * MAX_PROGRAMS)
                           + (size_t)VANNE_COUNT * MAX_PROGRAMS * (JSON_OBJECT_SIZE(16) + perSchedule)
                           + 512; // marge fixe (clé "schedules" + alignement)
    DynamicJsonDocument doc(capacity);
    JsonArray arr = doc.createNestedArray("schedules");
    for(int v=0;v<VANNE_COUNT;v++){
        for(int p=0;p<MAX_PROGRAMS;p++){
            Schedule& s = valves[v].schedules[p];
            // N'exporter que les actifs OU toutes (pour édition)
            JsonObject o = arr.createNestedObject();
            o["valve"]            = v;
            o["schedIdx"]         = p;
            o["active"]           = s.active;
            o["hour"]             = s.hour;
            o["minute"]           = s.minute;
            o["durationSec"]      = s.durationSec;
            o["name"]             = s.name;
            o["weekDays"]         = s.weekDays;
            o["intervalStartMonth"] = s.intervalStartMonth;
            o["intervalStartDay"]   = s.intervalStartDay;
            o["calMode"]          = s.calMode;
            o["intervalDays"]     = s.intervalDays;
            o["seasonStartMonth"] = s.seasonStartMonth;
            o["seasonStartDay"]   = s.seasonStartDay;
            o["seasonEndMonth"]   = s.seasonEndMonth;
            o["seasonEndDay"]     = s.seasonEndDay;
        }
    }
    if(doc.overflowed()){
        // Ne devrait plus arriver avec le dimensionnement ci-dessus, mais on
        // journalise pour diagnostic immédiat plutôt qu'une troncature muette.
        logSys("ERREUR: buffer JSON programmes insuffisant (overflow)");
    }
    String out; serializeJson(doc,out);
    return out;
}

void webSetup(){
    // ── SPA principale
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* req){
        req->send(200,"text/html",WEB_HTML);
    });

    // ── Statut complet GET
    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* req){
        jsonResp(req, buildStatusJson());
    });

    // ── Ouvrir vanne POST /api/valve/open  {valve,duration,source}
    server.on("/api/valve/open", HTTP_POST, [](AsyncWebServerRequest* req){},
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
            StaticJsonDocument<256> doc;
            if(deserializeJson(doc,data,len)){jsonResp(req,"{\"ok\":false}",400);return;}
            int idx  = doc["valve"] | -1;
            uint32_t dur = doc["duration"] | (uint32_t)sysConfig.maxOpenSec;
            if(idx<0||idx>=VANNE_COUNT){jsonResp(req,"{\"ok\":false}",400);return;}
            bool ok = valveHardOpen(idx, CmdSource::WEB, dur);
            jsonResp(req, ok ? "{\"ok\":true}" : "{\"ok\":false,\"reason\":\"priority\"}");
        }
    );

    // ── Fermer vanne POST /api/valve/close  {valve}
    server.on("/api/valve/close", HTTP_POST, [](AsyncWebServerRequest* req){},
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
            StaticJsonDocument<128> doc;
            if(deserializeJson(doc,data,len)){jsonResp(req,"{\"ok\":false}",400);return;}
            int idx = doc["valve"] | -1;
            if(idx<0||idx>=VANNE_COUNT){jsonResp(req,"{\"ok\":false}",400);return;}
            valveClose(idx, CmdSource::WEB);
            jsonResp(req,"{\"ok\":true}");
        }
    );

    // ── Forcer vanne POST /api/valve/force  {valve,duration}
    server.on("/api/valve/force", HTTP_POST, [](AsyncWebServerRequest* req){},
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
            StaticJsonDocument<128> doc;
            if(deserializeJson(doc,data,len)){jsonResp(req,"{\"ok\":false}",400);return;}
            int idx  = doc["valve"] | -1;
            uint32_t dur = doc["duration"] | (uint32_t)sysConfig.manualForceSec;
            if(idx<0||idx>=VANNE_COUNT){jsonResp(req,"{\"ok\":false}",400);return;}
            valveHardOpen(idx, CmdSource::WEB, dur);   // WEB = priorité 1
            jsonResp(req,"{\"ok\":true}");
        }
    );

    // ── Fermer tout POST /api/valve/closeall
    server.on("/api/valve/closeall", HTTP_POST, [](AsyncWebServerRequest* req){
        valveCloseAll(CmdSource::WEB);
        jsonResp(req,"{\"ok\":true}");
    });

    // ── Programmes GET
    server.on("/api/schedules", HTTP_GET, [](AsyncWebServerRequest* req){
        jsonResp(req, schedulesToJson());
    });

    // ── Sauver programme POST /api/schedule/save
    //
    // IMPORTANT (correctif bug "changement de vanne") :
    // Un programme est identifié de façon stable par (origValve, schedIdx) tel
    // qu'il existait AVANT modification. Si la vanne cible (valve) diffère de
    // origValve, on ne réutilise JAMAIS le même schedIdx sur la nouvelle vanne :
    // on cherche un slot libre dédié sur la vanne de destination, on y copie le
    // programme, puis on libère l'ancien slot. Cela évite d'écraser silencieusement
    // un programme existant sur la vanne de destination et évite que la ligne
    // "saute" à un index arbitraire dans la liste à plat retournée par /api/schedules.
    server.on("/api/schedule/save", HTTP_POST, [](AsyncWebServerRequest* req){},
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
            StaticJsonDocument<512> doc;
            if(deserializeJson(doc,data,len)){jsonResp(req,"{\"ok\":false}",400);return;}
            int v      = doc["valve"]    | -1;
            int idx    = doc["schedIdx"] | -1;
            int origV  = doc["origValve"] | -1;
            bool isMove = (origV>=0 && origV!=v);

            if(v<0||v>=VANNE_COUNT){jsonResp(req,"{\"ok\":false,\"reason\":\"valve\"}",400);return;}

            if(isMove){
                // Déplacement vers une autre vanne : ne JAMAIS réutiliser le même
                // schedIdx tel quel — chercher un slot libre sur la vanne cible.
                int freeIdx = -1;
                for(int p=0;p<MAX_PROGRAMS;p++){
                    if(!valves[v].schedules[p].active){ freeIdx=p; break; }
                }
                if(freeIdx<0){
                    jsonResp(req,"{\"ok\":false,\"reason\":\"full\"}");
                    return;
                }
                idx = freeIdx;
            } else if(idx<0){
                // Nouveau programme sur la même vanne : chercher un slot libre
                for(int p=0;p<MAX_PROGRAMS;p++){
                    if(!valves[v].schedules[p].active){ idx=p; break; }
                }
                if(idx<0){jsonResp(req,"{\"ok\":false,\"reason\":\"full\"}");return;}
            }

            if(idx<0||idx>=MAX_PROGRAMS){jsonResp(req,"{\"ok\":false}",400);return;}

            Schedule& s = valves[v].schedules[idx];
            s.active           = doc["active"]           | true;
            s.hour             = doc["hour"]             | 6;
            s.minute           = doc["minute"]           | 0;
            s.durationSec      = doc["durationSec"]      | 900;
            s.weekDays         = doc["weekDays"]         | 0b0111111;
            s.calMode          = doc["calMode"]          | 0;
            s.intervalDays     = doc["intervalDays"]     | 2;
            s.intervalStartMonth = doc["intervalStartMonth"] | s.intervalStartMonth;
            s.intervalStartDay   = doc["intervalStartDay"]   | s.intervalStartDay;
            s.seasonStartMonth = doc["seasonStartMonth"] | 4;
            s.seasonStartDay   = doc["seasonStartDay"]   | 1;
            s.seasonEndMonth   = doc["seasonEndMonth"]   | 10;
            s.seasonEndDay     = doc["seasonEndDay"]     | 31;
            if(doc.containsKey("name")) strlcpy(s.name, doc["name"], sizeof(s.name));

            // Libérer l'ancien slot UNIQUEMENT après avoir écrit le nouveau avec succès
            if(isMove){
                valves[origV].schedules[ (int)(doc["schedIdx"] | -1) ] = Schedule();
            }

            schedSave();
            // Renvoyer la position finale pour que le frontend puisse resynchroniser
            // immédiatement la sélection en cours d'édition sans devoir deviner.
            String resp = "{\"ok\":true,\"valve\":"+String(v)+",\"schedIdx\":"+String(idx)+"}";
            jsonResp(req,resp);
        }
    );

    // ── Supprimer programme POST /api/schedule/delete {valve,schedIdx}
    server.on("/api/schedule/delete", HTTP_POST, [](AsyncWebServerRequest* req){},
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
            StaticJsonDocument<128> doc;
            if(deserializeJson(doc,data,len)){jsonResp(req,"{\"ok\":false}",400);return;}
            int v=doc["valve"]|-1, p=doc["schedIdx"]|-1;
            if(v<0||v>=VANNE_COUNT||p<0||p>=MAX_PROGRAMS){jsonResp(req,"{\"ok\":false}",400);return;}
            valves[v].schedules[p] = Schedule();  // reset
            schedSave();
            jsonResp(req,"{\"ok\":true}");
        }
    );

    // ── Toggle actif programme POST /api/schedule/toggle {valve,schedIdx,active}
    server.on("/api/schedule/toggle", HTTP_POST, [](AsyncWebServerRequest* req){},
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
            StaticJsonDocument<128> doc;
            if(deserializeJson(doc,data,len)){jsonResp(req,"{\"ok\":false}",400);return;}
            int v=doc["valve"]|-1, p=doc["schedIdx"]|-1;
            bool act=doc["active"]|false;
            if(v<0||v>=VANNE_COUNT||p<0||p>=MAX_PROGRAMS){jsonResp(req,"{\"ok\":false}",400);return;}
            valves[v].schedules[p].active=act;
            schedSave();
            jsonResp(req,"{\"ok\":true}");
        }
    );

    // ── Journal GET /api/log?n=200
    server.on("/api/log", HTTP_GET, [](AsyncWebServerRequest* req){
        uint16_t n=200;
        if(req->hasParam("n")) n=req->getParam("n")->value().toInt();
        jsonResp(req, logToJson(n));
    });

    // ── Config GET
    server.on("/api/config", HTTP_GET, [](AsyncWebServerRequest* req){
        jsonResp(req, configToJson());
    });

    // ── Config POST /api/config
    server.on("/api/config", HTTP_POST, [](AsyncWebServerRequest* req){},
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
            StaticJsonDocument<1024> doc;
            if(deserializeJson(doc,data,len)){jsonResp(req,"{\"ok\":false}",400);return;}
            if(doc.containsKey("ssid"))   strlcpy(sysConfig.ssid,   doc["ssid"],   32);
            if(doc.containsKey("wifiPass") && strlen(doc["wifiPass"])>0)
                strlcpy(sysConfig.wifiPass, doc["wifiPass"], 64);
            if(doc.containsKey("ntpServer")) strlcpy(sysConfig.ntpServer,doc["ntpServer"],48);
            if(doc.containsKey("nodeId"))    strlcpy(sysConfig.nodeId,   doc["nodeId"],   24);
            sysConfig.tzOffset     = doc["tzOffset"]     | sysConfig.tzOffset;
            sysConfig.loraFreq     = doc["loraFreq"]     | sysConfig.loraFreq;
            sysConfig.loraPower    = doc["loraPower"]    | sysConfig.loraPower;
            sysConfig.irrigMode    = doc["irrigMode"]    | sysConfig.irrigMode;
            sysConfig.maxOpenSec   = doc["maxOpenSec"]   | sysConfig.maxOpenSec;
            sysConfig.manualForceSec=doc["manualForceSec"]|sysConfig.manualForceSec;
            // Noms vannes
            if(doc.containsKey("valveNames")){
                JsonArray arr = doc["valveNames"];
                for(int i=0;i<VANNE_COUNT && i<(int)arr.size();i++)
                    strlcpy(valves[i].name, arr[i]|"", 24);
            }
            configSave();
            jsonResp(req,"{\"ok\":true}");
        }
    );

    // ── Reset POST /api/reset
    server.on("/api/reset", HTTP_POST, [](AsyncWebServerRequest* req){
        jsonResp(req,"{\"ok\":true}");
        logSys("Redémarrage demandé via Web");
        delay(300);
        ESP.restart();
    });

    // ── Compat legacy /reset GET
    server.on("/reset", HTTP_GET, [](AsyncWebServerRequest* req){
        req->send(200,"text/plain","Redémarrage...");
        delay(300); ESP.restart();
    });

    // WebSocket
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    server.begin();
    logSys("Serveur HTTP démarré");
}

// ============================================================
// SECTION 13 — OLED
// ============================================================

void oledUpdate(){
    unsigned long now = millis();
    if(now - lastOledMs < 2000 && lastOledMs != 0) return;
    lastOledMs = now;

    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);

    // ── Page spéciale portail captif
    if (captivePortalActive) {
        display.drawString(0,  0, "!! WiFi FAIL !!");
        display.drawString(0, 14, "AP: IrrigPro-Setup");
        display.drawString(0, 28, "-> 192.168.4.1");
        display.drawString(0, 42, "Config WiFi requise");
        display.display();
        return;
    }
    if (oledPage == 0) {
        display.drawString(0,0, "-Reseau WiFi:");
        display.drawString(0,14, WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "Deconnecte");
    } else if (oledPage == 1) {
        display.drawString(0,0, "Temperatures:");
        display.drawString(0,14, "Temp1: " + String(temperature1) + " C");
        display.drawString(0,28, "Temp2: " + String(temperature2) + " C");
        display.drawString(0,42, "TempR: " + String(temperatureRemote) + " C");
    } else if (oledPage == 2) {
        // Ligne 0: uptime
        display.drawString(0,0, "Uptime: " + String(now/60000) + "min");

        // Ligne 1..N: état vannes 2 par 2 (calculer dynamiquement selon VANNE_COUNT)
        int rows = (VANNE_COUNT + 1) / 2;
        for(int row=0; row<rows; row++){
            int i1 = row*2;
            int i2 = row*2 + 1;
            String s1 = "";
            String s2 = "";
            if(i1 < VANNE_COUNT) s1 = String(i1+1) + (valves[i1].isOpen?":ON ":":-- ");
            if(i2 < VANNE_COUNT) s2 = String(i2+1) + (valves[i2].isOpen?":ON":":--");
            display.drawString(0, 12 + row*13, s1 + s2);
        }
        // Afficher l'état des entrées (FORCE_INPUT_PINS) — LOW = appuyé
        String inStates = "Entrées: ";
        String serialStates = "";
        for(int i=0;i<VANNE_COUNT;i++){
            int pin = FORCE_INPUT_PINS[i];
            int val = digitalRead(pin);
            inStates += String("I") + String(i+1) + ":" + String(val) + " ";
            serialStates += String(pin) +":"+String(val) + (i+1<VANNE_COUNT?" ":"");
        }
        display.drawString(0,40, inStates);
        // Log concis pour debug (affiché toutes les secondes via oledUpdate)
        Serial.printf("[INPUTS] %s\n", serialStates.c_str());
        // Ligne 5: LoRa info
        display.drawString(0,52,"LoRa rx:"+String(loraRxCount)+" rssi:"+String((int)loraRssi));
    } else if (oledPage == 3) {
        // Compact 4-line I/O table: header + oPD + oPA + Ipx
        // Header with column numbers
        display.drawString(0, 0, "----0 1 2 3 4 5 6 7");

        // Row oPD: PD0..PD7 (VANNE_PINS -> PD0..PD3, OUT_PINS -> PD4..PD7)
        String rowPD = "oPD:";
        for(int col=0; col<8; col++){
            int val = -1;
            if(col < 4){
                if(col < VANNE_COUNT) val = digitalRead(VANNE_PINS[col]);
            } else {
                int idx = col - 4;
                if(idx < (int)(sizeof(OUT_PINS)/sizeof(OUT_PINS[0]))) val = digitalRead(OUT_PINS[idx]);
            }
            if(val < 0) rowPD += "  "; else rowPD += (val==HIGH?" 1":" 0");
        }
        display.drawString(0, 12, rowPD);

        // Row oPA: visualization LEDs (only present for first VANNE_COUNT columns)
        String rowPA = "oPA:";
        for(int col=0; col<8; col++){
            if(col < VANNE_COUNT){
                int v = digitalRead(LEDVISU_PINS[col]);
                rowPA += (v==HIGH?" 1":" 0");
            } else {
                rowPA += "  ";
            }
        }
        display.drawString(0, 36, rowPA);

        // Row Ipx: inputs PB0..PB7 (FORCE_INPUT_PINS)
        String rowI = "iPB :";
        for(int col=0; col<8; col++){
            if(col < INPUTCOUNT){
                int v = digitalRead(FORCE_INPUT_PINS[col]);
                rowI += (v==HIGH?" 1":" 0");
            } else {
                rowI += "  ";
            }
        }
        display.drawString(0, 24, rowI);
    }
    display.display();
}

// ============================================================
// SECTION 14 — OTA
// ============================================================

void otaSetup(){
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.onStart([](){
        valveCloseAll(CmdSource::WEB);
        logSys("OTA démarré — vannes fermées");
    });
    ArduinoOTA.onEnd([](){  logSys("OTA terminé"); });
    ArduinoOTA.onProgress([](unsigned int p, unsigned int t){
        Serial.printf("OTA %u%%\r",(p/(t/100)));
    });
    ArduinoOTA.onError([](ota_error_t e){
        Serial.printf("OTA Error[%u]\n",e);
    });
    ArduinoOTA.begin();
}

// ============================================================
// SECTION 14b — PORTAIL CAPTIF
// ============================================================
static const char CAPTIVE_HTML_1[] PROGMEM = R"CPEOF(
<!DOCTYPE html><html lang="fr"><head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>IrrigPro — Config WiFi</title>
<style>
  body{background:#0d1117;color:#e6edf3;font-family:system-ui,sans-serif;
       display:flex;align-items:center;justify-content:center;min-height:100vh;margin:0}
  .box{background:#161b22;border:1px solid #30363d;border-radius:12px;
       padding:32px 28px;width:100%;max-width:360px}
  h2{margin:0 0 20px;font-size:1.1rem;color:#388bfd;text-align:center}
  label{font-size:.82rem;color:#8b949e;display:block;margin-bottom:4px}
  input{width:100%;box-sizing:border-box;background:#21262d;border:1px solid #30363d;
        border-radius:6px;color:#e6edf3;padding:9px 12px;font-size:.95rem;margin-bottom:14px}
  button{width:100%;background:#2ea043;color:#fff;border:none;border-radius:6px;
         padding:11px;font-size:1rem;cursor:pointer;font-weight:600}
  button:hover{filter:brightness(1.15)}
  .note{font-size:.75rem;color:#8b949e;margin-top:14px;text-align:center}
  .logo{text-align:center;font-size:1.4rem;font-weight:700;margin-bottom:6px}
  .sub{text-align:center;font-size:.8rem;color:#8b949e;margin-bottom:20px}
</style>
</head><body>
<div class="box">
  <div class="logo">&#x1F6BF; IrrigPro</div>
  <div class="sub">Configuration WiFi</div>
  <h2>Connexion au r&eacute;seau</h2>
  <form method="POST" action="/save">
    <label>Nom du r&eacute;seau (SSID) <a href="#" onclick="fetch('/scan').then(r=>r.text()).then(t=>document.getElementById('ssid_list').innerHTML='<option value=\'\'>R&eacute;seaux d&eacute;tect&eacute;s...</option>'+t); return false;" style="float:right;color:#388bfd;text-decoration:none">&#x1F50D; Actualiser</a></label>
    <select id="ssid_list" onchange="document.getElementsByName('ssid')[0].value = this.value" style="width:100%;box-sizing:border-box;background:#21262d;border:1px solid #30363d;border-radius:6px;color:#e6edf3;padding:9px 12px;font-size:.95rem;margin-bottom:8px">
      <option value="">(S&eacute;lectionnez un r&eacute;seau scann&eacute;)</option>
)CPEOF";

static const char CAPTIVE_HTML_2[] PROGMEM = R"CPEOF(
    </select>
    <input type="text" name="ssid" placeholder="Ou saisissez un r&eacute;seau masqu&eacute;" required/>
    <label>Mot de passe</label>
    <input type="password" name="pass" placeholder="••••••••"/>
    <button type="submit">Enregistrer &amp; Red&eacute;marrer</button>
  </form>
    <div class="note">L'appareil red&eacute;marrera et se connectera au r&eacute;seau choisi.</div>
    <div id="timeout" class="note" style="margin-top:10px">Portail actif — retour en mode irrigation dans 60s</div>
</div>
<script>
    (function(){
        var t = 60;
        var el = document.getElementById('timeout');
        function updateText(s){ el.textContent = 'Portail actif — retour en mode irrigation dans ' + s + 's'; }
        var iv = setInterval(function(){
            if(t<=0){
                clearInterval(iv);
                el.textContent = 'Fin du portail — retour en cours...';
                setTimeout(function(){ location.reload(true); }, 1000);
                return;
            }
            updateText(t);
            t--;
        }, 1000);
        updateText(t);
        
        // Auto-refresh the scan list after 3 seconds
        setTimeout(function(){
            fetch('/scan').then(r=>r.text()).then(t=>{
                if(t && t.length>0 && t.indexOf('Recherche')===-1 && t.indexOf('Erreur')===-1) {
                    document.getElementById('ssid_list').innerHTML='<option value=\'\'>R&eacute;seaux d&eacute;tect&eacute;s...</option>'+t;
                }
            });
        }, 3000);
    })();
</script>
</body></html>
)CPEOF";

void startCaptivePortal() {
    captivePortalActive = true;

    // Passer en mode mixte AP_STA pour pouvoir scanner les réseaux
    WiFi.disconnect(); // ne pas passer 'true' sinon ça coupe la radio!
    delay(200);
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP("IrrigPro-Setup");   // pas de mot de passe = accès libre
    delay(500);

    // Lancer un scan asynchrone des réseaux WiFi en arrière-plan
    Serial.println("[CaptivePortal] Lancement du scan WiFi...");
    WiFi.scanNetworks(true);

    IPAddress apIP(192, 168, 4, 1);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));

    // DNS : renvoyer toutes les requêtes vers l'IP de l'AP (portail captif)
    dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    dnsServer.start(53, "*", apIP);

    Serial.println("[CaptivePortal] AP: IrrigPro-Setup  IP: 192.168.4.1");
    logSys("Portail captif actif — SSID: IrrigPro-Setup");

    // Affichage OLED
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0,  "WiFi: ECHEC config!");
    display.drawString(0, 14, "AP: IrrigPro-Setup");
    display.drawString(0, 28, "-> 192.168.4.1");
    display.drawString(0, 42, "Configurer le WiFi");
    display.display();

    // Servir la page de config sur le serveur dédié (port 80)
    auto servePortal = [](AsyncWebServerRequest* req) {
        String options = "";
        int n = WiFi.scanComplete();
        Serial.printf("[CaptivePortal] GET / -> scanComplete=%d\n", n);
        if (n == WIFI_SCAN_FAILED) {
            options = "<option value=\"\">Recherche en cours...</option>";
        } else if (n > 0) {
            for (int i = 0; i < n; ++i) {
                options += "<option value=\"" + WiFi.SSID(i) + "\">" + WiFi.SSID(i) + " (" + String(WiFi.RSSI(i)) + " dBm)</option>";
            }
        }
        
        String html = String(FPSTR(CAPTIVE_HTML_1)) + options + String(FPSTR(CAPTIVE_HTML_2));
        req->send(200, "text/html", html);
    };

    captiveServer.on("/scan", HTTP_GET, [](AsyncWebServerRequest* req) {
        String options = "";
        int n = WiFi.scanComplete();
        Serial.printf("[CaptivePortal] GET /scan -> scanComplete=%d\n", n);
        if (n == WIFI_SCAN_RUNNING) {
            options = "<option value=\"\">Recherche en cours...</option>";
        } else if (n == WIFI_SCAN_FAILED) {
            options = "<option value=\"\">Erreur scan</option>";
            WiFi.scanNetworks(true);
        } else if (n == 0) {
            options = "<option value=\"\">Aucun r&eacute;seau</option>";
            WiFi.scanNetworks(true);
        } else if (n > 0) {
            for (int i = 0; i < n; ++i) {
                options += "<option value=\"" + WiFi.SSID(i) + "\">" + WiFi.SSID(i) + " (" + String(WiFi.RSSI(i)) + " dBm)</option>";
            }
            WiFi.scanDelete();
            WiFi.scanNetworks(true); // relance pour le prochain refresh
        }
        req->send(200, "text/html", options);
    });
    captiveServer.on("/",                 HTTP_GET,  servePortal);
    captiveServer.on("/hotspot-detect.html", HTTP_GET, servePortal);  // iOS
    captiveServer.on("/generate_204",     HTTP_GET,  servePortal);   // Android
    captiveServer.on("/connecttest.txt",  HTTP_GET,  servePortal);   // Windows
    captiveServer.onNotFound(servePortal);

    // ── Traitement du formulaire POST /save
    captiveServer.on("/save", HTTP_POST, [](AsyncWebServerRequest* req) {
        String newSsid = "";
        String newPass = "";

        if (req->hasParam("ssid", true)) {
            newSsid = req->getParam("ssid", true)->value();
        }
        if (req->hasParam("pass", true)) {
            newPass = req->getParam("pass", true)->value();
        }

        if (newSsid.length() == 0) {
            req->send(400, "text/plain", "SSID vide");
            return;
        }

        // ── Sauvegarder dans NVS (seule opération safe dans un callback async)
        Preferences p2;
        p2.begin("irrigcfg", false);
        p2.putString("ssid",  newSsid);
        p2.putString("wpass", newPass);
        p2.end();
        Serial.printf("[CaptivePortal] Sauvegarde NVS SSID=%s\n", newSsid.c_str());

        // ── Afficher sur OLED AVANT que le WiFi se coupe
        display.clear();
        display.setFont(ArialMT_Plain_10);
        display.drawString(0,  0, "Params sauvegardes!");
        display.drawString(0, 14, newSsid);
        display.drawString(0, 30, "Redemarrage 2s...");
        display.display();

        // ── Répondre au navigateur (page de confirmation)
        req->send(200, "text/html",
            "<html><head><meta charset='UTF-8'/>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'/>"
            "</head><body style='background:#0d1117;color:#e6edf3;font-family:system-ui;"
            "display:flex;align-items:center;justify-content:center;min-height:100vh;margin:0'>"
            "<div style='text-align:center;padding:24px'>"
            "<div style='font-size:3rem'>&#10003;</div>"
            "<h2 style='color:#2ea043;margin:8px 0'>Enregistr&eacute;!</h2>"
            "<p style='color:#8b949e'>SSID: <strong style='color:#e6edf3'>" + newSsid + "</strong></p>"
            "<p style='color:#8b949e'>Red&eacute;marrage dans 2 secondes…</p>"
            "</div></body></html>");

        // ── Poser le flag : loop() fera ESP.restart() hors du callback
        pendingRestart = true;
        pendingRestartMs = millis();
    });

    captiveServer.begin();
    Serial.println("[CaptivePortal] Serveur HTTP portail démarré");
}

// ============================================================
// SECTION 15 — SETUP
// ============================================================

void setup(){
    Serial.begin(115200);
    Serial.println("\n=== IrrigPro v" SOFT_REV " ===");

    sensors.begin();
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // ── Init OLED
    pinMode(RST_OLED, OUTPUT);
    digitalWrite(RST_OLED, LOW); delay(50);
    digitalWrite(RST_OLED, HIGH);
    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);
    display.clear();
    display.drawString(0,0,"IrrigPro v" SOFT_REV);
    display.drawString(0,14,"Démarrage...");
    display.display();

    // ── Charger config NVS
    configLoad();
    schedLoad();

    // ── Init vannes — FERMETURE SÉCURITÉ au boot
    for(int i=0;i<VANNE_COUNT;i++){
        pinMode(VANNE_PINS[i], OUTPUT);
        digitalWrite(VANNE_PINS[i], LOW);
    }
    // Init LEDVISU pins (mirror of valve outputs)
    for(int i=0;i<VANNE_COUNT;i++){
        pinMode(LEDVISU_PINS[i], OUTPUT);
        digitalWrite(LEDVISU_PINS[i], LOW);
    }
    // Init generic OUT_PINS (sorties auxiliaires)
    for(int i=0;i<(int)(sizeof(OUT_PINS)/sizeof(OUT_PINS[0])); i++){
        pinMode(OUT_PINS[i], OUTPUT);
        digitalWrite(OUT_PINS[i], LOW);
        Serial.printf("[PIN INIT] OUT %d set OUTPUT LOW\n", OUT_PINS[i]);
    }
    logSys("Boot — toutes vannes fermées");

    // ── Init entrées forçage (toutes les broches listées)
    for(int i=0;i<INPUTCOUNT;i++){
        int p = FORCE_INPUT_PINS[i];
        pinMode(p, INPUT_PULLUP);
        Serial.printf("[PIN INIT] IN %d set INPUT_PULLUP\n", p);
    }

    // ── Init LoRa
    int loraState = radio.begin(
        sysConfig.loraFreq, LORA_BW, LORA_SF, LORA_CR,
        LORA_SYNCWORD, sysConfig.loraPower, LORA_PREAMBLE
    );
    radio.setDio1Action(loraSetFlag);
    if(loraState == RADIOLIB_ERR_NONE){
        radio.startReceive();
        Serial.println("LoRa OK");
        logSys("LoRa initialisé");
    } else {
        Serial.printf("LoRa FAIL code=%d\n", loraState);
        logSys("LoRa ERREUR init");
    }

    // ── WiFi (indiquer le SSID en cours de test)
    display.drawString(0,28, "WiFi: " + String(sysConfig.ssid));
    display.display();
    WiFi.begin(sysConfig.ssid, sysConfig.wifiPass);
    for(int t=0;t<20;t++){
        if(WiFi.status()==WL_CONNECTED) break;
        delay(500);
        Serial.print(".");
    }
    if(WiFi.status()==WL_CONNECTED){
        Serial.println("\nWiFi OK: "+WiFi.localIP().toString());
        logSys(("WiFi connecté: "+WiFi.localIP().toString()).c_str());
        // ── NTP
        timeInit();
        // Afficher l'adresse IP sur l'OLED au démarrage (affichage bref)
        {
            String ipAd = WiFi.localIP().toString();
            display.clear();
            display.drawString(0, 0, "IP "+ipAd);
            display.display();
            delay(3000);
        }
    } else {
        Serial.println("\nWiFi FAIL — lancement portail captif");
        logSys("WiFi FAIL — portail captif actif");
        startCaptivePortal();
        // En mode portail captif : NE PAS démarrer OTA, webSetup, ni le watchdog
        // Le loop() prend le relais avec dnsServer.processNextRequest()
        return;
    }

    // ── OTA
    otaSetup();

    // ── Web
    webSetup();

    // ── Watchdog
    esp_task_wdt_init(60, true);
    esp_task_wdt_add(nullptr);

    bootMs = millis();
    logSys("Système prêt");

    display.clear();
    display.drawString(0,0,"IrrigPro prêt");
    display.display();
}

// ============================================================
// SECTION 16 — LOOP (entièrement non-bloquant, sans delay())
// ============================================================

void loop(){
    // ── Bouton pour changer de page OLED
    if (digitalRead(BUTTON_PIN) == LOW) {
        if (millis() - lastButtonPress > 300) { // debounce
            oledPage = (oledPage + 1) % 4; // add dedicated IO page
            lastButtonPress = millis();
            lastOledMs = 0; // force refresh
        }
    }

    // ── Lecture températures (toutes les 10s)
    static unsigned long lastTempRead = 0;
    if (millis() - lastTempRead > 10000) {
        lastTempRead = millis();
        sensors.requestTemperatures();
        temperature1 = sensors.getTempCByIndex(0);
        temperature2 = sensors.getTempCByIndex(1);
    }

    // ── Portail captif actif : traiter uniquement DNS + WDT, pas le reste
    if (captivePortalActive) {
        if (pendingRestart) {
            if (millis() - pendingRestartMs > 2000) {
                Serial.println("[CaptivePortal] Redémarrage de l'ESP...");
                ESP.restart();
            }
        }
        dnsServer.processNextRequest();
        unsigned long now2 = millis();
        if(now2 - lastWdtMs >= 5000){ lastWdtMs = now2; esp_task_wdt_reset(); }
        
        if (!pendingRestart) {
            oledUpdate();
        }
        return;
    }

    // ── OTA
    ArduinoOTA.handle();

    // ── LoRa RX
    loraRxProcess();

    // ── LoRa TX périodique STATUS
    loraTxUpdate();

    // ── Entrées forçage manuel
    inputUpdate();

    // ── Timers vannes (fermeture auto)
    valveUpdate();

    // ── Vérification programmes (1×/min)
    schedCheck();

    // ── WebSocket broadcast état (1×/s max)
    wsBroadcastStatus();

    // ── WebSocket cleanup
    ws.cleanupClients();

    // ── OLED (2×/s)
    oledUpdate();

    // ── Watchdog reset
    unsigned long now = millis();
    if(now - lastWdtMs >= 5000){
        lastWdtMs = now;
        esp_task_wdt_reset();
    }

    // ── Gestion WiFi / portail captif (timeout 1min) + reconnexions périodiques
    static unsigned long lastWifiCheckMs = 0;
    if(now - lastWifiCheckMs > WIFI_RECONNECT_INTERVAL_MS){
        lastWifiCheckMs = now;
        if(WiFi.status() != WL_CONNECTED){
            if(!captivePortalActive){
                Serial.println("WiFi perdu — démarrage portail captif (1min)");
                WiFi.softAP(sysConfig.nodeId); // démarre l'AP pour configuration
                captivePortalActive = true;
                captivePortalStartMs = now;
                Serial.print("AP IP: "); Serial.println(WiFi.softAPIP().toString());
            } else {
                // Portail actif : vérifier timeout
                if(now - captivePortalStartMs >= CAPTIVE_PORTAL_TIMEOUT_MS){
                    Serial.println("Portail captif timeout — arrêt du portail, reprise irrigation et tentatives WiFi en arrière-plan");
                    WiFi.softAPdisconnect(true);
                    captivePortalActive = false;
                    // tenter reconnexion immédiatement aux anciens paramètres
                    Serial.println("Tentative reconnexion WiFi aux anciens paramètres...");
                    WiFi.begin(sysConfig.ssid, sysConfig.wifiPass);
                    lastWifiReconnectMs = now;
                }
            }
        } else {
            // WiFi rétabli : s'assurer que le portail est arrêté
            if(captivePortalActive){
                Serial.println("WiFi rétabli — arrêt portail captif");
                WiFi.softAPdisconnect(true);
                captivePortalActive = false;
            }
        }
    }

    // Tentatives périodiques de reconnexion si déconnecté et pas en portail
    if(WiFi.status() != WL_CONNECTED && !captivePortalActive){
        if(now - lastWifiReconnectMs >= WIFI_RECONNECT_INTERVAL_MS){
            lastWifiReconnectMs = now;
            Serial.println("Tentative périodique de reconnexion WiFi...");
            WiFi.begin(sysConfig.ssid, sysConfig.wifiPass);
        }
    }
}

// ============================================================
// FIN MainIocan.cpp
// ============================================================











#endif