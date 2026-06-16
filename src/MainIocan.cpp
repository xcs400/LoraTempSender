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

#include "WebContent.h"   // SPA HTML — seul fichier séparé

// ============================================================
// SECTION 1 — CONSTANTES & PINS
// ============================================================
#define NODE_ID_DEFAULT      "IRRIGATION01"
#define SOFT_REV             "2.0"
#define OTA_HOSTNAME         "esp32-irrigation"
#define OTA_PASSWORD         "irrigation2024"

// Vannes — 8 sorties (adapter aux broches réelles)
static const int VANNE_COUNT       = 4;
static const int VANNE_PINS[VANNE_COUNT]     = {47, 46, 45, 42};
//static const int VANNE_PINS[VANNE_COUNT]     = {47, 46, 45, 42, 41, 40, 39, 38};

// Entrées forçage manuel (INPUT_PULLUP actif bas)
static const int FORCE_INPUT_PINS[VANNE_COUNT] = {4, 5, 6, 7};
//static const int FORCE_INPUT_PINS[VANNE_COUNT] = {4, 5, 6, 7, 15, 16, 17, 18};

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
    uint8_t seasonStartMonth = 4;
    uint8_t seasonStartDay   = 1;
    uint8_t seasonEndMonth   = 10;
    uint8_t seasonEndDay     = 31;
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
    } else {
        logSys("NTP échec (sera retenté)");
    }
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
                    trigger = (s.intervalDays>0) && ((yday % s.intervalDays)==0);
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
            // Début appui
            inputPressMs[i] = now;
            inputActive[i]  = true;
        }
        else if(!pressed && inputActive[i]){
            // Relâchement
            unsigned long dur = now - inputPressMs[i];
            inputActive[i] = false;
            if(dur >= LONG_PRESS_MS){
                // Appui long → fermer
                valveClose(i, CmdSource::PHYS_INPUT);
            } else {
                // Appui court → ouvrir (durée config)
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
String schedulesToJson(){
    StaticJsonDocument<4096> doc;
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
            o["weekDays"]         = s.weekDays;
            o["calMode"]          = s.calMode;
            o["intervalDays"]     = s.intervalDays;
            o["seasonStartMonth"] = s.seasonStartMonth;
            o["seasonStartDay"]   = s.seasonStartDay;
            o["seasonEndMonth"]   = s.seasonEndMonth;
            o["seasonEndDay"]     = s.seasonEndDay;
        }
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
    server.on("/api/schedule/save", HTTP_POST, [](AsyncWebServerRequest* req){},
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
            StaticJsonDocument<512> doc;
            if(deserializeJson(doc,data,len)){jsonResp(req,"{\"ok\":false}",400);return;}
            int v   = doc["valve"]    | -1;
            int idx = doc["schedIdx"] | -1;
            if(v<0||v>=VANNE_COUNT){jsonResp(req,"{\"ok\":false}",400);return;}
            // Chercher slot libre si idx==-1
            if(idx<0){
                for(int p=0;p<MAX_PROGRAMS;p++){
                    if(!valves[v].schedules[p].active){ idx=p; break; }
                }
                if(idx<0){jsonResp(req,"{\"ok\":false,\"reason\":\"full\"}");return;}
            }
            if(idx>=MAX_PROGRAMS){jsonResp(req,"{\"ok\":false}",400);return;}
            Schedule& s = valves[v].schedules[idx];
            s.active           = doc["active"]           | true;
            s.hour             = doc["hour"]             | 6;
            s.minute           = doc["minute"]           | 0;
            s.durationSec      = doc["durationSec"]      | 900;
            s.weekDays         = doc["weekDays"]         | 0b0111111;
            s.calMode          = doc["calMode"]          | 0;
            s.intervalDays     = doc["intervalDays"]     | 2;
            s.seasonStartMonth = doc["seasonStartMonth"] | 4;
            s.seasonStartDay   = doc["seasonStartDay"]   | 1;
            s.seasonEndMonth   = doc["seasonEndMonth"]   | 10;
            s.seasonEndDay     = doc["seasonEndDay"]     | 31;
            schedSave();
            jsonResp(req,"{\"ok\":true}");
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
    if(now - lastOledMs < 2000) return;
    lastOledMs = now;

    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);

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
    // Ligne 5: LoRa info
    display.drawString(0,65,"LoRa rx:"+String(loraRxCount)+" rssi:"+String((int)loraRssi));
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
// SECTION 15 — SETUP
// ============================================================

void setup(){
    Serial.begin(115200);
    Serial.println("\n=== IrrigPro v" SOFT_REV " ===");

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
    logSys("Boot — toutes vannes fermées");

    // ── Init entrées forçage
    for(int i=0;i<VANNE_COUNT;i++)
        pinMode(FORCE_INPUT_PINS[i], INPUT_PULLUP);

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

    // ── WiFi
    display.drawString(0,28,"WiFi...");
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
        Serial.println("\nWiFi FAIL — mode offline");
        logSys("WiFi non connecté — offline");
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

    // ── Reconnexion WiFi si perdu
    static unsigned long lastWifiCheckMs = 0;
    if(now - lastWifiCheckMs > 30000){
        lastWifiCheckMs = now;
        if(WiFi.status() != WL_CONNECTED){
            Serial.println("WiFi perdu, reconnexion...");
            WiFi.reconnect();
        }
    }
}

// ============================================================
// FIN MainIocan.cpp
// ============================================================











#endif
