#ifdef IOCAN
// ============================================================
// Globals.cpp — Définitions des variables globales IOCAN
// ============================================================
// Chaque variable déclarée `extern` dans Globals.h est définie ici,
// une seule fois, pour éviter les erreurs de double-définition à
// l'édition de liens.
// ============================================================

#include "Globals.h"

// Hardware
SSD1306  display(0x3c, SDA_OLED, SCL_OLED);
SX1262   radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
Preferences prefs;
OneWire oneWire(6); // GPIO6 (J3-17) pour le bus OneWire
DallasTemperature sensors(&oneWire);

// Serveur web & WebSocket
AsyncWebServer  server(80);
AsyncWebSocket  ws("/ws");

// ── MQTT / Home Assistant
AsyncMqttClient mqttClient;
bool            mqttConnected = false;
unsigned long   lastMqttPubMs = 0;
unsigned long   lastMqttConnectAttemptMs = 0;

// Vannes
Valve  valves[VANNE_COUNT];
SysConfig sysConfig;

// Compteur d'impulsions (flow meter) connecté sur FORCE_INPUT_PINS[7]
// ISR matériel avec anti-rebond basique (microsecondes)
volatile unsigned long pulseCount = 0;
volatile unsigned long lastPulseUs = 0;
unsigned long persistedPulseCount = 0;

// ISR dédiée (doit être IRAM pour ESP32)
void IRAM_ATTR pulse_isr(){
    unsigned long nowUs = micros();
    if(nowUs - lastPulseUs < PULSE_DEBOUNCE_US) return;
    lastPulseUs = nowUs;
    pulseCount++;
}

// Consommation par vanne
ValveCons valveCons[VANNE_COUNT];
unsigned long lastDistributedTotal = 0;
// Drity flags pour throttling d'écriture NVS (cf. ConfigManager.h)
volatile bool valveConsDirty[VANNE_COUNT] = {false};

// Calibration débit
CalibState calibState;

// Journal circulaire
LogEntry  logBuf[LOG_MAX];
uint16_t  logHead  = 0;
uint16_t  logCount = 0;

// LoRa
volatile bool loraRxFlag = false;
unsigned long lastLoraTx = 0;
int           loraRxCount = 0;
float         loraRssi = 0;

// ISR DIO1 LoRa — on ne fait que poser un flag, le traitement se fait
// dans la loop() (loraRxProcess). Doit résider en IRAM pour ESP32/Xtensa.
void IRAM_ATTR loraSetFlag(){
    loraRxFlag = true;
}

// Temps
unsigned long bootMs       = 0;
bool          timeIsSynced = false;
unsigned long ntpSyncedAtMs = 0;
unsigned long lastNtpAttemptMs = 0;

// Entrées manuelles
unsigned long inputPressMs[VANNE_COUNT]  = {0};
bool          inputActive[VANNE_COUNT]   = {false};

// Températures
float temperature1 = 0;
float temperatureRemote = 0;
bool temp1Valid = false;

// OLED
unsigned long lastOledMs = 0;
int oledPage = 2; // 0=IP, 1=Temp, 2=Vanne (default)
unsigned long lastButtonPress = 0;

// WDT
unsigned long lastWdtMs = 0;

// Portail captif
bool          captivePortalActive = false;
bool          captivePortalScanAvailable = false;
DNSServer     dnsServer;
AsyncWebServer captiveServer(80);
unsigned long captivePortalStartMs = 0;
unsigned long lastWifiReconnectMs = 0;
bool          pendingRestart = false;
unsigned long pendingRestartMs = 0;

#endif // IOCAN
