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
// Initialisation du watchdog MQTT et du compteur d'échecs à 0. Le watchdog
// est armé par onMqttConnect() (mets à jour mqttLastActivityMs) ; tant
// qu'aucune connexion n'a eu lieu, il reste à 0 et n'a aucun effet.
unsigned long   mqttLastActivityMs        = 0;
uint8_t         mqttConsecutiveFailures   = 0;
unsigned long   mqttDisconnectMs          = 0;
unsigned long   lastMqttForceReconnectMs  = 0;
// Récupération MQTT au boot (mode CONS_MQTT_ONLY) : fenêtre de 3s après
// connexion pour récupérer les valeurs retained et mettre à jour la RAM.
// mqttRecoveryDone passe à true à l'expiration de la fenêtre dans mqttLoop().
#ifdef CONS_MQTT_ONLY
bool            mqttRecoveryDone    = false;
unsigned long   mqttRecoveryStartMs = 0;
#endif

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
// Initialisation : on part des coefficients de calibration mesurés
// (FLOW_COEFF_DEFAULTS, voir Globals.h) à la place de la valeur 1.0f
// codée en dur dans la struct. Sans ça, tant que valveConsLoad() n'a pas
// écrasé les valeurs (mode normal) ou tant que la recovery MQTT n'a pas
// livré les valeurs retained (mode CONS_MQTT_ONLY), la répartition des
// pulses se ferait en parts ÉGALES — alors qu'on a des coefficients
// réels mesurés pour cette installation.
//
// Le tableau valveCons[] est défini tel quel (l'initializer `= 1.0f` du
// membre flowCoeff s'applique à chaque instance). L'application des
// défauts FLOW_COEFF_DEFAULTS est faite dans setup() (voir
// MainIocan_S.cpp) AVANT valveConsLoad() et avant tout pulseDistribute(),
// en suivant l'ordre logique d'initialisation des compteurs.
ValveCons valveCons[VANNE_COUNT];
unsigned long lastDistributedTotal = 0;
// Drity flags pour throttling d'écriture NVS (cf. ConfigManager.h)
volatile bool valveConsDirty[VANNE_COUNT] = {false};

// Consommation "Vanne manuelle" (voir ManualValveState / Globals.h).
// Initialisée à zéro ici ; les valeurs NVS sont restaurées par
// manualValveLoad() dans setup() (avant tout pulseDistribute()).
ManualValveState manualValveState;
volatile bool     manualValveDirty = false;

// Calibration débit
CalibState calibState;

// Alarmes hydrauliques (voir AlarmManager.h)
// Tous les champs sont à zéro par défaut grâce aux initializers de la
// struct. `lastCloseMs` est positionné au boot pour que le délai de grâce
// post-fermeture soit écoulé dès la première seconde de loop() (sinon on
// déclencherait une fausse alarme UNEXPECTED_FLOW au démarrage si le
// capteur a déjà vu passer de l'eau avant reboot).
AlarmState alarmState;

// Journal circulaire
LogEntry  logBuf[LOG_MAX];
uint16_t  logHead  = 0;
uint16_t  logCount = 0;

// LoRa
volatile bool loraRxFlag = false;
volatile bool loraTxFlag = false;     // IRQ DIO1 = fin de TX
volatile uint8_t loraMode = 0;        // 0 = idle/RX, 1 = TX en cours
unsigned long lastLoraTx = 0;
int           loraRxCount = 0;
int           loraTxCount = 0;
float         loraRssi = 0;

// ISR DIO1 LoRa — on ne fait que poser un flag, le traitement se fait
// dans la loop() (loraRxProcess). Doit résider en IRAM pour ESP32/Xtensa.
void IRAM_ATTR loraSetFlag(){
    if(loraMode) loraTxFlag = true;   // fin de TX
    else         loraRxFlag = true;   // paquet RX reçu
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
int oledPreferredPage = 2;
unsigned long lastButtonPress = 0;
unsigned long oledLastActivityMs = 0;
unsigned long oledFlowPageStartPulseCount = 0;

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

// Historique des consommations (7 jours)
HistoryData historyData;

#endif // IOCAN
