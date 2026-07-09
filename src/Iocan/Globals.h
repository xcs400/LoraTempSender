#pragma once
#ifdef IOCAN
// ============================================================
// Globals.h — Constantes, structures & variables globales IOCAN
// ============================================================
// Ce fichier centralise tout ce qui est partagé entre les modules
// (pins, structs métier, variables d'état). Les variables sont
// déclarées ici avec `extern` et définies une seule fois dans
// Globals.cpp, pour éviter les doublons à l'édition de liens tout
// en gardant tous les modules suivants en "header-only" (faciles à
// inclure depuis MainIocan.cpp sans toucher à platformio.ini).
//
// Découpage d'origine : tout vivait dans un unique MainIocan.cpp.
// Ce fichier correspond aux anciennes SECTION 1, 2 et 3 (constantes,
// pins, types/structures, variables globales).
// ============================================================

#include <Arduino.h>
#include <SSD1306.h>
#include <RadioLib.h>
#include <Preferences.h>
#include <ESPAsyncWebServer.h>
#include <AsyncMqttClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DNSServer.h>

// ============================================================
// SECTION 1 — CONSTANTES & PINS
// ============================================================
#define NODE_ID_DEFAULT      "IRRIGATION01"
#define SOFT_REV             "2.3"
#define OTA_HOSTNAME         "esp32-irrigation"
#define OTA_PASSWORD         "irrigation2024"

// ENTREE SORTIE
static const int VANNE_COUNT       = 5;
static const int VANNE_PINS[5]     = {3,     //PD0
                                      2,     //PD1
                                      1,     //PD2
                                      38,    //PD3
                                      39,    //PD4
                                        };

static const int OUT_PINS[2]= {
                                      41,    //PD6
                                      42     //PD7
                                       };

static const int LEDVISU_PINS[5]   = {48, //PA0-LED
                                      46, //PA1-LED
                                      45, //PA2-LED
                                      37,//PA3-LED
                                      40 }; //Pd5-LED

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

// === Compteur d'impulsions (flow meter) connecté sur FORCE_INPUT_PINS[7]
#define PULSE_DEBOUNCE_US 2000UL  // 2ms debounce
#define PULSES_PER_LITRE 741.2f   //  sans calibre theorique:660.0f     // constante par défaut (ajuster selon capteur) 
  
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
 * J3-10: GPIO39 MTCK            [PD4-VANNE]    J2-10 : GPIO35 | FSPID  | LEDBlanche  {PB3-BP}
 * J3-09: GPIO40 MTDO            [PD5-LED]       J2-09 : GPIO36 | SPIIO7 | VEXT_CTL    {PB4-BP}
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
 * OLED_SDA -> GPIO17
 * OLED_SCL -> GPIO18
 * OLED_RST -> GPIO21
 * ---------------------------------------------------------------------------
 * Module LoRa SX127x intégré
 * ---------------------------------------------------------------------------
 * LoRa_NSS   -> GPIO8
 * LoRa_SCK   -> GPIO9
 * LoRa_MOSI  -> GPIO10
 * LoRa_MISO  -> GPIO11
 * LoRa_RST   -> GPIO12
 * LoRa_BUSY  -> GPIO13
 * LoRa_DIO1  -> GPIO14
 * ---------------------------------------------------------------------------
 * SPI Flash / FSPI
 * ---------------------------------------------------------------------------
 * FSPIWP     -> GPIO38
 * FSPID      -> GPIO35
 * FSPIIO4    -> GPIO33
 * FSPIIO5    -> GPIO34
 * FSPIIO7    -> GPIO36
 * FSPICS0    -> GPIO34
 * FSPICS1    -> GPIO26
 * FSPICLK    -> GPIO37
 * ---------------------------------------------------------------------------
 * UART
 * ---------------------------------------------------------------------------
 * UART0_TX   -> GPIO43
 * UART0_RX   -> GPIO44
 * UART1_RTS  -> GPIO19
 * UART1_CTS  -> GPIO20
 * ---------------------------------------------------------------------------
 * Alimentation
 * ---------------------------------------------------------------------------
 * 5V         -> J2-02
 * 3.3V       -> J3-02, J3-03
 * GND        -> J2-01, J3-01
 *****************************************************************************/

// Bouton pour l'affichage (BOOT = GPIO 0)
const int BUTTON_PIN = 0;

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

// Débitmètre lissé (voir FlowMeter.h)
#define FLOW_WINDOW_MS  4000UL    // fenêtre de calcul = 4 secondes
#define FLOW_SAMPLES    4         // 4 échantillons × 1s = 4s d'historique
#define OLED_SCREENSAVER_TIMEOUT_MS 60000UL // 1 minutes avant veille OLED

// Consommation par vanne (voir ValveCons.h)
#define CONS_HISTORY_DAYS 14
#define SAVE_LITRES_STEP 100.0f

// ── Coefficients de calibration débit par défaut (pulses/s mesurés SEUL) ─────
//
// Valeurs mesurées par calibration vanne par vanne, chacune ouverte seule
// pendant la durée standard (cf. calibStart() dans ValveCons.h). Tant que
// l'utilisateur n'a pas lancé /api/calibration/start, ces coefficients
// servent de défaut à la place de 1.0f (répartition égale).
//
// Référence : calibration du 06/07/2026 sur l'installation réelle :
 //   73.122f,   // V0
 //   59.233f,   // V1
 //   39.500f,   // V2
 //  116.011f,   // V3
 //   22.122f,   // V4
// Somme = 354.75 pulses/s → la part relative de chaque vanne est
// immédiate (V3 = 37.7%, V0 = 26.0%, etc.).
//
// IMPORTANT : en mode CONS_MQTT_ONLY, ces coefficients restent persistés
// en NVS (valveConsSaveFlowCoeff) car ce sont des DONNÉES DE CONFIGURATION
// (pas des compteurs). Si l'utilisateur relance une calibration, la valeur
// mesurée écrase le défaut pour cette vanne.
static const float FLOW_COEFF_DEFAULTS[VANNE_COUNT] = {
    73.122f,   // V0
    59.233f,   // V1
    39.500f,   // V2
   116.011f,   // V3
    22.122f,   // V4
};

// ── Stratégie de persistance NVS pour la consommation des vannes ──────────────
//
// Par défaut (CONS_MQTT_ONLY non défini), la consommation est flushée en NVS
// toutes les 30 secondes par valveConsFlushDirty() dans loop(). Avec 5 vannes
// et ~17 clés par vanne, cela représente jusqu'à ~85 écritures NVS/min pendant
// l'irrigation — ce qui use rapidement la flash (endurance ~100 000 cycles
// par secteur NVS).
//
// Mode CONS_MQTT_ONLY (décommenter pour activer) :
//   * Le flush NVS périodique toutes les 30s est DÉSACTIVÉ.
//   * PLUS IMPORTANT : les compteurs `pulse_total` (ConfigManager.h::pulseSave
//     et pulseLoad) ET les compteurs par vanne (valveConsSaveOne et
//     valveConsLoad) sont des NO-OPS — RIEN n'est persisté en NVS.
//   * Seules les données de CONFIGURATION (SSID, schedules, coefficients
//     de calibration) restent persistées en NVS, via les namespaces
//     "irrigcfg" et "schedcfg". Le namespace "irrcons" (conso par vanne)
//     n'est plus utilisé du tout en CONS_MQTT_ONLY.
//   * Les compteurs vivent uniquement en RAM. Au reboot, ils sont à zéro
//     puis ré-hydratés par la recovery MQTT (MqttManager.h), qui lit les
//     valeurs retained publiées par HA lors de la session précédente.
//   * Les valeurs publiées en MQTT (retained) sur le broker font office de
//     mémoire secondaire pour Home Assistant. Les données "en cours
//     d'arrosage" (non encore publiées) ne survivent PAS à un reboot brutal
//     sans flushing MQTT périodique (10s par défaut) — perte max = 10s de
//     conso sur les vannes ouvertes. Acceptable si le broker MQTT est fiable.
//   * L'historique journalier (14 jours) N'EST PAS conservé entre reboots
//     en mode CONS_MQTT_ONLY (les compteurs repartent de 0). Les données
//     historiques restent consultables via l'historique long-terme de HA.
//
// MOTIF (anti-empoisonnement) : avant ce mode, le firmware chargeait la
// valeur NVS de `pulse_total` au boot puis écrasait la NVS avec la valeur
// MQTT si cette dernière était "plus grande" (logique binaire). Si HA
// retenait une valeur buggée (4 294 957 568 ≈ 4.29G observée), la NVS
// était ré-écrite avec cette valeur à chaque reboot, puis le firmware
// republiait 4.29G en MQTT retained → cycle vicieux d'auto-empoisonnement.
// En CONS_MQTT_ONLY, la NVS n'est plus jamais lue/écrite pour les compteurs,
// donc impossible de ré-injecter la valeur buggée depuis la NVS. La
// recovery MQTT (rejet > 4.29G) reste en place comme garde-fou.
//
// DÉCOMMENTER la ligne suivante pour activer le mode économie flash :
#define CONS_MQTT_ONLY  1

// ============================================================
// SECTION 2 — TYPES & STRUCTURES
// ============================================================

enum class CmdSource : uint8_t { NONE=0, AUTO=1, WEB=2, PHYS_INPUT=3, LORA=4 };

inline const char* srcStr(CmdSource s){
    switch(s){
        case CmdSource::AUTO:      return "AUTO";
        case CmdSource::WEB:       return "WEB";
        case CmdSource::PHYS_INPUT:return "INPUT";
        case CmdSource::LORA:      return "LORA";
        default:                   return "NONE";
    }
}
inline int srcPrio(CmdSource s){
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
    uint8_t weekDays      = 0b0111111;  // 6 bits actifs = Lun à Sam (pas Lun-Ven)
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

    Valve(){ snprintf(name,24,"V0"); }
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
    int32_t tzOffset      = 3600;         // UTC+1 hiver (conservé pour compatibilité UI)
    // Chaîne POSIX TZ — gère automatiquement heure été/hiver (DST).
    // France : CET-1CEST,M3.5.0,M10.5.0/3
    // Autres : voir https://github.com/nayarsystems/posix_tz_db
    // Si vide, on retombe sur configTime(tzOffset, 0, ...) sans DST.
    char    tzPosix[48]   = "CET-1CEST,M3.5.0,M10.5.0/3";
    char    ntpServer[48] = "pool.ntp.org";
    uint8_t irrigMode     = MODE_PARALLEL;
    uint32_t maxOpenSec   = 3600;
    uint16_t manualForceSec = FORCE_MANUAL_DUR_S;
    // ── MQTT / Home Assistant
    bool    mqttEnabled   = true;
    char    mqttHost[64]  = "192.168.1.70";
    uint16_t mqttPort     = 1883;
    char    mqttUser[32]  = "mosquitouser";
    char    mqttPass[48]  = "expresso";
    char    mqttPrefix[32]= "homeassistant";   // base du topic discovery
    char    mqttId[32]    = "irrpro_hs3";      // unique_id pour HA
};

// --- Consommation par vanne ---
struct DayStat {
    uint32_t ymd;       // ex: 20260624 (YYYYMMDD — nécessite uint32_t, max uint16_t=65535)
    uint32_t pulses;    // pulses attribués ce jour-là
    float    litres;    // = pulses / PULSES_PER_LITRE
};

struct ValveCons {
    unsigned long pulsesTotal = 0;   // total cumulé (persisté)
    uint32_t todayYmd = 0;           // YYYYMMDD (nécessite uint32_t, max uint16_t=65535)
    uint32_t todayPulses = 0;
    uint16_t todayIdx = 0;           // index d'écriture dans history (anneau)
    DayStat history[CONS_HISTORY_DAYS];
    // Coefficient relatif de débit (mesuré seul pendant calibration). La
    // valeur par défaut est surchargée dans Globals.cpp à partir de
    // FLOW_COEFF_DEFAULTS (calibration du 06/07/2026). Si l'utilisateur
    // n'a jamais calibré, la répartition utilise ces coefficients mesurés
    // au lieu de 1.0f (répartition égale) — bien plus juste dès le boot.
    float flowCoeff = 1.0f;
    // Résidu fractionnaire accumulé d'un appel de pulseDistribute() à l'autre
    // (voir ValveCons.h::pulseDistribute() pour le détail de l'algorithme).
    float carry = 0.0f;

    float instantFlowLpm = 0.0f;

};

// --- Alarmes hydrauliques (voir AlarmManager.h) ---
//
// Struct + extern ci-dessous ; les définitions vivent dans Globals.cpp.
// On centralise l'état pour que MQTT, LoRa, WebSocket et l'API REST
// lisent la MÊME source de vérité (cf. AlarmManager.h::alarmTick()).
struct AlarmState {
    uint8_t       code       = 0;       // 0=OK, 1=NO_FLOW, 2=UNEXPECTED_FLOW
    bool          active     = false;
    unsigned long sinceMs    = 0;       // millis() du passage en alarme
    unsigned long openSinceMs = 0;      // millis() de la dernière ouverture vanne (pour délai grâce)
    unsigned long lastCloseMs = 0;      // millis() de la dernière fermeture (idem, anti-fantômes)
    char          msg[80]    = "";      // libellé court pour logs / LoRa
};

// --- Calibration débit ---
enum class CalibPhase : uint8_t { IDLE=0, RUNNING=1, DONE=2, ABORTED=3, FAILED=4 };

struct CalibState {
    CalibPhase phase = CalibPhase::IDLE;
    int        currentValve = -1;       // vanne en cours de mesure (-1 = aucune)
    uint16_t   durationSec  = 60;        // durée de mesure par vanne, configurable UI
    unsigned long phaseStartMs = 0;      // millis() du début de la mesure en cours
    unsigned long pulseSnapshotAtStart = 0; // total pulses au début de la mesure en cours
    float      resultCoeff[VANNE_COUNT] = {0}; // pulses/seconde mesurés par vanne (résultat brut)
    char       failReason[48] = "";      // raison d'échec/abandon, pour affichage UI
};

// ============================================================
// SECTION 3 — VARIABLES GLOBALES (déclarations extern)
// ============================================================

// Hardware
extern SSD1306  display;
extern SX1262   radio;
extern Preferences prefs;
extern OneWire oneWire;
extern DallasTemperature sensors;

// Serveur web & WebSocket
extern AsyncWebServer  server;
extern AsyncWebSocket  ws;

// ── MQTT / Home Assistant
extern AsyncMqttClient mqttClient;
extern bool            mqttConnected;
extern unsigned long   lastMqttPubMs;
extern unsigned long   lastMqttConnectAttemptMs;
// Watchdog MQTT : timestamp de la dernière activité (connect/ping/pong/pub/sub/receive).
// Si on dépasse MQTT_WATCHDOG_MS (120s = 2 × keepalive) sans activité alors qu'on
// était connecté, on force une déconnexion/reconnexion. Corrige le cas où
// AsyncTCP perd un paquet keepalive pendant la nuit (routeur qui a oublié
// l'association STA, coupure RF brève, etc.) et n'envoie jamais
// onMqttDisconnect() → mqttConnected reste à true mais aucun message ne passe
// → HA affiche l'appareil "online" mais ne reçoit plus rien (et inversement
// côté ESP, on continue à croire qu'on est connecté).
extern unsigned long   mqttLastActivityMs;
// Compteur d'échecs CONSÉCUTIFS de connect() — utilisé pour le backoff
// exponentiel (5s, 10s, 20s, 40s, 60s max) afin de ne pas marteler un
// broker qui ne répond plus, tout en restant réactif quand le réseau
// revient. Remis à 0 dans onMqttConnect() et dans mqttLoop() quand le
// WiFi est KO (on ne doit pas pénaliser un échec de cause WiFi).
extern uint8_t        mqttConsecutiveFailures;
// Timestamp de la dernière déconnexion MQTT (millis()). Mis à 0 par
// onMqttDisconnect() et lu par onMqttConnect() pour calculer le downtime
// et l'afficher dans la trace logSys de la reconnexion. Si = 0, on n'a
// jamais été déconnecté depuis le boot (cas du premier connect).
extern unsigned long   mqttDisconnectMs;
const unsigned long MQTT_PUB_INTERVAL_MS    = 10000UL;  // 10 s
const unsigned long MQTT_RECONNECT_MS       = 15000UL;  // 15 s entre tentatives (throttle simple)
const unsigned long MQTT_WATCHDOG_MS        = 120000UL; // 2 min sans activité → reconnexion forcée
const unsigned long MQTT_BACKOFF_MIN_MS     = 5000UL;   // 5 s avant retry après 1er échec
const unsigned long MQTT_BACKOFF_MAX_MS     = 60000UL;  // 60 s max entre retries (backoff)
// Purge périodique de la connexion MQTT (filet de sécurité contre les
// sessions TCP zombies que ni le watchdog d'inactivité ni le keepalive
// MQTT ne détectent — voir commentaire dans mqttLoop()).
// 10 min = 600 s : suffisamment long pour ne pas générer de churn
// réseau visible côté HA, suffisamment court pour purger un socket
// bloqué silencieusement par AsyncTCP/AsyncMqttClient sur ESP32-S3
// (bug connu de certaines versions, socket figée sans FIN/RST).
const unsigned long MQTT_FORCE_RECONNECT_MS = 600000UL; // 10 min
// Timestamp de la dernière purge/reconnexion forcée périodique.
// Initialisé à 0 dans Globals.cpp ; armé dans mqttLoop() à chaque
// passage où on déclenche la purge.
extern unsigned long   lastMqttForceReconnectMs;
// Récupération MQTT au boot (mode CONS_MQTT_ONLY)
// Voir MqttManager.h pour la logique et Globals.cpp pour les définitions.
#ifdef CONS_MQTT_ONLY
extern bool            mqttRecoveryDone;
extern unsigned long   mqttRecoveryStartMs;
#endif

// Vannes
extern Valve  valves[];
extern SysConfig sysConfig;

// Compteur d'impulsions (flow meter)
extern volatile unsigned long pulseCount;
extern volatile unsigned long lastPulseUs;
extern unsigned long persistedPulseCount;
void IRAM_ATTR pulse_isr(); // définie dans Globals.cpp — ISR du compteur d'impulsions

// Consommation par vanne
extern ValveCons valveCons[];
extern unsigned long lastDistributedTotal;
// Marquage NVS "dirty" par vanne — cf. ConfigManager.h pour le rationnel.
// valveConsDirty[v] = true signifie que la conso de la vanne v en RAM
// n'a pas encore été flushée en NVS. Flush déclenché toutes les 30 s par
// loop() (ou immédiatement sur transition d'état / reset).
extern volatile bool valveConsDirty[];

// Applique les coefficients de calibration par défaut (mesurés le
// 06/07/2026 sur l'installation réelle, voir FLOW_COEFF_DEFAULTS plus
// haut) à valveCons[]. À appeler dans setup() AVANT valveConsLoad() pour
// que la NVS ait la priorité si elle contient déjà une calibration
// utilisateur (mode normal), ou pour initialiser en mode CONS_MQTT_ONLY
// (où flowCoeff reste la seule chose lue en NVS — les compteurs sont
// ré-hydratés par la recovery MQTT).
inline void applyFlowCoeffDefaults(){
    for(int i=0;i<VANNE_COUNT;i++){
        valveCons[i].flowCoeff = FLOW_COEFF_DEFAULTS[i];
    }
}

// Calibration débit
extern CalibState calibState;

// Alarmes hydrauliques (voir AlarmManager.h)
extern AlarmState alarmState;

// Journal circulaire
extern LogEntry  logBuf[];
extern uint16_t  logHead;
extern uint16_t  logCount;

// LoRa
extern volatile bool loraRxFlag;
extern volatile bool loraTxFlag;
extern volatile uint8_t loraMode;
extern unsigned long lastLoraTx;
extern int           loraRxCount;
extern int           loraTxCount;
extern float         loraRssi;
// ISR DIO1 du module LoRa — définie dans Globals.cpp (IRAM, ne pas inliner)
void IRAM_ATTR loraSetFlag();

// Temps
extern unsigned long bootMs;
extern bool          timeIsSynced;
extern unsigned long ntpSyncedAtMs;
extern unsigned long lastNtpAttemptMs;

// Entrées manuelles
extern unsigned long inputPressMs[];
extern bool          inputActive[];

// Températures
extern float temperature1;
extern float temperatureRemote;
extern bool  temp1Valid;

// OLED
extern unsigned long lastOledMs;
extern int oledPage;
extern int oledPreferredPage;
extern unsigned long lastButtonPress;
extern unsigned long oledLastActivityMs;
extern unsigned long oledFlowPageStartPulseCount;

// WDT
extern unsigned long lastWdtMs;

// Portail captif
extern bool          captivePortalActive;
extern bool          captivePortalScanAvailable;
extern DNSServer     dnsServer;
extern AsyncWebServer captiveServer;
extern unsigned long captivePortalStartMs;
const unsigned long CAPTIVE_PORTAL_TIMEOUT_MS = 120000UL; // 2 min
extern unsigned long lastWifiReconnectMs;
const unsigned long WIFI_RECONNECT_INTERVAL_MS = 30000UL; // 30 s
extern bool          pendingRestart;
extern unsigned long pendingRestartMs;

#endif // IOCAN
