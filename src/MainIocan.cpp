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
//
// CORRECTIFS MQTT (révision précédente) :
//   1) onMqttConnect() s'abonnait à un topic différent de celui publié
//      dans la discovery (mqttTopicNode() vs mqttTopic()) -> les commandes
//      envoyées depuis Home Assistant n'atteignaient jamais l'ESP32.
//      Corrigé : abonnement calculé exactement comme le command_topic publié.
//   2) flow_lpm republiait toujours 0.0 côté MQTT (calcul dupliqué non
//      partagé). Corrigé : calcul de débit extrait dans computeFlowLpm(),
//      partagé entre buildStatusJson() (WebSocket) et mqttPublishState().
//   3) Ajout d'un Last Will Testament (LWT) sur le topic availability,
//      pour que Home Assistant détecte une perte de connexion brutale
//      (crash / coupure WiFi) et pas seulement une déconnexion propre.
//
// CORRECTIF RÉPARTITION DES PULSES (cette révision) :
//   4) BUG : avec plusieurs vannes ouvertes en parallèle au même flowCoeff
//      (cas par défaut, avant toute calibration), la totalité des pulses
//      du débitmètre partagé était attribuée à la PREMIÈRE vanne ouverte
//      (plus petit index), 0% pour les autres — peu importe la durée
//      d'ouverture simultanée. Symptôme observé sur le terrain : 2 vannes
//      ouvertes ensemble, 273 pulses comptés, 273 attribués à la vanne 0,
//      0 à la vanne 1.
//      CAUSE : pulseDistribute() tronquait la part exacte de chaque vanne
//      à l'entier inférieur, puis distribuait le reliquat d'arrondi (=
//      delta entier quand toutes les parts exactes sont < 1, ce qui arrive
//      très souvent avec un débitmètre à faible résolution) 1 pulse à la
//      fois en suivant l'ORDRE DES INDEX — donc systématiquement vers la
//      vanne 0 dès qu'elle est ouverte, sans jamais laisser sa chance aux
//      autres vannes ouvertes en même temps.
//      CORRECTIF : accumulateur d'erreur persistant ("carry") par vanne,
//      qui survit d'un appel de pulseDistribute() à l'autre (au lieu d'être
//      réinitialisé à chaque tour). Chaque vanne accumule sa part exacte
//      (fractionnaire comprise) dans son carry ; on extrait la partie
//      entière du carry comme part distribuée ce tour-ci. Ainsi, même une
//      vanne dont la part exacte reste < 1 pulse à chaque appel verra son
//      carry grossir progressivement jusqu'à dépasser 1.0 et recevoir enfin
//      sa part — aucune vanne ouverte n'est plus structurellement privée.
//      Le bilan global (somme des parts = pulses réellement comptés) reste
//      garanti exact via un garde-fou dédié. Le champ `carry` est persisté
//      en NVS (comme flowCoeff) pour ne pas réinitialiser la répartition à
//      chaque reboot.
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
#include <nvs_flash.h>
#include <time.h>
#include <esp_task_wdt.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DNSServer.h>
#include <AsyncMqttClient.h>

#include "WebContent.h"   // SPA HTML — seul fichier séparé

// ============================================================
// SECTION 1 — CONSTANTES & PINS
// ============================================================
#define NODE_ID_DEFAULT      "IRRIGATION01"
#define SOFT_REV             "2.3"
#define OTA_HOSTNAME         "esp32-irrigation"
#define OTA_PASSWORD         "irrigation2024"



// Capteurs de température
const int oneWireBus = 6; // GPIO6 (J3-17) pour le bus OneWire
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);


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
// ISR matériel avec anti-rebond basique (microsecondes)
volatile unsigned long pulseCount = 0;
volatile unsigned long lastPulseUs = 0;
#define PULSE_DEBOUNCE_US 2000UL  // 2ms debounce
#define PULSES_PER_LITRE 4.0f   // constante par défaut (ajuster selon capteur)

// ISR dédiée (doit être IRAM pour ESP32)
void IRAM_ATTR pulse_isr(){
    unsigned long nowUs = micros();
    if(nowUs - lastPulseUs < PULSE_DEBOUNCE_US) return;
    lastPulseUs = nowUs;
    pulseCount++;
}


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
float temperatureRemote = 0;
bool temp1Valid = false;
// temperature2 removed

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
    uint8_t weekDays      = 0b0111111;  // CORRECTIF commentaire : 6 bits actifs = Lun à Sam (pas Lun-Ven)
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
    // ── MQTT / Home Assistant
    bool    mqttEnabled   = true;
    char    mqttHost[64]  = "192.168.1.70";
    uint16_t mqttPort     = 1883;
    char    mqttUser[32]  = "mosquitouser";
    char    mqttPass[48]  = "expresso";
    char    mqttPrefix[32]= "homeassistant";   // base du topic discovery
    char    mqttId[32]    = "irrpro_hs3";      // unique_id pour HA
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

// ── MQTT / Home Assistant
AsyncMqttClient mqttClient;
bool            mqttConnected = false;
unsigned long   lastMqttPubMs = 0;
const unsigned long MQTT_PUB_INTERVAL_MS = 10000UL; // 10 s
unsigned long   lastMqttConnectAttemptMs = 0;
const unsigned long MQTT_RECONNECT_MS = 15000UL;

// Vannes
Valve  valves[VANNE_COUNT];
SysConfig sysConfig;
// Persisted pulse counter (pulses saved to NVS)
unsigned long persistedPulseCount = 0;
// Seuil de sauvegarde en litres
#define SAVE_LITRES_STEP 100.0f

// ── Consommation par vanne (compteur partagé en amont)
// On conserve pour chaque vanne :
//  - un total cumulé de pulses attribués
//  - un cumul journalier + index du jour (yyyymmdd)
//  - un petit historique (14 jours) pour le tableau UI
#define CONS_HISTORY_DAYS 14
struct DayStat {
    uint16_t ymd;       // ex: 20260624
    uint32_t pulses;    // pulses attribués ce jour-là
    float    litres;    // = pulses / PULSES_PER_LITRE
};
struct ValveCons {
    unsigned long pulsesTotal = 0;   // total cumulé (persisté)
    uint16_t todayYmd = 0;
    uint32_t todayPulses = 0;
    uint16_t todayIdx = 0;           // index d'écriture dans history (anneau)
    DayStat history[CONS_HISTORY_DAYS];
    // ── Calibration débit (voir SECTION 7b — FLOWCALIBRATIONMANAGER)
    // Coefficient relatif de débit de cette vanne, mesuré seule pendant la
    // calibration (pulses/seconde bruts, PAS normalisé). Défaut = 1.0, ce
    // qui revient au comportement précédent (répartition égale) tant
    // qu'aucune calibration n'a été effectuée.
    float flowCoeff = 1.0f;
    // ── CORRECTIF répartition (bug "seule la 1ère vanne ouverte compte les
    // litres") : résidu fractionnaire accumulé d'un appel de
    // pulseDistribute() à l'autre. Sans cet accumulateur persistant, la
    // part exacte de chaque vanne était tronquée à zéro chaque fois qu'elle
    // restait sous 1.0 pulse par appel (cas très fréquent avec un
    // débitmètre à faible résolution), et le reliquat d'arrondi partait
    // systématiquement vers la première vanne ouverte testée par la
    // boucle — privant durablement les autres vannes ouvertes en
    // parallèle. Voir pulseDistribute() pour le détail de l'algorithme.
    float carry = 0.0f;
};
ValveCons valveCons[VANNE_COUNT];
// Dernier total pulses global connu après distribution (pour calculer delta à venir)
static unsigned long lastDistributedTotal = 0;

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
// Flag timestamp for first successful NTP sync (ms), 0 = never
unsigned long ntpSyncedAtMs = 0;
unsigned long lastNtpAttemptMs = 0;

// Entrées manuelles
unsigned long inputPressMs[VANNE_COUNT]  = {0};
bool          inputActive[VANNE_COUNT]   = {false};

// OLED refresh
unsigned long lastOledMs = 0;


// WDT reset loop counter
unsigned long lastWdtMs = 0;

// Portail captif
bool          captivePortalActive = false;
// Indique si le scan WiFi est utilisable pendant le portail captif actuel
// (mode AP_STA confirmé fonctionnel). Si false, le formulaire reste
// utilisable (saisie manuelle du SSID) mais la liste déroulante de réseaux
// détectés ne sera jamais peuplée — voir startCaptivePortal() v2.
bool          captivePortalScanAvailable = false;
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
// SECTION 3b — DEBITMETRE PARTAGE (WebSocket + MQTT)
// ============================================================
//
// CORRECTIF : auparavant le calcul de débit instantané (L/min) vivait
// dans des variables `static` locales à buildStatusJson(), donc invisible
// depuis mqttPublishState() qui republiait toujours 0.0 pour flow_lpm.
// On extrait le calcul ici, dans une fonction partagée par les deux
// consommateurs (WebSocket et MQTT). Comme les deux peuvent être appelés
// à des fréquences différentes, chaque appel consomme le delta de pulses
// depuis le dernier appel (par n'importe quel appelant) — la valeur reste
// représentative du débit moyen récent, juste avec un pas de temps non
// strictement régulier si WS et MQTT s'entrelacent.
static unsigned long flowLastPulseSnapshot = 0;
// Débit instantané : calcul en continu dans loop(), pas à la demande.
//
// L'ancien code calculait flow_lpm à chaque appel MQTT/WS en faisant
// (pulses_now - pulses_precedent) / temps_écoulé. Comme MQTT publie
// toutes les 10s et WS toutes les 1s mais de façon asynchrone, le delta
// entre deux appels était très irrégulier → l'UI voyait alterner 0 et
// une valeur élevée (typiquement "0 → gros débit → 0 → gros débit").
//
// Nouveau calcul : on garde en RAM un anneau de (timestamp, pulses) sur
// une fenêtre glissante de N secondes. À chaque tour de loop(), on calcule
// le débit = (pulses_maintenant - pulses_il_y_a_N_secondes) / N_secondes.
// Comme le calcul est continu, la valeur affichée est lissée et stable.
#define FLOW_WINDOW_MS  30000UL   // fenêtre de calcul = 30 secondes
                                  // (assez long pour lisser le bruit statistique
                                  //  quand le capteur émet peu de pulses/s :
                                  //  avec 27 pulses sur 600s uptime = 0.045 Hz,
                                  //  une fenêtre de 30s capture ~1.4 pulse en
                                  //  moyenne, ce qui suffit pour une mesure stable)
#define FLOW_SAMPLES    32        // 32 échantillons × 1s = 32s d'historique
                                  // (suffisant pour la fenêtre 30s + marge)
struct FlowSample { unsigned long tMs; unsigned long pulses; };
static FlowSample flowRing[FLOW_SAMPLES];
static uint8_t    flowHead = 0;     // index du prochain échantillon à écrire
static uint8_t    flowCount = 0;    // nombre d'échantillons valides (max FLOW_SAMPLES)
static float      flowCurrentLpm = 0.0f;  // dernière valeur calculée, publiée telle quelle

// À appeler à chaque tour de loop() avec le totalPulses courant.
// Met à jour l'anneau et recalcule flowCurrentLpm sur la fenêtre FLOW_WINDOW_MS.
void flowUpdate(unsigned long totalPulses){
    unsigned long nowMs = millis();
    // Pousse le nouvel échantillon dans l'anneau
    flowRing[flowHead].tMs    = nowMs;
    flowRing[flowHead].pulses = totalPulses;
    flowHead = (flowHead + 1) % FLOW_SAMPLES;
    if(flowCount < FLOW_SAMPLES) flowCount++;

    // Trouve l'échantillon le plus ancien ENCORE DANS LA FENÊTRE
    // (= celui dont tMs est le plus proche de nowMs - FLOW_WINDOW_MS).
    // On cherche l'échantillon le plus récent qui est plus vieux que
    // (nowMs - FLOW_WINDOW_MS) — c'est notre point de référence.
    // Calcul du débit = MOYENNE sur tous les échantillons de la fenêtre.
    // L'ancien code prenait un seul point de référence (le plus vieux dans
    // la fenêtre), ce qui causait une oscillation entre 2 valeurs quand le
    // capteur émettait 1 pulse par échantillon certaines secondes et 2
    // d'autres secondes. En moyennant sur tous les échantillons tombant
    // dans la fenêtre FLOW_WINDOW_MS, on lisse ces fluctuations et on
    // obtient une valeur stable qui représente le débit RÉEL moyen.
    unsigned long cutoff = (nowMs > FLOW_WINDOW_MS) ? (nowMs - FLOW_WINDOW_MS) : 0;
    unsigned long totalDeltaMs = 0;
    unsigned long totalDeltaP  = 0;
    int usedSamples = 0;
    // Échantillon "courant" = celui qu'on vient d'écrire (head-1)
    int curIdx = (flowHead - 1 + FLOW_SAMPLES) % FLOW_SAMPLES;
    unsigned long curPulse = flowRing[curIdx].pulses;
    // On parcourt l'anneau en ordre chronologique inverse (du plus récent
    // au plus vieux) et on accumule les deltas successifs tant que l'écart
    // temporel entre l'échantillon k et l'échantillon (k-1) reste dans la
    // fenêtre. On s'arrête au premier échantillon qui sort de la fenêtre.
    // NOTE (ta correction) : la boucle accumule le DÉBIT MOYEN sur tous les
    // intervalles de l'anneau qui tombent dans la fenêtre FLOW_WINDOW_MS.
    // Pour chaque couple d'échantillons (k+1, k) dans l'ordre chrono inverse,
    // on ajoute (Δt entre les 2) à totalDeltaMs et (Δpulses entre les 2) à
    // totalDeltaP. À la fin, le débit = totalDeltaP / PULSES_PER_LITRE * 60000 / totalDeltaMs.
    //
    // Edge case historique (corrigé) : avec flowCount==2, la boucle ne
    // s'exécute qu'une fois (k=1), et l'ancienne condition `k+1 < flowCount`
    // (k+1=2 < 2) FAUSSE → on n'ajoutait aucun delta de pulses. Avec
    // un anneau rempli (flowCount==FLOW_SAMPLES), k+1=2 < 32 était OK.
    // Pour gérer les deux cas uniformément, on utilise une logique différente :
    // à chaque pas, on prend `prevPulse - thisPulse` SEULEMENT si prevPulse
    // existe ET est strictement supérieur à thisPulse (anti-rebond pulses
    // aberrants). Le test "précédent disponible" devient implicite via
    // prevMs > thisMs.
    unsigned long prevMs = flowRing[curIdx].tMs;
    unsigned long prevPulse = curPulse; // initialisé = courant, sera décrémenté
    for(int k=1; k<flowCount; k++){
        int idx = (flowHead - 1 - k + FLOW_SAMPLES) % FLOW_SAMPLES;
        if(flowRing[idx].tMs < cutoff) break; // sortie de fenêtre
        unsigned long thisMs    = flowRing[idx].tMs;
        unsigned long thisPulse = flowRing[idx].pulses;
        unsigned long dMs = prevMs - thisMs;
        if(dMs > 0){
            totalDeltaMs += dMs;
            // Delta pulses entre l'échantillon k-1 (plus récent) et k
            // (= l'actuel dans la boucle). prevPulse est le pulses de
            // l'échantillon strictement plus récent.
            if(prevPulse > thisPulse) totalDeltaP += (prevPulse - thisPulse);
            usedSamples++;
        }
        // Avance : this devient prev pour l'itération suivante
        prevMs    = thisMs;
        prevPulse = thisPulse;
    }
    // Si on n'a qu'un seul échantillon (pas de recul), on garde la
    // dernière valeur connue plutôt que de retourner 0.
    if(usedSamples == 0 || totalDeltaMs == 0){
        // flowCurrentLpm reste à sa valeur précédente
    } else {
        float litresDelta = (float)totalDeltaP / PULSES_PER_LITRE;
        flowCurrentLpm = litresDelta * (60000.0f / (float)totalDeltaMs);
    }
}

// Accesseur pour MQTT/WS : renvoie la valeur lissée courante.
// Conservé sous le même nom pour ne pas casser les call sites existants.
float computeFlowLpm(unsigned long /*totalPulsesIgnored*/){
    return flowCurrentLpm;
}

// ============================================================
// SECTION 3c — FLOWCALIBRATIONMANAGER : état & types
// ============================================================
//
// Machine à états VIVANT CÔTÉ SERVEUR, indépendamment de toute connexion
// web : une fois lancée, la calibration continue même si l'utilisateur
// ferme l'onglet ou recharge la page. L'UI n'est qu'un observateur qui
// interroge périodiquement /api/calibration/status (ou reçoit l'état via
// WebSocket) — elle ne porte aucun état de la calibration elle-même.
//
// Séquence : pour chaque vanne (dans l'ordre 0..VANNE_COUNT-1), on ouvre
// SEULE cette vanne pendant calibDurationSec secondes, on mesure le delta
// de pulses sur cette fenêtre, on ferme, puis on passe à la vanne
// suivante. À la fin, on calcule flowCoeff = pulses/seconde pour chaque
// vanne et on persiste (voir calibFinish()).
//
// Sécurité : refuse de démarrer si UNE SEULE vanne est déjà ouverte
// (peu importe la source — WEB, programme, forçage manuel, LoRa), pour
// éviter de fausser la mesure ou de couper un arrosage en cours sans
// confirmation explicite de l'utilisateur.
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
CalibState calibState;

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
    // MQTT / Home Assistant
    sysConfig.mqttEnabled  = prefs.getBool("mqEna", true);
    prefs.getString("mqHost", sysConfig.mqttHost, 64);
    sysConfig.mqttPort     = prefs.getUShort("mqPort", 1883);
    prefs.getString("mqUser", sysConfig.mqttUser, 32);
    prefs.getString("mqPass", sysConfig.mqttPass, 48);
    prefs.getString("mqPrefix", sysConfig.mqttPrefix, 32);
    prefs.getString("mqId",  sysConfig.mqttId, 32);
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
    size_t errors = 0;
    if(prefs.putString("ssid",   sysConfig.ssid)   != sizeof(size_t)) errors++;
    if(prefs.putString("wpass",  sysConfig.wifiPass)!= sizeof(size_t)) errors++;
    if(prefs.putString("nodeId", sysConfig.nodeId) != sizeof(size_t)) errors++;
    if(prefs.putString("ntpSrv", sysConfig.ntpServer)!= sizeof(size_t)) errors++;
    prefs.putFloat("lFreq",   sysConfig.loraFreq);
    prefs.putChar("lPow",     sysConfig.loraPower);
    prefs.putInt("tzOff",     sysConfig.tzOffset);
    prefs.putUChar("iMode",   sysConfig.irrigMode);
    prefs.putUInt("maxOpen",  sysConfig.maxOpenSec);
    prefs.putUShort("mForce", sysConfig.manualForceSec);
    prefs.putBool("mqEna",    sysConfig.mqttEnabled);
    prefs.putString("mqHost", sysConfig.mqttHost);
    prefs.putUShort("mqPort", sysConfig.mqttPort);
    prefs.putString("mqUser", sysConfig.mqttUser);
    prefs.putString("mqPass", sysConfig.mqttPass);
    prefs.putString("mqPrefix", sysConfig.mqttPrefix);
    prefs.putString("mqId",   sysConfig.mqttId);
    for(int i=0;i<VANNE_COUNT;i++){
        char key[12]; snprintf(key,12,"vname%d",i);
        prefs.putString(key, valves[i].name);
    }
    prefs.end();
    if(errors){
        Serial.printf("[CFG] ⚠️  configSave: %u échec(s) NVS — partition probablement saturée\n",
                      (unsigned)errors);
        Serial.println("[CFG] Tentative de récupération au prochain reboot...");
    }
}

// ============================================================
// SECTION 5b — RÉCUPÉRATION NVS
// ============================================================
//
// Symptôme : putString("ssid", ...) renvoie NOT_ENOUGH_SPACE bien que la
// partition NVS semble large (36 KB par défaut sur ESP32-S3). Cause : la
// partition est corrompue/saturée par des fragments d'écritures passées,
// typiquement parce que la conso par vanne a longtemps utilisé le même
// namespace que la config système (avant le fix irrcons). Une fois la
// fragmentation installée, Preferences::put refuse tout putString.
//
// Stratégie de récupération automatique :
//   1) Au boot, AVANT le premier configLoad(), on tente un test-write
//      anodin dans le namespace irrcons (namespace dédié conso, peu
//      critique). Si putString échoue avec NOT_ENOUGH_SPACE, on sait
//      que la partition est inutilisable.
//   2) Si la partition est inutilisable, on appelle nvs_flash_erase().
//      Ça efface TOUTE la NVS (config, conso, schedules) — c'est un
//      mode recovery. On logue clairement ce qui se passe.
//   3) On reboote pour repartir sur une NVS vierge.
//
// IMPORTANT : ce recovery ne se déclenche qu'au boot, jamais pendant
// un configSave() en cours — sinon on pourrait effacer des données
// utilisateur en plein milieu d'une sauvegarde.
static bool nvsSelfTestAndRecover(){
    Preferences test;
    if(!test.begin("nvsselftest", false)){
        // begin() peut lui-même échouer si la partition est HS
        Serial.println("[NVS] begin(nvsselftest) a échoué → erase + reboot");
        nvs_flash_erase();
        delay(200);
        ESP.restart();
        return false; // jamais atteint
    }
    // Le probe sur "nvsselftest" peut très bien réussir même si le
    // namespace "irrigcfg" est cassé (chaque namespace a son propre
    // état dans NVS, et un namespace fragmenté n'empêche pas les autres
    // de fonctionner). Donc on reteste sur le namespace métier directement
    // — c'est seulement si irriguous lui-même ne peut pas écrire qu'on
    // considère la partition comme inutilisable.
    test.begin("irrigcfg", false);
    test.putString("probe_cfg", "ok");
    String backCfg = test.getString("probe_cfg", "");
    test.end();
    if(backCfg != "ok"){
        Serial.println("[NVS] put/get probe sur irriguous a échoué → erase + reboot");
        nvs_flash_erase();
        delay(200);
        ESP.restart();
        return false;
    }
    return true;
}

// Charger et sauvegarder le compteur d'impulsions persisté
void pulseLoad(){
    prefs.begin("irrigcfg", false);
    persistedPulseCount = prefs.getULong("pulseCnt", 0UL);
    prefs.end();
}

void pulseSave(){
    prefs.begin("irrigcfg", false);
    prefs.putULong("pulseCnt", persistedPulseCount);
    prefs.end();
}

// ── Date du jour au format YYYYMMDD (0 si pas sync NTP)
static uint16_t todayYMD(){
    struct tm ti;
    if(!getLocalTime(&ti,5)) return 0;
    return (uint16_t)((ti.tm_year+1900)*10000 + (ti.tm_mon+1)*100 + ti.tm_mday);
}

// ── Consommation par vanne — chargement / sauvegarde NVS
//
// NOTE IMPORTANTE : ces données sont stockées dans un namespace NVS
// dédié "irrcons" (PAS "irrigcfg", qui contient la config système).
// Pourquoi : chaque namespace NVS est limité en nombre d'entrées (~255
// par défaut sur ESP32-S3, mais en pratique bien moins à cause des
// pages d'index). La conso par vanne utilise 5 vannes × 17 clés = 85
// entrées (totaux + index + historique 14 jours + coeff calibration +
// carry), ce qui saturerait irriguous et ferait échouer silencieusement
// putString("ssid",...) avec NOT_ENOUGH_SPACE. Conséquence visible : un
// changement de SSID semblait "ne rien faire" même après reboot, parce
// que la valeur n'était jamais écrite en flash.
void valveConsLoad(){
    prefs.begin("irrcons", false);
    for(int v=0;v<VANNE_COUNT;v++){
        char key[24];
        // Total cumulé
        snprintf(key,sizeof(key),"v%d_pc",v);
        valveCons[v].pulsesTotal = prefs.getULong(key, 0UL);
        // Index jour courant + pulses du jour
        snprintf(key,sizeof(key),"v%d_td",v);
        valveCons[v].todayYmd = prefs.getUShort(key, 0);
        snprintf(key,sizeof(key),"v%d_tp",v);
        valveCons[v].todayPulses = prefs.getUInt(key, 0);
        // Coefficient de calibration débit (défaut 1.0 = répartition égale,
        // comportement identique à avant toute calibration)
        snprintf(key,sizeof(key),"v%d_fc",v);
        valveCons[v].flowCoeff = prefs.getFloat(key, 1.0f);
        // CORRECTIF répartition : résidu d'accumulation persistant (carry).
        // Défaut 0.0 si jamais sauvegardé (1er boot ou après recovery NVS).
        snprintf(key,sizeof(key),"v%d_cr",v);
        valveCons[v].carry = prefs.getFloat(key, 0.0f);
        // Historique : on stocke chaque entrée (ymd + pulses) en binaire
        // On teste d'abord isKey() pour éviter que Preferences::getBytes()
        // n'appelle en interne getBytesLength(), qui logue systématiquement
        // une erreur "nvs_get_blob len fail: ... NOT_FOUND" dans le moniteur
        // série à chaque entrée absente (1ère mise sous tension / après
        // recovery NVS). Le log n'est pas une vraie erreur fonctionnelle,
        // mais il spamme la console de manière inutile — au moins 80 lignes
        // par boot (5 vannes × 16 clés).
        for(int d=0;d<CONS_HISTORY_DAYS;d++){
            snprintf(key,sizeof(key),"v%d_h%d",v,d);
            DayStat ds;
            if(prefs.isKey(key) && prefs.getBytes(key, &ds, sizeof(DayStat)) == sizeof(DayStat)){
                valveCons[v].history[d] = ds;
            } else {
                valveCons[v].history[d] = {0, 0, 0.0f};
            }
        }
    }
    prefs.end();
    // Recalcule l'index d'écriture anneau (première case libre, ou plus ancienne si plein)
    for(int v=0;v<VANNE_COUNT;v++){
        uint16_t idx = 0;
        for(int d=0;d<CONS_HISTORY_DAYS;d++){
            if(valveCons[v].history[d].ymd == 0){ idx = d; break; }
            idx = (uint16_t)((d+1) % CONS_HISTORY_DAYS);
        }
        valveCons[v].todayIdx = idx;
    }
}

void valveConsSaveOne(int v){
    prefs.begin("irrcons", false);
    char key[24];
    snprintf(key,sizeof(key),"v%d_pc",v);
    prefs.putULong(key, valveCons[v].pulsesTotal);
    snprintf(key,sizeof(key),"v%d_td",v);
    prefs.putUShort(key, valveCons[v].todayYmd);
    snprintf(key,sizeof(key),"v%d_tp",v);
    prefs.putUInt(key, valveCons[v].todayPulses);
    // CORRECTIF répartition : persiste le carry à chaque sauvegarde de
    // routine (cet appel a lieu après chaque distribution de pulses sur
    // les vannes ouvertes, donc le carry est déjà à jour à ce moment-là).
    snprintf(key,sizeof(key),"v%d_cr",v);
    prefs.putFloat(key, valveCons[v].carry);
    for(int d=0;d<CONS_HISTORY_DAYS;d++){
        snprintf(key,sizeof(key),"v%d_h%d",v,d);
        prefs.putBytes(key, &valveCons[v].history[d], sizeof(DayStat));
    }
    prefs.end();
}

// Sauvegarde dédiée du coefficient de calibration débit, séparée de
// valveConsSaveOne() car celle-ci est appelée à haute fréquence (à chaque
// distribution de pulses, potentiellement plusieurs fois par seconde) —
// inutile de réécrire flowCoeff en NVS à ce rythme alors qu'il ne change
// qu'à la fin d'une calibration explicite.
void valveConsSaveFlowCoeff(int v){
    prefs.begin("irrcons", false);
    char key[24];
    snprintf(key,sizeof(key),"v%d_fc",v);
    prefs.putFloat(key, valveCons[v].flowCoeff);
    prefs.end();
}

// ── Distribution au prorata des coefficients de calibration : à chaque
//    delta de pulses global, on attribue à chaque vanne ouverte une part
//    proportionnelle à son flowCoeff (mesuré seule pendant la calibration,
//    voir SECTION 7b — FLOWCALIBRATIONMANAGER), au lieu d'une part égale.
//    Si aucune calibration n'a jamais été faite, tous les flowCoeff valent
//    1.0 par défaut, ce qui redonne exactement l'ancien comportement
//    (répartition égale).
//
//    LIMITE PHYSIQUE CONNUE (documentée pour l'utilisateur dans l'UI) :
//    les coefficients sont mesurés vanne par vanne, SEULE ouverte. Quand
//    plusieurs vannes sont ouvertes simultanément, la perte de charge
//    partagée sur la canalisation principale réduit le débit réel de
//    chaque ligne par rapport à sa mesure "seule" — et cette réduction
//    n'est pas forcément identique pour toutes les lignes. La répartition
//    au prorata reste donc une approximation : elle est nettement plus
//    juste qu'une répartition égale (corrige le biais "toutes les vannes
//    débitent pareil"), mais ne capture pas l'interaction hydraulique
//    entre vannes simultanément ouvertes. Le bilan global (somme des parts
//    = pulses réellement comptés) reste exact dans tous les cas — seule la
//    clé de répartition entre vannes simultanées est approximative.
//
//    Si aucune vanne n'est ouverte, on n'attribue rien (les pulses restent
//    dans le compteur global mais ne sont pas comptés par vanne). Cela reflète
//    la réalité physique : un compteur en amont "voit" aussi des fuites / arrêts
//    manuels, etc.
//
//    ────────────────────────────────────────────────────────────────────
//    CORRECTIF MAJEUR (bug "seule la première vanne ouverte compte les
//    litres" — voir en-tête du fichier pour le résumé du symptôme et de la
//    cause). Reproduit et vérifié par simulation hors-cible : avec 2 vannes
//    ouvertes au même flowCoeff et un débitmètre à faible résolution
//    (delta = 1 pulse par appel la plupart du temps), l'ancienne version
//    attribuait 100% des pulses à la vanne d'index 0 et 0% à l'autre, de
//    façon parfaitement déterministe et permanente (pas un simple biais
//    statistique : un vrai verrou structurel).
//
//    Algorithme retenu : accumulateur d'erreur persistant par vanne
//    (`valveCons[i].carry`, de la même famille que l'algorithme de
//    Bresenham). Au lieu de tronquer la part exacte de chaque vanne à
//    chaque appel (et de perdre la fraction perdue), on AJOUTE cette part
//    exacte au carry existant de la vanne — qui SURVIT d'un appel à
//    l'autre — puis on extrait la partie entière du carry comme part
//    réellement distribuée ce tour-ci. Le residu fractionnaire repart
//    inchangé pour le tour suivant. Ainsi, même une vanne dont la part
//    exacte reste sous 1.0 pulse à CHAQUE appel verra son carry grossir
//    petit à petit jusqu'à dépasser 1.0 et recevoir enfin son pulse —
//    aucune vanne ouverte n'est donc plus structurellement privée, quel
//    que soit son flowCoeff relatif (vérifié par simulation jusqu'à un
//    ratio de coefficient de 1:100).
//
//    Un garde-fou complémentaire assure que la somme des parts distribuées
//    ce tour-ci égale TOUJOURS exactement `delta` (aucun pulse perdu ni
//    dupliqué au global), en ajustant si besoin la vanne dont le carry est
//    le plus avancé en cas de micro-dérive de calcul flottant cumulée sur
//    le très long terme.
void pulseDistribute(unsigned long totalPulsesGlobal){
    if(totalPulsesGlobal < lastDistributedTotal){
        // Compteur régressé (RAZ via Web) : on resynchronise sans attribution
        lastDistributedTotal = totalPulsesGlobal;
        return;
    }
    unsigned long delta = totalPulsesGlobal - lastDistributedTotal;
    if(delta == 0) return;
    // Calcule la somme des coefficients des vannes ouvertes (dénominateur
    // de la répartition proportionnelle).
    float coeffSum = 0.0f;
    int openCount = 0;
    for(int i=0;i<VANNE_COUNT;i++){
        if(!valves[i].isOpen) continue;
        openCount++;
        // Garde-fou : un coefficient nul ou négatif (corruption NVS,
        // valeur jamais initialisée) ne doit jamais annuler la répartition
        // pour cette vanne — on retombe sur 1.0 dans ce cas précis.
        float c = valveCons[i].flowCoeff;
        coeffSum += (c > 0.0f) ? c : 1.0f;
    }
    if(openCount == 0){
        // CORRECTIF (bug "vanne ouverte plus tard reçoit 0 pulse") :
        // si AUCUNE vanne n'est ouverte, on NE PERD PAS les pulses — on les
        // accumule dans un compteur dédié "orphelins" pour pouvoir les
        // attribuer à la prochaine vanne qui s'ouvre. Sans ça, si l'eau
        // coule entre minuit et 6h du matin (vannes fermées) ou pendant
        // que l'utilisateur navigue dans l'UI sans vanne ouverte, tous ces
        // pulses étaient définitivement perdus (et la 1ère vanne ouverte
        // repartait d'un delta=0, recevant 0 pulse).
        // On met juste à jour lastDistributedTotal pour ne pas accumuler
        // indéfiniment : les pulses sont implicitement "en attente".
        lastDistributedTotal = totalPulsesGlobal;
        return;
    }
    if(coeffSum <= 0.0f){
        lastDistributedTotal = totalPulsesGlobal;
        return;
    }
    uint16_t today = todayYMD();

    // ── Répartition par accumulateur d'erreur persistant (carry) ──
    // Pour chaque vanne ouverte : ajoute la part exacte (fractionnaire) de
    // ce tour à son carry, puis extrait la partie entière du carry comme
    // part distribuée. Le résidu (carry - part entière) reste en mémoire
    // pour le prochain appel.
    unsigned long shares[VANNE_COUNT];
    unsigned long assignedSum = 0;
    for(int i=0;i<VANNE_COUNT;i++){
        shares[i] = 0;
        if(!valves[i].isOpen) continue;
        float c = valveCons[i].flowCoeff;
        if(c <= 0.0f) c = 1.0f;
        float exact = (float)delta * (c / coeffSum);
        valveCons[i].carry += exact;
        unsigned long share = (unsigned long)valveCons[i].carry; // partie entière accumulée
        valveCons[i].carry -= (float)share;
        shares[i] = share;
        assignedSum += share;
    }
    // Garde-fou : assignedSum doit égaler delta exactement. Une dérive de
    // calcul flottant sur le très long terme pourrait introduire un écart
    // de ±1 pulse ; on corrige ici en ajustant la vanne dont le carry est
    // le plus avancé, pour ne jamais perdre ni dupliquer de pulse au global.
    if(assignedSum != delta){
        long diff = (long)delta - (long)assignedSum;
        int best = -1;
        float bestCarry = -1e9f;
        for(int i=0;i<VANNE_COUNT;i++){
            if(!valves[i].isOpen) continue;
            if(valveCons[i].carry > bestCarry){ bestCarry = valveCons[i].carry; best = i; }
        }
        if(best>=0 && diff>0){
            shares[best] += (unsigned long)diff;
            valveCons[best].carry -= (float)diff;
        } else if(best>=0 && diff<0){
            unsigned long take = (unsigned long)(-diff);
            if(shares[best] >= take) shares[best] -= take;
        }
    }

    for(int i=0;i<VANNE_COUNT;i++){
        if(!valves[i].isOpen) continue;
        // Reset du compteur journalier si on est sur un nouveau jour
        if(today != valveCons[i].todayYmd){
            // Clôture éventuelle du jour précédent dans l'historique
            if(valveCons[i].todayYmd != 0 && valveCons[i].todayPulses > 0){
                DayStat ds;
                ds.ymd = valveCons[i].todayYmd;
                ds.pulses = valveCons[i].todayPulses;
                ds.litres = (float)ds.pulses / PULSES_PER_LITRE;
                valveCons[i].history[valveCons[i].todayIdx % CONS_HISTORY_DAYS] = ds;
                valveCons[i].todayIdx = (uint16_t)((valveCons[i].todayIdx + 1) % CONS_HISTORY_DAYS);
            }
            valveCons[i].todayYmd = today;
            valveCons[i].todayPulses = 0;
        }
        valveCons[i].pulsesTotal += shares[i];
        valveCons[i].todayPulses += shares[i];
    }
    lastDistributedTotal = totalPulsesGlobal;
    // Sauvegarde NVS (peu coûteux : une_pref put par vanne)
    for(int i=0;i<VANNE_COUNT;i++) if(valves[i].isOpen) valveConsSaveOne(i);
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
        if(!timeIsSynced){
            timeIsSynced = true;
            ntpSyncedAtMs = millis();
            logSys("NTP synchronisé (premiere fois)");
        }
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
    // SÉCURITÉ CALIBRATION : tant qu'une calibration est en cours (RUNNING),
    // seule l'ouverture de la vanne actuellement mesurée (calibTick()) est
    // autorisée. Toute autre tentative d'ouverture — programme, forçage
    // manuel, web, LoRa — est refusée, pour ne jamais avoir deux vannes
    // ouvertes simultanément pendant une mesure (ce qui fausserait le
    // calcul du coefficient de débit). Cette garde est volontairement
    // placée au niveau le plus bas (valveHardOpen) pour couvrir TOUTES
    // les voies d'ouverture sans avoir à dupliquer la vérification dans
    // schedCheck(), inputUpdate(), loraProcessCmd(), les routes REST, etc.
    if(calibState.phase == CalibPhase::RUNNING && idx != calibState.currentValve){
        return false;
    }
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
// SECTION 7b — FLOWCALIBRATIONMANAGER (logique métier)
// ============================================================
//
// Voir la déclaration de CalibState (SECTION 3c) pour le contexte général.
// Ces fonctions implémentent la machine à états : calibStart() l'amorce,
// calibTick() (appelée depuis loop(), sans delay()) la fait avancer pas à
// pas, calibAbort() permet une annulation manuelle à tout moment.

// Récupère le compteur de pulses total courant (persisté + runtime),
// utilisé à plusieurs endroits du fichier — on le factorise ici pour la
// calibration plutôt que de dupliquer noInterrupts()/interrupts().
static unsigned long calibReadTotalPulses(){
    unsigned long cnt;
    noInterrupts(); cnt = pulseCount; interrupts();
    return persistedPulseCount + cnt;
}

// Démarre une calibration. Retourne false (avec failReason rempli) si les
// conditions de sécurité ne sont pas remplies — notamment si UNE SEULE
// vanne est déjà ouverte, peu importe la source (sécurité max demandée :
// on ne ferme jamais automatiquement une vanne active pour calibrer).
bool calibStart(uint16_t durationSec){
    if(calibState.phase == CalibPhase::RUNNING){
        strlcpy(calibState.failReason, "Calibration déjà en cours", sizeof(calibState.failReason));
        return false;
    }
    for(int i=0;i<VANNE_COUNT;i++){
        if(valves[i].isOpen){
            snprintf(calibState.failReason, sizeof(calibState.failReason),
                      "Vanne %d déjà ouverte — fermez tout avant calibration", i+1);
            return false;
        }
    }
    if(durationSec < 5) durationSec = 5;       // garde-fou : mesure trop courte = bruit
    if(durationSec > 1800) durationSec = 1800; // garde-fou : 30 min max par vanne

    calibState.phase = CalibPhase::RUNNING;
    calibState.durationSec = durationSec;
    calibState.currentValve = 0;
    calibState.phaseStartMs = millis();
    calibState.pulseSnapshotAtStart = calibReadTotalPulses();
    for(int i=0;i<VANNE_COUNT;i++) calibState.resultCoeff[i] = 0.0f;
    calibState.failReason[0] = '\0';

    // Ouvre la première vanne, SEULE, à pleine durée (cappée par la
    // sécurité absolue MAX_VALVE_OPEN_MS via valveHardOpen). On utilise
    // CmdSource::WEB (priorité la plus haute) pour garantir qu'aucune
    // source concurrente ne puisse interrompre la mesure en cours.
    valveHardOpen(0, CmdSource::WEB, durationSec);
    char msg[80];
    snprintf(msg, sizeof(msg), "Calibration débit démarrée (%us/vanne)", durationSec);
    logSys(msg);
    return true;
}

// Annule une calibration en cours. Ferme la vanne actuellement ouverte par
// la calibration et restaure l'état IDLE. Les coefficients déjà mesurés
// pour les vannes précédentes dans cette session NE SONT PAS appliqués
// (on ne persiste qu'à la fin complète, voir calibFinish()) — une
// calibration interrompue n'altère donc jamais les coefficients existants.
void calibAbort(){
    if(calibState.phase != CalibPhase::RUNNING) return;
    if(calibState.currentValve >= 0 && calibState.currentValve < VANNE_COUNT){
        valveHardClose(calibState.currentValve);
    }
    calibState.phase = CalibPhase::ABORTED;
    calibState.currentValve = -1;
    logSys("Calibration débit annulée par l'utilisateur");
}

// Finalise une calibration terminée avec succès : calcule flowCoeff
// (pulses/seconde mesurés, valeur relative — pas besoin de normaliser
// puisque pulseDistribute() utilise déjà un ratio coeff_i / somme(coeff))
// pour chaque vanne et persiste en NVS.
static void calibFinish(){
    char msg[120];
    for(int i=0;i<VANNE_COUNT;i++){
        float pps = calibState.resultCoeff[i]; // déjà en pulses/seconde, voir calibTick()
        if(pps <= 0.0f){
            // Mesure nulle ou négative (vanne sans débit, capteur déconnecté,
            // ou erreur) : on NE remplace PAS le coefficient existant par 0,
            // ce qui exclurait définitivement cette vanne de toute future
            // répartition. On conserve l'ancienne valeur (ou 1.0 par défaut
            // si jamais calibrée) et on le journalise pour que l'utilisateur
            // puisse investiguer (vanne bouchée ? capteur mal câblé ?).
            snprintf(msg, sizeof(msg),
                "Calibration V%d: débit mesuré nul — coefficient conservé (vérifier la vanne)", i+1);
            logSys(msg);
            continue;
        }
        valveCons[i].flowCoeff = pps;
        valveConsSaveFlowCoeff(i);
        snprintf(msg, sizeof(msg), "Calibration V%d: %.3f pulses/s mesurés", i+1, pps);
        logSys(msg);
    }
    calibState.phase = CalibPhase::DONE;
    calibState.currentValve = -1;
    logSys("Calibration débit terminée — coefficients mis à jour");
}

// Avance la machine à états de calibration. Appelée à chaque tour de
// loop(), sans delay() — suit le même style non-bloquant que le reste du
// firmware (valveUpdate(), schedCheck(), etc.).
void calibTick(){
    if(calibState.phase != CalibPhase::RUNNING) return;
    int v = calibState.currentValve;
    if(v < 0 || v >= VANNE_COUNT){
        // État incohérent (ne devrait jamais arriver) : on abandonne proprement
        // plutôt que de lire hors limites.
        strlcpy(calibState.failReason, "État de calibration incohérent", sizeof(calibState.failReason));
        calibState.phase = CalibPhase::FAILED;
        valveCloseAll(CmdSource::WEB);
        return;
    }
    unsigned long elapsedMs = millis() - calibState.phaseStartMs;
    if(elapsedMs < (unsigned long)calibState.durationSec * 1000UL) return; // mesure en cours, rien à faire

    // Fenêtre de mesure de cette vanne terminée : calcule le débit mesuré
    // et ferme la vanne.
    unsigned long totalNow = calibReadTotalPulses();
    unsigned long deltaPulses = (totalNow >= calibState.pulseSnapshotAtStart)
                               ? (totalNow - calibState.pulseSnapshotAtStart) : 0;
    float pps = (float)deltaPulses / (float)calibState.durationSec;
    calibState.resultCoeff[v] = pps;
    valveHardClose(v);

    char msg[80];
    snprintf(msg, sizeof(msg), "Calibration V%d terminée: %lu pulses en %us", v+1, deltaPulses, calibState.durationSec);
    logSys(msg);

    int nextV = v + 1;
    if(nextV >= VANNE_COUNT){
        // Toutes les vannes ont été mesurées : finalise.
        calibFinish();
        return;
    }
    // Passe à la vanne suivante : ouvre SEULE, redémarre le chrono de mesure.
    calibState.currentValve = nextV;
    calibState.phaseStartMs = millis();
    calibState.pulseSnapshotAtStart = calibReadTotalPulses();
    valveHardOpen(nextV, CmdSource::WEB, calibState.durationSec);
}

// Construit le JSON d'état de calibration pour /api/calibration/status et
// pour l'inclusion dans buildStatusJson() (affichage live sans recharger
// la page, même après reconnexion WebSocket suite à un reload).
String calibStatusJson(){
    StaticJsonDocument<512> doc;
    const char* phaseStr = "idle";
    switch(calibState.phase){
        case CalibPhase::RUNNING: phaseStr = "running"; break;
        case CalibPhase::DONE:    phaseStr = "done";     break;
        case CalibPhase::ABORTED: phaseStr = "aborted";  break;
        case CalibPhase::FAILED:  phaseStr = "failed";   break;
        default:                  phaseStr = "idle";     break;
    }
    doc["phase"] = phaseStr;
    doc["currentValve"] = calibState.currentValve;
    doc["durationSec"] = calibState.durationSec;
    if(calibState.phase == CalibPhase::RUNNING){
        unsigned long elapsedMs = millis() - calibState.phaseStartMs;
        unsigned long remainMs = ((unsigned long)calibState.durationSec*1000UL > elapsedMs)
                                ? ((unsigned long)calibState.durationSec*1000UL - elapsedMs) : 0;
        doc["remainingSec"] = remainMs/1000UL;
        unsigned long totalNow = calibReadTotalPulses();
        unsigned long deltaPulses = (totalNow >= calibState.pulseSnapshotAtStart)
                                   ? (totalNow - calibState.pulseSnapshotAtStart) : 0;
        doc["livePulses"] = deltaPulses;
    }
    if(calibState.failReason[0]) doc["failReason"] = calibState.failReason;
    JsonArray res = doc.createNestedArray("results");
    for(int i=0;i<VANNE_COUNT;i++) res.add(calibState.resultCoeff[i]);
    JsonArray coeffs = doc.createNestedArray("flowCoeffs");
    for(int i=0;i<VANNE_COUNT;i++) coeffs.add(valveCons[i].flowCoeff);
    String out; serializeJson(doc, out);
    return out;
}

// ============================================================
// SECTION 8 — SCHEDULEMANAGER
// ============================================================

// Vérifie si un programme doit se déclencher maintenant
// Appelé périodiquement dans loop (réduit pour précision)
unsigned long lastSchedCheckMs = 0;
const unsigned long SCHED_CHECK_INTERVAL_MS = 5000UL; // vérifier toutes les 5s pour réduire délai

void schedCheck(){
    if(!timeIsSynced) return;
    unsigned long now = millis();
    if(now - lastSchedCheckMs < SCHED_CHECK_INTERVAL_MS) return;
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
            // Trigger if current time has reached scheduled hour:minute.
            // Allow a small window (>= scheduled time and < scheduled time + 65s)
            if(s.hour != curH) continue;
            // compute seconds since start of minute
            int curSec = ti.tm_sec;
            if(s.minute != curM && s.minute != ((curM - (curSec>65?1:0) + 60) % 60)) continue;
            if(s.minute != curM) {
                // If we are slightly past the minute due to check timing, allow trigger
                if(!( (curM == s.minute && curSec >=0) || (curM == (s.minute+1)%60 && curSec < 65) )) continue;
            }
            
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
                // Si bouton OFF: forcer la fermeture même si la vanne était forcée par Web/Prog
                valveHardClose(i);
                logAdd(i, "Fermée — bouton OFF (annule forçage)");
            } else {
                bool ok = valveHardOpen(i, CmdSource::PHYS_INPUT, sysConfig.manualForceSec);
                if(!ok) logAdd(i, "Forçage bouton ignoré (calibration en cours ou priorité supérieure)");
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
                        if(!timeIsSynced){
                            timeIsSynced = true;
                            ntpSyncedAtMs = millis();
                            logSys("Heure synchronisée via LoRa (premiere fois)");
                        } else {
                            logSys("Heure synchronisée via LoRa");
                        }
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
    // CORRECTIF (bug "TRem: -- °C" permanent) : ce champ avait été retiré
    // du STATUS lors d'un refactor précédent, mais le frontend (WebContent.h,
    // handleStatus()) lit toujours data.tempR pour l'afficher sur le dashboard.
    // Sans ce champ, la température distante reçue par LoRa n'était jamais
    // visible dans l'UI, même quand temperatureRemote contenait une valeur
    // valide. On le réintègre pour rétablir l'affichage.
    doc["tempR"]  = temperatureRemote;
    // add current local time for UI
    time_t nowt = nowEpoch();
    if(nowt) doc["time"] = (long)nowt;
    JsonArray arr = doc.createNestedArray("valves");
    uint16_t today = todayYMD();
    for(int i=0;i<VANNE_COUNT;i++){
        Valve& v = valves[i];
        JsonObject o = arr.createNestedObject();
        o["name"]        = v.name;
        o["state"]       = v.isOpen ? 1 : 0;
        o["source"]      = srcStr(v.source);
        o["remainingSec"]= v.remainingSec;
        o["openedAt"]    = (long)v.openedAt;
        o["totalOpenSec"]= v.totalOpenSec;
        // ── Conso par vanne (litres aujourd'hui + total cumulé)
        float litresToday = (valveCons[i].todayYmd == today)
                          ? (float)valveCons[i].todayPulses / PULSES_PER_LITRE
                          : 0.0f;
        float litresTotal = (float)valveCons[i].pulsesTotal / PULSES_PER_LITRE;
        o["litresToday"] = litresToday;
        o["litresTotal"] = litresTotal;
        o["pulsesToday"] = (valveCons[i].todayYmd == today) ? (long)valveCons[i].todayPulses : 0;
        o["pulsesTotal"] = (long)valveCons[i].pulsesTotal;
    }
    
    // ── Entrées/Sorties pour la page web E/S (similaire à OLED page 3)
    JsonArray ioOut = doc.createNestedArray("ioOut");
    for(int col=0; col<8; col++){
        if(col < 4){
            if(col < VANNE_COUNT) ioOut.add(digitalRead(VANNE_PINS[col]));
            else ioOut.add(-1);
        } else {
            int idx = col - 4;
            if(idx < (int)(sizeof(OUT_PINS)/sizeof(OUT_PINS[0]))) ioOut.add(digitalRead(OUT_PINS[idx]));
            else ioOut.add(-1);
        }
    }
    JsonArray ioLed = doc.createNestedArray("ioLed");
    for(int col=0; col<8; col++){
        if(col < VANNE_COUNT) ioLed.add(digitalRead(LEDVISU_PINS[col]));
        else ioLed.add(-1);
    }
    JsonArray ioIn = doc.createNestedArray("ioIn");
    for(int col=0; col<8; col++){
        if(col < VANNE_COUNT) ioIn.add(digitalRead(FORCE_INPUT_PINS[col]));
        else ioIn.add(-1);
    }
    // Pulse counter (persisted + current) and instantaneous flow
    unsigned long cnt;
    noInterrupts(); cnt = pulseCount; interrupts();
    unsigned long totalPulses = persistedPulseCount + cnt;
    doc["pulses"] = totalPulses;
    float litresTotal = (float)totalPulses / PULSES_PER_LITRE;
    doc["litres"] = litresTotal;
    // CORRECTIF : calcul de débit désormais partagé avec MQTT via computeFlowLpm()
    // (auparavant dupliqué ici avec des statics locales invisibles depuis mqttPublishState()).
    // On force 2 décimales via la chaîne (ArduinoJson tronque parfois les
    // .0 quand la valeur tombe pile sur un entier ; on sérialise en string
    // pour garantir la cohérence de l'affichage UI).
    {
        float f = computeFlowLpm(totalPulses);
        char buf[16];
        snprintf(buf, sizeof(buf), "%.2f", (double)f);
        doc["flow_lpm"] = buf;
    }
    // AMÉLIORATION : expose l'état de connexion MQTT pour affichage d'un badge
    // dans l'UI (à côté du badge WebSocket existant), pour que l'utilisateur
    // sache si la liaison Home Assistant fonctionne sans avoir à consulter
    // les logs série.
    doc["mqttConnected"] = mqttConnected;
    // AMÉLIORATION (calibration débit) : résumé léger de l'état de
    // calibration pour que l'UI puisse suivre la progression en direct via
    // WebSocket, y compris après un reload de page (l'état vit côté
    // firmware, pas côté navigateur — voir SECTION 7b). On ne renvoie ici
    // qu'un résumé ; le détail complet (résultats par vanne, coefficients)
    // est disponible via GET /api/calibration/status.
    if(calibState.phase == CalibPhase::RUNNING){
        JsonObject calib = doc.createNestedObject("calib");
        calib["phase"] = "running";
        calib["currentValve"] = calibState.currentValve;
        unsigned long elapsedMs = millis() - calibState.phaseStartMs;
        unsigned long totalMs = (unsigned long)calibState.durationSec*1000UL;
        unsigned long remainMs = (totalMs > elapsedMs) ? (totalMs - elapsedMs) : 0;
        calib["remainingSec"] = remainMs/1000UL;
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
// SECTION 11b — MQTT + Home Assistant Auto-Discovery
// ============================================================
//
// On publie au démarrage des messages "config" retained sous
// <prefix>/<component>/<nodeId>/<object_id>/config  (schéma HA officiel).
// HA scanne ces topics, crée les entités correspondantes, et les met à
// jour automatiquement dès qu'on publie sur le state_topic associé.
//
// Composants exposés :
//   - sensor       : temperature1, temperature_remote, pulse_total,
//                    litres_total, flow_lpm
//   - sensor (×N)  : valve_N_litres_today, valve_N_litres_total
//   - binary_sensor(×N) : valve_N (état ouvert/fermé)
//   - switch  (×N) : valve_N (commande via <cmd_topic>)
//
// Les commandes émises par HA (switch) arrivent sur
//   <prefix>/switch/<mqttId>/valve_N/set  (payload: ON / OFF)
// et sont routées vers valveHardOpen / valveHardClose.
//
// CORRECTIF IMPORTANT : le topic auquel l'ESP32 s'abonne dans
// onMqttConnect() DOIT être construit exactement de la même façon que le
// command_topic publié dans mqttPublishDiscovery() (mqttTopic("switch",oid)),
// soit <prefix>/switch/<mqttId>/<objId>/set. L'ancienne version utilisait
// mqttTopicNode() (= <prefix>/<mqttId>) pour l'abonnement, ce qui donnait
// un chemin totalement différent du command_topic réellement publié à HA :
// les clics sur les switches dans Home Assistant n'atteignaient alors
// jamais l'ESP32, sans aucune erreur visible (le message MQTT partait
// juste vers un topic non souscrit).

static String mqttTopic(const char* component, const char* objId){
    // ex: homeassistant/sensor/irrpro_hs3/temperature1
    String s;
    s.reserve(128);
    s = sysConfig.mqttPrefix;
    s += '/';
    s += component;
    s += '/';
    s += sysConfig.mqttId;
    s += '/';
    s += objId;
    return s;
}

static String mqttTopicNode(){
    String s;
    s.reserve(64);
    s = sysConfig.mqttPrefix;
    s += '/';
    s += sysConfig.mqttId;
    return s;
}

// ── Publication d'un message "config" retained
//
// CORRECTIF CRITIQUE (entités absentes de Home Assistant) :
// Le schéma MQTT Discovery de HA exige que le JSON de description soit
// publié sur <prefix>/<component>/<node>/<objId>/config — un topic DISTINCT
// du state_topic (<prefix>/<component>/<node>/<objId>). L'ancienne version
// publiait le JSON de config sur le MÊME topic que mqttPublishState() utilise
// pour la valeur (mqttTopic() sans suffixe). Résultat : le JSON discovery
// était immédiatement écrasé par la prochaine valeur numérique publiée
// (10s plus tard), donc HA ne voyait jamais de payload "config" valide et
// ne créait aucune entité, malgré l'arrivée correcte des valeurs sur les
// topics — exactement le symptôme observé (valeurs visibles dans
// l'explorateur MQTT, mais 0 entité dans Settings → Devices & Services).
static void mqttPublishConfig(const char* component, const char* objId, const String& payload){
    if(!mqttConnected) return;
    String topic = mqttTopic(component, objId) + "/config";
    // qos 0, retain true
    mqttClient.publish(topic.c_str(), 0, true, payload.c_str(), payload.length());
}

static String deviceJson(){
    StaticJsonDocument<256> doc;
    JsonArray ids = doc.createNestedArray("identifiers");
    ids.add(sysConfig.mqttId);
    doc["name"]         = String("IrrigPro ") + sysConfig.mqttId;
    doc["model"]        = "ESP32 IoCan";
    doc["manufacturer"] = "IrrigPro";
    doc["sw_version"]   = SOFT_REV;
    String out; serializeJson(doc, out);
    return out;
}

// Helper: injecte le bloc "device" dans un document (en parsant puis copiant).
// On ne peut pas directement affecter un StaticJsonDocument à un JsonObject
// dans ArduinoJson v6, on utilise donc un parse round-trip sur la chaîne.
static void injectDevice(JsonObject doc){
    StaticJsonDocument<256> d;
    if(deserializeJson(d, deviceJson()) == DeserializationError::Ok){
        JsonObject src = d.as<JsonObject>();
        JsonObject dst = doc.createNestedObject("device");
        for(JsonPair kv : src){
            dst[kv.key()] = kv.value();
        }
    }
}

static void mqttPublishDiscovery(){
    if(!mqttConnected) return;
    String nodeTopic = mqttTopicNode(); // pour avail_topic et command_topic racine

    // ── Capteurs température / conso globale
    struct SensDef { const char* obj; const char* name; const char* unit; const char* devClass; };
    SensDef defs[] = {
        {"temperature1",     "Température locale",       "°C",   "temperature"},
        {"temperature_remote","Température distante",     "°C",   "temperature"},
        {"pulse_total",      "Compteur pulses (total)",  "pulses",""},
        {"litres_total",     "Litres total",             "L",    "volume"},
        {"flow_lpm",         "Débit instantané",         "L/min","volume_flow_rate"},
    };
    for(size_t i=0;i<sizeof(defs)/sizeof(defs[0]);i++){
        StaticJsonDocument<512> doc;
        // CORRECTIF (dépréciation HA 2026.4) : "object_id" pour fixer l'entity_id
        // est déprécié par Home Assistant ; le remplacement officiel est
        // "default_entity_id" avec le préfixe de plateforme inclus (ex: "sensor.xxx").
        doc["name"]           = defs[i].name;
        doc["default_entity_id"] = "sensor." + String(sysConfig.mqttId) + "_" + defs[i].obj;
        doc["unique_id"]      = String(sysConfig.mqttId) + "_" + defs[i].obj;
        doc["state_topic"]    = mqttTopic("sensor", defs[i].obj);
        doc["availability_topic"] = nodeTopic + "/availability";
        doc["payload_available"]  = "online";
        doc["payload_not_available"] = "offline";
        if(defs[i].devClass[0]) doc["device_class"] = defs[i].devClass;
        if(defs[i].unit[0])     doc["unit_of_measurement"] = defs[i].unit;
        doc["state_class"]     = "measurement";
        injectDevice(doc.as<JsonObject>());
        String out; serializeJson(doc, out);
        mqttPublishConfig("sensor", defs[i].obj, out);
    }

    // ── Une entité par vanne : sensor (litres today+total) + binary_sensor + switch
    for(int v=0;v<VANNE_COUNT;v++){
        char objBuf[24];
        const char* vname = (valves[v].name[0] ? valves[v].name : (snprintf(objBuf,sizeof(objBuf),"Vanne %d",v+1), objBuf));

        // Sensor litres_today
        {
            char oid[24]; snprintf(oid,sizeof(oid),"valve_%d_litres_today",v);
            StaticJsonDocument<512> doc;
            doc["name"]           = String(vname) + " — litres aujourd'hui";
            doc["default_entity_id"] = "sensor." + String(sysConfig.mqttId) + "_" + oid;
            doc["unique_id"]      = String(sysConfig.mqttId) + "_" + oid;
            doc["state_topic"]    = mqttTopic("sensor", oid);
            doc["availability_topic"] = nodeTopic + "/availability";
            doc["payload_available"]  = "online";
            doc["payload_not_available"] = "offline";
            doc["unit_of_measurement"] = "L";
            doc["state_class"]    = "total_increasing";
            doc["device_class"]   = "volume";
            injectDevice(doc.as<JsonObject>());
            String out; serializeJson(doc, out);
            mqttPublishConfig("sensor", oid, out);
        }
        // Sensor litres_total
        {
            char oid[24]; snprintf(oid,sizeof(oid),"valve_%d_litres_total",v);
            StaticJsonDocument<512> doc;
            doc["name"]           = String(vname) + " — litres total";
            doc["default_entity_id"] = "sensor." + String(sysConfig.mqttId) + "_" + oid;
            doc["unique_id"]      = String(sysConfig.mqttId) + "_" + oid;
            doc["state_topic"]    = mqttTopic("sensor", oid);
            doc["availability_topic"] = nodeTopic + "/availability";
            doc["payload_available"]  = "online";
            doc["payload_not_available"] = "offline";
            doc["unit_of_measurement"] = "L";
            doc["state_class"]    = "total_increasing";
            doc["device_class"]   = "volume";
            injectDevice(doc.as<JsonObject>());
            String out; serializeJson(doc, out);
            mqttPublishConfig("sensor", oid, out);
        }
        // Binary sensor : ouvert/fermé
        {
            char oid[24]; snprintf(oid,sizeof(oid),"valve_%d",v);
            StaticJsonDocument<512> doc;
            doc["name"]           = String(vname) + " — état";
            doc["default_entity_id"] = "binary_sensor." + String(sysConfig.mqttId) + "_" + oid + "_state";
            doc["unique_id"]      = String(sysConfig.mqttId) + "_" + oid + "_state";
            doc["state_topic"]    = mqttTopic("binary_sensor", oid);
            doc["availability_topic"] = nodeTopic + "/availability";
            doc["payload_available"]  = "online";
            doc["payload_not_available"] = "offline";
            doc["payload_on"]    = "ON";
            doc["payload_off"]   = "OFF";
            injectDevice(doc.as<JsonObject>());
            String out; serializeJson(doc, out);
            mqttPublishConfig("binary_sensor", oid, out);
        }
        // Switch : commande on/off
        {
            char oid[24]; snprintf(oid,sizeof(oid),"valve_%d",v);
            StaticJsonDocument<512> doc;
            doc["name"]           = String(vname) + " — commande";
            doc["default_entity_id"] = "switch." + String(sysConfig.mqttId) + "_" + oid + "_switch";
            doc["unique_id"]      = String(sysConfig.mqttId) + "_" + oid + "_switch";
            doc["state_topic"]    = mqttTopic("switch", oid);
            doc["command_topic"]  = mqttTopic("switch", oid) + "/set";
            doc["availability_topic"] = nodeTopic + "/availability";
            doc["payload_available"]  = "online";
            doc["payload_not_available"] = "offline";
            doc["payload_on"]    = "ON";
            doc["payload_off"]   = "OFF";
            doc["retain"]        = false;
            injectDevice(doc.as<JsonObject>());
            String out; serializeJson(doc, out);
            mqttPublishConfig("switch", oid, out);
        }
    }
    Serial.println("[MQTT] Discovery publié");
}

// ── Publication de l'état complet (capteurs + vannes)
static void mqttPublishState(){
    if(!mqttConnected) return;
    // Calculs partagés
    unsigned long cnt;
    noInterrupts(); cnt = pulseCount; interrupts();
    unsigned long totalPulses = persistedPulseCount + cnt;
    float litresTotal = (float)totalPulses / PULSES_PER_LITRE;

    // Capteurs globaux
    auto pub = [](const char* comp, const char* oid, const String& payload){
        String topic = mqttTopic(comp, oid);
        mqttClient.publish(topic.c_str(), 0, true, payload.c_str(), payload.length());
    };
    pub("sensor","temperature1", String(temperature1,2));
    pub("sensor","temperature_remote", String(temperatureRemote,2));
    pub("sensor","pulse_total", String((unsigned long)totalPulses));
    pub("sensor","litres_total", String(litresTotal,2));
    // CORRECTIF : flow_lpm est désormais calculé via computeFlowLpm(), la même
    // fonction partagée utilisée par buildStatusJson() pour le WebSocket — au
    // lieu de republier 0.0 en permanence comme c'était le cas auparavant.
    pub("sensor","flow_lpm", String(computeFlowLpm(totalPulses),2));

    // Vannes
    uint16_t today = todayYMD();
    for(int v=0;v<VANNE_COUNT;v++){
        char oid[24];
        float litresToday = (valveCons[v].todayYmd == today)
                          ? (float)valveCons[v].todayPulses / PULSES_PER_LITRE
                          : 0.0f;
        float litresTotV = (float)valveCons[v].pulsesTotal / PULSES_PER_LITRE;
        snprintf(oid,sizeof(oid),"valve_%d_litres_today",v);
        pub("sensor", oid, String(litresToday,2));
        snprintf(oid,sizeof(oid),"valve_%d_litres_total",v);
        pub("sensor", oid, String(litresTotV,2));
        snprintf(oid,sizeof(oid),"valve_%d",v);
        pub("binary_sensor", oid, valves[v].isOpen ? "ON" : "OFF");
        pub("switch", oid, valves[v].isOpen ? "ON" : "OFF");
    }
}

// ── Handler des commandes switch venues de HA
static void mqttHandleMessage(char* topic, char* payload, size_t len){
    String t(topic);
    String p(payload, len);
    // topic attendu : <prefix>/switch/<mqttId>/valve_N/set
    int idxSlash = t.lastIndexOf('/');
    if(idxSlash < 0) return;
    String leaf = t.substring(idxSlash+1); // "set" attendu
    if(leaf != "set") return;
    int v = -1;
    // parser ".../valve_<N>" juste avant "/set"
    String base = t.substring(0, idxSlash); // retire /set
    int s = base.lastIndexOf('/');
    if(s < 0) return;
    String obj = base.substring(s+1); // ex: valve_2
    if(sscanf(obj.c_str(),"valve_%d",&v)!=1) return;
    if(v<0||v>=VANNE_COUNT) return;
    bool on = (p.indexOf("ON")>=0);
    if(on){
        valveHardOpen(v, CmdSource::WEB, sysConfig.maxOpenSec);
        logAdd(v, "Ouverte via Home Assistant");
    }else{
        valveHardClose(v);
        logAdd(v, "Fermée via Home Assistant");
    }
}

static void onMqttConnect(bool sessionPresent){
    mqttConnected = true;
    Serial.println("[MQTT] Connecté");
    // CORRECTIF : le topic de souscription doit être construit EXACTEMENT
    // comme le command_topic publié dans mqttPublishDiscovery(), c'est-à-dire
    // mqttTopic("switch", oid) + "/set" = <prefix>/switch/<mqttId>/<oid>/set.
    // L'ancienne version utilisait mqttTopicNode() + "/switch/+/set"
    // (= <prefix>/<mqttId>/switch/+/set), un chemin différent qui ne
    // correspondait à aucun message réellement publié par HA -> les
    // commandes de Home Assistant n'arrivaient jamais à l'ESP32.
    String cmdTopic = String(sysConfig.mqttPrefix) + "/switch/" + sysConfig.mqttId + "/+/set";
    mqttClient.subscribe(cmdTopic.c_str(), 0);
    Serial.print("[MQTT] Abonné à: "); Serial.println(cmdTopic);
    // Publication disponibilité
    String availTopic = mqttTopicNode() + "/availability";
    mqttClient.publish(availTopic.c_str(), 0, true, "online", 6);
    // Discovery + état initial
    mqttPublishDiscovery();
    mqttPublishState();
}

static void onMqttDisconnect(AsyncMqttClientDisconnectReason r){
    mqttConnected = false;
    // Libellé lisible de la raison — pratique quand le broker refuse la
    // connexion (auth invalide, version protocole incompatible, etc.) ou
    // quand l'ESP n'arrive simplement pas à joindre le port TCP.
    const char* reasonStr = "?";
    switch(r){
        case AsyncMqttClientDisconnectReason::TCP_DISCONNECTED:          reasonStr = "TCP_DISCONNECTED (broker injoignable ou port fermé)"; break;
        case AsyncMqttClientDisconnectReason::MQTT_UNACCEPTABLE_PROTOCOL_VERSION: reasonStr = "MQTT_PROTOCOL_VERSION"; break;
        case AsyncMqttClientDisconnectReason::MQTT_IDENTIFIER_REJECTED: reasonStr = "MQTT_IDENTIFIER_REJECTED"; break;
        case AsyncMqttClientDisconnectReason::MQTT_SERVER_UNAVAILABLE:  reasonStr = "MQTT_SERVER_UNAVAILABLE"; break;
        case AsyncMqttClientDisconnectReason::MQTT_MALFORMED_CREDENTIALS: reasonStr = "MQTT_MALFORMED_CREDENTIALS"; break;
        case AsyncMqttClientDisconnectReason::MQTT_NOT_AUTHORIZED:       reasonStr = "MQTT_NOT_AUTHORIZED (login/mot de passe refusés)"; break;
        case AsyncMqttClientDisconnectReason::ESP8266_NOT_ENOUGH_SPACE:  reasonStr = "NOT_ENOUGH_SPACE"; break;
        case AsyncMqttClientDisconnectReason::TLS_BAD_FINGERPRINT:       reasonStr = "TLS_BAD_FINGERPRINT"; break;
        default:                                                        reasonStr = "UNKNOWN"; break;
    }
    Serial.printf("[MQTT] Déconnecté (%d = %s)\n", (int)r, reasonStr);
}

static void mqttSetup(){
    if(!sysConfig.mqttEnabled){
        Serial.println("[MQTT] Désactivé dans la config — client non démarré");
        return;
    }
    Serial.printf("[MQTT] Démarrage: host='%s' port=%u user='%s' id='%s'\n",
                  sysConfig.mqttHost, (unsigned)sysConfig.mqttPort,
                  sysConfig.mqttUser, sysConfig.mqttId);

    // setServer() a un comportement différent selon qu'on lui passe une IP
    // numérique ou un hostname. Pour une IP (notre cas par défaut), on
    // utilise IPAddress() qui évite toute ambiguïté sur la signature et
    // contourne un bug connu d'AsyncMqttClient sur ESP32-S3 où le passage
    // d'un char* numérique peut être mal interprété comme un hostname,
    // provoquant une résolution DNS qui timeout silencieusement (= le
    // symptôme exact qu'on observe : TCP_DISCONNECTED sans message
    // d'erreur intermédiaire).
    IPAddress brokerIp;
    if(brokerIp.fromString(sysConfig.mqttHost)){
        Serial.printf("[MQTT] Broker IP parsée: %s\n", brokerIp.toString().c_str());
        mqttClient.setServer(brokerIp, sysConfig.mqttPort);
    } else {
        Serial.printf("[MQTT] Host non-IP, résolution DNS: %s\n", sysConfig.mqttHost);
        mqttClient.setServer(sysConfig.mqttHost, sysConfig.mqttPort);
    }

    // ── Test TCP brut avant de laisser AsyncMqttClient tenter sa chance.
    // Si même un client TCP standard ne peut pas joindre le port, alors
    // AsyncMqttClient n'a aucune chance non plus et on évite de flooder
    // les logs avec des TCP_DISCONNECTED en boucle. À l'inverse, si le
    // port répond ici mais qu'AsyncMqttClient échoue quand même, on
    // saura que le problème vient de la lib asynchrone et pas du réseau.
    Serial.printf("[MQTT] Test TCP brut vers %s:%u...\n",
                  brokerIp.toString().c_str(), (unsigned)sysConfig.mqttPort);
    {
        WiFiClient probe;
        probe.setTimeout(2000);
        bool tcpOk = probe.connect(brokerIp, sysConfig.mqttPort);
        if(tcpOk){
            Serial.println("[MQTT] Test TCP OK — broker joignable");
            probe.stop();
        } else {
            Serial.println("[MQTT] Test TCP ÉCHEC — broker injoignable depuis l'ESP32");
            Serial.println("[MQTT] Causes possibles: VLAN séparé, isolation AP, route manquante");
            Serial.println("[MQTT] Connexion MQTT abandonnée — pas de retry pendant 60s");
            // On bloque les retries pendant 60s pour éviter le spam
            lastMqttConnectAttemptMs = millis() + 45000UL;
            return;
        }
    }

    if(strlen(sysConfig.mqttUser)>0){
        mqttClient.setCredentials(sysConfig.mqttUser, sysConfig.mqttPass);
    }
    // CORRECTIF (Last Will Testament) : si la connexion TCP tombe sans
    // déconnexion MQTT propre (crash, coupure WiFi brutale), le broker
    // publiera automatiquement "offline" (retained) sur ce topic. Sans LWT,
    // Home Assistant continuait à afficher l'appareil comme "online" pour
    // toujours après un crash, car le topic availability restait figé sur
    // la dernière valeur retained ("online") publiée avant la coupure.
    String availTopic = mqttTopicNode() + "/availability";
    mqttClient.setWill(availTopic.c_str(), 0, true, "offline");
    mqttClient.setKeepAlive(60);
    mqttClient.setCleanSession(true);
    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onMessage([](char* topic, char* payload, AsyncMqttClientMessageProperties, size_t len, size_t, size_t){
        mqttHandleMessage(topic, payload, len);
    });
    // Sur ESP32-S3, AsyncMqttClient partage la pile AsyncTCP avec le
    // AsyncWebServer. Si on lance connect() immédiatement après avoir
    // enregistré tous les handlers, la connexion TCP sortante peut
    // être refusée silencieusement par la pile partagée. Test TCP brut
    // préalable réussi ci-dessus, mais AsyncMqttClient utilise un
    // canal asynchrone différent. On retente plusieurs fois avec délai
    // croissant pour donner à la pile le temps de se stabiliser.
    Serial.printf("[MQTT] WiFi status=%d, RSSI=%d, heap=%u — lancement connect()\n",
                  (int)WiFi.status(), WiFi.RSSI(), (unsigned)ESP.getFreeHeap());
    for(int attempt=0; attempt<3; attempt++){
        if(attempt > 0){
            Serial.printf("[MQTT] Retry connect() #%d après %d ms\n",
                          attempt+1, 500 * attempt);
            delay(500 * attempt);
        }
        mqttClient.connect();
        // Laisse au client 1500 ms pour établir le socket TCP et recevoir
        // le CONNACK. Si ça n'aboutit pas, on retente.
        for(int wait=0; wait<30; wait++){
            if(mqttConnected) break;
            delay(50);
        }
        if(mqttConnected){
            Serial.printf("[MQTT] ✓ Connecté dès la tentative #%d\n", attempt+1);
            break;
        }
    }
    if(!mqttConnected){
        Serial.println("[MQTT] ✗ Toutes les tentatives échouées — mqttLoop() retentera périodiquement");
    }
    lastMqttConnectAttemptMs = millis();
}

static void mqttLoop(){
    if(!sysConfig.mqttEnabled){ mqttConnected = false; return; }
    if(!mqttConnected && WiFi.status()==WL_CONNECTED){
        unsigned long now = millis();
        if(now - lastMqttConnectAttemptMs > MQTT_RECONNECT_MS){
            lastMqttConnectAttemptMs = now;
            Serial.println("[MQTT] Reconnexion…");
            mqttClient.connect();
        }
    }
    if(mqttConnected){
        unsigned long now = millis();
        if(now - lastMqttPubMs > MQTT_PUB_INTERVAL_MS){
            lastMqttPubMs = now;
            mqttPublishState();
        }
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
    doc["mqttEnabled"]  = sysConfig.mqttEnabled;
    doc["mqttHost"]     = sysConfig.mqttHost;
    doc["mqttPort"]     = sysConfig.mqttPort;
    doc["mqttUser"]     = sysConfig.mqttUser;
    doc["mqttPass"]     = sysConfig.mqttPass;
    doc["mqttPrefix"]   = sysConfig.mqttPrefix;
    doc["mqttId"]       = sysConfig.mqttId;
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
    // On utilise beginResponse_P() pour servir directement depuis PROGMEM,
    // SANS recopier la chaîne (~52 KB) dans une String heap. C'est essentiel
    // car req->send(200,"text/html",WEB_HTML) duplique la page en RAM et
    // peut faire échouer malloc() quand la heap est fragmentée (~138 KB
    // libres observés sur heltec-Iocan-HS3), donnant une page blanche
    // sans erreur côté navigateur (Content-Length incorrect / troncature).
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* req){
        Serial.println("[HTTP] GET /");
        const char* p = (const char*)WEB_HTML;
        const size_t total = strlen_P(p);
        AsyncWebServerResponse* resp = req->beginResponse_P(
            200, "text/html; charset=utf-8",
            (const uint8_t*)p, total);
        resp->addHeader("Cache-Control", "no-store");
        req->send(resp);
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
            // Fermer impérativement la vanne même si elle était forcée par une source
            valveHardClose(idx);
            logAdd(idx, "Fermée via Web (annule forçage)");
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

    // ── Compteur d'impulsions (GET /api/pulse)
    server.on("/api/pulse", HTTP_GET, [](AsyncWebServerRequest* req){
        // Lecture atomique
        unsigned long cnt;
        noInterrupts(); cnt = pulseCount; interrupts();
        char buf[128];
        float litres = (float)cnt / PULSES_PER_LITRE;
        snprintf(buf, sizeof(buf), "{\"pulses\":%lu,\"litres\":%.3f,\"pulses_per_litre\":%.1f}", cnt, litres, (double)PULSES_PER_LITRE);
        req->send(200, "application/json", String(buf));
    });

    // ── Reset compteur POST /api/pulse/reset
    server.on("/api/pulse/reset", HTTP_POST, [](AsyncWebServerRequest* req){
        // reset persisted and runtime counters
        noInterrupts(); pulseCount = 0; interrupts();
        persistedPulseCount = 0;
        pulseSave();
        // Reset aussi le suivi par vanne
        for(int v=0;v<VANNE_COUNT;v++){
            valveCons[v].pulsesTotal = 0;
            valveCons[v].todayPulses = 0;
            valveCons[v].todayYmd = todayYMD();
            valveCons[v].todayIdx = 0;
            // CORRECTIF répartition : remettre le carry à zéro aussi. Sans
            // ça, un résidu fractionnaire accumulé avant le reset (donc
            // basé sur d'anciens pulses déjà remis à zéro par ailleurs)
            // fausserait légèrement la toute première distribution après
            // le reset (léger à-coup, pas une perte de bilan global, mais
            // autant repartir propre puisque l'utilisateur attend un vrai
            // zéro partout).
            valveCons[v].carry = 0.0f;
            for(int d=0;d<CONS_HISTORY_DAYS;d++){
                valveCons[v].history[d].ymd = 0;
                valveCons[v].history[d].pulses = 0;
                valveCons[v].history[d].litres = 0;
            }
            valveConsSaveOne(v);
        }
        lastDistributedTotal = 0;
        req->send(200, "application/json", String("{\"ok\":true}"));
        logSys("Compteur impulsions + suivi par vanne remis a zero");
    });

    // ── Consommation par vanne (GET /api/consumption)
    server.on("/api/consumption", HTTP_GET, [](AsyncWebServerRequest* req){
        DynamicJsonDocument doc(2048);
        JsonArray arr = doc.createNestedArray("valves");
        uint16_t today = todayYMD();
        for(int v=0;v<VANNE_COUNT;v++){
            // Si on a changé de jour depuis le dernier check, on capture l'ancien jour dans l'historique
            // (déjà géré côté runtime par pulseDistribute, mais on aligne ici pour l'affichage immédiat)
            float litresTotal = (float)valveCons[v].pulsesTotal / PULSES_PER_LITRE;
            float litresToday = (valveCons[v].todayYmd == today)
                              ? (float)valveCons[v].todayPulses / PULSES_PER_LITRE
                              : 0.0f;
            JsonObject o = arr.createNestedObject();
            o["valve"]            = v;
            o["name"]             = valves[v].name;
            o["pulsesTotal"]      = valveCons[v].pulsesTotal;
            o["litresTotal"]      = litresTotal;
            o["todayYmd"]         = valveCons[v].todayYmd;
            o["pulsesToday"]      = valveCons[v].todayPulses;
            o["litresToday"]      = litresToday;
            // Coefficient de calibration débit actuel (pulses/seconde mesurés
            // seul, ou 1.0 par défaut si jamais calibré) — utile pour que
            // l'UI affiche les coefficients courants même hors calibration.
            o["flowCoeff"]        = valveCons[v].flowCoeff;
            JsonArray hist = o.createNestedArray("history");
            // On parcourt l'historique par ordre chronologique inverse (le plus récent d'abord)
            int start = (valveCons[v].todayIdx - 1 + CONS_HISTORY_DAYS) % CONS_HISTORY_DAYS;
            for(int k=0;k<CONS_HISTORY_DAYS;k++){
                int idx = (start - k + CONS_HISTORY_DAYS) % CONS_HISTORY_DAYS;
                const DayStat& ds = valveCons[v].history[idx];
                if(ds.ymd == 0) continue;
                JsonObject h = hist.createNestedObject();
                h["ymd"] = ds.ymd;
                h["pulses"] = ds.pulses;
                h["litres"] = ds.litres;
            }
        }
        String out; serializeJson(doc, out);
        req->send(200, "application/json", out);
    });

    // ── Calibration débit : démarrer POST /api/calibration/start {durationSec}
    //
    // Sécurité : calibStart() refuse si une vanne est déjà ouverte (peu
    // importe la source) — pas de fermeture automatique, conformément au
    // choix de sécurité maximale. La réponse {ok:false, reason:"..."} permet
    // à l'UI d'afficher le message d'erreur exact à l'utilisateur.
    server.on("/api/calibration/start", HTTP_POST, [](AsyncWebServerRequest* req){},
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
            StaticJsonDocument<128> doc;
            uint16_t durationSec = 60;
            if(len>0 && !deserializeJson(doc,data,len)){
                durationSec = doc["durationSec"] | 60;
            }
            bool ok = calibStart(durationSec);
            if(ok){
                jsonResp(req, "{\"ok\":true}");
            } else {
                String resp = "{\"ok\":false,\"reason\":\"" + String(calibState.failReason) + "\"}";
                jsonResp(req, resp, 409); // 409 Conflict : état actuel incompatible
            }
        }
    );

    // ── Calibration débit : annuler POST /api/calibration/abort
    server.on("/api/calibration/abort", HTTP_POST, [](AsyncWebServerRequest* req){
        calibAbort();
        jsonResp(req, "{\"ok\":true}");
    });

    // ── Calibration débit : statut GET /api/calibration/status
    // Utilisé par l'UI pour suivre la progression en polling (en complément
    // du résumé léger inclus dans le STATUS WebSocket), et pour retrouver
    // l'état exact après un reload de page puisque la calibration vit
    // entièrement côté firmware.
    server.on("/api/calibration/status", HTTP_GET, [](AsyncWebServerRequest* req){
        jsonResp(req, calibStatusJson());
    });

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
    //
    // Notes de conception :
    //  * Chaque champ n'est mis à jour QUE si la clé est présente et non
    //    nulle côté JSON. Sans ce garde-fou, un payload partiel (rare mais
    //    possible) écraserait silencieusement des champs valides.
    //  * Le mot de passe WiFi n'est mis à jour QUE si non vide (évite
    //    d'effacer un pass existant quand l'UI envoie "" pour "non modifié").
    //  * Quand le SSID ou le mot de passe WiFi changent réellement, on
    //    déclenche un reboot ~1 s après la réponse HTTP. Sans ça, le module
    //    WiFi garde l'ancien SSID en interne jusqu'au prochain reset manuel,
    //    et l'utilisateur voit "rien n'a changé même après reboot" parce que
    //    il n'a pas pensé à rebooter, OU le reboot manuel ne suffit pas si
    //    configSave() a échoué silencieusement (corrigé par les logs).
    server.on("/api/config", HTTP_POST, [](AsyncWebServerRequest* req){},
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
            StaticJsonDocument<1024> doc;
            DeserializationError err = deserializeJson(doc, data, len);
            if(err){
                Serial.printf("[CFG] POST /api/config: JSON parse error: %s (len=%u)\n",
                              err.c_str(), (unsigned)len);
                jsonResp(req, "{\"ok\":false,\"reason\":\"json-parse\"}", 400);
                return;
            }

            // Détecter un vrai changement de SSID/pass AVANT modification
            char prevSsid[32]; strlcpy(prevSsid, sysConfig.ssid, 32);
            char prevPass[64]; strlcpy(prevPass, sysConfig.wifiPass, 64);
            bool wifiChanged = false;

            // --- WiFi : SSID / pass
            const char* newSsid = doc["ssid"] | (const char*)nullptr;
            if(newSsid){
                strlcpy(sysConfig.ssid, newSsid, sizeof(sysConfig.ssid));
                if(strcmp(prevSsid, sysConfig.ssid) != 0) wifiChanged = true;
            }
            const char* newPass = doc["wifiPass"] | (const char*)nullptr;
            if(newPass && strlen(newPass) > 0){
                strlcpy(sysConfig.wifiPass, newPass, sizeof(sysConfig.wifiPass));
                if(strcmp(prevPass, sysConfig.wifiPass) != 0) wifiChanged = true;
            }

            // --- Autres champs système
            const char* v;
            if((v = doc["ntpServer"]   | (const char*)nullptr)) strlcpy(sysConfig.ntpServer, v, sizeof(sysConfig.ntpServer));
            if((v = doc["nodeId"]      | (const char*)nullptr)) strlcpy(sysConfig.nodeId,    v, sizeof(sysConfig.nodeId));
            // Les numériques : ArduinoJson `|` avec la valeur courante
            // préserve la valeur existante si la clé est absente. Pour les
            // champs que l'UI envoie TOUJOURS, on garde ce pattern ; pour
            // les flags qu'on veut pouvoir remettre à 0, on testerait
            // containsKey() à la place.
            sysConfig.tzOffset       = doc["tzOffset"]       | sysConfig.tzOffset;
            sysConfig.loraFreq       = doc["loraFreq"]       | sysConfig.loraFreq;
            sysConfig.loraPower      = doc["loraPower"]      | sysConfig.loraPower;
            sysConfig.irrigMode      = doc["irrigMode"]      | sysConfig.irrigMode;
            sysConfig.maxOpenSec     = doc["maxOpenSec"]     | sysConfig.maxOpenSec;
            sysConfig.manualForceSec = doc["manualForceSec"] | sysConfig.manualForceSec;

            // --- MQTT
            if(doc.containsKey("mqttEnabled")) sysConfig.mqttEnabled = doc["mqttEnabled"].as<bool>();
            if((v = doc["mqttHost"]   | (const char*)nullptr)) strlcpy(sysConfig.mqttHost,   v, sizeof(sysConfig.mqttHost));
            sysConfig.mqttPort   = doc["mqttPort"]   | sysConfig.mqttPort;
            if((v = doc["mqttUser"]   | (const char*)nullptr)) strlcpy(sysConfig.mqttUser,   v, sizeof(sysConfig.mqttUser));
            if((v = doc["mqttPass"]   | (const char*)nullptr)) strlcpy(sysConfig.mqttPass,   v, sizeof(sysConfig.mqttPass));
            if((v = doc["mqttPrefix"] | (const char*)nullptr)) strlcpy(sysConfig.mqttPrefix, v, sizeof(sysConfig.mqttPrefix));
            if((v = doc["mqttId"]     | (const char*)nullptr)) strlcpy(sysConfig.mqttId,     v, sizeof(sysConfig.mqttId));

            // --- Noms de vannes
            if(doc.containsKey("valveNames")){
                JsonArray arr = doc["valveNames"];
                for(int i=0; i<VANNE_COUNT && i<(int)arr.size(); i++){
                    const char* nm = arr[i] | (const char*)nullptr;
                    strlcpy(valves[i].name, nm ? nm : "", sizeof(valves[i].name));
                }
            }

            // --- Persistance NVS + log de ce qui a vraiment été écrit
            configSave();
            Serial.printf("[CFG] saved: ssid='%s' nodeId='%s' wifiChanged=%d\n",
                          sysConfig.ssid, sysConfig.nodeId, wifiChanged);

            // --- Redémarrage auto si WiFi a changé
            if(wifiChanged){
                jsonResp(req, "{\"ok\":true,\"restart\":true,\"reason\":\"wifi-changed\"}");
                Serial.println("[CFG] WiFi modifié → redémarrage dans 800 ms");
                static bool restartScheduled = false;
                if(!restartScheduled){
                    restartScheduled = true;
                    xTaskCreate([](void*){
                        vTaskDelay(pdMS_TO_TICKS(800));
                        Serial.println("[CFG] Redémarrage pour appliquer le nouveau WiFi");
                        ESP.restart();
                    }, "cfgReboot", 2048, NULL, 1, NULL);
                }
                return;
            }

            jsonResp(req, "{\"ok\":true}");
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

    // Catch-all : trace toute requête non routée pour faciliter le diag
    server.onNotFound([](AsyncWebServerRequest* req){
        Serial.printf("[HTTP] 404 %s %s\n",
            req->methodToString(), req->url().c_str());
        req->send(404, "text/plain", "not found");
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

    if (oledPage == 0) {
        if (captivePortalActive) {
            display.drawString(0,  0, "!! WiFi FAIL !!");
            display.drawString(0, 14, "AP: IrrigPro-Setup");
            display.drawString(0, 28, "-> 192.168.4.1");
            display.drawString(0, 42, "Config WiFi requise");
        } else {
            display.drawString(0,0, "-Reseau WiFi:");
            display.drawString(0,14, WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "Deconnecte");
            // Afficher date et heure si synchronisé
            struct tm ti;
            if(getLocalTime(&ti,5) && timeIsSynced){
                char dateBuf[24];
                char timeBuf[16];
                strftime(dateBuf, sizeof(dateBuf), "%Y-%m-%d", &ti);
                strftime(timeBuf, sizeof(timeBuf), "%H:%M:%S", &ti);
                display.drawString(0,28, String(dateBuf));
                display.drawString(0,40, String("Heure: ") + String(timeBuf));
                // restore visualization LEDs to reflect valve states
                for(int i=0;i<VANNE_COUNT;i++){
                    digitalWrite(LEDVISU_PINS[i], valves[i].isOpen ? HIGH : LOW);
                }
                // show temporary confirmation message after first sync
                if(ntpSyncedAtMs && millis() - ntpSyncedAtMs < 5000){
                    display.drawString(0,52, "NTP OK — vannes auto actives");
                }
            } else {
                display.drawString(0,28, "Heure: inconnue (NTP non sync)");
                // Clignoter LEDs de visualisation comme alarme (ne pas activer/fermer vannes)
                bool blink = ((millis() / 500) & 1) == 0;
                for(int i=0;i<VANNE_COUNT;i++){
                    digitalWrite(LEDVISU_PINS[i], blink ? HIGH : LOW);
                }
            }
        }
    } else if (oledPage == 1) {
        display.drawString(0,0, "Temperatures:");
        display.drawString(0,14, "Temp1: " + String(temperature1) + " C");
        // Temp2 removed; no longer display TempR on OLED (freed for pulse info below)
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
        // Afficher températures à la place des entrées
        String t1 = (isnan(temperature1) ? String("T1: -- °C") : String("T1:")+String(temperature1,2)+" °C");
        display.drawString(0,40, t1);
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
        // Ligne supplémentaire: compteur d'impulsions (connecté sur PB7)
        unsigned long cnt;
        noInterrupts(); cnt = pulseCount; interrupts();
        float litres = (float)cnt / PULSES_PER_LITRE;
        display.drawString(0, 48, String("Pulse:") + String(cnt) + " L:" + String(litres,3));
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

    // CORRECTIF v2 (bug "AP n'apparaît plus" persistant malgré le cycle
    // WIFI_OFF -> WIFI_AP_STA -> softAP() de la première tentative) :
    //
    // Le retry précédent partait de l'hypothèse "driver pas encore stabilisé
    // après l'échec STA, il faut juste attendre plus longtemps". Mais le
    // même échec ("set AP config failed") se reproduisait de façon
    // strictement identique sur les deux tentatives malgré le reset complet
    // entre les deux — ce qui élimine l'hypothèse d'un simple problème de
    // timing. La cause la plus probable, documentée sur ESP32-S3 (fragilité
    // du driver RF en mode mixte AP_STA après un échec de connexion STA,
    // contrairement à l'ESP32 classique où le même code fonctionne) :
    // démarrer DIRECTEMENT en WIFI_AP_STA est ce qui échoue, pas le délai
    // avant.
    //
    // Fix v2 : on démarre l'AP en mode WIFI_AP PUR (sans STA mélangé), on
    // vérifie que softAPIP() renvoie bien une adresse non-nulle (softAP()
    // peut renvoyer "succès" sans configuration IP correcte selon certains
    // retours terrain), et on ne bascule en WIFI_AP_STA (pour permettre le
    // scan WiFi depuis le portail) QU'APRÈS avoir confirmé que l'AP pur
    // fonctionne. Si même le mode AP pur échoue, le scan est simplement
    // désactivé pour cette session (le portail reste utilisable pour saisir
    // un SSID manuellement, voir le champ texte du formulaire).
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(300);

    IPAddress apIP(192, 168, 4, 1);

    bool apOk = false;
    bool scanAvailable = false;
    for(int attempt=0; attempt<2 && !apOk; attempt++){
        WiFi.mode(WIFI_AP);          // AP PUR — pas de mode mixte sur cette tentative
        delay(300);
        WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
        bool callOk = WiFi.softAP("IrrigPro-Setup");
        delay(300); // laisser le temps à l'event AP_START de se propager avant de lire l'IP
        IPAddress gotIP = WiFi.softAPIP();
        apOk = callOk && (gotIP != IPAddress(0,0,0,0));
        if(!apOk){
            Serial.printf("[CaptivePortal] Tentative %d échouée (callOk=%d, ip=%s)\n",
                          attempt+1, callOk, gotIP.toString().c_str());
            WiFi.mode(WIFI_OFF);
            delay(500);
        }
    }

    if(apOk){
        Serial.println("[CaptivePortal] AP pur (WIFI_AP) démarré avec succès");
        // L'AP fonctionne en mode pur : on tente maintenant la bascule vers
        // AP_STA pour permettre le scan WiFi depuis le portail. Si CETTE
        // transition spécifique re-casse l'AP (comportement observé sur
        // certains S3), on revient immédiatement en AP pur plutôt que de
        // laisser le portail dans un état cassé.
        WiFi.mode(WIFI_AP_STA);
        delay(300);
        IPAddress checkIP = WiFi.softAPIP();
        if(checkIP != IPAddress(0,0,0,0)){
            scanAvailable = true;
            Serial.println("[CaptivePortal] Bascule AP_STA OK — scan WiFi disponible");
        } else {
            Serial.println("[CaptivePortal] Bascule AP_STA a cassé l'AP — retour en WIFI_AP pur (sans scan)");
            WiFi.mode(WIFI_AP);
            delay(300);
            WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
            WiFi.softAP("IrrigPro-Setup");
            delay(300);
            scanAvailable = false;
        }
    } else {
        Serial.println("[CaptivePortal] ERREUR: AP pur a échoué après 2 tentatives — portail inaccessible");
        logSys("ERREUR: démarrage AP portail captif a échoué (2 tentatives, AP pur)");
    }

    // Lancer un scan asynchrone des réseaux WiFi en arrière-plan, UNIQUEMENT
    // si la bascule AP_STA a réussi (sinon WiFi.scanNetworks() exigerait
    // le mode STA actif, absent en WIFI_AP pur, et échouerait silencieusement
    // ou pire, redéclencherait l'instabilité qu'on vient d'éviter).
    captivePortalScanAvailable = scanAvailable;
    if(scanAvailable){
        Serial.println("[CaptivePortal] Lancement du scan WiFi...");
        WiFi.scanNetworks(true);
    } else {
        Serial.println("[CaptivePortal] Scan WiFi désactivé pour cette session (AP_STA indisponible)");
    }

    // DNS : renvoyer toutes les requêtes vers l'IP de l'AP (portail captif)
    dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    dnsServer.start(53, "*", apIP);

    Serial.print("[CaptivePortal] AP: IrrigPro-Setup  IP: "); Serial.println(WiFi.softAPIP().toString());
    logSys(apOk ? "Portail captif actif — SSID: IrrigPro-Setup"
                : "Portail captif démarré MAIS AP radio indisponible (voir log série)");

    // Affichage OLED
    display.clear();
    display.setFont(ArialMT_Plain_10);
    if(apOk){
        display.drawString(0, 0,  "WiFi: ECHEC config!");
        display.drawString(0, 14, "AP: IrrigPro-Setup");
        display.drawString(0, 28, "-> 192.168.4.1");
        display.drawString(0, 42, "Configurer le WiFi");
    } else {
        display.drawString(0, 0,  "WiFi: ECHEC config!");
        display.drawString(0, 14, "AP: ECHEC demarrage");
        display.drawString(0, 28, "Voir port serie");
        display.drawString(0, 42, "Redemarrage conseille");
    }
    display.display();

    // Servir la page de config sur le serveur dédié (port 80)
    auto servePortal = [](AsyncWebServerRequest* req) {
        String options = "";
        if(!captivePortalScanAvailable){
            options = "<option value=\"\">Scan indisponible — saisir le r&eacute;seau ci-dessous</option>";
        } else {
            int n = WiFi.scanComplete();
            Serial.printf("[CaptivePortal] GET / -> scanComplete=%d\n", n);
            if (n == WIFI_SCAN_FAILED) {
                options = "<option value=\"\">Recherche en cours...</option>";
            } else if (n > 0) {
                for (int i = 0; i < n; ++i) {
                    options += "<option value=\"" + WiFi.SSID(i) + "\">" + WiFi.SSID(i) + " (" + String(WiFi.RSSI(i)) + " dBm)</option>";
                }
            }
        }
        
        String html = String(FPSTR(CAPTIVE_HTML_1)) + options + String(FPSTR(CAPTIVE_HTML_2));
        req->send(200, "text/html", html);
    };

    captiveServer.on("/scan", HTTP_GET, [](AsyncWebServerRequest* req) {
        String options = "";
        if(!captivePortalScanAvailable){
            options = "<option value=\"\">Scan indisponible — saisir le r&eacute;seau ci-dessous</option>";
            req->send(200, "text/html", options);
            return;
        }
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

    // ── Auto-réparation NVS si partition saturée/corrompue.
    // Doit être appelé AVANT tout configLoad() pour qu'on reparte d'une
    // base saine. Ne fait rien si la NVS est OK.
    nvsSelfTestAndRecover();

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

    // ── Attacher interruption matérielle pour le compteur d'impulsions sur PB7 (index 7)
    // Détecte front descendant (capteur en pull-up) et anti-rebond simple
    {
        int pulsePin = FORCE_INPUT_PINS[7];
        // ISR
        attachInterrupt(digitalPinToInterrupt(pulsePin), pulse_isr, FALLING);
        Serial.printf("[PULSE] Interrupt attached on pin %d\n", pulsePin);
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
        // ── OTA
        otaSetup();
        // ── Web
        webSetup();
        // Délai avant MQTT : sur ESP32-S3, AsyncMqttClient partage la pile
        // TCP (AsyncTCP) avec AsyncWebServer. Si on lance MQTT immédiatement
        // après webSetup(), la pile TCP peut ne pas être encore stabilisée
        // et le premier connect() échoue en TCP_DISCONNECTED. 1 seconde
        // laisse le temps au serveur web de terminer ses initialisations
        // internes (handlers, ws, etc.) avant qu'on ouvre une nouvelle
        // connexion TCP sortante.
        delay(1000);
        // ── MQTT / Home Assistant
        mqttSetup();
    } else {
        Serial.println("\nWiFi FAIL — lancement portail captif");
        logSys("WiFi FAIL — portail captif actif");
        startCaptivePortal();
        // Ne pas démarrer otaSetup() et webSetup() en mode AP
    }

    // Charger compteur persistant
    pulseLoad();
    valveConsLoad();
    // Aligne la distribution sur le total connu au boot (persistant + runtime courant)
    {
        unsigned long cnt;
        noInterrupts(); cnt = pulseCount; interrupts();
        lastDistributedTotal = persistedPulseCount + cnt;
    }

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
        // Considerer valeurs <= -100 comme capteur déconnecté (-127 typique)
        bool v1 = !(isnan(temperature1) || temperature1 <= -100.0);
        if(v1 != temp1Valid){
            temp1Valid = v1;
            if(!temp1Valid) logSys("Temp1: capteur absent ou erreur");
            else { char b[40]; snprintf(b,40, "Temp1: %.2f C", temperature1); logSys(b); }
        } else if(temp1Valid){
            // log occasional stable reading (every 6th read ~1min) to avoid spam
            static int cnt1 = 0; cnt1 = (cnt1+1)%6; if(cnt1==0){ char b[40]; snprintf(b,40, "Temp1: %.2f C", temperature1); logSys(b); }
        }
    }

    // ── Portail captif
    if (captivePortalActive) {
        if (pendingRestart) {
            if (millis() - pendingRestartMs > 2000) {
                Serial.println("[CaptivePortal] Redémarrage de l'ESP...");
                ESP.restart();
            }
            return; // Bloquer l'irrigation uniquement pendant le redémarrage (les 2 secondes de délai)
        }
        dnsServer.processNextRequest();
    }

    // ── OTA
    if (!captivePortalActive) ArduinoOTA.handle();

    // ── MQTT / Home Assistant (auto-reconnexion + publication état)
    mqttLoop();

    // ── LoRa RX
    loraRxProcess();

    // ── LoRa TX périodique STATUS
    loraTxUpdate();

    // ── Entrées forçage manuel
    inputUpdate();

    // ── Vérifier si on doit sauvegarder le compteur (tous les N litres)
    {
        unsigned long cnt;
        noInterrupts(); cnt = pulseCount; interrupts();
        unsigned long totalPulses = persistedPulseCount + cnt;
        // ── Débit instantané lissé : sous-échantillonnage à ~1 Hz pour que
        // l'anneau glissant couvre bien la fenêtre FLOW_WINDOW_MS (5s).
        // Si on appelait flowUpdate() à chaque tour de loop (~10ms), l'anneau
        // de 16 échantillons ne couvrirait que ~160ms et la référence "il y
        // a 5s" tomberait toujours sur le plus vieil échantillon disponible,
        // soit ~160ms en arrière → fenêtre trop petite → débit très instable
        // ou figé à 0. Un échantillon par seconde × fenêtre 5s = 5 points
        // disponibles dans la fenêtre : stable et lissé.
        static unsigned long lastFlowUpdateMs = 0;
        if(millis() - lastFlowUpdateMs >= 1000){
            lastFlowUpdateMs = millis();
            flowUpdate(totalPulses);
        }
        // Distribue les pulses aux vannes ouvertes (CORRECTIF : accumulateur
        // d'erreur persistant par vanne, voir pulseDistribute())
        pulseDistribute(totalPulses);
        float totalLitres = (float)totalPulses / PULSES_PER_LITRE;
        static unsigned long lastSavedStep = 0;
        unsigned long step = (unsigned long)(floor(totalLitres / SAVE_LITRES_STEP));
        if(step > lastSavedStep){
            // sauvegarder le nombre de pulses correspondant à step * SAVE_LITRES_STEP
            unsigned long pulsesToSave = (unsigned long)(step * SAVE_LITRES_STEP * PULSES_PER_LITRE);
            persistedPulseCount = pulsesToSave;
            pulseSave();
            lastSavedStep = step;
            char b[80]; snprintf(b,80, "Pulse sauvegardees: %lu (%.1f L)", persistedPulseCount, (double)persistedPulseCount / PULSES_PER_LITRE);
            logSys(b);
        }
    }

    // ── Timers vannes (fermeture auto)
    valveUpdate();

    // ── Machine à états de calibration débit (avance pas à pas, non-bloquant)
    calibTick();

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
                // CORRECTIF (bug latent, même famille que "AP n'apparaît plus") :
                // ce bloc se contentait d'appeler WiFi.softAP(sysConfig.nodeId)
                // sans jamais démarrer captiveServer (routes /, /save, /scan).
                // Même quand softAP() réussissait, le SSID pouvait apparaître
                // mais RIEN ne répondait derrière (page blanche / timeout) —
                // et le SSID utilisé (sysConfig.nodeId) différait en plus de
                // celui du portail au boot ("IrrigPro-Setup"), ce qui aurait
                // dérouté l'utilisateur. On réutilise startCaptivePortal(),
                // qui contient désormais la séquence softAP sécurisée
                // (reset propre du driver, vérification du retour, retry)
                // ET démarre réellement le serveur HTTP du portail.
                Serial.println("WiFi perdu — démarrage portail captif (1min)");
                startCaptivePortal();
                captivePortalStartMs = now;
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

    // Réessayer NTP périodiquement tant que non synchronisé
    if(WiFi.status() == WL_CONNECTED && !timeIsSynced){
        const unsigned long NTP_RETRY_MS = 60000UL; // 60s
        if(now - lastNtpAttemptMs >= NTP_RETRY_MS){
            lastNtpAttemptMs = now;
            Serial.println("Tentative NTP (retry)...");
            timeInit();
        }
    }
}

// ============================================================
// FIN MainIocan.cpp
// ============================================================











#endif