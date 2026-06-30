#ifdef IOCAN

// ============================================================
// MainIocan.cpp — Contrôleur d'arrosage professionnel 8 vannes
// PlatformIO / ESP32 — RadioLib SX1262 / AsyncWebServer / OTA
// ÉVOLUTION DE : projet IOCAN chauffe-eau 4 vannes
//
// ============================================================
// DÉCOUPAGE EN MODULES (cette révision)
// ============================================================
// Le fichier faisait ~1800 lignes en un seul .cpp. Pour la maintenabilité,
// toute la logique a été déplacée dans src/Iocan/*.h, ce fichier ne
// contenant plus que les includes, setup() et loop(). Chaque module
// header-only correspond à une ancienne SECTION du fichier monolithique :
//
//   Globals.h/.cpp    : SECTION 1-3  — pins, structs, variables globales
//   LoggerManager.h   : SECTION 4    — journal circulaire
//   ConfigManager.h   : SECTION 5-5b — NVS (config, recovery, conso, schedules)
//   FlowMeter.h       : SECTION 3b   — débit lissé partagé WS/MQTT
//   TimeManager.h     : SECTION 6    — NTP + calendrier
//   ValveManager.h    : SECTION 7    — ouverture/fermeture vannes
//   ValveCons.h       : SECTION 3c/7b— distribution pulses + calibration débit
//   ScheduleManager.h : SECTION 8    — programmes calendrier
//   ManualInput.h     : SECTION 9    — entrées physiques de forçage
//   LoRaManager.h     : SECTION 10   — trames STATUS/CMD/TIME_SYNC
//   OtaManager.h      : SECTION 14   — OTA (ArduinoOTA)
//   WsManager.h       : SECTION 11   — buildStatusJson + broadcast WebSocket
//   MqttManager.h     : SECTION 11b  — MQTT + Home Assistant Discovery
//   WebManager.h      : SECTION 12   — routes REST
//   OledManager.h     : SECTION 13   — affichage écran OLED
//   CaptivePortal.h   : SECTION 14b  — portail captif WiFi
//
// Aucune logique n'a été modifiée lors de ce découpage : chaque fonction a
// été déplacée telle quelle dans le module correspondant. Voir l'en-tête
// de chaque module pour le détail des correctifs déjà appliqués
// précédemment (MQTT discovery/LWT, répartition des pulses par carry
// persistant, débit lissé, etc.) — ils restent inchangés.
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

#include "Iocan/Globals.h"
#include "Iocan/LoggerManager.h"
#include "Iocan/ConfigManager.h"
#include "Iocan/FlowMeter.h"
#include "Iocan/TimeManager.h"
#include "Iocan/ValveManager.h"
#include "Iocan/ValveCons.h"
#include "Iocan/ScheduleManager.h"
#include "Iocan/ManualInput.h"
#include "Iocan/LoRaManager.h"
#include "Iocan/OtaManager.h"
#include "Iocan/WsManager.h"
#include "Iocan/MqttManager.h"
#include "Iocan/WebManager.h"
#include "Iocan/OledManager.h"
#include "Iocan/CaptivePortal.h"

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
    // ── Initialise le cache d'état NVS (utilisé/free/total) pour l'UI.
    // Le cache sera rafraîchi périodiquement dans la loop(), mais on
    // prend un snapshot immédiat pour que la première requête /api/nvs/status
    // ou le premier broadcast WebSocket ait une valeur valide.
    nvsStatsRefresh();

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

    // ── Rafraîchissement périodique du cache d'état NVS (toutes les 5s).
    // On ne le fait PAS à chaque tour de loop (l'appel à nvs_get_stats()
    // parcourt toutes les entrées, ce n'est pas gratuit) mais toutes les
    // 5s c'est largement suffisant pour l'UI et reste imperceptible.
    static unsigned long lastNvsStatsMs = 0;
    if(millis() - lastNvsStatsMs >= 5000UL){
        lastNvsStatsMs = millis();
        nvsStatsRefresh();
    }

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
        // l'anneau glissant couvre bien la fenêtre FLOW_WINDOW_MS.
        // Si on appelait flowUpdate() à chaque tour de loop (~10ms), l'anneau
        // de 32 échantillons ne couvrirait que ~320ms et la référence "il y
        // a 30s" tomberait toujours sur le plus vieil échantillon disponible
        // → fenêtre trop petite → débit très instable ou figé à 0. Un
        // échantillon par seconde × fenêtre 30s = jusqu'à 30 points
        // disponibles dans la fenêtre : stable et lissé.
        static unsigned long lastFlowUpdateMs = 0;
        if(millis() - lastFlowUpdateMs >= 1000){
            lastFlowUpdateMs = millis();
            flowUpdate(totalPulses);
        }
        // Distribue les pulses aux vannes ouvertes (accumulateur d'erreur
        // persistant par vanne, voir ValveCons.h::pulseDistribute())
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

    // ── Flush NVS conso par vanne (toutes les 30 s) — cf. ConfigManager.h
    // pour le rationnel (anti-saturation NVS). Sans ce throttle, on flushe
    // ~600×/min et la partition sature en quelques heures.
    {
        static unsigned long lastValveConsFlushMs = 0;
        if(millis() - lastValveConsFlushMs >= 30000UL){
            lastValveConsFlushMs = millis();
            valveConsFlushDirty();
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
