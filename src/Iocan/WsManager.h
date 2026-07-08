#pragma once
#ifdef IOCAN
// ============================================================
// WsManager.h — WebSocket broadcast (buildStatusJson)
// ============================================================
// Correspond à la SECTION 11 du fichier d'origine.
//
// buildStatusJson() est LA fonction d'état centrale : elle alimente à la
// fois le broadcast WebSocket périodique (wsBroadcastStatus) et la
// réponse à la connexion d'un nouveau client (onWsEvent), et sert de
// modèle pour mqttPublishState() qui republie les mêmes données par
// topic MQTT (voir MqttManager.h).
// ============================================================

#include "Globals.h"
#include "ConfigManager.h"
#include "FlowMeter.h"
#include "ValveCons.h"

inline unsigned long lastWsBroadcastMs = 0;

inline String buildStatusJson(){
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
        // Débit instantané PAR VANNE (L/min), lissé sur FLOW_WINDOW_MS
        // (4s) par valveFlowUpdateAll() appelé à 1 Hz depuis loop().
        // Méthode identique au débit global : moyenne sur un anneau de
        // (timestamp, pulses) glissant — garantit que Σ instantFlowLpm[i]
        // reste ≈ flow_lpm global (±1 pulse de marge, dû au carry de
        // pulseDistribute()). On formate en string "%.2f" pour éviter
        // qu'ArduinoJson ne tronque les .0 quand la valeur tombe pile
        // sur un entier (cohérent avec ce qui est fait pour flow_lpm
        // global plus bas).
        {
            float f = valveCons[i].instantFlowLpm;
            char buf[16];
            snprintf(buf, sizeof(buf), "%.2f", (double)f);
            o["flow_lpm"] = buf;
        }
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
    // AMÉLIORATION (NVS) : expose le niveau de remplissage de la partition
    // NVS (utilisé / total + pourcentage) pour que l'UI puisse afficher
    // une jauge dans la page Configuration et alerter l'utilisateur si la
    // partition s'approche de la saturation. Les données proviennent du
    // cache rafraîchi périodiquement (toutes les 5s) par la loop() pour
    // éviter de marteler le driver NVS à chaque broadcast.
    {
        if(millis() - nvsStatsLastMs > 5000UL) nvsStatsRefresh();
        JsonObject nvs = doc.createNestedObject("nvs");
        nvs["used"]  = (long)nvsStatsCached.usedEntries;
        nvs["free"]  = (long)nvsStatsCached.freeEntries;
        nvs["total"] = (long)nvsStatsCached.totalEntries;
        if(nvsStatsCached.totalEntries > 0){
            int pct = (int)((100UL * nvsStatsCached.usedEntries) / nvsStatsCached.totalEntries);
            if(pct > 100) pct = 100;
            nvs["usedPct"] = pct;
        } else {
            nvs["usedPct"] = 0;
        }
    }
    // AMÉLIORATION (calibration débit) : résumé léger de l'état de
    // calibration pour que l'UI puisse suivre la progression en direct via
    // WebSocket, y compris après un reload de page (l'état vit côté
    // firmware, pas côté navigateur — voir ValveCons.h). On ne renvoie ici
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

inline void wsBroadcastStatus(){
    if(ws.count()==0) return;
    unsigned long now = millis();
    if(now - lastWsBroadcastMs < 1000) return;   // max 1×/s
    lastWsBroadcastMs = now;
    String json = buildStatusJson();
    ws.textAll(json);
}

inline void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len)
{
    if(type == WS_EVT_CONNECT){
        // Envoyer l'état complet à la connexion
        client->text(buildStatusJson());
    }
}

#endif // IOCAN
