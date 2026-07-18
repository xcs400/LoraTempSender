#pragma once
#ifdef IOCAN
// ============================================================
// MqttManager.h — MQTT + Home Assistant Auto-Discovery
// ============================================================
// Schéma HA : <prefix>/<component>/<mqttId>/<object_id>/config retained.
// Commandes HA : <prefix>/switch/<mqttId>/<objId>/set → valveHardOpen/Close.
//
// Mécanismes de fiabilité conservés (voir discussion) :
//   - Watchdog d'inactivité : armé uniquement sur PUBACK/SUBACK/message reçu
//     (un publish() QoS 0 ne prouve rien sur l'état réel de la session).
//   - Canari QoS 1 toutes les 45s (< keepAlive 60s) comme preuve de vie.
//   - Reconnexion préventive toutes les 15min (purge état zombie pile TCP).
//   - Backoff exponentiel 5/10/20/40/60s plafonné à 10 échecs.
//   - LWT sur <prefix>/<mqttId>/availability (static String — voir mqttSetup).
//   - IPAddress pour setServer() (char* numérique = bug hostname ESP32-S3).
//
// Simplifications apportées vs version précédente :
//   - Suppression du double sondage TCP manuel (setup + loop) : redondant
//     avec onMqttDisconnect(), qui donne déjà la raison de l'échec.
//   - mqttSetup() n'est plus bloquant (plus de boucle connect()+delay()) :
//     mqttLoop() gère déjà proprement les retries avec backoff.
//   - Discovery HA : entités "sensor" par vanne générées via une table
//     (ValveSensorDef[]) au lieu de 5 blocs JSON copiés-collés.
//   - Recovery MQTT (CONS_MQTT_ONLY) : comparaison + log factorisés dans
//     mqttRecoverIfGreater(), le rollover journalier via la macro
//     MQTT_ROLLOVER_TODAY() (les champs todayYmd sont des bitfields et
//     ne supportent pas une prise de référence).
//   - Reset des compteurs vanne / vanne-manuelle factorisé dans
//     resetValveCounters() / resetManualValveCounters().
// ============================================================

#include "Globals.h"
#include "LoggerManager.h"
#include "ConfigManager.h"
#include "FlowMeter.h"
#include "ValveManager.h"
#include "AlarmManager.h"

inline bool mqttDiscoveryPublished = false;

// État interne du module (canari, reconnexion préventive, anti-flood de logs)
inline unsigned long lastMqttCanaryMs         = 0;
inline unsigned long lastMqttForceReconnectMs = 0;
inline uint8_t       lastLoggedMqttFailureCount = 0;
inline unsigned long lastMqttDownHeartbeatMs  = 0;

#ifndef MQTT_DOWN_HEARTBEAT_MS
#define MQTT_DOWN_HEARTBEAT_MS 1800000UL // 30 min
#endif
#ifndef MQTT_CANARY_INTERVAL_MS
#define MQTT_CANARY_INTERVAL_MS 45000UL   // 45s < keepAlive 60s
#endif
#ifndef MQTT_FORCED_RECONNECT_MS
#define MQTT_FORCED_RECONNECT_MS 900000UL // 15 min
#endif

// ────────────────────────────────────────────────────────────
// Helpers topics / payloads
// ────────────────────────────────────────────────────────────

inline String mqttTopic(const char* component, const char* objId){
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

inline String mqttTopicNode(){
    String s;
    s.reserve(64);
    s = sysConfig.mqttPrefix;
    s += '/';
    s += sysConfig.mqttId;
    return s;
}

inline String mqttAvailabilityTopic(){
    return mqttTopicNode() + "/availability";
}

inline String mqttPayloadFloat(float value){
    if(!isfinite(value)) return "unknown";
    char buf[24];
    snprintf(buf, sizeof(buf), "%.2f", value);
    return String(buf);
}

// Publication d'un JSON de discovery retained sur <topic>/config
inline void mqttPublishConfig(const char* component, const char* objId, const String& payload){
    if(!mqttConnected) return;
    String topic = mqttTopic(component, objId) + "/config";
    mqttClient.publish(topic.c_str(), 0, true, payload.c_str(), payload.length());
}

inline String deviceJson(){
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

// Helper : injecte le bloc "device" dans un document via parse round-trip
// (ArduinoJson v6 ne permet pas d'affecter directement un StaticJsonDocument
// à un JsonObject).
inline void injectDevice(JsonObject doc){
    StaticJsonDocument<256> d;
    if(deserializeJson(d, deviceJson()) == DeserializationError::Ok){
        JsonObject src = d.as<JsonObject>();
        JsonObject dst = doc.createNestedObject("device");
        for(JsonPair kv : src){
            dst[kv.key()] = kv.value();
        }
    }
}

// Champs d'availability communs à toutes les entités HA
inline void fillAvailability(JsonObject doc){
    doc["availability_topic"]    = mqttAvailabilityTopic();
    doc["payload_available"]     = "online";
    doc["payload_not_available"] = "offline";
}

inline bool parseValveObjId(const String& objId, const char* suffix, int& vOut){
    if(!objId.startsWith("valve_") || !objId.endsWith(suffix)) return false;
    String mid = objId.substring(6, objId.length() - strlen(suffix));
    if(mid.length() == 0) return false;
    for(size_t i=0;i<mid.length();i++) if(!isDigit(mid[i])) return false;
    vOut = mid.toInt();
    return vOut >= 0 && vOut < VANNE_COUNT;
}

// ────────────────────────────────────────────────────────────
// Discovery HA — helpers génériques par type d'entité
// ────────────────────────────────────────────────────────────

// entityObjId : object_id/unique_id (peut différer du topicId, ex. switch)
inline void publishSensorDiscovery(const char* topicId, const char* entityObjId, const String& name,
                                    const char* unit, const char* deviceClass, const char* stateClass,
                                    const char* icon = nullptr){
    StaticJsonDocument<512> doc;
    doc["name"]      = name;
    doc["object_id"] = String(entityObjId);
    doc["unique_id"] = String(sysConfig.mqttId) + "_" + entityObjId;
    doc["state_topic"] = mqttTopic("sensor", topicId);
    fillAvailability(doc.as<JsonObject>());
    if(unit && unit[0])        doc["unit_of_measurement"] = unit;
    if(deviceClass && deviceClass[0]) doc["device_class"] = deviceClass;
    if(stateClass && stateClass[0])   doc["state_class"]  = stateClass;
    if(icon && icon[0])        doc["icon"] = icon;
    injectDevice(doc.as<JsonObject>());
    String out; serializeJson(doc, out);
    mqttPublishConfig("sensor", topicId, out);
}

inline void publishButtonDiscovery(const char* objId, const String& name, const char* icon = "mdi:water-minus"){
    StaticJsonDocument<512> doc;
    doc["name"]          = name;
    doc["object_id"]     = String(objId);
    doc["unique_id"]     = String(sysConfig.mqttId) + "_" + objId;
    doc["command_topic"] = mqttTopic("button", objId) + "/set";
    fillAvailability(doc.as<JsonObject>());
    doc["payload_press"] = "PRESS";
    if(icon && icon[0]) doc["icon"] = icon;
    injectDevice(doc.as<JsonObject>());
    String out; serializeJson(doc, out);
    mqttPublishConfig("button", objId, out);
}

inline void publishBinarySensorDiscovery(const char* topicId, const char* entityObjId, const String& name,
                                          const char* deviceClass = nullptr){
    StaticJsonDocument<512> doc;
    doc["name"]      = name;
    doc["object_id"] = String(entityObjId);
    doc["unique_id"] = String(sysConfig.mqttId) + "_" + entityObjId;
    doc["state_topic"] = mqttTopic("binary_sensor", topicId);
    fillAvailability(doc.as<JsonObject>());
    doc["payload_on"]  = "ON";
    doc["payload_off"] = "OFF";
    if(deviceClass && deviceClass[0]) doc["device_class"] = deviceClass;
    injectDevice(doc.as<JsonObject>());
    String out; serializeJson(doc, out);
    mqttPublishConfig("binary_sensor", topicId, out);
}

inline void publishSwitchDiscovery(const char* topicId, const char* entityObjId, const String& name){
    StaticJsonDocument<512> doc;
    doc["name"]          = name;
    doc["object_id"]     = String(entityObjId);
    doc["unique_id"]     = String(sysConfig.mqttId) + "_" + entityObjId;
    doc["state_topic"]   = mqttTopic("switch", topicId);
    doc["command_topic"] = mqttTopic("switch", topicId) + "/set";
    fillAvailability(doc.as<JsonObject>());
    doc["payload_on"]  = "ON";
    doc["payload_off"] = "OFF";
    doc["retain"]      = false;
    injectDevice(doc.as<JsonObject>());
    String out; serializeJson(doc, out);
    mqttPublishConfig("switch", topicId, out);
}

// Table des 5 sensors "vanne" partageant strictement le même schéma
// (object_id == topic id, component "sensor"). Le suffixe est ajouté au
// nom de la vanne pour le libellé HA et à "valve_N" pour le topic/objId.
struct ValveSensorDef {
    const char* suffix;      // ex: "_litres_today"
    const char* nameSuffix;  // ex: " — litres aujourd'hui"
    const char* unit;
    const char* deviceClass; // "" si aucun
    const char* stateClass;
};
static const ValveSensorDef VALVE_SENSOR_DEFS[] = {
    {"_litres_today", " — litres aujourd'hui",  "L",      "volume", "total_increasing"},
    {"_litres_total", " — litres total",        "L",      "volume", "total_increasing"},
    {"_pulses_today", " — pulses aujourd'hui",  "pulses", "",       "total_increasing"},
    {"_pulses_total", " — pulses total",        "pulses", "",       "total_increasing"},
    // Débit instantané par vanne, lissé sur FLOW_WINDOW_MS (4s), mis à
    // jour 1×/s par valveFlowUpdateAll() (voir ValveCons.h) et publié à
    // chaque mqttPublishState() (~10s). Utile pour détecter une vanne qui
    // goutte, ou avoir une vision temps réel dans HA hors WebSocket.
    {"_flow_lpm",      " — débit instantané",   "L/min",  "volume_flow_rate", "measurement"},
};

// ────────────────────────────────────────────────────────────
// Discovery HA — publication complète
// ────────────────────────────────────────────────────────────

inline void mqttPublishDiscovery(){
    if(!mqttConnected) return;

    // Capteurs globaux
    struct SensDef { const char* obj; const char* name; const char* unit; const char* devClass; };
    SensDef defs[] = {
        {"temperature1",      "Température locale",       "°C",   "temperature"},
        {"temperature_remote","Température distante",     "°C",   "temperature"},
        {"pulse_total",       "Compteur pulses (total)",  "pulses",""},
        {"litres_total",      "Litres total",             "L",    "volume"},
        {"flow_lpm",          "Débit instantané",         "L/min","volume_flow_rate"},
    };
    for(auto& d : defs){
        publishSensorDiscovery(d.obj, d.obj, d.name, d.unit, d.devClass, "measurement");
    }

    // Bouton de réinitialisation compteur global
    publishButtonDiscovery("pulse_total_reset", "Remise à zéro conso globale");

    // Compteurs "Vanne manuelle" (cf. Globals.h::ManualValveState) : 4
    // sensors (litres/pulses × today/total) + 1 bouton RAZ dédié. Permet à
    // HA de tracer l'usage manuel (tuyau, etc.) séparément des vannes
    // automatisées. L'alarme UNEXPECTED_FLOW reste active.
    publishSensorDiscovery("manual_valve_litres_today", "manual_valve_litres_today",
                            "VanneManuelle — litres aujourd'hui", "L", "volume", "total_increasing", "mdi:water-pump");
    publishSensorDiscovery("manual_valve_litres_total", "manual_valve_litres_total",
                            "VanneManuelle — litres total", "L", "volume", "total_increasing", "mdi:water-pump");
    publishSensorDiscovery("manual_valve_pulses_today", "manual_valve_pulses_today",
                            "VanneManuelle — pulses aujourd'hui", "pulses", "", "total_increasing", "mdi:pulse");
    publishSensorDiscovery("manual_valve_pulses_total", "manual_valve_pulses_total",
                            "VanneManuelle — pulses total", "pulses", "", "total_increasing", "mdi:pulse");
    // Bouton RAZ dédié "VanneManuelle" — n'affecte QUE ce compteur.
    publishButtonDiscovery("manual_valve_reset", "VanneManuelle — Remise à zéro");

    // Alarmes hydrauliques (3 entités : agrégat ON/OFF + code 0/1/2 + libellé)
    // Codes : 0=OK, 1=NO_FLOW (vanne ouverte mais pas de débit), 2=UNEXPECTED_FLOW (fuite)
    publishBinarySensorDiscovery("alarm", "alarm", "Alarme hydraulique", "problem");
    publishSensorDiscovery("alarm_code", "alarm_code", "Code alarme", nullptr, nullptr, "measurement", "mdi:water-alert");
    publishSensorDiscovery("alarm_message", "alarm_message", "Message alarme", nullptr, nullptr, nullptr, "mdi:message-alert-outline");

    // ── Une entité par vanne : 5 sensors (table) + binary_sensor + switch + bouton RAZ
    for(int v=0; v<VANNE_COUNT; v++){
        char objBuf[24];
        const char* vname = (valves[v].name[0] ? valves[v].name : (snprintf(objBuf,sizeof(objBuf),"V%d",v), objBuf));

        for(auto& sd : VALVE_SENSOR_DEFS){
            char oid[32];
            snprintf(oid, sizeof(oid), "valve_%d%s", v, sd.suffix);
            publishSensorDiscovery(oid, oid, String(vname) + sd.nameSuffix, sd.unit, sd.deviceClass, sd.stateClass);
        }

        char vOid[16]; snprintf(vOid, sizeof(vOid), "valve_%d", v);

        // Binary sensor : ouvert/fermé (object_id/unique_id = vOid + "_state")
        {
            String entityId = String(vOid) + "_state";
            publishBinarySensorDiscovery(vOid, entityId.c_str(), String(vname) + " — état");
        }
        // Switch : commande on/off (object_id/unique_id = vOid + "_switch")
        {
            String entityId = String(vOid) + "_switch";
            publishSwitchDiscovery(vOid, entityId.c_str(), String(vname) + " — commande");
        }
        // Bouton de RAZ pour cette vanne
        {
            char oid[32]; snprintf(oid, sizeof(oid), "valve_%d_reset_cons", v);
            publishButtonDiscovery(oid, String(vname) + " — Remise à zéro conso");
        }
    }
    // Serial.println("[MQTT] Discovery publié");
}

// ────────────────────────────────────────────────────────────
// Publication de l'état complet (capteurs + vannes)
// ────────────────────────────────────────────────────────────

inline void mqttPublishState(){
    if(!mqttConnected) return;
    // Calculs partagés
    unsigned long cnt;
    noInterrupts(); cnt = pulseCount; interrupts();
    unsigned long totalPulses = persistedPulseCount + cnt;
    float litresTotal = (float)totalPulses / PULSES_PER_LITRE;

    auto pub = [](const char* comp, const char* oid, const String& payload){
        String topic = mqttTopic(comp, oid);
        mqttClient.publish(topic.c_str(), 0, true, payload.c_str(), payload.length());
    };

    // Capteurs globaux
    pub("sensor","temperature1", mqttPayloadFloat(temperature1));
    pub("sensor","temperature_remote", mqttPayloadFloat(temperatureRemote));
    if (totalPulses > 4290000000UL) {
        logSys("[MQTT] Rejet ecriture pulse_total > 4.29G");
    } else {
        pub("sensor","pulse_total", String((unsigned long)totalPulses));
        pub("sensor","litres_total", String(litresTotal,2));
    }
    // flow_lpm calculé via computeFlowLpm(), la même fonction partagée
    // utilisée par buildStatusJson() pour le WebSocket.
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
        if (valveCons[v].todayPulses > 4290000000UL) {
            // Rejet silencieux
        } else {
            pub("sensor", oid, String(litresToday,2));
        }

        snprintf(oid,sizeof(oid),"valve_%d_litres_total",v);
        if (valveCons[v].pulsesTotal > 4290000000UL) {
            char b[80]; snprintf(b,80,"[MQTT] Rejet ecriture V%d_total > 4.29G", v+1);
            logSys(b);
        } else {
            pub("sensor", oid, String(litresTotV,2));
        }

        // Valeurs brutes (pulses) par vanne, miroir de valveCons[v].pulsesTotal
        // et valveCons[v].todayPulses. Mêmes garde-fous > 4.29G que pour la
        // version litres (un unsigned long 32 bits ne peut pas représenter
        // plus de ~4.29×10^9 ; HA recevrait sinon "4294967295" qui
        // empoisonnerait ses statistiques "total_increasing" durablement).
        snprintf(oid,sizeof(oid),"valve_%d_pulses_today",v);
        if (valveCons[v].todayPulses > 4290000000UL) {
            // Rejet silencieux
        } else {
            pub("sensor", oid, String((unsigned long)valveCons[v].todayPulses));
        }
        snprintf(oid,sizeof(oid),"valve_%d_pulses_total",v);
        if (valveCons[v].pulsesTotal > 4290000000UL) {
            char b[80]; snprintf(b,80,"[MQTT] Rejet ecriture V%d_pulses_total > 4.29G", v+1);
            logSys(b);
        } else {
            pub("sensor", oid, String((unsigned long)valveCons[v].pulsesTotal));
        }

        // Débit instantané par vanne (RAM, 1 Hz). Pas de garde >4.29G ici :
        // un float ne peut pas atteindre cette valeur (PULSES_PER_LITRE=741.2
        // → max mesurable ≈ 5300 L/min).
        snprintf(oid,sizeof(oid),"valve_%d_flow_lpm",v);
        pub("sensor", oid, String(valveCons[v].instantFlowLpm, 2));

        snprintf(oid,sizeof(oid),"valve_%d",v);
        pub("binary_sensor", oid, valves[v].isOpen ? "ON" : "OFF");
        pub("switch", oid, valves[v].isOpen ? "ON" : "OFF");
    }

    // Alarmes hydrauliques (retained pour que HA garde la dernière valeur
    // connue après reboot/déco broker — une alarme fuite doit rester
    // visible tant qu'elle n'a pas été acquittée).
    pub("binary_sensor", "alarm", alarmState.active ? "ON" : "OFF");
    pub("sensor",        "alarm_code",    String((unsigned)alarmState.code));
    pub("sensor",        "alarm_message", String(alarmState.msg));

    // Compteurs "Vanne manuelle" (cf. Globals.h::ManualValveState). Publiés
    // en retained pour que la recovery MQTT au prochain boot (mode
    // CONS_MQTT_ONLY) puisse ré-hydrater la RAM. Mêmes seuils de rejet
    // > 4.29G que les autres compteurs.
    float mvlLitresToday = (manualValveState.todayYmd == today)
                          ? (float)manualValveState.todayPulses / PULSES_PER_LITRE
                          : 0.0f;
    float mvlLitresTotal = (float)manualValveState.pulsesTotal / PULSES_PER_LITRE;
    pub("sensor", "manual_valve_litres_today", String(mvlLitresToday, 2));
    if(manualValveState.pulsesTotal > 4290000000UL){
        logSys("[MQTT] Rejet ecriture manual_valve_pulses_total > 4.29G");
    } else {
        pub("sensor", "manual_valve_litres_total", String(mvlLitresTotal, 2));
        pub("sensor", "manual_valve_pulses_total", String((unsigned long)manualValveState.pulsesTotal));
    }
    if(manualValveState.todayPulses > 4290000000UL){
        // Rejet silencieux
    } else {
        pub("sensor", "manual_valve_pulses_today", String((unsigned long)manualValveState.todayPulses));
    }
}

// ★ Canari de confirmation de vie de la session.
// Contrairement aux valeurs de capteurs publiées en QoS 0 par
// mqttPublishState() (aucune garantie de livraison, aucun accusé de
// réception possible), ce message dédié est publié en QoS 1. Le broker
// répond obligatoirement par un PUBACK, capté par onMqttPublishAck()
// ci-dessous — c'est cette confirmation, et elle seule, qui réarme le
// watchdog d'inactivité. Publié plus fréquemment que le keepAlive MQTT
// (60s) pour ne pas dépendre uniquement du PINGREQ natif de la lib.
inline void mqttSendCanary(){
    if(!mqttConnected) return;
    String availTopic = mqttAvailabilityTopic();
    mqttClient.publish(availTopic.c_str(), 1, true, "online", 6);
}

// ────────────────────────────────────────────────────────────
// Reset des compteurs (partagé entre commandes HA et endpoints Web)
// ────────────────────────────────────────────────────────────

inline void resetValveCounters(int v){
    valveCons[v].pulsesTotal = 0;
    valveCons[v].todayPulses = 0;
    valveCons[v].carry = 0.0f;
    for(int d=0;d<CONS_HISTORY_DAYS;d++) valveCons[v].history[d] = {0,0,0.0f};
}

inline void resetManualValveCounters(){
    manualValveState.pulsesTotal = 0;
    manualValveState.todayPulses = 0;
    manualValveState.todayYmd    = todayYMD();
    manualValveState.todayIdx    = 0;
    manualValveState.carry       = 0.0f;
    for(int d=0;d<CONS_HISTORY_DAYS;d++) manualValveState.history[d] = {0,0,0.0f};
}

// ────────────────────────────────────────────────────────────
// Recovery MQTT (mode CONS_MQTT_ONLY) — helpers
// ────────────────────────────────────────────────────────────

#ifdef CONS_MQTT_ONLY
// Compare une valeur restaurée depuis MQTT retained à l'état RAM courant.
// Un compteur ne peut que progresser entre deux sessions : une valeur MQTT
// plus basse que la RAM est forcément anormale et on garde la valeur locale.
// Retourne true (et logue) si mqttVal doit remplacer currentVal.
inline bool mqttRecoverIfGreater(unsigned long mqttVal, unsigned long currentVal, const char* label){
    if(mqttVal > currentVal){
        char b[140];
        snprintf(b, sizeof(b), "[RECOVERY] %s: RAM=%lu < MQTT=%lu — restauré",
                 label, currentVal, mqttVal);
        logSys(b);
        return true;
    }
    return false;
}

// NOTE : pas de helper par référence pour le rollover todayYmd/todayPulses
// (certains de ces champs sont des bitfields dans leurs structs — on peut
// leur assigner une valeur mais pas y lier une référence non-const). Le
// rollover est donc fait inline à chaque site d'appel via la macro
// ci-dessous, qui reste lisible tout en restant compatible bitfields.
// Sans cette étape AVANT la comparaison de recovery (todayYmd à 0 après un
// reset NVS, ou pointant sur une date antérieure), la condition d'égalité
// serait toujours fausse et la recovery ne se déclencherait jamais.
#define MQTT_ROLLOVER_TODAY(stateRef, today) \
    do { if((stateRef).todayYmd != (today)) { (stateRef).todayYmd = (today); (stateRef).todayPulses = 0; } } while(0)
#endif

// ────────────────────────────────────────────────────────────
// Handler des commandes switch venues de HA + récupération des retained
// ────────────────────────────────────────────────────────────

inline void mqttHandleMessage(char* topic, char* payload, size_t len){
    String t(topic);
    String p(payload, len);
    // Tout message REÇU = preuve réelle d'activité bidirectionnelle sur la
    // socket → reset du watchdog (contrairement à un simple publish()
    // sortant, ceci prouve que le broker nous parle encore).
    mqttLastActivityMs = millis();

#ifdef CONS_MQTT_ONLY
    // ── Phase de récupération au boot : fenêtre de 3s après connexion MQTT.
    // On lit les valeurs retained publiées lors de la session précédente et
    // on met à jour la RAM si MQTT > NVS. On ne traite pas comme commande
    // switch pour ne pas interférer.
    //
    // `pulse_total`/`litres_total` (compteur global) SONT récupérés depuis
    // MQTT retained au boot, au même titre que les compteurs par vanne : en
    // CONS_MQTT_ONLY rien n'est persisté en NVS pour le total (pulseLoad/
    // pulseSave sont des no-ops), donc MQTT retained EST la source de
    // vérité après chaque reboot.
    if(!mqttRecoveryDone){
        String sensorBase = String(sysConfig.mqttPrefix) + "/sensor/" + sysConfig.mqttId + "/";
        if(t.startsWith(sensorBase)){
            String objId = t.substring(sensorBase.length());
            float val = p.toFloat();
            uint16_t today = todayYMD();
            unsigned long cnt;

            int v = -1;
            if(objId == "pulse_total"){
                // Compteur global en pulses (source de vérité, plus précise
                // que litres_total qui n'a que 2 décimales).
                unsigned long mqttPulses = (unsigned long)(val + 0.5f);
                noInterrupts(); cnt = pulseCount; interrupts();
                unsigned long liveTotal = persistedPulseCount + cnt;
                if(mqttRecoverIfGreater(mqttPulses, liveTotal, "pulse_total")){
                    persistedPulseCount = mqttPulses;
                    // CRITIQUE : realigner lastDistributedTotal sur le total
                    // restauré, sinon pulseDistribute() verra un delta énorme
                    // (mqttPulses - 0) au prochain tour et attribuera TOUS
                    // ces pulses à la vanne manuelle (si aucune vanne auto
                    // n'est ouverte) — contaminant manualValveState.pulsesTotal.
                    lastDistributedTotal = mqttPulses + cnt;
                    // pas de pulseSave() : no-op en CONS_MQTT_ONLY.
                }
            } else if(objId == "litres_total"){
                // Fallback si pulse_total n'a pas été retenu (ex. valeur
                // > 4.29G publiée une fois, ou discovery régénérée sans).
                // Conversion L→pulses moins précise, mais toujours mieux
                // que de repartir de 0.
                unsigned long mqttPulses = (unsigned long)(val * PULSES_PER_LITRE + 0.5f);
                noInterrupts(); cnt = pulseCount; interrupts();
                unsigned long liveTotal = persistedPulseCount + cnt;
                if(mqttRecoverIfGreater(mqttPulses, liveTotal, "litres_total (fallback pulses)")){
                    persistedPulseCount = mqttPulses;
                    lastDistributedTotal = mqttPulses + cnt; // même raison que ci-dessus
                }
            } else if(parseValveObjId(objId, "_litres_total", v)){
                unsigned long mqttPulses = (unsigned long)(val * PULSES_PER_LITRE + 0.5f);
                char lbl[24]; snprintf(lbl,sizeof(lbl),"V%d litres_total",v+1);
                if(mqttRecoverIfGreater(mqttPulses, valveCons[v].pulsesTotal, lbl)){
                    valveCons[v].pulsesTotal = mqttPulses;
                    valveConsMarkDirty(v);
                }
            } else if(parseValveObjId(objId, "_litres_today", v)){
                unsigned long mqttPulses = (unsigned long)(val * PULSES_PER_LITRE + 0.5f);
                if(today != 0){
                    MQTT_ROLLOVER_TODAY(valveCons[v], today);
                    char lbl[24]; snprintf(lbl,sizeof(lbl),"V%d litres_today",v+1);
                    if(mqttRecoverIfGreater(mqttPulses, valveCons[v].todayPulses, lbl)){
                        valveCons[v].todayPulses = (uint32_t)mqttPulses;
                        valveConsMarkDirty(v);
                    }
                }
            } else if(parseValveObjId(objId, "_pulses_total", v)){
                // Valeur brute en pulses (pas de conversion depuis des litres).
                unsigned long mqttPulses = (unsigned long)(val + 0.5f);
                char lbl[24]; snprintf(lbl,sizeof(lbl),"V%d pulses_total",v+1);
                if(mqttRecoverIfGreater(mqttPulses, valveCons[v].pulsesTotal, lbl)){
                    valveCons[v].pulsesTotal = mqttPulses;
                    valveConsMarkDirty(v);
                }
            } else if(parseValveObjId(objId, "_pulses_today", v)){
                unsigned long mqttPulses = (unsigned long)(val + 0.5f);
                if(today != 0){
                    MQTT_ROLLOVER_TODAY(valveCons[v], today);
                    char lbl[24]; snprintf(lbl,sizeof(lbl),"V%d pulses_today",v+1);
                    if(mqttRecoverIfGreater(mqttPulses, valveCons[v].todayPulses, lbl)){
                        valveCons[v].todayPulses = (uint32_t)mqttPulses;
                        valveConsMarkDirty(v);
                    }
                }
            } else if(objId == "manual_valve_litres_total"){
                // ── Recovery "Vanne manuelle" (cf. Globals.h) ──
                unsigned long mqttPulses = (unsigned long)(val * PULSES_PER_LITRE + 0.5f);
                if(mqttRecoverIfGreater(mqttPulses, manualValveState.pulsesTotal, "VanneManuelle litres_total")){
                    manualValveState.pulsesTotal = mqttPulses;
                    manualValveDirty = true;
                }
            } else if(objId == "manual_valve_litres_today"){
                unsigned long mqttPulses = (unsigned long)(val * PULSES_PER_LITRE + 0.5f);
                if(today != 0){
                    MQTT_ROLLOVER_TODAY(manualValveState, today);
                    if(mqttRecoverIfGreater(mqttPulses, manualValveState.todayPulses, "VanneManuelle litres_today")){
                        manualValveState.todayPulses = (uint32_t)mqttPulses;
                        manualValveDirty = true;
                    }
                }
            } else if(objId == "manual_valve_pulses_total"){
                unsigned long mqttPulses = (unsigned long)(val + 0.5f);
                if(mqttRecoverIfGreater(mqttPulses, manualValveState.pulsesTotal, "VanneManuelle pulses_total")){
                    manualValveState.pulsesTotal = mqttPulses;
                    manualValveDirty = true;
                }
            } else if(objId == "manual_valve_pulses_today"){
                unsigned long mqttPulses = (unsigned long)(val + 0.5f);
                if(today != 0){
                    MQTT_ROLLOVER_TODAY(manualValveState, today);
                    if(mqttRecoverIfGreater(mqttPulses, manualValveState.todayPulses, "VanneManuelle pulses_today")){
                        manualValveState.todayPulses = (uint32_t)mqttPulses;
                        manualValveDirty = true;
                    }
                }
            }
            return; // message de récupération traité — pas une commande switch
        }
    }
#endif

    // topic attendu : <prefix>/<component>/<mqttId>/<objId>/set
    int idxSlash = t.lastIndexOf('/');
    if(idxSlash < 0) return;
    String leaf = t.substring(idxSlash+1); // "set" attendu
    if(leaf != "set") return;

    // Isoler le chemin de base et le composant (switch ou button)
    String base = t.substring(0, idxSlash); // ex: homeassistant/switch/irrpro_hs3/valve_0
    int s = base.lastIndexOf('/');
    if(s < 0) return;
    String obj = base.substring(s+1); // ex: valve_0 ou pulse_total_reset

    // Identifier composant
    String componentStr = sysConfig.mqttPrefix;
    bool isSwitch = t.startsWith(componentStr + "/switch/");
    bool isButton = t.startsWith(componentStr + "/button/");

    if(isSwitch) {
        int v = -1;
        if(sscanf(obj.c_str(),"valve_%d",&v)==1 && v>=0 && v<VANNE_COUNT){
            bool on = (p.indexOf("ON")>=0);
            if(on){
                valveHardOpen(v, CmdSource::WEB, sysConfig.maxOpenSec);
                logAdd(v, "Ouverte via Home Assistant");
            }else{
                valveHardClose(v);
                logAdd(v, "Fermée via Home Assistant");
            }
        }
    }
    else if(isButton && p.indexOf("PRESS")>=0) {
        if(obj == "pulse_total_reset") {
            noInterrupts(); pulseCount = 0; interrupts();
            persistedPulseCount = 0; // no-op effectif en CONS_MQTT_ONLY, conservé pour la version non-CONS_MQTT_ONLY
            for(int vv=0;vv<VANNE_COUNT;vv++){
                resetValveCounters(vv);
                valveConsSaveOne(vv);
            }
            // Reset aussi le compteur "Vanne manuelle" — le bouton
            // "pulse_total_reset" remet TOUT à zéro, par cohérence avec
            // l'endpoint Web /api/pulse/reset.
            resetManualValveCounters();
            manualValveFlushOne();
            pulseSave(); // no-op en CONS_MQTT_ONLY, persiste en mode normal
            lastDistributedTotal = 0;
            lastMqttPubMs = 0; // Force update MQTT
            logSys("Compteur global remis a zero via Home Assistant");
        }
        else if(obj == "manual_valve_reset"){
            // ── Reset dédié "Vanne manuelle" ── N'affecte QUE ce compteur
            // (et son rollover jour), pas le compteur global ni les vannes
            // automatisées.
            resetManualValveCounters();
            manualValveFlushOne();
            lastMqttPubMs = 0; // Force MAJ MQTT immédiate
            logSys("Compteur VanneManuelle remis a zero via Home Assistant");
        }
        else {
            int v = -1;
            if(sscanf(obj.c_str(),"valve_%d_reset_cons",&v)==1 && v>=0 && v<VANNE_COUNT){
                resetValveCounters(v);
                valveConsFlushOne(v);
                lastMqttPubMs = 0; // Force update MQTT
                logAdd(v, "Compteur conso remis a zero via HA");
            }
        }
    }
}

// Callback PUBACK : preuve fiable d'activité récente (canari QoS 1)
inline void onMqttPublishAck(uint16_t packetId){
    (void)packetId;
    mqttLastActivityMs = millis();
}

// Callback SUBACK : confirmation broker d'un abonnement
inline void onMqttSubscribeAck(uint16_t packetId, uint8_t qos){
    (void)packetId; (void)qos;
    mqttLastActivityMs = millis();
}

inline void onMqttConnect(bool sessionPresent){
    mqttConnected = true;
    mqttDiscoveryPublished = false;
    logSys("[MQTT] onMqttConnect callback reçu");
    unsigned long downtimeMs = (mqttDisconnectMs > 0) ? (millis() - mqttDisconnectMs) : 0;
    mqttConsecutiveFailures = 0;
    mqttLastActivityMs      = millis();
    lastMqttCanaryMs         = millis();
    lastMqttForceReconnectMs = millis();
    lastLoggedMqttFailureCount = 0;
    lastMqttDownHeartbeatMs    = millis();
    {
        char b[160];
        snprintf(b, sizeof(b), "[MQTT] ✓ Reconnecté (downtime ~%lu s, échecs consécutifs remis à 0)",
                 (unsigned long)(downtimeMs / 1000UL));
        logSys(b);
    }
    String cmdTopic = String(sysConfig.mqttPrefix) + "/switch/" + sysConfig.mqttId + "/+/set";
    mqttClient.subscribe(cmdTopic.c_str(), 0);
    String btnTopic = String(sysConfig.mqttPrefix) + "/button/" + sysConfig.mqttId + "/+/set";
    mqttClient.subscribe(btnTopic.c_str(), 0);
    String availTopic = mqttAvailabilityTopic();
    mqttClient.publish(availTopic.c_str(), 0, true, "online", 6);
#ifdef CONS_MQTT_ONLY
    // Recovery : s'abonne aux retained et ouvre la fenêtre 3s
    String recovPattern = String(sysConfig.mqttPrefix) + "/sensor/" + sysConfig.mqttId + "/+";
    mqttClient.subscribe(recovPattern.c_str(), 0);
    mqttRecoveryDone    = false;
    mqttRecoveryStartMs = millis();
    logSys("[CONS] Recovery MQTT démarrée (fenêtre 3s)");
#else
    mqttPublishDiscovery();
    mqttPublishState();
#endif
}

inline void onMqttDisconnect(AsyncMqttClientDisconnectReason r){
    mqttConnected = false;
    mqttDiscoveryPublished = false;
    {
        char tb[120];
        snprintf(tb, sizeof(tb), "[MQTT] onMqttDisconnect: reason=%d, %lu ms après",
                 (int)r, (unsigned long)(millis() - lastMqttConnectAttemptMs));
        logSys(tb);
    }
    lastMqttConnectAttemptMs = millis();
    mqttLastActivityMs = 0;
    mqttDisconnectMs = millis();
    if(mqttConsecutiveFailures < 10) mqttConsecutiveFailures++;

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
    {
        char b[200];
        snprintf(b, sizeof(b), "[MQTT] ✗ Déconnecté: %s (échecs=%u, prochain retry dans %lu s)",
                 reasonStr,
                 (unsigned)mqttConsecutiveFailures,
                 (unsigned long)(min(MQTT_BACKOFF_MAX_MS,
                                     MQTT_BACKOFF_MIN_MS * (1UL << min((int)mqttConsecutiveFailures-1, 4)))) / 1000UL);
        logSys(b);
    }
}

// ────────────────────────────────────────────────────────────
// Setup — non bloquant : configure le client et lance UNE tentative de
// connexion. mqttLoop() prend le relais avec son propre backoff en cas
// d'échec ; onMqttDisconnect() donne déjà la raison précise (plus besoin
// d'un sondage TCP manuel redondant ici).
// ────────────────────────────────────────────────────────────

inline void mqttSetup(){
    if(!sysConfig.mqttEnabled) return;

    // IPAddress évite le bug AsyncMqttClient ESP32-S3 où un char* numérique
    // est mal interprété comme hostname (DNS qui timeout silencieusement).
    IPAddress brokerIp;
    if(brokerIp.fromString(sysConfig.mqttHost)){
        mqttClient.setServer(brokerIp, sysConfig.mqttPort);
    } else {
        mqttClient.setServer(sysConfig.mqttHost, sysConfig.mqttPort);
    }

    mqttClient.setClientId(sysConfig.mqttId);
    if(strlen(sysConfig.mqttUser)>0){
        mqttClient.setCredentials(sysConfig.mqttUser, sysConfig.mqttPass);
    }

    // ★ CRITICAL : AsyncMqttClient ne COPIE pas la chaîne du topic LWT — il
    // stocke le pointeur c_str() tel quel. Une String locale serait détruite
    // à la fin de mqttSetup() et Mosquitto recevrait des déchets mémoire en
    // guise de topic → RST instantané. static String garantit que le buffer
    // reste vivant pour tous les connect() ultérieurs.
    static String availTopic;
    availTopic = mqttAvailabilityTopic();
    mqttClient.setWill(availTopic.c_str(), 0, true, "offline");

    mqttClient.setKeepAlive(60);
    mqttClient.setCleanSession(true);
    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onPublish(onMqttPublishAck);
    mqttClient.onSubscribe(onMqttSubscribeAck);
    mqttClient.onMessage([](char* topic, char* payload, AsyncMqttClientMessageProperties, size_t len, size_t, size_t){
        mqttHandleMessage(topic, payload, len);
    });

    {
        char b[160];
        snprintf(b, sizeof(b), "[MQTT] Setup: host=%s port=%u id=%s (heap=%u, RSSI=%d)",
                 sysConfig.mqttHost, (unsigned)sysConfig.mqttPort,
                 sysConfig.mqttId, (unsigned)ESP.getFreeHeap(), WiFi.RSSI());
        logSys(b);
    }

    // Une seule tentative non bloquante ici ; mqttLoop() gère le reste
    // (retry + backoff exponentiel) si elle échoue.
    mqttClient.connect();
    lastMqttConnectAttemptMs = millis();
}

// ────────────────────────────────────────────────────────────
// Loop
// ────────────────────────────────────────────────────────────

inline void mqttLoop(){
    if(!sysConfig.mqttEnabled){ mqttConnected = false; return; }

    unsigned long now = millis();
    bool wifiUp = (WiFi.status() == WL_CONNECTED);

    // ── RESET DU COMPTEUR D'ÉCHECS QUAND LE WIFI EST KO ──
    // Si on a une rafale d'échecs MQTT, c'est potentiellement à cause d'une
    // perte WiFi transitoire. Tant que le WiFi n'est pas rétabli, on
    // n'incrémente pas le compteur d'échecs (la pile TCP refusera toujours
    // le connect()). Sans ce reset, on accumulerait des échecs fantômes
    // pendant la coupure WiFi et on aurait un backoff de 60s APRÈS le
    // retour du WiFi — exactement le bug observé "ça reste KO toute la nuit".
    if(!wifiUp){
        mqttConsecutiveFailures = 0;
    }

    // ── WATCHDOG D'INACTIVITÉ ──
    // Seules les confirmations réelles (PUBACK/SUBACK/messages) réarment ce
    // timer, garantissant la détection d'une socket zombie.
    if(mqttConnected && mqttLastActivityMs > 0
       && (now - mqttLastActivityMs) > MQTT_WATCHDOG_MS){
        logSys("[MQTT] Watchdog inactivité (aucun PUBACK/SUBACK/RX) — reconnexion forcée");
        mqttClient.disconnect(true); // brutale, pour éviter de polluer la pile TCP asynchrone
        mqttConnected = false;
        mqttLastActivityMs = 0;
        lastMqttConnectAttemptMs = millis(); // redémarre le calcul du backoff
    }

    // Reconnexion préventive périodique (purge état zombie éventuel).
    if (mqttConnected && (now - lastMqttForceReconnectMs) > MQTT_FORCED_RECONNECT_MS) {
        lastMqttForceReconnectMs = now;
        logSys("[MQTT] Reconnexion préventive périodique (purge état zombie éventuel)");
        mqttClient.disconnect(true);
        mqttConnected = false;
        mqttLastActivityMs = 0;
        lastMqttConnectAttemptMs = millis();
    }

    if(!mqttConnected && wifiUp){
        // ── CALCUL DU DÉLAI DE RETRY (backoff exponentiel) ──
        // 1 échec : 5s ; 2 : 10s ; 3 : 20s ; 4 : 40s ; 5+ : 60s
        unsigned long backoffDelay = MQTT_RECONNECT_MS;
        if(mqttConsecutiveFailures > 0){
            unsigned long shift = min((unsigned long)mqttConsecutiveFailures - 1, 4UL);
            unsigned long candidate = MQTT_BACKOFF_MIN_MS * (1UL << shift);
            backoffDelay = min(candidate, MQTT_BACKOFF_MAX_MS);
        }
        if(now - lastMqttConnectAttemptMs > backoffDelay){
            lastMqttConnectAttemptMs = now;
            // Log uniquement lors d'une transition, pour ne pas spammer une
            // fois arrivé au backoff max (60s). Rappel discret toutes les
            // 30 min (MQTT_DOWN_HEARTBEAT_MS) sinon.
            if(mqttConsecutiveFailures != lastLoggedMqttFailureCount){
                lastLoggedMqttFailureCount = mqttConsecutiveFailures;
                lastMqttDownHeartbeatMs = now;
                char b[160];
                snprintf(b, sizeof(b), "[MQTT] Retry #%u (délai=%lu s, WiFi OK=%d, heap=%u)",
                         (unsigned)mqttConsecutiveFailures,
                         (unsigned long)(backoffDelay / 1000UL),
                         (int)wifiUp, (unsigned)ESP.getFreeHeap());
                logSys(b);
            } else if(now - lastMqttDownHeartbeatMs > MQTT_DOWN_HEARTBEAT_MS){
                lastMqttDownHeartbeatMs = now;
                char b[160];
                snprintf(b, sizeof(b), "[MQTT] Toujours déconnecté (échecs=%u, délai=%lu s, WiFi OK=%d)",
                         (unsigned)mqttConsecutiveFailures,
                         (unsigned long)(backoffDelay / 1000UL),
                         (int)wifiUp);
                logSys(b);
            }

            // Reconfigure au cas où l'IP du broker aurait changé (DHCP),
            // puis tente la connexion. onMqttDisconnect() nous donnera la
            // raison précise en cas d'échec (plus besoin d'un sondage TCP
            // manuel préalable, redondant avec ce callback).
            IPAddress brokerIp;
            if(brokerIp.fromString(sysConfig.mqttHost)){
                mqttClient.setServer(brokerIp, sysConfig.mqttPort);
            } else {
                mqttClient.setServer(sysConfig.mqttHost, sysConfig.mqttPort);
            }
            mqttClient.connect();
        }
    }

    if(mqttConnected){
        if(!mqttDiscoveryPublished){
            mqttDiscoveryPublished = true;
#ifdef CONS_MQTT_ONLY
            // En mode CONS_MQTT_ONLY on est encore abonné aux topics
            // sensor/+ pour la recovery : publier maintenant provoquerait un
            // auto-écho (le firmware recevrait son propre message retained
            // comme une "valeur de session précédente" et le retraiterait).
            // On diffère ce premier publish à la fin de la fenêtre de recovery.
            if(mqttRecoveryDone){
                mqttPublishDiscovery();
                mqttPublishState();
                lastMqttPubMs = now;
            }
#else
            mqttPublishDiscovery();
            mqttPublishState();
            lastMqttPubMs = now;
#endif
        }

#ifdef CONS_MQTT_ONLY
        // ── Fin de fenêtre de récupération (3 s après connexion) ──
        // On ferme la fenêtre, on flush les valeurs restaurées en NVS, on se
        // désabonne du pattern sensor (pour ne plus recevoir les états
        // futurs comme des retained de récupération), puis on publie la
        // discovery et l'état fusionné final.
        if(!mqttRecoveryDone && (now - mqttRecoveryStartMs >= 3000UL)){
            mqttRecoveryDone = true;
            String recovPattern = String(sysConfig.mqttPrefix) + "/sensor/" + sysConfig.mqttId + "/+";
            mqttClient.unsubscribe(recovPattern.c_str());
            valveConsFlushDirty();
            logSys("[CONS] Recovery MQTT terminée — NVS mis à jour");
            mqttPublishDiscovery();
            mqttPublishState();
            lastMqttPubMs = now;
        }
#endif
        if(now - lastMqttPubMs > MQTT_PUB_INTERVAL_MS){
            lastMqttPubMs = now;
#ifdef CONS_MQTT_ONLY
            // En mode recovery, ne pas publier l'état avant la fin de la
            // fenêtre (on ne voudrait pas de retained avec des valeurs
            // partielles sur le broker).
            if(mqttRecoveryDone) mqttPublishState();
#else
            mqttPublishState();
#endif
            // Pas de mqttLastActivityMs=millis() ici — un publish() QoS 0
            // sortant ne prouve rien. Voir watchdog ci-dessus et canari
            // ci-dessous pour la vraie confirmation.
        }

        // Canari QoS 1 périodique — seule source de confirmation continue de
        // vie de la session en dehors des messages reçus.
        if(now - lastMqttCanaryMs > MQTT_CANARY_INTERVAL_MS){
            lastMqttCanaryMs = now;
            mqttSendCanary();
        }
    }
}

#endif // IOCAN