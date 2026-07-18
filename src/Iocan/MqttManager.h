#pragma once
#ifdef IOCAN
// ============================================================
// MqttManager.h — MQTT + Home Assistant Auto-Discovery
// ============================================================
// Schéma HA : <prefix>/<component>/<mqttId>/<object_id>/config retained.
// Commandes HA : <prefix>/switch/<mqttId>/<objId>/set → valveHardOpen/Close.
//
// Mécanismes de fiabilité :
//   - Watchdog d'inactivité : armé uniquement sur PUBACK/SUBACK/message reçu
//     (un publish() QoS 0 ne prouve rien sur l'état réel de la session).
//   - Canari QoS 1 toutes les 45s (< keepAlive 60s) comme preuve de vie.
//   - Reconnexion préventive toutes les 15min (purge état zombie pile TCP).
//   - Backoff exponentiel 5/10/20/40/60s plafonné à 10 échecs.
//   - LWT sur <prefix>/<mqttId>/availability (static String — voir mqttSetup).
//   - IPAddress pour setServer() (char* numérique = bug hostname ESP32-S3).
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


inline bool parseValveObjId(const String& objId, const char* suffix, int& vOut){
    if(!objId.startsWith("valve_") || !objId.endsWith(suffix)) return false;
    String mid = objId.substring(6, objId.length() - strlen(suffix));
    if(mid.length() == 0) return false;
    for(size_t i=0;i<mid.length();i++) if(!isDigit(mid[i])) return false;
    vOut = mid.toInt();
    return vOut >= 0 && vOut < VANNE_COUNT;
}


inline void mqttPublishDiscovery(){
    if(!mqttConnected) return;
    String nodeTopic = mqttTopicNode();

    // Capteurs globaux
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
        doc["name"]           = defs[i].name;
        doc["object_id"]      = String(defs[i].obj);
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

    // Bouton de réinitialisation compteur global
    {
        StaticJsonDocument<512> doc;
        doc["name"]           = "Remise à zéro conso globale";
        doc["object_id"]      = "pulse_total_reset";
        doc["unique_id"]      = String(sysConfig.mqttId) + "_pulse_total_reset";
        doc["command_topic"]  = mqttTopic("button", "pulse_total_reset") + "/set";
        doc["availability_topic"] = nodeTopic + "/availability";
        doc["payload_available"]  = "online";
        doc["payload_not_available"] = "offline";
        doc["payload_press"]  = "PRESS";
        doc["icon"]           = "mdi:water-minus";
        injectDevice(doc.as<JsonObject>());
        String out; serializeJson(doc, out);
        mqttPublishConfig("button", "pulse_total_reset", out);
    }

    // Compteurs "Vanne manuelle" (cf. Globals.h::ManualValveState) :
    // 4 sensors (litres/pulses × today/total) + 1 button RAZ dédié.
    // Permet à HA de tracer l'usage manuel (tuyau, etc.) séparément des
    // vannes automatisées. L'alarme UNEXPECTED_FLOW reste active.
    {
        struct MVS { const char* obj; const char* name; const char* unit; bool isLitres; };
        MVS mvs[] = {
            {"manual_valve_litres_today", "VanneManuelle — litres aujourd'hui", "L",     true},
            {"manual_valve_litres_total", "VanneManuelle — litres total",        "L",     true},
            {"manual_valve_pulses_today", "VanneManuelle — pulses aujourd'hui",  "pulses",false},
            {"manual_valve_pulses_total", "VanneManuelle — pulses total",        "pulses",false},
        };
        for(auto& m : mvs){
            StaticJsonDocument<512> doc;
            doc["name"]         = m.name;
            doc["object_id"]    = String(m.obj);
            doc["unique_id"]    = String(sysConfig.mqttId) + "_" + m.obj;
            doc["state_topic"]  = mqttTopic("sensor", m.obj);
            doc["availability_topic"] = nodeTopic + "/availability";
            doc["payload_available"]  = "online";
            doc["payload_not_available"] = "offline";
            doc["unit_of_measurement"] = m.unit;
            doc["state_class"]  = "total_increasing";
            if(m.isLitres){
                doc["device_class"] = "volume";
                doc["icon"] = "mdi:water-pump";
            } else {
                doc["icon"] = "mdi:pulse";
            }
            injectDevice(doc.as<JsonObject>());
            String out; serializeJson(doc, out);
            mqttPublishConfig("sensor", m.obj, out);
        }
    }
    // Bouton RAZ dédié "VanneManuelle" — n'affecte QUE ce compteur.
    {
        const char* objId = "manual_valve_reset";
        StaticJsonDocument<512> doc;
        doc["name"]           = "VanneManuelle — Remise à zéro";
        doc["object_id"]      = String(objId);
        doc["unique_id"]      = String(sysConfig.mqttId) + "_" + objId;
        doc["command_topic"]  = mqttTopic("button", objId) + "/set";
        doc["availability_topic"] = nodeTopic + "/availability";
        doc["payload_available"]  = "online";
        doc["payload_not_available"] = "offline";
        doc["payload_press"]  = "PRESS";
        doc["icon"]           = "mdi:water-minus";
        injectDevice(doc.as<JsonObject>());
        String out; serializeJson(doc, out);
        mqttPublishConfig("button", objId, out);
    }

    // Alarmes hydrauliques (3 entités : agrégat ON/OFF + code 0/1/2 + libellé)
    // Codes : 0=OK, 1=NO_FLOW (vanne ouverte mais pas de débit), 2=UNEXPECTED_FLOW (fuite)
    {
        // binary_sensor : alarme active
        StaticJsonDocument<512> doc;
        doc["name"]           = "Alarme hydraulique";
        doc["object_id"]      = "alarm";
        doc["unique_id"]      = String(sysConfig.mqttId) + "_alarm";
        doc["state_topic"]    = mqttTopic("binary_sensor", "alarm");
        doc["availability_topic"] = nodeTopic + "/availability";
        doc["payload_available"]  = "online";
        doc["payload_not_available"] = "offline";
        doc["payload_on"]     = "ON";
        doc["payload_off"]    = "OFF";
        doc["device_class"]   = "problem";
        injectDevice(doc.as<JsonObject>());
        String out; serializeJson(doc, out);
        mqttPublishConfig("binary_sensor", "alarm", out);
    }
    {
        // sensor : code d'alarme
        StaticJsonDocument<512> doc;
        doc["name"]           = "Code alarme";
        doc["object_id"]      = "alarm_code";
        doc["unique_id"]      = String(sysConfig.mqttId) + "_alarm_code";
        doc["state_topic"]    = mqttTopic("sensor", "alarm_code");
        doc["availability_topic"] = nodeTopic + "/availability";
        doc["payload_available"]  = "online";
        doc["payload_not_available"] = "offline";
        doc["state_class"]    = "measurement";
        doc["icon"]           = "mdi:water-alert";
        injectDevice(doc.as<JsonObject>());
        String out; serializeJson(doc, out);
        mqttPublishConfig("sensor", "alarm_code", out);
    }
    {
        // sensor : libellé court de l'alarme
        StaticJsonDocument<512> doc;
        doc["name"]           = "Message alarme";
        doc["object_id"]      = "alarm_message";
        doc["unique_id"]      = String(sysConfig.mqttId) + "_alarm_message";
        doc["state_topic"]    = mqttTopic("sensor", "alarm_message");
        doc["availability_topic"] = nodeTopic + "/availability";
        doc["payload_available"]  = "online";
        doc["payload_not_available"] = "offline";
        doc["icon"]           = "mdi:message-alert-outline";
        injectDevice(doc.as<JsonObject>());
        String out; serializeJson(doc, out);
        mqttPublishConfig("sensor", "alarm_message", out);
    }

    // ── Une entité par vanne : sensor (litres today+total) + binary_sensor + switch
    for(int v=0;v<VANNE_COUNT;v++){
        char objBuf[24];
        const char* vname = (valves[v].name[0] ? valves[v].name : (snprintf(objBuf,sizeof(objBuf),"V%d",v), objBuf));

        // Sensor litres_today
        {
            char oid[24]; snprintf(oid,sizeof(oid),"valve_%d_litres_today",v);
            StaticJsonDocument<512> doc;
            doc["name"]           = String(vname) + " — litres aujourd'hui";
            doc["object_id"]      = String(oid);
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
            doc["object_id"]      = String(oid);
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
        // Sensor pulses_today (valeur brute, sans conversion en litres)
        // Utile pour le debug et les automations qui veulent raisonner
        // directement en nombre d'impulsions (notamment quand la valeur
        // convertie en litres est suspecte — arrondi, overflow, etc.).
        {
            char oid[24]; snprintf(oid,sizeof(oid),"valve_%d_pulses_today",v);
            StaticJsonDocument<512> doc;
            doc["name"]           = String(vname) + " — pulses aujourd'hui";
            doc["object_id"]      = String(oid);
            doc["unique_id"]      = String(sysConfig.mqttId) + "_" + oid;
            doc["state_topic"]    = mqttTopic("sensor", oid);
            doc["availability_topic"] = nodeTopic + "/availability";
            doc["payload_available"]  = "online";
            doc["payload_not_available"] = "offline";
            doc["unit_of_measurement"] = "pulses";
            doc["state_class"]    = "total_increasing";
            injectDevice(doc.as<JsonObject>());
            String out; serializeJson(doc, out);
            mqttPublishConfig("sensor", oid, out);
        }
        // Sensor pulses_total (valeur brute, sans conversion en litres)
        {
            char oid[24]; snprintf(oid,sizeof(oid),"valve_%d_pulses_total",v);
            StaticJsonDocument<512> doc;
            doc["name"]           = String(vname) + " — pulses total";
            doc["object_id"]      = String(oid);
            doc["unique_id"]      = String(sysConfig.mqttId) + "_" + oid;
            doc["state_topic"]    = mqttTopic("sensor", oid);
            doc["availability_topic"] = nodeTopic + "/availability";
            doc["payload_available"]  = "online";
            doc["payload_not_available"] = "offline";
            doc["unit_of_measurement"] = "pulses";
            doc["state_class"]    = "total_increasing";
            injectDevice(doc.as<JsonObject>());
            String out; serializeJson(doc, out);
            mqttPublishConfig("sensor", oid, out);
        }
        // Sensor instant_flow_lpm : débit instantané PAR VANNE, lissé sur
        // FLOW_WINDOW_MS (4s) — publié à chaque mqttPublishState() (~10s)
        // via valveCons[v].instantFlowLpm (mis à jour 1×/s par
        // valveFlowUpdateAll() dans ValveCons.h). Utile pour détecter
        // une vanne qui goutte (débit non nul alors qu'elle est fermée),
        // ou pour avoir une vision en temps réel dans HA même quand le
        // WebSocket n'est pas ouvert. Voir buildStatusJson() côté
        // WebSocket qui expose la même valeur via valves[i].flow_lpm.
        {
            char oid[24]; snprintf(oid,sizeof(oid),"valve_%d_flow_lpm",v);
            StaticJsonDocument<512> doc;
            doc["name"]           = String(vname) + " — débit instantané";
            doc["object_id"]      = String(oid);
            doc["unique_id"]      = String(sysConfig.mqttId) + "_" + oid;
            doc["state_topic"]    = mqttTopic("sensor", oid);
            doc["availability_topic"] = nodeTopic + "/availability";
            doc["payload_available"]  = "online";
            doc["payload_not_available"] = "offline";
            doc["unit_of_measurement"] = "L/min";
            doc["state_class"]    = "measurement";
            doc["device_class"]   = "volume_flow_rate";
            injectDevice(doc.as<JsonObject>());
            String out; serializeJson(doc, out);
            mqttPublishConfig("sensor", oid, out);
        }
        // Binary sensor : ouvert/fermé
        {
            char oid[24]; snprintf(oid,sizeof(oid),"valve_%d",v);
            StaticJsonDocument<512> doc;
            doc["name"]           = String(vname) + " — état";
            doc["object_id"]      = String(oid) + "_state";
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
            doc["object_id"]      = String(oid) + "_switch";
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
        // Bouton de RAZ pour cette vanne
        {
            char oid[32]; snprintf(oid,sizeof(oid),"valve_%d_reset_cons",v);
            StaticJsonDocument<512> doc;
            doc["name"]           = String(vname) + " — Remise à zéro conso";
            doc["object_id"]      = String(oid);
            doc["unique_id"]      = String(sysConfig.mqttId) + "_" + oid;
            doc["command_topic"]  = mqttTopic("button", oid) + "/set";
            doc["availability_topic"] = nodeTopic + "/availability";
            doc["payload_available"]  = "online";
            doc["payload_not_available"] = "offline";
            doc["payload_press"]  = "PRESS";
            doc["icon"]           = "mdi:water-minus";
            injectDevice(doc.as<JsonObject>());
            String out; serializeJson(doc, out);
            mqttPublishConfig("button", oid, out);
        }
    }
    // Serial.println("[MQTT] Discovery publié");
}

// ── Publication de l'état complet (capteurs + vannes)
inline void mqttPublishState(){
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
    pub("sensor","temperature1", mqttPayloadFloat(temperature1));
    pub("sensor","temperature_remote", mqttPayloadFloat(temperatureRemote));
    if (totalPulses > 4290000000UL) {
        logSys("[MQTT] Rejet ecriture pulse_total > 4.29G");
    } else {
        pub("sensor","pulse_total", String((unsigned long)totalPulses));
        pub("sensor","litres_total", String(litresTotal,2));
    }
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
        if (valveCons[v].todayPulses > 4290000000UL) {
            // Rejet silently
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
        // version litres : un unsigned long 32 bits ne peut pas représenter
        // plus de ~4.29×10^9, donc on n'envoie PAS la valeur retenue dans
        // ce cas (Home Assistant recevrait "4294967295" qui empoisonnerait
        // ses statistiques "total_increasing" sur le long terme).
        snprintf(oid,sizeof(oid),"valve_%d_pulses_today",v);
        if (valveCons[v].todayPulses > 4290000000UL) {
            // Rejet silencieux (cohérent avec litres_today)
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
        snprintf(oid,sizeof(oid),"valve_%d_flow_lpm",v);
        // Débit instantané par vanne, calculé en RAM par
        // valveFlowUpdateAll() à 1 Hz. On formate à 2 décimales (idem
        // flow_lpm global) pour rester cohérent avec ce que l'UI
        // affiche. Pas de garde >4.29G ici (un float ne peut pas
        // atteindre cette valeur : PULSES_PER_LITRE=741.2 → max débit
        // mesurable = 65535pulses/s × 60/741.2 ≈ 5300 L/min, soit
        // ~5300.00 une fois sérialisé).
        pub("sensor", oid, String(valveCons[v].instantFlowLpm, 2));

        snprintf(oid,sizeof(oid),"valve_%d",v);
        pub("binary_sensor", oid, valves[v].isOpen ? "ON" : "OFF");
        pub("switch", oid, valves[v].isOpen ? "ON" : "OFF");
    }

    // ── Alarmes hydrauliques (voir AlarmManager.h) ──
    // Publié en retained pour que HA garde la dernière valeur connue
    // même après un reboot / déco du broker — une alarme fuite doit
    // rester visible tant qu'elle n'a pas été acquittée.
    pub("binary_sensor", "alarm", alarmState.active ? "ON" : "OFF");
    pub("sensor",        "alarm_code",    String((unsigned)alarmState.code));
    pub("sensor",        "alarm_message", String(alarmState.msg));

    // ── Compteurs "Vanne manuelle" (cf. Globals.h::ManualValveState) ──
    // Litres aujourd'hui + total. Publiés en retained pour que HA garde
    // la dernière valeur connue même après reboot / déco broker — la
    // recovery MQTT au prochain boot s'en servira pour ré-hydrater la
    // RAM (mode CONS_MQTT_ONLY). Mêmes seuils de rejet > 4.29G que les
    // autres compteurs pour éviter d'empoisonner HA avec une valeur
    // castée en unsigned long 32 bits.
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
        // silencieux (cohérent avec litres_today)
    } else {
        pub("sensor", "manual_valve_pulses_today", String((unsigned long)manualValveState.todayPulses));
    }



}

// ★ FIX : canari de confirmation de vie de la session.
// Contrairement aux valeurs de capteurs publiées en QoS 0 par
// mqttPublishState() (aucune garantie de livraison, aucun accusé de
// réception possible), ce message dédié est publié en QoS 1. Le broker
// répond obligatoirement par un PUBACK, capté par onMqttPublishAck()
// ci-dessous — c'est cette confirmation, et elle seule, qui réarme le
// watchdog d'inactivité. Publié plus fréquemment que le keepAlive MQTT
// (60s) pour ne pas dépendre uniquement du PINGREQ natif de la lib (qui
// de toute façon ne partirait jamais tant qu'il y a du trafic sortant
// périodique, cf. explication en tête de fichier).
inline void mqttSendCanary(){
    if(!mqttConnected) return;
    String availTopic = mqttTopicNode() + "/availability";
    mqttClient.publish(availTopic.c_str(), 1, true, "online", 6);
}

// ── Handler des commandes switch venues de HA + récupération des retained
inline void mqttHandleMessage(char* topic, char* payload, size_t len){
    String t(topic);
    String p(payload, len);
    // Tout message REÇU = preuve réelle d'activité bidirectionnelle sur la
    // socket → reset du watchdog. (Contrairement à un simple publish()
    // sortant, ceci prouve que le broker nous parle encore.)
    mqttLastActivityMs = millis();


  //  char b[200];
  //  snprintf(b, sizeof(b), "MQTT RX : %s = %s",
  //          topic,
   //         p.c_str());
//
  //  logSys(b);

#ifdef CONS_MQTT_ONLY
    // ── Phase de récupération au boot : fenêtre de 3s après connexion MQTT.
    // On lit les valeurs retained publiées lors de la session précédente et on
    // met à jour la RAM si MQTT > NVS (un compteur ne peut que progresser).
    // On ne traite pas comme commande switch pour ne pas interférer.
    //
    // NOTE : `pulse_total` et `litres_total` (compteur global) SONT
    // récupérés depuis MQTT retained au boot, au même titre que les
    // compteurs par vanne. En mode CONS_MQTT_ONLY, rien n'est persisté
    // en NVS pour le total (ConfigManager.h::pulseLoad/pulseSave sont
    // des no-ops), donc MQTT retained EST la source de vérité après
    // chaque reboot.
    //
    // Règle de récupération : on ne récupère la valeur MQTT que si elle
    // est strictement supérieure à l'état RAM courant (persistedPulseCount
    // + pulseCount). Un compteur ne peut que progresser entre deux
    // sessions, donc une valeur MQTT plus basse que notre RAM est
    // forcément anormale et on conserve notre valeur locale.
    if(!mqttRecoveryDone){
        String sensorBase = String(sysConfig.mqttPrefix) + "/sensor/" + sysConfig.mqttId + "/";
        if(t.startsWith(sensorBase)){
            String objId = t.substring(sensorBase.length());
            float val = p.toFloat();
            if(objId == "pulse_total"){
                // Compteur global en pulses (source de vérité, plus précise
                // que litres_total qui n'a que 2 décimales). En CONS_MQTT_ONLY
                // rien n'est persisté en NVS pour le total, donc MQTT retained
                // EST la seule référence entre deux boots.
                unsigned long mqttPulses = (unsigned long)(val + 0.5f);
                unsigned long cnt;
                noInterrupts(); cnt = pulseCount; interrupts();
                unsigned long liveTotal = persistedPulseCount + cnt;
                if(mqttPulses > liveTotal){
                    char b[120]; snprintf(b,sizeof(b),
                        "[RECOVERY] pulse_total: live=%lu < MQTT=%lu — restauré",
                        liveTotal, mqttPulses);
                    logSys(b);
                    persistedPulseCount = mqttPulses;
                    // CRITIQUE : realigner lastDistributedTotal sur le
                    // total restauré, sinon pulseDistribute() verra un
                    // delta énorme (mqttPulses - 0) au prochain tour et
                    // attribuera TOUS ces pulses à la vanne manuelle (si
                    // aucune vanne auto n'est ouverte) — contaminant
                    // manualValveState.pulsesTotal avec le compteur global.
                    lastDistributedTotal = mqttPulses + cnt;
                    // pas de pulseSave() : no-op en CONS_MQTT_ONLY
                    // (cf. ConfigManager.h). Au prochain reboot, la
                    // valeur sera de nouveau lue depuis MQTT retained.
                }
            } else if(objId == "litres_total"){
                // Fallback si pulse_total n'a pas été retenu (ex. : valeur
                // > 4.29G publiée une fois, ou discovery régénérée sans).
                // Conversion L→pulses moins précise (2 décimales perdues),
                // mais toujours mieux que de repartir de 0.
                unsigned long mqttPulses = (unsigned long)(val * PULSES_PER_LITRE + 0.5f);
                unsigned long cnt;
                noInterrupts(); cnt = pulseCount; interrupts();
                unsigned long liveTotal = persistedPulseCount + cnt;
                if(mqttPulses > liveTotal){
                    char b[120]; snprintf(b,sizeof(b),
                        "[RECOVERY] litres_total: live=%lu < MQTT=%lu pulses — restauré",
                        liveTotal, mqttPulses);
                    logSys(b);
                    persistedPulseCount = mqttPulses;
                    // CRITIQUE : realigner lastDistributedTotal (même
                    // raison que pour pulse_total ci-dessus — sinon
                    // pulseDistribute() attribue le delta à la vanne
                    // manuelle).
                    lastDistributedTotal = mqttPulses + cnt;
                }
            } else {
                    // valve_N_litres_today ou valve_N_litres_total (publiés en litres)
                    int v = -1;
                    if(parseValveObjId(objId, "_litres_total", v)){
                        unsigned long mqttPulses = (unsigned long)(val * PULSES_PER_LITRE + 0.5f);
                        if(mqttPulses > valveCons[v].pulsesTotal){
                            char b[120]; snprintf(b,sizeof(b),
                                "[RECOVERY] V%d litres_total: RAM=%lu < MQTT=%lu pulses — restauré",
                                v+1, valveCons[v].pulsesTotal, mqttPulses);
                            logSys(b);
                            valveCons[v].pulsesTotal = mqttPulses;
                            valveConsMarkDirty(v);
                        }
                    } else if( parseValveObjId(objId, "_litres_today", v  )){
                        unsigned long mqttPulses = (unsigned long)(val * PULSES_PER_LITRE + 0.5f);
                        uint16_t today = todayYMD();

                        if(today != 0){
                            // Init / rollover journalier : si todayYmd est 0
                            // (NVS jamais initialisée, ex. après un reset
                            // complet) ou pointe sur une date antérieure, on
                            // bascule sur la date du jour AVANT de comparer
                            // les compteurs. Sans cette étape, la condition
                            // `todayYmd == today` est toujours fausse et la
                            // recovery ne se déclenche jamais alors que la
                            // valeur MQTT retained est valide.
                            if(valveCons[v].todayYmd != today){
                                valveCons[v].todayYmd = today;
                                valveCons[v].todayPulses = 0;
                            }
                            if(mqttPulses > valveCons[v].todayPulses){
                                char b[120]; snprintf(b,sizeof(b),
                                    "[RECOVERY] V%d litres_today: RAM=%u < MQTT=%lu pulses — restauré",
                                    v+1, valveCons[v].todayPulses, mqttPulses);
                                logSys(b);
                                valveCons[v].todayPulses = (uint32_t)mqttPulses;
                                valveConsMarkDirty(v);
                            }
                        }
                    } else if(parseValveObjId(objId, "_pulses_total", v)){
                        // Valeur brute en pulses (pas de conversion depuis des litres).
                        unsigned long mqttPulses = (unsigned long)(val + 0.5f);
                        if(mqttPulses > valveCons[v].pulsesTotal){
                            char b[120]; snprintf(b,sizeof(b),
                                "[RECOVERY] V%d pulses_total: RAM=%lu < MQTT=%lu — restauré",
                                v+1, valveCons[v].pulsesTotal, mqttPulses);
                            logSys(b);
                            valveCons[v].pulsesTotal = mqttPulses;
                            valveConsMarkDirty(v);
                        }
                    } else if(parseValveObjId(objId, "_pulses_today", v)){
                        unsigned long mqttPulses = (unsigned long)(val + 0.5f);
                        uint16_t today = todayYMD();
                        if(today != 0){
                            // Mêmes règles que litres_today ci-dessus : on
                            // initialise/roule todayYmd AVANT de comparer.
                            if(valveCons[v].todayYmd != today){
                                valveCons[v].todayYmd = today;
                                valveCons[v].todayPulses = 0;
                            }
                            if(mqttPulses > valveCons[v].todayPulses){
                                char b[120]; snprintf(b,sizeof(b),
                                    "[RECOVERY] V%d pulses_today: RAM=%u < MQTT=%lu — restauré",
                                    v+1, valveCons[v].todayPulses, mqttPulses);
                                logSys(b);
                                valveCons[v].todayPulses = (uint32_t)mqttPulses;
                                valveConsMarkDirty(v);
                            }
                        }
                    } else if(objId == "manual_valve_litres_total"){
                        // ── Recovery "Vanne manuelle" (cf. Globals.h) ──
                        unsigned long mqttPulses = (unsigned long)(val * PULSES_PER_LITRE + 0.5f);
                        char b[140];
                        snprintf(b,sizeof(b),
                            "[RECOVERY] VanneManuelle litres_total: RAM=%lu pulses MQTT=%lu (val=%.2f)",
                            manualValveState.pulsesTotal, mqttPulses, val);
                        logSys(b);
                        if(mqttPulses > manualValveState.pulsesTotal){
                            snprintf(b,sizeof(b),
                                "[RECOVERY] VanneManuelle litres_total: RAM=%lu < MQTT=%lu pulses — restauré",
                                manualValveState.pulsesTotal, mqttPulses);
                            logSys(b);
                            manualValveState.pulsesTotal = mqttPulses;
                            manualValveDirty = true;
                        }
                    } else if(objId == "manual_valve_litres_today"){
                        unsigned long mqttPulses = (unsigned long)(val * PULSES_PER_LITRE + 0.5f);
                        uint16_t today = todayYMD();
                        char b[140];
                        snprintf(b,sizeof(b),
                            "[RECOVERY] VanneManuelle litres_today: RAM=%u pulses MQTT=%lu (val=%.2f)",
                            (unsigned)manualValveState.todayPulses, mqttPulses, val);
                        logSys(b);
                        if(today != 0){
                            if(manualValveState.todayYmd != today){
                                manualValveState.todayYmd = today;
                                manualValveState.todayPulses = 0;
                            }
                            if(mqttPulses > manualValveState.todayPulses){
                                snprintf(b,sizeof(b),
                                    "[RECOVERY] VanneManuelle litres_today: RAM=%u < MQTT=%lu pulses — restauré",
                                    (unsigned)manualValveState.todayPulses, mqttPulses);
                                logSys(b);
                                manualValveState.todayPulses = (uint32_t)mqttPulses;
                                manualValveDirty = true;
                            }
                        }
                    } else if(objId == "manual_valve_pulses_total"){
                        unsigned long mqttPulses = (unsigned long)(val + 0.5f);
                        char b[140];
                        snprintf(b,sizeof(b),
                            "[RECOVERY] VanneManuelle pulses_total: RAM=%lu MQTT=%lu (val=%.1f)",
                            manualValveState.pulsesTotal, mqttPulses, val);
                        logSys(b);
                        if(mqttPulses > manualValveState.pulsesTotal){
                            snprintf(b,sizeof(b),
                                "[RECOVERY] VanneManuelle pulses_total: RAM=%lu < MQTT=%lu — restauré",
                                manualValveState.pulsesTotal, mqttPulses);
                            logSys(b);
                            manualValveState.pulsesTotal = mqttPulses;
                            manualValveDirty = true;
                        }
                    } else if(objId == "manual_valve_pulses_today"){
                        unsigned long mqttPulses = (unsigned long)(val + 0.5f);
                        uint16_t today = todayYMD();
                        char b[140];
                        snprintf(b,sizeof(b),
                            "[RECOVERY] VanneManuelle pulses_today: RAM=%u MQTT=%lu (val=%.1f)",
                            (unsigned)manualValveState.todayPulses, mqttPulses, val);
                        logSys(b);
                        if(today != 0){
                            if(manualValveState.todayYmd != today){
                                manualValveState.todayYmd = today;
                                manualValveState.todayPulses = 0;
                            }
                            if(mqttPulses > manualValveState.todayPulses){
                                snprintf(b,sizeof(b),
                                    "[RECOVERY] VanneManuelle pulses_today: RAM=%u < MQTT=%lu — restauré",
                                    (unsigned)manualValveState.todayPulses, mqttPulses);
                                logSys(b);
                                manualValveState.todayPulses = (uint32_t)mqttPulses;
                                manualValveDirty = true;
                            }
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
            persistedPulseCount = 0; // no-op effectif en CONS_MQTT_ONLY (pulseSave() vide), mais conservé pour la version non-CONS_MQTT_ONLY
            for(int vv=0;vv<VANNE_COUNT;vv++){
                valveCons[vv].pulsesTotal = 0;
                valveCons[vv].todayPulses = 0;
                valveCons[vv].carry = 0.0f;
                for(int d=0;d<CONS_HISTORY_DAYS;d++) valveCons[vv].history[d] = {0,0,0.0f};
                valveConsSaveOne(vv);
            }
            // Reset aussi le compteur "Vanne manuelle" (cf. Globals.h) —
            // le bouton "pulse_total_reset" remet TOUT à zéro, par
            // cohérence avec l'endpoint Web /api/pulse/reset.
            manualValveState.pulsesTotal = 0;
            manualValveState.todayPulses  = 0;
            manualValveState.todayYmd     = todayYMD();
            manualValveState.todayIdx     = 0;
            manualValveState.carry        = 0.0f;
            for(int d=0;d<CONS_HISTORY_DAYS;d++){
                manualValveState.history[d] = {0,0,0.0f};
            }
            manualValveFlushOne();
            pulseSave(); // no-op en CONS_MQTT_ONLY, persiste en mode normal
            lastDistributedTotal = 0;
            lastMqttPubMs = 0; // Force update MQTT
            logSys("Compteur global remis a zero via Home Assistant");
        }
        else if(obj == "manual_valve_reset"){
            // ── Reset dédié "Vanne manuelle" (cf. Globals.h) ──
            // N'affecte QUE le compteur manuel (et son rollover jour),
            // pas le compteur global ni les vannes automatisées.
            manualValveState.pulsesTotal = 0;
            manualValveState.todayPulses  = 0;
            manualValveState.todayYmd     = todayYMD();
            manualValveState.todayIdx     = 0;
            manualValveState.carry        = 0.0f;
            for(int d=0;d<CONS_HISTORY_DAYS;d++){
                manualValveState.history[d] = {0,0,0.0f};
            }
            manualValveFlushOne();
            lastMqttPubMs = 0; // Force MAJ MQTT immédiate
            logSys("Compteur VanneManuelle remis a zero via Home Assistant");
        }
        else {
            int v = -1;
            if(sscanf(obj.c_str(),"valve_%d_reset_cons",&v)==1 && v>=0 && v<VANNE_COUNT){
                valveCons[v].pulsesTotal = 0;
                valveCons[v].todayPulses = 0;
                valveCons[v].carry = 0.0f;
                for(int d=0;d<CONS_HISTORY_DAYS;d++) valveCons[v].history[d] = {0,0,0.0f};
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
    String availTopic = mqttTopicNode() + "/availability";
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

    // Test TCP brut : si le port est fermé, on bloque les retries 60s
    {
        WiFiClient probe;
        probe.setTimeout(2000);
        if(probe.connect(brokerIp, sysConfig.mqttPort)){
            logSys("[MQTT] Test TCP OK — broker joignable");
            probe.stop();
        } else {
            logSys("[MQTT] ✗ Test TCP ÉCHEC — (broker injoignable: check VLAN/DNS)");
            lastMqttConnectAttemptMs = millis() + 45000UL;
            return;
        }
    }

    mqttClient.setClientId(sysConfig.mqttId);
    if(strlen(sysConfig.mqttUser)>0){
        mqttClient.setCredentials(sysConfig.mqttUser, sysConfig.mqttPass);
    }

    // ★ CRITICAL : AsyncMqttClient ne COPIE pas la chaîne du topic LWT —
    // il stocke le pointeur c_str() tel quel. Une String locale serait
    // détruite à la fin de mqttSetup() et Mosquitto recevrait des déchets
    // mémoire en guise de topic → RST instantané. static String garantit
    // que le buffer reste vivant pour tous les connect() ultérieurs.
    static String availTopic;
    availTopic = mqttTopicNode() + "/availability";
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

    for(int attempt=0; attempt<3; attempt++){
        if(attempt > 0){
            char rb[80];
            snprintf(rb, sizeof(rb), "[MQTT] Retry connect() #%d après %d ms", attempt+1, 500 * attempt);
            logSys(rb);
            delay(500 * attempt);
        }
        mqttClient.connect();
        for(int wait=0; wait<30; wait++){
            if(mqttConnected) break;
            delay(50);
        }
        if(mqttConnected){
            char cb[80];
            snprintf(cb, sizeof(cb), "[MQTT] ✓ Connecté dès la tentative #%d", attempt+1);
            logSys(cb);
            break;
        }
    }
    if(!mqttConnected){
        logSys("[MQTT] ✗ Setup: 3 tentatives échouées — retry via mqttLoop()");
    }
    lastMqttConnectAttemptMs = millis();
}

inline void mqttLoop(){
    if(!sysConfig.mqttEnabled){ mqttConnected = false; return; }

    unsigned long now = millis();
    bool wifiUp = (WiFi.status() == WL_CONNECTED);

    // ── RESET DU COMPTEUR D'ÉCHECS QUAND LE WIFI EST KO ──
    // Si on a une rafale d'échecs MQTT, c'est potentiellement à cause
    // d'une perte WiFi transitoire. Tant que le WiFi n'est pas rétabli,
    // on n'incrémente pas le compteur d'échecs (la pile TCP refusera
    // toujours le connect()). Sans ce reset, on accumulerait des
    // échecs fantômes pendant la coupure WiFi et on aurait un backoff
    // de 60s APRÈS le retour du WiFi — exactement le bug observé
    // "ça reste KO toute la nuit".
    if(!wifiUp){
        mqttConsecutiveFailures = 0;
    }

    // ── WATCHDOG D'INACTIVITÉ ──
    // Seules les confirmations réelles (PUBACK/SUBACK/messages) réarment
    // ce timer, garantissant la détection d'une socket zombie.
    if(mqttConnected && mqttLastActivityMs > 0 
       && (now - mqttLastActivityMs) > MQTT_WATCHDOG_MS){
        // Serial.printf("[MQTT] ⚠ Watchdog: aucune activité CONFIRMÉE depuis %lu s — forçage reconnexion\n",
        //              (unsigned long)((now - mqttLastActivityMs) / 1000UL));
        logSys("[MQTT] Watchdog inactivité (aucun PUBACK/SUBACK/RX) — reconnexion forcée");
        // Déconnexion brutale pour éviter de polluer la pile TCP asynchrone
        mqttClient.disconnect(true);
        mqttConnected = false;
        mqttLastActivityMs = 0;
        // On pose l'instant d'échec pour redémarrer le calcul du backoff
        // et éviter une boucle de reconnexion ininterrompue
        lastMqttConnectAttemptMs = millis();
    }

        // Reconnexion préventive périodique.
        if (mqttConnected && (now - lastMqttForceReconnectMs) > MQTT_FORCED_RECONNECT_MS) {
            lastMqttForceReconnectMs = now;
            logSys("[MQTT] Reconnexion préventive périodique (purge état zombie éventuel)");
            delay(100);
            mqttClient.disconnect(true);
            mqttConnected = false;
            mqttLastActivityMs = 0;
            // Éviter l'integer underflow dans le script de reconnexion
            lastMqttConnectAttemptMs = millis();
        }

    if(!mqttConnected && wifiUp){
        now = millis(); // FIX : actualisation de now après delay() ou callbacks
        // ── CALCUL DU DÉLAI DE RETRY (backoff exponentiel) ──
        // Pour 1 échec : 5s ; 2 échecs : 10s ; 3 : 20s ; 4 : 40s ; 5+ : 60s
        unsigned long backoffDelay = MQTT_RECONNECT_MS;
        if(mqttConsecutiveFailures > 0){
            unsigned long shift = min((unsigned long)mqttConsecutiveFailures - 1, 4UL);
            unsigned long candidate = MQTT_BACKOFF_MIN_MS * (1UL << shift);
            backoffDelay = min(candidate, MQTT_BACKOFF_MAX_MS);
        }
        if(now - lastMqttConnectAttemptMs > backoffDelay){
            lastMqttConnectAttemptMs = now;
            // Serial.printf("[MQTT] Reconnexion (échecs=%u, délai=%lu s)…\n",
            //              (unsigned)mqttConsecutiveFailures,
            //              (unsigned long)(backoffDelay / 1000UL));
            // On logue via logSys uniquement lors d'une transition
            // pour ne pas spammer les logs une fois arrivé au backoff max (60s).
            // Un rappel discret est émis toutes les 30 min (MQTT_DOWN_HEARTBEAT_MS).
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

            // ── DIAGNOSTIC TCP SYNCHRONE AVANT ASYNCMQTTCLIENT ──
            {
                WiFiClient probe;
                probe.setTimeout(2000);
                bool tcpOk = probe.connect(sysConfig.mqttHost, sysConfig.mqttPort);
                if(tcpOk){
                    probe.stop();
                    // Test TCP Ok. On injecte explicitement l'IPAddress
                    // ou le hostname à configurer.
                    IPAddress brokerIp;
                    bool ipOk = brokerIp.fromString(sysConfig.mqttHost);
                    {
                        char tb[120];
                        snprintf(tb, sizeof(tb), "[MQTT] probe OK → setServer(%s) + connect()",
                                 ipOk ? brokerIp.toString().c_str() : sysConfig.mqttHost);
                        logSys(tb);
                    }
                    if(ipOk){
                        mqttClient.setServer(brokerIp, sysConfig.mqttPort);
                    } else {
                        mqttClient.setServer(sysConfig.mqttHost, sysConfig.mqttPort);
                    }
                    mqttClient.setClientId(sysConfig.mqttId);
                    if(strlen(sysConfig.mqttUser)>0){
                        mqttClient.setCredentials(sysConfig.mqttUser, sysConfig.mqttPass);
                    }
                    mqttClient.connect();
                } else {
                    logSys("[MQTT] ✗ Diagnostic: Probe TCP a échoué ! LwIP/routeur refuse le trafic sur ce port.");
                    onMqttDisconnect(AsyncMqttClientDisconnectReason::TCP_DISCONNECTED);
                }
            }
        }
    }

    if(mqttConnected){
if(!mqttDiscoveryPublished){
    mqttDiscoveryPublished = true;
    #ifdef CONS_MQTT_ONLY
        // En mode CONS_MQTT_ONLY on est encore abonné aux topics sensor/+
        // pour la recovery : publier maintenant provoquerait un auto-écho
        // (le firmware recevrait son propre message retained comme une
        // "valeur de session précédente" et la re-traiterait). On diffère
        // ce premier publish à la fin de la fenêtre de recovery.
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
        // ── Fin de fenêtre de récupération (3 s après connexion) ─────────────
        // On ferme la fenêtre, on flush les valeurs restaurées en NVS, on
        // se désabonne du pattern sensor (pour ne plus recevoir les états
        // futurs comme des retained de récupération), puis on publie
        // la discovery et l'état fusionné final.
        if(!mqttRecoveryDone && (now - mqttRecoveryStartMs >= 3000UL)){
            mqttRecoveryDone = true;
            // Désabonnement du pattern de recovery
            String recovPattern = String(sysConfig.mqttPrefix) + "/sensor/" + sysConfig.mqttId + "/+";
            mqttClient.unsubscribe(recovPattern.c_str());
            // Flush NVS des valeurs éventuellement restaurées
            valveConsFlushDirty();
            logSys("[CONS] Recovery MQTT terminée — NVS mis à jour");
            // Publication discovery + état fusionné après récupération
            mqttPublishDiscovery();
            mqttPublishState();
            lastMqttPubMs = now;
        }
#endif
        if(now - lastMqttPubMs > MQTT_PUB_INTERVAL_MS){
            lastMqttPubMs = now;
#ifdef CONS_MQTT_ONLY
            // En mode recovery, ne pas publier l'état avant la fin de la fenêtre
            // (on ne voudrait pas retained avec des valeurs partielles sur le broker)
            if(mqttRecoveryDone) mqttPublishState();
#else
            mqttPublishState();
#endif
            // ★ FIX : plus de mqttLastActivityMs=millis() ici — un publish()
            // QoS 0 sortant ne prouve rien. Voir watchdog ci-dessus et
            // canari ci-dessous pour la vraie confirmation.
        }

        // ★ FIX : canari QoS 1 périodique — seule source de confirmation
        // continue de vie de la session en dehors des messages reçus.
        if(now - lastMqttCanaryMs > MQTT_CANARY_INTERVAL_MS){
            lastMqttCanaryMs = now;
            mqttSendCanary();
        }
    }
}

#endif // IOCAN
