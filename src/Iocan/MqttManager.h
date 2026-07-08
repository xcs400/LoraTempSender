#pragma once
#ifdef IOCAN
// ============================================================
// MqttManager.h — MQTT + Home Assistant Auto-Discovery
// ============================================================
// Correspond à la SECTION 11b du fichier d'origine.
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
// ============================================================

#include "Globals.h"
#include "LoggerManager.h"
#include "ConfigManager.h"
#include "FlowMeter.h"
#include "ValveManager.h"

inline bool mqttDiscoveryPublished = false;

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
inline void mqttPublishConfig(const char* component, const char* objId, const String& payload){
    if(!mqttConnected) return;
    String topic = mqttTopic(component, objId) + "/config";
    Serial.printf("[MQTT] discovery -> %s\n", topic.c_str());
    // qos 0, retain true
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

// Helper: injecte le bloc "device" dans un document (en parsant puis copiant).
// On ne peut pas directement affecter un StaticJsonDocument à un JsonObject
// dans ArduinoJson v6, on utilise donc un parse round-trip sur la chaîne.
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

inline void mqttPublishDiscovery(){
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
        // Payload minimal et largement compatible avec Home Assistant.
        // On évite les champs plus controversés comme default_entity_id et on
        // garde uniquement les clés reconnues de façon stable par l'intégration MQTT.
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
    Serial.println("[MQTT] Discovery publié");
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
}

// ── Handler des commandes switch venues de HA + récupération des retained
inline void mqttHandleMessage(char* topic, char* payload, size_t len){
    String t(topic);
    String p(payload, len);
    // Tout message reçu = activité sur la socket → reset du watchdog.
    mqttLastActivityMs = millis();

#ifdef CONS_MQTT_ONLY
    // ── Phase de récupération au boot : fenêtre de 3s après connexion MQTT.
    // On lit les valeurs retained publiées lors de la session précédente et on
    // met à jour la RAM si MQTT > NVS (un compteur ne peut que progresser).
    // On ne traite pas comme commande switch pour ne pas interférer.
    //
    // ATTENTION : `pulse_total` est INTENTIONNELLEMENT EXCLU de cette
    // récupération (voir le bloc `if(objId == "pulse_total")` ci-dessous).
    // Raison : le mode CONS_MQTT_ONLY ne persiste PAS pulse_total en NVS
    // (ConfigManager.h::pulseLoad/pulseSave sont des no-ops), et on n'écrit
    // JAMAIS `persistedPulseCount` à partir de MQTT retenu. C'est le fix
    // anti-empoisonnement : si HA retenait une valeur buggée (4.29G
    // observée), on ne la ré-injecterait plus dans la NVS à chaque reboot.
    // Les compteurs par vanne (valve_N_litres_total/today) sont en
    // revanche toujours récupérés depuis MQTT (leur persistance NVS est
    // toujours active, c'est juste la persistance NVS du total global qui
    // a été supprimée pour casser le cycle).
    if(!mqttRecoveryDone){
        String sensorBase = String(sysConfig.mqttPrefix) + "/sensor/" + sysConfig.mqttId + "/";
        if(t.startsWith(sensorBase)){
            String objId = t.substring(sensorBase.length());
            float val = p.toFloat();
            if(objId == "pulse_total"){
                    // Mode CONS_MQTT_ONLY : pulse_total n'est PAS persisté en NVS
                    // (voir Globals.h / ConfigManager.h). On ignore donc totalement
                    // la valeur MQTT retained ici : pas de "recovery" possible.
                    // La valeur publiée par HA reste sa propre référence ; le
                    // firmware repart de 0 à chaque reboot pour pulse_total.
                    // (l'auto-empoisonnement observé venait précisément de ce
                    // mécanisme : la valeur buggée 4.29G retenue par HA
                    // écrasait la NVS à chaque reboot).
                } else {
                    // valve_N_litres_today ou valve_N_litres_total (publiés en litres)
                    int v = -1;
                    if(sscanf(objId.c_str(), "valve_%d_litres_total", &v) == 1 && v >= 0 && v < VANNE_COUNT){
                        unsigned long mqttPulses = (unsigned long)(val * PULSES_PER_LITRE + 0.5f);
                        if(mqttPulses > 4290000000UL){
                            char b[80]; snprintf(b,80,"[MQTT] Rejet V%d_total > 4.29G", v+1);
                            logSys(b);
                        } else if(mqttPulses > valveCons[v].pulsesTotal){
                            char b[80]; snprintf(b,sizeof(b),
                                "[RECOVERY] V%d litres_total: NVS=%lu < MQTT=%lu pulses — restauré",
                                v+1, valveCons[v].pulsesTotal, mqttPulses);
                            logSys(b);
                            valveCons[v].pulsesTotal = mqttPulses;
                            valveConsMarkDirty(v);
                        }
                    } else if(sscanf(objId.c_str(), "valve_%d_litres_today", &v) == 1 && v >= 0 && v < VANNE_COUNT){
                        unsigned long mqttPulses = (unsigned long)(val * PULSES_PER_LITRE + 0.5f);
                        if(mqttPulses > 4290000000UL){
                            // Ignorer tacitement ou petit log
                        } else {
                            uint16_t today = todayYMD();
                            if(today != 0 && valveCons[v].todayYmd == today && mqttPulses > valveCons[v].todayPulses){
                                char b[80]; snprintf(b,sizeof(b),
                                    "[RECOVERY] V%d litres_today: NVS=%u < MQTT=%lu pulses — restauré",
                                    v+1, valveCons[v].todayPulses, mqttPulses);
                                logSys(b);
                                valveCons[v].todayPulses = (uint32_t)mqttPulses;
                                valveConsMarkDirty(v);
                            }
                        }
                    } else if(sscanf(objId.c_str(), "valve_%d_pulses_total", &v) == 1 && v >= 0 && v < VANNE_COUNT){
                        // Valeur brute en pulses (pas de conversion depuis des litres).
                        // Mêmes règles : on ne récupère que si MQTT > NVS et < 4.29G.
                        unsigned long mqttPulses = (unsigned long)(val + 0.5f);
                        if(mqttPulses > 4290000000UL){
                            char b[80]; snprintf(b,80,"[MQTT] Rejet V%d_pulses_total > 4.29G", v+1);
                            logSys(b);
                        } else if(mqttPulses > valveCons[v].pulsesTotal){
                            char b[80]; snprintf(b,sizeof(b),
                                "[RECOVERY] V%d pulses_total: NVS=%lu < MQTT=%lu — restauré",
                                v+1, valveCons[v].pulsesTotal, mqttPulses);
                            logSys(b);
                            valveCons[v].pulsesTotal = mqttPulses;
                            valveConsMarkDirty(v);
                        }
                    } else if(sscanf(objId.c_str(), "valve_%d_pulses_today", &v) == 1 && v >= 0 && v < VANNE_COUNT){
                        unsigned long mqttPulses = (unsigned long)(val + 0.5f);
                        if(mqttPulses > 4290000000UL){
                            // Ignorer tacitement
                        } else {
                            uint16_t today = todayYMD();
                            if(today != 0 && valveCons[v].todayYmd == today && mqttPulses > valveCons[v].todayPulses){
                                char b[80]; snprintf(b,sizeof(b),
                                    "[RECOVERY] V%d pulses_today: NVS=%u < MQTT=%lu — restauré",
                                    v+1, valveCons[v].todayPulses, mqttPulses);
                                logSys(b);
                                valveCons[v].todayPulses = (uint32_t)mqttPulses;
                                valveConsMarkDirty(v);
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
            pulseSave(); // no-op en CONS_MQTT_ONLY, persiste en mode normal
            lastDistributedTotal = 0;
            lastMqttPubMs = 0; // Force update MQTT
            logSys("Compteur global remis a zero via Home Assistant");
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

inline void onMqttConnect(bool sessionPresent){
    mqttConnected = true;
    mqttDiscoveryPublished = false;
    // Reset du compteur d'échecs et armement du watchdog d'inactivité.
    // Le watchdog a besoin d'un timestamp d'activité fraîche à chaque
    // connexion réussie, sinon il considérerait immédiatement la connexion
    // comme morte.
    mqttConsecutiveFailures = 0;
    mqttLastActivityMs      = millis();
    Serial.println("[MQTT] Connecté");
    String cmdTopic = String(sysConfig.mqttPrefix) + "/switch/" + sysConfig.mqttId + "/+/set";
    mqttClient.subscribe(cmdTopic.c_str(), 0);
    mqttLastActivityMs = millis(); // subscribe = activité
    String btnTopic = String(sysConfig.mqttPrefix) + "/button/" + sysConfig.mqttId + "/+/set";
    mqttClient.subscribe(btnTopic.c_str(), 0);
    mqttLastActivityMs = millis(); // subscribe = activité
    Serial.print("[MQTT] Abonné à: "); Serial.println(cmdTopic);
    Serial.print("[MQTT] Abonné à: "); Serial.println(btnTopic);
    // Publication disponibilité
    String availTopic = mqttTopicNode() + "/availability";
    mqttClient.publish(availTopic.c_str(), 0, true, "online", 6);
    mqttLastActivityMs = millis(); // publish = activité
#ifdef CONS_MQTT_ONLY
    // ── Lancer la récupération des valeurs retained ────────────────────────────
    // On s'abonne à <prefix>/sensor/<mqttId>/+ pour recevoir TOUTES les
    // valeurs retained publiées lors de la session précédente. Le broker
    // les livre immédiatement sur chaque topic connu. mqttHandleMessage()
    // s'en empare pendant la fenêtre mqttRecoveryDone==false.
    // Après 3s (dans mqttLoop()), la fenêtre se ferme, on se désabonne et
    // on flush les valeurs restaurées en NVS.
    String recovPattern = String(sysConfig.mqttPrefix) + "/sensor/" + sysConfig.mqttId + "/+";
    mqttClient.subscribe(recovPattern.c_str(), 0);
    mqttRecoveryDone    = false;
    mqttRecoveryStartMs = millis();
    Serial.print("[MQTT] Recovery MQTT activée, abonné à: "); Serial.println(recovPattern);
    logSys("[CONS] Recovery MQTT démarrée (fenêtre 3s)");
    // NB : on NE publie pas discovery ni state ici — on attend la fin de la
    // fenêtre de récupération (mqttLoop) pour publier l'état fusionné juste.
#else
    // Discovery + état initial (mode normal)
    mqttPublishDiscovery();
    mqttPublishState();
#endif
}

inline void onMqttDisconnect(AsyncMqttClientDisconnectReason r){
    mqttConnected = false;
    mqttDiscoveryPublished = false;
    // ── RESET DU THROTTLE DE RECONNEXION ──
    // lastMqttConnectAttemptMs est remis à 0 pour que la prochaine boucle
    // mqttLoop() puisse retenter IMMÉDIATEMENT (sous réserve du backoff
    // exponentiel ci-dessous). Sans ce reset, si la dernière tentative
    // vient d'être lancée et qu'onMqttDisconnect arrive dans la même
    // seconde, on attendrait 15s de plus pour rien (le throttle utilise
    // "now - last > 15000", et last vient d'être posé).
    lastMqttConnectAttemptMs = 0;
    mqttLastActivityMs = 0; // désarmement du watchdog (rien à surveiller tant qu'on n'est pas connecté)
    // ── BACKOFF EXPONENTIEL ──
    // On incrémente le compteur d'échecs CONSÉCUTIFS. mqttLoop() s'en sert
    // pour calculer le délai avant la prochaine tentative : 5s, 10s, 20s,
    // 40s, 60s (plafond). Remis à 0 dans onMqttConnect() lors d'une
    // reconnexion réussie.
    if(mqttConsecutiveFailures < 10) mqttConsecutiveFailures++;
    Serial.printf("[MQTT] Échecs consécutifs: %u (prochain retry dans %lu s)\n",
                  (unsigned)mqttConsecutiveFailures,
                  (unsigned long)(min(MQTT_BACKOFF_MAX_MS,
                                      MQTT_BACKOFF_MIN_MS * (1UL << min((int)mqttConsecutiveFailures-1, 4)))) / 1000UL);
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

inline void mqttSetup(){
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

    mqttClient.setClientId(sysConfig.mqttId);
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
    // Si on se croit connecté (mqttConnected=true) mais qu'aucune
    // activité (publish, subscribe, message reçu) n'a eu lieu depuis
    // MQTT_WATCHDOG_MS, c'est que la socket TCP est probablement
    // morte dans un état zombie (routeur qui a perdu l'association
    // STA, coupure RF brève, broker rebooté silencieusement, etc.).
    // AsyncMqttClient n'a pas de mécanisme de ping natif visible côté
    // firmware, donc on simule : on force la déconnexion, onMqttDisconnect
    // est appelé, le compteur d'échecs s'incrémente, et la logique
    // de backoff prend le relais.
    if(mqttConnected && mqttLastActivityMs > 0
       && (now - mqttLastActivityMs) > MQTT_WATCHDOG_MS){
        Serial.printf("[MQTT] ⚠ Watchdog: aucune activité depuis %lu s — forçage reconnexion\n",
                      (unsigned long)((now - mqttLastActivityMs) / 1000UL));
        logSys("[MQTT] Watchdog inactivité — reconnexion forcée");
        mqttClient.disconnect();
        // On force mqttConnected à false ici, en plus de onMqttDisconnect,
        // pour que la branche de reconnexion ci-dessous s'exécute tout
        // de suite au prochain tour de loop() (disconnect() étant
        // asynchrone, onMqttDisconnect peut arriver après).
        mqttConnected = false;
        mqttLastActivityMs = 0;
    }

    if(!mqttConnected && wifiUp){
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
            Serial.printf("[MQTT] Reconnexion (échecs=%u, délai=%lu s)…\n",
                          (unsigned)mqttConsecutiveFailures,
                          (unsigned long)(backoffDelay / 1000UL));
            mqttClient.connect();
        }
    }

    if(mqttConnected){
        if(!mqttDiscoveryPublished){
            mqttDiscoveryPublished = true;
            Serial.println("[MQTT] publication discovery forcée après connexion");
            mqttPublishDiscovery();
            mqttPublishState();
            mqttLastActivityMs = millis(); // activity
            lastMqttPubMs = now;
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
            mqttLastActivityMs = millis(); // activity
            // Flush NVS des valeurs éventuellement restaurées
            valveConsFlushDirty();
            logSys("[CONS] Recovery MQTT terminée — NVS mis à jour");
            // Publication discovery + état fusionné après récupération
            mqttPublishDiscovery();
            mqttPublishState();
            mqttLastActivityMs = millis(); // activity
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
            mqttLastActivityMs = millis(); // activity
        }
    }
}

#endif // IOCAN
