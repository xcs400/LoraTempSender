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
inline void mqttHandleMessage(char* topic, char* payload, size_t len){
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

inline void onMqttConnect(bool sessionPresent){
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

inline void onMqttDisconnect(AsyncMqttClientDisconnectReason r){
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

#endif // IOCAN
