#pragma once
#ifdef IOCAN
// ============================================================
// ConfigManager.h — NVS Preferences (config, récupération, conso, schedules)
// ============================================================
// Correspond aux SECTION 5 et 5b du fichier d'origine :
//   - configLoad() / configSave()      : config système
//   - nvsSelfTestAndRecover()          : auto-réparation NVS corrompue
//   - pulseLoad() / pulseSave()        : compteur d'impulsions persistant
//   - todayYMD()                       : date du jour (YYYYMMDD)
//   - valveConsLoad() / valveConsSaveOne() / valveConsSaveFlowCoeff()
//   - schedSave() / schedLoad()        : programmes par vanne
// ============================================================

#include "Globals.h"
#include "LoggerManager.h"
#include <nvs_flash.h>

inline void configLoad(){
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

inline void configSave(){
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
// RÉCUPÉRATION NVS
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
inline bool nvsSelfTestAndRecover(){
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
inline void pulseLoad(){
    prefs.begin("irrigcfg", false);
    persistedPulseCount = prefs.getULong("pulseCnt", 0UL);
    prefs.end();
}

inline void pulseSave(){
    prefs.begin("irrigcfg", false);
    prefs.putULong("pulseCnt", persistedPulseCount);
    prefs.end();
}

// ── Date du jour au format YYYYMMDD (0 si pas sync NTP)
inline uint16_t todayYMD(){
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
inline void valveConsLoad(){
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
        // Résidu d'accumulation persistant (carry).
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

inline void valveConsSaveOne(int v){
    prefs.begin("irrcons", false);
    char key[24];
    snprintf(key,sizeof(key),"v%d_pc",v);
    prefs.putULong(key, valveCons[v].pulsesTotal);
    snprintf(key,sizeof(key),"v%d_td",v);
    prefs.putUShort(key, valveCons[v].todayYmd);
    snprintf(key,sizeof(key),"v%d_tp",v);
    prefs.putUInt(key, valveCons[v].todayPulses);
    // Persiste le carry à chaque sauvegarde de routine (cet appel a lieu
    // après chaque distribution de pulses sur les vannes ouvertes, donc
    // le carry est déjà à jour à ce moment-là).
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
inline void valveConsSaveFlowCoeff(int v){
    prefs.begin("irrcons", false);
    char key[24];
    snprintf(key,sizeof(key),"v%d_fc",v);
    prefs.putFloat(key, valveCons[v].flowCoeff);
    prefs.end();
}

// Sauvegarde/chargement des programmes
inline void schedSave(){
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

inline void schedLoad(){
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

#endif // IOCAN
