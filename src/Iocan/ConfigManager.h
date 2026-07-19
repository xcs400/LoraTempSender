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
    prefs.getString("tzPosix", sysConfig.tzPosix, 48);
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
        char def[24]; snprintf(def,24,"V%d",i);
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
    if(sysConfig.tzPosix[0]) prefs.putString("tzPosix", sysConfig.tzPosix);
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

// Charger et sauvegarder le compteur d'impulsions persisté.
//
// En mode CONS_MQTT_ONLY (Globals.h) : pulseLoad/pulseSave sont des NO-OPS.
// Le compteur de pulses total n'est PAS persisté en NVS — uniquement la
// valeur runtime `pulseCount` en RAM. Au reboot, on repart donc de zéro
// pour `pulse_total`, et la valeur publiée en MQTT retained (par HA) reste
// la référence de persistance. Ça élimine l'auto-empoisonnement observé :
// avant, si HA retenait une valeur buggée (ex: 4 294 957 568), la recovery
// MQTT au boot écrasait la NVS avec cette valeur à chaque reboot → cycle
// vicieux. Voir MqttManager.h pour le détail.
//
// NB : `lastDistributedTotal` (utilisé par pulseDistribute) est initialisé
// à 0 dans le setup() en cohérence (voir MainIocan_S.cpp).
#ifdef CONS_MQTT_ONLY
inline void pulseLoad(){
    persistedPulseCount = 0UL; // pas de persistance en mode CONS_MQTT_ONLY
}
inline void pulseSave(){
    // no-op : pas de persistance en mode CONS_MQTT_ONLY
}
#else
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
#endif

// Déclarations anticipées pour les helpers de consommation
inline void valveConsFlushDirty();
inline void manualValveFlushDirty(); // défini plus bas, après manualValveLoad/SaveOne

// ── Redémarrage sécurisé : flush NVS avant ESP.restart() ─────────────────
//
// À utiliser à la place d'un ESP.restart() nu partout où on peut l'anticiper
// (reset Web, portail captif, timeout WiFi, etc.). Garantit que la conso en
// RAM est persistée même si le flush périodique 30s n'a pas encore eu lieu.
// NE protège PAS contre une coupure secteur (cas traité par la récupération
// MQTT au boot, voir MqttManager.h).
//
// Séquence :
//   1) Flush compteur pulse global (pulseSave) — toujours.
//   2) Flush conso par vanne dirty (valveConsFlushDirty) — toujours.
//   2b) Flush conso "vanne manuelle" dirty (manualValveFlushDirty) — toujours.
//   3) Petit délai pour laisser les transactions NVS terminer.
//   4) ESP.restart().
inline void safeRestart(const char* reason = nullptr){
    if(reason && reason[0]) logSys(reason);
#ifdef CONS_MQTT_ONLY
    // Mode CONS_MQTT_ONLY : on ne synchronise PAS persistedPulseCount avec
    // pulseCount avant écriture — pulseSave() est un no-op de toute façon,
    // et on ne veut surtout pas réintroduire de persistance du compteur
    // global (cause de l'auto-empoisonnement).
#else
    // Synchronise les pulses runtime avant d'écrire
    {
        unsigned long cnt;
        noInterrupts(); cnt = pulseCount; interrupts();
        unsigned long total = persistedPulseCount + cnt;
        // On enregistre le total complet (pas seulement les paliers de SAVE_LITRES_STEP)
        persistedPulseCount = total;
    }
    pulseSave();
#endif
    valveConsFlushDirty();
    manualValveFlushDirty();
    Serial.println("[SAFE] Flush NVS avant restart OK");
    delay(300);
    ESP.restart();
}


// ── Date du jour au format YYYYMMDD (0 si pas sync NTP)
inline uint32_t todayYMD(){
    struct tm ti;
    if(!getLocalTime(&ti,5)) return 0;
    return (uint32_t)((ti.tm_year+1900)*10000 + (ti.tm_mon+1)*100 + ti.tm_mday);
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
//
// ── Mode CONS_MQTT_ONLY (Globals.h) ─────────────────────────────────────
// Les COMPTEURS par vanne (pulsesTotal / todayPulses / carry / historique)
// ne sont PLUS persistés en NVS. Les compteurs vivent uniquement en RAM
// et sont ré-hydratés au boot via la recovery MQTT (voir
// MqttManager.h::mqttHandleMessage — les blocs "[RECOVERY] Vx litres_*"
// utilisent les valeurs retained publiées par HA lors de la session
// précédente). C'est la même stratégie que pour `pulse_total` (voir
// pulseLoad/pulseSave plus haut), appliquée à chaque vanne.
//
// En contrepartie, le coefficient de calibration `flowCoeff` reste
// persisté en NVS (valveConsSaveFlowCoeff ci-dessous) car c'est une
// donnée de configuration, pas un compteur. Une calibration perdue
// forcerait l'utilisateur à la refaire.
//
// Avantage : élimine l'auto-empoisonnement observé sur `pulse_total` ET
// sur les compteurs par vanne. Si HA retenait une valeur buggée
// (4.29G observée sur `pulse_total`), on n'écrasera plus la NVS avec à
// chaque reboot — mais on continue de récupérer la valeur MQTT (qui peut
// toujours être incorrecte). Le contrôle anti-poison dans la recovery
// (rejet si > 4.29G) reste donc essentiel, voir MqttManager.h.
inline void valveConsLoad(){
#ifdef CONS_MQTT_ONLY
    // Mode CONS_MQTT_ONLY : aucune lecture NVS. Les compteurs sont à 0
    // au boot et seront ré-hydratés par la recovery MQTT dans
    // mqttHandleMessage() dès que le broker livrera les valeurs retained.
    // flowCoeff reste géré par valveConsSaveFlowCoeff (donnée de config).
    for(int v=0;v<VANNE_COUNT;v++){
        valveCons[v].pulsesTotal = 0UL;
        valveCons[v].todayYmd    = 0;
        valveCons[v].todayPulses = 0;
        valveCons[v].carry       = 0.0f;
        for(int d=0;d<CONS_HISTORY_DAYS;d++){
            valveCons[v].history[d] = {0, 0, 0.0f};
        }
        valveCons[v].todayIdx    = 0;
    }
#else
    prefs.begin("irrcons", false);
    for(int v=0;v<VANNE_COUNT;v++){
        char key[24];
        // Total cumulé
        snprintf(key,sizeof(key),"v%d_pc",v);
        valveCons[v].pulsesTotal = prefs.getULong(key, 0UL);
        // Index jour courant + pulses du jour
        snprintf(key,sizeof(key),"v%d_td",v);
        valveCons[v].todayYmd = prefs.getUInt(key, 0);  // uint32_t — YYYYMMDD dépasse uint16_t
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
#endif
}

inline void valveConsSaveOne(int v){
#ifdef CONS_MQTT_ONLY
    // Mode CONS_MQTT_ONLY : pas de persistance NVS des compteurs.
    // La valeur vit en RAM et sera publiée en MQTT retained (HA fait foi).
    (void)v; // éviter warning unused
#else
    prefs.begin("irrcons", false);
    char key[24];
    snprintf(key,sizeof(key),"v%d_pc",v);
    prefs.putULong(key, valveCons[v].pulsesTotal);
    snprintf(key,sizeof(key),"v%d_td",v);
    prefs.putUInt(key, valveCons[v].todayYmd);  // uint32_t
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
#endif
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

// ────────────────────────────────────────────────────────────────────
// THROTTLING D'ÉCRITURE NVS POUR LA CONSO PAR VANNE
// ────────────────────────────────────────────────────────────────────
//
// DIAGNOSTIC (cf. logs du user) : putString() retournait NOT_ENOUGH_SPACE
// après quelques heures d'utilisation en irrigation réelle. Cause : le
// `pulseDistribute()` (ValveCons.h) appelait `valveConsSaveOne(i)` à CHAQUE
// tour de loop pour chaque vanne ouverte — soit ~10-20×/s × 5 vannes × ~17
// clés par Preferences::put = plusieurs centaines de flashes par seconde.
// La partition NVS (32 KB) sature en quelques heures à ce rythme, et tout
// putString ultérieur (y compris pour le SSID WiFi) échoue silencieusement.
// Symptôme observable : "le SSID semble ne pas se sauvegarder" alors que
// `configSave()` est bien appelé — c'est `prefs.putString` qui échoue.
//
// SOLUTION : on n'écrit en NVS que quand c'est utile, c'est-à-dire :
//   * Sur transition d'état vanne (ouvre/ferme) — appelle valveConsSaveOne()
//     directement pour persister immédiatement l'état
//   * Périodiquement (1×/30 s par vanne) tant qu'elle est ouverte
//   * Sur demande explicite (reset compteur, reset config, reboot propre)
//
// Le reste du temps, on travaille en RAM et un `valveConsDirty` flag
// marque les vannes qui ont besoin d'un flush. La perte maximale en cas
// de crash subite (sans reboot propre) est de 30 s de conso sur les
// vannes ouvertes — négligeable, et de toute façon rattrapable au reboot
// suivant grâce au recalcul `lastDistributedTotal` (cf. setup()).
//
// NB : le carry fractionnaire (algorithme de répartition) est lui aussi
// protégé : s'il n'est pas flush à temps, on perd < 1 pulse, ce qui est
// insignifiant.
extern volatile bool valveConsDirty[];  // défini dans Globals.cpp
inline void valveConsMarkDirty(int v){
    if(v<0||v>=VANNE_COUNT) return;
    valveConsDirty[v] = true;
}
inline void valveConsFlushDirty(){
    for(int v=0;v<VANNE_COUNT;v++){
        if(valveConsDirty[v]){
            valveConsSaveOne(v);
            valveConsDirty[v] = false;
        }
    }
}
// Flush immédiat de la conso d'une vanne spécifique en NVS.
// Appelé depuis :
//   - ValveManager.h::valveHardClose() — en mode CONS_MQTT_ONLY uniquement
//     (voir Globals.h) : persiste immédiatement les litres de la session
//     à chaque fermeture, puisque le flush périodique 30s est désactivé.
//   - /api/pulse/reset (WebManager.h) : persiste la remise à zéro.
//   - valveConsFlushDirty() : flush en lot (mode normal, toutes les 30s).
inline void valveConsFlushOne(int v){
    if(v<0||v>=VANNE_COUNT) return;
    valveConsSaveOne(v);
    valveConsDirty[v] = false;
}

// ────────────────────────────────────────────────────────────────────
// CONSOMMATION "VANNE MANUELLE" — persistance NVS
// ────────────────────────────────────────────────────────────────────
//
// Même schéma que valveConsLoad / valveConsSaveOne : namespace dédié
// "irrmnl" (pour "irrigation manual"), clés préfixées "mnl_". En mode
// CONS_MQTT_ONLY, les load/save sont des no-ops (cohérent avec le reste
// de la conso) — la valeur vit uniquement en RAM et sera ré-hydratée
// par la recovery MQTT (MqttManager.h) si elle a été publiée retained.
//
// En mode normal, on flush :
//   * immédiatement sur reset (via manualValveFlushOne)
//   * immédiatement sur transition vannes (toutes fermées → ouverture,
//     ou l'inverse) : voir valveHardOpen / valveHardClose
//   * périodiquement (toutes les 30 s) tant que manualValveDirty=true
inline void manualValveLoad(){
#ifdef CONS_MQTT_ONLY
    // Pas de persistance en CONS_MQTT_ONLY — la valeur sera ré-hydratée
    // par la recovery MQTT au boot si elle a été publiée retained.
#else
    prefs.begin("irrmnl", false);
    manualValveState.pulsesTotal = prefs.getULong("mnl_pt", 0UL);
    manualValveState.todayYmd    = prefs.getUInt("mnl_td", 0);
    manualValveState.todayPulses = prefs.getUInt("mnl_tp", 0);
    manualValveState.todayIdx    = prefs.getUShort("mnl_ti", 0);
    manualValveState.flowCoeff   = prefs.getFloat("mnl_fc", 1.0f);
    manualValveState.carry       = prefs.getFloat("mnl_cr", 0.0f);
    for(int d=0;d<CONS_HISTORY_DAYS;d++){
        char key[16]; snprintf(key,16,"mnl_h%d",d);
        size_t n = prefs.getBytes(key, &manualValveState.history[d], sizeof(DayStat));
        if(n != sizeof(DayStat)){
            manualValveState.history[d] = {0, 0, 0.0f};
        }
    }
    prefs.end();
    // Recalcule l'index d'écriture anneau (idem valveConsLoad)
    uint16_t idx = 0;
    for(int d=0;d<CONS_HISTORY_DAYS;d++){
        if(manualValveState.history[d].ymd == 0){ idx = d; break; }
        idx = (uint16_t)((d+1) % CONS_HISTORY_DAYS);
    }
    manualValveState.todayIdx = idx;
#endif
}

inline void manualValveSaveOne(){
#ifdef CONS_MQTT_ONLY
    // no-op (cohérent avec valveConsSaveOne)
#else
    prefs.begin("irrmnl", false);
    prefs.putULong("mnl_pt", manualValveState.pulsesTotal);
    prefs.putUInt("mnl_td",  manualValveState.todayYmd);
    prefs.putUInt("mnl_tp",  manualValveState.todayPulses);
    prefs.putUShort("mnl_ti", manualValveState.todayIdx);
    prefs.putFloat("mnl_fc", manualValveState.flowCoeff);
    prefs.putFloat("mnl_cr", manualValveState.carry);
    for(int d=0;d<CONS_HISTORY_DAYS;d++){
        char key[16]; snprintf(key,16,"mnl_h%d",d);
        prefs.putBytes(key, &manualValveState.history[d], sizeof(DayStat));
    }
    prefs.end();
#endif
}

// Flush immédiat sur demande (reset Web, transition de vanne, etc.).
// Appelé par :
//   - WebManager.h::pulse_reset (en plus de valveConsSaveOne)
//   - valveHardOpen / valveHardClose pour persister immédiatement quand
//     une transition vannes ↔ vanne manuelle a eu lieu (évite une perte
//     de 30 s de conso en cas de coupure).
inline void manualValveFlushOne(){
    manualValveSaveOne();
    manualValveDirty = false;
}

// Flush en lot, utilisé par la loop() toutes les 30 s en mode normal.
inline void manualValveFlushDirty(){
    if(manualValveDirty){
        manualValveFlushOne();
    }
}

// ────────────────────────────────────────────────────────────
inline void historyLoad(){
#ifdef CONS_MQTT_ONLY
    // Pas de persistance en CONS_MQTT_ONLY — la valeur sera ré-hydratée
    // par la recovery MQTT au boot si elle a été publiée retained.
#else
    prefs.begin("irrhist", false);
    historyData.headIdx = prefs.getUChar("hist_hi", 0);
    historyData.initialized = prefs.getBool("hist_init", false);
    for(int d=0;d<HISTORY_DAYS;d++){
        char dateKey[16]; snprintf(dateKey,16,"hist_d%d",d);
        historyData.days[d].totalLitres = prefs.getFloat(dateKey, 0.0f);
        for(int v=0;v<VANNE_COUNT;v++){
            char valveLitresKey[32]; snprintf(valveLitresKey,32,"hist_vl_%d_%d",d,v);
            historyData.days[d].valveLitres[v] = prefs.getFloat(valveLitresKey, 0.0f);
            char valveTotalKey[32]; snprintf(valveTotalKey,32,"hist_vt_%d_%d",d,v);
            historyData.days[d].valveTotal[v] = prefs.getFloat(valveTotalKey, 0.0f);
        }
    }
    prefs.end();
#endif
}

inline void historySaveOne(){
#ifdef CONS_MQTT_ONLY
    // no-op (cohérent avec valveConsSaveOne)
#else
    prefs.begin("irrhist", false);
    prefs.putUChar("hist_hi", historyData.headIdx);
    prefs.putBool("hist_init", historyData.initialized);
    for(int d=0;d<HISTORY_DAYS;d++){
        char dateKey[16]; snprintf(dateKey,16,"hist_d%d",d);
        prefs.putFloat(dateKey, historyData.days[d].totalLitres);
        for(int v=0;v<VANNE_COUNT;v++){
            char valveLitresKey[32]; snprintf(valveLitresKey,32,"hist_vl_%d_%d",d,v);
            prefs.putFloat(valveLitresKey, historyData.days[d].valveLitres[v]);
            char valveTotalKey[32]; snprintf(valveTotalKey,32,"hist_vt_%d_%d",d,v);
            prefs.putFloat(valveTotalKey, historyData.days[d].valveTotal[v]);
        }
    }
    prefs.end();
#endif
}

inline void historyFlushOne(){
    historySaveOne();
}

inline void historyFlushDirty(){
    // Pas de flag dirty pour l'historique — on sauvegarde à chaque modification
    // (via historySaveOne) pour garantir la persistance immédiate.
    // Si besoin d'un flush périodique, on peut ajouter un flag.
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

// ============================================================
// ÉTAT NVS (remplissage) + FORMATAGE AVEC PRÉSERVATION WiFi
// ============================================================
//
// But : exposer à l'UI (page Configuration) le niveau de remplissage de
// la partition NVS, et permettre à l'utilisateur de la reformater
// depuis l'UI. Le formatage efface TOUTE la NVS (config, conso par
// vanne, programmes, calibration, journal) — c'est un mode recovery.
// IMPORTANT : on préserve sysConfig.ssid / sysConfig.wifiPass en RAM
// pendant l'opération pour les réécrire en NVS juste après l'erase,
// afin de ne pas perdre le réseau WiFi.
//
// Implémentation :
//   * nvsStats()         : remplit une nvs_stats_t via nvs_get_stats().
//   * nvsStatsToJson()   : sérialise au format JSON pour l'API REST
//                           et le broadcast WebSocket STATUS.
//   * nvsFormatAndRestore(): efface NVS, recharge configLoad() (qui
//                           recrée des défauts), réécrit ssid+wifiPass
//                           courants (préservés en RAM), puis reboote.
//   * nvsStatsCached     : cache temps réel (refresh ~5s) pour ne pas
//                           marteler le driver NVS à chaque broadcast WS.
struct NvsStats {
    size_t usedEntries;
    size_t freeEntries;
    size_t totalEntries;
    bool   ok;
};
inline NvsStats nvsStatsCached = {0,0,0,false};
inline unsigned long nvsStatsLastMs = 0;

inline NvsStats nvsStats(){
    NvsStats s = {0,0,0,false};
    // nvs_get_stats() remplit la structure avec le détail de la partition
    // NVS. Disponible sur ESP32 Arduino Core 2.x+ (et ESP-IDF). Renvoie
    // ESP_OK si succès.
    nvs_stats_t st;
    if(nvs_get_stats(NULL, &st) == ESP_OK){
        s.usedEntries  = st.used_entries;
        s.freeEntries  = st.free_entries;
        s.totalEntries = st.total_entries;
        s.ok = true;
    }
    return s;
}

inline void nvsStatsRefresh(){
    nvsStatsCached = nvsStats();
    nvsStatsLastMs = millis();
}

inline String nvsStatsToJson(){
    // Si cache trop vieux (> 5s) on rafraîchit pour rester pertinent
    if(millis() - nvsStatsLastMs > 5000UL){
        nvsStatsRefresh();
    }
    StaticJsonDocument<256> doc;
    doc["ok"]     = nvsStatsCached.ok;
    doc["used"]   = (long)nvsStatsCached.usedEntries;
    doc["free"]   = (long)nvsStatsCached.freeEntries;
    doc["total"]  = (long)nvsStatsCached.totalEntries;
    if(nvsStatsCached.totalEntries > 0){
        int pct = (int)((100UL * nvsStatsCached.usedEntries) / nvsStatsCached.totalEntries);
        if(pct > 100) pct = 100;
        doc["usedPct"] = pct;
    } else {
        doc["usedPct"] = 0;
    }
    String out; serializeJson(doc, out);
    return out;
}

// Formate la partition NVS et préserve ssid/wifiPass. Redémarre ensuite.
// ATTENTION : cette opération est IRRÉVERSIBLE. Toutes les données
// persistées (config complète sauf ssid/pass, conso par vanne,
// programmes, calibration, journal) sont effacées. Les vannes en cours
// d'ouverture sont refermées avant le reboot.
inline void nvsFormatAndRestore(){
    // ── 1) Sauvegarder en RAM le SSID et mot de passe WiFi courants
    char keepSsid[32]; strlcpy(keepSsid, sysConfig.ssid, 32);
    char keepPass[64]; strlcpy(keepPass, sysConfig.wifiPass, 64);
    char keepNodeId[24]; strlcpy(keepNodeId, sysConfig.nodeId, 24);
    logSys("FORMAT NVS demandé — effacement en cours...");

    // ── 2) Fermer toutes les vannes par sécurité (matériel d'abord)
    for(int i=0;i<VANNE_COUNT;i++){
        digitalWrite(VANNE_PINS[i], LOW);
        digitalWrite(LEDVISU_PINS[i], LOW);
        valves[i].isOpen = false;
        valves[i].source = CmdSource::NONE;
    }
    for(int i=0;i<(int)(sizeof(OUT_PINS)/sizeof(OUT_PINS[0])); i++){
        digitalWrite(OUT_PINS[i], LOW);
    }

    // ── 3) Effacer la partition NVS
    esp_err_t err = nvs_flash_erase();
    Serial.printf("[NVS] nvs_flash_erase() = %s\n",
                  err == ESP_OK ? "OK" : "FAIL");
    if(err != ESP_OK){
        logSys("FORMAT NVS ÉCHEC — reboot sans formatage");
        delay(500);
        ESP.restart();
        return;
    }

    // ── 4) Réinitialiser la NVS (re-crée les structures internes)
    err = nvs_flash_init();
    Serial.printf("[NVS] nvs_flash_init() = %s\n",
                  err == ESP_OK ? "OK" : "FAIL");
    logSys("NVS effacée et réinitialisée");

    // ── 5) Restaurer ssid/pass/nodeId (réécrit en NVS immédiatement)
    strlcpy(sysConfig.ssid, keepSsid, 32);
    strlcpy(sysConfig.wifiPass, keepPass, 64);
    strlcpy(sysConfig.nodeId, keepNodeId, 24);
    // configSave() va utiliser les autres champs de sysConfig (valeurs
    // par défaut puisque configLoad() n'a pas été rejouée avant
    // configSave() — c'est volontaire, on repart sur des défauts sains).
    configSave();
    Serial.printf("[NVS] WiFi restauré: ssid='%s'\n", sysConfig.ssid);
    logSys("FORMAT NVS terminé — WiFi préservé, redémarrage...");

    // ── 6) Petit délai pour laisser le temps au log d'être flush
    delay(300);
    ESP.restart();
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
