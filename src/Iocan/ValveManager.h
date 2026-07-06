#pragma once
#ifdef IOCAN
// ============================================================
// ValveManager.h — États, priorités, sécurités des vannes
// ============================================================
// Correspond à la SECTION 7 du fichier d'origine.
//
// Dépend de calibState (Globals.h) pour la garde de sécurité calibration
// dans valveHardOpen() : pendant une calibration, seule la vanne en cours
// de mesure peut être ouverte, quelle que soit la source de la commande.
// ============================================================

#include "Globals.h"
#include "LoggerManager.h"

// Ferme physiquement une vanne, met à jour état
inline void valveHardClose(int idx){
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
#ifdef CONS_MQTT_ONLY
    // Mode CONS_MQTT_ONLY : pas de flush NVS ici. En mode CONS_MQTT_ONLY,
    // valveConsFlushOne() est un no-op (les compteurs par vanne ne sont
    // pas persistés en NVS — ConfigManager.h). La fermeture de vanne
    // publie simplement l'état MQTT à la prochaine itération de
    // mqttLoop() (mqttPublishState), qui sera retained sur HA. La
    // cohérence "valeur MQTT après reboot = valeur de la session
    // précédente" reste ainsi garantie via le broker, sans aucune
    // écriture NVS.
    // (L'appel à valveConsFlushOne est conservé par compatibilité de
    // chemin de code, mais n'effectue rien en CONS_MQTT_ONLY.)
    valveConsFlushOne(idx);
#endif
}

// Ouvre physiquement une vanne avec priorité et durée
// Retourne false si refusé (priorité inférieure)
inline bool valveHardOpen(int idx, CmdSource src, uint32_t durationSec){
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
inline bool valveClose(int idx, CmdSource src){
    if(idx<0||idx>=VANNE_COUNT) return false;
    int prio = srcPrio(src);
    if(valves[idx].isOpen && valves[idx].priority < prio) return false;
    valveHardClose(idx);
    return true;
}

// Mise à jour des timers vannes (appelé dans loop, sans delay)
inline void valveUpdate(){
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
inline void valveCloseAll(CmdSource src=CmdSource::WEB){
    for(int i=0;i<VANNE_COUNT;i++) valveHardClose(i);
    // ensure visualization LEDs are also cleared
    for(int i=0;i<VANNE_COUNT;i++) digitalWrite(LEDVISU_PINS[i], LOW);
    logSys("Toutes vannes fermées");
}

#endif // IOCAN
