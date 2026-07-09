#pragma once
#ifdef IOCAN
// ============================================================
// AlarmManager.h — Détection d'anomalies hydrauliques
// ============================================================
// Deux alarmes surveillées en continu par alarmTick() (appelée 1×/s
// depuis loop()) :
//
//   1) ALARM_NO_FLOW_WHEN_OPEN
//      Au moins une vanne est ouverte ET AUCUN débit significatif n'est
//      détecté sur la fenêtre FLOW_WINDOW_MS après un délai de grâce.
//      Symptôme typique : électrovanne HS, solénoïde défectueux, coupure
//      d'alimentation 12V du manifold, vanne manuelle fermée en amont.
//
//   2) ALARM_UNEXPECTED_FLOW
//      AUCUNE vanne ouverte ET un débit ≥ ALARM_LEAK_LPM (0.2 L/min) est
//      détecté sur la fenêtre glissante. Symptôme typique : fuite sur le
//      réseau (joint, électrolyse, électrovanne qui ne se ferme plus).
//
// Anti-fausses-alarmes :
//   - ALARM_NO_FLOW_DELAY_MS (10 s par défaut) après chaque ouverture :
//     laisse au système le temps d'amorcer l'eau dans les canalisations
//     (les vannes motorisées 24V prennent typiquement 1-3 s à s'ouvrir, et
//     le premier pulse arrive après que la chambre du capteur se soit
//     remplie — sur une longue ligne vide, ça peut prendre 5-8 s).
//   - ALARM_UNEXPECTED_FLOW_DELAY_MS (15 s par défaut) après chaque
//     fermeture : le résidu d'eau dans la chambre du capteur peut
//     produire 1-3 pulses "fantômes" dans la fenêtre glissante après
//     fermeture, on attend que ça se stabilise.
//   - Pour ALARM_NO_FLOW, on exige un débit > 0 (>= 0.05 L/min) PUIS un
//     retour à 0 — pas de lever d'alarme pendant la phase d'amorçage où
//     flow_lpm est encore en train de monter.
//
// État publié :
//   - alarmCode (uint8_t, 0 = OK) : 1 = NO_FLOW, 2 = UNEXPECTED_FLOW
//   - alarmActive (bool) : true si une alarme est en cours
//   - alarmSinceMs : timestamp millis() du passage en alarme
//   - alarmMsg : libellé court pour les logs / LoRa
// ============================================================

#include "Globals.h"
#include "LoggerManager.h"
#include "FlowMeter.h"

// ── Configuration (constantes — pourrait migrer en NVS plus tard) ──
#define ALARM_NO_FLOW_DELAY_MS        10000UL   // 10 s après ouverture avant vérification
#define ALARM_UNEXPECTED_FLOW_DELAY_MS 15000UL  // 15 s après dernière fermeture
#define ALARM_LEAK_LPM                0.2f      // seuil fuite (L/min)
#define ALARM_NO_FLOW_MIN_LPM         0.05f     // seuil "débit significatif" (L/min)

// ── Helpers de comptage vannes ouvertes (factorisés pour la lisibilité) ──
inline int alarmCountOpenValves(){
    int n = 0;
    for(int i=0;i<VANNE_COUNT;i++) if(valves[i].isOpen) n++;
    return n;
}

// ── Initialisation (appelée depuis setup() ou implicitement à 0) ──
// Les variables du struct AlarmState sont déjà à 0 (initializer C++),
// pas besoin de reset explicite. Conservé pour la symétrie / évolution future.
inline void alarmReset(){
    alarmState.code        = 0;
    alarmState.active      = false;
    alarmState.sinceMs     = 0;
    alarmState.msg[0]      = '\0';
    alarmState.openSinceMs = 0;   // pas de vanne ouverte "en cours"
    alarmState.lastCloseMs = millis(); // pour gérer le délai post-fermeture
}

// ── À appeler à chaque ouverture de vanne réussie (depuis valveHardOpen)
//    pour réinitialiser le délai de grâce.
inline void alarmOnValveOpened(int idx){
    (void)idx;
    alarmState.openSinceMs = millis();
    // Si une alarme NO_FLOW était en cours, on la lève : l'utilisateur vient
    // d'ouvrir une vanne, l'amorçage va prendre quelques secondes — on évite
    // de re-déclencher l'alarme dans la même seconde.
    if(alarmState.active && alarmState.code == 1){
        alarmState.active = false;
        alarmState.code = 0;
        logSys("[ALARM] NO_FLOW levée (nouvelle ouverture vanne)");
    }
}

// ── À appeler à chaque fermeture de vanne (depuis valveHardClose)
inline void alarmOnValveClosed(int idx){
    (void)idx;
    if(alarmCountOpenValves() == 0){
        alarmState.lastCloseMs = millis();
    }
    // Si une alarme UNEXPECTED_FLOW était en cours et qu'on vient de fermer
    // la seule vanne qui aurait pu causer le débit : on lève l'alarme.
    // (Cas typique : électrovanne HS qui ne se ferme plus → on la force via
    //  valveHardClose → l'alarme fuite doit retomber.)
    if(alarmState.active && alarmState.code == 2 && alarmCountOpenValves() == 0){
        alarmState.active = false;
        alarmState.code = 0;
        logSys("[ALARM] UNEXPECTED_FLOW levée (toutes vannes fermées)");
    }
}

// ── Machine d'état des alarmes — appelée 1×/s depuis loop() ──
// Lit flowCurrentLpm (déjà lissé sur FLOW_WINDOW_MS par flowUpdate() dans
// la même passe 1Hz) et l'état des vannes, et met à jour alarmState.
inline void alarmTick(){
    unsigned long now = millis();
    int openCount = alarmCountOpenValves();
    float flow = flowCurrentLpm;

    // ── 1) NO_FLOW_WHEN_OPEN ──
    // Au moins une vanne ouverte, délai de grâce écoulé depuis la dernière
    // ouverture, et débit < seuil "significatif".
    if(openCount > 0){
        unsigned long sinceOpen = (alarmState.openSinceMs == 0)
                                  ? ALARM_NO_FLOW_DELAY_MS // pas encore armé
                                  : (now - alarmState.openSinceMs);
        if(sinceOpen >= ALARM_NO_FLOW_DELAY_MS){
            if(flow < ALARM_NO_FLOW_MIN_LPM){
                if(!alarmState.active || alarmState.code != 1){
                    // Transition IDLE → ALARM
                    alarmState.active  = true;
                    alarmState.code    = 1;
                    alarmState.sinceMs = now;
                    snprintf(alarmState.msg, sizeof(alarmState.msg),
                             "Vanne(s) ouverte(s) mais aucun débit (%.2f L/min)", (double)flow);
                    logSys(alarmState.msg);
                }
            } else {
                // Débit OK → si une alarme NO_FLOW était en cours, on la lève
                if(alarmState.active && alarmState.code == 1){
                    alarmState.active = false;
                    alarmState.code   = 0;
                    logSys("[ALARM] NO_FLOW levée (débit rétabli)");
                }
            }
        }
        // Si depuisOpen < ALARM_NO_FLOW_DELAY_MS, on est dans la phase
        // d'amorçage — on ne fait rien, même si le débit est encore faible.
    } else {
        // Aucune vanne ouverte : on ne peut PAS être en alarme NO_FLOW.
        // La levée est faite dans alarmOnValveClosed() lors de la transition,
        // mais on la duplique ici par sécurité (au cas où openSinceMs n'a
        // pas été mis à jour correctement lors d'une transition rapide).
        if(alarmState.active && alarmState.code == 1){
            alarmState.active = false;
            alarmState.code   = 0;
        }
    }

    // ── 2) UNEXPECTED_FLOW ──
    // Aucune vanne ouverte, délai post-fermeture écoulé, et débit ≥ seuil fuite.
    if(openCount == 0){
        unsigned long sinceClose = (alarmState.lastCloseMs == 0)
                                   ? ALARM_UNEXPECTED_FLOW_DELAY_MS
                                   : (now - alarmState.lastCloseMs);
        if(sinceClose >= ALARM_UNEXPECTED_FLOW_DELAY_MS){
            if(flow >= ALARM_LEAK_LPM){
                if(!alarmState.active || alarmState.code != 2){
                    alarmState.active  = true;
                    alarmState.code    = 2;
                    alarmState.sinceMs = now;
                    snprintf(alarmState.msg, sizeof(alarmState.msg),
                             "Fuite détectée: %.2f L/min alors qu'aucune vanne n'est ouverte", (double)flow);
                    logSys(alarmState.msg);
                }
            } else {
                // Débit retombé sous le seuil fuite
                if(alarmState.active && alarmState.code == 2){
                    alarmState.active = false;
                    alarmState.code   = 0;
                    logSys("[ALARM] UNEXPECTED_FLOW levée (débit retombé)");
                }
            }
        }
    }
    // Note : on NE lève PAS l'alarme UNEXPECTED_FLOW quand une vanne s'ouvre
    // (logique : un débit attendu avec vanne ouverte n'est plus une fuite).
    // C'est la fermeture explicite de la vanne (alarmOnValveClosed) qui la
    // lève, et alarmTick() la re-déclenchera si le débit persiste après le
    // délai post-fermeture.
}

#endif // IOCAN
