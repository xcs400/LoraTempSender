#pragma once
#ifdef IOCAN
// ============================================================
// ManualInput.h — ManualOverrideManager (entrées physiques)
// ============================================================
// Correspond à la SECTION 9 du fichier d'origine.
// ============================================================

#include "Globals.h"
#include "LoggerManager.h"
#include "ValveManager.h"

inline void inputUpdate(){
    unsigned long now = millis();
    for(int i=0;i<VANNE_COUNT;i++){
        bool pressed = (digitalRead(FORCE_INPUT_PINS[i]) == LOW);
        if(pressed && !inputActive[i]){
            // Début appui (mémoriser timestamp)
            inputPressMs[i] = now;
            inputActive[i]  = true;
        }
        else if(!pressed && inputActive[i]){
            // Relâchement -> vérifier debounce puis toggle l'état de la vanne
            unsigned long dur = now - inputPressMs[i];
            inputActive[i] = false;
            const unsigned long DEBOUNCE_MS = 50;
            if(dur < DEBOUNCE_MS) {
                // bruit, ignorer
                continue;
            }
            // Toggle: si ouverte -> fermer, sinon ouvrir pour la durée configurée
            if(valves[i].isOpen){
                // Si bouton OFF: forcer la fermeture même si la vanne était forcée par Web/Prog
                valveHardClose(i);
                logAdd(i, "Fermée — bouton OFF (annule forçage)");
            } else {
                bool ok = valveHardOpen(i, CmdSource::PHYS_INPUT, sysConfig.manualForceSec);
                if(!ok) logAdd(i, "Forçage bouton ignoré (calibration en cours ou priorité supérieure)");
            }
        }
    }
}

#endif // IOCAN
