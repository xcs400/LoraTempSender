#pragma once
#ifdef IOCAN
// ============================================================
// FlowMeter.h — Débitmètre partagé (WebSocket + MQTT)
// ============================================================
// Correspond à la SECTION 3b du fichier d'origine.
//
// CORRECTIF : auparavant le calcul de débit instantané (L/min) vivait
// dans des variables `static` locales à buildStatusJson(), donc invisible
// depuis mqttPublishState() qui republiait toujours 0.0 pour flow_lpm.
// On extrait le calcul ici, dans une fonction partagée par les deux
// consommateurs (WebSocket et MQTT).
//
// Calcul : on garde en RAM un anneau de (timestamp, pulses) sur une
// fenêtre glissante de FLOW_WINDOW_MS millisecondes. flowUpdate() est
// appelée ~1×/s depuis loop() et recalcule le débit moyen sur tous les
// intervalles de l'anneau qui tombent dans la fenêtre — ce qui lisse les
// fluctuations qu'aurait un calcul instantané sur un débitmètre à faible
// résolution.
// ============================================================

#include "Globals.h"

struct FlowSample { unsigned long tMs; unsigned long pulses; };
// NOTE : ce header est conçu pour n'être inclus que depuis MainIocan.cpp
// (translation unit unique). On utilise `inline` (et non `static`) pour
// que ces variables aient une définition unique partagée même si jamais
// ce header se retrouvait inclus depuis plusieurs .cpp (C++17).
inline FlowSample flowRing[FLOW_SAMPLES];
inline uint8_t    flowHead = 0;     // index du prochain échantillon à écrire
inline uint8_t    flowCount = 0;    // nombre d'échantillons valides (max FLOW_SAMPLES)
inline float      flowCurrentLpm = 0.0f;  // dernière valeur calculée, publiée telle quelle

// À appeler à chaque tour de loop() avec le totalPulses courant.
// Met à jour l'anneau et recalcule flowCurrentLpm sur la fenêtre FLOW_WINDOW_MS.
inline void flowUpdate(unsigned long totalPulses){
    unsigned long nowMs = millis();
    // Pousse le nouvel échantillon dans l'anneau
    flowRing[flowHead].tMs    = nowMs;
    flowRing[flowHead].pulses = totalPulses;
    flowHead = (flowHead + 1) % FLOW_SAMPLES;
    if(flowCount < FLOW_SAMPLES) flowCount++;

    // Calcul du débit = MOYENNE sur tous les échantillons de la fenêtre.
    // En moyennant sur tous les échantillons tombant dans la fenêtre
    // FLOW_WINDOW_MS, on lisse les fluctuations et on obtient une valeur
    // stable qui représente le débit RÉEL moyen.
    unsigned long cutoff = (nowMs > FLOW_WINDOW_MS) ? (nowMs - FLOW_WINDOW_MS) : 0;
    unsigned long totalDeltaMs = 0;
    unsigned long totalDeltaP  = 0;
    int usedSamples = 0;
    // Échantillon "courant" = celui qu'on vient d'écrire (head-1)
    int curIdx = (flowHead - 1 + FLOW_SAMPLES) % FLOW_SAMPLES;
    unsigned long curPulse = flowRing[curIdx].pulses;
    // On parcourt l'anneau en ordre chronologique inverse (du plus récent
    // au plus vieux) et on accumule les deltas successifs tant que l'écart
    // temporel reste dans la fenêtre.
    unsigned long prevMs = flowRing[curIdx].tMs;
    unsigned long prevPulse = curPulse; // initialisé = courant, sera décrémenté
    for(int k=1; k<flowCount; k++){
        int idx = (flowHead - 1 - k + FLOW_SAMPLES) % FLOW_SAMPLES;
        if(flowRing[idx].tMs < cutoff) break; // sortie de fenêtre
        unsigned long thisMs    = flowRing[idx].tMs;
        unsigned long thisPulse = flowRing[idx].pulses;
        unsigned long dMs = prevMs - thisMs;
        if(dMs > 0){
            totalDeltaMs += dMs;
            if(prevPulse > thisPulse) totalDeltaP += (prevPulse - thisPulse);
            usedSamples++;
        }
        // Avance : this devient prev pour l'itération suivante
        prevMs    = thisMs;
        prevPulse = thisPulse;
    }
    // Si on n'a qu'un seul échantillon (pas de recul), on garde la
    // dernière valeur connue plutôt que de retourner 0.
    if(usedSamples == 0 || totalDeltaMs == 0){
        // flowCurrentLpm reste à sa valeur précédente
    } else {
        float litresDelta = (float)totalDeltaP / PULSES_PER_LITRE;
        flowCurrentLpm = litresDelta * (60000.0f / (float)totalDeltaMs);
    }
}

// Accesseur pour MQTT/WS : renvoie la valeur lissée courante.
// Conservé sous le même nom pour ne pas casser les call sites existants.
inline float computeFlowLpm(unsigned long /*totalPulsesIgnored*/){
    return flowCurrentLpm;
}

#endif // IOCAN
