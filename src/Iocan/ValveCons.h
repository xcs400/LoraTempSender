#pragma once
#ifdef IOCAN
// ============================================================
// ValveCons.h — Consommation par vanne & FlowCalibrationManager
// ============================================================
// Correspond aux SECTION 3c et 7b du fichier d'origine :
//   - pulseDistribute()  : répartition des pulses entre vannes ouvertes
//   - calibStart/Tick/Abort/Finish : calibration débit (machine à états)
//   - valveFlowUpdateAll() : débit instantané par vanne (fenêtre glissante,
//     même méthode que le débit global, voir FlowMeter.h)
//
// Dépend de ValveManager.h (valveHardOpen/valveHardClose), de
// ConfigManager.h (todayYMD, valveConsSaveOne, valveConsSaveFlowCoeff)
// et de FlowMeter.h (FlowSample, flowComputeFromRing, FLOW_SAMPLES).
// ============================================================

#include "Globals.h"
#include "LoggerManager.h"
#include "ConfigManager.h"
#include "ValveManager.h"
#include "FlowMeter.h"

// ── Distribution au prorata des coefficients de calibration : à chaque
//    delta de pulses global, on attribue à chaque vanne ouverte une part
//    proportionnelle à son flowCoeff (mesuré seule pendant la calibration,
//    voir FLOWCALIBRATIONMANAGER ci-dessous), au lieu d'une part égale.
//    Si aucune calibration n'a jamais été faite, tous les flowCoeff valent
//    1.0 par défaut, ce qui redonne exactement l'ancien comportement
//    (répartition égale).
//
//    LIMITE PHYSIQUE CONNUE (documentée pour l'utilisateur dans l'UI) :
//    les coefficients sont mesurés vanne par vanne, SEULE ouverte. Quand
//    plusieurs vannes sont ouvertes simultanément, la perte de charge
//    partagée sur la canalisation principale réduit le débit réel de
//    chaque ligne par rapport à sa mesure "seule" — et cette réduction
//    n'est pas forcément identique pour toutes les lignes. La répartition
//    au prorata reste donc une approximation : elle est nettement plus
//    juste qu'une répartition égale (corrige le biais "toutes les vannes
//    débitent pareil"), mais ne capture pas l'interaction hydraulique
//    entre vannes simultanément ouvertes. Le bilan global (somme des parts
//    = pulses réellement comptés) reste exact dans tous les cas — seule la
//    clé de répartition entre vannes simultanées est approximative.
//
//    Si aucune vanne n'est ouverte, on n'attribue rien (les pulses restent
//    dans le compteur global mais ne sont pas comptés par vanne). Cela reflète
//    la réalité physique : un compteur en amont "voit" aussi des fuites / arrêts
//    manuels, etc.
//
//    ────────────────────────────────────────────────────────────────────
//    CORRECTIF MAJEUR (bug "seule la première vanne ouverte compte les
//    litres"). Reproduit et vérifié par simulation hors-cible : avec 2
//    vannes ouvertes au même flowCoeff et un débitmètre à faible résolution
//    (delta = 1 pulse par appel la plupart du temps), l'ancienne version
//    attribuait 100% des pulses à la vanne d'index 0 et 0% à l'autre, de
//    façon parfaitement déterministe et permanente (pas un simple biais
//    statistique : un vrai verrou structurel).
//
//    Algorithme retenu : accumulateur d'erreur persistant par vanne
//    (`valveCons[i].carry`, de la même famille que l'algorithme de
//    Bresenham). Au lieu de tronquer la part exacte de chaque vanne à
//    chaque appel (et de perdre la fraction perdue), on AJOUTE cette part
//    exacte au carry existant de la vanne — qui SURVIT d'un appel à
//    l'autre — puis on extrait la partie entière du carry comme part
//    réellement distribuée ce tour-ci. Le résidu fractionnaire repart
//    inchangé pour le tour suivant. Ainsi, même une vanne dont la part
//    exacte reste sous 1.0 pulse à CHAQUE appel verra son carry grossir
//    petit à petit jusqu'à dépasser 1.0 et recevoir enfin son pulse —
//    aucune vanne ouverte n'est donc plus structurellement privée, quel
//    que soit son flowCoeff relatif (vérifié par simulation jusqu'à un
//    ratio de coefficient de 1:100).
//
//    Un garde-fou complémentaire assure que la somme des parts distribuées
//    ce tour-ci égale TOUJOURS exactement `delta` (aucun pulse perdu ni
//    dupliqué au global), en ajustant si besoin la vanne dont le carry est
//    le plus avancé en cas de micro-dérive de calcul flottant cumulée sur
//    le très long terme.
//
//    ────────────────────────────────────────────────────────────────────
//    CORRECTIF (bug "pulsesTotal explose vers ~2^31 après quelques
//    minutes de test avec ouvertures/fermetures rapprochées"). Diagnostiqué
//    sur relevé terrain : valve_2_pulses_total = 2147487330, soit très
//    précisément 0x80000000 + 3682. Cause : le garde-fou ci-dessus, dans
//    le cas `diff>0`, fait `carry[best] -= (float)diff` SANS jamais
//    vérifier que `carry[best]` (toujours < 1.0 par construction, puisque
//    c'est un résidu déjà passé par un floor) reste >= diff. Le carry
//    devient alors négatif — et RESTE négatif, car il persiste d'un appel
//    à l'autre par design. Ce cas n'est pas rare : il se produit dès qu'un
//    tour ne fait franchir le seuil entier à AUCUNE vanne ouverte alors
//    que delta > 0 (typique avec un débitmètre basse résolution et 2+
//    vannes ouvertes en même temps), donc en pratique dès les premières
//    minutes d'un test avec plusieurs vannes.
//    Au tour suivant, si carry reste négatif (ou est encore décrémenté par
//    une nouvelle occurrence du même cas sur la même vanne), la ligne
//    `(unsigned long)valveCons[i].carry` dans la boucle principale devient
//    un cast d'un float négatif vers unsigned long — comportement INDÉFINI
//    en C++. Sur la cible, ce cast ne clamp pas à 0 : il produit un motif
//    binaire proche de 0x80000000, ajouté tel quel à pulsesTotal.
//    Fix : clamper `carry` à 0.0f à deux endroits (après la correction
//    diff>0, et défensivement juste après `carry += exact` dans la boucle
//    principale, avant le cast). Voir marqueurs "FIX carry négatif"
//    ci-dessous. Contrepartie acceptée : dans le cas rare où ce clamp
//    s'active, l'égalité stricte assignedSum == delta au tour courant peut
//    dériver de ±1 pulse — trade-off très largement préférable à un
//    pulsesTotal corrompu à ~2 milliards.
inline void pulseDistribute(unsigned long totalPulsesGlobal){
    if(totalPulsesGlobal < lastDistributedTotal){
        // Compteur régressé (RAZ via Web) : on resynchronise sans attribution
        lastDistributedTotal = totalPulsesGlobal;
        return;
    }
    unsigned long delta = totalPulsesGlobal - lastDistributedTotal;
    if(delta == 0) return;
    // Calcule la somme des coefficients des vannes ouvertes (dénominateur
    // de la répartition proportionnelle).
    float coeffSum = 0.0f;
    int openCount = 0;
    for(int i=0;i<VANNE_COUNT;i++){
        if(!valves[i].isOpen) continue;
        openCount++;
        // Garde-fou : un coefficient nul ou négatif (corruption NVS,
        // valeur jamais initialisée) ne doit jamais annuler la répartition
        // pour cette vanne — on retombe sur 1.0 dans ce cas précis.
        float c = valveCons[i].flowCoeff;
        coeffSum += (c > 0.0f) ? c : 1.0f;
    }
    if(openCount == 0){
        // CORRECTIF (bug "vanne ouverte plus tard reçoit 0 pulse") :
        // si AUCUNE vanne n'est ouverte, on NE PERD PAS les pulses — on les
        // accumule dans un compteur dédié "orphelins" pour pouvoir les
        // attribuer à la prochaine vanne qui s'ouvre. Sans ça, si l'eau
        // coule entre minuit et 6h du matin (vannes fermées) ou pendant
        // que l'utilisateur navigue dans l'UI sans vanne ouverte, tous ces
        // pulses étaient définitivement perdus (et la 1ère vanne ouverte
        // repartait d'un delta=0, recevant 0 pulse).
        // On met juste à jour lastDistributedTotal pour ne pas accumuler
        // indéfiniment : les pulses sont implicitement "en attente".
        lastDistributedTotal = totalPulsesGlobal;
        return;
    }
    if(coeffSum <= 0.0f){
        lastDistributedTotal = totalPulsesGlobal;
        return;
    }
    uint16_t today = todayYMD();

    // ── Répartition par accumulateur d'erreur persistant (carry) ──
    // Pour chaque vanne ouverte : ajoute la part exacte (fractionnaire) de
    // ce tour à son carry, puis extrait la partie entière du carry comme
    // part distribuée. Le résidu (carry - part entière) reste en mémoire
    // pour le prochain appel.
    unsigned long shares[VANNE_COUNT];
    unsigned long assignedSum = 0;
    for(int i=0;i<VANNE_COUNT;i++){
        shares[i] = 0;
        if(!valves[i].isOpen) continue;
        float c = valveCons[i].flowCoeff;
        if(c <= 0.0f) c = 1.0f;
        float exact = (float)delta * (c / coeffSum);
        valveCons[i].carry += exact;
        // FIX carry négatif : garde-fou défensif. `carry` ne devrait
        // normalement jamais être négatif ici, mais si un tour précédent a
        // laissé un résidu négatif (voir le 2e garde-fou plus bas), on ne
        // veut à AUCUN prix caster une valeur négative en unsigned long
        // (comportement indéfini en C++, cause racine du bug
        // pulsesTotal ~ 2^31 observé sur le terrain).
        if(valveCons[i].carry < 0.0f) valveCons[i].carry = 0.0f;
        unsigned long share = (unsigned long)valveCons[i].carry; // partie entière accumulée
        valveCons[i].carry -= (float)share;
        shares[i] = share;
        assignedSum += share;
    }
    // Garde-fou : assignedSum doit égaler delta exactement. Une dérive de
    // calcul flottant sur le très long terme pourrait introduire un écart
    // de ±1 pulse ; on corrige ici en ajustant la vanne dont le carry est
    // le plus avancé, pour ne jamais perdre ni dupliquer de pulse au global.
    if(assignedSum != delta){
        long diff = (long)delta - (long)assignedSum;
        int best = -1;
        float bestCarry = -1e9f;
        for(int i=0;i<VANNE_COUNT;i++){
            if(!valves[i].isOpen) continue;
            if(valveCons[i].carry > bestCarry){ bestCarry = valveCons[i].carry; best = i; }
        }
        if(best>=0 && diff>0){
            shares[best] += (unsigned long)diff;
            valveCons[best].carry -= (float)diff;
            // FIX carry négatif : `diff` peut dépasser `carry[best]` (qui est
            // toujours < 1.0 par construction, car déjà passé par un floor
            // juste au-dessus). Sans ce clamp, carry devient négatif et LE
            // RESTE (il persiste d'un appel à l'autre par design) jusqu'à
            // provoquer, au tour suivant, le cast UB décrit en tête de
            // fonction. C'est la cause racine confirmée du bug
            // "pulsesTotal explose vers ~2^31".
            if(valveCons[best].carry < 0.0f) valveCons[best].carry = 0.0f;
        } else if(best>=0 && diff<0){
            unsigned long take = (unsigned long)(-diff);
            if(shares[best] >= take) shares[best] -= take;
        }
    }

    for(int i=0;i<VANNE_COUNT;i++){
        if(!valves[i].isOpen) continue;
        // Reset du compteur journalier si on est sur un nouveau jour
        if(today != valveCons[i].todayYmd){
            // Clôture éventuelle du jour précédent dans l'historique
            if(valveCons[i].todayYmd != 0 && valveCons[i].todayPulses > 0){
                DayStat ds;
                ds.ymd = valveCons[i].todayYmd;
                ds.pulses = valveCons[i].todayPulses;
                ds.litres = (float)ds.pulses / PULSES_PER_LITRE;
                valveCons[i].history[valveCons[i].todayIdx % CONS_HISTORY_DAYS] = ds;
                valveCons[i].todayIdx = (uint16_t)((valveCons[i].todayIdx + 1) % CONS_HISTORY_DAYS);
            }
            valveCons[i].todayYmd = today;
            valveCons[i].todayPulses = 0;
        }
        valveCons[i].pulsesTotal += shares[i];
        valveCons[i].todayPulses += shares[i];
    }
    lastDistributedTotal = totalPulsesGlobal;
    // ── Throttling NVS (cf. ConfigManager.h) : on ne flushe PLUS en NVS à
    // chaque tour de loop. On marque juste un flag `dirty` par vanne, et
    // loop() appelle valveConsFlushDirty() toutes les 30s. Sauvegarde
    // totale : ~3 put/vanne/min au lieu de ~600/min — divise l'usure NVS
    // par 200× et élimine la cause du NOT_ENOUGH_SPACE.
    for(int i=0;i<VANNE_COUNT;i++) if(valves[i].isOpen) valveConsMarkDirty(i);
}

// ============================================================
// DÉBIT INSTANTANÉ PAR VANNE (fenêtre glissante)
// ============================================================
// Même méthode que le débit global (cf. FlowMeter.h::flowUpdate /
// flowComputeFromRing) : un anneau (timestamp, pulses cumulés) par vanne,
// et un calcul de débit moyen sur la fenêtre FLOW_WINDOW_MS. Cette
// cohérence de méthode permet de comparer/sommer les débits par vanne
// avec le débit global déjà affiché (doc["flow_lpm"] dans
// buildStatusJson()) sans écart de méthodologie.
//
// Alimenté en continu (vanne ouverte ou non) pour que le débit d'une
// vanne qui vient de se fermer redescende progressivement à 0 dans la
// fenêtre au lieu de chuter instantanément à l'appel suivant.
inline FlowSample valveFlowRing[VANNE_COUNT][FLOW_SAMPLES];
inline uint8_t    valveFlowHead[VANNE_COUNT]  = {0};
inline uint8_t    valveFlowCount[VANNE_COUNT] = {0};

// À appeler ~1×/s depuis loop(), à côté de l'appel existant à flowUpdate().
inline void valveFlowUpdateAll(){
    for(int i=0;i<VANNE_COUNT;i++){
        unsigned long nowMs = millis();
        valveFlowRing[i][valveFlowHead[i]].tMs    = nowMs;
        valveFlowRing[i][valveFlowHead[i]].pulses = valveCons[i].pulsesTotal;
        valveFlowHead[i] = (valveFlowHead[i] + 1) % FLOW_SAMPLES;
        if(valveFlowCount[i] < FLOW_SAMPLES) valveFlowCount[i]++;

        valveCons[i].instantFlowLpm = flowComputeFromRing(
            valveFlowRing[i], valveFlowHead[i], valveFlowCount[i],
            valveCons[i].instantFlowLpm
        );
    }
}

// Remet à zéro le débit et l'historique d'anneau d'une vanne à sa
// fermeture, pour ne pas laisser une dernière valeur affichée "figée".
// À appeler depuis valveHardClose() (ValveManager.h).
inline void valveConsResetInstantFlow(int i){
    valveCons[i].instantFlowLpm = 0.0f;
    valveFlowHead[i]  = 0;
    valveFlowCount[i] = 0;
}

// ============================================================
// FLOWCALIBRATIONMANAGER — logique métier
// ============================================================
//
// Machine à états VIVANT CÔTÉ SERVEUR, indépendamment de toute connexion
// web : une fois lancée, la calibration continue même si l'utilisateur
// ferme l'onglet ou recharge la page. L'UI n'est qu'un observateur qui
// interroge périodiquement /api/calibration/status (ou reçoit l'état via
// WebSocket) — elle ne porte aucun état de la calibration elle-même.
//
// Séquence : pour chaque vanne (dans l'ordre 0..VANNE_COUNT-1), on ouvre
// SEULE cette vanne pendant calibDurationSec secondes, on mesure le delta
// de pulses sur cette fenêtre, on ferme, puis on passe à la vanne
// suivante. À la fin, on calcule flowCoeff = pulses/seconde pour chaque
// vanne et on persiste (voir calibFinish()).
//
// Sécurité : refuse de démarrer si UNE SEULE vanne est déjà ouverte
// (peu importe la source — WEB, programme, forçage manuel, LoRa), pour
// éviter de fausser la mesure ou de couper un arrosage en cours sans
// confirmation explicite de l'utilisateur.

// Récupère le compteur de pulses total courant (persisté + runtime),
// utilisé à plusieurs endroits du fichier — on le factorise ici pour la
// calibration plutôt que de dupliquer noInterrupts()/interrupts().
inline unsigned long calibReadTotalPulses(){
    unsigned long cnt;
    noInterrupts(); cnt = pulseCount; interrupts();
    return persistedPulseCount + cnt;
}

// Démarre une calibration. Retourne false (avec failReason rempli) si les
// conditions de sécurité ne sont pas remplies — notamment si UNE SEULE
// vanne est déjà ouverte, peu importe la source (sécurité max demandée :
// on ne ferme jamais automatiquement une vanne active pour calibrer).
inline bool calibStart(uint16_t durationSec){
    if(calibState.phase == CalibPhase::RUNNING){
        strlcpy(calibState.failReason, "Calibration déjà en cours", sizeof(calibState.failReason));
        return false;
    }
    for(int i=0;i<VANNE_COUNT;i++){
        if(valves[i].isOpen){
            snprintf(calibState.failReason, sizeof(calibState.failReason),
                      "V%d déjà ouverte — fermez tout avant calibration", i);
            return false;
        }
    }
    if(durationSec < 5) durationSec = 5;       // garde-fou : mesure trop courte = bruit
    if(durationSec > 1800) durationSec = 1800; // garde-fou : 30 min max par vanne

    calibState.phase = CalibPhase::RUNNING;
    calibState.durationSec = durationSec;
    calibState.currentValve = 0;
    calibState.phaseStartMs = millis();
    calibState.pulseSnapshotAtStart = calibReadTotalPulses();
    for(int i=0;i<VANNE_COUNT;i++) calibState.resultCoeff[i] = 0.0f;
    calibState.failReason[0] = '\0';

    // Ouvre la première vanne, SEULE, à pleine durée (cappée par la
    // sécurité absolue MAX_VALVE_OPEN_MS via valveHardOpen). On utilise
    // CmdSource::WEB (priorité la plus haute) pour garantir qu'aucune
    // source concurrente ne puisse interrompre la mesure en cours.
    valveHardOpen(0, CmdSource::WEB, durationSec);
    char msg[80];
    snprintf(msg, sizeof(msg), "Calibration débit démarrée (%us/vanne)", durationSec);
    logSys(msg);
    return true;
}

// Annule une calibration en cours. Ferme la vanne actuellement ouverte par
// la calibration et restaure l'état IDLE. Les coefficients déjà mesurés
// pour les vannes précédentes dans cette session NE SONT PAS appliqués
// (on ne persiste qu'à la fin complète, voir calibFinish()) — une
// calibration interrompue n'altère donc jamais les coefficients existants.
inline void calibAbort(){
    if(calibState.phase != CalibPhase::RUNNING) return;
    if(calibState.currentValve >= 0 && calibState.currentValve < VANNE_COUNT){
        valveHardClose(calibState.currentValve);
    }
    calibState.phase = CalibPhase::ABORTED;
    calibState.currentValve = -1;
    logSys("Calibration débit annulée par l'utilisateur");
}

// Finalise une calibration terminée avec succès : calcule flowCoeff
// (pulses/seconde mesurés, valeur relative — pas besoin de normaliser
// puisque pulseDistribute() utilise déjà un ratio coeff_i / somme(coeff))
// pour chaque vanne et persiste en NVS.
inline void calibFinish(){
    char msg[120];
    for(int i=0;i<VANNE_COUNT;i++){
        float pps = calibState.resultCoeff[i]; // déjà en pulses/seconde, voir calibTick()
        if(pps <= 0.0f){
            // Mesure nulle ou négative (vanne sans débit, capteur déconnecté,
            // ou erreur) : on NE remplace PAS le coefficient existant par 0,
            // ce qui exclurait définitivement cette vanne de toute future
            // répartition. On conserve l'ancienne valeur (ou 1.0 par défaut
            // si jamais calibrée) et on le journalise pour que l'utilisateur
            // puisse investiguer (vanne bouchée ? capteur mal câblé ?).
            snprintf(msg, sizeof(msg),
                "Calibration V%d: débit mesuré nul — coefficient conservé (vérifier la vanne)", i+1);
            logSys(msg);
            continue;
        }
        valveCons[i].flowCoeff = pps;
        valveConsSaveFlowCoeff(i);
        snprintf(msg, sizeof(msg), "Calibration V%d: %.3f pulses/s mesurés", i+1, pps);
        logSys(msg);
    }
    calibState.phase = CalibPhase::DONE;
    calibState.currentValve = -1;
    logSys("Calibration débit terminée — coefficients mis à jour");
}

// Avance la machine à états de calibration. Appelée à chaque tour de
// loop(), sans delay() — suit le même style non-bloquant que le reste du
// firmware (valveUpdate(), schedCheck(), etc.).
inline void calibTick(){
    if(calibState.phase != CalibPhase::RUNNING) return;
    int v = calibState.currentValve;
    if(v < 0 || v >= VANNE_COUNT){
        // État incohérent (ne devrait jamais arriver) : on abandonne proprement
        // plutôt que de lire hors limites.
        strlcpy(calibState.failReason, "État de calibration incohérent", sizeof(calibState.failReason));
        calibState.phase = CalibPhase::FAILED;
        valveCloseAll(CmdSource::WEB);
        return;
    }
    unsigned long elapsedMs = millis() - calibState.phaseStartMs;
    if(elapsedMs < (unsigned long)calibState.durationSec * 1000UL) return; // mesure en cours, rien à faire

    // Fenêtre de mesure de cette vanne terminée : calcule le débit mesuré
    // et ferme la vanne.
    unsigned long totalNow = calibReadTotalPulses();
    unsigned long deltaPulses = (totalNow >= calibState.pulseSnapshotAtStart)
                               ? (totalNow - calibState.pulseSnapshotAtStart) : 0;
    float pps = (float)deltaPulses / (float)calibState.durationSec;
    calibState.resultCoeff[v] = pps;
    valveHardClose(v);

    char msg[80];
    snprintf(msg, sizeof(msg), "Calibration V%d terminée: %lu pulses en %us", v+1, deltaPulses, calibState.durationSec);
    logSys(msg);

    int nextV = v + 1;
    if(nextV >= VANNE_COUNT){
        // Toutes les vannes ont été mesurées : finalise.
        calibFinish();
        return;
    }
    // Passe à la vanne suivante : ouvre SEULE, redémarre le chrono de mesure.
    calibState.currentValve = nextV;
    calibState.phaseStartMs = millis();
    calibState.pulseSnapshotAtStart = calibReadTotalPulses();
    valveHardOpen(nextV, CmdSource::WEB, calibState.durationSec);
}

// Construit le JSON d'état de calibration pour /api/calibration/status et
// pour l'inclusion dans buildStatusJson() (affichage live sans recharger
// la page, même après reconnexion WebSocket suite à un reload).
inline String calibStatusJson(){
    StaticJsonDocument<512> doc;
    const char* phaseStr = "idle";
    switch(calibState.phase){
        case CalibPhase::RUNNING: phaseStr = "running"; break;
        case CalibPhase::DONE:    phaseStr = "done";     break;
        case CalibPhase::ABORTED: phaseStr = "aborted";  break;
        case CalibPhase::FAILED:  phaseStr = "failed";   break;
        default:                  phaseStr = "idle";     break;
    }
    doc["phase"] = phaseStr;
    doc["currentValve"] = calibState.currentValve;
    doc["durationSec"] = calibState.durationSec;
    if(calibState.phase == CalibPhase::RUNNING){
        unsigned long elapsedMs = millis() - calibState.phaseStartMs;
        unsigned long remainMs = ((unsigned long)calibState.durationSec*1000UL > elapsedMs)
                                ? ((unsigned long)calibState.durationSec*1000UL - elapsedMs) : 0;
        doc["remainingSec"] = remainMs/1000UL;
        unsigned long totalNow = calibReadTotalPulses();
        unsigned long deltaPulses = (totalNow >= calibState.pulseSnapshotAtStart)
                                   ? (totalNow - calibState.pulseSnapshotAtStart) : 0;
        doc["livePulses"] = deltaPulses;
    }
    if(calibState.failReason[0]) doc["failReason"] = calibState.failReason;
    JsonArray res = doc.createNestedArray("results");
    for(int i=0;i<VANNE_COUNT;i++) res.add(calibState.resultCoeff[i]);
    JsonArray coeffs = doc.createNestedArray("flowCoeffs");
    for(int i=0;i<VANNE_COUNT;i++) coeffs.add(valveCons[i].flowCoeff);
    // Constante de calibration (pulses/litre) : exposée au frontend pour qu'il
    // puisse convertir les flowCoeffs (en pulses/s) en L/min côté UI, sans
    // hardcoder la valeur dans la SPA. Source de vérité = firmware.
    doc["pulsesPerLitre"] = (float)PULSES_PER_LITRE;
    String out; serializeJson(doc, out);
    return out;
}

#endif // IOCAN