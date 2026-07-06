#pragma once
#ifdef IOCAN
// ============================================================
// ScheduleManager.h — Programmes + calendrier avancé
// ============================================================
// Correspond à la SECTION 8 du fichier d'origine.
// ============================================================

#include "Globals.h"
#include "LoggerManager.h"
#include "TimeManager.h"
#include "ValveManager.h"

// Vérifie si un programme doit se déclencher maintenant
// Appelé périodiquement dans loop (réduit pour précision)
inline unsigned long lastSchedCheckMs = 0;
const unsigned long SCHED_CHECK_INTERVAL_MS = 1000UL; // vérifier toutes les 1s pour la précision de déclenchement

// Anti-redéclenchement : pour chaque créneau (valve, schedIdx, jour-yyyy-mm-dd),
// on retient le nombre de secondes de l'horodatage système du dernier déclenchement.
// Tant qu'on n'est pas passé au jour suivant (YYYYMMDD change), on ne re-déclenche pas.
// Stockage compact : 5 vannes × 10 programmes × 4 octets (epoch du dernier tir) = 200 octets.
struct SchedLastFire { uint32_t epoch = 0; uint32_t ymd = 0; };
inline SchedLastFire schedLastFire[VANNE_COUNT][MAX_PROGRAMS];

// Date du jour au format YYYYMMDD pour l'anti-redéclenchement
static inline uint32_t schedYMD(const struct tm& ti){
    return (uint32_t)(
        (ti.tm_year+1900)*10000 +
        (ti.tm_mon+1)*100 +
        ti.tm_mday);
}

inline void schedCheck(){
    if(!timeIsSynced) return;
    unsigned long now = millis();
    if(now - lastSchedCheckMs < SCHED_CHECK_INTERVAL_MS) return;
    lastSchedCheckMs = now;

    struct tm ti;
    if(!getLocalTime(&ti,5)) return;
    int curH = ti.tm_hour;
    int curM = ti.tm_min;
    int curSec = ti.tm_sec;
    int dow  = (ti.tm_wday + 6) % 7;  // 0=Lun ... 6=Dim
    int yday = ti.tm_yday;
    uint32_t today = schedYMD(ti);

    // Fenêtre de déclenchement : on calcule l'écart en secondes entre
    // l'horodatage courant et l'heure programmée (HH:MM:00).
    // WIN_SEC = 65 : on accepte le tir si on est entre 0 et 64 s après
    // l'heure prévue, pour absorber un cycle de loop retardé par une
    // tâche lourde (WiFi, NVS, ...).
    //
    // NOTE : l'ancienne implémentation comparait curSec < WIN_SEC sur la
    // minute suivante, mais curSec ∈ [0..59] donc curSec < 65 est TOUJOURS
    // vrai → fenêtre effective = 2 minutes pleines, pas 65 s. Corrigé.
    const int WIN_SEC = 65;

    for(int v=0;v<VANNE_COUNT;v++){
        for(int p=0;p<MAX_PROGRAMS;p++){
            Schedule& s = valves[v].schedules[p];
            if(!s.active) continue;
            // Secondes écoulées depuis minuit (courant et programmé)
            int curTotalSec   = curH * 3600 + curM  * 60 + curSec;
            int schedTotalSec = (int)s.hour * 3600 + (int)s.minute * 60;
            int elapsed = curTotalSec - schedTotalSec;
            // Accepter si on est dans [0, WIN_SEC) après l'heure programmée
            bool inWindow = (elapsed >= 0 && elapsed < WIN_SEC);
            if(!inWindow) continue;

            // Anti-redéclenchement : si on a déjà tiré ce programme aujourd'hui, on saute.
            SchedLastFire& lf = schedLastFire[v][p];
            if(lf.ymd == today && lf.epoch != 0) continue;

            // Vérification de la condition calendaire
            bool trigger = false;
            switch(s.calMode){
                case 0: // hebdomadaire
                    trigger = !!(s.weekDays & (1<<dow));
                    break;
                case 1: // intervalle
                    if(s.intervalDays>0){
                        int startY = monthDayToYday(ti.tm_year+1900, s.intervalStartMonth, s.intervalStartDay);
                        int daysInYear = 365 + (( (ti.tm_year+1900)%4==0 && ((ti.tm_year+1900)%100!=0 || (ti.tm_year+1900)%400==0))?1:0);
                        int diff = (yday - startY + daysInYear) % s.intervalDays;
                        trigger = (diff == 0);
                    } else trigger = false;
                    break;
                case 2: // saison
                    trigger = inSeason(s);
                    break;
            }
            if(!trigger) continue;

            // N'ouvrir que si non commandée par priorité strictement supérieure.
            // (PRIO_AUTO = 4, plus haute que WEB/INPUT/LORA — un programme peut
            //  toujours ré-armer une vanne qu'il a lui-même ouverte, sinon
            //  une seule exécution par jour était garantie, ce qui n'est pas
            //  le comportement souhaité pour des programmes à créneaux courts.)
            Valve& vv = valves[v];
            if(vv.isOpen && vv.priority < PRIO_AUTO){
                logAdd(v,"Programme ignoré (priorité supérieure active)");
                continue;
            }

            // Mémorise le tir AVANT d'ouvrir pour qu'un refus de valveHardOpen
            // (calibration en cours, par ex.) ne boucle pas.
            time_t nowEpoch = time(nullptr);
            lf.epoch = (uint32_t)nowEpoch;
            lf.ymd   = today;

            char dbg[80];
            snprintf(dbg, sizeof(dbg), "Tir prog v%d[%d] %02d:%02d (dow=%d, yday=%d)",
                     v, p, s.hour, s.minute, dow, yday);
            logAdd(v, dbg);
            valveHardOpen(v, CmdSource::AUTO, s.durationSec);
        }
    }
}

#endif // IOCAN
