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
const unsigned long SCHED_CHECK_INTERVAL_MS = 5000UL; // vérifier toutes les 5s pour réduire délai

inline void schedCheck(){
    if(!timeIsSynced) return;
    unsigned long now = millis();
    if(now - lastSchedCheckMs < SCHED_CHECK_INTERVAL_MS) return;
    lastSchedCheckMs = now;

    struct tm ti;
    if(!getLocalTime(&ti,5)) return;
    int curH = ti.tm_hour;
    int curM = ti.tm_min;
    int dow  = (ti.tm_wday + 6) % 7;  // 0=Lun
    int yday = ti.tm_yday;

    for(int v=0;v<VANNE_COUNT;v++){
        for(int p=0;p<MAX_PROGRAMS;p++){
            Schedule& s = valves[v].schedules[p];
            if(!s.active) continue;
            // Trigger if current time has reached scheduled hour:minute.
            // Allow a small window (>= scheduled time and < scheduled time + 65s)
            if(s.hour != curH) continue;
            // compute seconds since start of minute
            int curSec = ti.tm_sec;
            if(s.minute != curM && s.minute != ((curM - (curSec>65?1:0) + 60) % 60)) continue;
            if(s.minute != curM) {
                // If we are slightly past the minute due to check timing, allow trigger
                if(!( (curM == s.minute && curSec >=0) || (curM == (s.minute+1)%60 && curSec < 65) )) continue;
            }

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

            // N'ouvrir que si non commandée par priorité supérieure
            Valve& vv = valves[v];
            if(vv.isOpen && vv.priority < PRIO_AUTO){
                logAdd(v,"Programme ignoré (priorité supérieure active)");
                continue;
            }
            valveHardOpen(v, CmdSource::AUTO, s.durationSec);
        }
    }
}

#endif // IOCAN
