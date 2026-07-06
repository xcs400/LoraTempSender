#pragma once
#ifdef IOCAN
// ============================================================
// TimeManager.h — NTP + utilitaires calendrier
// ============================================================
// Correspond à la SECTION 6 du fichier d'origine.
// ============================================================

#include "Globals.h"
#include "LoggerManager.h"

inline void timeInit(){
    // Si une chaîne POSIX TZ est disponible, on l'utilise : elle encode
    // les règles DST (heure été/hiver) automatiquement.
    // Exemple France : "CET-1CEST,M3.5.0,M10.5.0/3"
    // configTzTime(posixTZ, server) équivaut à setenv("TZ",tz,1) + tzset()
    // puis configTime(0,0,server).
    if(sysConfig.tzPosix[0] != '\0'){
        configTzTime(sysConfig.tzPosix, sysConfig.ntpServer);
    } else {
        // Fallback : offset fixe sans DST (ancien comportement)
        configTime(sysConfig.tzOffset, 0, sysConfig.ntpServer);
    }
    struct tm ti;
    if(getLocalTime(&ti,5000)){
        if(!timeIsSynced){
            timeIsSynced = true;
            ntpSyncedAtMs = millis();
            // Calcule l'offset UTC réel (DST inclus) de façon portable.
            // tm_gmtoff est une extension GNU absente de la toolchain ESP32 (newlib).
            // On diff l'heure locale et l'heure UTC obtenue via gmtime_r().
            time_t now_utc = time(nullptr);
            struct tm ti_utc;
            gmtime_r(&now_utc, &ti_utc);
            long totalOffsetSec = ((long)ti.tm_hour  - (long)ti_utc.tm_hour)  * 3600L
                                + ((long)ti.tm_min   - (long)ti_utc.tm_min)   * 60L;
            // Correction si la diff dépasse une demi-journée (changement de jour UTC/local)
            if(totalOffsetSec >  43200L) totalOffsetSec -= 86400L;
            if(totalOffsetSec < -43200L) totalOffsetSec += 86400L;
            char tzmsg[120];
            snprintf(tzmsg, sizeof(tzmsg),
                     "NTP synchronise — TZ='%s', offset reel UTC%+ld (%+ld s DST=%s), heure locale %04d-%02d-%02d %02d:%02d:%02d",
                     sysConfig.tzPosix[0] ? sysConfig.tzPosix : "fixe",
                     totalOffsetSec/3600L,
                     totalOffsetSec,
                     ti.tm_isdst > 0 ? "oui" : "non",
                     ti.tm_year+1900, ti.tm_mon+1, ti.tm_mday,
                     ti.tm_hour, ti.tm_min, ti.tm_sec);
            logSys(tzmsg);
        }
    } else if (oledPage == 2) {
        logSys("NTP échec (sera retenté)");
    }
}

// Convertit (année,mois,jour) en day-of-year (0-based)
inline int monthDayToYday(int year, int month, int day){
    const int mdaysNorm[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    bool leap = ((year%4==0) && (year%100!=0 || year%400==0));
    int yday = 0;
    for(int m=1;m<month;m++){
        if(m==2) yday += mdaysNorm[1] + (leap?1:0);
        else yday += mdaysNorm[m-1];
    }
    yday += (day-1);
    return yday;
}

// Retourne true si on est dans la saison active
inline bool inSeason(const Schedule& s){
    struct tm ti;
    if(!getLocalTime(&ti,5)) return true;
    int md   = (ti.tm_mon+1)*100 + ti.tm_mday;
    int mds  = s.seasonStartMonth*100 + s.seasonStartDay;
    int mde  = s.seasonEndMonth  *100 + s.seasonEndDay;
    return md>=mds && md<=mde;
}

#endif // IOCAN
