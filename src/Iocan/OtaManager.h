#pragma once
#ifdef IOCAN
// ============================================================
// OtaManager.h — OTA (ArduinoOTA)
// ============================================================
// Correspond à la SECTION 14 du fichier d'origine.
//
// Dépend de ValveManager.h (valveCloseAll) : par sécurité, toutes les
// vannes sont fermées au démarrage d'un flash OTA, pour éviter qu'un
// arrosage continue pendant un reflash (et potentiellement après un
// reboot dans un état logiciel transitoire).
// ============================================================

#include "Globals.h"
#include "LoggerManager.h"
#include "ValveManager.h"

inline void otaSetup(){
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.onStart([](){
        valveCloseAll(CmdSource::WEB);
        logSys("OTA démarré — vannes fermées");
    });
    ArduinoOTA.onEnd([](){  logSys("OTA terminé"); });
    ArduinoOTA.onProgress([](unsigned int p, unsigned int t){
        Serial.printf("OTA %u%%\r",(p/(t/100)));
    });
    ArduinoOTA.onError([](ota_error_t e){
        Serial.printf("OTA Error[%u]\n",e);
    });
    ArduinoOTA.begin();
}

#endif // IOCAN
