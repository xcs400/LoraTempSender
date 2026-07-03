#pragma once
#ifdef IOCAN
// ============================================================
// LoRaManager.h — Trames STATUS / CMD / TIME_SYNC
// ============================================================
// Correspond à la SECTION 10 du fichier d'origine.
// ============================================================

#include "Globals.h"
#include "LoggerManager.h"
#include "ValveManager.h"

// NOTE : loraSetFlag() est déclarée IRAM_ATTR (interruption matérielle
// DIO1 du module LoRa). Comme pulse_isr() dans Globals.cpp, ce type de
// fonction ne peut PAS être `inline`/header-only sur Xtensa : le linker
// place la table de littéraux (l32r) relative à l'IRAM, et une fonction
// dupliquée par inlining dans plusieurs unités de traduction casse cette
// relocalisation ("dangerous relocation: l32r: literal placed after use").
// Elle est donc définie une seule fois dans Globals.cpp et seulement
// déclarée ici (voir Globals.h).

// Construit la trame STATUS JSON
inline String loraBuildStatus(){
    StaticJsonDocument<1024> doc;
    doc["model"]   = "IRRIGATION";
    doc["id"]   = sysConfig.nodeId;
    doc["tempLocal"] = temperature1;      // température locale (DS18B20)
    doc["uptime"] = millis()/1000;
  //  doc["rssi"]   = loraRssi;
    doc["alarm"]  = 0;
    JsonArray vannesArr   = doc.createNestedArray("vannes");
    JsonArray remArr      = doc.createNestedArray("remaining");
    JsonArray srcArr      = doc.createNestedArray("source");
    for(int i=0;i<VANNE_COUNT;i++){
        vannesArr.add(valves[i].isOpen ? 1 : 0);
        remArr.add(valves[i].remainingSec);
        srcArr.add(srcStr(valves[i].source));
    }
    String out; serializeJson(doc,out);
    return out;
}

// Traitement commande LoRa reçue
inline void loraProcessCmd(JsonDocument& doc){
    const char* cmd = doc["cmd"];
    if(!cmd) return;

    if(strcmp(cmd,"OPEN")==0){
        int valve = doc["valve"] | -1;
        uint32_t dur = doc["duration"] | (uint32_t)sysConfig.maxOpenSec;
        if(valve>=0 && valve<VANNE_COUNT)
            valveHardOpen(valve, CmdSource::LORA, dur);
    }
    else if(strcmp(cmd,"CLOSE")==0){
        int valve = doc["valve"] | -1;
        if(valve>=0 && valve<VANNE_COUNT)
            valveClose(valve, CmdSource::LORA);
        else if(valve==-1)
            valveCloseAll(CmdSource::LORA);
    }
    else if(strcmp(cmd,"FORCE")==0){
        int valve = doc["valve"] | -1;
        uint32_t dur = doc["duration"] | (uint32_t)sysConfig.manualForceSec;
        if(valve>=0 && valve<VANNE_COUNT)
            valveHardOpen(valve, CmdSource::LORA, dur);
    }
    else if(strcmp(cmd,"GET_STATUS")==0){
        // Répondre immédiatement
        String msg = loraBuildStatus();
        Serial.print("[LoRa TX CMD->GET_STATUS] len=");
        Serial.print(msg.length());
        Serial.print(" payload=");
        Serial.println(msg);
        int st = radio.startTransmit(msg);
        Serial.printf("[LoRa TX] startTransmit st=%d (0=OK)\n", st);
        if(st != RADIOLIB_ERR_NONE) {
            Serial.printf("[LoRa TX] ECHEC code=%d\n", st);
            logSys(("LoRa TX err: " + String(st)).c_str());
            int st2 = radio.startReceive();
            Serial.printf("[LoRa TX] startReceive apres echec st=%d\n", st2);
        } else {
            loraMode = 1;   // ISR sait qu'on attend TX done
        }
    }
    else if(strcmp(cmd,"CLOSE_ALL")==0){
        valveCloseAll(CmdSource::LORA);
    }
}

// Réception LoRa (non-bloquant)
inline void loraRxProcess(){
    // Si on est en cours de TX, l'ISR a levé loraTxFlag — on traite ça
    // séparément pour ne PAS interpréter la fin de TX comme un paquet RX.
    if(loraTxFlag){
        loraTxFlag = false;
        loraMode = 0;
        loraTxCount++;
        int st = radio.finishTransmit();
        Serial.printf("[LoRa TX] finishTransmit st=%d (0=OK), txCount=%d\n",
                      st, loraTxCount);
        if(st != RADIOLIB_ERR_NONE){
            logSys(("LoRa finishTransmit err: " + String(st)).c_str());
        }
        // Reprendre RX immédiatement après TX
        int st2 = radio.startReceive();
        Serial.printf("[LoRa TX] startReceive apres TX st=%d\n", st2);
        if(st2 != RADIOLIB_ERR_NONE){
            logSys(("LoRa RX restart err: " + String(st2)).c_str());
        }
        return;
    }
    if(!loraRxFlag) return;
    loraRxFlag = false;
    String msg;
    int st = radio.readData(msg);
    if(st == RADIOLIB_ERR_NONE && radio.getPacketLength()>0){
        loraRssi = radio.getRSSI();
        loraRxCount++;
        Serial.print("[LoRa RX] "); Serial.println(msg);
        logSys(("LoRa reçu: "+msg.substring(0,40)).c_str());

        StaticJsonDocument<512> doc;
        if(deserializeJson(doc,msg) == DeserializationError::Ok){
            const char* id = doc["id"];
            if (id && strcmp(id, "Yaourt1") == 0) {
                temperatureRemote = doc["TempCelsius"] | temperatureRemote;
            }

            const char* type = doc["type"];
            if(type){
                if(strcmp(type,"CMD")==0)       loraProcessCmd(doc);
                else if(strcmp(type,"TIME_SYNC")==0){
                    long epoch = doc["epoch"] | 0L;
                    if(epoch>0){
                        struct timeval tv = {epoch,0};
                        settimeofday(&tv,nullptr);
                        if(!timeIsSynced){
                            timeIsSynced = true;
                            ntpSyncedAtMs = millis();
                            logSys("Heure synchronisée via LoRa (premiere fois)");
                        } else {
                            logSys("Heure synchronisée via LoRa");
                        }
                    }
                }
            }
        }
    } else if(st != RADIOLIB_ERR_RX_TIMEOUT){
        Serial.printf("[LoRa] Err lecture %d\n",st);
    }
    // Reprendre la réception
    radio.startReceive();
}

// Envoi STATUS périodique
inline void loraTxUpdate(){
    unsigned long now = millis();
    if(now - lastLoraTx < LORA_TX_INTERVAL_MS) return;
    lastLoraTx = now;
    String msg = loraBuildStatus();
    Serial.print("[LoRa TX STATUS] len=");
    Serial.print(msg.length());
    Serial.print(" payload=");
    Serial.println(msg);
    int st = radio.startTransmit(msg);
    Serial.printf("[LoRa TX] startTransmit st=%d (0=OK)\n", st);
    if(st != RADIOLIB_ERR_NONE) {
        Serial.printf("[LoRa TX] ECHEC code=%d - on retente le RX\n", st);
        logSys(("LoRa TX STATUS err: " + String(st)).c_str());
        // Echec : on remet en RX
        int st2 = radio.startReceive();
        Serial.printf("[LoRa TX] startReceive apres echec st=%d\n", st2);
    } else {
        loraMode = 1;  // ISR sait qu'on attend TX done
    }
}

#endif // IOCAN