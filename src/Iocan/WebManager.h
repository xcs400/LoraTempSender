#pragma once
#ifdef IOCAN
// ============================================================
// WebManager.h — Routes REST (AsyncWebServer)
// ============================================================
// Correspond à la SECTION 12 du fichier d'origine.
// ============================================================

#include "Globals.h"
#include "LoggerManager.h"
#include "ConfigManager.h"
#include "ValveManager.h"
#include "ValveCons.h"
#include "WsManager.h"
#include "WebContent.h"   // SPA HTML — seul fichier séparé (inchangé)

// Helper: réponse JSON 200
inline void jsonResp(AsyncWebServerRequest* req, const String& body, int code=200){
    AsyncWebServerResponse* r = req->beginResponse(code,"application/json",body);
    r->addHeader("Access-Control-Allow-Origin","*");
    req->send(r);
}

// Construit le JSON config pour /api/config GET
inline String configToJson(){
    StaticJsonDocument<1024> doc;
    doc["ssid"]         = sysConfig.ssid;
    doc["ntpServer"]    = sysConfig.ntpServer;
    doc["tzOffset"]     = sysConfig.tzOffset;
    doc["tzPosix"]      = sysConfig.tzPosix;
    doc["loraFreq"]     = sysConfig.loraFreq;
    doc["loraPower"]    = sysConfig.loraPower;
    doc["nodeId"]       = sysConfig.nodeId;
    doc["irrigMode"]    = sysConfig.irrigMode;
    doc["maxOpenSec"]   = sysConfig.maxOpenSec;
    doc["manualForceSec"]=sysConfig.manualForceSec;
    doc["mqttEnabled"]  = sysConfig.mqttEnabled;
    doc["mqttHost"]     = sysConfig.mqttHost;
    doc["mqttPort"]     = sysConfig.mqttPort;
    doc["mqttUser"]     = sysConfig.mqttUser;
    doc["mqttPass"]     = sysConfig.mqttPass;
    doc["mqttPrefix"]   = sysConfig.mqttPrefix;
    doc["mqttId"]       = sysConfig.mqttId;
    JsonArray names = doc.createNestedArray("valveNames");
    for(int i=0;i<VANNE_COUNT;i++) names.add(valves[i].name);
    String out; serializeJson(doc,out);
    return out;
}

// Construit le JSON des programmes (flat list)
//
// IMPORTANT (correctif bug "V3/V4 invisibles dans les programmes") :
// Avec VANNE_COUNT vannes × MAX_PROGRAMS slots × ~16 champs par programme,
// l'ancien buffer StaticJsonDocument<4096> était sous-dimensionné (besoin
// réel de l'ordre de 8-10 Ko avec VANNE_COUNT=4, et bien plus avec 8 vannes).
// ArduinoJson ne signale pas d'erreur quand le pool mémoire est plein : les
// objets ajoutés après saturation sont silencieusement vides ou tronqués.
// Comme la boucle remplit V1 puis V2 puis V3 puis V4 dans cet ordre, ce sont
// les DERNIÈRES vannes traitées (V3, V4, ...) qui se retrouvaient amputées —
// exactement le symptôme observé : leurs programmes "n'apparaissaient pas".
// On utilise donc un DynamicJsonDocument dimensionné dynamiquement selon
// VANNE_COUNT et MAX_PROGRAMS, avec une marge de sécurité, et on vérifie
// explicitement le résultat de overflowed() pour journaliser le problème
// au lieu de le laisser passer silencieusement si jamais la taille venait
// à manquer de nouveau (ex: noms de programmes plus longs).
inline String schedulesToJson(){
    // ~220 octets par programme (slot JsonObject + 16 champs + chaîne name)
    // est une estimation large pour ArduinoJson v6 sur ESP32 (32-bit).
    const size_t perSchedule = 220;
    const size_t capacity = JSON_ARRAY_SIZE(VANNE_COUNT * MAX_PROGRAMS)
                           + (size_t)VANNE_COUNT * MAX_PROGRAMS * (JSON_OBJECT_SIZE(16) + perSchedule)
                           + 512; // marge fixe (clé "schedules" + alignement)
    DynamicJsonDocument doc(capacity);
    // Champ méta: nombre total de slots et nombre utilisés (actifs ou nommés).
    JsonObject meta = doc.createNestedObject("meta");
    meta["vanneCount"]   = VANNE_COUNT;
    meta["maxPerValve"]  = MAX_PROGRAMS;
    int usedCount = 0;
    for(int v=0;v<VANNE_COUNT;v++){
        for(int p=0;p<MAX_PROGRAMS;p++){
            Schedule& s = valves[v].schedules[p];
            if(s.active || s.name[0] != '\0') usedCount++;
        }
    }
    meta["usedCount"] = usedCount;

    JsonArray arr = doc.createNestedArray("schedules");
    for(int v=0;v<VANNE_COUNT;v++){
        for(int p=0;p<MAX_PROGRAMS;p++){
            Schedule& s = valves[v].schedules[p];
            // Indicateur explicite de validité: un slot est "utilisé" dès
            // qu'il est actif OU qu'il porte un nom (même si désactivé).
            bool used = s.active || (s.name[0] != '\0');
            // On n'exporte que les slots utilisés pour éviter le bruit dans
            // le JSON (et fournir un export "humainement" lisible). Le frontend
            // peut distinguer un slot libre via l'absence d'entrée.
            if(!used) continue;
            JsonObject o = arr.createNestedObject();
            o["valid"]            = true;   // marqueur explicite (lecture/import)
            o["valve"]            = v;
            o["schedIdx"]         = p;
            o["active"]           = s.active;
            o["hour"]             = s.hour;
            o["minute"]           = s.minute;
            o["durationSec"]      = s.durationSec;
            o["name"]             = s.name;
            o["weekDays"]         = s.weekDays;
            o["intervalStartMonth"] = s.intervalStartMonth;
            o["intervalStartDay"]   = s.intervalStartDay;
            o["calMode"]          = s.calMode;
            o["intervalDays"]     = s.intervalDays;
            o["seasonStartMonth"] = s.seasonStartMonth;
            o["seasonStartDay"]   = s.seasonStartDay;
            o["seasonEndMonth"]   = s.seasonEndMonth;
            o["seasonEndDay"]     = s.seasonEndDay;
        }
    }
    if(doc.overflowed()){
        // Ne devrait plus arriver avec le dimensionnement ci-dessus, mais on
        // journalise pour diagnostic immédiat plutôt qu'une troncature muette.
        logSys("ERREUR: buffer JSON programmes insuffisant (overflow)");
    }
    String out; serializeJson(doc,out);
    return out;
}

inline void webSetup(){
    // ── SPA principale
    // On utilise beginResponse_P() pour servir directement depuis PROGMEM,
    // SANS recopier la chaîne (~52 KB) dans une String heap. C'est essentiel
    // car req->send(200,"text/html",WEB_HTML) duplique la page en RAM et
    // peut faire échouer malloc() quand la heap est fragmentée (~138 KB
    // libres observés sur heltec-Iocan-HS3), donnant une page blanche
    // sans erreur côté navigateur (Content-Length incorrect / troncature).
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* req){
        Serial.println("[HTTP] GET /");
        const char* p = (const char*)WEB_HTML;
        const size_t total = strlen_P(p);
        AsyncWebServerResponse* resp = req->beginResponse_P(
            200, "text/html; charset=utf-8",
            (const uint8_t*)p, total);
        resp->addHeader("Cache-Control", "no-store");
        req->send(resp);
    });

    // ── Statut complet GET
    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* req){
        jsonResp(req, buildStatusJson());
    });

    // ── Ouvrir vanne POST /api/valve/open  {valve,duration,source}
    server.on("/api/valve/open", HTTP_POST, [](AsyncWebServerRequest* req){},
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
            StaticJsonDocument<256> doc;
            if(deserializeJson(doc,data,len)){jsonResp(req,"{\"ok\":false}",400);return;}
            int idx  = doc["valve"] | -1;
            uint32_t dur = doc["duration"] | (uint32_t)sysConfig.maxOpenSec;
            if(idx<0||idx>=VANNE_COUNT){jsonResp(req,"{\"ok\":false}",400);return;}
            bool ok = valveHardOpen(idx, CmdSource::WEB, dur);
            jsonResp(req, ok ? "{\"ok\":true}" : "{\"ok\":false,\"reason\":\"priority\"}");
        }
    );

    // ── Fermer vanne POST /api/valve/close  {valve}
    server.on("/api/valve/close", HTTP_POST, [](AsyncWebServerRequest* req){},
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
            StaticJsonDocument<128> doc;
            if(deserializeJson(doc,data,len)){jsonResp(req,"{\"ok\":false}",400);return;}
            int idx = doc["valve"] | -1;
            if(idx<0||idx>=VANNE_COUNT){jsonResp(req,"{\"ok\":false}",400);return;}
            // Fermer impérativement la vanne même si elle était forcée par une source
            valveHardClose(idx);
            logAdd(idx, "Fermée via Web (annule forçage)");
            jsonResp(req,"{\"ok\":true}");
        }
    );

    // ── Forcer vanne POST /api/valve/force  {valve,duration}
    server.on("/api/valve/force", HTTP_POST, [](AsyncWebServerRequest* req){},
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
            StaticJsonDocument<128> doc;
            if(deserializeJson(doc,data,len)){jsonResp(req,"{\"ok\":false}",400);return;}
            int idx  = doc["valve"] | -1;
            uint32_t dur = doc["duration"] | (uint32_t)sysConfig.manualForceSec;
            if(idx<0||idx>=VANNE_COUNT){jsonResp(req,"{\"ok\":false}",400);return;}
            valveHardOpen(idx, CmdSource::WEB, dur);   // WEB = priorité 1
            jsonResp(req,"{\"ok\":true}");
        }
    );

    // ── Compteur d'impulsions (GET /api/pulse)
    server.on("/api/pulse", HTTP_GET, [](AsyncWebServerRequest* req){
        // Lecture atomique
        unsigned long cnt;
        noInterrupts(); cnt = pulseCount; interrupts();
        char buf[128];
        float litres = (float)cnt / PULSES_PER_LITRE;
        snprintf(buf, sizeof(buf), "{\"pulses\":%lu,\"litres\":%.3f,\"pulses_per_litre\":%.1f}", cnt, litres, (double)PULSES_PER_LITRE);
        req->send(200, "application/json", String(buf));
    });

    // ── Reset compteur POST /api/pulse/reset
    server.on("/api/pulse/reset", HTTP_POST, [](AsyncWebServerRequest* req){
        // reset persisted and runtime counters
        noInterrupts(); pulseCount = 0; interrupts();
        persistedPulseCount = 0;
        pulseSave();
        // Reset aussi le suivi par vanne
        for(int v=0;v<VANNE_COUNT;v++){
            valveCons[v].pulsesTotal = 0;
            valveCons[v].todayPulses = 0;
            valveCons[v].todayYmd = todayYMD();
            valveCons[v].todayIdx = 0;
            // CORRECTIF répartition : remettre le carry à zéro aussi. Sans
            // ça, un résidu fractionnaire accumulé avant le reset (donc
            // basé sur d'anciens pulses déjà remis à zéro par ailleurs)
            // fausserait légèrement la toute première distribution après
            // le reset (léger à-coup, pas une perte de bilan global, mais
            // autant repartir propre puisque l'utilisateur attend un vrai
            // zéro partout).
            valveCons[v].carry = 0.0f;
            for(int d=0;d<CONS_HISTORY_DAYS;d++){
                valveCons[v].history[d].ymd = 0;
                valveCons[v].history[d].pulses = 0;
                valveCons[v].history[d].litres = 0;
            }
            valveConsSaveOne(v);
        }
        lastDistributedTotal = 0;
        lastMqttPubMs = 0; // Force la publication immédiate des zéros sur MQTT
        req->send(200, "application/json", String("{\"ok\":true}"));
        logSys("Compteur impulsions + suivi par vanne remis a zero");
    });

    // ── Reset compteur conso d'une vanne POST /api/valve/reset_cons
    server.on("/api/valve/reset_cons", HTTP_POST, [](AsyncWebServerRequest* req){},
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
            StaticJsonDocument<128> doc;
            if(deserializeJson(doc,data,len)){jsonResp(req,"{\"ok\":false}",400);return;}
            int v = doc["valve"] | -1;
            String type = doc["type"] | "all";
            if(v<0||v>=VANNE_COUNT){jsonResp(req,"{\"ok\":false}",400);return;}
            
            if(type == "today") {
                valveCons[v].todayPulses = 0;
                logAdd(v, "Compteur jour remis a zero");
            } else {
                valveCons[v].pulsesTotal = 0;
                valveCons[v].todayPulses = 0;
                valveCons[v].carry = 0.0f;
                for(int d=0;d<CONS_HISTORY_DAYS;d++){
                    valveCons[v].history[d].ymd = 0;
                    valveCons[v].history[d].pulses = 0;
                    valveCons[v].history[d].litres = 0;
                }
                logAdd(v, "Compteur consommation remis a zero");
            }
            valveConsFlushOne(v);
            lastMqttPubMs = 0; // Force MAJ MQTT immédiate
            jsonResp(req,"{\"ok\":true}");
        }
    );

    // ── Consommation par vanne (GET /api/consumption)
    server.on("/api/consumption", HTTP_GET, [](AsyncWebServerRequest* req){
        DynamicJsonDocument doc(2048);
        JsonArray arr = doc.createNestedArray("valves");
        uint16_t today = todayYMD();
        for(int v=0;v<VANNE_COUNT;v++){
            // Si on a changé de jour depuis le dernier check, on capture l'ancien jour dans l'historique
            // (déjà géré côté runtime par pulseDistribute, mais on aligne ici pour l'affichage immédiat)
            float litresTotal = (float)valveCons[v].pulsesTotal / PULSES_PER_LITRE;
            float litresToday = (valveCons[v].todayYmd == today)
                              ? (float)valveCons[v].todayPulses / PULSES_PER_LITRE
                              : 0.0f;
            JsonObject o = arr.createNestedObject();
            o["valve"]            = v;
            o["name"]             = valves[v].name;
            o["pulsesTotal"]      = valveCons[v].pulsesTotal;
            o["litresTotal"]      = litresTotal;
            o["todayYmd"]         = valveCons[v].todayYmd;
            o["pulsesToday"]      = valveCons[v].todayPulses;
            o["litresToday"]      = litresToday;
            // Coefficient de calibration débit actuel (pulses/seconde mesurés
            // seul, ou 1.0 par défaut si jamais calibré) — utile pour que
            // l'UI affiche les coefficients courants même hors calibration.
            o["flowCoeff"]        = valveCons[v].flowCoeff;
            JsonArray hist = o.createNestedArray("history");
            // On parcourt l'historique par ordre chronologique inverse (le plus récent d'abord)
            int start = (valveCons[v].todayIdx - 1 + CONS_HISTORY_DAYS) % CONS_HISTORY_DAYS;
            for(int k=0;k<CONS_HISTORY_DAYS;k++){
                int idx = (start - k + CONS_HISTORY_DAYS) % CONS_HISTORY_DAYS;
                const DayStat& ds = valveCons[v].history[idx];
                if(ds.ymd == 0) continue;
                JsonObject h = hist.createNestedObject();
                h["ymd"] = ds.ymd;
                h["pulses"] = ds.pulses;
                h["litres"] = ds.litres;
            }
        }
        String out; serializeJson(doc, out);
        req->send(200, "application/json", out);
    });

    // ── Calibration débit : démarrer POST /api/calibration/start {durationSec}
    //
    // Sécurité : calibStart() refuse si une vanne est déjà ouverte (peu
    // importe la source) — pas de fermeture automatique, conformément au
    // choix de sécurité maximale. La réponse {ok:false, reason:"..."} permet
    // à l'UI d'afficher le message d'erreur exact à l'utilisateur.
    server.on("/api/calibration/start", HTTP_POST, [](AsyncWebServerRequest* req){},
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
            StaticJsonDocument<128> doc;
            uint16_t durationSec = 60;
            if(len>0 && !deserializeJson(doc,data,len)){
                durationSec = doc["durationSec"] | 60;
            }
            bool ok = calibStart(durationSec);
            if(ok){
                jsonResp(req, "{\"ok\":true}");
            } else {
                String resp = "{\"ok\":false,\"reason\":\"" + String(calibState.failReason) + "\"}";
                jsonResp(req, resp, 409); // 409 Conflict : état actuel incompatible
            }
        }
    );

    // ── Calibration débit : annuler POST /api/calibration/abort
    server.on("/api/calibration/abort", HTTP_POST, [](AsyncWebServerRequest* req){
        calibAbort();
        jsonResp(req, "{\"ok\":true}");
    });

    // ── Calibration débit : statut GET /api/calibration/status
    // Utilisé par l'UI pour suivre la progression en polling (en complément
    // du résumé léger inclus dans le STATUS WebSocket), et pour retrouver
    // l'état exact après un reload de page puisque la calibration vit
    // entièrement côté firmware.
    server.on("/api/calibration/status", HTTP_GET, [](AsyncWebServerRequest* req){
        jsonResp(req, calibStatusJson());
    });

    // ── Fermer tout POST /api/valve/closeall
    server.on("/api/valve/closeall", HTTP_POST, [](AsyncWebServerRequest* req){
        valveCloseAll(CmdSource::WEB);
        jsonResp(req,"{\"ok\":true}");
    });

    // ── Programmes GET
    server.on("/api/schedules", HTTP_GET, [](AsyncWebServerRequest* req){
        jsonResp(req, schedulesToJson());
    });

    // ── Import global des programmes depuis JSON
    server.on("/api/schedules/import", HTTP_POST, [](AsyncWebServerRequest* req){},
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
            DynamicJsonDocument doc(16384);
            DeserializationError err = deserializeJson(doc, data, len);
            if(err){ jsonResp(req, "{\"ok\":false,\"reason\":\"json\"}", 400); return; }

            JsonArray arr;
            if(doc.is<JsonArray>()){
                arr = doc.as<JsonArray>();
            } else if(doc.containsKey("schedules") && doc["schedules"].is<JsonArray>()){
                arr = doc["schedules"].as<JsonArray>();
            } else if(doc.containsKey("programmes") && doc["programmes"].is<JsonArray>()){
                arr = doc["programmes"].as<JsonArray>();
            } else {
                jsonResp(req, "{\"ok\":false,\"reason\":\"format\"}", 400); return;
            }

            for(int v=0;v<VANNE_COUNT;v++){
                for(int p=0;p<MAX_PROGRAMS;p++){
                    valves[v].schedules[p] = Schedule();
                }
            }

            for(JsonVariant item : arr){
                if(!item.is<JsonObject>()) continue;
                int v = item["valve"] | -1;
                int idx = item["schedIdx"] | -1;
                if(v<0 || v>=VANNE_COUNT || idx<0 || idx>=MAX_PROGRAMS) continue;

                Schedule s = Schedule();
                s.active           = item["active"]           | false;
                s.hour             = item["hour"]             | 6;
                s.minute           = item["minute"]           | 0;
                s.durationSec      = item["durationSec"]      | 900;
                s.weekDays         = item["weekDays"]         | 0b0111111;
                s.calMode          = item["calMode"]          | 0;
                s.intervalDays     = item["intervalDays"]     | 2;
                s.intervalStartMonth = item["intervalStartMonth"] | s.intervalStartMonth;
                s.intervalStartDay   = item["intervalStartDay"]   | s.intervalStartDay;
                s.seasonStartMonth = item["seasonStartMonth"] | 4;
                s.seasonStartDay   = item["seasonStartDay"]   | 1;
                s.seasonEndMonth   = item["seasonEndMonth"]   | 10;
                s.seasonEndDay     = item["seasonEndDay"]     | 31;
                if(item.containsKey("name")) strlcpy(s.name, item["name"], sizeof(s.name));
                valves[v].schedules[idx] = s;
            }

            schedSave();
            jsonResp(req, "{\"ok\":true}");
        }
    );

    // ── Sauver programme POST /api/schedule/save
    //
    // IMPORTANT (correctif bug "changement de vanne") :
    // Un programme est identifié de façon stable par (origValve, schedIdx) tel
    // qu'il existait AVANT modification. Si la vanne cible (valve) diffère de
    // origValve, on ne réutilise JAMAIS le même schedIdx sur la nouvelle vanne :
    // on cherche un slot libre dédié sur la vanne de destination, on y copie le
    // programme, puis on libère l'ancien slot. Cela évite d'écraser silencieusement
    // un programme existant sur la vanne de destination et évite que la ligne
    // "saute" à un index arbitraire dans la liste à plat retournée par /api/schedules.
    server.on("/api/schedule/save", HTTP_POST, [](AsyncWebServerRequest* req){},
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
            StaticJsonDocument<512> doc;
            if(deserializeJson(doc,data,len)){jsonResp(req,"{\"ok\":false}",400);return;}
            int v      = doc["valve"]    | -1;
            int idx    = doc["schedIdx"] | -1;
            int origV  = doc["origValve"] | -1;
            bool isMove = (origV>=0 && origV!=v);

            if(v<0||v>=VANNE_COUNT){jsonResp(req,"{\"ok\":false,\"reason\":\"valve\"}",400);return;}

            if(isMove){
                // Déplacement vers une autre vanne : ne JAMAIS réutiliser le même
                // schedIdx tel quel — chercher un slot libre sur la vanne cible.
                int freeIdx = -1;
                for(int p=0;p<MAX_PROGRAMS;p++){
                    if(!valves[v].schedules[p].active){ freeIdx=p; break; }
                }
                if(freeIdx<0){
                    jsonResp(req,"{\"ok\":false,\"reason\":\"full\"}");
                    return;
                }
                idx = freeIdx;
            } else if(idx<0){
                // Nouveau programme sur la même vanne : chercher un slot libre
                for(int p=0;p<MAX_PROGRAMS;p++){
                    if(!valves[v].schedules[p].active){ idx=p; break; }
                }
                if(idx<0){jsonResp(req,"{\"ok\":false,\"reason\":\"full\"}");return;}
            }

            if(idx<0||idx>=MAX_PROGRAMS){jsonResp(req,"{\"ok\":false}",400);return;}

            Schedule& s = valves[v].schedules[idx];
            s.active           = doc["active"]           | true;
            s.hour             = doc["hour"]             | 6;
            s.minute           = doc["minute"]           | 0;
            s.durationSec      = doc["durationSec"]      | 900;
            s.weekDays         = doc["weekDays"]         | 0b0111111;
            s.calMode          = doc["calMode"]          | 0;
            s.intervalDays     = doc["intervalDays"]     | 2;
            s.intervalStartMonth = doc["intervalStartMonth"] | s.intervalStartMonth;
            s.intervalStartDay   = doc["intervalStartDay"]   | s.intervalStartDay;
            s.seasonStartMonth = doc["seasonStartMonth"] | 4;
            s.seasonStartDay   = doc["seasonStartDay"]   | 1;
            s.seasonEndMonth   = doc["seasonEndMonth"]   | 10;
            s.seasonEndDay     = doc["seasonEndDay"]     | 31;
            if(doc.containsKey("name")) strlcpy(s.name, doc["name"], sizeof(s.name));

            // Libérer l'ancien slot UNIQUEMENT après avoir écrit le nouveau avec succès
            if(isMove){
                valves[origV].schedules[ (int)(doc["schedIdx"] | -1) ] = Schedule();
            }

            schedSave();
            // Renvoyer la position finale pour que le frontend puisse resynchroniser
            // immédiatement la sélection en cours d'édition sans devoir deviner.
            String resp = "{\"ok\":true,\"valve\":"+String(v)+",\"schedIdx\":"+String(idx)+"}";
            jsonResp(req,resp);
        }
    );

    // ── Supprimer programme POST /api/schedule/delete {valve,schedIdx}
    server.on("/api/schedule/delete", HTTP_POST, [](AsyncWebServerRequest* req){},
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
            StaticJsonDocument<128> doc;
            if(deserializeJson(doc,data,len)){jsonResp(req,"{\"ok\":false}",400);return;}
            int v=doc["valve"]|-1, p=doc["schedIdx"]|-1;
            if(v<0||v>=VANNE_COUNT||p<0||p>=MAX_PROGRAMS){jsonResp(req,"{\"ok\":false}",400);return;}
            valves[v].schedules[p] = Schedule();  // reset
            schedSave();
            jsonResp(req,"{\"ok\":true}");
        }
    );

    // ── Toggle actif programme POST /api/schedule/toggle {valve,schedIdx,active}
    server.on("/api/schedule/toggle", HTTP_POST, [](AsyncWebServerRequest* req){},
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
            StaticJsonDocument<128> doc;
            if(deserializeJson(doc,data,len)){jsonResp(req,"{\"ok\":false}",400);return;}
            int v=doc["valve"]|-1, p=doc["schedIdx"]|-1;
            bool act=doc["active"]|false;
            if(v<0||v>=VANNE_COUNT||p<0||p>=MAX_PROGRAMS){jsonResp(req,"{\"ok\":false}",400);return;}
            valves[v].schedules[p].active=act;
            schedSave();
            jsonResp(req,"{\"ok\":true}");
        }
    );

    // ── Journal GET /api/log?n=200
    server.on("/api/log", HTTP_GET, [](AsyncWebServerRequest* req){
        uint16_t n=200;
        if(req->hasParam("n")) n=req->getParam("n")->value().toInt();
        jsonResp(req, logToJson(n));
    });

    // ── Config GET
    server.on("/api/config", HTTP_GET, [](AsyncWebServerRequest* req){
        jsonResp(req, configToJson());
    });

    // ── Config POST /api/config
    //
    // Notes de conception :
    //  * Chaque champ n'est mis à jour QUE si la clé est présente et non
    //    nulle côté JSON. Sans ce garde-fou, un payload partiel (rare mais
    //    possible) écraserait silencieusement des champs valides.
    //  * Le mot de passe WiFi n'est mis à jour QUE si non vide (évite
    //    d'effacer un pass existant quand l'UI envoie "" pour "non modifié").
    //  * Quand le SSID ou le mot de passe WiFi changent réellement, on
    //    déclenche un reboot ~1 s après la réponse HTTP. Sans ça, le module
    //    WiFi garde l'ancien SSID en interne jusqu'au prochain reset manuel,
    //    et l'utilisateur voit "rien n'a changé même après reboot" parce que
    //    il n'a pas pensé à rebooter, OU le reboot manuel ne suffit pas si
    //    configSave() a échoué silencieusement (corrigé par les logs).
    server.on("/api/config", HTTP_POST, [](AsyncWebServerRequest* req){},
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
            StaticJsonDocument<1024> doc;
            DeserializationError err = deserializeJson(doc, data, len);
            if(err){
                Serial.printf("[CFG] POST /api/config: JSON parse error: %s (len=%u)\n",
                              err.c_str(), (unsigned)len);
                jsonResp(req, "{\"ok\":false,\"reason\":\"json-parse\"}", 400);
                return;
            }

            // Détecter un vrai changement de SSID/pass AVANT modification
            char prevSsid[32]; strlcpy(prevSsid, sysConfig.ssid, 32);
            char prevPass[64]; strlcpy(prevPass, sysConfig.wifiPass, 64);
            bool wifiChanged = false;

            // --- WiFi : SSID / pass
            const char* newSsid = doc["ssid"] | (const char*)nullptr;
            if(newSsid){
                strlcpy(sysConfig.ssid, newSsid, sizeof(sysConfig.ssid));
                if(strcmp(prevSsid, sysConfig.ssid) != 0) wifiChanged = true;
            }
            const char* newPass = doc["wifiPass"] | (const char*)nullptr;
            if(newPass && strlen(newPass) > 0){
                strlcpy(sysConfig.wifiPass, newPass, sizeof(sysConfig.wifiPass));
                if(strcmp(prevPass, sysConfig.wifiPass) != 0) wifiChanged = true;
            }

            // --- Autres champs système
            const char* v;
            if((v = doc["ntpServer"]   | (const char*)nullptr)) strlcpy(sysConfig.ntpServer, v, sizeof(sysConfig.ntpServer));
            if((v = doc["nodeId"]      | (const char*)nullptr)) strlcpy(sysConfig.nodeId,    v, sizeof(sysConfig.nodeId));
            // Les numériques : ArduinoJson `|` avec la valeur courante
            // préserve la valeur existante si la clé est absente. Pour les
            // champs que l'UI envoie TOUJOURS, on garde ce pattern ; pour
            // les flags qu'on veut pouvoir remettre à 0, on testerait
            // containsKey() à la place.
            sysConfig.tzOffset       = doc["tzOffset"]       | sysConfig.tzOffset;
            if((v = doc["tzPosix"] | (const char*)nullptr)) strlcpy(sysConfig.tzPosix, v, sizeof(sysConfig.tzPosix));
            sysConfig.loraFreq       = doc["loraFreq"]       | sysConfig.loraFreq;
            sysConfig.loraPower      = doc["loraPower"]      | sysConfig.loraPower;
            sysConfig.irrigMode      = doc["irrigMode"]      | sysConfig.irrigMode;
            sysConfig.maxOpenSec     = doc["maxOpenSec"]     | sysConfig.maxOpenSec;
            sysConfig.manualForceSec = doc["manualForceSec"] | sysConfig.manualForceSec;

            // --- MQTT
            if(doc.containsKey("mqttEnabled")) sysConfig.mqttEnabled = doc["mqttEnabled"].as<bool>();
            if((v = doc["mqttHost"]   | (const char*)nullptr)) strlcpy(sysConfig.mqttHost,   v, sizeof(sysConfig.mqttHost));
            sysConfig.mqttPort   = doc["mqttPort"]   | sysConfig.mqttPort;
            if((v = doc["mqttUser"]   | (const char*)nullptr)) strlcpy(sysConfig.mqttUser,   v, sizeof(sysConfig.mqttUser));
            if((v = doc["mqttPass"]   | (const char*)nullptr)) strlcpy(sysConfig.mqttPass,   v, sizeof(sysConfig.mqttPass));
            if((v = doc["mqttPrefix"] | (const char*)nullptr)) strlcpy(sysConfig.mqttPrefix, v, sizeof(sysConfig.mqttPrefix));
            if((v = doc["mqttId"]     | (const char*)nullptr)) strlcpy(sysConfig.mqttId,     v, sizeof(sysConfig.mqttId));

            // --- Noms de vannes
            if(doc.containsKey("valveNames")){
                JsonArray arr = doc["valveNames"];
                for(int i=0; i<VANNE_COUNT && i<(int)arr.size(); i++){
                    const char* nm = arr[i] | (const char*)nullptr;
                    strlcpy(valves[i].name, nm ? nm : "", sizeof(valves[i].name));
                }
            }

            // --- Persistance NVS + log de ce qui a vraiment été écrit
            configSave();
            Serial.printf("[CFG] saved: ssid='%s' nodeId='%s' wifiChanged=%d\n",
                          sysConfig.ssid, sysConfig.nodeId, wifiChanged);

            // --- Redémarrage auto si WiFi a changé
            if(wifiChanged){
                jsonResp(req, "{\"ok\":true,\"restart\":true,\"reason\":\"wifi-changed\"}");
                Serial.println("[CFG] WiFi modifié → redémarrage dans 800 ms");
                static bool restartScheduled = false;
                if(!restartScheduled){
                    restartScheduled = true;
                    xTaskCreate([](void*){
                        vTaskDelay(pdMS_TO_TICKS(800));
                        Serial.println("[CFG] Redémarrage pour appliquer le nouveau WiFi");
                        ESP.restart();
                    }, "cfgReboot", 2048, NULL, 1, NULL);
                }
                return;
            }

            jsonResp(req, "{\"ok\":true}");
        }
    );

    // ── Scan WiFi GET /api/wifi/scan
    //
    // Endpoint utilisé par la page de configuration principale (device
    // connecté en STA à un réseau) pour afficher la liste des réseaux
    // visibles — même comportement que le /scan du portail captif, mais en
    // JSON pour pouvoir afficher SSID + RSSI + type de chiffrement.
    //
    // Machine à états :
    //   * 1er appel  → déclenche un scan async (scanNetworks(true)),
    //                  répond {ok:true, running:true} — le navigateur
    //                  attend ~1.5 s puis réinterroge
    //   * appels suivants pendant que le scan tourne → {running:true}
    //   * scan terminé  → retourne la liste, puis relance un scan en
    //                    arrière-plan pour le prochain refresh (idem
    //                    captiveServer.on("/scan") pour rester cohérent)
    //   * mode STA absent (portail captif actif sans AP_STA) → le scan
    //                  échouera silencieusement côté driver ; on renvoie
    //                  {ok:false, reason:"captive"} pour que l'UI affiche
    //                  un message clair au lieu d'une liste vide.
    server.on("/api/wifi/scan", HTTP_GET, [](AsyncWebServerRequest* req){
        // En mode portail captif, on NE lance PAS de scan ici — le captive
        // portail a déjà sa propre page /scan optimisée et utiliser le
        // scan STA pendant le portail peut perturber l'AP (cf. startCaptivePortal).
        if(captivePortalActive){
            jsonResp(req, "{\"ok\":false,\"running\":false,\"reason\":\"captive\"}");
            return;
        }

        int n = WiFi.scanComplete();
        if(n == WIFI_SCAN_RUNNING){
            jsonResp(req, "{\"ok\":true,\"running\":true}");
            return;
        }
        if(n == WIFI_SCAN_FAILED){
            // Relance un scan et indique running
            WiFi.scanNetworks(true);
            jsonResp(req, "{\"ok\":true,\"running\":true}");
            return;
        }
        if(n == 0){
            // Aucun réseau trouvé au dernier scan — on en relance un pour
            // le prochain refresh (cohérent avec captiveServer.on("/scan"))
            WiFi.scanDelete();
            WiFi.scanNetworks(true);
            jsonResp(req, "{\"ok\":true,\"running\":false,\"networks\":[]}");
            return;
        }
        if(n > 0){
            // Construit la liste triée par RSSI décroissant (plus fort
            // en premier) — plus convivial que l'ordre brut du driver.
            // Petit algo de tri par insertion (n reste petit, < 30 réseaux
            // visibles en pratique) pour éviter un std::sort non disponible.
            int order[40];
            for(int i=0;i<n;i++) order[i]=i;
            for(int i=1;i<n;i++){
                int key = order[i];
                int32_t keyRssi = WiFi.RSSI(key);
                int j = i-1;
                while(j>=0 && WiFi.RSSI(order[j]) < keyRssi){
                    order[j+1] = order[j];
                    j--;
                }
                order[j+1] = key;
            }

            // Heap malloc pour rester safe sur heap fragmentée
            // (sizeof(JSON_OBJECT_SIZE(4)) ≈ 32 B × 4 ≈ 128 B par objet ;
            // 32 réseaux max → 4 KB, largement OK sur ESP32).
            StaticJsonDocument<4096> doc;
            doc["ok"] = true;
            doc["running"] = false;
            JsonArray arr = doc.createNestedArray("networks");
            for(int k=0;k<n;k++){
                int i = order[k];
                JsonObject o = arr.createNestedObject();
                String ssid = WiFi.SSID(i);
                // SSID vide = réseau masqué — on le saute pour l'UI (pas
                // sélectionnable) tout en gardant le comptage cohérent.
                if(ssid.length()==0) continue;
                o["ssid"]    = ssid;
                o["rssi"]    = WiFi.RSSI(i);
                o["channel"] = WiFi.channel(i);
                // encryptionType() renvoie un enum wifi_auth_mode_t (ESP32) :
                //   WIFI_AUTH_OPEN=0, WIFI_AUTH_WEP=1, WIFI_AUTH_WPA_PSK=2,
                //   WPA2_PSK=3, WPA_WPA2_PSK=4, WPA2_ENTERPRISE=5, WPA3_PSK=6...
                // On le transforme en libellé court pour l'UI.
                uint8_t enc = WiFi.encryptionType(i);
                const char* encLabel = "WPA";
                switch(enc){
                    case WIFI_AUTH_OPEN:            encLabel = "Ouvert";   break;
                    case WIFI_AUTH_WEP:             encLabel = "WEP";     break;
                    case WIFI_AUTH_WPA_PSK:         encLabel = "WPA";     break;
                    case WIFI_AUTH_WPA2_PSK:        encLabel = "WPA2";    break;
                    case WIFI_AUTH_WPA_WPA2_PSK:    encLabel = "WPA/WPA2";break;
                    case WIFI_AUTH_WPA2_ENTERPRISE: encLabel = "WPA2-Ent";break;
                    case WIFI_AUTH_WPA3_PSK:        encLabel = "WPA3";    break;
                    #ifdef WIFI_AUTH_WPA2_WPA3_PSK
                    case WIFI_AUTH_WPA2_WPA3_PSK:   encLabel = "WPA2/3";  break;
                    #endif
                    default:                        encLabel = "?";       break;
                }
                o["enc"]      = encLabel;
            }
            // Relance un scan pour le prochain refresh (cf. captiveServer)
            WiFi.scanDelete();
            WiFi.scanNetworks(true);
            String out;
            serializeJson(doc, out);
            jsonResp(req, out);
            return;
        }
        // Cas inattendu (ni RUNNING, ni FAILED, ni 0, ni >0) — on déclenche
        // un scan frais et on répond running.
        WiFi.scanNetworks(true);
        jsonResp(req, "{\"ok\":true,\"running\":true}");
    });

    // ── Reset POST /api/reset
    server.on("/api/reset", HTTP_POST, [](AsyncWebServerRequest* req){
        jsonResp(req,"{\"ok\":true}");
        safeRestart("Redémarrage demandé via Web");
    });

    // ── Compat legacy /reset GET
    server.on("/reset", HTTP_GET, [](AsyncWebServerRequest* req){
        req->send(200,"text/plain","Redémarrage...");
        safeRestart("Redémarrage legacy /reset");
    });

    // ── État NVS (utilisé / libre / total) GET /api/nvs/status
    //
    // Sert à la page Configuration pour afficher le niveau de remplissage
    // de la partition NVS (barre de progression). L'état est rafraîchi
    // périodiquement par la loop() (toutes les 5s) et mis en cache ; cette
    // route renvoie directement le cache pour rester réactive.
    server.on("/api/nvs/status", HTTP_GET, [](AsyncWebServerRequest* req){
        jsonResp(req, nvsStatsToJson());
    });

    // ── Formatage NVS POST /api/format
    //
    // Efface TOUTE la partition NVS (config, conso par vanne, programmes,
    // calibration, journal) puis réécrit immédiatement ssid + wifiPass +
    // nodeId pour préserver la connexion WiFi, et reboote. Le firmware
    // redémarrera sur des défauts sains pour tous les autres champs
    // (NTP, LoRa, MQTT, etc.) et l'utilisateur pourra les re-régler via
    // l'UI.
    //
    // Sécurité : on NE bloque PAS l'appel côté serveur (pas de mot de
    // passe séparé), mais l'UI affichera une confirmation explicite à
    // 2 temps (l'utilisateur doit taper "FORMAT" dans un prompt) pour
    // éviter les fausses manipulations.
    server.on("/api/format", HTTP_POST, [](AsyncWebServerRequest* req){
        // Répondre OK avant le reboot (sinon le client n'a pas le temps
        // de recevoir la réponse HTTP).
        jsonResp(req, "{\"ok\":true,\"restart\":true,\"reason\":\"nvs-format\"}");
        // Petit délai pour laisser passer la réponse HTTP, puis on formate
        // et on reboote. La fonction nvsFormatAndRestore() :
        //   1) ferme toutes les vannes
        //   2) sauvegarde ssid/wifiPass en RAM
        //   3) appelle nvs_flash_erase()
        //   4) appelle nvs_flash_init()
        //   5) restaure ssid/wifiPass avec configSave()
        //   6) reboot
        xTaskCreate([](void*){
            vTaskDelay(pdMS_TO_TICKS(400));
            nvsFormatAndRestore();
        }, "nvsFormat", 4096, NULL, 1, NULL);
    });

    // Catch-all : trace toute requête non routée pour faciliter le diag
    server.onNotFound([](AsyncWebServerRequest* req){
        Serial.printf("[HTTP] 404 %s %s\n",
            req->methodToString(), req->url().c_str());
        req->send(404, "text/plain", "not found");
    });

    // WebSocket
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    server.begin();
    logSys("Serveur HTTP démarré");
}

#endif // IOCAN
