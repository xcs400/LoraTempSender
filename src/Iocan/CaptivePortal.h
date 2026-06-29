#pragma once
#ifdef IOCAN
// ============================================================
// CaptivePortal.h — Portail captif de configuration WiFi
// ============================================================
// Correspond à la SECTION 14b du fichier d'origine.
// ============================================================

#include "Globals.h"
#include "LoggerManager.h"

static const char CAPTIVE_HTML_1[] PROGMEM = R"CPEOF(
<!DOCTYPE html><html lang="fr"><head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>IrrigPro — Config WiFi</title>
<style>
  body{background:#0d1117;color:#e6edf3;font-family:system-ui,sans-serif;
       display:flex;align-items:center;justify-content:center;min-height:100vh;margin:0}
  .box{background:#161b22;border:1px solid #30363d;border-radius:12px;
       padding:32px 28px;width:100%;max-width:360px}
  h2{margin:0 0 20px;font-size:1.1rem;color:#388bfd;text-align:center}
  label{font-size:.82rem;color:#8b949e;display:block;margin-bottom:4px}
  input{width:100%;box-sizing:border-box;background:#21262d;border:1px solid #30363d;
        border-radius:6px;color:#e6edf3;padding:9px 12px;font-size:.95rem;margin-bottom:14px}
  button{width:100%;background:#2ea043;color:#fff;border:none;border-radius:6px;
         padding:11px;font-size:1rem;cursor:pointer;font-weight:600}
  button:hover{filter:brightness(1.15)}
  .note{font-size:.75rem;color:#8b949e;margin-top:14px;text-align:center}
  .logo{text-align:center;font-size:1.4rem;font-weight:700;margin-bottom:6px}
  .sub{text-align:center;font-size:.8rem;color:#8b949e;margin-bottom:20px}
</style>
</head><body>
<div class="box">
  <div class="logo">&#x1F6BF; IrrigPro</div>
  <div class="sub">Configuration WiFi</div>
  <h2>Connexion au r&eacute;seau</h2>
  <form method="POST" action="/save">
    <label>Nom du r&eacute;seau (SSID) <a href="#" onclick="fetch('/scan').then(r=>r.text()).then(t=>document.getElementById('ssid_list').innerHTML='<option value=\'\'>R&eacute;seaux d&eacute;tect&eacute;s...</option>'+t); return false;" style="float:right;color:#388bfd;text-decoration:none">&#x1F50D; Actualiser</a></label>
    <select id="ssid_list" onchange="document.getElementsByName('ssid')[0].value = this.value" style="width:100%;box-sizing:border-box;background:#21262d;border:1px solid #30363d;border-radius:6px;color:#e6edf3;padding:9px 12px;font-size:.95rem;margin-bottom:8px">
      <option value="">(S&eacute;lectionnez un r&eacute;seau scann&eacute;)</option>
)CPEOF";

static const char CAPTIVE_HTML_2[] PROGMEM = R"CPEOF(
    </select>
    <input type="text" name="ssid" placeholder="Ou saisissez un r&eacute;seau masqu&eacute;" required/>
    <label>Mot de passe</label>
    <input type="password" name="pass" placeholder="••••••••"/>
    <button type="submit">Enregistrer &amp; Red&eacute;marrer</button>
  </form>
    <div class="note">L'appareil red&eacute;marrera et se connectera au r&eacute;seau choisi.</div>
    <div id="timeout" class="note" style="margin-top:10px">Portail actif — retour en mode irrigation dans 60s</div>
</div>
<script>
    (function(){
        var t = 60;
        var el = document.getElementById('timeout');
        function updateText(s){ el.textContent = 'Portail actif — retour en mode irrigation dans ' + s + 's'; }
        var iv = setInterval(function(){
            if(t<=0){
                clearInterval(iv);
                el.textContent = 'Fin du portail — retour en cours...';
                setTimeout(function(){ location.reload(true); }, 1000);
                return;
            }
            updateText(t);
            t--;
        }, 1000);
        updateText(t);

        // Auto-refresh the scan list after 3 seconds
        setTimeout(function(){
            fetch('/scan').then(r=>r.text()).then(t=>{
                if(t && t.length>0 && t.indexOf('Recherche')===-1 && t.indexOf('Erreur')===-1) {
                    document.getElementById('ssid_list').innerHTML='<option value=\'\'>R&eacute;seaux d&eacute;tect&eacute;s...</option>'+t;
                }
            });
        }, 3000);
    })();
</script>
</body></html>
)CPEOF";

inline void startCaptivePortal() {
    captivePortalActive = true;

    // CORRECTIF v2 (bug "AP n'apparaît plus" persistant malgré le cycle
    // WIFI_OFF -> WIFI_AP_STA -> softAP() de la première tentative) :
    //
    // Le retry précédent partait de l'hypothèse "driver pas encore stabilisé
    // après l'échec STA, il faut juste attendre plus longtemps". Mais le
    // même échec ("set AP config failed") se reproduisait de façon
    // strictement identique sur les deux tentatives malgré le reset complet
    // entre les deux — ce qui élimine l'hypothèse d'un simple problème de
    // timing. La cause la plus probable, documentée sur ESP32-S3 (fragilité
    // du driver RF en mode mixte AP_STA après un échec de connexion STA,
    // contrairement à l'ESP32 classique où le même code fonctionne) :
    // démarrer DIRECTEMENT en WIFI_AP_STA est ce qui échoue, pas le délai
    // avant.
    //
    // Fix v2 : on démarre l'AP en mode WIFI_AP PUR (sans STA mélangé), on
    // vérifie que softAPIP() renvoie bien une adresse non-nulle (softAP()
    // peut renvoyer "succès" sans configuration IP correcte selon certains
    // retours terrain), et on ne bascule en WIFI_AP_STA (pour permettre le
    // scan WiFi depuis le portail) QU'APRÈS avoir confirmé que l'AP pur
    // fonctionne. Si même le mode AP pur échoue, le scan est simplement
    // désactivé pour cette session (le portail reste utilisable pour saisir
    // un SSID manuellement, voir le champ texte du formulaire).
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(300);

    IPAddress apIP(192, 168, 4, 1);

    bool apOk = false;
    bool scanAvailable = false;
    for(int attempt=0; attempt<2 && !apOk; attempt++){
        WiFi.mode(WIFI_AP);          // AP PUR — pas de mode mixte sur cette tentative
        delay(300);
        WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
        bool callOk = WiFi.softAP("IrrigPro-Setup");
        delay(300); // laisser le temps à l'event AP_START de se propager avant de lire l'IP
        IPAddress gotIP = WiFi.softAPIP();
        apOk = callOk && (gotIP != IPAddress(0,0,0,0));
        if(!apOk){
            Serial.printf("[CaptivePortal] Tentative %d échouée (callOk=%d, ip=%s)\n",
                          attempt+1, callOk, gotIP.toString().c_str());
            WiFi.mode(WIFI_OFF);
            delay(500);
        }
    }

    if(apOk){
        Serial.println("[CaptivePortal] AP pur (WIFI_AP) démarré avec succès");
        // L'AP fonctionne en mode pur : on tente maintenant la bascule vers
        // AP_STA pour permettre le scan WiFi depuis le portail. Si CETTE
        // transition spécifique re-casse l'AP (comportement observé sur
        // certains S3), on revient immédiatement en AP pur plutôt que de
        // laisser le portail dans un état cassé.
        WiFi.mode(WIFI_AP_STA);
        delay(300);
        IPAddress checkIP = WiFi.softAPIP();
        if(checkIP != IPAddress(0,0,0,0)){
            scanAvailable = true;
            Serial.println("[CaptivePortal] Bascule AP_STA OK — scan WiFi disponible");
        } else {
            Serial.println("[CaptivePortal] Bascule AP_STA a cassé l'AP — retour en WIFI_AP pur (sans scan)");
            WiFi.mode(WIFI_AP);
            delay(300);
            WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
            WiFi.softAP("IrrigPro-Setup");
            delay(300);
            scanAvailable = false;
        }
    } else {
        Serial.println("[CaptivePortal] ERREUR: AP pur a échoué après 2 tentatives — portail inaccessible");
        logSys("ERREUR: démarrage AP portail captif a échoué (2 tentatives, AP pur)");
    }

    // Lancer un scan asynchrone des réseaux WiFi en arrière-plan, UNIQUEMENT
    // si la bascule AP_STA a réussi (sinon WiFi.scanNetworks() exigerait
    // le mode STA actif, absent en WIFI_AP pur, et échouerait silencieusement
    // ou pire, redéclencherait l'instabilité qu'on vient d'éviter).
    captivePortalScanAvailable = scanAvailable;
    if(scanAvailable){
        Serial.println("[CaptivePortal] Lancement du scan WiFi...");
        WiFi.scanNetworks(true);
    } else {
        Serial.println("[CaptivePortal] Scan WiFi désactivé pour cette session (AP_STA indisponible)");
    }

    // DNS : renvoyer toutes les requêtes vers l'IP de l'AP (portail captif)
    dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    dnsServer.start(53, "*", apIP);

    Serial.print("[CaptivePortal] AP: IrrigPro-Setup  IP: "); Serial.println(WiFi.softAPIP().toString());
    logSys(apOk ? "Portail captif actif — SSID: IrrigPro-Setup"
                : "Portail captif démarré MAIS AP radio indisponible (voir log série)");

    // Affichage OLED
    display.clear();
    display.setFont(ArialMT_Plain_10);
    if(apOk){
        display.drawString(0, 0,  "WiFi: ECHEC config!");
        display.drawString(0, 14, "AP: IrrigPro-Setup");
        display.drawString(0, 28, "-> 192.168.4.1");
        display.drawString(0, 42, "Configurer le WiFi");
    } else {
        display.drawString(0, 0,  "WiFi: ECHEC config!");
        display.drawString(0, 14, "AP: ECHEC demarrage");
        display.drawString(0, 28, "Voir port serie");
        display.drawString(0, 42, "Redemarrage conseille");
    }
    display.display();

    // Servir la page de config sur le serveur dédié (port 80)
    auto servePortal = [](AsyncWebServerRequest* req) {
        String options = "";
        if(!captivePortalScanAvailable){
            options = "<option value=\"\">Scan indisponible — saisir le r&eacute;seau ci-dessous</option>";
        } else {
            int n = WiFi.scanComplete();
            Serial.printf("[CaptivePortal] GET / -> scanComplete=%d\n", n);
            if (n == WIFI_SCAN_FAILED) {
                options = "<option value=\"\">Recherche en cours...</option>";
            } else if (n > 0) {
                for (int i = 0; i < n; ++i) {
                    options += "<option value=\"" + WiFi.SSID(i) + "\">" + WiFi.SSID(i) + " (" + String(WiFi.RSSI(i)) + " dBm)</option>";
                }
            }
        }

        String html = String(FPSTR(CAPTIVE_HTML_1)) + options + String(FPSTR(CAPTIVE_HTML_2));
        req->send(200, "text/html", html);
    };

    captiveServer.on("/scan", HTTP_GET, [](AsyncWebServerRequest* req) {
        String options = "";
        if(!captivePortalScanAvailable){
            options = "<option value=\"\">Scan indisponible — saisir le r&eacute;seau ci-dessous</option>";
            req->send(200, "text/html", options);
            return;
        }
        int n = WiFi.scanComplete();
        Serial.printf("[CaptivePortal] GET /scan -> scanComplete=%d\n", n);
        if (n == WIFI_SCAN_RUNNING) {
            options = "<option value=\"\">Recherche en cours...</option>";
        } else if (n == WIFI_SCAN_FAILED) {
            options = "<option value=\"\">Erreur scan</option>";
            WiFi.scanNetworks(true);
        } else if (n == 0) {
            options = "<option value=\"\">Aucun r&eacute;seau</option>";
            WiFi.scanNetworks(true);
        } else if (n > 0) {
            for (int i = 0; i < n; ++i) {
                options += "<option value=\"" + WiFi.SSID(i) + "\">" + WiFi.SSID(i) + " (" + String(WiFi.RSSI(i)) + " dBm)</option>";
            }
            WiFi.scanDelete();
            WiFi.scanNetworks(true); // relance pour le prochain refresh
        }
        req->send(200, "text/html", options);
    });
    captiveServer.on("/",                 HTTP_GET,  servePortal);
    captiveServer.on("/hotspot-detect.html", HTTP_GET, servePortal);  // iOS
    captiveServer.on("/generate_204",     HTTP_GET,  servePortal);   // Android
    captiveServer.on("/connecttest.txt",  HTTP_GET,  servePortal);   // Windows
    captiveServer.onNotFound(servePortal);

    // ── Traitement du formulaire POST /save
    captiveServer.on("/save", HTTP_POST, [](AsyncWebServerRequest* req) {
        String newSsid = "";
        String newPass = "";

        if (req->hasParam("ssid", true)) {
            newSsid = req->getParam("ssid", true)->value();
        }
        if (req->hasParam("pass", true)) {
            newPass = req->getParam("pass", true)->value();
        }

        if (newSsid.length() == 0) {
            req->send(400, "text/plain", "SSID vide");
            return;
        }

        // ── Sauvegarder dans NVS (seule opération safe dans un callback async)
        Preferences p2;
        p2.begin("irrigcfg", false);
        p2.putString("ssid",  newSsid);
        p2.putString("wpass", newPass);
        p2.end();
        Serial.printf("[CaptivePortal] Sauvegarde NVS SSID=%s\n", newSsid.c_str());

        // ── Afficher sur OLED AVANT que le WiFi se coupe
        display.clear();
        display.setFont(ArialMT_Plain_10);
        display.drawString(0,  0, "Params sauvegardes!");
        display.drawString(0, 14, newSsid);
        display.drawString(0, 30, "Redemarrage 2s...");
        display.display();

        // ── Répondre au navigateur (page de confirmation)
        req->send(200, "text/html",
            "<html><head><meta charset='UTF-8'/>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'/>"
            "</head><body style='background:#0d1117;color:#e6edf3;font-family:system-ui;"
            "display:flex;align-items:center;justify-content:center;min-height:100vh;margin:0'>"
            "<div style='text-align:center;padding:24px'>"
            "<div style='font-size:3rem'>&#10003;</div>"
            "<h2 style='color:#2ea043;margin:8px 0'>Enregistr&eacute;!</h2>"
            "<p style='color:#8b949e'>SSID: <strong style='color:#e6edf3'>" + newSsid + "</strong></p>"
            "<p style='color:#8b949e'>Red&eacute;marrage dans 2 secondes…</p>"
            "</div></body></html>");

        // ── Poser le flag : loop() fera ESP.restart() hors du callback
        pendingRestart = true;
        pendingRestartMs = millis();
    });

    captiveServer.begin();
    Serial.println("[CaptivePortal] Serveur HTTP portail démarré");
}

#endif // IOCAN
