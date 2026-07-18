#pragma once
// ============================================================
// WebHTML_Config.h — Page Configuration HTML
// ============================================================
// Contenu identique à l'ancien bloc PAGE CONFIGURATION de WebContent.h.
// IDs conservés à l'identique pour que loadConfig() / saveConfig() /
// loadNvsStatus() / formatNvs() / refreshConsumption() (JS dans
// WebContent.h) retrouvent tous les éléments qu'ils manipulent :
//   • valve-names-grid
//   • cfg-ssid, cfg-wpass, cfg-ntp, cfg-tz, cfg-tz-posix
//   • cfg-lfreq, cfg-lpow, cfg-nodeid
//   • cfg-seq, cfg-maxopen, cfg-forcedu
//   • pulse-count, pulse-litres
//   • cons-body
//   • cfg-mqena, cfg-mqhost, cfg-mqport, cfg-mquser, cfg-mqpass,
//     cfg-mqprefix, cfg-mqid
//   • nvs-pct, nvs-fill, nvs-detail
//   • wifi-scan-btn, wifi-scan-status, wifi-networks
// Ne pas modifier les IDs sans mettre à jour le JS en parallèle.



const char WEB_CONFIG_HTML[] PROGMEM = R"HTML(


<!-- ══ PAGE CONFIGURATION ════════════════════════════ -->
<div id="page-config" class="page">
  <div class="card" style="padding:24px">

    <div class="config-section">
      <h3>Noms des vannes</h3>
      <div id="valve-names-grid" style="display:grid;grid-template-columns:repeat(auto-fill,minmax(200px,1fr));gap:10px"></div>
    </div>

    <div class="config-section">
      <h3>WiFi</h3>
      <div class="config-row">
        <div><label>SSID</label><input id="cfg-ssid" type="text"></div>
        <div><label>Mot de passe</label><input id="cfg-wpass" type="password"></div>
      </div>
      <!-- Bloc "réseaux détectés" : même fonction que la liste de la page
           AP du portail captif (SSID + RSSI + type de chiffrement + barres
           de signal), accessible ici depuis la page de configuration
           principale quand l'ESP est déjà connecté en STA. -->
      <div style="margin-top:10px;display:flex;gap:10px;align-items:center">
        <button class="btn btn-ghost btn-sm" id="wifi-scan-btn" onclick="scanWifiNetworks()">📡 Réseaux détectés</button>
        <span id="wifi-scan-status" style="font-size:.78rem;color:var(--text-muted)"></span>
      </div>
      <div id="wifi-networks" style="margin-top:8px"></div>
    </div>

    <div class="config-section">
      <h3>NTP / Heure</h3>
      <div class="config-row">
        <div><label>Serveur NTP</label><input id="cfg-ntp" type="text"></div>
        <div><label>Fuseau horaire (secondes, hiver)</label><input id="cfg-tz" type="number"></div>
      </div>
      <div class="config-row" style="margin-top:8px">
        <div style="flex:1">
          <label>Fuseau POSIX (heure été/hiver automatique) <a href="https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv" target="_blank" style="font-size:.75rem;color:var(--blue)">[aide]</a></label>
          <input id="cfg-tz-posix" type="text" placeholder="ex: CET-1CEST,M3.5.0,M10.5.0/3">
          <div style="font-size:.72rem;color:var(--text-muted);margin-top:3px">
            ▸ France (Paris) : <code style="color:var(--blue);cursor:pointer" onclick="document.getElementById('cfg-tz-posix').value=this.textContent">CET-1CEST,M3.5.0,M10.5.0/3</code>
            &nbsp;—&nbsp;Si renseigné, prend la priorité sur le champ secondes ci-dessus.
          </div>
        </div>
      </div>
    </div>

    <div class="config-section">
      <h3>LoRa</h3>
      <div class="config-row">
        <div><label>Fréquence (MHz)</label><input id="cfg-lfreq" type="number" step="0.1"></div>
        <div><label>Puissance (dBm)</label><input id="cfg-lpow" type="number"></div>
      </div>
      <div class="config-row">
        <div><label>ID Nœud</label><input id="cfg-nodeid" type="text"></div>
      </div>
      <div style="margin-top:12px;display:flex;gap:10px;align-items:center">
        <button class="btn btn-ghost btn-sm" onclick="api('POST','/api/lora/status').then(()=>alert('Sync LoRa envoyée'))">Sync LoRa</button>
        <span style="font-size:.78rem;color:var(--text-muted)">Force l'émission immédiate d'un message STATUS vers les autres nœuds.</span>
      </div>
    </div>

    <div class="config-section">
      <h3>Arrosage</h3>
      <div class="toggle-row">
        <label>Mode séquentiel (une vanne à la fois)</label>
        <label class="toggle"><input type="checkbox" id="cfg-seq"><span class="toggle-slider"></span></label>
      </div>
      <div class="config-row" style="margin-top:12px">
        <div><label>Durée max ouverture (s)</label><input id="cfg-maxopen" type="number"></div>
        <div><label>Durée forçage manuel (s)</label><input id="cfg-forcedu" type="number"></div>
      </div>
    </div>

    <div class="config-section">
      <h3>Compteur d'impulsions</h3>
      <div style="display:flex;gap:12px;align-items:center;">
        <div style="font-size:1rem">Total: <span id="pulse-count">—</span> pulses (<span id="pulse-litres">—</span> L)</div>
        <button class="btn btn-ghost btn-sm" onclick="refreshPulse()">Actualiser</button>
        <button class="btn btn-red btn-sm" onclick="if(confirm('Remettre le compteur à zéro ?')) resetPulse()">RAZ</button>
      </div>
    </div>

    <div class="config-section">
      <h3>Consommation par vanne</h3>
      <div style="font-size:.78rem;color:var(--text-muted);margin-bottom:10px">
        Division simple du compteur global par le nombre de vannes ouvertes (calibration à venir).
      </div>
      <div style="display:flex;gap:8px;margin-bottom:10px">
        <button class="btn btn-ghost btn-sm" onclick="refreshConsumption()">Actualiser</button>
      </div>
      <div style="overflow-x:auto">
        <table class="tbl" id="cons-table">
          <thead>
            <tr>
              <th>Vanne</th>
              <th>Nom</th>
              <th>Aujourd'hui (L)</th>
              <th>Total (L)</th>
              <th>Détails 14 j</th>
            </tr>
          </thead>
          <tbody id="cons-body">
            <tr><td colspan="5" style="text-align:center;color:var(--text-muted);padding:18px">Chargement…</td></tr>
          </tbody>
        </table>
      </div>
    </div>

    <div class="config-section">
      <h3>Home Assistant (MQTT)</h3>
      <div style="font-size:.78rem;color:var(--text-muted);margin-bottom:10px">
        Découverte automatique des capteurs + vannes dans Home Assistant.
        Les switchs sont commandables depuis HA (un clic = ouverture / fermeture).
      </div>
      <div class="toggle-row">
        <label>Activer MQTT</label>
        <label class="toggle"><input type="checkbox" id="cfg-mqena"><span class="toggle-slider"></span></label>
      </div>
      <div class="config-row" style="margin-top:12px">
        <div><label>Broker (host)</label><input id="cfg-mqhost" type="text"/></div>
        <div><label>Port</label><input id="cfg-mqport" type="number"/></div>
      </div>
      <div class="config-row">
        <div><label>Utilisateur</label><input id="cfg-mquser" type="text"/></div>
        <div><label>Mot de passe</label><input id="cfg-mqpass" type="password"/></div>
      </div>
      <div class="config-row">
        <div><label>Préfixe topic (défaut: homeassistant)</label><input id="cfg-mqprefix" type="text"/></div>
        <div><label>ID nœud MQTT (unique_id HA)</label><input id="cfg-mqid" type="text"/></div>
      </div>
      <div style="margin-top:12px;display:flex;gap:10px;align-items:center">
        <button class="btn btn-ghost btn-sm" onclick="api('POST','/api/mqtt/disconnect').then(()=>alert('Déconnexion MQTT forcée (Debug)'))">⚠ Forcer Déconnexion</button>
        <span style="font-size:.78rem;color:var(--text-muted)">Pour debug (simule une perte de connexion réseau).</span>
      </div>
    </div>

    <!-- ── MAINTENANCE / ÉTAT NVS ──────────────────────────────
         Jauge de remplissage de la partition NVS + bouton pour
         formater (effacer toute la config persistée, mais en gardant
         le SSID et mot de passe WiFi). Le rafraichissement se fait
         toutes les 5s par le firmware (cache) + 1 appel à l'ouverture
         de la page pour avoir la valeur immédiate. -->
    <div class="config-section">
      <h3>Maintenance — Mémoire flash (NVS)</h3>
      <div class="nvs-gauge-wrap" id="nvs-gauge-wrap">
        <div class="nvs-gauge-label">
          <span>Remplissage partition NVS</span>
          <span class="pct" id="nvs-pct">—</span>
        </div>
        <div class="nvs-gauge"><div class="nvs-gauge-fill" id="nvs-fill"></div></div>
        <div class="nvs-gauge-detail" id="nvs-detail">Chargement…</div>
      </div>
      <div style="display:flex;gap:10px;align-items:center;margin-top:14px;flex-wrap:wrap">
        <button class="btn btn-ghost btn-sm" onclick="loadNvsStatus()">↻ Actualiser l'état</button>
        <button class="btn btn-red btn-sm" onclick="formatNvs()">⚠ Formater la mémoire flash</button>
        <span style="font-size:.72rem;color:var(--text-muted);flex:1">
          Le formatage efface toute la configuration (sauf le SSID et mot de passe WiFi qui sont recopiés immédiatement après).<br/>
          Utile en cas de partition NVS saturée ou corrompue.
        </span>
      </div>
    </div>

    <div style="display:flex;gap:10px;justify-content:flex-end;border-top:1px solid var(--border);padding-top:18px">
      <button class="btn btn-ghost" onclick="loadConfig()">Annuler</button>
      <button class="btn btn-blue" onclick="saveConfig()">Sauvegarder</button>
      <button class="btn btn-red" onclick="if(confirm('Redémarrer ?')) api('POST','/api/reset')">Redémarrer</button>
    </div>

  </div>
</div>

)HTML";
