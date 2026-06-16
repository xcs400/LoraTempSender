#pragma once
// ============================================================
// WebContent.h — Interface SPA HTML/CSS/JS
// Contrôleur d'arrosage professionnel 8 vannes
// Séparée de MainIocan.cpp pour lisibilité
// ============================================================

const char WEB_HTML[] PROGMEM = R"HTMLEOF(
<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>IrrigPro — Contrôleur d'arrosage</title>
<style>
/* ── TOKENS ─────────────────────────────────────────── */
:root {
  --bg:        #0d1117;
  --surface:   #161b22;
  --surface2:  #21262d;
  --border:    #30363d;
  --text:      #e6edf3;
  --text-muted:#8b949e;
  --green:     #2ea043;
  --green-dim: #1a4429;
  --orange:    #d29922;
  --orange-dim:#3d2b00;
  --red:       #da3633;
  --red-dim:   #3d0f0e;
  --blue:      #388bfd;
  --blue-dim:  #0d2d6e;
    /* Couleurs par vanne (8 max) */
    --vcol0: #1f77b4; 
    --vcol1: #ff7f0e; 
    --vcol2: #2ca02c; 
    --vcol3: #d62728;
    --vcol4: #9467bd; 
    --vcol5: #8c564b; 
    --vcol6: #e377c2; 
    --vcol7: #7f7f7f;
  --radius:    10px;
  --font:      'Inter', system-ui, sans-serif;
}
/* ── RESET ──────────────────────────────────────────── */
*,*::before,*::after{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--text);font-family:var(--font);min-height:100vh}
a{color:var(--blue);text-decoration:none}
button{cursor:pointer;font-family:inherit}
input,select{font-family:inherit}

/* ── LAYOUT ─────────────────────────────────────────── */
#app{display:flex;flex-direction:column;min-height:100vh}
header{
  background:var(--surface);border-bottom:1px solid var(--border);
  padding:0 24px;display:flex;align-items:center;gap:16px;height:56px;position:sticky;top:0;z-index:100
}
header h1{font-size:1rem;font-weight:600;letter-spacing:.5px;color:var(--text)}
header .badge{
  font-size:.7rem;background:var(--green-dim);color:var(--green);
  border:1px solid var(--green);border-radius:20px;padding:2px 10px
}
#ws-status{font-size:.7rem;margin-left:auto;display:flex;align-items:center;gap:6px}
#ws-dot{width:8px;height:8px;border-radius:50%;background:var(--red)}
#ws-dot.ok{background:var(--green)}

nav{
  background:var(--surface);border-bottom:1px solid var(--border);
  display:flex;overflow-x:auto;padding:0 16px
}
nav button{
  background:none;border:none;color:var(--text-muted);padding:14px 18px;
  font-size:.85rem;font-weight:500;white-space:nowrap;border-bottom:2px solid transparent;
  transition:color .15s,border-color .15s
}
nav button.active{color:var(--blue);border-bottom-color:var(--blue)}
nav button:hover{color:var(--text)}

main{flex:1;padding:24px;max-width:1200px;width:100%;margin:0 auto}

/* ── PAGES ──────────────────────────────────────────── */
.page{display:none}
.page.active{display:block}

/* ── CARDS DASHBOARD ────────────────────────────────── */
.valve-grid{
  display:grid;
  grid-template-columns:repeat(auto-fill,minmax(260px,1fr));
  gap:16px
}
.valve-card{
  background:var(--surface);border:1px solid var(--border);
  border-radius:var(--radius);padding:18px;
  transition:border-color .2s,box-shadow .2s;position:relative;overflow:hidden
}
.valve-card::before{
  content:'';position:absolute;top:0;left:0;right:0;height:3px;background:var(--border);
  transition:background .2s
}
.valve-card.open::before{background:var(--green)}
.valve-card.forced::before{background:var(--orange)}
.valve-card.alarm::before{background:var(--red)}

.valve-card .vc-header{display:flex;justify-content:space-between;align-items:flex-start;margin-bottom:12px}
.valve-card .vc-name{font-weight:600;font-size:.95rem}
.valve-card .vc-num{
  font-size:.7rem;color:var(--text-muted);background:var(--surface2);
  border-radius:20px;padding:2px 8px
}
.valve-card .vc-badge{
  display:inline-block;font-size:.72rem;padding:3px 10px;border-radius:20px;font-weight:600;
  margin-bottom:10px;
}
.badge-closed{background:var(--surface2);color:var(--text-muted)}
.badge-open{background:var(--green-dim);color:var(--green)}
.badge-forced{background:var(--orange-dim);color:var(--orange)}
.badge-alarm{background:var(--red-dim);color:var(--red)}

.valve-card .vc-remaining{font-size:1.4rem;font-weight:700;font-variant-numeric:tabular-nums;min-height:34px}
.valve-card .vc-meta{font-size:.75rem;color:var(--text-muted);margin:6px 0 14px}
.valve-card .vc-actions{display:flex;gap:8px;flex-wrap:wrap}

.btn{
  border:none;border-radius:6px;padding:7px 14px;font-size:.8rem;font-weight:600;
  transition:filter .15s,transform .1s;
}
.btn:hover{filter:brightness(1.15)}
.btn:active{transform:scale(.97)}
.btn-green{background:var(--green);color:#fff}
.btn-red{background:var(--red);color:#fff}
.btn-orange{background:var(--orange);color:#000}
.btn-ghost{background:var(--surface2);color:var(--text);border:1px solid var(--border)}
.btn-blue{background:var(--blue);color:#fff}
.btn-sm{padding:5px 10px;font-size:.75rem}

/* ── MODAL FORÇAGE ──────────────────────────────────── */
.modal-overlay{
  display:none;position:fixed;inset:0;background:rgba(0,0,0,.65);z-index:200;
  align-items:center;justify-content:center
}
.modal-overlay.open{display:flex}
.modal{
  background:var(--surface);border:1px solid var(--border);border-radius:var(--radius);
  padding:28px;min-width:320px;max-width:90vw
}
.modal h3{font-size:1rem;margin-bottom:16px}
.modal label{font-size:.82rem;color:var(--text-muted);display:block;margin-bottom:4px}
.modal input[type=number]{
  width:100%;background:var(--surface2);border:1px solid var(--border);
  border-radius:6px;color:var(--text);padding:8px 12px;font-size:.9rem;margin-bottom:14px
}
.modal-actions{display:flex;gap:10px;justify-content:flex-end}

/* ── TABLES ─────────────────────────────────────────── */
.card{
  background:var(--surface);border:1px solid var(--border);
  border-radius:var(--radius);overflow:hidden;margin-bottom:20px
}
.card-header{
  padding:16px 20px;border-bottom:1px solid var(--border);
  display:flex;justify-content:space-between;align-items:center
}
.card-header h2{font-size:.95rem;font-weight:600}
.tbl{width:100%;border-collapse:collapse;font-size:.84rem}
.tbl th{
  background:var(--surface2);color:var(--text-muted);font-weight:500;
  text-align:left;padding:10px 14px;border-bottom:1px solid var(--border)
}
.tbl td{padding:10px 14px;border-bottom:1px solid var(--border);vertical-align:middle}
.tbl tr:last-child td{border-bottom:none}
.tbl tr:hover td{background:var(--surface2)}

/* ── FORMULAIRE PROGRAMME ───────────────────────────── */
.form-grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(140px,1fr));gap:12px;margin-bottom:14px}
.form-group label{font-size:.78rem;color:var(--text-muted);display:block;margin-bottom:4px}
.form-group input,.form-group select{
  width:100%;background:var(--surface2);border:1px solid var(--border);
  border-radius:6px;color:var(--text);padding:7px 10px;font-size:.85rem
}
.days-picker{display:flex;gap:6px;flex-wrap:wrap;margin-bottom:14px}
.day-btn{
  background:var(--surface2);border:1px solid var(--border);color:var(--text-muted);
  border-radius:6px;padding:5px 10px;font-size:.78rem;cursor:pointer;transition:all .15s
}
.day-btn.sel{background:var(--blue-dim);border-color:var(--blue);color:var(--blue)}

/* ── CALENDRIER ─────────────────────────────────────── */
.cal-nav{display:flex;align-items:center;gap:16px;margin-bottom:16px}
.cal-nav h2{font-size:1rem;font-weight:600;min-width:160px;text-align:center}
.cal-grid{display:grid;grid-template-columns:repeat(7,1fr);gap:4px}
.cal-day-name{
  text-align:center;font-size:.72rem;color:var(--text-muted);
  padding:6px;font-weight:600
}
.cal-day{
  background:var(--surface2);border:1px solid var(--border);
  border-radius:6px;padding:8px 4px;text-align:center;min-height:56px;
  font-size:.82rem;cursor:default;position:relative
}
.cal-day.today{border-color:var(--blue);color:var(--blue)}
.cal-day.has-sched{background:var(--green-dim)}
.cal-day.other-month{opacity:.35}
.cal-day .cal-dots{display:flex;justify-content:center;gap:2px;margin-top:4px;flex-wrap:wrap}
.cal-dot{width:5px;height:5px;border-radius:50%;background:var(--green)}
.cal-badge{display:inline-block;padding:2px 6px;border-radius:8px;font-size:.7rem;background:var(--blue-dim);color:var(--blue);margin:2px}
.cal-badge[data-valve]{color:#fff}

/* ── JOURNAL ─────────────────────────────────────────── */
.log-list{max-height:520px;overflow-y:auto;padding:4px 0}
.log-entry{
  display:flex;gap:12px;padding:8px 20px;border-bottom:1px solid var(--border);
  font-size:.82rem;align-items:baseline
}
.log-entry:last-child{border-bottom:none}
.log-ts{color:var(--text-muted);font-variant-numeric:tabular-nums;white-space:nowrap;min-width:90px}
.log-badge{
  font-size:.7rem;padding:2px 7px;border-radius:20px;font-weight:600;white-space:nowrap
}
.log-badge.sys{background:var(--surface2);color:var(--text-muted)}
.log-badge.v{background:var(--blue-dim);color:var(--blue)}
.log-msg{color:var(--text)}

/* ── CONFIG ─────────────────────────────────────────── */
.config-section{margin-bottom:28px}
.config-section h3{font-size:.85rem;font-weight:600;color:var(--text-muted);text-transform:uppercase;letter-spacing:.8px;margin-bottom:14px;padding-bottom:8px;border-bottom:1px solid var(--border)}
.config-row{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-bottom:12px}
.config-row label{font-size:.82rem;color:var(--text-muted);display:block;margin-bottom:4px}
.config-row input,.config-row select{
  width:100%;background:var(--surface2);border:1px solid var(--border);
  border-radius:6px;color:var(--text);padding:8px 12px;font-size:.85rem
}
.toggle-row{display:flex;align-items:center;justify-content:space-between;padding:10px 0;border-bottom:1px solid var(--border)}
.toggle-row label{font-size:.85rem}
.toggle{position:relative;width:44px;height:24px}
.toggle input{opacity:0;width:0;height:0}
.toggle-slider{
  position:absolute;cursor:pointer;inset:0;background:var(--border);
  border-radius:24px;transition:.2s
}
.toggle-slider::before{
  content:'';position:absolute;height:18px;width:18px;left:3px;bottom:3px;
  background:#fff;border-radius:50%;transition:.2s
}
.toggle input:checked+.toggle-slider{background:var(--green)}
.toggle input:checked+.toggle-slider::before{transform:translateX(20px)}

/* ── RESPONSIVE ─────────────────────────────────────── */
@media(max-width:640px){
  main{padding:14px}
  .valve-grid{grid-template-columns:1fr}
  .config-row{grid-template-columns:1fr}
}
/* ── LOADER ─────────────────────────────────────────── */
.spinner{display:inline-block;width:18px;height:18px;border:2px solid var(--border);border-top-color:var(--blue);border-radius:50%;animation:spin .7s linear infinite}
@keyframes spin{to{transform:rotate(360deg)}}
/* Programmes table: inactive row styling */
.sched-row.inactive td{opacity:.6;color:var(--text-muted)}
</style>
</head>
<body>
<div id="app">

<!-- HEADER -->
<header>
  <svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="#388bfd" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2z"/><path d="M12 6v6l4 2"/></svg>
  <h1>IrrigPro</h1>
  <span class="badge">4 VANNES</span>
  <div id="ws-status">
    <span id="ws-dot"></span>
    <span id="ws-label">Déconnecté</span>
  </div>
</header>

<!-- NAV -->
<nav>
  <button class="active" onclick="showPage('dashboard',this)">Dashboard</button>
  <button onclick="showPage('programmes',this)">Programmes</button>
  <button onclick="showPage('calendrier',this)">Calendrier</button>
  <button onclick="showPage('journal',this)">Journal</button>
  <button onclick="showPage('config',this)">Configuration</button>
</nav>

<main>

<!-- ══ PAGE DASHBOARD ══════════════════════════════════ -->
<div id="page-dashboard" class="page active">
  <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:18px">
    <div>
      <div style="font-size:1.1rem;font-weight:700">Tableau de bord</div>
      <div style="font-size:.8rem;color:var(--text-muted)" id="uptime-label">—</div>
      <div id="valve-legend" style="margin-top:6px;display:flex;gap:8px;flex-wrap:wrap"></div>
    </div>
    <div style="display:flex;gap:8px">
      <button class="btn btn-ghost btn-sm" onclick="closeAll()">Tout fermer</button>
      <button class="btn btn-ghost btn-sm" onclick="api('POST','/api/lora/status')">Sync LoRa</button>
    </div>
  </div>
  <div class="valve-grid" id="valve-grid">
    <!-- cartes générées par JS -->
  </div>
</div>

<!-- ══ PAGE PROGRAMMES ═════════════════════════════════ -->
<div id="page-programmes" class="page">
  <div class="card">
    <div class="card-header">
      <h2>Programmes d'arrosage</h2>
      <button class="btn btn-blue btn-sm" onclick="openSchedModal()">+ Ajouter</button>
    </div>
    <table class="tbl" id="sched-table">
      <thead>
        <tr>
          <th>Vanne</th><th>Nom</th><th>Heure</th><th>Durée</th><th>Jours</th><th>Mode</th><th>Actif</th><th>Actions</th>
        </tr>
      </thead>
      <tbody id="sched-body">
        <tr><td colspan="8" style="text-align:center;color:var(--text-muted);padding:24px">Chargement…</td></tr>
      </tbody>
    </table>
  </div>
</div>

<!-- ══ PAGE CALENDRIER ════════════════════════════════ -->
<div id="page-calendrier" class="page">
  <div class="card" style="padding:20px">
    <div class="cal-nav">
      <button class="btn btn-ghost btn-sm" onclick="calPrev()">◀</button>
      <h2 id="cal-title">Juin 2024</h2>
      <button class="btn btn-ghost btn-sm" onclick="calNext()">▶</button>
    </div>
    <div class="cal-grid" id="cal-grid">
      <!-- généré JS -->
    </div>
  </div>
</div>

<!-- ══ PAGE JOURNAL ═══════════════════════════════════ -->
<div id="page-journal" class="page">
  <div class="card">
    <div class="card-header">
      <h2>Journal des événements</h2>
      <div style="display:flex;gap:8px;align-items:center">
        <span id="log-count" style="font-size:.8rem;color:var(--text-muted)"></span>
        <button class="btn btn-ghost btn-sm" onclick="loadLog()">Actualiser</button>
      </div>
    </div>
    <div class="log-list" id="log-list">
      <div style="padding:24px;text-align:center;color:var(--text-muted)">Chargement…</div>
    </div>
  </div>
</div>

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
    </div>

    <div class="config-section">
      <h3>NTP / Heure</h3>
      <div class="config-row">
        <div><label>Serveur NTP</label><input id="cfg-ntp" type="text"></div>
        <div><label>Fuseau horaire (secondes)</label><input id="cfg-tz" type="number"></div>
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

    <div style="display:flex;gap:10px;justify-content:flex-end;border-top:1px solid var(--border);padding-top:18px">
      <button class="btn btn-ghost" onclick="loadConfig()">Annuler</button>
      <button class="btn btn-blue" onclick="saveConfig()">Sauvegarder</button>
      <button class="btn btn-red" onclick="if(confirm('Redémarrer ?')) api('POST','/api/reset')">Redémarrer</button>
    </div>

  </div>
</div>

</main>
</div><!-- #app -->

<!-- ══ MODAL FORÇAGE ══════════════════════════════════ -->
<div class="modal-overlay" id="force-modal">
  <div class="modal">
    <h3>Forcer vanne <span id="force-modal-name"></span></h3>
    <label>Durée (secondes)</label>
    <input type="number" id="force-duration" value="1800" min="1" max="86400"/>
    <div class="modal-actions">
      <button class="btn btn-ghost" onclick="closeModal('force-modal')">Annuler</button>
      <button class="btn btn-orange" onclick="confirmForce()">Forcer</button>
    </div>
  </div>
</div>

<!-- ══ MODAL PROGRAMME ════════════════════════════════ -->
<div class="modal-overlay" id="sched-modal">
  <div class="modal" style="min-width:360px">
    <h3 id="sched-modal-title">Nouveau programme</h3>
    <input type="hidden" id="sched-edit-valve"/>
    <input type="hidden" id="sched-edit-idx"/>
    <div class="form-grid">
      <div class="form-group">
        <label>Vanne</label>
        <select id="sched-valve">
          <!-- généré JS -->
        </select>
      </div>
      <div class="form-group">
        <label>Nom programme</label>
        <input type="text" id="sched-name" placeholder="Nom (optionnel)" />
      </div>
      <div class="form-group">
        <label>Heure</label>
        <input type="time" id="sched-time" value="06:00"/>
      </div>
      <div class="form-group">
        <label>Durée (s)</label>
        <input type="number" id="sched-dur" value="900" min="30"/>
      </div>
      <div class="form-group">
        <label>Mode calendrier</label>
        <select id="sched-calmode" onchange="updateSchedCalMode()">
          <option value="0">Hebdomadaire</option>
          <option value="1">Intervalle</option>
          <option value="2">Saisonnier</option>
        </select>
      </div>
    </div>
    <!-- Jours semaine (mode hebdo) -->
    <div id="sched-days-row">
      <label style="font-size:.78rem;color:var(--text-muted);display:block;margin-bottom:6px">Jours</label>
      <div class="days-picker">
        <span class="day-btn sel" data-d="0">Lun</span>
        <span class="day-btn sel" data-d="1">Mar</span>
        <span class="day-btn sel" data-d="2">Mer</span>
        <span class="day-btn sel" data-d="3">Jeu</span>
        <span class="day-btn sel" data-d="4">Ven</span>
        <span class="day-btn" data-d="5">Sam</span>
        <span class="day-btn" data-d="6">Dim</span>
      </div>
    </div>
    <!-- Mode intervalle -->
    <div id="sched-interval-row" style="display:none" class="form-group">
      <label>Tous les N jours</label>
      <input type="number" id="sched-interval-n" value="2" min="1" max="365"/>
      <label style="margin-top:8px">Date de départ</label>
      <input type="date" id="sched-interval-start" />
    </div>
    <!-- Mode saison -->
    <div id="sched-season-row" style="display:none">
      <div class="form-grid">
        <div class="form-group"><label>Début (MM-JJ)</label><input type="text" id="sched-season-start" placeholder="04-01"/></div>
        <div class="form-group"><label>Fin (MM-JJ)</label><input type="text" id="sched-season-end" placeholder="10-31"/></div>
      </div>
    </div>
    <div class="modal-actions" style="margin-top:16px">
      <button class="btn btn-ghost" onclick="closeModal('sched-modal')">Annuler</button>
      <button class="btn btn-blue" onclick="saveSched()">Enregistrer</button>
    </div>
  </div>
</div>

<script>
// ══════════════════════════════════════════════════════════
// STATE
// ══════════════════════════════════════════════════════════
let valves = [];
let schedules = [];    // flat list {valve,idx,...}
let logEntries = [];
let sysConfig = {};
let wsConn = null;
let calDate = new Date();
let forceValveIdx = -1;

const DAY_NAMES = ['Lun','Mar','Mer','Jeu','Ven','Sam','Dim'];
const MONTH_NAMES = ['Janvier','Février','Mars','Avril','Mai','Juin','Juillet','Août','Septembre','Octobre','Novembre','Décembre'];

// ══════════════════════════════════════════════════════════
// UTILS
// ══════════════════════════════════════════════════════════
function api(method, url, body) {
  return fetch(url, {
    method,
    headers: body ? {'Content-Type':'application/json'} : {},
    body: body ? JSON.stringify(body) : undefined
  }).then(r => r.json()).catch(()=>({}));
}

function fmtSec(s) {
  if (!s || s <= 0) return '—';
  const h = Math.floor(s/3600), m = Math.floor((s%3600)/60), sec = s%60;
  if (h) return `${h}h ${m.toString().padStart(2,'0')}m`;
  if (m) return `${m}m ${sec.toString().padStart(2,'0')}s`;
  return `${sec}s`;
}

function fmtEpoch(e) {
  if (!e) return '—';
  const d = new Date(e*1000);
  return d.toLocaleString('fr-FR',{month:'2-digit',day:'2-digit',hour:'2-digit',minute:'2-digit'});
}

function weekdayBits(bits) {
  return DAY_NAMES.filter((_,i)=> bits & (1<<i)).join(' ');
}

// Vérifie si le programme `s` est actif pour la date `date` (ignore l'heure)
function scheduleMatchesOnDate(s, date){
  if(!s || !s.active) return false;
  const y = date.getFullYear(), m = date.getMonth(), d = date.getDate();
  if(s.calMode===0){ // hebdo
    const dow = (date.getDay()+6)%7; return !!(s.weekDays & (1<<dow));
  }
  if(s.calMode===1){ // intervalle
    if(!s.intervalDays || s.intervalDays<=0) return false;
    const yday = Math.floor((Date.UTC(y,m,d) - Date.UTC(y,0,1)) / 86400000);
    const startMonth = s.intervalStartMonth || 1;
    const startDay   = s.intervalStartDay   || 1;
    const startYday = Math.floor((Date.UTC(y, startMonth-1, startDay) - Date.UTC(y,0,1)) / 86400000);
    const isLeap = ((y%4===0) && (y%100!==0 || y%400===0));
    const daysInYear = 365 + (isLeap?1:0);
    const diff = (yday - startYday + daysInYear) % s.intervalDays;
    return diff === 0;
  }
  if(s.calMode===2){ // saison
    const md = (m+1)*100 + d;
    const start = (s.seasonStartMonth||1)*100 + (s.seasonStartDay||1);
    const end   = (s.seasonEndMonth||12)*100 + (s.seasonEndDay||31);
    return md>=start && md<=end;
  }
  return false;
}

// Retourne le prochain événement (objet {text,dt,sched}) pour la vanne idx ou null
function getNextEventForValve(idx){
  if(!schedules || !schedules.length) return null;
  const now = new Date();
  let best = null;
  for(const s of schedules){
    if(!s || !s.active) continue;
    if(s.valve !== idx) continue;
    // chercher jusqu'à 365 jours
    for(let off=0; off<366; off++){
      const cand = new Date(now.getFullYear(), now.getMonth(), now.getDate()+off);
      if(!scheduleMatchesOnDate(s, cand)) continue;
      const hh = s.hour || 0, mm = s.minute || 0;
      const candDt = new Date(cand.getFullYear(), cand.getMonth(), cand.getDate(), hh, mm, 0);
      if(candDt <= now) continue;
      if(!best || candDt < best.dt) best = {dt:candDt, sched:s};
      break; // pour ce programme on prend la première occurrence future
    }
  }
  if(!best) return null;
  const ms = best.dt - now;
  const days = Math.floor(ms/86400000);
  const hours = Math.floor((ms%86400000)/3600000);
  const minutes = Math.floor((ms%3600000)/60000);
  let text='';
  if(days>0) text = `dans ${days} j${days>1?'s':''} à ${String(best.dt.getHours()).padStart(2,'0')}h${String(best.dt.getMinutes()).padStart(2,'0')}`;
  else if(hours>0) text = `dans ${hours} h ${minutes} m`;
  else text = `dans ${minutes} m`;
  return {text,dt:best.dt,sched:best.sched};
}

function showPage(id, btn) {
  document.querySelectorAll('.page').forEach(p=>p.classList.remove('active'));
  document.querySelectorAll('nav button').forEach(b=>b.classList.remove('active'));
  document.getElementById('page-'+id).classList.add('active');
  if(btn) btn.classList.add('active');
  if(id==='programmes') renderSchedules();
  if(id==='calendrier') loadSchedules().then(()=>renderCalendar());
  if(id==='journal') loadLog();
  if(id==='config') loadConfig();
}

function closeModal(id) {
  document.getElementById(id).classList.remove('open');
}

// ══════════════════════════════════════════════════════════
// WEBSOCKET
// ══════════════════════════════════════════════════════════
function connectWS() {
  const wsUrl = 'ws://' + location.host + '/ws';
  wsConn = new WebSocket(wsUrl);
  wsConn.onopen = () => {
    document.getElementById('ws-dot').className = 'ok';
    document.getElementById('ws-label').textContent = 'Connecté';
  };
  wsConn.onclose = () => {
    document.getElementById('ws-dot').className = '';
    document.getElementById('ws-label').textContent = 'Déconnecté';
    setTimeout(connectWS, 3000);
  };
  wsConn.onmessage = (ev) => {
    try {
      const data = JSON.parse(ev.data);
      if (data.type === 'STATUS') handleStatus(data);
      if (data.type === 'LOG') prependLog(data.entry);
    } catch(e){}
  };
}

function handleStatus(data) {
  // data: {type, uptime, valves:[{name,state,source,remainingSec,openedAt,totalOpenSec},...]}
  valves = data.valves || valves;
  document.getElementById('uptime-label').textContent =
    'Uptime: ' + fmtSec(data.uptime || 0) + '  |  Heap: ' + (data.heap ? Math.round(data.heap/1024)+'kB' : '—');
  buildSchedValveSelect();
  renderLegend();
  renderValveCards();
}

function renderLegend(){
  const el = document.getElementById('valve-legend');
  if(!el) return;
  el.innerHTML = '';
  for(let i=0;i<valves.length;i++){
    const name = (valves[i]&&valves[i].name)||('V'+(i+1));
    const dot = `<span style="display:inline-flex;align-items:center;gap:6px;margin-right:6px">
      <span style="width:12px;height:12px;border-radius:3px;background:var(--vcol${i});display:inline-block;"></span>
      <span style="font-size:.78rem;color:var(--text-muted)">V${i+1} ${name}</span>
    </span>`;
    el.innerHTML += dot;
  }
}

// ══════════════════════════════════════════════════════════
// DASHBOARD — CARTES VANNES
// ══════════════════════════════════════════════════════════
function renderValveCards() {
  const grid = document.getElementById('valve-grid');
  if (!valves.length) {
    grid.innerHTML = '<div style="color:var(--text-muted);padding:24px">Aucune donnée reçue…</div>';
    return;
  }
  grid.innerHTML = valves.map((v,i) => {
    const isOpen   = v.state === 1;
    const isForced = isOpen && (v.source === 'INPUT' || v.source === 'WEB');
    const cardCls  = isForced ? 'forced' : isOpen ? 'open' : '';
    const badgeHtml = isForced
      ? `<span class="vc-badge badge-forced">⚡ Forcée (${v.source})</span>`
      : isOpen
        ? `<span class="vc-badge badge-open">● Ouverte (${v.source})</span>`
        : `<span class="vc-badge badge-closed">◌ Fermée</span>`;
    const nextEv = getNextEventForValve(i);
    const nextHtml = nextEv ? ( (nextEv.sched && nextEv.sched.name ? (nextEv.sched.name+' — ') : '') + nextEv.text ) : '—';
    return `
    <div class="valve-card ${cardCls}" id="vc-${i}" style="--vcol:var(--vcol${i}); --vcol-fg:#fff">
      <div class="vc-header">
        <span class="vc-name">${v.name||'Vanne '+(i+1)}</span>
        <span class="vc-num" style="background:var(--vcol);color:var(--vcol-fg)">V${i+1}</span>
      </div>
      ${badgeHtml}
      <div class="vc-remaining">${isOpen ? fmtSec(v.remainingSec) : '—'}</div>
      <div class="vc-meta">Dernier démarrage: ${fmtEpoch(v.openedAt)}<br>
        Total cumulé: ${fmtSec(v.totalOpenSec)}<br>
        Prochain: ${nextHtml}</div>
      <div class="vc-actions">
        <button class="btn btn-green btn-sm" onclick="openValve(${i})">Ouvrir</button>
        <button class="btn btn-red btn-sm" onclick="closeValve(${i})">Fermer</button>
        <button class="btn btn-orange btn-sm" onclick="showForceModal(${i})">Forcer</button>
      </div>
    </div>`;
  }).join('');
}

function openValve(idx) {
  api('POST','/api/valve/open',{valve:idx,duration:sysConfig.maxOpenSec||3600,source:'WEB'})
    .then(()=>requestStatus());
}
function closeValve(idx) {
  api('POST','/api/valve/close',{valve:idx,source:'WEB'})
    .then(()=>requestStatus());
}
function closeAll() {
  api('POST','/api/valve/closeall').then(()=>requestStatus());
}
function showForceModal(idx) {
  forceValveIdx = idx;
  const name = (valves[idx]&&valves[idx].name) || 'V'+(idx+1);
  document.getElementById('force-modal-name').textContent = name;
  document.getElementById('force-duration').value = sysConfig.manualForceSec || 1800;
  document.getElementById('force-modal').classList.add('open');
}
function confirmForce() {
  const dur = parseInt(document.getElementById('force-duration').value)||1800;
  api('POST','/api/valve/force',{valve:forceValveIdx,duration:dur,source:'WEB'})
    .then(()=>requestStatus());
  closeModal('force-modal');
}
function requestStatus() {
  api('GET','/api/status').then(d=>{
    if(d.valves) handleStatus(d);
  });
}

// ══════════════════════════════════════════════════════════
// PROGRAMMES
// ══════════════════════════════════════════════════════════
function loadSchedules() {
  return api('GET','/api/schedules').then(data=>{
    schedules = data.schedules || [];
    renderSchedules();
    return schedules;
  });
}

function renderSchedules() {
  const tbody = document.getElementById('sched-body');
  if(!schedules || !schedules.length){
    tbody.innerHTML = '<tr><td colspan="8" style="text-align:center;color:var(--text-muted);padding:24px">Aucun programme</td></tr>';
    return;
  }
  const CAL_MODES = ['Hebdo','Intervalle','Saison'];
  let rows = '';
  for(let i=0;i<schedules.length;i++){
    const s = schedules[i];
    if(!s) continue;
    const rowCls = s.active ? '' : 'sched-row inactive';
    rows += `<tr class="${rowCls}">
      <td>${(valves[s.valve]&&valves[s.valve].name)||'V'+(s.valve+1)}</td>
      <td>${s.name || ''}</td>
      <td>${String(s.hour).padStart(2,'0')}:${String(s.minute).padStart(2,'0')}</td>
      <td>${fmtSec(s.durationSec)}</td>
      <td style="font-size:.75rem">
        ${s.calMode==1?'Tous les '+s.intervalDays+'j': s.calMode==2?'Saison': weekdayBits(s.weekDays)||'—'}
      </td>
      <td><span style="font-size:.75rem;color:var(--text-muted)">${CAL_MODES[s.calMode||0]}</span></td>
      <td>
        <label class="toggle" style="width:36px;height:20px">
          <input type="checkbox" ${s.active?'checked':''} onchange="toggleSched(${s.valve},${s.schedIdx},this.checked)">
          <span class="toggle-slider"></span>
        </label>
      </td>
      <td>
        <button class="btn btn-ghost btn-sm" onclick="editSched(${i})">Éditer</button>
        <button class="btn btn-red btn-sm" onclick="deleteSched(${s.valve},${s.schedIdx})">Suppr.</button>
      </td>
    </tr>`;
  }
  tbody.innerHTML = rows;
}

function openSchedModal() {
  document.getElementById('sched-modal-title').textContent = 'Nouveau programme';
  document.getElementById('sched-edit-valve').value = '';
  document.getElementById('sched-edit-idx').value = '';
  document.getElementById('sched-valve').value = 0;
  document.getElementById('sched-name').value = '';
  document.getElementById('sched-time').value = '06:00';
  document.getElementById('sched-dur').value = 900;
  document.getElementById('sched-calmode').value = 0;
  // interval start default = today
  const today = new Date();
  document.getElementById('sched-interval-start').value = today.toISOString().slice(0,10);
  updateSchedCalMode();
  // Reset jours
  document.querySelectorAll('.day-btn').forEach(b=>{
    b.classList.toggle('sel', parseInt(b.dataset.d)<5);
  });
  document.getElementById('sched-modal').classList.add('open');
}

function editSched(flatIdx) {
  const s = schedules[flatIdx];
  document.getElementById('sched-modal-title').textContent = 'Modifier programme';
  document.getElementById('sched-edit-valve').value = s.valve;
  document.getElementById('sched-edit-idx').value = s.schedIdx;
  document.getElementById('sched-valve').value = s.valve;
  document.getElementById('sched-time').value =
    String(s.hour).padStart(2,'0')+':'+String(s.minute).padStart(2,'0');
  document.getElementById('sched-dur').value = s.durationSec;
  document.getElementById('sched-name').value = s.name || '';
  document.getElementById('sched-calmode').value = s.calMode||0;
  document.getElementById('sched-interval-n').value = s.intervalDays||2;
  if(s.intervalStartMonth && s.intervalStartDay){
    const y = (new Date()).getFullYear();
    const mm = String(s.intervalStartMonth).padStart(2,'0');
    const dd = String(s.intervalStartDay).padStart(2,'0');
    document.getElementById('sched-interval-start').value = `${y}-${mm}-${dd}`;
  } else {
    document.getElementById('sched-interval-start').value = (new Date()).toISOString().slice(0,10);
  }
  if(s.seasonStartMonth) {
    document.getElementById('sched-season-start').value =
      String(s.seasonStartMonth).padStart(2,'0')+'-'+String(s.seasonStartDay).padStart(2,'0');
    document.getElementById('sched-season-end').value =
      String(s.seasonEndMonth).padStart(2,'0')+'-'+String(s.seasonEndDay).padStart(2,'0');
  }
  document.querySelectorAll('.day-btn').forEach(b=>{
    b.classList.toggle('sel', !!(s.weekDays & (1<<parseInt(b.dataset.d))));
  });
  updateSchedCalMode();
  document.getElementById('sched-modal').classList.add('open');
}

function updateSchedCalMode() {
  const m = parseInt(document.getElementById('sched-calmode').value);
  document.getElementById('sched-days-row').style.display    = m===0?'block':'none';
  document.getElementById('sched-interval-row').style.display= m===1?'block':'none';
  document.getElementById('sched-season-row').style.display  = m===2?'block':'none';
}

document.querySelectorAll('.day-btn').forEach(b=>{
  b.addEventListener('click',()=>b.classList.toggle('sel'));
});

function saveSched() {
  const time = document.getElementById('sched-time').value.split(':');
  let weekDays = 0;
  document.querySelectorAll('.day-btn.sel').forEach(b=>{ weekDays |= (1<<parseInt(b.dataset.d)); });
  const calMode = parseInt(document.getElementById('sched-calmode').value);
  const [sm,sd] = (document.getElementById('sched-season-start').value||'01-01').split('-').map(Number);
  const [em,ed] = (document.getElementById('sched-season-end').value||'12-31').split('-').map(Number);
  const body = {
    valve: parseInt(document.getElementById('sched-valve').value),
    schedIdx: document.getElementById('sched-edit-idx').value !== ''
              ? parseInt(document.getElementById('sched-edit-idx').value) : -1,
    origValve: (function(){ const v=document.getElementById('sched-edit-valve').value; return v!==''?parseInt(v):undefined })(),
    name: (document.getElementById('sched-name').value||'').substr(0,23),
    hour: parseInt(time[0]), minute: parseInt(time[1]),
    durationSec: parseInt(document.getElementById('sched-dur').value),
    calMode, weekDays,
    intervalDays: parseInt(document.getElementById('sched-interval-n').value)||2,
    intervalStartMonth: (function(){
      const v=document.getElementById('sched-interval-start').value; if(!v) return undefined; return parseInt(v.split('-')[1]);
    })(),
    intervalStartDay: (function(){
      const v=document.getElementById('sched-interval-start').value; if(!v) return undefined; return parseInt(v.split('-')[2]);
    })(),
    seasonStartMonth:sm||1, seasonStartDay:sd||1,
    seasonEndMonth:em||12, seasonEndDay:ed||31,
    active: true
  };
  api('POST','/api/schedule/save',body).then(()=>loadSchedules());
  closeModal('sched-modal');
}

function deleteSched(valve, schedIdx) {
  if(!confirm('Supprimer ce programme ?')) return;
  api('POST','/api/schedule/delete',{valve,schedIdx}).then(()=>loadSchedules());
}

function toggleSched(valve, schedIdx, active) {
  api('POST','/api/schedule/toggle',{valve,schedIdx,active}).then(()=>{
    loadSchedules();
    renderCalendar();
  });
}

// ══════════════════════════════════════════════════════════
// CALENDRIER
// ══════════════════════════════════════════════════════════
function calPrev() { calDate.setMonth(calDate.getMonth()-1); renderCalendar(); }
function calNext() { calDate.setMonth(calDate.getMonth()+1); renderCalendar(); }

function renderCalendar() {
  const y = calDate.getFullYear(), m = calDate.getMonth();
  document.getElementById('cal-title').textContent = MONTH_NAMES[m]+' '+y;
  const today = new Date();

  // Construire map jour -> programmes actifs (pour afficher badges Vx)
  const dayScheds = {};
  const daysInMonth = new Date(y,m+1,0).getDate();
  schedules.forEach(s=>{
    if(!s || !s.active) return;
    for(let d=1; d<=daysInMonth; d++){
      const date = new Date(y,m,d);
      const dow = (date.getDay()+6)%7; // 0=Lun
      let match = false;
      if(s.calMode===0) match = !!(s.weekDays & (1<<dow));
      else if(s.calMode===1){
        if(s.intervalDays && s.intervalDays>0){
          const yday = Math.floor((Date.UTC(y, m, d) - Date.UTC(y, 0, 1)) / 86400000);
          const startMonth = s.intervalStartMonth || 1;
          const startDay   = s.intervalStartDay   || 1;
          const startYday = Math.floor((Date.UTC(y, startMonth-1, startDay) - Date.UTC(y,0,1)) / 86400000);
          const isLeap = ((y%4===0) && (y%100!==0 || y%400===0));
          const daysInYear = 365 + (isLeap?1:0);
          const diff = (yday - startYday + daysInYear) % s.intervalDays;
          match = (diff === 0);
        } else match = false;
      }
      else if(s.calMode===2){
        const md = (m+1)*100 + d;
        const start = s.seasonStartMonth*100 + s.seasonStartDay;
        const end   = s.seasonEndMonth*100 + s.seasonEndDay;
        match = md>=start && md<=end;
      }
      if(match){
        dayScheds[d] = dayScheds[d] || [];
        dayScheds[d].push(s);
      }
    }
  });

  const firstDay = new Date(y,m,1);
  const startDow = (firstDay.getDay()+6)%7;

  let html = DAY_NAMES.map(d=>`<div class="cal-day-name">${d}</div>`).join('');

  // Cases vides avant le 1er
  for(let i=0;i<startDow;i++) html += '<div class="cal-day other-month"></div>';

  for(let d=1; d<=daysInMonth; d++){
    const isToday = today.getFullYear()===y && today.getMonth()===m && today.getDate()===d;
    const scheds = dayScheds[d] || [];
    const hasSched = scheds.length>0;
    let badges = '';
    if(hasSched){
      badges = '<div class="cal-dots">';
      for(let sd of scheds){
        const title = ((sd.name||'') + ' ' + String(sd.hour).padStart(2,'0')+':'+String(sd.minute).padStart(2,'0') + ' ('+fmtSec(sd.durationSec)+')').replace(/"/g,'&quot;');
        // use data-tip for custom tooltip and data-valve for coloring
        badges += `<span class="cal-badge" data-valve="${sd.valve}" data-tip="${title}" style="background:var(--vcol${sd.valve});">V${sd.valve+1}</span>`;
      }
      badges += '</div>';
    }
    html += `<div class="cal-day ${isToday?'today':''} ${hasSched?'has-sched':''}">${d}${badges}</div>`;
  }

  document.getElementById('cal-grid').innerHTML = html;

  // Tooltip helper (works for hover and touch)
  const tt = document.getElementById('cal-tooltip') || (function(){
    const d = document.createElement('div'); d.id='cal-tooltip'; d.style.position='fixed'; d.style.zIndex=300; d.style.background='rgba(0,0,0,0.85)'; d.style.color='#fff'; d.style.padding='6px 8px'; d.style.borderRadius='6px'; d.style.fontSize='0.85rem'; d.style.maxWidth='280px'; d.style.display='none'; d.style.pointerEvents='none'; document.body.appendChild(d); return d;
  })();
  let ttHideTimer = null;
  function showTooltip(el, text){
    if(!text) return;
    tt.textContent = text;
    tt.style.display = 'block';
    const r = el.getBoundingClientRect();
    let top = r.top - tt.offsetHeight - 8; if(top<8) top = r.bottom + 8;
    let left = r.left + (r.width/2) - (tt.offsetWidth/2);
    if(left<8) left = 8; if(left + tt.offsetWidth > window.innerWidth-8) left = window.innerWidth - tt.offsetWidth - 8;
    tt.style.top = top + 'px'; tt.style.left = left + 'px';
    if(ttHideTimer) clearTimeout(ttHideTimer);
    ttHideTimer = setTimeout(()=>{ tt.style.display='none'; }, 6000);
  }
  function hideTooltip(){ if(tt) tt.style.display='none'; }
  // delegated events
  document.querySelectorAll('.cal-badge').forEach(b=>{
    b.addEventListener('mouseenter', e=> showTooltip(b, b.dataset.tip));
    b.addEventListener('mouseleave', e=> hideTooltip());
    b.addEventListener('click', e=> { e.preventDefault(); showTooltip(b, b.dataset.tip); });
    b.addEventListener('touchstart', e=> { showTooltip(b, b.dataset.tip); });
  });
}

// ══════════════════════════════════════════════════════════
// JOURNAL
// ══════════════════════════════════════════════════════════
function loadLog() {
  api('GET','/api/log?n=200').then(data=>{
    logEntries = data||[];
    document.getElementById('log-count').textContent = logEntries.length + ' événements';
    renderLog();
  });
}

function renderLog() {
  const list = document.getElementById('log-list');
  if(!logEntries.length){
    list.innerHTML='<div style="padding:24px;text-align:center;color:var(--text-muted)">Aucun événement</div>';
    return;
  }
  list.innerHTML = [...logEntries].reverse().map(e=>{
    const dt = e.epoch ? new Date(e.epoch*1000).toLocaleString('fr-FR') : '—';
    const badge = e.valve==='SYS'
      ? '<span class="log-badge sys">SYS</span>'
      : `<span class="log-badge v">V${e.valve}</span>`;
    return `<div class="log-entry"><span class="log-ts">${dt}</span>${badge}<span class="log-msg">${e.msg}</span></div>`;
  }).join('');
}

function prependLog(entry) {
  logEntries.push(entry);
  if(logEntries.length>1000) logEntries.shift();
  const list=document.getElementById('log-list');
  if(list && document.getElementById('page-journal').classList.contains('active')) renderLog();
}

// ══════════════════════════════════════════════════════════
// CONFIGURATION
// ══════════════════════════════════════════════════════════
function loadConfig() {
  api('GET','/api/config').then(cfg=>{
    sysConfig = cfg;
    document.getElementById('cfg-ssid').value    = cfg.ssid||'';
    document.getElementById('cfg-wpass').value   = '';
    document.getElementById('cfg-ntp').value     = cfg.ntpServer||'pool.ntp.org';
    document.getElementById('cfg-tz').value      = cfg.tzOffset||3600;
    document.getElementById('cfg-lfreq').value   = cfg.loraFreq||868.0;
    document.getElementById('cfg-lpow').value    = cfg.loraPower||10;
    document.getElementById('cfg-nodeid').value  = cfg.nodeId||'IRRIGATION01';
    document.getElementById('cfg-seq').checked   = cfg.irrigMode===1;
    document.getElementById('cfg-maxopen').value = cfg.maxOpenSec||3600;
    document.getElementById('cfg-forcedu').value = cfg.manualForceSec||1800;
    // Noms vannes
    const grid = document.getElementById('valve-names-grid');
    grid.innerHTML = (cfg.valveNames||[]).map((n,i)=>
      `<div class="form-group"><label>Vanne ${i+1}</label>
       <input type="text" id="vname-${i}" value="${n||'Vanne '+(i+1)}"/></div>`
    ).join('');
  });
}

function saveConfig() {
  const valveNames = [];
  for(let i=0;i<8;i++){
    const el=document.getElementById('vname-'+i);
    valveNames.push(el?el.value:'Vanne '+(i+1));
  }
  const body = {
    ssid: document.getElementById('cfg-ssid').value,
    wifiPass: document.getElementById('cfg-wpass').value||undefined,
    ntpServer: document.getElementById('cfg-ntp').value,
    tzOffset: parseInt(document.getElementById('cfg-tz').value),
    loraFreq: parseFloat(document.getElementById('cfg-lfreq').value),
    loraPower: parseInt(document.getElementById('cfg-lpow').value),
    nodeId: document.getElementById('cfg-nodeid').value,
    irrigMode: document.getElementById('cfg-seq').checked?1:0,
    maxOpenSec: parseInt(document.getElementById('cfg-maxopen').value),
    manualForceSec: parseInt(document.getElementById('cfg-forcedu').value),
    valveNames
  };
  api('POST','/api/config',body).then(()=>alert('Configuration sauvegardée'));
}

// ══════════════════════════════════════════════════════════
// INIT
// ══════════════════════════════════════════════════════════
function buildSchedValveSelect() {
  const sel = document.getElementById('sched-valve');
  const count = (valves && valves.length) ? valves.length : 8;
  const prev = sel.value;
  sel.innerHTML = Array.from({length:count},(_,i)=>`<option value="${i}">Vanne ${i+1}</option>`).join('');
  // restore previous selection if still valid
  if(prev !== undefined && prev !== null && prev !== ''){
    const opt = sel.querySelector(`option[value="${prev}"]`);
    if(opt) sel.value = prev;
  }
}

function init() {
  buildSchedValveSelect();
  connectWS();
  requestStatus();
  loadSchedules();
  // Actualisation auto toutes les 10s si WS déconnecté
  setInterval(()=>{ if(!wsConn||wsConn.readyState!==1) requestStatus(); }, 10000);
}

init();
</script>
</body>
</html>
)HTMLEOF";
