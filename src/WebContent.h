#pragma once
// ============================================================
// WebContent.h — Interface SPA HTML/CSS/JS
// Contrôleur d'arrosage professionnel 8 vannes
// Séparée de MainIocan.cpp pour lisibilité
//
// CORRECTIFS appliqués dans cette révision (revue complète demandée) :
//   1) handleStatus() lit désormais data.tempR, qui est maintenant bien
//      envoyé par buildStatusJson() côté firmware (voir MainIocan.cpp).
//   2) saveConfig() : fallback "8" remplacé par une constante VALVE_COUNT_FALLBACK
//      alignée sur le VANNE_COUNT réel du firmware (5), pour éviter d'envoyer
//      des noms de vannes fantômes si /api/config n'a pas encore répondu.
//   3) buildSchedValveSelect() : même fallback corrigé (5 au lieu de 8), pour
//      éviter de proposer des vannes inexistantes dans le modal programme
//      avant la première réponse status.
//   4) closeAll() : ajout d'une confirmation utilisateur avant de fermer
//      toutes les vannes (action destructive sans garde-fou auparavant).
//   5) scheduleMatchesOnDate() (mode intervalle) : calcul du jour de l'année
//      aligné sur UTC (comme renderCalendar()) au lieu d'un calcul en heure
//      locale, pour éviter une désynchronisation d'un jour entre le badge
//      du calendrier et le texte "Prochain événement" sur la carte vanne.
//   6) Commentaire weekDays par défaut clarifié (cohérent avec le firmware).
//   B) saveConfig() : vérifie désormais d.ok avant d'afficher "sauvegardée",
//      pour ne pas induire l'utilisateur en erreur en cas d'échec réseau.
//   A) Badge MQTT ajouté dans le header, reflète mqttConnected envoyé par
//      le firmware dans buildStatusJson().
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
/* Badge MQTT (amélioration A) */
#mqtt-status{font-size:.7rem;display:flex;align-items:center;gap:6px}
#mqtt-dot{width:8px;height:8px;border-radius:50%;background:var(--text-muted)}
#mqtt-dot.ok{background:var(--green)}
#mqtt-dot.off{background:var(--red)}

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
  /* darker surface with stronger tint */
  background:color-mix(in srgb, var(--vcol) 16%, var(--surface));
  /* full saturated border using valve color for emphasis */
  border:2px solid var(--vcol);
  border-radius:var(--radius);padding:18px;
  transition:border-color .2s,box-shadow .2s;position:relative;overflow:hidden;box-shadow:0 6px 18px rgba(0,0,0,0.35)
}
.valve-card::before{
  content:'';position:absolute;top:0;left:0;right:0;height:3px;
  background:color-mix(in srgb, var(--vcol) 30%, transparent);
  transition:background .2s, height .2s;
}
.valve-card.open::before, .valve-card.forced::before, .valve-card.alarm::before {
  background:var(--vcol);
  height: 4px;
}

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
/* Petit indicateur "débit calibré" affiché à côté du badge d'état.
   Toujours visible (vanne ouverte ou fermée) pour rappeler la capacité
   hydraulique de la ligne, indépendamment de l'état courant. */
.vc-flow{
  display:inline-block;font-size:.68rem;padding:2px 8px;border-radius:20px;
  background:var(--blue-dim);color:var(--blue);font-weight:600;
  font-variant-numeric:tabular-nums;margin-left:6px;vertical-align:middle;
  border:1px solid color-mix(in srgb, var(--blue) 40%, transparent);
}
.vc-flow.uncal{background:var(--surface2);color:var(--text-muted);border-color:var(--border);font-weight:500}

.valve-card .vc-remaining{font-size:1.4rem;font-weight:700;font-variant-numeric:tabular-nums;min-height:34px;color:var(--text)}
/* make remaining time red when valve is open/forced/alarm */
.valve-card.open .vc-remaining,
.valve-card.forced .vc-remaining,
.valve-card.alarm .vc-remaining{ color: var(--red); }
/* ── Volume restant ─────────────────────────────────────────
   Affiché sous le temps restant quand une vanne est ouverte.
   Couleur bleue pour différencier visuellement du temps (rouge).
   Calcul côté client : flowCoeff[i] (pulses/s) × remainingSec /
   pulsesPerLitre = litres restants. Si pas de calibration, on retombe
   sur l'estimation "1.0 pulse/s" (= 1.0 / PULSES_PER_LITRE L/s) ce qui
   correspond au comportement par défaut avant toute calibration. */
.valve-card .vc-remaining-l{
  font-size:1rem;font-weight:600;font-variant-numeric:tabular-nums;
  color:var(--blue);margin-top:2px;min-height:18px;
}
.valve-card .vc-remaining-l .vc-remaining-l-label{
  font-size:.7rem;font-weight:500;color:var(--text-muted);
  margin-right:6px;text-transform:uppercase;letter-spacing:.4px;
}
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

/* ── JAUGE NVS ──────────────────────────────────────────
 * Petite barre horizontale qui affiche le remplissage de la partition
 * NVS (utilisé / total). Couleur qui vire au rouge/orange au-dessus
 * de 70% / 90% pour alerter visuellement. */
.nvs-gauge-wrap{
  display:flex;flex-direction:column;gap:6px;
  padding:10px 12px;background:var(--surface2);
  border:1px solid var(--border);border-radius:8px;
}
.nvs-gauge-label{
  display:flex;justify-content:space-between;align-items:center;
  font-size:.78rem;color:var(--text-muted);
}
.nvs-gauge-label .pct{font-weight:600;color:var(--text)}
.nvs-gauge{
  width:100%;height:8px;background:var(--border);
  border-radius:4px;overflow:hidden;
}
.nvs-gauge-fill{
  height:100%;width:0%;background:var(--green);
  transition:width .3s ease, background-color .3s ease;
}
.nvs-gauge-fill.warn{background:var(--orange)}
.nvs-gauge-fill.danger{background:var(--red)}
.nvs-gauge-detail{font-size:.7rem;color:var(--text-muted);text-align:right}

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
/* Toggle standard (44×24) — utilisé dans le modal programme.
   Le toggle "compact" du tableau (36×20) est surchargé ci-dessous pour
   garder un fond visible (le rond blanc doit laisser au moins ~3 px de
   piste de chaque côté, sinon il masque quasi totalement la couleur
   "actif" sur les petits écrans). */
.toggle{position:relative;width:44px;height:24px;flex:0 0 auto}
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

/* Variante compacte pour la colonne "Actif" du tableau Programmes.
   Par défaut, le toggle global est 44×24, mais en colonne de table on
   l'inline à 36×20 : avec les valeurs standards, le curseur rond (18px)
   ne laisse que 1 px de piste verticale et 3 px latéraux → le fond
   "actif" vert est quasi invisible. On adapte le rond à 14 px pour
   qu'il reste une bande colorée visible de chaque côté. */
.toggle.compact{width:36px;height:20px}
.toggle.compact .toggle-slider{border-radius:10px}
.toggle.compact .toggle-slider::before{
  height:14px;width:14px;left:3px;bottom:3px;
  box-shadow:0 1px 2px rgba(0,0,0,.25) /* léger relief pour le voir
    sur fond clair quand même */
}
.toggle.compact input:checked+.toggle-slider::before{
  transform:translateX(16px) /* 36 - 14 - 3*2 = 16 */
}

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

/* ── BOÎTE STATUS SYSTÈME ────────────────────────────────────
   Regroupe les métriques globales (uptime, T°, conso, santé)
   dans une zone visuellement distincte des cartes de vannes :
   fond plus contrasté, cartes internes alignées en grille,
   couleurs sémantiques par métrique. */
.status-box{
  background:linear-gradient(135deg, var(--surface) 0%, var(--surface2) 100%);
  border:1px solid var(--border);
  border-radius:var(--radius);
  padding:16px 18px 12px;
  margin-bottom:20px;
  box-shadow:0 1px 3px rgba(0,0,0,.25);
}
.status-header{
  display:flex;align-items:center;gap:10px;
  padding-bottom:12px;margin-bottom:14px;
  border-bottom:1px solid var(--border);
}
.status-header h3{
  font-size:.95rem;font-weight:600;color:var(--text);
  margin:0;flex:1;letter-spacing:.3px;
}
.status-time{
  font-size:.78rem;color:var(--text-muted);
  font-variant-numeric:tabular-nums;
}
.status-grid{
  display:grid;
  grid-template-columns:repeat(auto-fit, minmax(170px, 1fr));
  gap:10px;
  margin-bottom:14px;
}
.status-card{
  display:flex;align-items:center;gap:10px;
  background:var(--bg);
  border:1px solid var(--border);
  border-radius:8px;
  padding:10px 12px;
  min-height:62px;
}
.status-card-icon{
  width:36px;height:36px;border-radius:8px;
  display:flex;align-items:center;justify-content:center;
  flex-shrink:0;
}
.status-card-body{flex:1;min-width:0;}
.status-card-label{
  font-size:.68rem;color:var(--text-muted);
  text-transform:uppercase;letter-spacing:.5px;
  margin-bottom:2px;
}
.status-card-value{
  font-size:1rem;font-weight:600;color:var(--text);
  font-variant-numeric:tabular-nums;
  line-height:1.2;
  white-space:nowrap;overflow:hidden;text-overflow:ellipsis;
}
.status-card-sub{
  font-size:.72rem;color:var(--text-muted);
  margin-top:2px;
  font-variant-numeric:tabular-nums;
}
.status-conso{
  background:var(--bg);
  border:1px solid var(--border);
  border-radius:8px;
  padding:8px 10px;
}
.status-conso-header{
  display:flex;justify-content:space-between;align-items:center;
  font-size:.8rem;font-weight:600;color:var(--text);
  padding:4px 4px 8px;
  border-bottom:1px solid var(--border);
  margin-bottom:6px;
}
.status-conso-tbl{
  font-size:.78rem;
}
.status-conso-tbl th{
  font-weight:500;color:var(--text-muted);
  font-size:.7rem;text-transform:uppercase;letter-spacing:.4px;
  padding:6px 4px;
}
.status-conso-tbl td{
  padding:6px 4px;
  border-top:1px solid var(--border);
  font-variant-numeric:tabular-nums;
}
</style>
</head>
<body>
<div id="app">

<!-- HEADER -->
<header>
  <svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="#388bfd" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2z"/><path d="M12 6v6l4 2"/></svg>
  <span class="badge">Mon Beau jardin bien arrosé</span>
  <div style="margin-left:auto;display:flex;align-items:center;gap:18px">
    <!-- Badge MQTT (amélioration A) -->
    <div id="mqtt-status">
      <span id="mqtt-dot"></span>
      <span id="mqtt-label">MQTT: —</span>
    </div>
    <div id="ws-status">
      <span id="ws-dot"></span>
      <span id="ws-label">Déconnecté</span>
    </div>
  </div>
</header>

<!-- NAV -->
<nav>
  <button class="active" onclick="showPage('dashboard',this)">Dashboard</button>
  <button onclick="showPage('programmes',this)">Programmes</button>
  <button onclick="showPage('calendrier',this)">Calendrier</button>
  <button onclick="showPage('calibration',this)">Calibration</button>
  <button onclick="showPage('journal',this)">Journal</button>
  <button onclick="showPage('config',this)">Configuration</button>
  <button onclick="showPage('io',this)">Entr&eacute;es/Sorties</button>
</nav>

<main>

<!-- ══ PAGE DASHBOARD ══════════════════════════════════ -->
<div id="page-dashboard" class="page active">
  <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:18px">
    <div>
      <div style="font-size:1.1rem;font-weight:700">Tableau de bord</div>
    </div>
    <div style="display:flex;gap:8px">
      <button class="btn btn-ghost btn-sm" onclick="closeAll()">Tout fermer</button>
    </div>
  </div>

  <!-- ══ BOÎTE STATUS SYSTÈME ══════════════════════════════
       Distinction visuelle forte vs les cartes de vanne :
         • fond bleu-dim/var(--surface) au lieu de la grille blanche
         • cartes internes (uptime, T°, conso) avec icônes inline
         • tableau conso compact en bas
       Mise à jour automatique via handleStatus() / refreshConsumption(). -->
  <div class="status-box">
    <div class="status-header">
      <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="var(--blue)" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
        <circle cx="12" cy="12" r="10"/><polyline points="12 6 12 12 16 14"/>
      </svg>
      <h3>État du système</h3>
      <span class="status-time" id="status-time">—</span>
    </div>

    <div class="status-grid">
      <!-- Carte uptime -->
      <div class="status-card">
        <div class="status-card-icon" style="background:var(--blue-dim)">
          <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="var(--blue)" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
            <circle cx="12" cy="12" r="10"/><polyline points="12 6 12 12 16 14"/>
          </svg>
        </div>
        <div class="status-card-body">
          <div class="status-card-label">Uptime</div>
          <div class="status-card-value" id="uptime-label">—</div>
        </div>
      </div>

      <!-- Carte températures -->
      <div class="status-card">
        <div class="status-card-icon" style="background:var(--orange-dim)">
          <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="var(--orange)" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
            <path d="M14 14.76V3.5a2.5 2.5 0 0 0-5 0v11.26a4 4 0 1 0 5 0z"/>
          </svg>
        </div>
        <div class="status-card-body">
          <div class="status-card-label">Températures</div>
          <div class="status-card-value" id="temps-label">T1: -- °C</div>
          <div class="status-card-sub" id="tempsR-label">TRem: -- °C</div>
        </div>
      </div>

      <!-- Carte conso globale -->
      <div class="status-card">
        <div class="status-card-icon" style="background:var(--green-dim)">
          <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="var(--green)" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
            <path d="M12 2.69l5.66 5.66a8 8 0 1 1-11.31 0z"/>
          </svg>
        </div>
        <div class="status-card-body">
          <div class="status-card-label">Volume total</div>
          <div class="status-card-value" id="litres-label">— L</div>
          <div class="status-card-sub">
            <span id="pulse-label">—</span> pulses ·
            <span style="color:var(--blue);font-weight:600" id="flow-label">— L/min</span>
          </div>
        </div>
      </div>

      <!-- Carte mémoire / santé -->
      <div class="status-card">
        <div class="status-card-icon" style="background:var(--surface2)">
          <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="var(--text-muted)" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
            <rect x="4" y="4" width="16" height="16" rx="2"/>
            <rect x="9" y="9" width="6" height="6"/>
            <line x1="9" y1="1" x2="9" y2="4"/><line x1="15" y1="1" x2="15" y2="4"/>
            <line x1="9" y1="20" x2="9" y2="23"/><line x1="15" y1="20" x2="15" y2="23"/>
            <line x1="20" y1="9" x2="23" y2="9"/><line x1="20" y1="14" x2="23" y2="14"/>
            <line x1="1" y1="9" x2="4" y2="9"/><line x1="1" y1="14" x2="4" y2="14"/>
          </svg>
        </div>
        <div class="status-card-body">
          <div class="status-card-label">Santé ESP</div>
          <div class="status-card-value" id="heap-label">— KB</div>
          <div class="status-card-sub" id="wifi-label">WiFi —</div>
        </div>
      </div>
    </div>

    <!-- Tableau compact conso par vanne -->
    <div class="status-conso">
      <div class="status-conso-header">
        <span>💧 Consommation par vanne</span>
        <button class="btn btn-ghost btn-sm" onclick="refreshConsumption()" title="Actualiser">↻</button>
      </div>
      <table class="tbl status-conso-tbl">
        <thead>
          <tr>
            <th>Vanne</th>
            <th>Nom</th>
            <th style="text-align:right">Aujourd'hui</th>
            <th style="text-align:right">Total</th>
          </tr>
        </thead>
        <tbody id="status-cons-body">
          <tr><td colspan="4" style="text-align:center;color:var(--text-muted);padding:10px">—</td></tr>
        </tbody>
      </table>
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
      <div style="display:flex;gap:8px;flex-wrap:wrap;align-items:center">
        <button class="btn btn-ghost btn-sm" onclick="exportSchedules()">⬇ Export JSON</button>
        <button class="btn btn-blue btn-sm" onclick="document.getElementById('sched-import-file').click()">⬆ Import JSON</button>
        <input id="sched-import-file" type="file" accept="application/json,.json" style="display:none" onchange="importSchedulesFromFile(event)">
        <button class="btn btn-blue btn-sm" onclick="openSchedModal()">+ Ajouter</button>
      </div>
    </div>
    <div style="overflow-x: auto;">
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
</div>

<!-- ══ PAGE CALIBRATION ═════════════════════════════════ -->
<div id="page-calibration" class="page">
  <div class="card">
    <div class="card-header">
      <h2>Calibration du débitmètre</h2>
      <span id="calib-phase-badge" class="badge" style="background:var(--surface2);color:var(--text-muted);font-size:.75rem">inactif</span>
    </div>

    <div style="padding:8px 4px 16px;color:var(--text-muted);font-size:.86rem;line-height:1.5">
      La calibration mesure le débit de chaque vanne <strong>une par une</strong>, en l'ouvrant
      seule pendant la durée choisie. Le coefficient <code>flowCoeff</code> (en pulses/seconde)
      est calculé automatiquement et utilisé pour répartir les pulses globaux entre vannes
      ouvertes simultanément. <strong>Fermez toutes les vannes avant de lancer</strong> (sécurité
      intégrée côté firmware : démarrage refusé si une vanne est ouverte).
    </div>

    <!-- Formulaire de lancement -->
    <div class="config-section" id="calib-launch-form">
      <h3>Lancer une calibration</h3>
      <div class="config-row">
        <div>
          <label>Durée par vanne (secondes)</label>
          <input type="number" id="calib-duration" value="60" min="5" max="600"/>
          <div style="font-size:.75rem;color:var(--text-muted);margin-top:4px">
            Recommandé : 30-120 s. Plus c'est long, plus la mesure est précise.
          </div>
        </div>
      </div>
      <div style="margin-top:14px;display:flex;gap:8px;align-items:center">
        <button id="calib-start-btn" class="btn btn-green" onclick="startCalibration()">
          ▶ Démarrer la calibration
        </button>
        <button id="calib-abort-btn" class="btn btn-red" onclick="abortCalibration()" style="display:none">
          ■ Annuler
        </button>
        <span id="calib-msg" style="font-size:.85rem;color:var(--text-muted)"></span>
      </div>
    </div>

    <!-- Progression en direct -->
    <div class="config-section" id="calib-progress" style="display:none">
      <h3>Progression</h3>
      <div style="display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-bottom:14px">
        <div>
          <div style="font-size:.78rem;color:var(--text-muted);text-transform:uppercase;letter-spacing:.5px">Vanne en cours</div>
          <div style="font-size:1.5rem;font-weight:700" id="calib-current-valve">—</div>
        </div>
        <div>
          <div style="font-size:.78rem;color:var(--text-muted);text-transform:uppercase;letter-spacing:.5px">Temps restant</div>
          <div style="font-size:1.5rem;font-weight:700;color:var(--blue)" id="calib-remaining">— s</div>
        </div>
      </div>
      <div style="background:var(--bg);border-radius:8px;height:8px;overflow:hidden;margin-bottom:6px">
        <div id="calib-bar" style="height:100%;background:var(--green);width:0%;transition:width 0.5s linear"></div>
      </div>
      <div style="font-size:.78rem;color:var(--text-muted);text-align:right">
        Vanne <span id="calib-progress-valve">0</span> / <span id="calib-progress-total">0</span>
      </div>
    </div>

    <!-- Résultats -->
    <div class="config-section">
      <h3>Coefficients actuels</h3>
      <div style="font-size:.78rem;color:var(--text-muted);margin-bottom:10px">
        Mis à jour automatiquement après une calibration. Le firmware les utilise pour
        pondérer la répartition des pulses entre vannes ouvertes simultanément.
      </div>
      <table class="tbl" style="max-width:500px">
        <thead>
          <tr>
            <th>Vanne</th>
            <th style="text-align:right">flowCoeff (pulses/s)</th>
            <th style="text-align:right">Équiv. L/min</th>
          </tr>
        </thead>
        <tbody id="calib-coeff-body">
          <tr><td colspan="3" style="text-align:center;color:var(--text-muted);padding:18px">Chargement…</td></tr>
        </tbody>
      </table>
    </div>

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

<!-- ══ PAGE E/S ═════════════════════════════════════ -->
<div id="page-io" class="page">
  <div class="card">
    <div class="card-header">
      <h2>Entr&eacute;es / Sorties Mat&eacute;rielles</h2>
    </div>
    <div style="overflow-x: auto; padding: 16px;">
      <table class="tbl" style="font-family: monospace; text-align: center; width:100%; max-width:600px; margin:0 auto">
        <thead>
          <tr>
            <th style="text-align:left; width:60px">Pin</th>
            <th>0</th><th>1</th><th>2</th><th>3</th><th>4</th><th>5</th><th>6</th><th>7</th>
          </tr>
        </thead>
        <tbody id="io-body">
          <tr><td colspan="9" style="text-align:center;color:var(--text-muted);padding:24px">En attente des donn&eacute;es...</td></tr>
        </tbody>
      </table>
      <div style="margin-top: 24px; font-size: 0.85rem; color: var(--text-muted); max-width:600px; margin-left:auto; margin-right:auto; padding:12px; background:var(--surface2); border-radius:6px">
        <strong style="color:var(--text)">L&eacute;gende :</strong><br>
        <div style="margin-top:6px"><strong>oPD</strong> : Sorties Puissance (0-3: Vannes, 4-7: Sorties auxiliaires)</div>
        <div><strong>oPA</strong> : Sorties LEDs visualisation (0-3: État des vannes)</div>
        <div><strong>In</strong> : Entr&eacute;es for&ccedil;age manuel (Boutons poussoirs)</div>
      </div>
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
            🇫🇷 France (Paris) : <code style="color:var(--blue);cursor:pointer" onclick="document.getElementById('cfg-tz-posix').value=this.textContent">CET-1CEST,M3.5.0,M10.5.0/3</code>
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

</main>
</div><!-- #app -->

<!-- ══ MODAL FORÇAGE ══════════════════════════════════ -->
<div class="modal-overlay" id="force-modal">
  <div class="modal">
    <h3>Ouvrir la vanne <span id="force-modal-name"></span></h3>
    <label>Durée (secondes)</label>
    <input type="number" id="force-duration" value="1800" min="1" max="86400"/>
    <div class="modal-actions">
      <button class="btn btn-ghost" onclick="closeModal('force-modal')">Annuler</button>
      <button class="btn btn-green" onclick="confirmForce()">Ouvrir</button>
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
        <label>Mode calendrier</label>
        <select id="sched-calmode" onchange="updateSchedCalMode()">
          <option value="0">Hebdomadaire</option>
          <option value="1">Intervalle</option>
          <option value="2">Saisonnier</option>
        </select>
      </div>
      <div class="form-group" style="grid-column: 1 / -1">
        <label>Unité de la durée</label>
        <div style="display:flex;gap:6px;flex-wrap:wrap">
          <button type="button" class="day-btn sel" id="sched-unit-sec" onclick="setSchedUnit('sec')">⏱ Durée (s)</button>
          <button type="button" class="day-btn" id="sched-unit-l"   onclick="setSchedUnit('L')">💧 Volume (L)</button>
        </div>
        <div id="sched-dur-row" class="form-group" style="margin-top:8px">
          <label>Durée (s)</label>
          <input type="number" id="sched-dur" value="900" min="30" oninput="syncSchedFromSec()"/>
        </div>
        <div id="sched-vol-row" class="form-group" style="margin-top:8px;display:none">
          <label>Volume (L)</label>
          <input type="number" id="sched-vol" value="20" min="0.1" step="0.1" oninput="syncSchedFromVol()"/>
          <div id="sched-vol-hint" style="font-size:.72rem;color:var(--text-muted);margin-top:4px">
            Conversion basée sur le coefficient de calibration de la vanne.
          </div>
        </div>
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

// CORRECTIF (bugs 2 & 3) : fallback aligné sur le VANNE_COUNT réel du
// firmware (5 vannes actuellement), au lieu de 8. Utilisé uniquement
// quand les données serveur (valves[] ou sysConfig.valveNames) ne sont
// pas encore disponibles (ex: avant la première réponse status/config).
// Si le nombre de vannes physiques change côté firmware, mettre à jour
// cette constante en conséquence (ou, mieux, ne plus en avoir besoin du
// tout le jour où /api/config est garanti répondre avant tout rendu).
const VALVE_COUNT_FALLBACK = 5;

// Verrou anti-rebuild pendant l'édition d'un programme.
// Tant que le modal "sched-modal" est ouvert, on ne reconstruit pas
// le <select id="sched-valve"> au gré des messages STATUS périodiques
// (WebSocket ~1×/s). Avant ce correctif, buildSchedValveSelect() était
// appelée à chaque handleStatus() et pouvait perdre/réinitialiser la
// sélection de vanne en cours d'édition, donnant l'impression que
// "la vanne 1 revient toute seule".
let schedModalOpen = false;

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

// CORRECTIF (bug 5) : day-of-year désormais calculé en UTC pur, comme dans
// renderCalendar(), au lieu d'un calcul basé sur l'heure locale du
// navigateur. Avant ce correctif, les deux fonctions pouvaient diverger
// d'un jour autour des changements d'heure été/hiver, ce qui désynchronisait
// le badge affiché sur le calendrier et le texte "Prochain événement" sur
// la carte de la vanne pour un même programme en mode intervalle.
function ydayUTC(dt){
  return Math.floor((Date.UTC(dt.getFullYear(), dt.getMonth(), dt.getDate()) - Date.UTC(dt.getFullYear(),0,1)) / 86400000);
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
    // CORRECTIF (bug 5) : calcul en UTC, cohérent avec renderCalendar()
    const yday = ydayUTC(date);
    const startMonth = s.intervalStartMonth || 1;
    const startDay   = s.intervalStartDay   || 1;
    const startDate = new Date(y, startMonth-1, startDay);
    const startYday = ydayUTC(startDate);
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
  const seconds = Math.floor((ms%60000)/1000);
  let text='';
  if(days>0) text = `dans ${days} j${days>1?'s':''} à ${String(best.dt.getHours()).padStart(2,'0')}h${String(best.dt.getMinutes()).padStart(2,'0')}`;
  else if(hours>0) text = `dans ${hours} h ${minutes} m`;
  else if(minutes>0) text = `dans ${minutes} m ${seconds} s`;
  else text = `dans ${seconds} s`;
  return {text,dt:best.dt,sched:best.sched};
}

function showPage(id, btn) {
  document.querySelectorAll('.page').forEach(p=>p.classList.remove('active'));
  document.querySelectorAll('nav button').forEach(b=>b.classList.remove('active'));
  document.getElementById('page-'+id).classList.add('active');
  if(btn) btn.classList.add('active');
  if(id==='programmes') renderSchedules();
  if(id==='calendrier') loadSchedules().then(()=>renderCalendar());
  if(id==='calibration') refreshCalibration();
  if(id==='journal') loadLog();
  if(id==='config') loadConfig();
}

function closeModal(id) {
  document.getElementById(id).classList.remove('open');
  if(id==='sched-modal') schedModalOpen = false;
}

// ══════════════════════════════════════════════════════════
// CALIBRATION DÉBITMÈTRE
// ══════════════════════════════════════════════════════════
function startCalibration(){
  const dur = parseInt(document.getElementById('calib-duration').value) || 60;
  const msg = document.getElementById('calib-msg');
  msg.textContent = 'Démarrage…';
  msg.style.color = 'var(--text-muted)';
  api('POST','/api/calibration/start',{durationSec: dur}).then(d=>{
    if(d && d.ok){
      msg.textContent = '';
      document.getElementById('calib-launch-form').style.display = 'none';
      document.getElementById('calib-progress').style.display = 'block';
      document.getElementById('calib-phase-badge').textContent = 'EN COURS';
      document.getElementById('calib-phase-badge').style.background = 'var(--orange-dim)';
      document.getElementById('calib-phase-badge').style.color = 'var(--orange)';
      // Polling toutes les 2 s pour suivre la progression
      if(window._calibPoll) clearInterval(window._calibPoll);
      window._calibPoll = setInterval(refreshCalibration, 2000);
      refreshCalibration();
    } else {
      msg.textContent = '✗ Échec : ' + (d && d.reason ? d.reason : 'inconnu');
      msg.style.color = 'var(--red)';
    }
  }).catch(err => {
    msg.textContent = '✗ Erreur réseau : ' + err;
    msg.style.color = 'var(--red)';
  });
}

function abortCalibration(){
  if(!confirm('Annuler la calibration en cours ?')) return;
  api('POST','/api/calibration/abort',{}).then(()=>{
    refreshCalibration();
  });
}

function refreshCalibration(){
  api('GET','/api/calibration/status').then(d=>{
    if(!d) return;
    const phase   = d.phase || 'idle';
    const phaseFr = {idle:'inactif', running:'EN COURS', done:'terminé', aborted:'annulé', failed:'échoué'}[phase] || phase;
    const badge = document.getElementById('calib-phase-badge');
    badge.textContent = phaseFr;
    const colors = {
      idle:     {bg:'var(--surface2)', fg:'var(--text-muted)'},
      running:  {bg:'var(--orange-dim)',fg:'var(--orange)'},
      done:     {bg:'var(--green-dim)', fg:'var(--green)'},
      aborted:  {bg:'var(--red-dim)',   fg:'var(--red)'},
      failed:   {bg:'var(--red-dim)',   fg:'var(--red)'},
    };
    const c = colors[phase] || colors.idle;
    badge.style.background = c.bg;
    badge.style.color      = c.fg;

    // Affichage progression si en cours
    const progBox = document.getElementById('calib-progress');
    const launchBox = document.getElementById('calib-launch-form');
    if(phase === 'running'){
      progBox.style.display = 'block';
      launchBox.style.display = 'none';
      const v = (d.currentValve !== undefined ? d.currentValve : -1);
      const total = (window.VANNE_COUNT_FALLBACK || 5);
      document.getElementById('calib-current-valve').textContent = (v >= 0 ? 'V'+v : '—');
      const remain = d.remainingSec !== undefined ? d.remainingSec : 0;
      document.getElementById('calib-remaining').textContent = remain + ' s';
      document.getElementById('calib-progress-valve').textContent = v;
      document.getElementById('calib-progress-total').textContent  = total;
      // Barre de progression : % du cycle en cours
      const durationSec = d.durationSec || 60;
      const pct = Math.max(0, Math.min(100, ((durationSec - remain) / durationSec) * 100));
      document.getElementById('calib-bar').style.width = pct.toFixed(1) + '%';
    } else {
      progBox.style.display = 'none';
      launchBox.style.display = 'block';
      if(window._calibPoll){ clearInterval(window._calibPoll); window._calibPoll = null; }
    }

    // Tableau des coefficients — on lit depuis /api/consumption si dispo,
    // sinon on prend d. flowCoeffs s'il est dans le payload.
    // pulsesPerLitre est désormais fourni par le firmware dans le même
    // payload (cf. calibStatusJson() côté C++) pour éviter tout
    // désynchronisation entre la constante embarquée et la valeur utilisée
    // par l'UI. On retombe sur la valeur par défaut (741.2, alignée sur
    // PULSES_PER_LITRE dans Globals.h) si jamais le firmware ne la fournit
    // pas (rétro-compat).
    const tbody = document.getElementById('calib-coeff-body');
    if(d.flowCoeffs && Array.isArray(d.flowCoeffs)){
      // Mémorise les coeffs pour que le modal "Nouveau programme" puisse
      // convertir une saisie en litres en secondes (sans avoir à faire un
      // nouvel appel API). On stocke une copie pour éviter toute
      // référence croisée avec le DOM.
      window.__flowCoeffs = d.flowCoeffs.map(c => Number(c) || 0);
      const ppl = (d.pulsesPerLitre && d.pulsesPerLitre > 0)
                  ? d.pulsesPerLitre
                  : (window.PULSES_PER_LITRE || 741.2);
      // Mémorise pour les rafraîchissements successifs (debug / autres pages).
      window.PULSES_PER_LITRE = ppl;
      tbody.innerHTML = d.flowCoeffs.map((c,i)=>{
        const lpm = (c > 0 && ppl > 0) ? (c * 60 / ppl).toFixed(2) : '—';
        const cdisp = c > 0 ? c.toFixed(3) : '<span style="color:var(--text-muted)">non calibré</span>';
        return `<tr>
          <td><strong>V${i}</strong></td>
          <td style="text-align:right;font-variant-numeric:tabular-nums">${cdisp}</td>
          <td style="text-align:right;color:var(--blue);font-weight:600">${lpm}</td>
        </tr>`;
      }).join('');
      // Si le modal programme est ouvert, on doit aussi actualiser
      // l'état "enabled" du toggle Volume pour la vanne courante
      // (au cas où une nouvelle calibration vient de finir).
      if(schedModalOpen) refreshSchedUnitAvailability();
    }
  });
}
// Constantes exposées pour refreshCalibration() — définies plus haut
// dans le state (VALVE_COUNT_FALLBACK = 5). On garde une référence
// directe pour éviter un ReferenceError.
window.VALNE_COUNT_FALLBACK = 5;

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
  // Show uptime and current time (use data.time if provided)
  let timeStr = '—';
  if(data.time){
    const d = new Date(data.time * 1000);
    timeStr = d.toLocaleString('fr-FR',{hour: '2-digit', minute: '2-digit', second: '2-digit'});
  }
  // Bandeau "État du système" — nouvelle UI (cards au lieu d'un texte brut)
  const upEl  = document.getElementById('uptime-label');
  const timeEl = document.getElementById('status-time');
  if (upEl)  upEl.textContent  = fmtSec(data.uptime || 0);
  if (timeEl) timeEl.textContent = timeStr;

  if (document.getElementById('temps-label')) {
    document.getElementById('temps-label').textContent =
      `T1: ${data.temp1 !== undefined ? Number(data.temp1).toFixed(2) : '--'} °C`;
  }
  const tempR = document.getElementById('tempsR-label');
  if (tempR) {
    tempR.textContent = `TRem: ${data.tempR !== undefined ? Number(data.tempR).toFixed(2) : '--'} °C`;
  }
  const litresEl = document.getElementById('litres-label');
  if (litresEl) {
    litresEl.textContent = (data.litres !== undefined ? Number(data.litres).toFixed(2) : '--') + ' L';
  }
  const pulseEl = document.getElementById('pulse-label');
  if (pulseEl) {
    pulseEl.textContent = (data.pulses !== undefined ? data.pulses : '--');
  }
  if (document.getElementById('flow-label')) {
    const flow = data.flow_lpm !== undefined ? Number(data.flow_lpm).toFixed(2) : '0.00';
    document.getElementById('flow-label').textContent = flow + ' L/min';
  }
  // Carte "Santé ESP" : heap libre + état WiFi
  const heapEl = document.getElementById('heap-label');
  if (heapEl) {
    const kb = (data.heap !== undefined ? Math.round(data.heap/1024) : '--');
    heapEl.textContent = kb + ' KB libres';
  }
  const wifiEl = document.getElementById('wifi-label');
  if (wifiEl) {
    // data.heap est exposé par le firmware mais pas l'état WiFi directement ;
    // on le déduit du badge WebSocket déjà mis à jour plus haut.
    const wsDot = document.getElementById('ws-dot');
    wifiEl.textContent = wsDot && wsDot.className === 'ok' ? 'WiFi connecté' : 'WiFi —';
  }
  // Badge MQTT (amélioration A) — reflète mqttConnected envoyé par le firmware
  const mqttDot = document.getElementById('mqtt-dot');
  const mqttLabel = document.getElementById('mqtt-label');
  if (mqttDot && mqttLabel && data.mqttConnected !== undefined) {
    mqttDot.className = data.mqttConnected ? 'ok' : 'off';
    mqttLabel.textContent = 'MQTT: ' + (data.mqttConnected ? 'connecté' : 'déconnecté');
  }
  // Jauge NVS — on l'actualise à chaque STATUS (≈1×/s par le WebSocket)
  // pour que l'utilisateur suive l'évolution du remplissage en direct
  // sans avoir à cliquer sur "Actualiser". On ne fait la mise à jour
  // DOM QUE si la page Config est visible, pour ne pas manipuler
  // inutilement des éléments cachés.
  if (data.nvs && document.getElementById('page-config') &&
      document.getElementById('page-config').classList.contains('active')) {
    const pctEl = document.getElementById('nvs-pct');
    const fill  = document.getElementById('nvs-fill');
    const det   = document.getElementById('nvs-detail');
    if(pctEl && fill && det){
      const pct = data.nvs.usedPct || 0;
      pctEl.textContent = pct + '%';
      fill.style.width  = pct + '%';
      fill.classList.remove('warn','danger');
      if(pct >= 90)      fill.classList.add('danger');
      else if(pct >= 70) fill.classList.add('warn');
      det.textContent = `${data.nvs.used} / ${data.nvs.total} entrées utilisées (${data.nvs.free} libres)`;
    }
  }
  // Ne reconstruit le select des vannes du modal programme QUE si ce
  // dernier n'est pas en cours d'édition — sinon la sélection de
  // l'utilisateur est préservée jusqu'à la fermeture du modal.
  if(!schedModalOpen) buildSchedValveSelect();
  renderValveCards();
  
  if (data.ioOut && data.ioLed && data.ioIn) {
    const buildRow = (label, arr) => {
      let r = `<tr><td style="text-align:left; font-weight:bold">${label}</td>`;
      for(let i=0; i<8; i++) {
        const val = arr[i];
        if(val < 0) r += `<td style="color:var(--text-muted)">--</td>`;
        else r += `<td><span style="display:inline-block;width:24px;height:24px;line-height:24px;border-radius:4px;background:${val===1?'var(--green)':'var(--surface2)'};color:${val===1?'#fff':''}">${val}</span></td>`;
      }
      return r + `</tr>`;
    };
    const html = buildRow('oPD', data.ioOut) + buildRow('oPA', data.ioLed) + buildRow('In', data.ioIn);
    const tbody = document.getElementById('io-body');
    if(tbody) tbody.innerHTML = html;
  }
  // Pulse info via WS (if present)
  if(data.pulses !== undefined){
    const p = data.pulses || 0;
    const l = data.litres || 0;
    const elP = document.getElementById('pulse-count'); if(elP) elP.textContent = p;
    const elL = document.getElementById('pulse-litres'); if(elL) elL.textContent = l.toFixed(2);
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
    // Débit calibré (L/min) pour cette vanne, affiché à côté du badge
    // d'état. Calcul identique à celui du tableau de calibration
    // (refreshCalibration) : flowCoeff * 60 / pulsesPerLitre. Mis en
    // cache par refreshCalibration() dans window.__flowCoeffs et
    // window.PULSES_PER_LITRE. Si la vanne n'a jamais été calibrée,
    // on affiche un libellé discret "non calibré" plutôt qu'une valeur
    // inventée (cohérent avec le reste de l'UI).
    // NOTE: doit être déclaré AVANT badgeHtml qui le référence —
    // sinon "can't access lexical declaration ... before initialization".
    const _fc = (window.__flowCoeffs && window.__flowCoeffs[i]) ? Number(window.__flowCoeffs[i]) : 0;
    const _ppl = (window.PULSES_PER_LITRE && window.PULSES_PER_LITRE > 0) ? window.PULSES_PER_LITRE : 0;
    const lpmHtml = (_fc > 0 && _ppl > 0)
      ? `<span class="vc-flow" title="Débit calibré de la vanne">💧 ${(_fc * 60 / _ppl).toFixed(2)} L/min</span>`
      : `<span class="vc-flow uncal" title="Cette vanne n'a pas encore été calibrée">— non calibré</span>`;
    // Le badge L/min est accolé au badge d'état. Pour les vannes
    // fermées : affichage "à côté du status fermé" (demande explicite).
    // Pour les vannes ouvertes/forcées : idem, pour rappeler la capacité
    // hydraulique de la ligne (utile à la lecture des litres restants).
    const badgeHtml = (isForced
      ? `<span class="vc-badge badge-forced">⚡ Forcée (${v.source})</span>`
      : isOpen
        ? `<span class="vc-badge badge-open">● Ouverte (${v.source})</span>`
        : `<span class="vc-badge badge-closed">◌ Fermée</span>`) + lpmHtml;
    const nextEv = getNextEventForValve(i);
    const nextHtml = nextEv ? ( (nextEv.sched && nextEv.sched.name ? (nextEv.sched.name+' — ') : '') + nextEv.text ) : '—';
    // Volume restant = flowCoeff (pulses/s) × remainingSec / pulsesPerLitre
    // flowCoeffs est mis en cache par refreshCalibration() dans window.__flowCoeffs
    // (et pulsesPerLitre dans window.PULSES_PER_LITRE). Tant qu'aucune
    // calibration n'a été faite, flowCoeff[i] = 0 (non significatif) — on
    // NE montre PAS d'estimation trompeuse, on affiche "— L" jusqu'à la
    // première calibration (cohérent avec la ligne "non calibré" du
    // tableau de calibration). Cf. raisonnement dans refreshCalibration().
    let remainingLitresHtml = '';
    if(isOpen){
      const fc = (window.__flowCoeffs && window.__flowCoeffs[i]) ? Number(window.__flowCoeffs[i]) : 0;
      const ppl = (window.PULSES_PER_LITRE && window.PULSES_PER_LITRE > 0) ? window.PULSES_PER_LITRE : 0;
      if(fc > 0 && ppl > 0){
        const l = (fc * (v.remainingSec || 0)) / ppl;
        remainingLitresHtml = `<div class="vc-remaining-l"><span class="vc-remaining-l-label">💧 Restant</span>${l.toFixed(2)} L</div>`;
      } else {
        // Vanne ouverte mais pas (encore) calibrée : on affiche un libellé
        // discret plutôt qu'une valeur inventée, pour rester honnête avec
        // l'utilisateur. "calibrer" est cliquable vers la page calibration.
        remainingLitresHtml = `<div class="vc-remaining-l" style="color:var(--text-muted);font-weight:500"><span class="vc-remaining-l-label">💧 Restant</span>— (non calibré)</div>`;
      }
    }
    return `
    <div class="valve-card ${cardCls}" id="vc-${i}" style="--vcol:var(--vcol${i}); --vcol-fg:#fff">
      <div class="vc-header">
        <span class="vc-name">${v.name||'V'+i}</span>
        <span class="vc-num" style="background:var(--vcol);color:var(--vcol-fg)">V${i}</span>
      </div>
      ${badgeHtml}
      <div class="vc-remaining">${isOpen ? fmtSec(v.remainingSec) : '—'}</div>
      ${remainingLitresHtml}
      <div class="vc-meta">Dernier démarrage: ${fmtEpoch(v.openedAt)}<br>
        Total cumulé: ${fmtSec(v.totalOpenSec)}</div>
      <div style="margin: 4px 0 10px; display:flex; gap:8px; flex-wrap:wrap; align-items:center;">
        <span style="font-size:.75rem;padding:4px 10px;border-radius:20px;background:var(--blue-dim);color:var(--blue);display:flex;align-items:center;gap:6px">
          💧 Aujourd'hui: <strong>${(v.litresToday||0).toFixed(2)} L</strong>
          <button class="btn" style="padding:0 4px;height:18px;font-size:0.6rem;font-weight:bold;background:var(--blue);color:#fff;border:none;border-radius:10px" onclick="resetValveCons(${i}, 'today')" title="Remettre le jour à zéro">RAZ</button>
        </span>
        <span style="font-size:.75rem;padding:4px 10px;border-radius:20px;background:var(--surface2);color:var(--text-muted);display:flex;align-items:center;gap:6px">
          Total: <strong style="color:var(--text)">${(v.litresTotal||0).toFixed(2)} L</strong>
          <button class="btn" style="padding:0 4px;height:18px;font-size:0.6rem;font-weight:bold;background:var(--border);color:var(--text);border:none;border-radius:10px" onclick="resetValveCons(${i}, 'total')" title="Remettre le total à zéro">RAZ</button>
        </span>
      </div>
      <div style="margin: 8px 0 14px; padding: 8px 12px; background: var(--surface2); border-radius: 6px; border-left: 3px solid var(--blue); font-size: .8rem;">
        <span style="color:var(--text-muted);font-size:.75rem;display:block;margin-bottom:2px;text-transform:uppercase;letter-spacing:0.5px">Prochain événement</span>
        <strong style="color:var(--text);">${nextHtml}</strong>
      </div>
      <div class="vc-actions">
        <button class="btn btn-green btn-sm" onclick="showForceModal(${i})">Ouvrir</button>
        <button class="btn btn-red btn-sm" onclick="closeValve(${i})">Fermer</button>
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
function resetValveCons(idx, type) {
  const msg = (type === 'today') 
    ? 'Remettre à zéro la consommation d\'aujoudui pour cette vanne ?'
    : 'Remettre à zéro la consommation TOTALE (et l\'historique) de cette vanne ?';
  if(!confirm(msg)) return;
  api('POST','/api/valve/reset_cons',{valve:idx, type:type}).then(()=>{
    requestStatus();
    if (document.getElementById('page-config').classList.contains('active')) refreshConsumption();
  });
}
// CORRECTIF (bug 4) : "Tout fermer" coupe toutes les vannes y compris
// celles forcées manuellement ou en cours de programme — action destructive
// sans garde-fou auparavant. On ajoute une confirmation, cohérente avec
// le RAZ du compteur d'impulsions qui en a déjà une.
function closeAll() {
  if(!confirm('Fermer TOUTES les vannes maintenant (y compris celles forcées ou en programme) ?')) return;
  api('POST','/api/valve/closeall').then(()=>requestStatus());
}
function showForceModal(idx) {
  forceValveIdx = idx;
  const name = (valves[idx]&&valves[idx].name) || 'V'+idx;
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
function exportSchedules() {
  // Calcul du volume (en L) pour chaque programme, à partir de la
  // durée stockée côté firmware et du coefficient de calibration de
  // la vanne concernée :
  //   litres = durationSec * flowCoeff[i] / pulsesPerLitre
  // On inclut cette valeur dans l'export pour deux raisons :
  //   1) Permettre à l'utilisateur de voir/retoucher une saisie qui
  //      était à l'origine en litres, même si la calibration a changé
  //      depuis (round-trip UI : édition en L → envoi en s au firmware
  //      → on retrouve le L à la réimport).
  //   2) Pouvoir réimporter le JSON sur une vanne AYANT UN COEFF
  //      DIFFÉRENT en gardant la même "intention" en litres (on
  //      recalcule alors durationSec à la volée à l'import).
  // Si la vanne n'est pas calibrée (flowCoeff = 0), on stocke `null`
  // et l'import se contente de conserver durationSec tel quel.
  const fcArr = window.__flowCoeffs || [];
  const ppl = (window.PULSES_PER_LITRE && window.PULSES_PER_LITRE > 0) ? window.PULSES_PER_LITRE : 0;
  const payload = {
    exportedAt: new Date().toISOString(),
    version: 1,
    pulsesPerLitre: ppl || null,
    schedules: (schedules || []).map(s => {
      if (!s) return null;
      const out = {...s};
      if (ppl > 0) {
        const fc = (fcArr[s.valve] !== undefined) ? Number(fcArr[s.valve]) : 0;
        if (fc > 0) {
          // Arrondi à 2 décimales pour la lisibilité du JSON (la
          // précision sub-millilitre n'a pas de sens physique sur un
          // arrosage).
          out.litres = Math.round(((s.durationSec * fc) / ppl) * 100) / 100;
        } else {
          out.litres = null;
        }
      } else {
        out.litres = null;
      }
      return out;
    }).filter(Boolean)
  };
  const blob = new Blob([JSON.stringify(payload, null, 2)], {type:'application/json'});
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = `programmes-${new Date().toISOString().slice(0,19).replace(/:/g,'-')}.json`;
  document.body.appendChild(a);
  a.click();
  a.remove();
  URL.revokeObjectURL(url);
}

function importSchedulesFromFile(evt) {
  const file = evt && evt.target && evt.target.files && evt.target.files[0];
  if(!file) return;
  const reader = new FileReader();
  reader.onload = () => {
    try {
      const data = JSON.parse(reader.result);
      const imported = Array.isArray(data) ? data : (data && Array.isArray(data.schedules) ? data.schedules : null);
      if(!imported) {
        alert('Fichier JSON invalide : aucune liste de programmes trouvée.');
        return;
      }
      // Re-évaluation des durées : si l'entrée importée porte un
      // champ `litres` ET que la vanne de destination a un flowCoeff
      // connu, on RECALCULE durationSec à partir des litres et du
      // coeff de la vanne cible. C'est ce qui permet de garder
      // l'intention en litres de l'utilisateur même quand on importe
      // sur une autre vanne (coeff différent). Si pas de litres, ou
      // pas de coeff, on garde durationSec tel quel (rétro-compat).
      // On utilise d'abord le pulsesPerLitre courant, puis en
      // repli celui stocké dans l'export lui-même (pour qu'un
      // export fait sur un autre appareil / firmware reste importable
      // même si les constantes diffèrent — l'important est d'avoir
      // un pulsesPerLitre cohérent avec l'export).
      const fcArr = window.__flowCoeffs || [];
      const ppl   = (window.PULSES_PER_LITRE && window.PULSES_PER_LITRE > 0)
                    ? window.PULSES_PER_LITRE
                    : (data && data.pulsesPerLitre ? Number(data.pulsesPerLitre) : 0);
      const normalized = imported.map(s => {
        if (!s) return s;
        const o = {...s};
        if (o.litres !== undefined && o.litres !== null && Number(o.litres) > 0 && ppl > 0) {
          const fc = (fcArr[o.valve] !== undefined) ? Number(fcArr[o.valve]) : 0;
          if (fc > 0) {
            o.durationSec = Math.max(1, Math.round((Number(o.litres) * ppl) / fc));
          }
        }
        return o;
      });
      if(!confirm('Remplacer tous les programmes actuels par le contenu du fichier sélectionné ?')) {
        return;
      }
      api('POST','/api/schedules/import',{schedules: normalized}).then(d=>{
        if(d && d.ok){
          loadSchedules();
          alert('Import des programmes terminé.');
        } else {
          alert('Échec de l\'import des programmes.');
        }
      });
    } catch (err) {
      alert('Fichier JSON invalide : ' + err.message);
    }
  };
  reader.readAsText(file);
  evt.target.value = '';
}

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
  // Seuls les programmes actifs (ou en cours d'utilisation) ont un sens à
  // afficher comme "ligne de programme". Le backend renvoie TOUJOURS les
  // VANNE_COUNT × MAX_PROGRAMS slots (actifs ou non) afin que le frontend
  // puisse retrouver une ligne par sa position (valve, schedIdx). On filtre
  // ici l'affichage sur les slots qui ont réellement été configurés au moins
  // une fois (nom non vide, ou actif, ou horaire différent par défaut) pour
  // éviter d'afficher des dizaines de lignes vides "Lun Mar Mer Jeu Ven".
  // NOTE (bug 6, clarifié) : la valeur par défaut weekDays=63 (0b0111111)
  // correspond à 6 jours actifs (Lun à Sam), pas "Lun-Ven" — cohérent avec
  // le commentaire corrigé côté firmware (Schedule::weekDays).
  const meaningful = schedules.filter(s => {
    if (!s) return false;
    if (s.active) return true;
    if (s.name && s.name.length > 0) return true;
    if (s.hour !== 6 || s.minute !== 0 || s.durationSec !== 900) return true;
    if (s.calMode !== 0 || s.weekDays !== 63) return true;
    return false;
  });
  if(!meaningful.length){
    tbody.innerHTML = '<tr><td colspan="8" style="text-align:center;color:var(--text-muted);padding:24px">Aucun programme</td></tr>';
    return;
  }
  // Tri stable par vanne puis par heure, pour un affichage prévisible qui
  // ne "saute" jamais : une ligne reste à une position cohérente entre deux
  // rendus tant que ses données (vanne/heure) ne changent pas vraiment.
  const sorted = [...meaningful].sort((a,b)=>{
    if(a.valve !== b.valve) return a.valve - b.valve;
    const ta = (a.hour||0)*60 + (a.minute||0);
    const tb = (b.hour||0)*60 + (b.minute||0);
    return ta - tb;
  });
  const CAL_MODES = ['Hebdo','Intervalle','Saison'];
  let rows = '';
  for(const s of sorted){
    // Retrouver l'index dans le tableau `schedules` plat d'origine pour
    // editSched(), qui a besoin de l'objet exact (avec valve/schedIdx).
    const flatIdx = schedules.indexOf(s);
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
        <label class="toggle compact" title="${s.active?'Programme actif':'Programme inactif'}">
          <input type="checkbox" ${s.active?'checked':''} onchange="toggleSched(${s.valve},${s.schedIdx},this.checked)">
          <span class="toggle-slider"></span>
        </label>
      </td>
      <td>
        <button class="btn btn-ghost btn-sm" onclick="editSched(${flatIdx})">Éditer</button>
        <button class="btn btn-ghost btn-sm" onclick="dupSched(${flatIdx})" title="Dupliquer ce programme sur une vanne" style="margin-left:4px">Copier</button>
        <button class="btn btn-red btn-sm" onclick="deleteSched(${s.valve},${s.schedIdx})" style="margin-left:4px">Suppr.</button>
      </td>
    </tr>`;
  }
  tbody.innerHTML = rows;
}

function openSchedModal() {
  schedModalOpen = true;
  document.getElementById('sched-modal-title').textContent = 'Nouveau programme';
  document.getElementById('sched-edit-valve').value = '';
  document.getElementById('sched-edit-idx').value = '';
  buildSchedValveSelect();
  document.getElementById('sched-valve').value = 0;
  document.getElementById('sched-name').value = '';
  document.getElementById('sched-time').value = '06:00';
  document.getElementById('sched-dur').value = 900;
  document.getElementById('sched-vol').value = 20;
  document.getElementById('sched-calmode').value = 0;
  // interval start default = today
  const today = new Date();
  document.getElementById('sched-interval-start').value = today.toISOString().slice(0,10);
  setSchedUnit('sec'); // par défaut on saisit en secondes
  updateSchedCalMode();
  // Reset jours
  document.querySelectorAll('.day-btn').forEach(b=>{
    b.classList.toggle('sel', parseInt(b.dataset.d)<5);
  });
  document.getElementById('sched-modal').classList.add('open');
}

function editSched(flatIdx) {
  schedModalOpen = true;
  const s = schedules[flatIdx];
  document.getElementById('sched-modal-title').textContent = 'Modifier programme';
  document.getElementById('sched-edit-valve').value = s.valve;
  document.getElementById('sched-edit-idx').value = s.schedIdx;
  // Reconstruit le select AVANT d'imposer la valeur, pour être sûr que
  // l'option correspondant à s.valve existe bel et bien dans le DOM.
  buildSchedValveSelect();
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
  // Conversion volume : on tente d'abord le mode "Volume" (L) — la
  // fonction setSchedUnit() rebascule automatiquement en mode durée
  // si la vanne n'a pas de flowCoeff. L'utilisateur pourra toujours
  // revenir à la saisie en secondes via le toggle.
  setSchedUnit('L');
  // Synchronise le champ litres à partir de la valeur stockée
  // (fallback : on recalcule depuis durationSec, et inversement).
  // PRÉFÉRENCE : si l'entrée porte un champ `litres` (export JSON
  // qui préservait l'intention en litres), on l'utilise tel quel
  // pour pré-remplir le champ Volume, plutôt que de recalculer à
  // partir de durationSec (qui peut être en léger écart si les
  // coeffs ont changé). C'est ce qui permet de garder la saisie
  // originale de l'utilisateur même après un round-trip export/import.
  const fc  = getCurrentValveFlowCoeff();
  const ppl = (window.PULSES_PER_LITRE && window.PULSES_PER_LITRE > 0)
              ? window.PULSES_PER_LITRE : 0;
  if(fc > 0 && ppl > 0){
    let litres;
    if (s.litres !== undefined && s.litres !== null && Number(s.litres) > 0) {
      // Valeur exportée en litres : on l'utilise directement, même si
      // elle ne correspond pas exactement à durationSec * fc / ppl
      // (cas typique : la vanne a été recalibrée entre temps). Le
      // champ secondes est juste re-synchronisé pour rester cohérent
      // à l'affichage — la vraie valeur de référence au save() est
      // l'unité actuellement sélectionnée (L ou sec).
      litres = Number(s.litres);
    } else {
      litres = s.durationSec * fc / ppl;
    }
    document.getElementById('sched-vol').value = litres.toFixed(2);
    document.getElementById('sched-dur').value = s.durationSec;
  } else {
    // Pas de calibration : on force l'affichage en secondes et on
    // efface le champ litres pour éviter toute confusion.
    setSchedUnit('sec');
    document.getElementById('sched-dur').value = s.durationSec;
    document.getElementById('sched-vol').value = '';
  }
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

// ══════════════════════════════════════════════════════════
// PROGRAMME — UNITÉ DURÉE / VOLUME + CONVERSION VIA COEFF
// ══════════════════════════════════════════════════════════
//
// Le firmware ne stocke QUE la durée d'ouverture (durationSec, en
// secondes). L'UI peut laisser l'utilisateur saisir indifféremment
// une durée OU un volume en litres. La conversion litres<->secondes
// utilise :
//   • le coeff de calibration de la VANNE concernée (flowCoeff, en
//     pulses/s) — exposé par /api/calibration/status ET /api/consumption
//   • la constante d'étalonnage du capteur (pulsesPerLitre) — exposée
//     par /api/calibration/status
// Formules :
//   flowLpm   = flowCoeff * 60 / pulsesPerLitre
//   duration  = litres * pulsesPerLitre / flowCoeff     (litres → secondes)
//   litres    = duration  * flowCoeff / pulsesPerLitre (secondes → litres)
//
// Si la vanne n'a pas encore de flowCoeff (calibration jamais faite),
// le mode "Volume" est désactivé et un message invite l'utilisateur à
// calibrer la vanne depuis l'onglet Calibration.

let schedUnit = 'sec';   // 'sec' | 'L' — état UI, pas persisté

function setSchedUnit(unit){
  schedUnit = (unit === 'L') ? 'L' : 'sec';
  const btnSec = document.getElementById('sched-unit-sec');
  const btnL   = document.getElementById('sched-unit-l');
  const rowSec = document.getElementById('sched-dur-row');
  const rowL   = document.getElementById('sched-vol-row');
  if(btnSec && btnL){
    btnSec.classList.toggle('sel', schedUnit === 'sec');
    btnL.classList.toggle('sel',   schedUnit === 'L');
  }
  if(rowSec) rowSec.style.display = (schedUnit === 'sec') ? 'block' : 'none';
  if(rowL)   rowL.style.display   = (schedUnit === 'L')   ? 'block' : 'none';
  // Re-applique l'état "enabled/disabled" du mode volume (selon que la
  // vanne courante a un flowCoeff ou non).
  refreshSchedUnitAvailability();
  // Si on vient de basculer en mode volume, on synchronise le champ
  // litres à partir de la valeur actuelle en secondes.
  if(schedUnit === 'L') syncSchedFromSec();
}

// Renvoie le flowCoeff (pulses/s) de la vanne sélectionnée dans le modal,
// ou 0 si indisponible / non calibré. Mémorise aussi la valeur pour
// pouvoir être rappelée après une actualisation distante (WebSocket).
function getCurrentValveFlowCoeff(){
  const sel = document.getElementById('sched-valve');
  if(!sel) return 0;
  const v = parseInt(sel.value);
  if(isNaN(v) || v < 0) return 0;
  // window.__flowCoeffs[v] = flowCoeff en pulses/s, mis à jour par
  // refreshCalibration() et refreshConsumption().
  const fc = (window.__flowCoeffs && window.__flowCoeffs[v]) ? Number(window.__flowCoeffs[v]) : 0;
  return (fc > 0) ? fc : 0;
}

function refreshSchedUnitAvailability(){
  const btnL   = document.getElementById('sched-unit-l');
  const volEl  = document.getElementById('sched-vol');
  const hintEl = document.getElementById('sched-vol-hint');
  if(!btnL || !volEl) return;
  const fc = getCurrentValveFlowCoeff();
  const ppl = (window.PULSES_PER_LITRE && window.PULSES_PER_LITRE > 0)
              ? window.PULSES_PER_LITRE : 0;
  const canUseL = (fc > 0 && ppl > 0);
  btnL.disabled = !canUseL;
  btnL.style.opacity = canUseL ? '1' : '0.5';
  btnL.style.cursor  = canUseL ? 'pointer' : 'not-allowed';
  if(canUseL){
    const lpm = (fc * 60 / ppl);
    hintEl.innerHTML = `Conversion: ${fc.toFixed(2)} pulses/s → ${lpm.toFixed(2)} L/min.`;
  } else {
    hintEl.innerHTML = '⚠ Vanne non calibrée — allez sur l\'onglet <strong>Calibration</strong> pour mesurer le débit de cette vanne.';
  }
  // Si la vanne n'est pas calibrée ET qu'on est en mode volume, on rebascule
  // automatiquement en mode durée (sinon l'utilisateur saisirait des litres
  // fantaisie qui ne donneraient rien de cohérent).
  if(!canUseL && schedUnit === 'L'){
    setSchedUnit('sec');
  }
  volEl.disabled = !canUseL;
}

// Lit la valeur en secondes et synchronise le champ litres (si visible).
function syncSchedFromSec(){
  if(schedUnit !== 'sec') return; // pas de boucle, on ne fait rien si on est en mode L
  const durEl = document.getElementById('sched-dur');
  const volEl = document.getElementById('sched-vol');
  if(!durEl || !volEl) return;
  const sec = parseFloat(durEl.value);
  if(!isFinite(sec) || sec <= 0){ volEl.value = ''; return; }
  const fc  = getCurrentValveFlowCoeff();
  const ppl = (window.PULSES_PER_LITRE && window.PULSES_PER_LITRE > 0)
              ? window.PULSES_PER_LITRE : 0;
  if(fc > 0 && ppl > 0){
    const litres = sec * fc / ppl;
    volEl.value = litres.toFixed(2);
  } else {
    volEl.value = '';
  }
}

// Lit la valeur en litres et synchronise le champ secondes (si visible).
function syncSchedFromVol(){
  if(schedUnit !== 'L') return;
  const durEl = document.getElementById('sched-dur');
  const volEl = document.getElementById('sched-vol');
  if(!durEl || !volEl) return;
  const litres = parseFloat(volEl.value);
  if(!isFinite(litres) || litres <= 0){ return; }
  const fc  = getCurrentValveFlowCoeff();
  const ppl = (window.PULSES_PER_LITRE && window.PULSES_PER_LITRE > 0)
              ? window.PULSES_PER_LITRE : 0;
  if(fc > 0 && ppl > 0){
    // durée en secondes pour faire passer `litres` litres à fc pulses/s
    const sec = Math.max(1, Math.round(litres * ppl / fc));
    durEl.value = sec;
  }
}

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
  // On ferme le modal et on relâche le verrou seulement APRÈS confirmation
  // du serveur + rechargement de la liste, pour éviter tout affichage
  // transitoire incohérent (ancienne ligne grisée, nouvelle ligne ailleurs).
  api('POST','/api/schedule/save',body).then(()=>{
    return loadSchedules();
  }).then(()=>{
    closeModal('sched-modal');
  });
}

function deleteSched(valve, schedIdx) {
  if(!confirm('Supprimer ce programme ?')) return;
  api('POST','/api/schedule/delete',{valve,schedIdx}).then(()=>loadSchedules());
}

function dupSched(flatIdx) {
  const s = schedules[flatIdx];
  let msg = "Sur quelle vanne voulez-vous dupliquer ce programme ? (Entrez le numéro) :\n\n";
  valves.forEach((v, i) => {
    msg += (i + 1) + " — " + (v.name || ('Vanne ' + (i + 1))) + "\n";
  });
  
  let destStr = prompt(msg, (s.valve + 1));
  if (!destStr) return; // annulé
  
  let destV = parseInt(destStr, 10) - 1;
  if (isNaN(destV) || destV < 0 || destV >= valves.length) {
    alert("Numéro de vanne invalide.");
    return;
  }
  
  // On propage la valeur "litres" si elle existe sur le programme
  // source : à l'enregistrement (saveSched) le modal fera la
  // conversion inverse si la vanne cible a un flowCoeff différent.
  // Si la vanne source a un flowCoeff connu et que le programme
  // source n'a pas de litres, on en calcule un à la volée pour que
  // l'intention "X litres sur la vanne source" survive à la copie
  // vers une vanne avec un coeff potentiellement différent.
  let litresToCarry = (s.litres !== undefined && s.litres !== null && Number(s.litres) > 0)
                     ? Number(s.litres)
                     : null;
  if (litresToCarry === null) {
    const srcFc = (window.__flowCoeffs && window.__flowCoeffs[s.valve] !== undefined)
                  ? Number(window.__flowCoeffs[s.valve]) : 0;
    const srcPpl = (window.PULSES_PER_LITRE && window.PULSES_PER_LITRE > 0) ? window.PULSES_PER_LITRE : 0;
    if (srcFc > 0 && srcPpl > 0) {
      litresToCarry = (s.durationSec * srcFc) / srcPpl;
    }
  }

  let body = {
    valve: destV,
    schedIdx: -1, // demande un nouvel emplacement
    active: s.active,
    hour: s.hour,
    minute: s.minute,
    durationSec: s.durationSec,
    weekDays: s.weekDays,
    calMode: s.calMode,
    intervalDays: s.intervalDays,
    intervalStartMonth: s.intervalStartMonth,
    intervalStartDay: s.intervalStartDay,
    seasonStartMonth: s.seasonStartMonth,
    seasonStartDay: s.seasonStartDay,
    seasonEndMonth: s.seasonEndMonth,
    seasonEndDay: s.seasonEndDay,
    name: (s.name ? (s.name + " (copie)") : "")
  };
  if (litresToCarry !== null) body.litres = litresToCarry;
  
  api('POST', '/api/schedule/save', body).then(r => {
    if(!r.ok){
      if(r.reason === 'full') alert("Impossible : plus de place libre sur cette vanne.");
      else alert("Erreur lors de la duplication.");
    }
    return loadSchedules();
  });
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
          // Calcul en UTC (cohérent avec ydayUTC() utilisé par
          // scheduleMatchesOnDate() — voir correctif bug 5 plus haut).
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
        // Convention 0-based (sd.valve=0 → "V0") cohérente avec le reste
        // de l'UI et avec les JSON d'export programmes.
        badges += `<span class="cal-badge" data-valve="${sd.valve}" data-tip="${title}" style="background:var(--vcol${sd.valve});">V${sd.valve}</span>`;
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
    document.getElementById('cfg-tz-posix').value = cfg.tzPosix||'CET-1CEST,M3.5.0,M10.5.0/3';
    document.getElementById('cfg-lfreq').value   = cfg.loraFreq||868.0;
    document.getElementById('cfg-lpow').value    = cfg.loraPower||10;
    document.getElementById('cfg-nodeid').value  = cfg.nodeId||'IRRIGATION01';
    document.getElementById('cfg-seq').checked   = cfg.irrigMode===1;
    document.getElementById('cfg-maxopen').value = cfg.maxOpenSec||3600;
    document.getElementById('cfg-forcedu').value = cfg.manualForceSec||1800;
    // MQTT
    document.getElementById('cfg-mqena').checked  = cfg.mqttEnabled !== false;
    document.getElementById('cfg-mqhost').value  = cfg.mqttHost||'192.168.1.70';
    document.getElementById('cfg-mqport').value  = cfg.mqttPort||1883;
    document.getElementById('cfg-mquser').value  = cfg.mqttUser||'';
    document.getElementById('cfg-mqpass').value  = '';
    document.getElementById('cfg-mqprefix').value= cfg.mqttPrefix||'homeassistant';
    document.getElementById('cfg-mqid').value    = cfg.mqttId||'irrpro_hs3';
    // Noms vannes
    const grid = document.getElementById('valve-names-grid');
    grid.innerHTML = (cfg.valveNames||[]).map((n,i)=>
      `<div class="form-group"><label>V${i}</label>
       <input type="text" id="vname-${i}" value="${n||'V'+i}"/></div>`
    ).join('');
  });
  // Charger aussi l'état NVS pendant qu'on est sur la page config
  loadNvsStatus();
}

// ──────────────────────────────────────────────────────────
// Maintenance NVS : jauge de remplissage + formatage flash
// ──────────────────────────────────────────────────────────
// Interroge /api/nvs/status et met à jour la jauge + le libellé.
// Affiche la valeur en pourcentage et en nombre d'entrées utilisées
// / totales. Change la couleur de la barre (vert < 70%, orange 70-90%,
// rouge > 90%) pour alerter visuellement.
function loadNvsStatus(){
  api('GET','/api/nvs/status').then(s=>{
    const pctEl = document.getElementById('nvs-pct');
    const fill  = document.getElementById('nvs-fill');
    const det   = document.getElementById('nvs-detail');
    if(!s || !s.ok){
      pctEl.textContent = 'N/A';
      fill.style.width  = '0%';
      det.textContent   = 'Indisponible';
      return;
    }
    const pct = s.usedPct || 0;
    pctEl.textContent = pct + '%';
    fill.style.width  = pct + '%';
    fill.classList.remove('warn','danger');
    if(pct >= 90)      fill.classList.add('danger');
    else if(pct >= 70) fill.classList.add('warn');
    det.textContent = `${s.used} / ${s.total} entrées utilisées (${s.free} libres)`;
  });
}

// Demande de formatage NVS. 2 confirmations successives pour éviter
// les fausses manipulations : on affiche d'abord un texte d'avertissement
// (l'utilisateur doit cliquer OK), puis on lui demande de retaper "FORMAT"
// en majuscules (le prompt natif du navigateur) avant d'envoyer
// réellement la requête au firmware. Le firmware répond OK puis reboote.
function formatNvs(){
  if(!confirm('⚠ Formater la mémoire flash (NVS) ?\n\n' +
              'Toute la configuration sera effacée (programmes, conso, calibration, journal).\n' +
              'Le SSID et le mot de passe WiFi seront recopiés immédiatement après.\n' +
              'Le boîtier va redémarrer.')){
    return;
  }
  const confirm2 = prompt('Pour confirmer, tapez exactement FORMAT (en majuscules) :');
  if(confirm2 !== 'FORMAT'){
    alert('Annulé.');
    return;
  }
  // Feedback visuel : on désactive le bouton et on affiche un message
  // avant que la requête parte. Le firmware répond puis reboot ~400 ms
  // après, on attend donc explicitement la réponse pour pouvoir informer
  // l'utilisateur (sinon l'UI reste bloquée sur "Formatage en cours…").
  const btn = event && event.target;
  if(btn){ btn.disabled = true; btn.textContent = 'Formatage en cours…'; }
  const det = document.getElementById('nvs-detail');
  if(det) det.textContent = 'Formatage en cours, redémarrage imminent…';

  // On n'utilise PAS api() ici car elle avale les erreurs réseau. Le reboot
  // ferme brutalement la socket TCP, ce qui ferait passer le résultat pour
  // un succès (r === {} → r.ok undefined). On fetch directement pour pouvoir
  // distinguer "réponse OK du firmware avant reboot" de "socket coupée".
  let answered = false;
  const ctrl = new AbortController();
  const timeoutId = setTimeout(()=>{
    if(answered) return;
    ctrl.abort();
    if(det) det.textContent = 'Pas de réponse du boîtier (timeout). Vérifiez l\'alimentation et la connexion WiFi.';
    if(btn){ btn.disabled = false; btn.textContent = '⚠ Formater la mémoire flash'; }
  }, 5000);

  fetch('/api/format', { method: 'POST', signal: ctrl.signal })
    .then(r => r.json().then(j => ({ status: r.status, body: j })))
    .then(({ status, body })=>{
      answered = true;
      clearTimeout(timeoutId);
      if(body && body.ok){
        if(btn) btn.textContent = '✓ Formatage OK — redémarrage…';
        if(det) det.textContent = 'Formatage terminé. Le boîtier redémarre, la page sera inaccessible quelques secondes.';
      } else {
        const reason = (body && body.reason) ? body.reason : 'inconnue';
        if(btn){ btn.disabled = false; btn.textContent = '⚠ Formater la mémoire flash'; }
        if(det) det.textContent = 'Échec du formatage : ' + reason;
        alert('Échec du formatage : ' + reason);
      }
    })
    .catch(err=>{
      // Si on a déjà reçu une réponse, ignorer une erreur tardive (race).
      if(answered) return;
      answered = true;
      clearTimeout(timeoutId);
      if(btn) btn.textContent = '⏳ Redémarrage…';
      if(det) det.textContent = 'Le boîtier ne répond plus (normal pendant le reboot). Patientez 10-20 s puis rechargez la page.';
    });
}

// ──────────────────────────────────────────────────────────
// Scan WiFi depuis la page de configuration principale
// ──────────────────────────────────────────────────────────
// Affiche la liste des réseaux visibles avec SSID + RSSI + type de
// chiffrement + barres de signal (même esprit que la page AP du portail
// captif). Le backend (MainIocan.cpp) gère un scan async : on interroge
// /api/wifi/scan et si le scan est en cours on ré-interroge 1.5 s plus tard.
function rssiToBars(rssi){
  // Barres type Wi-Fi Android : mapping indicatif basé sur RSSI typique.
  // -30 dBm = excellent, -90 dBm = limite. 5 niveaux.
  if(rssi >= -55) return 5;
  if(rssi >= -65) return 4;
  if(rssi >= -75) return 3;
  if(rssi >= -82) return 2;
  return 1;
}
function rssiColor(rssi){
  if(rssi >= -65) return 'var(--green)';
  if(rssi >= -75) return 'var(--orange)';
  return 'var(--red)';
}
function barsSvg(level){
  // 5 barres, colorées jusqu'au niveau `level`, les autres en gris dim.
  let s = '<svg width="22" height="14" viewBox="0 0 22 14" style="vertical-align:middle">';
  const widths  = [2,4,6,8,10];
  const heights = [3,5,7,9,11];
  for(let i=0;i<5;i++){
    const active = i < level;
    const color  = active ? rssiColor(-55 - i*8) : '#3a3f47';
    const h = heights[i];
    const w = widths[i];
    const y = 14 - h;
    const x = i * 4;
    s += `<rect x="${x}" y="${y}" width="${w}" height="${h}" rx="0.5" fill="${color}"/>`;
  }
  s += '</svg>';
  return s;
}
function renderWifiNetworks(list){
  const box = document.getElementById('wifi-networks');
  if(!list || !list.length){
    box.innerHTML = '<div style="font-size:.78rem;color:var(--text-muted);padding:6px 0">Aucun réseau détecté.</div>';
    return;
  }
  // Met en évidence le SSID actuellement configuré pour faciliter le repérage.
  const currentSsid = (window.sysConfig && sysConfig.ssid) ? sysConfig.ssid : '';
  box.innerHTML = list.map(n => {
    const isCurrent = (n.ssid === currentSsid);
    return `
      <div class="wifi-net-row" data-ssid="${escapeHtml(n.ssid)}"
           style="display:flex;align-items:center;gap:10px;padding:7px 10px;
                  background:var(--surface2);border:1px solid ${isCurrent ? 'var(--green)' : 'var(--border)'};
                  border-radius:6px;margin-bottom:4px;cursor:pointer">
        <div style="flex:0 0 auto">${barsSvg(rssiToBars(n.rssi))}</div>
        <div style="flex:1;min-width:0;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;font-size:.88rem">
          ${escapeHtml(n.ssid)}${isCurrent ? ' <span style="font-size:.7rem;color:var(--green)">(actuel)</span>' : ''}
        </div>
        <div style="flex:0 0 auto;font-size:.75rem;color:var(--text-muted)">
          ${n.enc || ''}
        </div>
        <div style="flex:0 0 auto;font-size:.78rem;color:var(--text-muted);min-width:48px;text-align:right">
          ${n.rssi} dBm
        </div>
      </div>`;
  }).join('');
  // Clic sur une ligne → remplit le champ SSID et focus le champ password.
  box.querySelectorAll('.wifi-net-row').forEach(row => {
    row.addEventListener('click', () => {
      const ssid = row.getAttribute('data-ssid');
      if(!ssid) return;
      document.getElementById('cfg-ssid').value = ssid;
      const pw = document.getElementById('cfg-wpass');
      if(pw) pw.focus();
    });
  });
}
function escapeHtml(s){
  return String(s).replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
}
let wifiScanInFlight = false;
function scanWifiNetworks(){
  if(wifiScanInFlight) return;
  wifiScanInFlight = true;
  const btn    = document.getElementById('wifi-scan-btn');
  const status = document.getElementById('wifi-scan-status');
  const box    = document.getElementById('wifi-networks');
  if(btn) btn.disabled = true;
  if(status) status.textContent = 'Recherche…';
  if(box) box.innerHTML = '<div style="font-size:.78rem;color:var(--text-muted);padding:6px 0">Scan en cours…</div>';

  const tick = (attempt) => {
    api('GET','/api/wifi/scan').then(d => {
      if(!d || !d.ok){
        const reason = (d && d.reason) || '';
        const msg = reason === 'captive'
          ? 'Portail captif actif — le scan est disponible sur sa page.'
          : 'Scan indisponible (mode STA inactif).';
        if(status) status.textContent = msg;
        if(box)    box.innerHTML    = '';
        wifiScanInFlight = false;
        if(btn) btn.disabled = false;
        return;
      }
      if(d.running){
        // Pas encore prêt — on ré-interroge dans 1.5 s, max 8 tentatives (12 s)
        if(attempt >= 8){
          if(status) status.textContent = 'Scan trop long — réessayez.';
          if(box)    box.innerHTML    = '';
          wifiScanInFlight = false;
          if(btn) btn.disabled = false;
          return;
        }
        setTimeout(() => tick(attempt+1), 1500);
        return;
      }
      // Résultat dispo
      const list = d.networks || [];
      if(status) status.textContent = list.length
        ? `${list.length} réseau${list.length>1?'x':''} détecté${list.length>1?'s':''}.`
        : 'Aucun réseau détecté.';
      renderWifiNetworks(list);
      wifiScanInFlight = false;
      if(btn) btn.disabled = false;
    }).catch(() => {
      if(status) status.textContent = 'Erreur réseau lors du scan.';
      if(box)    box.innerHTML    = '';
      wifiScanInFlight = false;
      if(btn) btn.disabled = false;
    });
  };
  tick(0);
}

function saveConfig() {
  const valveNames = [];
  // CORRECTIF (bug 2) : fallback aligné sur VALVE_COUNT_FALLBACK (5) au lieu
  // de 8 — évite d'envoyer des noms de vannes fantômes (V6/V7/V8 inexistantes)
  // si /api/config n'a pas encore répondu au moment du clic "Sauvegarder".
  const nameCount = (window.sysConfig && sysConfig.valveNames && sysConfig.valveNames.length)
                   ? sysConfig.valveNames.length : VALVE_COUNT_FALLBACK;
  for(let i=0;i<nameCount;i++){
    const el=document.getElementById('vname-'+i);
    valveNames.push(el?el.value:'V'+i);
  }
  const body = {
    ssid: document.getElementById('cfg-ssid').value,
    wifiPass: document.getElementById('cfg-wpass').value||undefined,
    ntpServer: document.getElementById('cfg-ntp').value,
    tzOffset: parseInt(document.getElementById('cfg-tz').value),
    tzPosix: document.getElementById('cfg-tz-posix').value||undefined,
    loraFreq: parseFloat(document.getElementById('cfg-lfreq').value),
    loraPower: parseInt(document.getElementById('cfg-lpow').value),
    nodeId: document.getElementById('cfg-nodeid').value,
    irrigMode: document.getElementById('cfg-seq').checked?1:0,
    maxOpenSec: parseInt(document.getElementById('cfg-maxopen').value),
    manualForceSec: parseInt(document.getElementById('cfg-forcedu').value),
    mqttEnabled: document.getElementById('cfg-mqena').checked,
    mqttHost: document.getElementById('cfg-mqhost').value,
    mqttPort: parseInt(document.getElementById('cfg-mqport').value),
    mqttUser: document.getElementById('cfg-mquser').value,
    mqttPass: document.getElementById('cfg-mqpass').value||undefined,
    mqttPrefix: document.getElementById('cfg-mqprefix').value,
    mqttId: document.getElementById('cfg-mqid').value,
    valveNames
  };
  // CORRECTIF (bug B) : on vérifie désormais d.ok avant d'afficher le message
  // de succès. Auparavant, api() catchait silencieusement toute erreur réseau
  // et renvoyait {} — le .then() s'exécutait quand même et affichait
  // "Configuration sauvegardée" même en cas d'échec réel de la requête,
  // ce qui pouvait faire croire à l'utilisateur que ses changements WiFi/MQTT
  // avaient été pris en compte alors que ce n'était pas le cas.
  api('POST','/api/config',body).then((d)=>{
    if(d && d.ok){
      if(d.restart){
        alert('Configuration sauvegardée.\n\nWiFi SSID/mot de passe modifiés — l\'ESP redémarre dans ~1 seconde pour appliquer les changements.');
      } else {
        alert('Configuration sauvegardée. Redémarrage recommandé pour appliquer MQTT.');
      }
    } else {
      alert('Échec de la sauvegarde — vérifiez la connexion à l\'appareil et réessayez.');
    }
  });
}

function refreshPulse(){
  api('GET','/api/pulse').then(d=>{
    if(!d) return;
    document.getElementById('pulse-count').textContent = d.pulses;
    document.getElementById('pulse-litres').textContent = (d.litres||0).toFixed(2);
  });
}

function resetPulse(){
  api('POST','/api/pulse/reset').then(r=>{ refreshPulse(); refreshConsumption(); alert('Compteur + suivi par vanne remis à zéro'); });
}

function fmtYMD(v){
  if(!v || v<10000000) return '—';
  const s = String(v);
  return s.substring(6,8)+'/'+s.substring(4,6);
}

function refreshConsumption(){
  api('GET','/api/consumption').then(d=>{
    if(!d || !d.valves){
      const msg = '<tr><td colspan="5" style="text-align:center;color:var(--text-muted);padding:18px">Pas de données</td></tr>';
      const tb1 = document.getElementById('cons-body');
      const tb2 = document.getElementById('status-cons-body');
      if(tb1) tb1.innerHTML = msg;
      if(tb2) tb2.innerHTML = msg.replace('colspan="5"','colspan="4"');
      return;
    }
    // Mémorise les flowCoeffs exposés par /api/consumption pour qu'ils
    // soient disponibles au moment où l'utilisateur ouvre le modal
    // "Nouveau programme" sans avoir à attendre un passage par l'onglet
    // Calibration (autre source d'initialisation de window.__flowCoeffs).
    if(Array.isArray(d.valves)){
      // Initialise le tableau si pas déjà fait (sinon on conserve
      // d'éventuelles valeurs plus précises venues de /api/calibration/status).
      if(!window.__flowCoeffs || !window.__flowCoeffs.length){
        window.__flowCoeffs = d.valves.map(v => Number(v.flowCoeff) || 0);
      } else {
        // Met à jour seulement les entrées qui étaient à 0 (pas calibrées) :
        // la calibration reste la source de vérité la plus précise.
        d.valves.forEach((v, i) => {
          if(!window.__flowCoeffs[i] && v.flowCoeff && v.flowCoeff > 0){
            window.__flowCoeffs[i] = Number(v.flowCoeff);
          }
        });
      }
    }
    // Version longue pour la page Configuration (5 colonnes : nom + détails 14j)
    const longHtml = d.valves.map(v=>{
      const detail = (v.history && v.history.length)
        ? v.history.slice().reverse().map(h=>`<span style="display:inline-block;padding:2px 6px;margin:2px;border-radius:4px;background:var(--surface2);color:var(--text);font-size:.72rem">${fmtYMD(h.ymd)}: <strong>${(h.litres||0).toFixed(1)} L</strong></span>`).join('')
        : '<span style="color:var(--text-muted)">—</span>';
      return `<tr>
        <td><strong>V${v.valve}</strong></td>
        <td>${v.name||'V'+v.valve}</td>
        <td style="color:var(--blue);font-weight:600">${(v.litresToday||0).toFixed(2)}</td>
        <td>${(v.litresTotal||0).toFixed(2)}</td>
        <td>${detail}</td>
      </tr>`;
    }).join('');
    // Version compacte pour la boîte "État du système" du dashboard
    // (4 colonnes, sans l'historique détaillé — plus lisible sur petit écran).
    const shortHtml = d.valves.map(v=>{
      const today  = v.litresToday  || 0;
      const total  = v.litresTotal  || 0;
      return `<tr>
        <td><strong>V${v.valve}</strong></td>
        <td>${v.name||'V'+v.valve}</td>
        <td style="text-align:right;color:${today>0?'var(--blue)':'var(--text-muted)'};font-weight:600">${today.toFixed(2)} L</td>
        <td style="text-align:right">${total.toFixed(2)} L</td>
      </tr>`;
    }).join('');
    const tb1 = document.getElementById('cons-body');
    const tb2 = document.getElementById('status-cons-body');
    if(tb1) tb1.innerHTML = longHtml;
    if(tb2) tb2.innerHTML = shortHtml || '<tr><td colspan="4" style="text-align:center;color:var(--text-muted);padding:10px">—</td></tr>';
  });
}

// ══════════════════════════════════════════════════════════
// INIT
// ══════════════════════════════════════════════════════════
function buildSchedValveSelect() {
  const sel = document.getElementById('sched-valve');
  // CORRECTIF (bug 3) : fallback aligné sur VALVE_COUNT_FALLBACK (5) au lieu
  // de 8 — évite de proposer des vannes fantômes (V6/V7/V8) dans le modal
  // programme si l'utilisateur clique "+ Ajouter" avant la première réponse
  // status/config qui peuple le tableau valves[].
  const count = (valves && valves.length) ? valves.length : VALVE_COUNT_FALLBACK;
  const prev = sel.value;
  sel.innerHTML = Array.from({length:count},(_,i)=>`<option value="${i}">${(valves[i]&&valves[i].name)?('V'+i+' — '+valves[i].name):('V'+i)}</option>`).join('');
  // restore previous selection if still valid
  if(prev !== undefined && prev !== null && prev !== ''){
    const opt = sel.querySelector(`option[value="${prev}"]`);
    if(opt) sel.value = prev;
  }
  // Listener "change" : à chaque changement de vanne, on ré-évalue
  // la disponibilité du mode Volume (la vanne cible a son propre
  // flowCoeff) et on resynchronise le champ litres↔secondes.
  // On attache le listener en mode "une fois" pour ne pas empiler
  // des handlers à chaque reconstruction du <select>.
  if(!sel.dataset.listenerAttached){
    sel.addEventListener('change', () => {
      if(schedModalOpen){
        refreshSchedUnitAvailability();
        // Si on est en mode secondes, on recalcule le champ litres
        // miroir ; si on est en mode litres, le champ secondes
        // miroir (les deux fonctions no-op sur le mode opposé).
        if(schedUnit === 'sec')      syncSchedFromSec();
        else if(schedUnit === 'L')   syncSchedFromVol();
      }
    });
    sel.dataset.listenerAttached = '1';
  }
}

function init() {
  buildSchedValveSelect();
  connectWS();
  requestStatus();
  refreshPulse();
  refreshConsumption();
  loadSchedules();
  // Charge aussi les coefficients de calibration au boot : ils sont
  // nécessaires au calcul du "volume restant" sur les cartes vanne du
  // dashboard (cf. renderValveCards). Sans cet appel, le premier
  // affichage après chargement de la page n'aurait que "— (non
  // calibré)" pour toutes les vannes, même celles déjà calibrées.
  // L'appel est léger (StaticJsonDocument de quelques centaines d'octets)
  // et idempotent : refreshCalibration() se contente de mettre à jour
  // window.__flowCoeffs et window.PULSES_PER_LITRE.
  refreshCalibration();
  // Actualisation auto toutes les 10s si WS déconnecté
  setInterval(()=>{ if(!wsConn||wsConn.readyState!==1) requestStatus(); }, 10000);
  // La conso par vanne change lentement, on rafraîchit toutes les 30 s
  setInterval(refreshConsumption, 30000);
}

init();
</script>
</body>
</html>
)HTMLEOF";