#pragma once
// ============================================================
// WebCSS.h — Styles CSS de l'interface web
// ============================================================

const char WEB_CSS[] PROGMEM = R"CSS(
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
/* Carte alarme hydraulique — bordure rouge + clignotement léger
   pour attirer l'œil sans être trop agressif. */
.status-card.alarm-active{
  border-color:var(--red);
  background:linear-gradient(0deg, rgba(248,81,73,0.08), rgba(248,81,73,0.08)), var(--bg);
  animation: alarm-blink 1.4s ease-in-out infinite;
}
@keyframes alarm-blink{
  0%,100%{ box-shadow:0 0 0 0 rgba(248,81,73,0.0); }
  50%    { box-shadow:0 0 0 3px rgba(248,81,73,0.25); }
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
)CSS";