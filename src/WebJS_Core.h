#pragma once
// ============================================================
// WebJS_Core.h — JavaScript principal (state, API, WS, handleStatus, init)
// ============================================================

#define WEB_JS_CORE R"JS(

<script>
// ----------------------------------------------------------
// STATE
// ----------------------------------------------------------
let valves = [];
let schedules = [];    // flat list {valve,idx,...}
let logEntries = [];
let sysConfig = {};
let wsConn = null;
let schedModalOpen = false;
let schedUnit = 'sec'; // 'sec' or 'L' — unité active du modal programme
let VALVE_COUNT_FALLBACK = 5; // aligné sur VANNE_COUNT du firmware

// ----------------------------------------------------------
// API
// ----------------------------------------------------------
function api(method, path, body) {
  const opts = {
    method: method,
    headers: { 'Content-Type': 'application/json' }
  };
  if (body) opts.body = JSON.stringify(body);
  return fetch(path, opts).then(r => {
    if (r.ok) return r.json();
    else throw new Error(`HTTP ${r.status}`);
  });
}

// ----------------------------------------------------------
// WEBSOCKET
// ----------------------------------------------------------
function connectWS() {
  const proto = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const wsUrl = `${proto}//${window.location.host}/ws`;
  wsConn = new WebSocket(wsUrl);
  wsConn.onopen = () => {
    console.log('[WS] Connecté');
    document.getElementById('ws-dot').className = 'ok';
    document.getElementById('ws-label').textContent = 'Connecté';
  };
  wsConn.onclose = () => {
    console.log('[WS] Déconnecté');
    document.getElementById('ws-dot').className = '';
    document.getElementById('ws-label').textContent = 'Déconnecté';
    // retry dans 3s
    setTimeout(connectWS, 3000);
  };
  wsConn.onmessage = (e) => {
    try {
      const d = JSON.parse(e.data);
      if (d.type === 'STATUS') handleStatus(d);
    } catch (err) {
      console.error('[WS] JSON invalide', e.data, err);
    }
  };
  wsConn.onerror = (err) => {
    console.error('[WS] Erreur', err);
  };
}

function requestStatus() {
  api('GET', '/api/status').then(handleStatus).catch(err => {
    console.error('Échec /api/status', err);
  });
}

// ----------------------------------------------------------
// NAVIGATION
// ----------------------------------------------------------
function showPage(id, btn) {
  // Masque toutes les pages
  document.querySelectorAll('.page').forEach(p => p.classList.remove('active'));
  // Désactive tous les boutons nav
  document.querySelectorAll('nav button').forEach(b => b.classList.remove('active'));
  // Affiche la page cible
  document.getElementById('page-' + id).classList.add('active');
  // Active le bouton nav
  if (btn) btn.classList.add('active');
  // Sauvegarde l'état dans l'URL (permet reload/bookmark)
  history.replaceState(null, '', '#' + id);
}

// ----------------------------------------------------------
// STATUS HANDLING
// ----------------------------------------------------------
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
  // Badge MQTT (amélioration A) — reflète mqttConnected envoyé par le firmware
  const mqttDot = document.getElementById('mqtt-dot');
  const mqttLabel = document.getElementById('mqtt-label');
  if (mqttDot && mqttLabel && data.mqttConnected !== undefined) {
    mqttDot.className = data.mqttConnected ? 'ok' : 'off';
    mqttLabel.textContent = 'MQTT: ' + (data.mqttConnected ? 'connecté' : 'déconnecté');
  }
  // Carte "Alarme hydraulique" — reflète data.alarm publié par WsManager
  // (mêmes champs que MQTT : active / code / msg / sinceSec). L'alarme
  // reste affichée même après levée (jusqu'à acquittement ou reload) pour
  // qu'on garde une trace visuelle du dernier incident.
  const alarmCard  = document.getElementById('alarm-card');
  const alarmIcon  = document.getElementById('alarm-icon');
  const alarmLabel = document.getElementById('alarm-label');
  const alarmSub   = document.getElementById('alarm-sub');
  if (alarmCard && alarmLabel && alarmIcon && alarmSub && data.alarm) {
    const a = data.alarm;
    const code = (a.code !== undefined) ? Number(a.code) : 0;
    const active = !!a.active;
    const codeLabel = (code === 1) ? 'NO_FLOW' : (code === 2 ? 'UNEXPECTED_FLOW' : 'OK');
    if (active) {
      alarmCard.classList.add('alarm-active');
      alarmIcon.style.background = 'var(--red-dim)';
      alarmIcon.querySelector('svg').setAttribute('stroke', 'var(--red)');
      alarmLabel.style.color = 'var(--red)';
      alarmLabel.textContent = codeLabel;
      // Message + durée écoulée depuis le déclenchement
      const sec = a.sinceSec || 0;
      const min = Math.floor(sec / 60), s = sec % 60;
      const dur = (min > 0) ? (min + ' min ' + s + ' s') : (s + ' s');
      alarmSub.textContent = (a.msg || '—') + ' · depuis ' + dur;
    } else {
      alarmCard.classList.remove('alarm-active');
      alarmIcon.style.background = 'var(--green-dim)';
      alarmIcon.querySelector('svg').setAttribute('stroke', 'var(--green)');
      alarmLabel.style.color = 'var(--green)';
      alarmLabel.textContent = 'Aucune alarme';
      alarmSub.textContent = (code > 0) ? ('Dernière: ' + codeLabel) : '—';
    }
  }
  // Carte "VanneManuelle" — reflète data.manualValve publié par WsManager
  // (mêmes champs que pour une vanne standard : litresToday, litresTotal,
  // flow_lpm, hasFlow). Permet à l'utilisateur de suivre la conso par la
  // vanne manuelle (robinet en aval du débitmètre) en temps réel.
  {
    const todayEl  = document.getElementById('manual-valve-today');
    const totalEl  = document.getElementById('manual-valve-total');
    const flowEl   = document.getElementById('manual-valve-flow');
    const cardEl   = document.getElementById('manual-valve-card');
    if(todayEl && totalEl && flowEl && cardEl){
      if(data.manualValve){
        const mv = data.manualValve;
        todayEl.textContent = (mv.litresToday || 0).toFixed(2) + ' L';
        totalEl.textContent = (mv.litresTotal || 0).toFixed(2);
        const flow = mv.flow_lpm !== undefined ? Number(mv.flow_lpm) : 0;
        flowEl.textContent = flow.toFixed(2) + ' L/min';
        // Indicateur visuel : si eau en train de couler (hasFlow=true),
        // on met un petit point vert clignotant sur la carte.
        if(mv.hasFlow){
          cardEl.classList.add('alarm-active'); // réutilise l'animation clignotante
          flowEl.style.color = 'var(--green)';
        } else {
          cardEl.classList.remove('alarm-active');
          flowEl.style.color = 'var(--blue)';
        }
      } else {
        todayEl.textContent = '— L';
        totalEl.textContent = '—';
        flowEl.textContent = '— L/min';
        cardEl.classList.remove('alarm-active');
      }
    }
  }
  // Jauge NVS — on l'actualise à chaque STATUS (˜1×/s par le WebSocket)
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
}

// ----------------------------------------------------------
// INIT
// ----------------------------------------------------------
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
  // flowCoeff) et on resynchronise le champ litres?secondes.
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

// ----------------------------------------------------------
// UTILS
// ----------------------------------------------------------
function fmtSec(v) {
  if (!v || v <= 0) return '—';
  const h = Math.floor(v / 3600);
  const m = Math.floor((v % 3600) / 60);
  const s = v % 60;
  if (h > 0) return `${h}h${m.toString().padStart(2, '0')}`;
  if (m > 0) return `${m}m${s.toString().padStart(2, '0')}`;
  return `${s}s`;
}
</script>
)JS"