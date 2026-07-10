#pragma once
// ============================================================
// WebJS_Dashboard.h — JavaScript Dashboard
// ============================================================
#define WEB_JS_DASHBOARD R"JS(

<script>
// ══════════════════════════════════════════════════════════
// DASHBOARD
// ══════════════════════════════════════════════════════════
function renderValveCards() {
  const grid = document.getElementById('valve-grid');
  if (!grid || !Array.isArray(valves)) return;
  grid.innerHTML = valves.map((v, i) => {
    const isOpen = !!v.state;
    const isForced = (v.source === 'INPUT' || v.source === 'WEB');
    const cardCls = (isOpen ? ' open' : '') + (isForced ? ' forced' : '');
    // Prochain événement programmé — on cherche dans schedules.flat
    // (mis à jour par loadSchedules) le prochain programme actif pour
    // cette vanne. Si trouvé, on affiche heure + durée dans une
    // boîte "Prochain événement" sous les badges.
    let nextEv = null;
    if(Array.isArray(schedules)){
      // Trie par heure (plus proche dans le futur en premier)
      const now = new Date();
      const todayScheds = schedules.filter(s => 
        s.valve === i && s.active && 
        s.weekDays & (1 << ((now.getDay() + 6) % 7)) // 0=dim, 6=sam
      );
      if(todayScheds.length > 0){
        // Trie par heure croissante
        todayScheds.sort((a,b) => {
          const [ha,ma] = [a.hour||0, a.minute||0];
          const [hb,mb] = [b.hour||0, b.minute||0];
          return (ha*60+ma) - (hb*60+mb);
        });
        // Prend le premier qui est dans le futur
        const nowMin = now.getHours() * 60 + now.getMinutes();
        for(const s of todayScheds){
          const schedMin = (s.hour||0)*60 + (s.minute||0);
          if(schedMin >= nowMin){
            nextEv = {
              text: (s.hour||0) + ':' + String(s.minute||0).padStart(2,'0'),
              duration: s.durationSec || 0
            };
            break;
          }
        }
        // Si aucun dans le futur aujourd'hui, on prend le premier de demain
        if(!nextEv && todayScheds.length > 0){
          nextEv = {
            text: 'Demain ' + (todayScheds[0].hour||0) + ':' + String(todayScheds[0].minute||0).padStart(2,'0'),
            duration: todayScheds[0].durationSec || 0
          };
        }
      }
    }
    const nextHtml = nextEv 
      ? '<div style="color:var(--text);font-weight:600;margin-bottom:3px">' + nextEv.text + '</div>' +
         '<div style="color:var(--text-muted);font-size:.78rem;margin-bottom:4px">⏱ Prochain</div>' +
         '<div style="font-size:.78rem">' +
           '<span style="color:var(--text-muted)">Durée :</span> <strong>' + fmtSec(nextEv.duration) + '</strong>' +
         '</div>'
      : '<span style="color:var(--text-muted)">—</span>';
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
        remainingLitresHtml = '<div class="vc-remaining-l"><span class="vc-remaining-l-label">💧 Restant</span>' + l.toFixed(2) + ' L</div>';
      } else {
        // Vanne ouverte mais pas (encore) calibrée : on affiche un libellé
        // discret plutôt qu'une valeur inventée, pour rester honnête avec
        // l'utilisateur. "calibrer" est cliquable vers la page calibration.
        remainingLitresHtml = '<div class="vc-remaining-l" style="color:var(--text-muted); font-weight:500"><span class="vc-remaining-l-label">💧 Restant</span>— (non calibré)</div>';
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
        <span style="color:var(--text-muted);font-size:.75rem;display:block;margin-bottom:4px;text-transform:uppercase;letter-spacing:0.5px">Prochain événement</span>
        ${nextHtml}
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
    ? 'Remettre à zéro la consommation d\'aujourd\'hui pour cette vanne ?'
    : 'Remettre à zéro la consommation TOTALE (et l\'historique) de cette vanne ?';
  if(!confirm(msg)) return;
  api('POST','/api/valve/reset_cons',{valve:idx, type:type}).then(()=>{
    requestStatus();
    if (document.getElementById('page-config').classList.contains('active')) refreshConsumption();
  });
}
function resetManualValve(){
  if(!confirm('Remettre à zéro le compteur VanneManuelle ?\n\nCela ne touche pas le compteur global ni les vannes automatisées.')) return;
  api('POST','/api/manual_valve/reset').then(r=>{
    refreshPulse();
    refreshConsumption();
    // Force un refresh du status pour mettre à jour la carte VanneManuelle
    requestStatus();
    alert('Compteur VanneManuelle remis à zéro');
  });
}
function closeAll() {
  if(!confirm('Fermer TOUTES les vannes ?\n\nCette action est irréversible.')) return;
  api('POST','/api/valve/closeall').then(()=>requestStatus());
}
function showForceModal(idx) {
  document.getElementById('force-valve').value = idx;
  document.getElementById('force-duration').value = sysConfig.manualForceSec || 1800;
  document.getElementById('force-modal').classList.add('open');
}
function closeForceModal() {
  document.getElementById('force-modal').classList.remove('open');
}
function forceValve() {
  const idx = parseInt(document.getElementById('force-valve').value);
  const dur = parseInt(document.getElementById('force-duration').value);
  if(isNaN(idx) || isNaN(dur) || idx < 0) return;
  api('POST','/api/valve/force',{valve:idx,duration:dur})
    .then(()=>{
      closeForceModal();
      requestStatus();
    });
}
function fmtEpoch(epoch) {
  if(!epoch || epoch <= 0) return '—';
  const d = new Date(epoch * 1000);
  return d.toLocaleString('fr-FR',{hour:'2-digit',minute:'2-digit'});
}
</script>
)JS"