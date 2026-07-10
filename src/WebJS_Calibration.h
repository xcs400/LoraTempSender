#pragma once
// ============================================================
// WebJS_Calibration.h — JavaScript Calibration
// ============================================================

#define WEB_JS_CALIBRATION R"JS(

<script>
// ══════════════════════════════════════════════════════════
// CALIBRATION
// ══════════════════════════════════════════════════════════
function refreshCalibration() {
  api('GET', '/api/calibration').then(d => {
    if(!d || !Array.isArray(d.flowMeters)) return;
    const tbody = document.getElementById('calib-body');
    if(!tbody) return;
    // Stocke les coeffs dans window.__flowCoeffs pour les utiliser
    // dans renderValveCards() pour estimer le volume restant.
    window.__flowCoeffs = d.flowMeters.map(m => m.flowCoeff);
    window.PULSES_PER_LITRE = d.pulsesPerLitre;
    tbody.innerHTML = d.flowMeters.map((m,i) => {
      const vname = (valves[i] && valves[i].name) ? valves[i].name : ('V'+i);
      const pulses = m.pulses || 0;
      const duration = m.durationSec || 0;
      const coeff = m.flowCoeff || 0;
      // Affiche le débit en L/min si coeff > 0
      const lpm = (coeff > 0 && d.pulsesPerLitre > 0) 
        ? ((coeff * 60) / d.pulsesPerLitre).toFixed(2) 
        : '—';
      const status = (coeff > 0) ? 'Calibré' : 'Non calibré';
      const cls = (coeff > 0) ? 'calib-ok' : 'calib-none';
      return '<tr>' +
        '<td><strong>V' + i + '</strong></td>' +
        '<td>' + vname + '</td>' +
        '<td>' + pulses + '</td>' +
        '<td>' + duration + '</td>' +
        '<td>' + coeff.toFixed(6) + '</td>' +
        '<td>' + lpm + ' L/min</td>' +
        '<td class="' + cls + '">' + status + '</td>' +
        '<td>' +
          '<button class="btn btn-ghost btn-sm" onclick="startCalibration(' + i + ')">Démarrer</button>' +
        '</td>' +
      '</tr>';
    }).join('');
  }).catch(err => {
    console.error('Échec /api/calibration', err);
  });
}

function startCalibration(valve) {
  const vname = (valves[valve] && valves[valve].name) ? valves[valve].name : ('V'+valve);
  const dur = prompt('Durée de calibration pour ' + vname + ' (secondes):', '30');
  if(!dur || isNaN(dur) || dur <= 0) return;
  api('POST', '/api/calibration/start', {valve: valve, durationSec: parseInt(dur)})
    .then(d => {
      if(d && d.ok) {
        alert('Calibration démarrée pour ' + vname + ' (' + dur + ' sec)');
        // Rafraîchit après quelques secondes
        setTimeout(() => refreshCalibration(), (parseInt(dur)+2)*1000);
      } else {
        alert('Échec du démarrage de calibration');
      }
    })
    .catch(err => {
      console.error('Échec startCalibration', err);
      alert('Échec du démarrage de calibration');
    });
}

function resetCalibration() {
  if(!confirm('Réinitialiser TOUTES les calibrations ?')) return;
  api('POST', '/api/calibration/reset')
    .then(d => {
      if(d && d.ok) {
        refreshCalibration();
        alert('Calibrations réinitialisées');
      } else {
        alert('Échec de la réinitialisation');
      }
    })
    .catch(err => {
      console.error('Échec resetCalibration', err);
      alert('Échec de la réinitialisation');
    });
}
</script>
)JS"