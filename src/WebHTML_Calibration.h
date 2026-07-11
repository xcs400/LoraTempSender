#pragma once
// ============================================================
// WebHTML_Calibration.h — Page Calibration HTML
// ============================================================
// Contenu identique à l'ancien bloc PAGE CALIBRATION de WebContent.h.
// IDs conservés à l'identique pour que refreshCalibration() (JS dans
// WebContent.h) retrouve tous les éléments qu'il manipule :
//   • calib-phase-badge, calib-launch-form, calib-progress
//   • calib-duration, calib-start-btn, calib-abort-btn, calib-msg
//   • calib-current-valve, calib-remaining, calib-bar
//   • calib-progress-valve, calib-progress-total
//   • calib-coeff-body
// Ne pas modifier les IDs sans mettre à jour refreshCalibration() en
// parallèle.



const char WEB_CALIBRATION_HTML[] PROGMEM = R"HTML(


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

)HTML";