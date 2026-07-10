#pragma once
// ============================================================
// WebHTML_Calibration.h — Page Calibration HTML
// ============================================================

#define WEB_CALIBRATION_HTML R"HTML(
<!-- ══ PAGE CALIBRATION ═════════════════════════════════ -->
<div id="page-calibration" class="page">
  <div class="card">
    <div class="card-header">
      <h2>Calibration du débitmètre</h2>
      <span id="calib-phase-badge" class="badge" style="background:var(--surface2);color:var(--text-muted);font-size:.75rem">inactif</span>
    </div>

    <div style="padding:8px 4px 16px;color:var(--text-muted;font-size:.86rem;line-height:1.5">
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

    <!-- Tableau des coefficients -->
    <div class="config-section">
      <h3>Coefficients de calibration</h3>
      <div style="overflow-x:auto">
        <table class="tbl">
          <thead>
            <tr>
              <th>Vanne</th>
              <th>Nom</th>
              <th>flowCoeff (pulses/s)</th>
              <th title="Estimation débit nominal (L/min)">Estim. débit</th>
              <th title="Capacité calibrée (L/min)">Capacité</th>
            </tr>
          </thead>
          <tbody id="calib-tbl-body">
            <tr><td colspan="5" style="text-align:center;color:var(--text-muted);padding:24px">Chargement…</td></tr>
          </tbody>
        </table>
      </div>
      <div style="margin-top:12px;font-size:.8rem;color:var(--text-muted">
        <strong>Capacité calibrée</strong> = flowCoeff / PULSES_PER_LITRE × 60.
        Si une vanne n'a jamais été calibrée, flowCoeff = 0 → capacité = 0 L/min.
      </div>
    </div>
  </div>
</div>
)HTML";