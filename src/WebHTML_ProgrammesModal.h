#pragma once
// ============================================================
// WebHTML_ProgrammesModal.h — Page Programmes HTML
// ============================================================

const char WEB_PROGRAMMESMODAL_HTML[] PROGMEM = R"HTML(
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

)HTML";