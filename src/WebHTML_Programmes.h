#pragma once
// ============================================================
// WebHTML_ProgrammesModal.h — Page Programmes HTML
// ============================================================

const char WEB_PROGRAMMES_HTML[] PROGMEM = R"HTML(




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
            <th>Vanne</th><th>Nom</th><th>Heure</th><th>Durée</th><th>Litres</th><th>Jours</th><th>Mode</th><th>Actif</th><th>Actions</th>
          </tr>
        </thead>
        <tbody id="sched-body">
          <tr><td colspan="9" style="text-align:center;color:var(--text-muted);padding:24px">Chargement…</td></tr>
        </tbody>
      </table>
    </div>
  </div>
</div>


)HTML";