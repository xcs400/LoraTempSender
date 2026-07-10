#pragma once
// ============================================================
// WebHTML_Journal.h — Page Journal HTML
// ============================================================

#define WEB_JOURNAL_HTML R"HTML(
<!-- ══ PAGE JOURNAL ════════════════════════════════════ -->
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
)HTML";