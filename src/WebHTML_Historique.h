#pragma once
// ============================================================
// WebHTML_Historique.h — Page Historique des consommations
// ============================================================
// Affiche un historique des consommations en litres sur 7 jours.
// Les données sont stockées dans un JSON via MQTT et récupérées
// depuis /api/history.

const char WEB_HISTORIQUE_HTML[] PROGMEM = R"HTML(
<div id="page-historique" class="page">
  <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:18px">
    <div>
      <div style="font-size:1.1rem;font-weight:700">Historique des consommations</div>
      <div style="font-size:0.85rem;color:var(--text-muted);margin-top:4px">
        Consommations par vanne sur les 7 derniers jours
      </div>
    </div>
    <div style="display:flex;gap:8px">
      <button class="btn btn-ghost btn-sm" onclick="loadHistory()">↻ Actualiser</button>
      <button class="btn btn-ghost btn-sm" onclick="exportHistory()">📥 Exporter</button>
    </div>
  </div>

  <!-- ══ STATISTIQUES GLOBALES ══════════════════════════════ -->
  <div class="status-box" style="margin-bottom:16px">
    <div class="status-header">
      <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="var(--blue)" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
        <path d="M3 3v18a2 2 0 0 0 2 2h14a2 2 0 0 0 2-2V3"/>
        <path d="M3 9h18"/>
        <path d="M3 15h18"/>
      </svg>
      <h3>Statistiques 7 jours</h3>
    </div>
    <div class="status-grid">
      <!-- Total consommé -->
      <div class="status-card">
        <div class="status-card-icon" style="background:var(--blue-dim)">
          <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="var(--blue)" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
            <path d="M12 2a10 10 0 1 0 0 20 10 10 0 0 0 0-20z"/>
          </svg>
        </div>
        <div class="status-card-body">
          <div class="status-card-label">Total 7j</div>
          <div class="status-card-value" id="hist-total-litres">— L</div>
        </div>
      </div>

      <!-- Consommation moyenne par jour -->
      <div class="status-card">
        <div class="status-card-icon" style="background:var(--green-dim)">
          <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="var(--green)" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
            <circle cx="12" cy="12" r="10"/>
            <line x1="12" y1="8" x2="12" y2="12"/>
            <line x1="12" y1="16" x2="12" y2="16"/>
          </svg>
        </div>
        <div class="status-card-body">
          <div class="status-card-label">Moyenne/jour</div>
          <div class="status-card-value" id="hist-moy-litres">— L</div>
        </div>
      </div>

      <!-- Jour le plus consommateur -->
      <div class="status-card">
        <div class="status-card-icon" style="background:var(--orange-dim)">
          <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="var(--orange)" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
            <polygon points="12 2 15.09 8.26 22 9.27 17 14.14 18.18 21.02 12 17.77 5.82 21.02 7 14.14 2 9.27 8.91 8.26 12 2"/>
          </svg>
        </div>
        <div class="status-card-body">
          <div class="status-card-label">Jour max</div>
          <div class="status-card-value" id="hist-jour-max">—</div>
        </div>
      </div>

      <!-- Dernière mise à jour -->
      <div class="status-card">
        <div class="status-card-icon" style="background:var(--surface2)">
          <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="var(--text-muted)" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
            <rect x="3" y="3" width="18" height="18" rx="2" ry="2"/>
            <circle cx="12" cy="16" r="1"/>
            <path d="M12 8v4l2 2"/>
          </svg>
        </div>
        <div class="status-card-body">
          <div class="status-card-label">Dernière MAJ</div>
          <div class="status-card-value" id="hist-dernier">—</div>
        </div>
      </div>
    </div>
  </div>

  <!-- ══ TABLEAU HISTORIQUE ═══════════════════════════════ -->
  <div class="card">
    <div class="card-header">
      <h2>Historique par vanne (7 jours)</h2>
    </div>
    <div style="padding:16px 20px;overflow-x:auto">
      <table class="tbl" id="history-table">
        <thead>
          <!--
            FIX (mismatch colonnes fixes vs dates glissantes) :
            l'historique est une fenêtre GLISSANTE de 7 jours calendaires,
            pas alignée sur un Lundi-Dimanche fixe. Les 7 colonnes de dates
            (id="hist-col-0" .. "hist-col-6") sont désormais peuplées
            dynamiquement par renderHistory() (WebContent.h) avec les
            vraies dates de la fenêtre en cours, triées du plus ancien
            au plus récent — au lieu d'en-têtes "Lundi/Mardi/..." fixes
            qui ne correspondaient à rien de réel.
          -->
          <tr>
            <th>Vanne</th>
            <th>Nom</th>
            <th style="text-align:right" id="hist-col-0">—</th>
            <th style="text-align:right" id="hist-col-1">—</th>
            <th style="text-align:right" id="hist-col-2">—</th>
            <th style="text-align:right" id="hist-col-3">—</th>
            <th style="text-align:right" id="hist-col-4">—</th>
            <th style="text-align:right" id="hist-col-5">—</th>
            <th style="text-align:right" id="hist-col-6">—</th>
            <th style="text-align:right">Total 7j</th>
          </tr>
        </thead>
        <tbody id="history-body">
          <tr><td colspan="10" style="text-align:center;color:var(--text-muted);padding:24px">Chargement…</td></tr>
        </tbody>
      </table>
    </div>
  </div>

  <!-- ══ GRAFIQUE ════════════════════════════════════════ -->
  <div class="card" style="margin-top:20px">
    <div class="card-header">
      <h2>Évolution globale (7 jours)</h2>
    </div>
    <div style="padding:16px 20px">
      <div id="history-chart" style="width:100%;height:200px;background:var(--surface2);border-radius:8px;display:flex;align-items:center;justify-content:center;color:var(--text-muted)">
        <span id="history-chart-placeholder">Graphique non disponible</span>
      </div>
    </div>
  </div>

  <!-- ══ BOUTON RESET ═════════════════════════════════════ -->
  <div style="margin-top:20px;display:flex;gap:10px;justify-content:flex-end">
    <button class="btn btn-red btn-sm" onclick="resetHistory()" style="display:none" id="history-reset-btn">
      🗑 Réinitialiser l'historique
    </button>
  </div>
</div>

)HTML";