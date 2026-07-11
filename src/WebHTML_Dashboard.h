#pragma once
// ============================================================
// WebHTML_Dashboard.h — Page Dashboard HTML
// ============================================================

const char WEB_DASHBOARD_HTML[] PROGMEM = R"HTML(
<div id="page-dashboard" class="page active">
  <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:18px">
    <div>
      <div style="font-size:1.1rem;font-weight:700">Tableau de bord</div>
    </div>
    <div style="display:flex;gap:8px">
      <button class="btn btn-ghost btn-sm" onclick="closeAll()">Tout fermer</button>
    </div>
  </div>

  <!-- ══ BOÎTE STATUS SYSTÈME ══════════════════════════════
       Distinction visuelle forte vs les cartes de vanne :
         • fond bleu-dim/var(--surface) au lieu de la grille blanche
         • cartes internes (uptime, T°, conso) avec icônes inline
         • tableau conso compact en bas
       Mise à jour automatique via handleStatus() / refreshConsumption(). -->
  <div class="status-box">
    <div class="status-header">
      <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="var(--blue)" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
        <circle cx="12" cy="12" r="10"/><polyline points="12 6 12 12 16 14"/>
      </svg>
      <h3>État du système</h3>
      <span class="status-time" id="status-time">—</span>
    </div>

    <div class="status-grid">
      <!-- Carte uptime -->
      <div class="status-card">
        <div class="status-card-icon" style="background:var(--blue-dim)">
          <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="var(--blue)" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
            <circle cx="12" cy="12" r="10"/><polyline points="12 6 12 12 16 14"/>
          </svg>
        </div>
        <div class="status-card-body">
          <div class="status-card-label">Uptime</div>
          <div class="status-card-value" id="uptime-label">—</div>
        </div>
      </div>

      <!-- Carte températures -->
      <div class="status-card">
        <div class="status-card-icon" style="background:var(--orange-dim)">
          <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="var(--orange)" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
            <path d="M14 14.76V3.5a2.5 2.5 0 0 0-5 0v11.26a4 4 0 1 0 5 0z"/>
          </svg>
        </div>
        <div class="status-card-body">
          <div class="status-card-label">Températures</div>
          <div class="status-card-value" id="temps-label">T1: -- °C</div>
          <div class="status-card-sub" id="tempsR-label">TRem: -- °C</div>
        </div>
      </div>

      <!-- Carte conso globale -->
      <div class="status-card">
        <div class="status-card-icon" style="background:var(--green-dim)">
          <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="var(--green)" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
            <path d="M12 2.69l5.66 5.66a8 8 0 1 1-11.31 0z"/>
          </svg>
        </div>
        <div class="status-card-body">
          <div class="status-card-label">Volume total</div>
          <div class="status-card-value" id="litres-label">— L</div>
          <div class="status-card-sub">
            <span id="pulse-label">—</span> pulses ·
            <span style="color:var(--blue);font-weight:600" id="flow-label">— L/min</span>
          </div>
        </div>
      </div>

      <!-- Carte alarme hydraulique -->
      <div class="status-card" id="alarm-card">
        <div class="status-card-icon" id="alarm-icon" style="background:var(--green-dim)">
          <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="var(--green)" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
            <path d="M9 12l2 2 4-4"/>
            <circle cx="12" cy="12" r="10"/>
          </svg>
        </div>
        <div class="status-card-body">
          <div class="status-card-label">Alarme hydraulique</div>
          <div class="status-card-value" id="alarm-label" style="color:var(--green)">Aucune alarme</div>
          <div class="status-card-sub" id="alarm-sub">—</div>
        </div>
      </div>

      <!-- Carte VanneManuelle (débitmètre voit couler alors qu'aucune vanne auto n'est ouverte) -->
      <div class="status-card" id="manual-valve-card">
        <div class="status-card-icon" style="background:var(--blue-dim)">
          <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="var(--blue)" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
            <path d="M12 2.69l5.66 5.66a8 8 0 1 1-11.31 0z"/>
          </svg>
        </div>
        <div class="status-card-body">
          <div class="status-card-label">VanneManuelle</div>
          <div class="status-card-value" id="manual-valve-today">— L</div>
          <div class="status-card-sub">
            Total: <span id="manual-valve-total">—</span> L
            <span id="manual-valve-flow" style="color:var(--blue);font-weight:600;margin-left:6px">— L/min</span>
          </div>
          <div style="margin-top:6px">
            <button class="btn btn-ghost btn-sm" onclick="resetManualValve()" title="Remettre le compteur VanneManuelle à zéro">↻ RAZ</button>
          </div>
        </div>
      </div>

      <!-- Carte mémoire / santé — masquée (encombre la grille, infos
           redondantes avec les badges header WiFi/MQTT/WS)
      <div class="status-card">
        <div class="status-card-icon" style="background:var(--surface2)">
          <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="var(--text-muted)" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
            <rect x="4" y="4" width="16" height="16" rx="2"/>
            <rect x="9" y="9" width="6" height="6"/>
            <line x1="9" y1="1" x2="9" y2="4"/><line x1="15" y1="1" x2="15" y2="4"/>
            <line x1="9" y1="20" x2="9" y2="23"/><line x1="15" y1="20" x2="15" y2="23"/>
            <line x1="20" y1="9" x2="23" y2="9"/><line x1="20" y1="14" x2="23" y2="14"/>
            <line x1="1" y1="9" x2="4" y2="9"/><line x1="1" y1="14" x2="4" y2="14"/>
          </svg>
        </div>
        <div class="status-card-body">
          <div class="status-card-label">Santé ESP</div>
          <div class="status-card-value" id="heap-label">— KB</div>
          <div class="status-card-sub" id="wifi-label">WiFi —</div>
        </div>
      </div>
      -->
    </div>

    <!-- Tableau compact conso par vanne -->
    <div class="status-conso">
      <div class="status-conso-header">
        <span>💧 Consommation par vanne</span>
        <button class="btn btn-ghost btn-sm" onclick="refreshConsumption()" title="Actualiser">↻</button>
      </div>
      <table class="tbl status-conso-tbl">
        <thead>
          <tr>
            <th>Vanne</th>
            <th>Nom</th>
            <th style="text-align:right">Aujourd'hui</th>
            <th style="text-align:right">Total</th>
            <th style="text-align:right">Débit (live)</th>
          </tr>
        </thead>
        <tbody id="status-cons-body">
          <tr><td colspan="5" style="text-align:center;color:var(--text-muted);padding:10px">—</td></tr>
        </tbody>
      </table>
    </div>
  </div>

  <div class="valve-grid" id="valve-grid">
    <!-- cartes générées par JS -->
  </div>
</div>



)HTML";