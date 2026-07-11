#pragma once
// ============================================================
// WebHTML_EOS.h — Page Entrées/Sorties HTML
// ============================================================

const char WEB_EOS_HTML[] PROGMEM = R"HTML(


<!-- ══ PAGE E/S ═════════════════════════════════════ -->
<div id="page-io" class="page">
  <div class="card">
    <div class="card-header">
      <h2>Entr&eacute;es / Sorties Mat&eacute;rielles</h2>
    </div>
    <div style="overflow-x: auto; padding: 16px;">
      <table class="tbl" style="font-family: monospace; text-align: center; width:100%; max-width:600px; margin:0 auto">
        <thead>
          <tr>
            <th style="text-align:left; width:60px">Pin</th>
            <th>0</th><th>1</th><th>2</th><th>3</th><th>4</th><th>5</th><th>6</th><th>7</th>
          </tr>
        </thead>
        <tbody id="io-body">
          <tr><td colspan="9" style="text-align:center;color:var(--text-muted);padding:24px">En attente des donn&eacute;es...</td></tr>
        </tbody>
      </table>
      <div style="margin-top: 24px; font-size: 0.85rem; color: var(--text-muted); max-width:600px; margin-left:auto; margin-right:auto; padding:12px; background:var(--surface2); border-radius:6px">
        <strong style="color:var(--text)">L&eacute;gende :</strong><br>
        <div style="margin-top:6px"><strong>oPD</strong> : Sorties Puissance (0-3: Vannes, 4-7: Sorties auxiliaires)</div>
        <div><strong>oPA</strong> : Sorties LEDs visualisation (0-3: État des vannes)</div>
        <div><strong>In</strong> : Entr&eacute;es for&ccedil;age manuel (Boutons poussoirs)</div>
      </div>
    </div>
  </div>
</div>

)HTML";