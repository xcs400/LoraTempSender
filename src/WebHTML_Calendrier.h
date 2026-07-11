#pragma once
// ============================================================
// WebHTML_Calendrier.h — Page Calendrier HTML
// ============================================================


const char WEB_CALENDRIER_HTML[] PROGMEM = R"HTML(

<!-- ══ PAGE CALENDRIER ════════════════════════════════ -->
<div id="page-calendrier" class="page">
  <div class="card" style="padding:20px">
    <div class="cal-nav">
      <button class="btn btn-ghost btn-sm" onclick="calPrev()">◀</button>
      <h2 id="cal-title">Juin 2024</h2>
      <button class="btn btn-ghost btn-sm" onclick="calNext()">▶</button>
    </div>
    <div class="cal-grid" id="cal-grid">
      <!-- généré JS -->
    </div>
  </div>
</div>
)HTML";

