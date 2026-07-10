#pragma once
// ============================================================
// WebJS_Journal.h — JavaScript Journal
// ============================================================
#define WEB_JS_JOURNAL R"JS(

<script>
// ══════════════════════════════════════════════════════════
// JOURNAL
// ══════════════════════════════════════════════════════════
function loadLogs() {
  api('GET', '/api/logs').then(d => {
    logs = d && Array.isArray(d.logs) ? d.logs : [];
    renderLogs();
  }).catch(err => {
    console.error('Échec /api/logs', err);
    logs = [];
    renderLogs();
  });
}

function renderLogs() {
  const container = document.getElementById('logs-container');
  if (!container) return;
  if (!Array.isArray(logs) || logs.length === 0) {
    container.innerHTML = '<div style="text-align:center;color:var(--text-muted);padding:24px">Aucun log</div>';
    return;
  }
  // Trie par timestamp décroissant
  logs.sort((a,b) => b.timestamp - a.timestamp);
  container.innerHTML = logs.map(log => {
    const d = new Date(log.timestamp * 1000);
    const dateStr = d.toLocaleString('fr-FR');
    const level = log.level || 'INFO';
    const cls = level === 'ERROR' ? 'log-error' : 
               level === 'WARN' ? 'log-warn' : 
               level === 'DEBUG' ? 'log-debug' : 'log-info';
    return '<div class="log-entry ' + cls + '">' +
      '<div class="log-date">' + dateStr + '</div>' +
      '<div class="log-level">' + level + '</div>' +
      '<div class="log-message">' + (log.message||'') + '</div>' +
      '</div>';
  }).join('');
}

function clearLogs() {
  if(!confirm('Effacer tous les logs ?')) return;
  api('POST', '/api/logs/clear')
    .then(d => {
      if(d && d.ok) {
        loadLogs();
      } else {
        alert('Échec de l\'effacement');
      }
    })
    .catch(err => {
      console.error('Échec clearLogs', err);
      alert('Échec de l\'effacement');
    });
}

function exportLogs() {
  api('GET', '/api/logs').then(d => {
    const blob = new Blob([JSON.stringify(d, null, 2)], {type: 'application/json'});
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'irrigation-logs.json';
    a.click();
    URL.revokeObjectURL(url);
  }).catch(err => {
    console.error('Échec export logs', err);
    alert('Échec de l\'export');
  });
}
</script>
)JS"