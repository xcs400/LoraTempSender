#pragma once


#define WEB_JS_CALENDRIER R"JS(


<script>
// ══════════════════════════════════════════════════════════
// CALENDRIER
// ══════════════════════════════════════════════════════════
function loadCalendar() {
  api('GET', '/api/calendar').then(d => {
    calendarEvents = d && Array.isArray(d.events) ? d.events : [];
    renderCalendar();
  }).catch(err => {
    console.error('Échec /api/calendar', err);
    calendarEvents = [];
    renderCalendar();
  });
}

function renderCalendar() {
  const container = document.getElementById('calendar-container');
  if (!container) return;
  if (!Array.isArray(calendarEvents) || calendarEvents.length === 0) {
    container.innerHTML = '<div style="text-align:center;color:var(--text-muted);padding:24px">Aucun événement</div>';
    return;
  }
  // Trie par date décroissante
  calendarEvents.sort((a,b) => (b.date||'')+(b.hour||0)+(b.minute||0) - (a.date||'')+(a.hour||0)+(a.minute||0));
  container.innerHTML = calendarEvents.map(e => {
    const vname = (valves[e.valve] && valves[e.valve].name) ? valves[e.valve].name : ('V'+e.valve);
    const time = (e.hour||0).toString().padStart(2,'0') + ':' + (e.minute||0).toString().padStart(2,'0');
    const dur = fmtSec(e.durationSec || 0);
    const type = e.type === 'manual' ? 'manuel' : (e.type === 'sched' ? 'programmé' : e.type);
    const cls = e.type === 'manual' ? 'cal-manual' : (e.type === 'sched' ? 'cal-sched' : '');
    return '<div class="calendar-event ' + cls + '">' +
      '<div class="cal-date">' + (e.date||'—') + '</div>' +
      '<div class="cal-time">' + time + '</div>' +
      '<div class="cal-valve">V' + e.valve + ' (' + vname + ')</div>' +
      '<div class="cal-duration">' + dur + '</div>' +
      '<div class="cal-type">' + type + '</div>' +
    '</div>';
  }).join('');
}

function exportCalendar() {
  api('GET', '/api/calendar').then(d => {
    const blob = new Blob([JSON.stringify(d, null, 2)], {type: 'application/json'});
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'irrigation-calendar.json';
    a.click();
    URL.revokeObjectURL(url);
  }).catch(err => {
    console.error('Échec export calendrier', err);
    alert('Échec de l\'export');
  });
}

function importCalendarFromFile(event) {
  const file = event.target.files[0];
  if(!file) return;
  const reader = new FileReader();
  reader.onload = e => {
    try {
      const data = JSON.parse(e.target.result);
      if(!confirm('Importer ' + (Array.isArray(data.events) ? data.events.length : 'ces') + ' événements ?\nCela remplacera TOUS les événements actuels.')) return;
      api('POST', '/api/calendar/import', data)
        .then(d => {
          if(d && d.ok) {
            loadCalendar();
            alert('Événements importés');
          } else {
            alert('Échec de l\'import');
          }
        })
        .catch(err => {
          console.error('Échec import calendrier', err);
          alert('Échec de l\'import');
        });
    } catch(err) {
      console.error('JSON invalide', err);
      alert('Fichier JSON invalide');
    }
  };
  reader.readAsText(file);
}
</script>
)JS"