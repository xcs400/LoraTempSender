#pragma once
// ============================================================
// WebJS_Programmes.h — JavaScript Programmes
// ============================================================
#define WEB_JS_PROGRAMMES R"JS(

<script>
// ══════════════════════════════════════════════════════════
// PROGRAMMES
// ══════════════════════════════════════════════════════════
function loadSchedules() {
  api(\'GET\', \'/api/schedules\').then(d => {
    schedules = (d && Array.isArray(d.schedules)) ? d.schedules : [];
    renderSchedules();
  }).catch(err => {
    console.error(\'Échec /api/schedules\', err);
    schedules = [];
    renderSchedules();
  });
}

function renderSchedules() {
  const tbody = document.getElementById(\'sched-body\');
  if (!tbody) return;
  if (!Array.isArray(schedules) || schedules.length === 0) {
    tbody.innerHTML = \'<tr><td colspan="8" style="text-align:center;color:var(--text-muted);padding:24px">Aucun programme</td></tr>\';
    return;
  }
  // Trie par vanne puis par heure
  schedules.sort((a,b) => {
    if (a.valve !== b.valve) return a.valve - b.valve;
    const [ha,ma] = [a.hour||0, a.minute||0];
    const [hb,mb] = [b.hour||0, b.minute||0];
    return (ha*60+ma) - (hb*60+mb);
  });
  tbody.innerHTML = schedules.map(s => {
    const vname = (valves[s.valve] && valves[s.valve].name) ? valves[s.valve].name : (\'V\'+s.valve);
    const time = (s.hour||0).toString().padStart(2,\'0\') + \':\' + (s.minute||0).toString().padStart(2,\'0\');
    const dur = fmtSec(s.durationSec || 0);
    // Jours : format lisible (Lun, Mar, etc.)
    const days = [];
    const dayNames = [\'Lun\',\'Mar\',\'Mer\',\'Jeu\',\'Ven\',\'Sam\',\'Dim\'];
    for(let i=0; i<7; i++) {
      if(s.weekDays & (1 << i)) days.push(dayNames[i]);
    }
    const daysStr = days.length === 7 ? \'Tous les jours\' : 
                   (days.length === 0 ? \'Jamais\' : days.join(\', \'));
    // Mode : hebdo / intervalle / saison
    let modeStr = \'Hebdo\';
    if(s.calMode === 1) modeStr = \'Intervalle\';
    else if(s.calMode === 2) modeStr = \'Saison\';
    const activeCls = s.active ? \'\' : \'inactive\';
    return \'<tr class="sched-row \' + activeCls + \'">\' +
      \'<td><strong>V\' + s.valve + \'</strong></td>\' +
      \'<td>\' + vname + \'</td>\' +
      \'<td>\' + time + \'</td>\' +
      \'<td>\' + dur + \'</td>\' +
      \'<td>\' + daysStr + \'</td>\' +
      \'<td>\' + modeStr + \'</td>\' +
      \'<td>\' +
        \'<div class="toggle compact">\' +
          \'<input type="checkbox" \' + (s.active?\'checked\':\'\') + \' onchange="toggleSchedule(\' + s.valve + \',\' + s.schedIdx + \',this.checked)">\' +
          \'<span class="toggle-slider"></span>\' +
        \'</div>\' +
      \'</td>\' +
      \'<td>\' +
        \'<button class="btn btn-ghost btn-sm" onclick="editSchedule(\' + s.valve + \',\' + s.schedIdx + \')">✏️ Éditer</button>\' +
        \'<button class="btn btn-red btn-sm" onclick="deleteSchedule(\' + s.valve + \',\' + s.schedIdx + \')" style="margin-left:4px">🗑️ Suppr</button>\' +
      \'</td>\' +
    \'</tr>\';
  }).join(\'\');
}

function toggleSchedule(valve, schedIdx, active) {
  api(\'POST\',\'/api/schedule/toggle\',{valve: valve, schedIdx: schedIdx, active: active})
    .then(()=>loadSchedules())
    .catch(err=>console.error(\'Échec toggleSchedule\',err));
}

function deleteSchedule(valve, schedIdx) {
  if(!confirm(\'Supprimer ce programme ?\')) return;
  api(\'POST\',\'/api/schedule/delete\',{valve: valve, schedIdx: schedIdx})
    .then(()=>loadSchedules())
    .catch(err=>console.error(\'Échec deleteSchedule\',err));
}

function editSchedule(valve, schedIdx) {
  const s = schedules.find(s => s.valve === valve && s.schedIdx === schedIdx);
  if(!s) return;
  openSchedModal();
  // Remplit le formulaire
  document.getElementById(\'sched-valve\').value = s.valve;
  document.getElementById(\'sched-name\').value = s.name || \'\';
  document.getElementById(\'sched-hour\').value = s.hour || 6;
  document.getElementById(\'sched-minute\').value = s.minute || 0;
  document.getElementById(\'sched-duration\').value = s.durationSec || 900;
  // Jours
  document.querySelectorAll(\'.day-btn\').forEach((btn, i) => {
    btn.classList.toggle(\'sel\', !!(s.weekDays & (1 << i)));
  });
  // Mode
  document.getElementById(\'sched-mode\').value = s.calMode || 0;
  document.getElementById(\'sched-interval-days\').value = s.intervalDays || 2;
  document.getElementById(\'sched-int-start\').value = (s.intervalStartMonth||1) + \'-\' + String(s.intervalStartDay||1).padStart(2,\'0\');
  document.getElementById(\'sched-season-start\').value = (s.seasonStartMonth||4) + \'-\' + String(s.seasonStartDay||1).padStart(2,\'0\');
  document.getElementById(\'sched-season-end\').value = (s.seasonEndMonth||10) + \'-\' + String(s.seasonEndDay||31).padStart(2,\'0\');
  // Active
  document.getElementById(\'sched-active\').checked = s.active;
  // Stocke les identifiants pour saveSched
  document.getElementById(\'sched-modal\').dataset.valve = s.valve;
  document.getElementById(\'sched-modal\').dataset.schedIdx = s.schedIdx;
  document.getElementById(\'sched-modal\').dataset.isNew = \'false\';
}

function openSchedModal() {
  schedModalOpen = true;
  // Reset formulaire
  document.getElementById(\'sched-valve\').value = 0;
  document.getElementById(\'sched-name\').value = \'\';
  document.getElementById(\'sched-hour\').value = 6;
  document.getElementById(\'sched-minute\').value = 0;
  document.getElementById(\'sched-duration\').value = 900;
  document.querySelectorAll(\'.day-btn\').forEach(btn => btn.classList.remove(\'sel\'));
  // Sélectionne Lun-Sam par défaut (bits 0-5)
  for(let i=0; i<6; i++) {
    document.querySelector(\'.day-btn:nth-child(\' + (i+1) + \')\').classList.add(\'sel\');
  }
  document.getElementById(\'sched-mode\').value = 0;
  document.getElementById(\'sched-interval-days\').value = 2;
  const now = new Date();
  document.getElementById(\'sched-int-start\').value = (now.getMonth()+1) + \'-\' + String(now.getDate()).padStart(2,\'0\');
  document.getElementById(\'sched-season-start\').value = \'4-01\';
  document.getElementById(\'sched-season-end\').value = \'10-31\';
  document.getElementById(\'sched-active\').checked = true;
  // Supprime les identifiants pour créer un nouveau
  delete document.getElementById(\'sched-modal\').dataset.valve;
  delete document.getElementById(\'sched-modal\').dataset.schedIdx;
  document.getElementById(\'sched-modal\').dataset.isNew = \'true\';
  // Affiche le modal
  document.getElementById(\'sched-modal\').classList.add(\'open\');
  // Force un refresh du select vanne pour être sûr qu\'il est à jour
  buildSchedValveSelect();
}

function closeSchedModal() {
  schedModalOpen = false;
  document.getElementById(\'sched-modal\').classList.remove(\'open\');
}

function saveSched() {
  const isNew = document.getElementById(\'sched-modal\').dataset.isNew === \'true\';
  const origV = isNew ? -1 : parseInt(document.getElementById(\'sched-modal\').dataset.valve);
  const origIdx = isNew ? -1 : parseInt(document.getElementById(\'sched-modal\').dataset.schedIdx);
  const v = parseInt(document.getElementById(\'sched-valve\').value);
  const name = document.getElementById(\'sched-name\').value.trim();
  const hour = parseInt(document.getElementById(\'sched-hour\').value);
  const minute = parseInt(document.getElementById(\'sched-minute\').value);
  const duration = parseInt(document.getElementById(\'sched-duration\').value);
  // Jours : convertit les boutons en bitmask (0=Lun, 6=Dim)
  let weekDays = 0;
  document.querySelectorAll(\'.day-btn\').forEach((btn, i) => {
    if(btn.classList.contains(\'sel\')) weekDays |= (1 << i);
  });
  const calMode = parseInt(document.getElementById(\'sched-mode\').value);
  const intervalDays = parseInt(document.getElementById(\'sched-interval-days\').value);
  const intStart = document.getElementById(\'sched-int-start\').value.split(\'-\');
  const intervalStartMonth = intStart[0] ? parseInt(intStart[0]) : 1;
  const intervalStartDay = intStart[1] ? parseInt(intStart[1]) : 1;
  const seasonStart = document.getElementById(\'sched-season-start\').value.split(\'-\');
  const seasonStartMonth = seasonStart[0] ? parseInt(seasonStart[0]) : 4;
  const seasonStartDay = seasonStart[1] ? parseInt(seasonStart[1]) : 1;
  const seasonEnd = document.getElementById(\'sched-season-end\').value.split(\'-\');
  const seasonEndMonth = seasonEnd[0] ? parseInt(seasonEnd[0]) : 10;
  const seasonEndDay = seasonEnd[1] ? parseInt(seasonEnd[1]) : 31;
  const active = document.getElementById(\'sched-active\').checked;
  const body = {
    valve: v,
    name: name,
    hour: hour,
    minute: minute,
    durationSec: duration,
    weekDays: weekDays,
    calMode: calMode,
    intervalDays: intervalDays,
    intervalStartMonth: intervalStartMonth,
    intervalStartDay: intervalStartDay,
    seasonStartMonth: seasonStartMonth,
    seasonStartDay: seasonStartDay,
    seasonEndMonth: seasonEndMonth,
    seasonEndDay: seasonEndDay,
    active: active
  };
  if(!isNew) {
    body.origValve = origV;
    body.schedIdx = origIdx;
  }
  api(\'POST\', \'/api/schedule/save\', body)
    .then(d => {
      if(d && d.ok) {
        closeSchedModal();
        loadSchedules();
      } else {
        alert(\'Échec de l\\\'enregistrement\');
      }
    })
    .catch(err => {
      console.error(\'Échec saveSched\', err);
      alert(\'Échec de l\\\'enregistrement\');
    });
}

function exportSchedules() {
  api(\'GET\', \'/api/schedules\').then(d => {
    const blob = new Blob([JSON.stringify(d, null, 2)], {type: \'application/json\'});
    const url = URL.createObjectURL(blob);
    const a = document.createElement(\'a\');
    a.href = url;
    a.download = \'irrigation-schedules.json\';
    a.click();
    URL.revokeObjectURL(url);
  }).catch(err => {
    console.error(\'Échec export\', err);
    alert(\'Échec de l\\\'export\');
  });
}

function importSchedulesFromFile(event) {
  const file = event.target.files[0];
  if(!file) return;
  const reader = new FileReader();
  reader.onload = e => {
    try {
      const data = JSON.parse(e.target.result);
      if(!confirm(\'Importer \' + (Array.isArray(data) ? data.length : \'ces\') + \' programmes ?\\nCela remplacera TOUS les programmes actuels.\')) return;
      api(\'POST\', \'/api/schedules/import\', data)
        .then(d => {
          if(d && d.ok) {
            loadSchedules();
            alert(\'Programmes importés\');
          } else {
            alert(\'Échec de l\\\'import\');
          }
        })
        .catch(err => {
          console.error(\'Échec import\', err);
          alert(\'Échec de l\\\'import\');
        });
    } catch(err) {
      console.error(\'JSON invalide\', err);
      alert(\'Fichier JSON invalide\');
    }
  };
  reader.readAsText(file);
}
</script>
)JS"