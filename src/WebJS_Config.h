#pragma once
// ============================================================
// WebJS_Config.h — JavaScript Configuration
// ============================================================
#define WEB_JS_CONFIG R"JS(


<script>
// ══════════════════════════════════════════════════════════
// CONFIGURATION
// ══════════════════════════════════════════════════════════
function loadConfig() {
  api('GET', '/api/config').then(d => {
    sysConfig = d;
    renderConfig();
  }).catch(err => {
    console.error('Échec /api/config', err);
  });
}

function renderConfig() {
  const form = document.getElementById('config-form');
  if(!form || !sysConfig) return;
  // Remplit les champs avec les valeurs actuelles
  document.getElementById('cfg-device-name').value = sysConfig.deviceName || '';
  document.getElementById('cfg-hostname').value = sysConfig.hostname || '';
  document.getElementById('cfg-wifi-ssid').value = sysConfig.wifiSsid || '';
  document.getElementById('cfg-wifi-pass').value = ''; // Ne pas afficher le mot de passe
  document.getElementById('cfg-ntp-server').value = sysConfig.ntpServer || 'pool.ntp.org';
  document.getElementById('cfg-tz').value = sysConfig.tz || 'CET-1CEST,M3.5.0,M10.5.0/3';
  document.getElementById('cfg-mqtt-enable').checked = !!sysConfig.mqttEnable;
  document.getElementById('cfg-mqtt-server').value = sysConfig.mqttServer || '';
  document.getElementById('cfg-mqtt-port').value = sysConfig.mqttPort || 1883;
  document.getElementById('cfg-mqtt-user').value = sysConfig.mqttUser || '';
  document.getElementById('cfg-mqtt-pass').value = ''; // Ne pas afficher le mot de passe
  document.getElementById('cfg-mqtt-prefix').value = sysConfig.mqttPrefix || 'irrigation';
  document.getElementById('cfg-max-open').value = sysConfig.maxOpenSec || 3600;
  document.getElementById('cfg-manual-force').value = sysConfig.manualForceSec || 1800;
  document.getElementById('cfg-pulses-per-litre').value = sysConfig.pulsesPerLitre || 100;
  document.getElementById('cfg-ota-enable').checked = !!sysConfig.otaEnable;
  // Affiche/masque les sections MQTT
  document.getElementById('mqtt-section').style.display = sysConfig.mqttEnable ? 'block' : 'none';
}

function saveConfig() {
  const cfg = {
    deviceName: document.getElementById('cfg-device-name').value,
    hostname: document.getElementById('cfg-hostname').value,
    wifiSsid: document.getElementById('cfg-wifi-ssid').value,
    wifiPass: document.getElementById('cfg-wifi-pass').value,
    ntpServer: document.getElementById('cfg-ntp-server').value,
    tz: document.getElementById('cfg-tz').value,
    mqttEnable: document.getElementById('cfg-mqtt-enable').checked,
    mqttServer: document.getElementById('cfg-mqtt-server').value,
    mqttPort: parseInt(document.getElementById('cfg-mqtt-port').value),
    mqttUser: document.getElementById('cfg-mqtt-user').value,
    mqttPass: document.getElementById('cfg-mqtt-pass').value,
    mqttPrefix: document.getElementById('cfg-mqtt-prefix').value,
    maxOpenSec: parseInt(document.getElementById('cfg-max-open').value),
    manualForceSec: parseInt(document.getElementById('cfg-manual-force').value),
    pulsesPerLitre: parseFloat(document.getElementById('cfg-pulses-per-litre').value),
    otaEnable: document.getElementById('cfg-ota-enable').checked
  };
  api('POST', '/api/config/save', cfg)
    .then(d => {
      if(d && d.ok) {
        alert('Configuration enregistrée');
        loadConfig();
      } else {
        alert('Échec de l\'enregistrement');
      }
    })
    .catch(err => {
      console.error('Échec saveConfig', err);
      alert('Échec de l\'enregistrement');
    });
}

function exportConfig() {
  api('GET', '/api/config').then(d => {
    const blob = new Blob([JSON.stringify(d, null, 2)], {type: 'application/json'});
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'irrigation-config.json';
    a.click();
    URL.revokeObjectURL(url);
  }).catch(err => {
    console.error('Échec export config', err);
    alert('Échec de l\'export');
  });
}

function importConfigFromFile(event) {
  const file = event.target.files[0];
  if(!file) return;
  const reader = new FileReader();
  reader.onload = e => {
    try {
      const data = JSON.parse(e.target.result);
      if(!confirm('Importer cette configuration ?\nCela remplacera la configuration actuelle.')) return;
      api('POST', '/api/config/import', data)
        .then(d => {
          if(d && d.ok) {
            loadConfig();
            alert('Configuration importée');
          } else {
            alert('Échec de l\'import');
          }
        })
        .catch(err => {
          console.error('Échec import config', err);
          alert('Échec de l\'import');
        });
    } catch(err) {
      console.error('JSON invalide', err);
      alert('Fichier JSON invalide');
    }
  };
  reader.readAsText(file);
}

function toggleMqttSection(checkbox) {
  document.getElementById('mqtt-section').style.display = checkbox.checked ? 'block' : 'none';
}

function restartSystem() {
  if(!confirm('Redémarrer le système ?')) return;
  api('POST', '/api/system/restart')
    .then(d => {
      if(d && d.ok) {
        alert('Redémarrage en cours...');
        // Attend quelques secondes avant de recharger la page
        setTimeout(() => location.reload(), 5000);
      } else {
        alert('Échec du redémarrage');
      }
    })
    .catch(err => {
      console.error('Échec restartSystem', err);
      alert('Échec du redémarrage');
    });
}

function factoryReset() {
  if(!confirm('⚠️ Réinitialisation Usine ⚠️\n\nCela effacera TOUTES les données (calibrations, programmes, configuration).\n\nContinuer ?')) return;
  api('POST', '/api/system/factory_reset')
    .then(d => {
      if(d && d.ok) {
        alert('Réinitialisation en cours...\nLa page va se recharger dans quelques secondes.');
        setTimeout(() => location.reload(), 8000);
      } else {
        alert('Échec de la réinitialisation');
      }
    })
    .catch(err => {
      console.error('Échec factoryReset', err);
      alert('Échec de la réinitialisation');
    });
}
</script>
)JS"