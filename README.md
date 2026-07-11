# LoRa Node Example for Heltec ESP32 wifi Lora module  (V2 or V2.1  accordind to the setting)
The program reads the  temperature of a DS18B20 , packages the data into a JSON format, and sends it over LoRa then goes to deep sleep for 2mn.
the vBatt voltage is also reported.

The SSD1306 OLED display is only use once to save battery.
The serial output (at 115200 baud rate) will report the JSON formatted data being sent ( only once to save battery)

On my Heltec 2,7ma are drawn from the battery during the deep sleep mode  (and 80-100ma during the wake up and ADC read and lora sending) 

OPENMQTTgateway is used on anoter modul to send the json to a MQTTbroker

## Hardware :
* ESP32  wifi Lora module , include:
    * SX12XX LoRa module.       (adapt region  #define BAND 868E6)
    * SSD1306 OLED display.
* DS18B20      is attached on pin 23 on the ESP (onewire)

## Pin Configuration (used on Heltec Module):
* SCK  - GPIO5
* MISO - GPIO19
* MOSI - GPIO27
* SS   - GPIO18
* RST  - GPIO14
* DI0  - GPIO26

### Software Setup:

* Clone this repository.
* Open the provided node program with PlatformIO  (use of Ardwino env)
* Upload the program to your ESP32.


## Data Format:
The data is sent in the following JSON format:

```json
{"model":"ESP32TEMP","id":"B0:B2:1C:F8:73:94","TempCelsius":14.31,"Elapsed":0,"Vbatt":4112,"Charge%":91}
```

---

## Architecture Web — Streaming de la SPA par chunks PROGMEM

### Le problème

La page d'accueil de l'interface (`/`) pèse **~50 KB** en HTML/CSS/JS. Si on la concatène en `String` côté firmware pour l'envoyer en une fois, on échoue à coup sûr : la heap de l'ESP32 est **fragmentée** après les allocations réseau (WiFi, AsyncWebServer, WebSocket) et il n'y a plus de bloc libre assez grand pour contenir 50 KB d'un coup. Résultat : **page blanche** côté navigateur, sans erreur visible.

### La solution : le streaming par chunks PROGMEM

Au lieu d'un seul gros `String`, on découpe la SPA en **blocs indépendants** stockés en flash (`PROGMEM`) et on les stream vers le navigateur **morceau par morceau** via le callback de `beginResponse()`.

Tout est orchestré dans [`webSetup()`](src/Iocan/WebManager.h) (handler `GET /`).

### Table des chunks

| Ordre | Symbole | Fichier source | Contenu |
|------:|---------|----------------|---------|
| 0 | `WEB_HTML0` | `WebContent.h` | `<!DOCTYPE html>`, `<head>` ouvrant, balise `<style>` ouvrante |
| 1 | `WEB_CSS` | `WebCSS.h` | Tout le CSS de l'interface |
| 2 | `WEB_HTML1` | `WebContent.h` | `</style>`, `</head>`, `<body>`, `<header>`, `<nav>`, `<main>` ouvrant |
| 3 | `WEB_DASHBOARD_HTML` | `WebHTML_Dashboard.h` | Page Dashboard |
| 4 | `WEB_PROGRAMMES_HTML` | `WebHTML_Programmes.h` | Page Programmes |
| 5 | `WEB_CALIBRATION_HTML` | `WebHTML_Calibration.h` | Page Calibration |
| 6 | `WEB_CONFIG_HTML` | `WebHTML_Config.h` | Page Configuration |
| 7 | `WEB_EOS_HTML` | `WebHTML_EOS.h` | Page Entrées/Sorties |
| 8 | `WEB_CALENDRIER_HTML` | `WebHTML_Calendrier.h` | Page Calendrier |
| 9 | `WEB_JOURNAL_HTML` | `WebHTML_Journal.h` | Page Journal |
| 10 | `WEB_HTML2` | `WebContent.h` | `</main>`, `</div>`, modaux (forçage, programme), `<script>` ouvrant |
| 11 | `WEB_PROGRAMMESMODAL_HTML` | `WebContent.h` | (réservé — extensions futures) |
| 12 | `WEB_JS` | `WebContent.h` | Tout le JavaScript de l'interface, `</script>`, `</body>`, `</html>` |

**Au total : 13 chunks.** Tous définis comme `const char[] PROGMEM` (jamais recopiés en RAM).

### Mécanisme de streaming

```cpp
// src/Iocan/WebManager.h → webSetup()
static const char* chunks[] = { WEB_HTML0, WEB_CSS, WEB_HTML1, ... };
static const size_t nbChunks = sizeof(chunks) / sizeof(chunks[0]);

// Précalcul des longueurs UNE SEULE fois (au 1er GET /)
static size_t lens[nbChunks];
static size_t totalLen = 0;
static bool initDone = false;
if (!initDone) {
    for (size_t i = 0; i < nbChunks; i++) {
        lens[i] = strlen_P(chunks[i]);   // scanne la flash, pas la RAM
        totalLen += lens[i];
    }
    initDone = true;
}

AsyncWebServerResponse* resp = req->beginResponse(
    "text/html; charset=utf-8",
    totalLen,                              // Content-Length exact
    [chunkIndex, offsetInChunk](uint8_t *buffer, size_t maxLen, size_t) {
        // Appelé plusieurs fois par AsyncWebServer, maxLen ≈ 1-4 KB
        size_t written = 0;
        while (written < maxLen && *chunkIndex < nbChunks) {
            const char* src = chunks[*chunkIndex];
            size_t remaining = lens[*chunkIndex] - *offsetInChunk;
            size_t toCopy = std::min(remaining, maxLen - written);
            memcpy_P(buffer + written, src + *offsetInChunk, toCopy);
            written += toCopy;
            *offsetInChunk += toCopy;
            if (*offsetInChunk >= lens[*chunkIndex]) {
                *chunkIndex += 1;     // chunk suivant
                *offsetInChunk = 0;
            }
        }
        return written;              // 0 = fin du flux
    }
);
```

**Points clés :**

- `strlen_P()` et `memcpy_P()` lisent **directement depuis la flash** (PROGMEM), sans jamais copier le HTML en RAM.
- `totalLen` est calculé une fois pour toutes (tableau `static`), donc le `Content-Length` HTTP est correct dès la 1ère réponse.
- Le couple `(chunkIndex, offsetInChunk)` est encapsulé dans un `std::shared_ptr<size_t>` capturé par la lambda : **chaque requête HTTP a sa propre progression**, donc 2 clients simultanés ne se marchent pas dessus.
- Ajouter un chunk = 1 ligne dans le tableau + 1 `#include "WebHTML_XXX.h"`. Aucune autre modification du `webSetup` n'est nécessaire.

### Ordre des pages dans la SPA

L'ordre de la table `chunks[]` reflète **l'ordre d'apparition dans le DOM** de chaque page :

```
<main>
  ┌─ WEB_DASHBOARD_HTML     → page-dashboard   (active par défaut)
  ├─ WEB_PROGRAMMES_HTML    → page-programmes
  ├─ WEB_CALIBRATION_HTML   → page-calibration
  ├─ WEB_CONFIG_HTML        → page-config
  ├─ WEB_EOS_HTML           → page-io
  ├─ WEB_CALENDRIER_HTML    → page-calendrier
  └─ WEB_JOURNAL_HTML       → page-journal
</main>
```

**L'ordre DOM est important** : le navigateur parse le HTML séquentiellement. Les `<script>` et les références JS (ex. `getElementById('page-config')`) doivent trouver les éléments correspondants **après** leur déclaration dans le flux. L'ordre actuel respecte cette contrainte.

> Note : l'ordre dans le tableau `chunks[]` ne suit pas exactement l'ordre ci-dessus pour optimiser l'assemblage (les chunks "cadres" `WEB_HTML0/1/2` et `WEB_CSS`/`WEB_JS` encadrent les pages). Le résultat DOM final est identique.

### Ajouter une nouvelle page

1. Créer `src/WebHTML_NewPage.h` :
   ```cpp
   #pragma once
   const char WEB_NEWPAGE_HTML[] PROGMEM = R"HTML(
   <div id="page-newpage" class="page">
     ...
   </div>
   )HTML";
   ```

2. Dans `WebContent.h`, insérer `#include "WebHTML_NewPage.h"` entre les chunks qui doivent l'encadrer.

3. Dans `WebManager.h → webSetup()`, ajouter `WEB_NEWPAGE_HTML` au tableau `chunks[]` à la position voulue.

4. Ajouter le bouton correspondant dans `<nav>` (chunk `WEB_HTML1`).

C'est tout. Le `strlen_P` et le `Content-Length` se mettent à jour automatiquement.
