#pragma once
#ifdef IOCAN
// ============================================================
// OledManager.h — Affichage écran OLED (4 pages)
// ============================================================
// Correspond à la SECTION 13 du fichier d'origine.
// ============================================================

#include "Globals.h"

inline void oledUpdate(){
    unsigned long now = millis();
    if(now - lastOledMs < 2000 && lastOledMs != 0) return;
    lastOledMs = now;

    unsigned long currentTotalPulses = persistedPulseCount;
    noInterrupts(); currentTotalPulses += pulseCount; interrupts();

    bool flowActive = computeFlowLpm(0) > 0.01f;
    if(flowActive && (oledPage != 4 || oledFlowPageStartPulseCount == 0)){
        oledPage = 4;
        oledPreferredPage = 4;
        oledLastActivityMs = now;
        oledFlowPageStartPulseCount = currentTotalPulses;
    } else if(!flowActive && now - oledLastActivityMs > OLED_SCREENSAVER_TIMEOUT_MS && oledPage != 5){
        oledPage = 5;
    }


    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);

    if (oledPage == 0) {
        if (captivePortalActive) {
            display.drawString(0,  0, "!! WiFi FAIL !!");
            display.drawString(0, 14, "AP: IrrigPro-Setup");
            display.drawString(0, 28, "-> 192.168.4.1");
            display.drawString(0, 42, "Config WiFi requise");
        } else {
            display.drawString(0,0, "-Reseau WiFi:");
            display.drawString(0,14, WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "Deconnecte");
            // Afficher date et heure si synchronisé
            struct tm ti;
            if(getLocalTime(&ti,5) && timeIsSynced){
                char dateBuf[24];
                char timeBuf[16];
                strftime(dateBuf, sizeof(dateBuf), "%Y-%m-%d", &ti);
                strftime(timeBuf, sizeof(timeBuf), "%H:%M:%S", &ti);
                display.drawString(0,28, String(dateBuf));
                display.drawString(0,40, String("Heure: ") + String(timeBuf));
                // restore visualization LEDs to reflect valve states
                for(int i=0;i<VANNE_COUNT;i++){
                    digitalWrite(LEDVISU_PINS[i], valves[i].isOpen ? HIGH : LOW);
                }
                // show temporary confirmation message after first sync
                if(ntpSyncedAtMs && millis() - ntpSyncedAtMs < 5000){
                    display.drawString(0,52, "NTP OK — vannes auto actives");
                }
            } else {
                display.drawString(0,28, "Heure: inconnue (NTP non sync)");
                // Clignoter LEDs de visualisation comme alarme (ne pas activer/fermer vannes)
                bool blink = ((millis() / 500) & 1) == 0;
                for(int i=0;i<VANNE_COUNT;i++){
                    digitalWrite(LEDVISU_PINS[i], blink ? HIGH : LOW);
                }
            }
        }
    } else if (oledPage == 1) {
        display.drawString(0,0, "Temperatures:");
        display.drawString(0,14, "Temp1: " + String(temperature1) + " C");
        // Temp2 removed; no longer display TempR on OLED (freed for pulse info below)
    } else if (oledPage == 2) {
        // Ligne 0: uptime
        display.drawString(0,0, "Uptime: " + String(now/60000) + "min");

        // Ligne 1..N: état vannes 2 par 2 (calculer dynamiquement selon VANNE_COUNT)
        int rows = (VANNE_COUNT + 1) / 2;
        for(int row=0; row<rows; row++){
            int i1 = row*2;
            int i2 = row*2 + 1;
            String s1 = "";
            String s2 = "";
            if(i1 < VANNE_COUNT) s1 = String(i1+1) + (valves[i1].isOpen?":ON ":":-- ");
            if(i2 < VANNE_COUNT) s2 = String(i2+1) + (valves[i2].isOpen?":ON":":--");
            display.drawString(0, 12 + row*13, s1 + s2);
        }
        // Afficher températures à la place des entrées
        String t1 = (isnan(temperature1) ? String("T1: -- °C") : String("T1:")+String(temperature1,2)+" °C");
        display.drawString(0,40, t1);
        // Ligne 5: LoRa info
        display.drawString(0,52,"LoRa rx:"+String(loraRxCount)+" rssi:"+String((int)loraRssi));
    } else if (oledPage == 3) {
        // Compact 4-line I/O table: header + oPD + oPA + Ipx
        // Header with column numbers
        display.drawString(0, 0, "----0 1 2 3 4 5 6 7");

        // Row oPD: PD0..PD7 (VANNE_PINS -> PD0..PD3, OUT_PINS -> PD4..PD7)
        String rowPD = "oPD:";
        for(int col=0; col<8; col++){
            int val = -1;
            if(col < 4){
                if(col < VANNE_COUNT) val = digitalRead(VANNE_PINS[col]);
            } else {
                int idx = col - 4;
                if(idx < (int)(sizeof(OUT_PINS)/sizeof(OUT_PINS[0]))) val = digitalRead(OUT_PINS[idx]);
            }
            if(val < 0) rowPD += "  "; else rowPD += (val==HIGH?" 1":" 0");
        }
        display.drawString(0, 12, rowPD);

        // Row oPA: visualization LEDs (only present for first VANNE_COUNT columns)
        String rowPA = "oPA:";
        for(int col=0; col<8; col++){
            if(col < VANNE_COUNT){
                int v = digitalRead(LEDVISU_PINS[col]);
                rowPA += (v==HIGH?" 1":" 0");
            } else {
                rowPA += "  ";
            }
        }
        display.drawString(0, 36, rowPA);

        // Row Ipx: inputs PB0..PB7 (FORCE_INPUT_PINS)
        String rowI = "iPB :";
        for(int col=0; col<8; col++){
            if(col < INPUTCOUNT){
                int v = digitalRead(FORCE_INPUT_PINS[col]);
                rowI += (v==HIGH?" 1":" 0");
            } else {
                rowI += "  ";
            }
        }
        display.drawString(0, 24, rowI);
        // Ligne supplémentaire: compteur d'impulsions (connecté sur PB7)
        unsigned long cnt;
        noInterrupts(); cnt = pulseCount; interrupts();
        float litres = (float)cnt / PULSES_PER_LITRE;
        display.drawString(0, 48, String("Pulse:") + String(cnt) + " L:" + String(litres,3));
    } else if (oledPage == 4) {
        display.setTextAlignment(TEXT_ALIGN_CENTER);

        display.setFont(ArialMT_Plain_24);
        float flowLpm = computeFlowLpm(0);
        display.drawString(64, 2, String(flowLpm, 1));

        display.setFont(ArialMT_Plain_10);
        display.drawString(64, 32, "L/min");

        display.setFont(ArialMT_Plain_16);
        display.drawString(64, 46, String((float)(currentTotalPulses - oledFlowPageStartPulseCount) / PULSES_PER_LITRE, 1) + " L");

    } else if (oledPage == 5) {
        display.setFont(ArialMT_Plain_10);

        for(int i=0;i<4;i++){
            int x1 = random(0, 128);
            int y1 = random(0, 64);
            int x2 = random(0, 128);
            int y2 = random(0, 64);
            display.drawLine(x1, y1, x2, y2);
        }
    
    //    display.drawRect(random(20, 90), random(10, 45), 18 + random(0, 20), 12 + random(0, 20));
    
    }
    display.display();
}

#endif // IOCAN
