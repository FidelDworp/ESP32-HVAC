╔══════════════════════════════════════════════════════════════════════╗
║                   ESP32C6_HVAC - TEST CHECKLIST                      ║
╚══════════════════════════════════════════════════════════════════════╝
═══════════════════════════════════════════════════════════════════════
COMPILATIE & UPLOAD
═══════════════════════════════════════════════════════════════════════

□ Deel 1-5 samengevoegd in Arduino IDE (in volgorde!)
□ Sketch compileert zonder errors
□ Upload naar ESP32-C6 geslaagd
□ Serial Monitor (115200 baud) toont startup messages
□ "HVAC Controller V53.1" zichtbaar
□ WiFi verbindt binnen 20s per poging
□ IP adres zichtbaar in Serial Monitor


═══════════════════════════════════════════════════════════════════════
TEST 1: JSON FORMAAT (ongewijzigd t.o.v. V53)
═══════════════════════════════════════════════════════════════════════

Ga naar: http://hvac.local/json (of http://192.168.1.36/json)

□ eco_online staat VOORAAN
□ R9 aanwezig (SCH pomp status: 0 of 1)
□ R10 aanwezig (WON pomp status: 0 of 1)
□ "Vent" aanwezig (achter HeatDem)
□ ET = eco_max_temp (80.0 default)
□ EB = eco_min_temp (60.0 default)
□ KW waarden = 0.0 (placeholders)

VOORBEELD JSON:
{
  "eco_online": 0,
  "KSTopH": 25.3, ... "KSQtot": -8.5,
  "KWTopH": 0.0, ... "KWQtot": 0.0,
  "EAv": 0.0, "EQtot": 0.0, "ET": 80.0, "EB": 60.0,
  "BB": 0, ... "IK": 0,
  "R1": 0, ... "R7": 0, "R9": 0, "R10": 0,
  "HeatDem": 0.0,
  "Vent": 0
}


═══════════════════════════════════════════════════════════════════════
TEST 2: ECO SECTIE UI - ALTIJD TONEN
═══════════════════════════════════════════════════════════════════════

Ga naar: http://hvac.local/

**Met ECO OFFLINE:**

□ ECO sectie titel = "ECO BOILER (Solar + Haarden)"
□ Rode badge = "OFFLINE" (geen extra tekst in titel!)
□ ETop (Hoogste temp): "NA" (rood)
□ EQtot (Energie): "NA" (rood)
□ ET (Tmax pomp): "80.0 °C" (grijs - setting!)
□ EB (Tmin pomp): "60.0 °C" (grijs - setting!)
□ Alle 4 velden ZICHTBAAR

**Met ECO ONLINE:** (test zodra je ECO controller hebt)

□ Groene badge = "ONLINE"
□ ETop (Hoogste temp): [groene waarde] °C
□ EQtot (Energie): [groene waarde] kWh
□ ET (Tmax pomp): "80.0 °C"
□ EB (Tmin pomp): "60.0 °C"


═══════════════════════════════════════════════════════════════════════
TEST 3: POMP CONTROLS - ON/OFF BUTTONS
═══════════════════════════════════════════════════════════════════════

**SCH Pomp:**

□ Zie je twee knoppen: [ON] [OFF]
□ Klik [ON] → Serial Monitor toont "ECO: SCH pump ON (manual)"
□ Check: pomp gaat aan (relay 8 LOW)
□ Klik [OFF] → Serial Monitor toont "ECO: SCH pump OFF (manual)"
□ Check: pomp gaat direct uit (relay 8 HIGH)
□ OFF werkt ook TIJDENS pompwerking (geen countdown!)

**WON Pomp:**

□ Zie je twee knoppen: [ON] [OFF]
□ Klik [ON] → Serial Monitor toont "ECO: WON pump ON (manual)"
□ Check: pomp gaat aan (relay 9 LOW)
□ Klik [OFF] → Serial Monitor toont "ECO: WON pump OFF (manual)"
□ Check: pomp gaat direct uit (relay 9 HIGH)
□ OFF werkt ook TIJDENS pompwerking

**Totalen & Events:**

□ Zie je "Laatste: [datum tijd] - [X.XX kWh]" onder elke pomp
□ Zie je "TOTAAL: [XXX.X kWh]" onder elke pomp
□ Als eerste keer: "Laatste: NA" en "TOTAAL: 0.0 kWh"


═══════════════════════════════════════════════════════════════════════
TEST 4: POMP LOGICA - START/STOP CONDITIES
═══════════════════════════════════════════════════════════════════════

**LET OP:** Deze test kun je pas doen met een werkende ECO controller!

**START conditie (OR logica):**
□ Pomp start als: temp_top > 80°C (Tmax)
□ OF: qtot > 12 kWh (threshold)
□ Serial: "ECO: Starting [WON/SCH] pump (30 min)"

**STOP conditie (OR logica):**
□ Pomp stopt als: temp_top < 60°C (Tmin)
□ OF: qtot < 10 kWh (threshold - hysteresis)
□ Serial: "ECO: Stopping [WON/SCH] pump (conditie bereikt)"


═══════════════════════════════════════════════════════════════════════
TEST 5: POMP FLOW - 30 MIN AFWISSELEND
═══════════════════════════════════════════════════════════════════════

**LET OP:** Deze test duurt lang! Check Serial Monitor.

□ Cycle start: WON of SCH (afwisselend)
□ Pomp draait 30 minuten
□ Serial: "ECO: [WON/SCH] pump finished (30 min)"
□ Wacht-fase: 30 minuten (beide pompen UIT)
□ Serial: "ECO: Wait period finished (30 min)"
□ Check START conditie nog nodig?
  → JA: Start andere pomp (wissel!)
  → NEE: "ECO: No more pumping needed" → IDLE

**Afwisseling:**
□ WON → wacht → SCH → wacht → WON → wacht → SCH...
□ Eerlijke verdeling!


═══════════════════════════════════════════════════════════════════════
TEST 6: POMP TRACKING - EVENTS & TOTALEN
═══════════════════════════════════════════════════════════════════════

**Na automatische pompwerking:**

□ "Laatste" update met datum/tijd + kWh
□ "TOTAAL" verhoogt met gepompte kWh
□ Totalen blijven bewaard na reboot (NVS)
□ Serial: "Saved [SCH/WON] pump event: X.XX kWh (totaal: XXX.X kWh)"

**Check NVS persistentie:**
□ Reboot ESP32 (power cycle)
□ Check: totalen nog correct na reboot
□ Check: laatste event timestamp nog correct


═══════════════════════════════════════════════════════════════════════
TEST 7: SETTINGS PAGINA
═══════════════════════════════════════════════════════════════════════

Ga naar: http://hvac.local/settings

□ "ECO Max Temp (°C)" veld zichtbaar
□ Default waarde = 80.0
□ "ECO Min Temp (°C)" veld zichtbaar
□ Default waarde = 60.0
□ Wijzig Tmax naar 75°C en save
□ Check na reboot: waarde blijft 75°C
□ Check /json: ET = 75.0


═══════════════════════════════════════════════════════════════════════
TEST 8: WIFI TIMEOUT - IPHONE HOTSPOT
═══════════════════════════════════════════════════════════════════════

**iPhone hotspot test:**

□ Zet iPhone hotspot aan
□ ESP32 configureren met hotspot SSID
□ Reboot ESP32
□ Serial Monitor: elke poging duurt 20 seconden (was 10s)
□ Serial Monitor: max 5 pogingen
□ Totaal max wachttijd: ~100 seconden
□ Check: verbindt binnen 1-2 pogingen (niet meer 3x resetten!)

**Serial output verwacht:**
```
Connecting to 'iPhone van Gebruiker'...
....................
✓ WiFi connected!
WiFi connected: 192.168.x.x
RSSI: -45 dBm
```


═══════════════════════════════════════════════════════════════════════
TEST 9: UI DETAILS
═══════════════════════════════════════════════════════════════════════

**Button spacing:**
□ Op brede pagina (>600px): knoppen hebben ruimte tussen
□ Status / OTA / JSON / Settings knoppen netjes uitgelijnd
□ Op smalle pagina (<600px): knoppen werken nog steeds

**ECO sectie kleur:**
□ ECO titel = BLAUW (niet groen!)
□ Background: #336699
□ Badge: groen (ONLINE) of rood (OFFLINE)

**Labels:**
□ "ETop" = "ETop (Hoogste temp)"
□ "ET" = "ET (Tmax pomp)"
□ "EB" = "EB (Tmin pomp)"
□ Labels duidelijk en consistent


═══════════════════════════════════════════════════════════════════════
VERSIE VERSCHILLEN - V53 vs V53.1
═══════════════════════════════════════════════════════════════════════

WAS (V53):
❌ EAv (gemiddelde temp) getoond
❌ ECO data alleen als ONLINE
❌ "OFFLINE" in blauwe balk titel
❌ Pomp toggles (switch UI)
❌ Toggles niet uitschakelbaar tijdens countdown
❌ Pomp logica: alleen qtot
❌ Pomp flow: 30 seconden (!)
❌ Geen totalen tracking
❌ WiFi timeout: 10s per poging

NU (V53.1):
✅ ETop (hoogste temp) getoond
✅ ECO data ALTIJD (NA als offline)
✅ "OFFLINE" alleen in badge
✅ Pomp buttons (ON/OFF)
✅ Direct uitschakelbaar
✅ Pomp logica: temp_top OR qtot (OR!)
✅ Pomp flow: 30 MINUTEN met wissel
✅ Cumulatieve totalen (persistent)
✅ WiFi timeout: 20s per poging


═══════════════════════════════════════════════════════════════════════
BEKENDE BEPERKINGEN
═══════════════════════════════════════════════════════════════════════

⚠️  mDNS niet 100% betrouwbaar op ESP32-C6
    → Gebruik IP adressen als mDNS faalt

⚠️  KW boiler (zoon) heeft nog geen sensoren
    → KW waarden blijven 0.0 tot sensoren geïnstalleerd

⚠️  ECO controller bestaat nog niet
    → ECO sectie toont "OFFLINE" / "NA" tot controller gemaakt

⚠️  Pomp kWh berekening = dummy (0.5 kWh)
    → TODO: Echte berekening op basis van Qtot verschil


═══════════════════════════════════════════════════════════════════════
VOLGENDE STAPPEN
═══════════════════════════════════════════════════════════════════════

1. ✅ V53.1 volledig testen met bovenstaande checklist
2. ⏳ ECO controller bouwen (sketch #3)
3. ⏳ ECO + HVAC integratie testen
4. ⏳ KW boiler sensoren installeren (zoon)
5. ⏳ Echte kWh berekening implementeren
6. ⏳ Long-term monitoring (dagen/weken)


═══════════════════════════════════════════════════════════════════════
TROUBLESHOOTING
═══════════════════════════════════════════════════════════════════════

PROBLEEM: Pomp buttons doen niks
→ Check Serial Monitor voor errors
→ Check browser console (F12) voor JavaScript errors
→ Check MCP23017 verbinding (relay pins 8 & 9)

PROBLEEM: Totalen resetten na reboot
→ Check NVS keys in preferences
→ Serial: "Saved [pump] event" zichtbaar?
→ Check NVS_TOTAL_SCH_KWH / NVS_TOTAL_WON_KWH keys

PROBLEEM: WiFi verbindt niet (iPhone)
→ Verhoog timeout nog meer (30s ipv 20s)
→ Check iPhone hotspot instellingen
→ Probeer andere WiFi netwerk

PROBLEEM: ECO data niet zichtbaar
→ Check: is ECO controller IP/mDNS correct?
→ Serial: "POLLING ECO BOILER" zichtbaar?
→ Check: zie je "NA" labels als offline? (moet WEL)


═══════════════════════════════════════════════════════════════════════
SUCCESS CRITERIA
═══════════════════════════════════════════════════════════════════════

V53.1 is geslaagd als:

✓ JSON formaat 100% correct (zoals Particle)
✓ ECO data ALTIJD zichtbaar (offline = NA)
✓ Pomp buttons werken (ON direct, OFF direct)
✓ Totalen worden bijgehouden en getoond
✓ Pomp logica gebruikt OR (temp OF qtot)
✓ Pompen wisselen netjes af (30 min per keer)
✓ WiFi verbindt binnen 2 pogingen (iPhone)
✓ UI is clean (geen dubbele OFFLINE, blauwe sectie)
