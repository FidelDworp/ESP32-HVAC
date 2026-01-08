# ESP32-HVAC = Centrale HVAC Controller (Vervanging van Particle sketch: HVAC_Photon.cpp)

**FiDel, 8 januari 2026**

Hallo allemaal,

Dit is de open-source sketch voor mijn centrale HVAC-controller in de kelder, die de oude Particle Photon vervangt.  
Het systeem is volledig herwerkt naar een gedistribueerd model: de intelligentie zit nu in de individuele kamercontrollers (universele ESP32-roomsketch, gebaseerd op mijn Testroom-project), en deze centrale ESP32 fungeert alleen als "domme" executor en monitor.

### Inleiding

Na jaren trouwe dienst met een Particle Photon als centrale HVAC-unit, migreer ik naar ESP32.  
De redenen:
- Lokale verwerking (geen cloud-afhankelijkheid meer voor basisfunctionaliteit)
- Betere integratie met mijn bestaande ESP32-roomcontrollers
- Matter-ondersteuning in de toekomst voor Apple Home
- Open-source en volledig zelf te onderhouden

Deze sketch draait op een ESP32-C6 en bestuurt:
- 1-16 verwarmingskleppen via MCP23017 I2C expander (Instelbaar)
- 1 Ventilatie fan (0-10V signaal via PWM + level shifter)
- 2 ECO-overpomp pompen (naar kelderboiler)
- Monitort 12 DS18B20 sensoren (6 in kelderboiler SCH, 6 in ECO-boiler)

De controller pollt periodiek de kamercontrollers (/json endpoint) voor:
- heating_on (0/1) → stuurt bijbehorende klep
- vent_percent (0-100) → max waarde bepaalt fan speed

### Hoe het werkt

1. **Kamercontrollers** (max 10, configureerbaar):
   - Elke kamer bepaalt zelf verwarming en ventilatiebehoefte.
   - Hosten JSON met `y` (heating) en `z` (vent %).

2. **Verwarmingscircuits**:
   - Configureerbaar 1-16 kleppen (zoals pixels in Testroom).
   - Elke klep heeft hernoembare naam (default "Circuit 1" etc.).
   - Relais LOW = AAN.
  
   Vermogen van de circuits in Zarlardinge Schuur:
   BandB = 1254 W
   WASPL = 1096 W
   INKOM = 1025 W
   KEUK = 1018 W
   EETPL = 916 W
   ZITPL = 460 W
   BADK = 832 W

4. **Duty-cycle**:
   - Per circuit Ton/Toff bijgehouden → % berekend en getoond.

5. **Boiler monitoring**:
   - 12 hardcoded DS18B20 addresses.
   - Qtot berekening voor kelder boiler (placeholder-formule, later kalibreren).

6. **ECO-overpompen**:
   - Automatisch als ECO Qtot > threshold (default 12 kWh) en er is verwarmingsvraag.
   - Hysteresis en max 5 min pomptijd.
   - Prioriteit SCH.

7. **Ventilatie**:
   - Max vent_percent van alle kamers → PWM op GPIO20.

8. **Webinterface**:
   - Hoofdpagina: status boilers, ventilatie, circuits (state + duty-cycle)
   - /settings: alle configuratie (namen, IP's rooms, thresholds, etc.)
     → persistent in NVS
   - /logdata: JSON voor Google Sheets push of pull (elke 10 min)

9. **Toekomst**:
   - Matter integratie (switches voor kleppen/pompen, fan voor ventilatie)

### Hardware

- ESP32-C6 devboard
- MCP23017 (adres 0x20) voor relays + Thermostats
  => 3 circuits met hardwired TSTAT:
     Pin 10 = Zitplaats, Pin 11 = Eetplaats, Pin 12 = Keuken.
- OneWire bus op GPIO3 (12 DS18B20)
- PWM ventilatie op GPIO20 → externe 0-10V converter
- I2C op GPIO13 (SDA) / GPIO11 (SCL)

### Installatie

1. Flash de sketch.
2. Eerste boot → verbind met AP "HVAC-Setup" → configureer WiFi (RSSI, PW).
3. Ga naar http://hvac.local/settings → vul room IP's/mDNS, circuitnamen, etc.
4. Save → reboot → klaar!
 
FiDel
Zarlardinge, België

-------------------------------------------------------

08jan26 10:00 Version 17: Wat nu werkt:

1. Circuit Struct Uitgebreid

Room controller data: setpoint, room_temp, heat_request, home_status
Override systeem: override_active, override_state, override_start

2. Intelligente Beslissings Logica (in pollRooms())
Prioriteit volgorde:
PRIO 0: Manual Override (1 uur) → FORCE ON/OFF
PRIO 1: Hardwired TSTAT → Input
PRIO 2: HTTP Poll → Parse aa, h, y, af, z
PRIO 3: Beslissing:
  - OFFLINE → TSTAT only
  - ONLINE + HOME (af=1) → TSTAT OR HTTP
  - ONLINE + AWAY (af=0) → HTTP only

3. Complete Dashboard
Nieuwe tabel met 14 kolommen:

INPUT: #, Naam, IP, mDNS, Set, Temp, Heat, Home
OUTPUT: TSTAT, Pomp, P, Duty, Vent
CONTROL: Override (ON/OFF knoppen + countdown timer)

Features:

✅ Horizontaal scrollbaar op mobile
✅ Override badge met live countdown (45:23)
✅ Rode rand bij actieve override
✅ ⚠️ waarschuwing in Pomp kolom
✅ Kleurcodering: Groen (Thuis), Oranje (Away), Rood (NA/offline)

4. Override Endpoints

/circuit_override_on?circuit=N → Force ON
/circuit_override_off?circuit=N → Force OFF
/circuit_override_cancel?circuit=N → Annuleren

5. JSON API Uitgebreid
/json endpoint bevat nu ook:

setpoint, room_temp, heat_request, home_status
override_active, override_state, override_remaining

------------------------

To do DRINGEND:
- 

To do Later:
- HTTP polling stability kan nog verbeterd worden
- mDNS werkt niet 100% betrouwbaar op ESP32-C6 (maar IP adressen werken prima)
- Overweeg poll_interval te verhogen als er connection issues zijn
*/

