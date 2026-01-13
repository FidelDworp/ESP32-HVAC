/* ESP32C6_HVAC.ino = Centrale HVAC controller voor kelder (ESP32-C6) op basis van particle sketch voor Flobecq
Transition from Photon based to ESP32 based Home automation system. Developed together with ChatGPT & Grok in januari '26.
Thuis bereikbaar op http://hvactest.local of http://192.168.1.36 => Andere controller: Naam (sectie DNS/MDNS) + static IP aanpassen!

10jan26 08:30 Version V52: UI & JSON Improvements: "TRY1"
10jan26 10:50 Version V52: UI & JSON Improvements: "TRY2"
10jan26 12:00 Version V53: JSON formaat EXACT zoals Particle Photon
11jan26 09:00 Version V53.1: ECO logica + UI fixes
11jan26 11:30 Version V53.2: Circuit-style pump buttons + captive portal
11jan26 13:00 Version V53.3: PATCH PLAN: Timer overflow fix + OFF override
11jan26 13:30 Version V53.4: Perfect resultaat!
12jan26 19:00 Version V53.4: Kleine update: JSON endpoint = /json
12jan26 22:00 Version V53.5: ECO-inspired improvements + core fixes
13jan26 21:20 Voeg Mac adres toe om Statisch IP adres op router in te stellen.


To do Later:
- mDNS werkt niet 100% betrouwbaar op ESP32-C6 (maar IP adressen werken prima)
*/


// ============== DEEL 1/5: HEADERS, STRUCTS & HELPER FUNCTIES ==============

#include <WiFi.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Update.h>
#include <Preferences.h>
#include <OneWireNg_CurrentPlatform.h>
#include <Adafruit_MCP23X17.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>

Preferences preferences;

#define ONE_WIRE_PIN   3
#define I2C_SDA       13
#define I2C_SCL       11
#define VENT_FAN_PIN  20

OneWireNg_CurrentPlatform ow(ONE_WIRE_PIN, false);
Adafruit_MCP23X17 mcp;
AsyncWebServer server(80);
DNSServer dnsServer;
const byte DNS_PORT = 53;

// NVS keys
const char* NVS_ROOM_ID = "room_id";
const char* NVS_WIFI_SSID = "wifi_ssid";
const char* NVS_WIFI_PASS = "wifi_password";
const char* NVS_STATIC_IP = "static_ip";
const char* NVS_CIRCUITS_NUM = "circuits_num";
const char* NVS_SENSOR_NICK_BASE = "sensor_nick_";
const char* NVS_ECO_THRESHOLD = "eco_thresh";
const char* NVS_ECO_HYSTERESIS = "eco_hyst";
const char* NVS_POLL_INTERVAL = "poll_interval";
const char* NVS_ECO_IP = "eco_ip";
const char* NVS_ECO_MDNS = "eco_mdns";
const char* NVS_ECO_MIN_TEMP = "eco_min_temp";
const char* NVS_ECO_MAX_TEMP = "eco_max_temp";
const char* NVS_BOILER_REF_TEMP = "boil_ref_t";
const char* NVS_BOILER_VOLUME = "boil_vol";
const char* NVS_LAST_SCH_PUMP = "last_sch_pump";
const char* NVS_LAST_WON_PUMP = "last_won_pump";
const char* NVS_LAST_SCH_KWH = "last_sch_kwh";
const char* NVS_LAST_WON_KWH = "last_won_kwh";
const char* NVS_TOTAL_SCH_KWH = "tot_sch_kwh";
const char* NVS_TOTAL_WON_KWH = "tot_won_kwh";

// Structs
struct Circuit {
  String name;
  String ip;
  String mdns;
  float power_kw;
  bool has_tstat;
  int tstat_pin;
  bool online;
  unsigned long last_seen;
  bool heating_on;
  int vent_request;
  unsigned long on_time;
  unsigned long off_time;
  unsigned long last_change;
  float duty_cycle;
  int setpoint;
  float room_temp;
  bool heat_request;
  int home_status;
  bool override_active;
  bool override_state;
  unsigned long override_start;
};

struct EcoBoilerData {
  bool online;
  float temp_avg;
  float qtot;
  float temp_top;
  float temp_bottom;
  unsigned long last_seen;
};

struct PumpEvent {
  unsigned long timestamp;
  float kwh_pumped;
};

// Global variables
String room_id = "HVAC";
String wifi_ssid = "";
String wifi_pass = "";
String static_ip_str = "";
String mac_address = "";  // Voor display in settings
IPAddress static_ip;
int circuits_num = 7;
Circuit circuits[16];
String sensor_nicknames[6];
float eco_threshold = 15.0;  // V53.5: Updated default
float eco_hysteresis = 5.0;  // V53.5: Updated default
int poll_interval = 10;
String eco_controller_ip = "";
String eco_controller_mdns = "eco";
float eco_min_temp = 80.0;  // V53.5: Stop temp (Tmin)
float eco_max_temp = 90.0;  // V53.5: Start temp (Tmax)
float boiler_ref_temp = 20.0;
float boiler_layer_volume = 50.0;

#define RELAY_PUMP_SCH 8
#define RELAY_PUMP_WON 9

int vent_percent = 0;
float sch_temps[6] = {-127,-127,-127,-127,-127,-127};
float eco_temps[6] = {-127,-127,-127,-127,-127,-127};
bool sensor_ok[6] = {false};
float sch_qtot = 0.0;
float eco_qtot = 0.0;

EcoBoilerData eco_boiler = {false, 0.0, 0.0, 0.0, 0.0, 0};

// Last pump events
PumpEvent last_sch_pump = {0, 0.0};
PumpEvent last_won_pump = {0, 0.0};

// Cumulatieve totalen (persistent in NVS)
float total_sch_kwh = 0.0;
float total_won_kwh = 0.0;

// V53.5: NIEUWE pump state machine met afzonderlijke wait states
enum EcoPumpState { ECO_IDLE, ECO_PUMP_SCH, ECO_WAIT_SCH, ECO_PUMP_WON, ECO_WAIT_WON };
EcoPumpState eco_pump_state = ECO_IDLE;
unsigned long eco_pump_timer = 0;
bool last_pump_was_sch = false;

// V53.5: Nieuwe tijdconstanten (1 min cycles!)
const unsigned long ECO_PUMP_DURATION = 1 * 60 * 1000UL;     // 1 min pompen
const unsigned long ECO_WAIT_SCH_DURATION = 1 * 60 * 1000UL; // 1 min wacht na SCH
const unsigned long ECO_WAIT_WON_DURATION = 2 * 60 * 1000UL; // 2 min wacht na WON

bool sch_pump_manual = false;
bool won_pump_manual = false;
unsigned long sch_pump_manual_start = 0;
unsigned long won_pump_manual_start = 0;
const unsigned long MANUAL_PUMP_DURATION = 60000UL;
bool sch_pump_manual_on = true;  // true = ON override, false = OFF override
bool won_pump_manual_on = true;

// V53.5: Trend tracking voor ECO data
float prev_eco_temp_top = 0.0;
float prev_eco_qtot = 0.0;

unsigned long last_poll = 0;
unsigned long last_temp_read = 0;
unsigned long uptime_sec = 0;
bool ap_mode_active = false;
bool mcp_available = false;

OneWireNg::Id sensor_addresses[6] = {
  {0x28,0xDB,0xB5,0x03,0x00,0x00,0x80,0xBB},
  {0x28,0x7C,0xF0,0x03,0x00,0x00,0x80,0x59},
  {0x28,0x72,0xDB,0x03,0x00,0x00,0x80,0xC2},
  {0x28,0xAA,0xFB,0x03,0x00,0x00,0x80,0x5F},
  {0x28,0x49,0xDD,0x03,0x00,0x00,0x80,0x4B},
  {0x28,0xC3,0xD6,0x03,0x00,0x00,0x80,0x1E}
};

// Helper functies
String getFormattedDateTime() {
  time_t now; 
  time(&now);
  if (now < 1700000000) return "tijd niet gesync";
  struct tm tm; 
  localtime_r(&now, &tm);
  char buf[32]; 
  strftime(buf, sizeof(buf), "%d-%m-%Y %H:%M:%S", &tm);
  return String(buf);
}

float calculateQtot(float temps[6]) {
  const float Cp = 1.16;
  float total_energy = 0.0;
  
  float T_layer0 = (boiler_ref_temp + temps[0]) / 2.0;
  if (T_layer0 > boiler_ref_temp) {
    total_energy += (T_layer0 - boiler_ref_temp) * boiler_layer_volume * Cp;
  }
  
  for (int i = 1; i < 6; i++) {
    float T_layer = (temps[i-1] + temps[i]) / 2.0;
    if (T_layer > boiler_ref_temp) {
      total_energy += (T_layer - boiler_ref_temp) * boiler_layer_volume * Cp;
    }
  }
  
  float T_layer6 = temps[5];
  if (T_layer6 > boiler_ref_temp) {
    total_energy += (T_layer6 - boiler_ref_temp) * boiler_layer_volume * Cp;
  }
  
  return total_energy / 1000.0;
}

// V53.5: NIEUW - Trend helper function (van ECO v1.3)
String getTrend(float current, float previous, float threshold = 0.1) {
  if (abs(current - previous) < threshold) return "→";
  return (current > previous) ? "↑" : "↓";
}

void readBoilerTemps() {
  if (millis() - last_temp_read < 2000) return;
  last_temp_read = millis();
  
  for (int i = 0; i < 6; i++) {
    ow.reset(); 
    ow.writeByte(0x55);
    for (int j = 0; j < 8; j++) ow.writeByte(sensor_addresses[i][j]);
    ow.writeByte(0x44); 
    delay(750);
    
    ow.reset(); 
    ow.writeByte(0x55);
    for (int j = 0; j < 8; j++) ow.writeByte(sensor_addresses[i][j]);
    ow.writeByte(0xBE);
    
    uint8_t data[9];
    for (int j = 0; j < 9; j++) data[j] = ow.readByte();
    
    uint8_t crc = 0;
    for (int j = 0; j < 8; j++) {
      uint8_t inbyte = data[j];
      for (int k = 0; k < 8; k++) {
        uint8_t mix = (crc ^ inbyte) & 0x01;
        crc >>= 1; 
        if (mix) crc ^= 0x8C; 
        inbyte >>= 1;
      }
    }
    
    if (crc == data[8]) {
      int16_t raw = (data[1] << 8) | data[0];
      sch_temps[i] = raw / 16.0; 
      sensor_ok[i] = true;
    } else {
      sch_temps[i] = -127.0; 
      sensor_ok[i] = false;
    }
  }
  
  sch_qtot = calculateQtot(sch_temps);
}

void checkPumpFeedback(float total_power) {
  if (!mcp_available) return;
  bool pump_should_be_on = (total_power > 0.01);
  bool pump_is_on = (mcp.digitalRead(7) == LOW);
  if (pump_should_be_on && !pump_is_on) {
    Serial.println("ALERT: Pump should be ON but is OFF!");
  } else if (!pump_should_be_on && pump_is_on) {
    Serial.println("ALERT: Pump should be OFF but is ON!");
  }
}

void pollEcoBoiler() {
  static unsigned long last_eco_poll = 0;
  if (millis() - last_eco_poll < (unsigned long)poll_interval * 1000) return;
  last_eco_poll = millis();
  
  if (eco_controller_ip.length() == 0 && eco_controller_mdns.length() == 0) {
    eco_boiler.online = false;
    return;
  }
  
  Serial.println("\n=== POLLING ECO BOILER ===");
  
  WiFiClient client;
  HTTPClient http;
  String url;
  
  if (eco_controller_ip.length() > 0) {
    url = "http://" + eco_controller_ip + "/json";  // V53.5: Changed to /json
  } else {
    Serial.printf("Resolving %s.local ... ", eco_controller_mdns.c_str());
    IPAddress resolvedIP;
    if (WiFi.hostByName((eco_controller_mdns + ".local").c_str(), resolvedIP)) {
      if (resolvedIP.toString() == "0.0.0.0" || resolvedIP[0] == 0) {
        Serial.printf("FAILED\n");
        eco_boiler.online = false;
        return;
      }
      Serial.printf("OK -> %s\n", resolvedIP.toString().c_str());
      url = "http://" + resolvedIP.toString() + "/json";  // V53.5: Changed to /json
    } else {
      Serial.printf("FAILED\n");
      eco_boiler.online = false;
      return;
    }
  }
  
  Serial.printf("ECO: Polling %s\n    ", url.c_str());
  
  http.begin(client, url);
  http.setTimeout(2000);
  http.setConnectTimeout(1000);
  http.setReuse(false);
  
  int httpCode = http.GET();
  Serial.printf("Result: %d", httpCode);
  
  if (httpCode == 200) {
    String payload = http.getString();
    Serial.printf(" (%d bytes) ✓\n", payload.length());
    
    eco_boiler.online = true;
    eco_boiler.last_seen = millis();
    
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, payload);
    
    if (!error) {
      eco_boiler.temp_avg = doc["EAv"] | 0.0;
      eco_boiler.qtot = doc["EQtot"] | 0.0;
      
      // V53.5: FIX - Correct JSON keys!
      eco_boiler.temp_top = doc["ETopH"] | 0.0;     // Was: doc["ET"]
      eco_boiler.temp_bottom = doc["EBotL"] | 0.0;  // Was: doc["EB"]
      
      eco_qtot = eco_boiler.qtot;
      
      Serial.printf("    ETopH=%.1f°C EQtot=%.2f kWh EBotL=%.1f°C\n", 
        eco_boiler.temp_top, eco_boiler.qtot, eco_boiler.temp_bottom);
      
      // V53.5: Update trend data (skip eerste poll)
      static int eco_poll_count = 0;
      eco_poll_count++;
      if (eco_poll_count > 1) {
        // Trends worden nu bijgehouden voor UI
      }
      prev_eco_temp_top = eco_boiler.temp_top;
      prev_eco_qtot = eco_boiler.qtot;
      
    } else {
      Serial.printf("    JSON parse error: %s\n", error.c_str());
    }
  } else {
    Serial.printf(" FAILED\n");
    eco_boiler.online = false;
  }
  
  http.end();
  client.stop();
}

void savePumpEvent(const char* pump_type, float kwh) {
  unsigned long timestamp = millis() / 1000;
  
  if (String(pump_type) == "SCH") {
    last_sch_pump.timestamp = timestamp;
    last_sch_pump.kwh_pumped = kwh;
    total_sch_kwh += kwh;
    preferences.putULong(NVS_LAST_SCH_PUMP, timestamp);
    preferences.putFloat(NVS_LAST_SCH_KWH, kwh);
    preferences.putFloat(NVS_TOTAL_SCH_KWH, total_sch_kwh);
    Serial.printf("Saved SCH pump event: %.2f kWh (totaal: %.2f kWh)\n", kwh, total_sch_kwh);
  } else if (String(pump_type) == "WON") {
    last_won_pump.timestamp = timestamp;
    last_won_pump.kwh_pumped = kwh;
    total_won_kwh += kwh;
    preferences.putULong(NVS_LAST_WON_PUMP, timestamp);
    preferences.putFloat(NVS_LAST_WON_KWH, kwh);
    preferences.putFloat(NVS_TOTAL_WON_KWH, total_won_kwh);
    Serial.printf("Saved WON pump event: %.2f kWh (totaal: %.2f kWh)\n", kwh, total_won_kwh);
  }
}

// V53.5: NIEUW - Smart pump status messages (inspired by ECO v1.3)
String getPumpStatusMessage() {
  // Manual override heeft hoogste prioriteit
  if (sch_pump_manual || won_pump_manual) {
    unsigned long elapsed_sch = millis() - sch_pump_manual_start;
    unsigned long elapsed_won = millis() - won_pump_manual_start;
    
    if (sch_pump_manual && elapsed_sch < MANUAL_PUMP_DURATION) {
      unsigned long remaining = (MANUAL_PUMP_DURATION - elapsed_sch) / 1000;
      return "🎮 Handmatig: SCH pomp " + String(sch_pump_manual_on ? "AAN" : "UIT") + 
             " (" + String(remaining) + "s resterend)";
    }
    if (won_pump_manual && elapsed_won < MANUAL_PUMP_DURATION) {
      unsigned long remaining = (MANUAL_PUMP_DURATION - elapsed_won) / 1000;
      return "🎮 Handmatig: WON pomp " + String(won_pump_manual_on ? "AAN" : "UIT") + 
             " (" + String(remaining) + "s resterend)";
    }
  }
  
  // ECO offline check
  if (!eco_boiler.online) {
    return "○ ECO boiler offline - geen automatische distributie";
  }
  
  // State-based context-aware messages
  switch (eco_pump_state) {
    case ECO_IDLE: {
      bool temp_trigger = eco_boiler.temp_top > eco_max_temp;
      bool energy_trigger = eco_boiler.qtot > eco_threshold;
      
      if (temp_trigger || energy_trigger) {
        String reason = "";
        if (temp_trigger) {
          reason = "Temp: " + String(eco_boiler.temp_top, 1) + "°C > " + String(eco_max_temp, 0) + "°C";
        }
        if (energy_trigger) {
          if (reason.length() > 0) reason += " EN ";
          reason += "Energie: " + String(eco_boiler.qtot, 1) + " > " + String(eco_threshold, 1) + " kWh";
        }
        return "⏸️ Trigger actief (" + reason + ") - wacht op cyclus start";
      } else {
        return "✓ Standby - Temp: " + String(eco_boiler.temp_top, 1) + "°C, Energie: " + 
               String(eco_boiler.qtot, 1) + " kWh (beide onder limiet)";
      }
    }
    
    case ECO_PUMP_SCH: {
      unsigned long elapsed = millis() - eco_pump_timer;
      unsigned long remaining = (ECO_PUMP_DURATION - elapsed) / 1000;
      return "🔵 SCH pompt naar Schuur (" + String(remaining) + "s / 60s) - Fair share transfer";
    }
    
    case ECO_WAIT_SCH: {
      unsigned long elapsed = millis() - eco_pump_timer;
      unsigned long remaining = (ECO_WAIT_SCH_DURATION - elapsed) / 1000;
      return "⏳ Wacht na SCH pomp (" + String(remaining) + "s / 60s) - WON is volgende";
    }
    
    case ECO_PUMP_WON: {
      unsigned long elapsed = millis() - eco_pump_timer;
      unsigned long remaining = (ECO_PUMP_DURATION - elapsed) / 1000;
      return "🟢 WON pompt naar Woning (" + String(remaining) + "s / 60s) - Fair share transfer";
    }
    
    case ECO_WAIT_WON: {
      unsigned long elapsed = millis() - eco_pump_timer;
      unsigned long remaining = (ECO_WAIT_WON_DURATION - elapsed) / 1000;
      return "⏳ Wacht na WON pomp (" + String(remaining) + "s / 120s) - SCH is volgende";
    }
    
    default:
      return "? Onbekende pump status";
  }
}







// ============== DEEL 2/5: ECO PUMPS & POLLING FUNCTIES ==============

// V53.5: handleEcoPumps() met nieuwe state machine (WAIT_SCH en WAIT_WON)
void handleEcoPumps() {
  if (!mcp_available) return;
  
  // Check timeouts en RESET manual flags
  bool sch_manual_active = sch_pump_manual && (millis() - sch_pump_manual_start < MANUAL_PUMP_DURATION);
  bool won_manual_active = won_pump_manual && (millis() - won_pump_manual_start < MANUAL_PUMP_DURATION);
  
  if (sch_pump_manual && !sch_manual_active) {
    Serial.println("\n=== MANUAL PUMP TIMEOUT ===");
    Serial.println("SCH pump manual mode expired (60s)");
    sch_pump_manual = false;
  }
  if (won_pump_manual && !won_manual_active) {
    Serial.println("\n=== MANUAL PUMP TIMEOUT ===");
    Serial.println("WON pump manual mode expired (60s)");
    won_pump_manual = false;
  }
  
  // Manual mode heeft prioriteit (ON of OFF!)
  if (sch_manual_active || won_manual_active) {
    mcp.digitalWrite(RELAY_PUMP_SCH, sch_manual_active ? (sch_pump_manual_on ? LOW : HIGH) : HIGH);
    mcp.digitalWrite(RELAY_PUMP_WON, won_manual_active ? (won_pump_manual_on ? LOW : HIGH) : HIGH);
    return;
  }
  
  // START conditie (OR logica)
  bool should_start = eco_boiler.online 
                   && ((eco_boiler.temp_top > eco_max_temp) || (eco_boiler.qtot > eco_threshold));
  
  // STOP conditie (OR logica)
  bool should_stop = !eco_boiler.online
                  || ((eco_boiler.temp_top < eco_min_temp) || (eco_boiler.qtot < (eco_threshold - eco_hysteresis)));
  
  // V53.5: State machine met 5 states
  switch (eco_pump_state) {
    case ECO_IDLE:
      // Beide pompen uit
      mcp.digitalWrite(RELAY_PUMP_SCH, HIGH);
      mcp.digitalWrite(RELAY_PUMP_WON, HIGH);
      
      // Check of we moeten starten
      if (should_start) {
        // Wissel tussen WON en SCH voor fair share
        if (last_pump_was_sch) {
          eco_pump_state = ECO_PUMP_WON;
          Serial.println("\n=== AUTO PUMP START ===");
          Serial.println("Pump: WON");
          Serial.println("Duration: 1 minute");
          Serial.printf("Reason: temp_top=%.1f°C>%.0f OR qtot=%.1f>%.1f kWh\n",
            eco_boiler.temp_top, eco_max_temp, eco_boiler.qtot, eco_threshold);
        } else {
          eco_pump_state = ECO_PUMP_SCH;
          Serial.println("\n=== AUTO PUMP START ===");
          Serial.println("Pump: SCH");
          Serial.println("Duration: 1 minute");
          Serial.printf("Reason: temp_top=%.1f°C>%.0f OR qtot=%.1f>%.1f kWh\n",
            eco_boiler.temp_top, eco_max_temp, eco_boiler.qtot, eco_threshold);
        }
        eco_pump_timer = millis();
      }
      break;
      
    case ECO_PUMP_SCH:
      // SCH pompt, WON uit
      mcp.digitalWrite(RELAY_PUMP_SCH, LOW);
      mcp.digitalWrite(RELAY_PUMP_WON, HIGH);
      
      // Check stop conditie
      if (should_stop) {
        Serial.println("\n=== AUTO PUMP STOP ===");
        Serial.println("Pump: SCH");
        Serial.printf("Reason: temp_top=%.1f<%.0f OR qtot=%.1f<%.1f kWh\n",
          eco_boiler.temp_top, eco_min_temp, eco_boiler.qtot, eco_threshold - eco_hysteresis);
        eco_pump_state = ECO_IDLE;
        mcp.digitalWrite(RELAY_PUMP_SCH, HIGH);
        
        // Save event
        float kwh_transferred = 0.5;  // TODO: Echte berekening
        savePumpEvent("SCH", kwh_transferred);
        last_pump_was_sch = true;
        break;
      }
      
      // Check timeout (1 min)
      if (millis() - eco_pump_timer >= ECO_PUMP_DURATION) {
        Serial.println("\n=== AUTO PUMP FINISHED ===");
        Serial.println("Pump: SCH");
        Serial.println("Duration: 1 minute completed");
        mcp.digitalWrite(RELAY_PUMP_SCH, HIGH);
        
        // Save event
        float kwh_transferred = 0.5;  // TODO: Echte berekening
        savePumpEvent("SCH", kwh_transferred);
        last_pump_was_sch = true;
        
        // V53.5: Ga naar WAIT_SCH
        eco_pump_state = ECO_WAIT_SCH;
        eco_pump_timer = millis();
        Serial.println("Entering wait phase after SCH: 1 minute");
      }
      break;
      
    // V53.5: NIEUWE state - Wait na SCH (1 min)
    case ECO_WAIT_SCH:
      // Beide pompen uit
      mcp.digitalWrite(RELAY_PUMP_SCH, HIGH);
      mcp.digitalWrite(RELAY_PUMP_WON, HIGH);
      
      // Check stop conditie
      if (should_stop) {
        Serial.println("\n=== CYCLE STOPPED ===");
        Serial.println("Reason: Stop conditie tijdens wacht fase");
        eco_pump_state = ECO_IDLE;
        break;
      }
      
      // Check timeout (1 min)
      if (millis() - eco_pump_timer >= ECO_WAIT_SCH_DURATION) {
        Serial.println("\n=== WAIT AFTER SCH FINISHED ===");
        
        if (should_start) {
          eco_pump_state = ECO_PUMP_WON;
          eco_pump_timer = millis();
          Serial.println("Starting WON pump (1 min)");
        } else {
          eco_pump_state = ECO_IDLE;
          Serial.println("No more pumping needed -> IDLE");
        }
      }
      break;
      
    case ECO_PUMP_WON:
      // WON pompt, SCH uit
      mcp.digitalWrite(RELAY_PUMP_SCH, HIGH);
      mcp.digitalWrite(RELAY_PUMP_WON, LOW);
      
      // Check stop conditie
      if (should_stop) {
        Serial.println("\n=== AUTO PUMP STOP ===");
        Serial.println("Pump: WON");
        Serial.printf("Reason: temp_top=%.1f<%.0f OR qtot=%.1f<%.1f kWh\n",
          eco_boiler.temp_top, eco_min_temp, eco_boiler.qtot, eco_threshold - eco_hysteresis);
        eco_pump_state = ECO_IDLE;
        mcp.digitalWrite(RELAY_PUMP_WON, HIGH);
        
        // Save event
        float kwh_transferred = 0.5;  // TODO: Echte berekening
        savePumpEvent("WON", kwh_transferred);
        last_pump_was_sch = false;
        break;
      }
      
      // Check timeout (1 min)
      if (millis() - eco_pump_timer >= ECO_PUMP_DURATION) {
        Serial.println("\n=== AUTO PUMP FINISHED ===");
        Serial.println("Pump: WON");
        Serial.println("Duration: 1 minute completed");
        mcp.digitalWrite(RELAY_PUMP_WON, HIGH);
        
        // Save event
        float kwh_transferred = 0.5;  // TODO: Echte berekening
        savePumpEvent("WON", kwh_transferred);
        last_pump_was_sch = false;
        
        // V53.5: Ga naar WAIT_WON
        eco_pump_state = ECO_WAIT_WON;
        eco_pump_timer = millis();
        Serial.println("Entering wait phase after WON: 2 minutes");
      }
      break;
      
    // V53.5: NIEUWE state - Wait na WON (2 min)
    case ECO_WAIT_WON:
      // Beide pompen uit
      mcp.digitalWrite(RELAY_PUMP_SCH, HIGH);
      mcp.digitalWrite(RELAY_PUMP_WON, HIGH);
      
      // Check stop conditie
      if (should_stop) {
        Serial.println("\n=== CYCLE STOPPED ===");
        Serial.println("Reason: Stop conditie tijdens wacht fase");
        eco_pump_state = ECO_IDLE;
        break;
      }
      
      // Check timeout (2 min)
      if (millis() - eco_pump_timer >= ECO_WAIT_WON_DURATION) {
        Serial.println("\n=== WAIT AFTER WON FINISHED ===");
        
        if (should_start) {
          eco_pump_state = ECO_PUMP_SCH;
          eco_pump_timer = millis();
          Serial.println("Starting SCH pump (1 min)");
        } else {
          eco_pump_state = ECO_IDLE;
          Serial.println("No more pumping needed -> IDLE");
        }
      }
      break;
  }
}

void pollRooms() {
  if (millis() - last_poll < (unsigned long)poll_interval * 1000) return;
  last_poll = millis();

  Serial.println("\n=== POLLING ROOMS ===");
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WARNING: WiFi disconnected!");
    WiFi.reconnect();
    delay(2000);
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Reconnect failed!");
      return;
    }
  } else {
    Serial.printf("WiFi OK - IP: %s, RSSI: %d dBm\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
  }

  vent_percent = 0;
  float total_power = 0.0;
  const unsigned long OVERRIDE_TIMEOUT = 600000UL;

  for (int i = 0; i < circuits_num; i++) {
    bool heating_demand = false;
    bool tstat_demand = false;
    bool http_demand = false;
    int home_status = 0;
    
    if (circuits[i].override_active) {
      unsigned long elapsed = millis() - circuits[i].override_start;
      if (elapsed < OVERRIDE_TIMEOUT) {
        heating_demand = circuits[i].override_state;
        unsigned long remaining = (OVERRIDE_TIMEOUT - elapsed) / 1000;
        Serial.printf("c%d: ⚠️ OVERRIDE %s (%lu:%02lu remaining)\n", 
          i, circuits[i].override_state ? "FORCE ON" : "FORCE OFF",
          remaining / 60, remaining % 60);
        goto apply_relay;
      } else {
        circuits[i].override_active = false;
        Serial.printf("c%d: Override timeout - terug naar AUTO\n", i);
      }
    }
    
    if (circuits[i].has_tstat && mcp_available && circuits[i].tstat_pin < 13) {
      bool tstat_on = (mcp.digitalRead(circuits[i].tstat_pin) == LOW);
      Serial.printf("c%d: TSTAT pin %d = %s\n", i, circuits[i].tstat_pin, tstat_on ? "ON" : "OFF");
      tstat_demand = tstat_on;
    }
    
    if (circuits[i].ip.length() > 0 || circuits[i].mdns.length() > 0) {
      WiFiClient client;
      HTTPClient http;
      String url;
      
      if (circuits[i].ip.length() > 0) {
        url = "http://" + circuits[i].ip + "/json";  // V53.5: Changed to /json
        Serial.printf("c%d: Polling %s\n", i, url.c_str());
        Serial.print("    ");
      } else {
        Serial.printf("c%d: Resolving %s.local ... ", i, circuits[i].mdns.c_str());
        IPAddress resolvedIP;
        if (WiFi.hostByName((circuits[i].mdns + ".local").c_str(), resolvedIP)) {
          if (resolvedIP.toString() == "0.0.0.0" || resolvedIP[0] == 0) {
            Serial.printf("FAILED (host not found)\n");
            circuits[i].online = false;
            circuits[i].vent_request = 0;
            goto decision_logic;
          }
          Serial.printf("OK -> %s\n", resolvedIP.toString().c_str());
          url = "http://" + resolvedIP.toString() + "/json";  // V53.5: Changed to /json
          Serial.printf("c%d: Polling %s\n", i, url.c_str());
          Serial.print("    ");
        } else {
          Serial.printf("FAILED (DNS error)\n");
          circuits[i].online = false;
          circuits[i].vent_request = 0;
          goto decision_logic;
        }
      }

      http.begin(client, url);
      http.setTimeout(4000);
      http.setConnectTimeout(1500);
      http.setReuse(false);
      
      int httpCode = http.GET();
      Serial.printf("Result: %d", httpCode);

      if (httpCode == 200) {
        String payload = http.getString();
        Serial.printf(" (%d bytes) ✓\n", payload.length());
        
        circuits[i].online = true;
        circuits[i].last_seen = millis();
        
        DynamicJsonDocument doc(2048);
        DeserializationError error = deserializeJson(doc, payload);
        
        if (!error) {
          int y_val = doc["y"] | 0;
          int z_val = doc["z"] | 0;
          int aa_val = doc["aa"] | 0;
          float h_val = doc["h"] | 0.0;
          int af_val = doc["af"] | 0;
          
          circuits[i].heat_request = (y_val == 1);
          circuits[i].vent_request = z_val;
          circuits[i].setpoint = aa_val;
          circuits[i].room_temp = h_val;
          circuits[i].home_status = af_val;
          
          http_demand = circuits[i].heat_request;
          home_status = circuits[i].home_status;
          
          Serial.printf("    y=%d z=%d aa=%d h=%.1f af=%d\n", 
            y_val, z_val, aa_val, h_val, af_val);
          
          if (z_val > vent_percent) vent_percent = z_val;
        } else {
          Serial.printf("    JSON parse error: %s\n", error.c_str());
        }
      } else if (httpCode == 404) {
        Serial.printf(" - 404 Not Found\n");
        circuits[i].online = false;
        circuits[i].vent_request = 0;
      } else if (httpCode < 0) {
        Serial.printf(" - Timeout/Unreachable\n");
        circuits[i].online = false;
        circuits[i].vent_request = 0;
      } else {
        Serial.printf(" - HTTP Error\n");
        circuits[i].online = false;
        circuits[i].vent_request = 0;
      }
      
      http.end();
      client.stop();
      delay(100);
    }
    
    decision_logic:
    if (!circuits[i].online) {
      heating_demand = tstat_demand;
      Serial.printf("c%d: OFFLINE → TSTAT only = %s\n", i, heating_demand ? "ON" : "OFF");
    } else {
      if (home_status == 1) {
        heating_demand = tstat_demand || http_demand;
        Serial.printf("c%d: HOME → TSTAT(%d) OR HTTP(%d) = %s\n", 
          i, tstat_demand, http_demand, heating_demand ? "ON" : "OFF");
      } else {
        heating_demand = http_demand;
        Serial.printf("c%d: AWAY → HTTP only = %s\n", i, heating_demand ? "ON" : "OFF");
      }
    }
    
    apply_relay:
    if (heating_demand != circuits[i].heating_on) {
      Serial.printf("c%d: Relay %s -> %s\n", i, 
        circuits[i].heating_on ? "ON" : "OFF", heating_demand ? "ON" : "OFF");
        
      if (mcp_available && i < 7) {
        mcp.digitalWrite(i, heating_demand ? LOW : HIGH);
      }
      if (heating_demand) {
        circuits[i].off_time += millis() - circuits[i].last_change;
      } else {
        circuits[i].on_time += millis() - circuits[i].last_change;
      }
      circuits[i].last_change = millis();
      circuits[i].heating_on = heating_demand;
    }
    
    unsigned long total = circuits[i].on_time + circuits[i].off_time;
    if (total > 0) {
      circuits[i].duty_cycle = 100.0 * circuits[i].on_time / total;
    }
    
    if (circuits[i].heating_on) total_power += circuits[i].power_kw;
  }
  
  Serial.printf("Total power: %.2f kW, Vent: %d%%\n", total_power, vent_percent);
  int pwm_value = map(vent_percent, 0, 100, 0, 255);
  analogWrite(VENT_FAN_PIN, pwm_value);
  Serial.printf("Vent PWM: %d/255 (%d%%)\n", pwm_value, vent_percent);
  checkPumpFeedback(total_power);
}

String getWifiScanJson() {
  DynamicJsonDocument doc(4096);
  int n = WiFi.scanNetworks();
  JsonArray networks = doc.createNestedArray("networks");
  for (int i = 0; i < n; i++) {
    JsonObject net = networks.createNestedObject();
    net["ssid"] = WiFi.SSID(i);
    net["rssi"] = WiFi.RSSI(i);
  }
  String json;
  serializeJson(doc, json);
  return json;
}

// V53.5: JSON met pump_status toegevoegd
String getLogData() {
  DynamicJsonDocument doc(2048);
  
  doc["eco_online"] = eco_boiler.online ? 1 : 0;
  
  doc["KSTopH"] = sch_temps[0];
  doc["KSTopL"] = sch_temps[1];
  doc["KSMidH"] = sch_temps[2];
  doc["KSMidL"] = sch_temps[3];
  doc["KSBotH"] = sch_temps[4];
  doc["KSBotL"] = sch_temps[5];
  
  float KSAv = 0;
  int valid_count = 0;
  for (int i = 0; i < 6; i++) {
    if (sensor_ok[i] && sch_temps[i] > -100) {
      KSAv += sch_temps[i];
      valid_count++;
    }
  }
  if (valid_count > 0) KSAv /= valid_count;
  doc["KSAv"] = KSAv;
  doc["KSQtot"] = sch_qtot;
  
  doc["KWTopH"] = 0.0;
  doc["KWTopL"] = 0.0;
  doc["KWMidH"] = 0.0;
  doc["KWMidL"] = 0.0;
  doc["KWBotH"] = 0.0;
  doc["KWBotL"] = 0.0;
  doc["KWAv"] = 0.0;
  doc["KWQtot"] = 0.0;
  
  doc["EAv"] = eco_boiler.temp_avg;
  doc["EQtot"] = eco_boiler.qtot;
  doc["ET"] = eco_max_temp;
  doc["EB"] = eco_min_temp;
  
  doc["BB"] = (int)circuits[0].duty_cycle;
  doc["WP"] = (int)circuits[1].duty_cycle;
  doc["BK"] = (int)circuits[2].duty_cycle;
  doc["ZP"] = (int)circuits[3].duty_cycle;
  doc["EP"] = (int)circuits[4].duty_cycle;
  doc["KK"] = (int)circuits[5].duty_cycle;
  doc["IK"] = (int)circuits[6].duty_cycle;
  
  doc["R1"] = circuits[0].heating_on ? 1 : 0;
  doc["R2"] = circuits[1].heating_on ? 1 : 0;
  doc["R3"] = circuits[2].heating_on ? 1 : 0;
  doc["R4"] = circuits[3].heating_on ? 1 : 0;
  doc["R5"] = circuits[4].heating_on ? 1 : 0;
  doc["R6"] = circuits[5].heating_on ? 1 : 0;
  doc["R7"] = circuits[6].heating_on ? 1 : 0;
  
  doc["R9"] = (sch_pump_manual || (mcp_available && mcp.digitalRead(RELAY_PUMP_SCH) == LOW)) ? 1 : 0;
  doc["R10"] = (won_pump_manual || (mcp_available && mcp.digitalRead(RELAY_PUMP_WON) == LOW)) ? 1 : 0;
  
  float total_power = 0.0;
  for (int i = 0; i < circuits_num; i++) {
    if (circuits[i].heating_on) total_power += circuits[i].power_kw;
  }
  doc["HeatDem"] = total_power;
  doc["Vent"] = vent_percent;
  
  // V53.5: NIEUW - pump_status in JSON
  String pumpStatus = getPumpStatusMessage();
  pumpStatus.replace("\"", "\\\"");  // Escape quotes
  doc["pump_status"] = pumpStatus;
  
  String json;
  serializeJson(doc, json);
  return json;
}












// ============== DEEL 3/5: MAIN PAGE UI ==============

String getMainPage() {
  float total_power = 0.0;
  for (int i = 0; i < circuits_num; i++) {
    if (circuits[i].heating_on) total_power += circuits[i].power_kw;
  }
  
  // V53.5: Calculate trends
  String temp_trend = getTrend(eco_boiler.temp_top, prev_eco_temp_top, 0.5);
  String qtot_trend = getTrend(eco_boiler.qtot, prev_eco_qtot, 0.1);
  
  // V53.5: Determine temp color class
  String temp_color_class = "temp-cold";
  if (eco_boiler.temp_top >= 90) temp_color_class = "temp-danger";
  else if (eco_boiler.temp_top >= 80) temp_color_class = "temp-hot";
  else if (eco_boiler.temp_top >= 60) temp_color_class = "temp-warm";
  
  // V53.5: Determine energy color class
  String energy_color_class = "energy-low";
  if (eco_boiler.qtot >= 20) energy_color_class = "energy-max";
  else if (eco_boiler.qtot >= 15) energy_color_class = "energy-high";
  else if (eco_boiler.qtot >= 10) energy_color_class = "energy-ok";
  
  // V53.5: Get pump status message
  String pumpStatusMsg = getPumpStatusMessage();
  
  String html;
  html.reserve(40000);
  html = R"rawliteral(
<!DOCTYPE html>
<html lang="nl">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>)rawliteral" + room_id + R"rawliteral( Status</title>
  <style>
    body {font-family:Arial,sans-serif;background:#fff;margin:0;padding:0;}
    .header {display:flex;background:#ffcc00;color:#000;padding:10px 15px;font-size:18px;font-weight:bold;align-items:center;}
    .header-left {flex:1;}
    .header-right {flex:1;text-align:right;font-size:15px;}
    
    /* V53.5: NIEUW - Status banner (prominent!) */
    .status-banner {
      background: linear-gradient(135deg, #336699 0%, #2a5580 100%);
      color: #fff;
      padding: 15px 20px;
      display: flex;
      align-items: center;
      border-bottom: 3px solid #ffcc00;
      font-size: 15px;
      box-shadow: 0 2px 4px rgba(0,0,0,0.1);
    }
    .status-label {
      font-weight: bold;
      margin-right: 10px;
      font-size: 16px;
    }
    .status-message {
      flex: 1;
      line-height: 1.4;
    }
    
    .container {display:flex;min-height:calc(100vh - 110px);}
    .sidebar {width:80px;padding:10px 5px;background:#fff;border-right:3px solid #c00;}
    .sidebar a {display:block;background:#369;color:#fff;padding:8px;text-decoration:none;font-weight:bold;font-size:12px;border-radius:6px;text-align:center;width:60px;margin:8px auto;}
    .sidebar a:hover {background:#036;}
    .sidebar a.active {background:#c00;}
    .main {flex:1;padding:15px;overflow-y:auto;}
    .group-title {font-size:17px;font-style:italic;font-weight:bold;color:#fff;background:#336699;padding:8px 12px;margin:20px 0 8px 0;border-radius:4px;}
    .section-divider {border-top:2px solid #ccc;margin:15px 0;}
    .blue-divider {border-top:3px solid #336699;margin:25px 0;}
    
    /* V53.5: Refresh knop naar beneden verplaatst - niet meer bovenaan! */
    .refresh-btn {background:#369;color:#fff;padding:10px 20px;border:none;border-radius:6px;cursor:pointer;font-size:14px;font-weight:bold;margin:20px 0;width:100%;max-width:300px;}
    .refresh-btn:hover {background:#036;}
    
    table {width:100%;border-collapse:collapse;margin-bottom:15px;}
    td.label {color:#369;font-size:13px;padding:8px 5px;border-bottom:1px solid #ddd;text-align:left;}
    td.value {background:#e6f0ff;font-size:13px;padding:8px 5px;border-bottom:1px solid #ddd;text-align:center;}
    tr.header-row {background:#336699;color:#fff;}
    tr.header-row td {color:#fff;font-weight:bold;padding:10px 5px;font-size:12px;}
    .status-ok {color:#0a0;font-weight:bold;}
    .status-na {color:#c00;font-weight:bold;}
    .status-away {color:#f80;font-weight:bold;}
    .eco-status-badge {display:inline-block;padding:4px 12px;border-radius:4px;font-weight:bold;font-size:12px;margin-left:10px;}
    .eco-online {background:#0a0;color:#fff;}
    .eco-offline {background:#c00;color:#fff;}
    .pump-info {font-size:11px;color:#666;font-style:italic;margin-top:5px;text-align:center;}
    .pump-total {font-size:12px;color:#369;font-weight:bold;margin-top:3px;text-align:center;}
    .circuits-table-wrapper {overflow-x:auto;-webkit-overflow-scrolling:touch;border:2px solid #ddd;border-radius:8px;margin:15px 0;}
    table.circuits-table {min-width:1400px;}
    tr.override-active {border-left:4px solid #c00;}
    .btn-override {padding:4px 8px;margin:2px;font-size:11px;cursor:pointer;border:none;border-radius:4px;background:#369;color:#fff;}
    .btn-override:hover {background:#036;}
    .btn-override-cancel {background:#c00;}
    .btn-override-cancel:hover {background:#900;}
    .override-badge {background:#c00;color:#fff;padding:4px 8px;border-radius:4px;font-size:11px;font-weight:bold;}
    .override-badge-off {background:#666;color:#fff;}
    
    /* V53.5: NIEUW - Temperature color zones */
    .temp-cold { color: #0af; font-weight: bold; }
    .temp-warm { color: #0a0; font-weight: bold; }
    .temp-hot { color: #fa0; font-weight: bold; }
    .temp-danger { color: #c00; font-weight: bold; }
    
    /* V53.5: NIEUW - Energy color zones */
    .energy-low { color: #999; }
    .energy-ok { color: #0a0; }
    .energy-high { color: #fa0; }
    .energy-max { color: #c00; }
    
    /* V53.5: NIEUW - Trend indicators */
    .trend { font-size: 18px; margin-left: 8px; display: inline-block; }
    .trend-up { color: #c00; }
    .trend-down { color: #0a0; }
    .trend-stable { color: #999; }
    
    @media (max-width: 600px) {
      .container {flex-direction:column;}
      .sidebar {width:100%;border-right:none;border-bottom:3px solid #c00;display:flex;justify-content:center;}
      .sidebar a {width:60px;margin:0 3px;}
      .main {padding:8px;}
      table.circuits-table {min-width:850px;}
    }
  </style>
</head>
<body>
  <div class="header">
    <div class="header-left">)rawliteral" + room_id + R"rawliteral(</div>
    <div class="header-right" id="header-time">)rawliteral" + String(uptime_sec) + " s &nbsp;&nbsp; " + getFormattedDateTime() + R"rawliteral(</div>
  </div>
  
  <!-- V53.5: NIEUW - Status banner bovenaan! -->
  <div class="status-banner">
    <div class="status-label">STATUS:</div>
    <div class="status-message">)rawliteral" + pumpStatusMsg + R"rawliteral(</div>
  </div>
  
  <div class="container">
    <div class="sidebar">
      <a href="/" class="active">Status</a>
      <a href="/update">OTA</a>
      <a href="/json">JSON</a>
      <a href="/settings">Settings</a>
    </div>
    <div class="main">
      <!-- V53.5: Refresh knop NIET meer bovenaan! Komt onderaan -->
      
      <div class="group-title">CONTROLLER STATUS</div>
      <table>
        <tr><td class="label">MCP23017</td><td class="value">)rawliteral" + String(mcp_available ? "Verbonden" : "Niet gevonden") + R"rawliteral(</td></tr>
        <tr><td class="label">WiFi</td><td class="value">)rawliteral" + WiFi.localIP().toString() + R"rawliteral(</td></tr>
        <tr><td class="label">WiFi RSSI</td><td class="value">)rawliteral" + String(WiFi.RSSI()) + " dBm" + R"rawliteral(</td></tr>
        <tr><td class="label">Free heap</td><td class="value">)rawliteral" + String((ESP.getFreeHeap() * 100) / ESP.getHeapSize()) + " %" + R"rawliteral(</td></tr>
      </table>

      <div class="section-divider"></div>

      <div class="group-title">SCH BOILER (Warmtepomp schuur)</div>
      <table>
        <tr class="header-row"><td class="label">Boiler sensors</td><td class="value">Temperatuur</td><td class="value">Status</td></tr>
)rawliteral";
  
  for (int i = 0; i < 6; i++) {
    String status = sensor_ok[i] ? "OK" : "Error";
    String temp_str = sensor_ok[i] ? String(sch_temps[i], 1) + " &deg;C" : "--";
    html += "<tr><td class=\"label\">" + sensor_nicknames[i] + "</td><td class=\"value\">" + temp_str + "</td><td class=\"value\">" + status + "</td></tr>";
  }
  
  html += R"rawliteral(
      </table>
      <table>
        <tr>
          <td class="label">Energieinhoud (QTot)</td>
          <td class="value"><b>)rawliteral";
  html += String(sch_qtot, 2) + " kWh";
  html += R"rawliteral(</b></td>
        </tr>
      </table>

      <div class="blue-divider"></div>

      <div class="group-title">ECO BOILER (Solar + Haarden)
        <span class="eco-status-badge )rawliteral";
  html += (eco_boiler.online ? "eco-online" : "eco-offline");
  html += R"rawliteral(">)rawliteral";
  html += (eco_boiler.online ? "ONLINE" : "OFFLINE");
  html += R"rawliteral(</span>
      </div>
      <table>
        <tr>
          <td class="label">ETop (Hoogste temp)</td>
          <td class="value">)rawliteral";
  
  if (eco_boiler.online) {
    // V53.5: Color coding + trend indicator
    String trend_class = temp_trend == "↑" ? "trend-up" : (temp_trend == "↓" ? "trend-down" : "trend-stable");
    html += "<span class=\"" + temp_color_class + "\">" + String(eco_boiler.temp_top, 1) + " &deg;C</span>";
    html += "<span class=\"trend " + trend_class + "\">" + temp_trend + "</span>";
  } else {
    html += "<span class=\"status-na\">NA</span>";
  }
  
  html += R"rawliteral(</td>
        </tr>
        <tr>
          <td class="label">EQtot (Energie)</td>
          <td class="value">)rawliteral";
  
  if (eco_boiler.online) {
    // V53.5: Color coding + trend indicator
    String trend_class = qtot_trend == "↑" ? "trend-up" : (qtot_trend == "↓" ? "trend-down" : "trend-stable");
    html += "<span class=\"" + energy_color_class + "\">" + String(eco_boiler.qtot, 2) + " kWh</span>";
    html += "<span class=\"trend " + trend_class + "\">" + qtot_trend + "</span>";
  } else {
    html += "<span class=\"status-na\">NA</span>";
  }
  
  html += R"rawliteral(</td>
        </tr>
        <tr>
          <td class="label">Tmax pomp (Start)</td>
          <td class="value">)rawliteral";
  html += String(eco_max_temp, 1) + " &deg;C";
  html += R"rawliteral(</td>
        </tr>
        <tr>
          <td class="label">Tmin pomp (Stop)</td>
          <td class="value">)rawliteral";
  html += String(eco_min_temp, 1) + " &deg;C";
  html += R"rawliteral(</td>
        </tr>
        <tr>
          <td class="label">SCH Pomp</td>
          <td class="value">)rawliteral";
  
  // Server-side check of timer nog geldig is
  if (sch_pump_manual) {
    unsigned long elapsed = millis() - sch_pump_manual_start;
    
    if (elapsed < MANUAL_PUMP_DURATION) {
      unsigned long remaining = (MANUAL_PUMP_DURATION - elapsed) / 1000;
      
      String badge_class = sch_pump_manual_on ? "override-badge" : "override-badge override-badge-off";
      String badge_text = sch_pump_manual_on ? "ON" : "OFF";
      
      html += "<span class=\"" + badge_class + " timer\" data-remaining=\"" + String(remaining) + "\">" + 
              badge_text + " " + String(remaining / 60) + ":" + String(remaining % 60, DEC) + "</span> ";
      html += "<button class=\"btn-override btn-override-cancel\" onclick=\"cancelPump('sch')\">×</button>";
    } else {
      html += "<button class=\"btn-override\" onclick=\"setPump('sch', true)\">ON</button> ";
      html += "<button class=\"btn-override\" onclick=\"setPump('sch', false)\">OFF</button>";
    }
  } else {
    html += "<button class=\"btn-override\" onclick=\"setPump('sch', true)\">ON</button> ";
    html += "<button class=\"btn-override\" onclick=\"setPump('sch', false)\">OFF</button>";
  }
  
  html += R"rawliteral(
)rawliteral";
  
  if (last_sch_pump.timestamp > 0) {
    time_t event_time = last_sch_pump.timestamp;
    struct tm tm;
    localtime_r(&event_time, &tm);
    char time_buf[32];
    strftime(time_buf, sizeof(time_buf), "%d-%m-%Y %H:%M", &tm);
    html += "            <div class=\"pump-info\">Laatste: " + String(time_buf) + " - " + String(last_sch_pump.kwh_pumped, 2) + " kWh</div>";
  } else {
    html += "            <div class=\"pump-info\">Laatste: NA</div>";
  }
  
  html += "            <div class=\"pump-total\">TOTAAL: " + String(total_sch_kwh, 1) + " kWh</div>";
  
  html += R"rawliteral(
          </td>
        </tr>
        <tr>
          <td class="label">WON Pomp</td>
          <td class="value">)rawliteral";
  
  if (won_pump_manual) {
    unsigned long elapsed = millis() - won_pump_manual_start;
    
    if (elapsed < MANUAL_PUMP_DURATION) {
      unsigned long remaining = (MANUAL_PUMP_DURATION - elapsed) / 1000;
      
      String badge_class = won_pump_manual_on ? "override-badge" : "override-badge override-badge-off";
      String badge_text = won_pump_manual_on ? "ON" : "OFF";
      
      html += "<span class=\"" + badge_class + " timer\" data-remaining=\"" + String(remaining) + "\">" + 
              badge_text + " " + String(remaining / 60) + ":" + String(remaining % 60, DEC) + "</span> ";
      html += "<button class=\"btn-override btn-override-cancel\" onclick=\"cancelPump('won')\">×</button>";
    } else {
      html += "<button class=\"btn-override\" onclick=\"setPump('won', true)\">ON</button> ";
      html += "<button class=\"btn-override\" onclick=\"setPump('won', false)\">OFF</button>";
    }
  } else {
    html += "<button class=\"btn-override\" onclick=\"setPump('won', true)\">ON</button> ";
    html += "<button class=\"btn-override\" onclick=\"setPump('won', false)\">OFF</button>";
  }
  
  html += R"rawliteral(
)rawliteral";
  
  if (last_won_pump.timestamp > 0) {
    time_t event_time = last_won_pump.timestamp;
    struct tm tm;
    localtime_r(&event_time, &tm);
    char time_buf[32];
    strftime(time_buf, sizeof(time_buf), "%d-%m-%Y %H:%M", &tm);
    html += "            <div class=\"pump-info\">Laatste: " + String(time_buf) + " - " + String(last_won_pump.kwh_pumped, 2) + " kWh</div>";
  } else {
    html += "            <div class=\"pump-info\">Laatste: NA</div>";
  }
  
  html += "            <div class=\"pump-total\">TOTAAL: " + String(total_won_kwh, 1) + " kWh</div>";
  
  html += R"rawliteral(
          </td>
        </tr>
      </table>

      <div class="section-divider"></div>

      <div class="group-title">VENTILATIE</div>
      <table>
        <tr><td class="label">Max request</td><td class="value">)rawliteral" + String(vent_percent) + " %" + R"rawliteral(</td></tr>
      </table>

      <div class="section-divider"></div>
      
      <div class="group-title">CIRCUITS</div>
      <div class="circuits-table-wrapper">
      <table class="circuits-table">
        <tr class="header-row">
          <td>#</td><td>Naam</td><td>IP</td><td>mDNS</td><td>Set</td><td>Temp</td><td>Heat</td><td>Home</td>
          <td>TSTAT</td><td>Pomp</td><td>P</td><td>Duty</td><td>Vent</td><td>Override</td>
        </tr>
)rawliteral";

  for (int i = 0; i < circuits_num; i++) {
    String row_class = circuits[i].override_active ? " class=\"override-active\"" : "";
    html += "<tr" + row_class + ">";
    html += "<td class=\"label\">" + String(i + 1) + "</td>";
    html += "<td class=\"value\">" + circuits[i].name + "</td>";
    
    String ip_status = circuits[i].ip.length() > 0 ? (circuits[i].online ? "✓" : "✗") : "-";
    String mdns_status = circuits[i].mdns.length() > 0 ? (circuits[i].online ? "✓" : "✗") : "-";
    html += "<td class=\"value " + String(circuits[i].online ? "status-ok" : "status-na") + "\">" + ip_status + "</td>";
    html += "<td class=\"value " + String(circuits[i].online ? "status-ok" : "status-na") + "\">" + mdns_status + "</td>";
    
    html += "<td class=\"value\">" + String(circuits[i].online && circuits[i].setpoint > 0 ? String(circuits[i].setpoint) + "&deg;C" : "--") + "</td>";
    html += "<td class=\"value\">" + String(circuits[i].online && circuits[i].room_temp > 0 ? String(circuits[i].room_temp, 1) + "&deg;C" : "--") + "</td>";
    String heat_str = circuits[i].online ? (circuits[i].heat_request ? "ON" : "OFF") : "--";
    html += "<td class=\"value\">" + heat_str + "</td>";
    
    String home_str = "--", home_class = "";
    if (circuits[i].online) {
      home_str = circuits[i].home_status == 1 ? "Thuis" : "Away";
      home_class = circuits[i].home_status == 1 ? "status-ok" : "status-away";
    } else {
      home_str = "NA";
      home_class = "status-na";
    }
    html += "<td class=\"value " + home_class + "\">" + home_str + "</td>";
    
    String tstat_status = "-";
    if (circuits[i].has_tstat && mcp_available && circuits[i].tstat_pin < 13) {
      tstat_status = (mcp.digitalRead(circuits[i].tstat_pin) == LOW) ? "ON" : "OFF";
    }
    html += "<td class=\"value\">" + tstat_status + "</td>";
    
    html += "<td class=\"value\">" + String(circuits[i].heating_on ? "<b>AAN</b>" : "UIT") + "</td>";
    html += "<td class=\"value\">" + (circuits[i].heating_on ? String(circuits[i].power_kw, 1) + " kW" : "0 kW") + "</td>";
    html += "<td class=\"value\">" + String(circuits[i].duty_cycle, 1) + "%</td>";
    html += "<td class=\"value\">" + String(circuits[i].vent_request) + "%</td>";
    
    html += "<td class=\"value\">";
    if (circuits[i].override_active) {
      unsigned long remaining = (600000UL - (millis() - circuits[i].override_start)) / 1000;
      html += "<span class=\"override-badge timer\" data-remaining=\"" + String(remaining) + "\">" + 
              String(circuits[i].override_state ? "ON" : "OFF") + " " + String(remaining / 60) + ":" + 
              String(remaining % 60, DEC) + "</span> ";
      html += "<button class=\"btn-override btn-override-cancel\" onclick=\"cancelOverride(" + String(i) + ")\">×</button>";
    } else {
      html += "<button class=\"btn-override\" onclick=\"setOverride(" + String(i) + ", true)\">ON</button> ";
      html += "<button class=\"btn-override\" onclick=\"setOverride(" + String(i) + ", false)\">OFF</button>";
    }
    html += "</td></tr>";
  }
  
  html += R"rawliteral(
        <tr style="border-top:2px solid #369;">
          <td colspan="9" class="label"><b>TOTAAL</b></td>
          <td class="value"></td>
          <td class="value"><b>)rawliteral" + String(total_power, 1) + " kW" + R"rawliteral(</b></td>
          <td colspan="3" class="value"><b>)rawliteral" + String(vent_percent) + " %" + R"rawliteral(</b></td>
        </tr>
      </table>
      </div>
      
      <!-- V53.5: Refresh knop ONDERAAN de pagina! -->
      <button class="refresh-btn" onclick="refreshData()">🔄 Refresh Data</button>
    </div>
  </div>

<script>
function setPump(type, state) {
  const endpoint = type === 'sch' 
    ? (state ? '/pump_sch_on' : '/pump_sch_off')
    : (state ? '/pump_won_on' : '/pump_won_off');
  fetch(endpoint).then(() => setTimeout(() => location.reload(), 500));
}

function cancelPump(type) {
  const endpoint = type === 'sch' ? '/pump_sch_cancel' : '/pump_won_cancel';
  fetch(endpoint).then(() => setTimeout(() => location.reload(), 500));
}

function setOverride(circuit, state) {
  const endpoint = state ? '/circuit_override_on' : '/circuit_override_off';
  fetch(endpoint + '?circuit=' + circuit).then(() => setTimeout(() => location.reload(), 500));
}

function cancelOverride(circuit) {
  fetch('/circuit_override_cancel?circuit=' + circuit).then(() => setTimeout(() => location.reload(), 500));
}

// Timer countdown met auto-refresh bij 0
setInterval(() => {
  document.querySelectorAll('.timer').forEach(badge => {
    let remaining = parseInt(badge.dataset.remaining);
    if (remaining > 0) {
      remaining--;
      badge.dataset.remaining = remaining;
      const state = badge.textContent.split(' ')[0];
      badge.textContent = state + ' ' + Math.floor(remaining/60) + ':' + (remaining%60).toString().padStart(2,'0');
    } else if (remaining === 0) {
      console.log('Timer reached 0 - auto-refreshing page');
      setTimeout(() => location.reload(), 1000);
    }
  });
}, 1000);

function refreshData() {
  location.reload();
}
</script>
</body>
</html>
)rawliteral";
  
  return html;
}















// ============== DEEL 4/5: SETTINGS PAGE & ENDPOINTS ==============

void setupWebServer() {
  // Captive portal - redirect all requests naar settings in AP mode
  server.onNotFound([](AsyncWebServerRequest *request){
    if (ap_mode_active) {
      request->redirect("/settings");
    } else {
      request->send(404, "text/plain", "Not found");
    }
  });

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html; charset=utf-8", getMainPage());
  });

  server.on("/json", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "application/json", getLogData());
  });

  server.on("/scan", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "application/json", getWifiScanJson());
  });

  server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = R"rawliteral(
<!DOCTYPE html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>)rawliteral" + room_id + R"rawliteral( - OTA</title>
<style>
body{font-family:Arial,sans-serif;background:#fff;margin:0;padding:20px;text-align:center;}
h1{color:#369;}
.button{background:#369;color:#fff;padding:12px 24px;border:none;border-radius:8px;cursor:pointer;font-size:16px;margin:10px;}
.button:hover{background:#036;}
.reboot{background:#c00;}
.reboot:hover{background:#900;}
</style></head><body>
<h1>OTA Firmware Update</h1>
<form method="POST" action="/update" enctype="multipart/form-data">
<input type="file" name="update" accept=".bin"><br><br>
<button class="button" type="submit">Upload Firmware</button>
</form><br>
<button class="button reboot" onclick="if(confirm('Reboot?'))location.href='/reboot'">Reboot</button>
<br><br><a href="/">← Terug</a>
</body></html>
)rawliteral";
    request->send(200, "text/html; charset=utf-8", html);
  });

  server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request) {
    bool success = !Update.hasError();
    request->send(200, "text/html", success 
      ? "<h2 style='color:#0f0'>Update OK!</h2><p>Rebooting...</p>" 
      : "<h2 style='color:#f00'>Update FAILED!</h2>");
    if (success) { delay(1000); ESP.restart(); }
  }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
    if (!index) {
      Serial.println("\n=== OTA UPDATE ===");
      Update.begin(UPDATE_SIZE_UNKNOWN);
    }
    Update.write(data, len);
    if (final && Update.end(true)) Serial.println("OTA OK");
  });

  server.on("/reboot", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", "<h2>Rebooting...</h2>");
    delay(500);
    ESP.restart();
  });

  server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *request){
    String sensorNamesHtml = "";
    for (int i = 0; i < 6; i++) {
      sensorNamesHtml += "<label style=\"display:block;margin:6px 0;\">Sensor " + String(i + 1) + ": ";
      sensorNamesHtml += "<input type=\"text\" name=\"sensor_nick_" + String(i) + "\" value=\"" + sensor_nicknames[i] + "\" style=\"width:220px;\"></label>";
    }
    
    String circuitsHtml = "";
    for (int i = 0; i < circuits_num; i++) {
      circuitsHtml += "<div style=\"background:#f5f5f5;border:1px solid #ccc;border-radius:8px;padding:15px;margin:15px 0;\">";
      circuitsHtml += "<h4 style=\"margin:0 0 10px 0;color:#369;\">Circuit " + String(i + 1) + "</h4>";
      circuitsHtml += "<table style=\"width:100%;\">";
      circuitsHtml += "<tr><td style=\"width:35%;padding:8px;\">Naam</td><td><input type=\"text\" name=\"circuit_name_" + String(i) + "\" value=\"" + circuits[i].name + "\" style=\"width:100%;padding:8px;\"></td></tr>";
      circuitsHtml += "<tr><td>IP adres</td><td><input type=\"text\" name=\"circuit_ip_" + String(i) + "\" value=\"" + circuits[i].ip + "\" style=\"width:100%;padding:8px;\"></td></tr>";
      circuitsHtml += "<tr><td>mDNS naam</td><td><input type=\"text\" name=\"circuit_mdns_" + String(i) + "\" value=\"" + circuits[i].mdns + "\" style=\"width:100%;padding:8px;\" placeholder=\"ZONDER .local\"></td></tr>";
      circuitsHtml += "<tr><td>Vermogen (kW)</td><td><input type=\"number\" step=\"0.001\" name=\"circuit_power_" + String(i) + "\" value=\"" + String(circuits[i].power_kw, 3) + "\" style=\"width:100%;padding:8px;\"></td></tr>";
      circuitsHtml += "<tr><td>TSTAT</td><td><input type=\"checkbox\" name=\"circuit_tstat_" + String(i) + "\" value=\"1\"" + String(circuits[i].has_tstat ? " checked" : "") + "> ";
      circuitsHtml += "Pin: <select name=\"circuit_tstat_pin_" + String(i) + "\" style=\"padding:8px;\">";
      circuitsHtml += "<option value=\"255\"" + String(circuits[i].tstat_pin == 255 ? " selected" : "") + ">Geen</option>";
      circuitsHtml += "<option value=\"10\"" + String(circuits[i].tstat_pin == 10 ? " selected" : "") + ">Pin 10</option>";
      circuitsHtml += "<option value=\"11\"" + String(circuits[i].tstat_pin == 11 ? " selected" : "") + ">Pin 11</option>";
      circuitsHtml += "<option value=\"12\"" + String(circuits[i].tstat_pin == 12 ? " selected" : "") + ">Pin 12</option>";
      circuitsHtml += "</select></td></tr>";
      circuitsHtml += "</table></div>";
    }

    String html = R"rawliteral(
<!DOCTYPE html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>)rawliteral" + room_id + R"rawliteral( - Settings</title>
<style>
body{font-family:Arial,sans-serif;background:#fff;margin:0;padding:0;}
.header{display:flex;background:#ffcc00;color:#000;padding:10px 15px;font-size:18px;font-weight:bold;}
.header-left{flex:1;}
.header-right{flex:1;text-align:right;font-size:15px;}
.container{display:flex;min-height:calc(100vh - 60px);}
.sidebar{width:80px;padding:10px 5px;background:#fff;border-right:3px solid #c00;}
.sidebar a{display:block;background:#369;color:#fff;padding:8px;margin:8px auto;text-decoration:none;font-weight:bold;font-size:12px;border-radius:6px;text-align:center;width:60px;}
.sidebar a:hover{background:#036;}
.sidebar a.active{background:#c00;}
.main{flex:1;padding:20px;overflow-y:auto;}
h2{color:#369;border-bottom:2px solid #369;padding-bottom:10px;}
.warning{background:#ffe6e6;border:2px solid #c00;padding:15px;margin:20px 0;border-radius:8px;text-align:center;font-weight:bold;color:#900;}
table{width:100%;margin:15px 0;}
td{padding:10px;}
input,select{padding:8px;border:1px solid #ccc;border-radius:4px;}
.btn{background:#369;color:#fff;padding:12px 30px;border:none;border-radius:6px;font-size:16px;cursor:pointer;margin:20px 10px;}
.btn:hover{background:#036;}
@media (max-width: 600px) {
  .container{flex-direction:column;}
  .sidebar{width:100%;border-right:none;border-bottom:3px solid #c00;display:flex;justify-content:center;}
  .sidebar a{width:60px;margin:0 3px;}
  .main{padding:8px;}
}
</style></head><body>
<div class="header">
  <div class="header-left">)rawliteral" + room_id + R"rawliteral(</div>
  <div class="header-right">Instellingen</div>
</div>
<div class="container">
  <div class="sidebar">
    <a href="/">Status</a>
    <a href="/update">OTA</a>
    <a href="/json">JSON</a>
    <a href="/settings" class="active">Settings</a>
  </div>
  <div class="main">
<div class="warning">OPGEPAST: Wijzigt permanente instellingen!<br>Verkeerde WiFi kan controller onbereikbaar maken!<br><br><strong>Geen WiFi?</strong> Controller start AP: HVAC-Setup<br>Ga naar http://192.168.4.1/settings</div>

<!-- MAC ADRES BOX -->
<div style="background:#e6f0ff;border:3px solid #336699;padding:20px;margin:20px;border-radius:8px;text-align:center;">
  <h3 style="margin:0 0 10px 0;color:#336699;">📡 Controller MAC Adres</h3>
  <div style="font-size:20px;font-weight:bold;color:#003366;font-family:monospace;background:#fff;padding:10px;border-radius:4px;display:inline-block;margin:10px 0;border:2px solid #336699;">)rawliteral" + mac_address + R"rawliteral(</div>
  <div style="font-size:13px;color:#666;margin-top:10px;">💡 Kopieer dit MAC-adres voor DHCP-reservering in je router</div>
</div>

<!-- AANBEVOLEN CONFIGURATIE -->
<div style="background:#fffacd;border:2px solid #336699;padding:15px;margin:20px;border-radius:8px;font-size:14px;">
  <h4 style="margin:0 0 10px 0;color:#336699;">✅ Aanbevolen: DHCP met MAC-reservering</h4>
  <ol style="margin:10px 0;padding-left:25px;line-height:1.6;">
    <li>Kopieer het MAC-adres hierboven</li>
    <li>Log in op je router (meestal 192.168.1.1)</li>
    <li>Ga naar: LAN → DHCP Server → Manual Assignment</li>
    <li>Voeg toe: MAC-adres + gewenst IP (bijv. 192.168.1.99)</li>
    <li>Laat hieronder het "Static IP" veld <strong>LEEG</strong></li>
    <li>Sla op en reboot deze controller</li>
  </ol>
</div>


<form action="/save_settings" method="get">
  
  <h2>WiFi Configuratie</h2>
  <table>
    <tr><td style="width:35%;">WiFi SSID</td><td><input type="text" name="wifi_ssid" value=")rawliteral" + wifi_ssid + R"rawliteral(" style="width:100%;"></td></tr>
    <tr><td>WiFi Password</td><td><input type="password" name="wifi_pass" value=")rawliteral" + wifi_pass + R"rawliteral(" style="width:100%;"></td></tr>
    <tr><td>Static IP</td><td><input type="text" name="static_ip" value=")rawliteral" + static_ip_str + R"rawliteral(" placeholder="leeg = DHCP" style="width:100%;"></td></tr>
  </table>

  <h2>Basis Instellingen</h2>
  <table>
    <tr><td style="width:35%;">Room naam</td><td><input type="text" name="room_id" value=")rawliteral" + room_id + R"rawliteral(" required style="width:100%;"></td></tr>
    <tr><td>Aantal circuits</td><td><input type="number" name="circuits_num" min="1" max="16" value=")rawliteral" + String(circuits_num) + R"rawliteral(" style="width:100%;"></td></tr>
    <tr><td>Poll interval (sec)</td><td><input type="number" min="5" name="poll_interval" value=")rawliteral" + String(poll_interval) + R"rawliteral(" style="width:100%;"></td></tr>
  </table>

  <h2>ECO Boiler Instellingen</h2>
  <table>
    <tr><td style="width:35%;">ECO IP adres</td><td><input type="text" name="eco_ip" value=")rawliteral" + eco_controller_ip + R"rawliteral(" style="width:100%;"></td></tr>
    <tr><td>ECO mDNS naam</td><td><input type="text" name="eco_mdns" value=")rawliteral" + eco_controller_mdns + R"rawliteral(" style="width:100%;"></td></tr>
    <tr><td>ECO Threshold (kWh)</td><td><input type="number" step="0.1" name="eco_thresh" value=")rawliteral" + String(eco_threshold) + R"rawliteral(" style="width:100%;"></td></tr>
    <tr><td>ECO Hysteresis (kWh)</td><td><input type="number" step="0.1" name="eco_hyst" value=")rawliteral" + String(eco_hysteresis) + R"rawliteral(" style="width:100%;"></td></tr>
    <tr><td>ECO Tmin - Stop (&deg;C)</td><td><input type="number" step="0.1" name="eco_min_temp" value=")rawliteral" + String(eco_min_temp) + R"rawliteral(" style="width:100%;"></td></tr>
    <tr><td>ECO Tmax - Start (&deg;C)</td><td><input type="number" step="0.1" name="eco_max_temp" value=")rawliteral" + String(eco_max_temp) + R"rawliteral(" style="width:100%;"></td></tr>
  </table>

  <h2>Boiler Qtot Berekening</h2>
  <table>
    <tr><td style="width:35%;">Reference temp (&deg;C)</td><td><input type="number" step="0.1" name="boiler_ref_temp" value=")rawliteral" + String(boiler_ref_temp) + R"rawliteral(" style="width:100%;"></td></tr>
    <tr><td>Volume per laag (L)</td><td><input type="number" step="1" name="boiler_volume" value=")rawliteral" + String(boiler_layer_volume) + R"rawliteral(" style="width:100%;"></td></tr>
  </table>

  <h2>Sensor Nicknames</h2>
  <div style="padding:10px;">)rawliteral" + sensorNamesHtml + R"rawliteral(</div>

  <h2>Verwarmingscircuits</h2>
  )rawliteral" + circuitsHtml + R"rawliteral(

  <div style="text-align:center;">
    <button type="submit" class="btn">Opslaan & Reboot</button>
    <a href="/" class="btn" style="display:inline-block;text-decoration:none;background:#c00;">Annuleren</a>
  </div>
</form>
</div></div>
</body></html>
)rawliteral";
    request->send(200, "text/html; charset=utf-8", html);
  });

  server.on("/save_settings", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("\n=== SAVE SETTINGS ===");
    
    if (request->hasArg("wifi_ssid")) preferences.putString(NVS_WIFI_SSID, request->arg("wifi_ssid"));
    if (request->hasArg("wifi_pass")) preferences.putString(NVS_WIFI_PASS, request->arg("wifi_pass"));
    if (request->hasArg("static_ip")) preferences.putString(NVS_STATIC_IP, request->arg("static_ip"));
    if (request->hasArg("room_id")) preferences.putString(NVS_ROOM_ID, request->arg("room_id"));
    if (request->hasArg("circuits_num")) preferences.putInt(NVS_CIRCUITS_NUM, request->arg("circuits_num").toInt());
    if (request->hasArg("poll_interval")) preferences.putInt(NVS_POLL_INTERVAL, request->arg("poll_interval").toInt());
    if (request->hasArg("eco_ip")) preferences.putString(NVS_ECO_IP, request->arg("eco_ip"));
    if (request->hasArg("eco_mdns")) preferences.putString(NVS_ECO_MDNS, request->arg("eco_mdns"));
    if (request->hasArg("eco_thresh")) preferences.putFloat(NVS_ECO_THRESHOLD, request->arg("eco_thresh").toFloat());
    if (request->hasArg("eco_hyst")) preferences.putFloat(NVS_ECO_HYSTERESIS, request->arg("eco_hyst").toFloat());
    if (request->hasArg("eco_min_temp")) preferences.putFloat(NVS_ECO_MIN_TEMP, request->arg("eco_min_temp").toFloat());
    if (request->hasArg("eco_max_temp")) preferences.putFloat(NVS_ECO_MAX_TEMP, request->arg("eco_max_temp").toFloat());
    if (request->hasArg("boiler_ref_temp")) preferences.putFloat(NVS_BOILER_REF_TEMP, request->arg("boiler_ref_temp").toFloat());
    if (request->hasArg("boiler_volume")) preferences.putFloat(NVS_BOILER_VOLUME, request->arg("boiler_volume").toFloat());
    
    for (int i = 0; i < 6; i++) {
      String param = "sensor_nick_" + String(i);
      if (request->hasArg(param.c_str())) {
        String nick = request->arg(param.c_str());
        nick.trim();
        if (nick.length() == 0) nick = "Sensor " + String(i + 1);
        preferences.putString((String(NVS_SENSOR_NICK_BASE) + i).c_str(), nick);
      }
    }
    
    int save_count = request->arg("circuits_num").toInt();
    if (save_count < 1) save_count = 1;
    if (save_count > 16) save_count = 16;

    for (int i = 0; i < save_count; i++) {
      String name_val = request->arg(("circuit_name_" + String(i)).c_str());
      if (name_val.length() == 0) name_val = "Circuit " + String(i + 1);
      preferences.putString(("c" + String(i) + "_name").c_str(), name_val);
      preferences.putString(("c" + String(i) + "_ip").c_str(), request->arg(("circuit_ip_" + String(i)).c_str()));
      
      String mdns_val = request->arg(("circuit_mdns_" + String(i)).c_str());
      mdns_val.replace(".local", "");
      mdns_val.trim();
      preferences.putString(("c" + String(i) + "_mdns").c_str(), mdns_val);
      preferences.putFloat(("c" + String(i) + "_power").c_str(), request->arg(("circuit_power_" + String(i)).c_str()).toFloat());
      preferences.putBool(("c" + String(i) + "_tstat").c_str(), request->hasArg(("circuit_tstat_" + String(i)).c_str()));
      
      int pin_val = 255;
      if (request->hasArg(("circuit_tstat_pin_" + String(i)).c_str())) {
        pin_val = request->arg(("circuit_tstat_pin_" + String(i)).c_str()).toInt();
        if (pin_val != 10 && pin_val != 11 && pin_val != 12) pin_val = 255;
      }
      preferences.putInt(("c" + String(i) + "_pin").c_str(), pin_val);
    }

    Serial.println("Settings saved!");
    request->send(200, "text/html", "<h2 style='text-align:center;color:#369;'>Opgeslagen! Rebooting...</h2>");
    delay(2000);
    ESP.restart();
  });

  server.on("/circuit_override_on", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasArg("circuit")) {
      int idx = request->arg("circuit").toInt();
      if (idx >= 0 && idx < circuits_num) {
        circuits[idx].override_active = true;
        circuits[idx].override_state = true;
        circuits[idx].override_start = millis();
      }
    }
    request->send(200, "text/plain", "OK");
  });

  server.on("/circuit_override_off", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasArg("circuit")) {
      int idx = request->arg("circuit").toInt();
      if (idx >= 0 && idx < circuits_num) {
        circuits[idx].override_active = true;
        circuits[idx].override_state = false;
        circuits[idx].override_start = millis();
      }
    }
    request->send(200, "text/plain", "OK");
  });

  server.on("/circuit_override_cancel", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasArg("circuit")) {
      int idx = request->arg("circuit").toInt();
      if (idx >= 0 && idx < circuits_num) {
        circuits[idx].override_active = false;
      }
    }
    request->send(200, "text/plain", "OK");
  });

  // Pump endpoints met ON/OFF override support
  server.on("/pump_sch_on", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("\n=== MANUAL PUMP CONTROL ===");
    Serial.println("Type: SCH");
    Serial.println("Action: ON (override)");
    Serial.println("Duration: 60 seconds");
    Serial.printf("Relay %d: LOW (pump ON)\n", RELAY_PUMP_SCH);
    
    sch_pump_manual = true;
    sch_pump_manual_on = true;
    sch_pump_manual_start = millis();
    
    if (mcp_available) mcp.digitalWrite(RELAY_PUMP_SCH, LOW);
    request->send(200, "text/plain", "OK");
  });

  server.on("/pump_sch_off", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("\n=== MANUAL PUMP CONTROL ===");
    Serial.println("Type: SCH");
    Serial.println("Action: OFF (override)");
    Serial.println("Duration: 60 seconds");
    Serial.printf("Relay %d: HIGH (pump OFF)\n", RELAY_PUMP_SCH);
    
    sch_pump_manual = true;
    sch_pump_manual_on = false;
    sch_pump_manual_start = millis();
    
    if (mcp_available) mcp.digitalWrite(RELAY_PUMP_SCH, HIGH);
    request->send(200, "text/plain", "OK");
  });

  server.on("/pump_sch_cancel", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("\n=== MANUAL PUMP CANCEL ===");
    Serial.println("Type: SCH");
    Serial.println("Manual mode cancelled");
    
    sch_pump_manual = false;
    request->send(200, "text/plain", "OK");
  });

  server.on("/pump_won_on", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("\n=== MANUAL PUMP CONTROL ===");
    Serial.println("Type: WON");
    Serial.println("Action: ON (override)");
    Serial.println("Duration: 60 seconds");
    Serial.printf("Relay %d: LOW (pump ON)\n", RELAY_PUMP_WON);
    
    won_pump_manual = true;
    won_pump_manual_on = true;
    won_pump_manual_start = millis();
    
    if (mcp_available) mcp.digitalWrite(RELAY_PUMP_WON, LOW);
    request->send(200, "text/plain", "OK");
  });

  server.on("/pump_won_off", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("\n=== MANUAL PUMP CONTROL ===");
    Serial.println("Type: WON");
    Serial.println("Action: OFF (override)");
    Serial.println("Duration: 60 seconds");
    Serial.printf("Relay %d: HIGH (pump OFF)\n", RELAY_PUMP_WON);
    
    won_pump_manual = true;
    won_pump_manual_on = false;
    won_pump_manual_start = millis();
    
    if (mcp_available) mcp.digitalWrite(RELAY_PUMP_WON, HIGH);
    request->send(200, "text/plain", "OK");
  });

  server.on("/pump_won_cancel", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("\n=== MANUAL PUMP CANCEL ===");
    Serial.println("Type: WON");
    Serial.println("Manual mode cancelled");
    
    won_pump_manual = false;
    request->send(200, "text/plain", "OK");
  });

  server.begin();
}

void factoryResetNVS() {
  Serial.println("\n=== FACTORY RESET NVS ===");
  preferences.begin("hvac-config", false);
  preferences.clear();
  preferences.end();
  Serial.println("NVS cleared! Reboot...");
  delay(1000);
  ESP.restart();
}
















// ============== DEEL 5/5: SETUP & LOOP ==============

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n=== HVAC Controller V53.5 ===");
  Serial.println("WIJZIGINGEN t.o.v. V53.4:");
  Serial.println("1. JSON parsing fix: ET/EB → ETopH/EBotL");
  Serial.println("2. Pump timers: 30 min → 1 min cycles");
  Serial.println("3. State machine: WAIT split → WAIT_SCH + WAIT_WON");
  Serial.println("4. Defaults: Tmin=80°C, Tmax=90°C");
  Serial.println("5. Smart status messages (context-aware)");
  Serial.println("6. Status banner bovenaan pagina");
  Serial.println("7. Trend indicators (↑↓→) voor temp & energie");
  Serial.println("8. Color coding voor temp zones");
  Serial.println("9. Refresh knop onderaan");
  Serial.println("10. pump_status in JSON");

  // Factory reset optie
  Serial.println("\nType 'R' binnen 3 sec voor NVS reset...");
  unsigned long start = millis();
  while (millis() - start < 3000) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 'R' || c == 'r') factoryResetNVS();
    }
  }

  // I2C & MCP23017
  Wire.begin(I2C_SDA, I2C_SCL);
  if (mcp.begin_I2C(0x20)) {
    Serial.println("MCP23017 OK!");
    mcp_available = true;
    
    for (int i = 0; i < 7; i++) {
      mcp.pinMode(i, OUTPUT);
      mcp.digitalWrite(i, HIGH);
    }
    mcp.pinMode(7, INPUT_PULLUP);
    mcp.pinMode(8, OUTPUT);
    mcp.digitalWrite(8, HIGH);
    mcp.pinMode(9, OUTPUT);
    mcp.digitalWrite(9, HIGH);
    mcp.pinMode(10, INPUT_PULLUP);
    mcp.pinMode(11, INPUT_PULLUP);
    mcp.pinMode(12, INPUT_PULLUP);
    mcp.pinMode(13, INPUT_PULLUP);
    mcp.pinMode(14, INPUT_PULLUP);
    mcp.pinMode(15, INPUT_PULLUP);
  } else {
    Serial.println("MCP23017 not found!");
    mcp_available = false;
  }

  // Load NVS
  preferences.begin("hvac-config", false);

  room_id = preferences.getString(NVS_ROOM_ID, "HVAC");
  wifi_ssid = preferences.getString(NVS_WIFI_SSID, "");
  wifi_pass = preferences.getString(NVS_WIFI_PASS, "");
  static_ip_str = preferences.getString(NVS_STATIC_IP, "");
  circuits_num = preferences.getInt(NVS_CIRCUITS_NUM, 7);
  circuits_num = constrain(circuits_num, 1, 16);
  
  eco_threshold = preferences.getFloat(NVS_ECO_THRESHOLD, 15.0);  // V53.5: Updated
  eco_hysteresis = preferences.getFloat(NVS_ECO_HYSTERESIS, 5.0);  // V53.5: Updated
  poll_interval = preferences.getInt(NVS_POLL_INTERVAL, 10);
  eco_controller_ip = preferences.getString(NVS_ECO_IP, "");
  eco_controller_mdns = preferences.getString(NVS_ECO_MDNS, "eco");
  eco_min_temp = preferences.getFloat(NVS_ECO_MIN_TEMP, 80.0);  // V53.5: Stop temp (was 60.0)
  eco_max_temp = preferences.getFloat(NVS_ECO_MAX_TEMP, 90.0);  // V53.5: Start temp (was 80.0)
  boiler_ref_temp = preferences.getFloat(NVS_BOILER_REF_TEMP, 20.0);
  boiler_layer_volume = preferences.getFloat(NVS_BOILER_VOLUME, 50.0);
  
  // Load last pump events
  last_sch_pump.timestamp = preferences.getULong(NVS_LAST_SCH_PUMP, 0);
  last_sch_pump.kwh_pumped = preferences.getFloat(NVS_LAST_SCH_KWH, 0.0);
  last_won_pump.timestamp = preferences.getULong(NVS_LAST_WON_PUMP, 0);
  last_won_pump.kwh_pumped = preferences.getFloat(NVS_LAST_WON_KWH, 0.0);
  
  // Load totalen
  total_sch_kwh = preferences.getFloat(NVS_TOTAL_SCH_KWH, 0.0);
  total_won_kwh = preferences.getFloat(NVS_TOTAL_WON_KWH, 0.0);
  
  Serial.printf("\nBoiler Qtot settings:\n");
  Serial.printf("  Reference temp: %.1f °C\n", boiler_ref_temp);
  Serial.printf("  Volume per laag: %.0f L\n", boiler_layer_volume);
  
  Serial.printf("\nECO Boiler settings:\n");
  Serial.printf("  Tmin (Stop): %.1f °C\n", eco_min_temp);
  Serial.printf("  Tmax (Start): %.1f °C\n", eco_max_temp);
  Serial.printf("  Threshold: %.1f kWh\n", eco_threshold);
  Serial.printf("  Hysteresis: %.1f kWh\n", eco_hysteresis);
  
  Serial.printf("\nECO Pomp totalen:\n");
  Serial.printf("  SCH totaal: %.1f kWh\n", total_sch_kwh);
  Serial.printf("  WON totaal: %.1f kWh\n", total_won_kwh);
  
  if (last_sch_pump.timestamp > 0) {
    Serial.printf("\nLast SCH pump event: %.2f kWh\n", last_sch_pump.kwh_pumped);
  }
  if (last_won_pump.timestamp > 0) {
    Serial.printf("Last WON pump event: %.2f kWh\n", last_won_pump.kwh_pumped);
  }

  for (int i = 0; i < 6; i++) {
    sensor_nicknames[i] = preferences.getString(
      (String(NVS_SENSOR_NICK_BASE) + i).c_str(), 
      "Sensor " + String(i + 1)
    );
  }

  for (int i = 0; i < 16; i++) {
    circuits[i].name = preferences.getString(("c" + String(i) + "_name").c_str(), "Circuit " + String(i + 1));
    circuits[i].ip = preferences.getString(("c" + String(i) + "_ip").c_str(), "");
    circuits[i].mdns = preferences.getString(("c" + String(i) + "_mdns").c_str(), "");
    circuits[i].power_kw = preferences.getFloat(("c" + String(i) + "_power").c_str(), 0.0);
    circuits[i].has_tstat = preferences.getBool(("c" + String(i) + "_tstat").c_str(), false);
    circuits[i].tstat_pin = preferences.getInt(("c" + String(i) + "_pin").c_str(), 255);
    
    circuits[i].online = false;
    circuits[i].heating_on = false;
    circuits[i].vent_request = 0;
    circuits[i].on_time = 1;
    circuits[i].off_time = 100;
    circuits[i].last_change = millis();
    circuits[i].duty_cycle = 0.0;
    circuits[i].override_active = false;
  }

  Serial.println("\n=== Circuit Config ===");
  for (int i = 0; i < circuits_num; i++) {
    Serial.printf("c%d: %s", i + 1, circuits[i].name.c_str());
    if (circuits[i].has_tstat) Serial.printf(" [TSTAT pin %d]", circuits[i].tstat_pin);
    if (circuits[i].ip.length() > 0) Serial.printf(" [IP: %s]", circuits[i].ip.c_str());
    if (circuits[i].mdns.length() > 0) Serial.printf(" [mDNS: %s]", circuits[i].mdns.c_str());
    Serial.printf(" [%.3f kW]\n", circuits[i].power_kw);
  }

  // WiFi verbinding met 20s timeout per poging
  WiFi.mode(WIFI_STA);

  if (static_ip_str.length() > 0 && static_ip.fromString(static_ip_str)) {
    IPAddress gateway(192, 168, 1, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.config(static_ip, gateway, subnet, gateway);
    Serial.println("Static IP: " + static_ip_str);
  }

  if (wifi_ssid.length() > 0) {
    Serial.printf("\nConnecting to '%s'...\n", wifi_ssid.c_str());
    
    int retry_count = 0;
    const int MAX_RETRIES = 5;
    bool connected = false;
    
    while (!connected && retry_count < MAX_RETRIES) {
      WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str());

      // Haal MAC op NA WiFi.begin voor betere compatibiliteit
      mac_address = WiFi.macAddress();
      Serial.println("MAC adres: " + mac_address);
      
      unsigned long start_attempt = millis();
      while (WiFi.status() != WL_CONNECTED && (millis() - start_attempt) < 20000) {
        delay(500);
        Serial.print(".");
      }
      
      if (WiFi.status() == WL_CONNECTED) {
        connected = true;
        Serial.println("\n✓ WiFi connected!");
      } else {
        retry_count++;
        Serial.printf("\n✗ Attempt %d/%d failed\n", retry_count, MAX_RETRIES);
        
        if (retry_count < MAX_RETRIES) {
          Serial.println("Disconnecting and retrying...");
          WiFi.disconnect();
          delay(2000);
        }
      }
    }
    
    if (!connected) {
      Serial.println("\n✗ All WiFi attempts failed -> AP mode");
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi failed -> AP mode");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("HVAC-Setup");
    ap_mode_active = true;
    
    Serial.println("\n=== CAPTIVE PORTAL ACTIVE ===");
    Serial.println("AP SSID: HVAC-Setup");
    Serial.println("AP IP: " + WiFi.softAPIP().toString());
    Serial.println("DNS: Wildcard redirect to settings page");
    Serial.println("→ Connect and settings will auto-open!");
    
    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
  } else {
    Serial.println("\nWiFi connected: " + WiFi.localIP().toString());
    Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
    
    // NTP SYNC
    Serial.println("\nSyncing NTP time...");
    configTime(3600, 3600, "pool.ntp.org", "time.nist.gov");
    setenv("TZ", "CET-1CEST,M3.5.0/02,M10.5.0/03", 1);
    tzset();
    
    int retry = 0;
    time_t now = 0;
    struct tm timeinfo;
    while (now < 1700000000 && retry < 20) {
      time(&now);
      localtime_r(&now, &timeinfo);
      delay(500);
      Serial.print(".");
      retry++;
    }
    
    if (now >= 1700000000) {
      Serial.println(" OK!");
      Serial.println("Time: " + getFormattedDateTime());
    } else {
      Serial.println(" TIMEOUT");
      Serial.println("Tijd wordt niet getoond tot NTP sync lukt");
    }
  }

  // mDNS
  if (MDNS.begin(room_id.c_str())) {
    Serial.println("mDNS: http://" + room_id + ".local");
  }

  setupWebServer();
  Serial.println("\nWeb server started!");
  Serial.println("Ready!\n");
}

void loop() {
  if (ap_mode_active) dnsServer.processNextRequest();

  uptime_sec = millis() / 1000;
  
  readBoilerTemps();
  pollRooms();
  pollEcoBoiler();
  handleEcoPumps();

  delay(100);
}
