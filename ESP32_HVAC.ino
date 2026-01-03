// ESP32_HVACTEST.ino = Centrale HVAC controller voor kelder (ESP32-C6)
// Transition from Photon based to ESP32 based Home automation system. Developed together with ChatGPT & Grok in januari '26.
// Thuis bereikbaar op http://hvactest.local of http://192.168.1.36 => Andere controller: Naam (sectie DNS/MDNS) + static IP aanpassen!
// 02jan26 23:30 Eerste herziene versie op basis van particle sketch (Versie 1.1 door GROK)


#include <WiFi.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_MCP23X17.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

Preferences preferences;

#define ONE_WIRE_PIN   3
#define I2C_SDA       13
#define I2C_SCL       11
#define VENT_FAN_PIN  20   // PWM → externe 0-10V converter

OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature ds18b20(&oneWire);
Adafruit_MCP23X17 mcp;

AsyncWebServer server(80);
DNSServer dnsServer;
const byte DNS_PORT = 53;

// === NVS keys ===
const char* NVS_ROOM_ID            = "room_id";
const char* NVS_WIFI_SSID          = "wifi_ssid";
const char* NVS_WIFI_PASS          = "wifi_password";
const char* NVS_STATIC_IP          = "static_ip";
const char* NVS_CIRCUITS_NUM       = "circuits_num";      // 1-16
const char* NVS_CIRCUIT_NICK_BASE  = "circuit_nick_";     // circuit_nick_0 t/m 15
const char* NVS_ROOM_IP_BASE       = "room_ip_";          // room_ip_0 t/m 9
const char* NVS_ROOM_NAME_BASE     = "room_name_";        // room_name_0 t/m 9
const char* NVS_ECO_THRESHOLD      = "eco_thresh";
const char* NVS_ECO_HYSTERESIS     = "eco_hyst";
const char* NVS_POLL_INTERVAL      = "poll_interval";

// === Runtime variabelen ===
String room_id = "HVAC";
String wifi_ssid = "";
String wifi_pass = "";
String static_ip_str = "";
IPAddress static_ip;

int circuits_num = 7;
String circuit_nicknames[16];

String room_ips[10];
String room_names[10];

float eco_threshold = 12.0;
float eco_hysteresis = 2.0;
int poll_interval = 20;

// Relays op MCP23017
#define RELAY_PUMP_SCH  8   // pin 8 = relay 9 (ECO → SCH)
#define RELAY_PUMP_WON  9   // pin 9 = relay 10 (ECO → WON)

// States
bool circuit_on[16] = {false};
bool pump_sch_on = false;
bool pump_won_on = false;
int vent_percent = 0;

// Duty-cycle
unsigned long circuit_on_time[16] = {0};
unsigned long circuit_off_time[16] = {0};
unsigned long circuit_last_change[16] = {0};
float circuit_dc[16] = {0.0};

// Boiler data
float sch_temps[6];
float eco_temps[6];
float sch_qtot = 0.0;
float eco_qtot = 0.0;

unsigned long last_poll = 0;
unsigned long last_energy_calc = 0;
unsigned long last_wifi_check = 0;

bool ap_mode_active = false;
wl_status_t last_wifi_status = WL_IDLE_STATUS;

// Hardcoded sensor addresses (SCH 0-5, ECO 6-11)
DeviceAddress sensor_addresses[12] = {
  {0x28,0xDB,0xB5,0x03,0x00,0x00,0x80,0xBB}, // KSTopH
  {0x28,0x7C,0xF0,0x03,0x00,0x00,0x80,0x59}, // KSTopL
  {0x28,0x72,0xDB,0x03,0x00,0x00,0x80,0xC2}, // KSMidH
  {0x28,0xAA,0xFB,0x03,0x00,0x00,0x80,0x5F}, // KSMidL
  {0x28,0x49,0xDD,0x03,0x00,0x00,0x80,0x4B}, // KSBotH
  {0x28,0xC3,0xD6,0x03,0x00,0x00,0x80,0x1E}, // KSBotL
  {0x28,0x3A,0xBC,0x07,0x00,0x00,0x80,0x58}, // ECO TopH
  {0x28,0x72,0x03,0x04,0x00,0x00,0x80,0x24}, // ECO TopL
  {0x28,0xD4,0xE7,0x03,0x00,0x00,0x80,0x89}, // ECO MidH
  {0x28,0x78,0xF9,0x03,0x00,0x00,0x80,0x76}, // ECO MidL
  {0x28,0x70,0xAD,0x07,0x00,0x00,0x80,0x53}, // ECO BotH
  {0x28,0x40,0xE1,0x03,0x00,0x00,0x80,0x78}  // ECO BotL
};

float calculateQtot(float temps[6]) {
  float avg_top = (temps[0] + temps[1]) / 2.0;
  float avg_mid = (temps[2] + temps[3]) / 2.0;
  float avg_bot = (temps[4] + temps[5]) / 2.0;
  float total = avg_top + avg_mid + avg_bot;
  return total * 0.1; // placeholder – later exacte formule
}

void readBoilerTemps() {
  ds18b20.requestTemperatures();
  for (int i = 0; i < 12; i++) {
    float temp = ds18b20.getTempC(sensor_addresses[i]);
    if (temp == DEVICE_DISCONNECTED_C) temp = -127.0;
    if (i < 6) sch_temps[i] = temp;
    else eco_temps[i-6] = temp;
  }
  sch_qtot = calculateQtot(sch_temps);
  eco_qtot = calculateQtot(eco_temps);
}

void pollRooms() {
  if (millis() - last_poll < (unsigned long)poll_interval * 1000) return;
  last_poll = millis();

  vent_percent = 0;

  for (int i = 0; i < 10; i++) {
    if (room_ips[i].length() == 0) continue;

    HTTPClient http;
    String url = "http://" + room_ips[i] + "/json";
    http.begin(url);
    http.setTimeout(5000);
    int httpCode = http.GET();

    if (httpCode == 200) {
      String payload = http.getString();
      DynamicJsonDocument doc(2048);
      DeserializationError error = deserializeJson(doc, payload);
      if (error) continue;

      bool heating = doc["y"] | false;      // heating_on
      int vent = doc["z"] | 0;              // vent_percent

      if (i < circuits_num) {
        bool new_state = heating;
        if (new_state != circuit_on[i]) {
          mcp.digitalWrite(i, new_state ? LOW : HIGH); // LOW = relay AAN
          if (new_state) {
            circuit_off_time[i] += millis() - circuit_last_change[i];
          } else {
            circuit_on_time[i] += millis() - circuit_last_change[i];
          }
          circuit_last_change[i] = millis();
          circuit_on[i] = new_state;
        }
      }

      if (vent > vent_percent) vent_percent = vent;
    }
    http.end();
  }

  int pwm_value = map(vent_percent, 0, 100, 0, 255);
  analogWrite(VENT_FAN_PIN, pwm_value);

  // Duty-cycle update
  for (int i = 0; i < circuits_num; i++) {
    unsigned long total = circuit_on_time[i] + circuit_off_time[i];
    if (total > 0) {
      circuit_dc[i] = 100.0 * circuit_on_time[i] / total;
    }
  }
}

void ecoPumpLogic() {
  static bool pumping = false;
  static unsigned long pump_start = 0;

  bool demand = false;
  for (int i = 0; i < circuits_num; i++) if (circuit_on[i]) { demand = true; break; }

  if (!demand) {
    pumping = false;
    mcp.digitalWrite(RELAY_PUMP_SCH, HIGH);
    mcp.digitalWrite(RELAY_PUMP_WON, HIGH);
    return;
  }

  if (eco_qtot > eco_threshold && !pumping) {
    pumping = true;
    pump_start = millis();
    mcp.digitalWrite(RELAY_PUMP_SCH, LOW);
  }

  if (pumping && (eco_qtot < eco_threshold - eco_hysteresis || millis() - pump_start > 300000)) {
    pumping = false;
    mcp.digitalWrite(RELAY_PUMP_SCH, HIGH);
  }
}

String getMainPage() {
  String html = "<!DOCTYPE html><html><head><title>HVAC Kelder</title><meta charset='utf-8'>"
                "<meta name='viewport' content='width=device-width, initial-scale=1'></head><body style='font-family:Arial'>";
  html += "<h1>HVAC Centrale Controller - " + room_id + "</h1>";
  html += "<h2>Boiler status</h2>SCH Qtot: " + String(sch_qtot, 2) + " kWh<br>"
          "ECO Qtot: " + String(eco_qtot, 2) + " kWh<br><br>";
  html += "<h2>Ventilatie</h2>Max request: " + String(vent_percent) + " %<br><br>";
  html += "<h2>Verwarmingscircuits</h2><table border='1' cellpadding='5'><tr>"
          "<th>#</th><th>Naam</th><th>State</th><th>Duty-cycle</th></tr>";
  for (int i = 0; i < circuits_num; i++) {
    html += "<tr><td>" + String(i+1) + "</td><td>" + circuit_nicknames[i] + "</td>"
            "<td>" + String(circuit_on[i] ? "AAN" : "UIT") + "</td>"
            "<td>" + String(circuit_dc[i], 1) + " %</td></tr>";
  }
  html += "</table><br><a href='/settings'>Instellingen</a> | "
          "<a href='/logdata'>Log JSON</a></body></html>";
  return html;
}

String getLogData() {
  DynamicJsonDocument doc(4096);
  doc["timestamp"] = millis() / 1000;
  doc["eco_qtot"] = eco_qtot;
  doc["sch_qtot"] = sch_qtot;
  doc["vent_percent"] = vent_percent;
  JsonArray circuits = doc.createNestedArray("circuits");
  for (int i = 0; i < circuits_num; i++) {
    JsonObject c = circuits.createNestedObject();
    c["id"] = i+1;
    c["name"] = circuit_nicknames[i];
    c["on"] = circuit_on[i];
    c["dc"] = circuit_dc[i];
  }
  String json;
  serializeJson(doc, json);
  return json;
}

void setupWebServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", getMainPage());
  });

  server.on("/json", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "application/json", getLogData());
  });

  server.on("/logdata", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "application/json", getLogData());
  });

  // Settings pagina (volledig werkend)
  server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = "<!DOCTYPE html><html><head><title>HVAC Settings</title></head><body><h1>Instellingen</h1><form action='/save_settings' method='POST'>";
    html += "Naam: <input name='room_id' value='" + room_id + "'><br><br>";
    html += "Aantal circuits (1-16): <input name='circuits_num' type='number' min='1' max='16' value='" + String(circuits_num) + "'><br><br>";
    html += "<h3>Circuit namen</h3>";
    for (int i = 0; i < 16; i++) {
      html += "Circuit " + String(i+1) + ": <input name='circuit_nick_" + String(i) + "' value='" + circuit_nicknames[i] + "'><br>";
    }
    html += "<h3>Room controllers (IP of mDNS naam)</h3>";
    for (int i = 0; i < 10; i++) {
      html += "Room " + String(i+1) + " IP/mDNS: <input name='room_ip_" + String(i) + "' value='" + room_ips[i] + "'> "
              "Naam: <input name='room_name_" + String(i) + "' value='" + room_names[i] + "'><br>";
    }
    html += "<br>ECO threshold (kWh): <input name='eco_thresh' type='number' step='0.1' value='" + String(eco_threshold) + "'><br>";
    html += "ECO hysteresis (kWh): <input name='eco_hyst' type='number' step='0.1' value='" + String(eco_hysteresis) + "'><br>";
    html += "Poll interval (sec): <input name='poll_interval' type='number' min='5' value='" + String(poll_interval) + "'><br><br>";
    html += "<input type='submit' value='Opslaan & Reboot'></form><br><a href='/'>Terug</a></body></html>";
    request->send(200, "text/html", html);
  });

  server.on("/save_settings", HTTP_POST, [](AsyncWebServerRequest *request){
    // Basis
    if (request->hasParam("room_id", true)) {
      room_id = request->getParam("room_id", true)->value();
      preferences.putString(NVS_ROOM_ID, room_id);
    }
    if (request->hasParam("circuits_num", true)) {
      circuits_num = request->getParam("circuits_num", true)->value().toInt();
      circuits_num = constrain(circuits_num, 1, 16);
      preferences.putInt(NVS_CIRCUITS_NUM, circuits_num);
    }
    // Circuit nicknames
    for (int i = 0; i < 16; i++) {
      String param = "circuit_nick_" + String(i);
      if (request->hasParam(param.c_str(), true)) {
        String nick = request->getParam(param.c_str(), true)->value();
        nick.trim();
        if (nick.length() == 0) nick = "Circuit " + String(i+1);
        circuit_nicknames[i] = nick;
        preferences.putString((String(NVS_CIRCUIT_NICK_BASE) + i).c_str(), nick);
      }
    }
    // Rooms
    for (int i = 0; i < 10; i++) {
      String ip_param = "room_ip_" + String(i);
      String name_param = "room_name_" + String(i);
      if (request->hasParam(ip_param.c_str(), true)) {
        room_ips[i] = request->getParam(ip_param.c_str(), true)->value();
        preferences.putString((String(NVS_ROOM_IP_BASE) + i).c_str(), room_ips[i]);
      }
      if (request->hasParam(name_param.c_str(), true)) {
        room_names[i] = request->getParam(name_param.c_str(), true)->value();
        preferences.putString((String(NVS_ROOM_NAME_BASE) + i).c_str(), room_names[i]);
      }
    }
    // ECO
    if (request->hasParam("eco_thresh", true)) {
      eco_threshold = request->getParam("eco_thresh", true)->value().toFloat();
      preferences.putFloat(NVS_ECO_THRESHOLD, eco_threshold);
    }
    if (request->hasParam("eco_hyst", true)) {
      eco_hysteresis = request->getParam("eco_hyst", true)->value().toFloat();
      preferences.putFloat(NVS_ECO_HYSTERESIS, eco_hysteresis);
    }
    if (request->hasParam("poll_interval", true)) {
      poll_interval = request->getParam("poll_interval", true)->value().toInt();
      if (poll_interval < 5) poll_interval = 5;
      preferences.putInt(NVS_POLL_INTERVAL, poll_interval);
    }

    request->send(200, "text/html", "<h2>Instellingen opgeslagen! Rebooting...</h2>");
    delay(1000);
    ESP.restart();
  });
}




void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== HVAC Controller boot ===");

  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("Wire.begin() uitgevoerd");

  // MCP23017 initialiseren (werkt ook als de chip nog niet aangesloten is)
  for (int i = 0; i < 16; i++) {
    mcp.pinMode(i, OUTPUT);
    mcp.digitalWrite(i, HIGH);  // alle relays uit
  }
  Serial.println("MCP23017 geïnitialiseerd (of gesimuleerd bij test zonder hardware)");

  preferences.begin("hvac-config", false);

  // Laden instellingen uit NVS
  room_id = preferences.getString(NVS_ROOM_ID, "HVAC");
  wifi_ssid = preferences.getString(NVS_WIFI_SSID, "");
  wifi_pass = preferences.getString(NVS_WIFI_PASS, "");
  static_ip_str = preferences.getString(NVS_STATIC_IP, "");

  circuits_num = preferences.getInt(NVS_CIRCUITS_NUM, 7);
  circuits_num = constrain(circuits_num, 1, 16);

  for (int i = 0; i < 16; i++) {
    String key = String(NVS_CIRCUIT_NICK_BASE) + i;
    circuit_nicknames[i] = preferences.getString(key.c_str(), "Circuit " + String(i+1));
  }

  for (int i = 0; i < 10; i++) {
    room_ips[i] = preferences.getString((String(NVS_ROOM_IP_BASE) + i).c_str(), "");
    room_names[i] = preferences.getString((String(NVS_ROOM_NAME_BASE) + i).c_str(), "Room " + String(i+1));
  }

  eco_threshold = preferences.getFloat(NVS_ECO_THRESHOLD, 12.0);
  eco_hysteresis = preferences.getFloat(NVS_ECO_HYSTERESIS, 2.0);
  poll_interval = preferences.getInt(NVS_POLL_INTERVAL, 20);

  // OneWire initialiseren
  ds18b20.begin();
  uint8_t deviceCount = ds18b20.getDeviceCount();
  if (deviceCount == 0) {
    Serial.println("Geen DS18B20 sensoren gevonden (normaal bij test zonder hardware)");
  } else {
    Serial.println(String(deviceCount) + " DS18B20 sensoren gevonden");
  }

  // WiFi connectie
  WiFi.mode(WIFI_STA);

  if (static_ip_str.length() > 0 && static_ip.fromString(static_ip_str)) {
    IPAddress gateway(192, 168, 1, 1);     // pas aan als jouw router een ander gateway heeft
    IPAddress subnet(255, 255, 255, 0);
    IPAddress dns(192, 168, 1, 1);
    WiFi.config(static_ip, gateway, subnet, dns);
  }

  WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str());

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi connectie mislukt → AP mode");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("HVAC-Setup");
    ap_mode_active = true;
    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
  } else {
    Serial.println("\nWiFi verbonden: " + WiFi.localIP().toString());
  }

  if (MDNS.begin(room_id.c_str())) {
    Serial.println("mDNS gestart: http://" + room_id + ".local");
  }

  setupWebServer();
  server.begin();
  Serial.println("Webserver gestart");
}





void loop() {

  if (ap_mode_active) dnsServer.processNextRequest();

  // WiFi reconnect check
  if (!ap_mode_active && WiFi.status() != WL_CONNECTED && millis() - last_wifi_check > 30000) {
    Serial.println("WiFi verloren → reconnect");
    WiFi.reconnect();
    last_wifi_check = millis();
  }

  readBoilerTemps();
  pollRooms();
  ecoPumpLogic();

  delay(100);
}
