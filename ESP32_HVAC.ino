/* ESP32C6_HVACTEST.ino = Centrale HVAC controller voor kelder (ESP32-C6) op basis van particle sketch voor Flobecq
Transition from Photon based to ESP32 based Home automation system. Developed together with ChatGPT & Grok in januari '26.
Thuis bereikbaar op http://hvactest.local of http://192.168.1.36 => Andere controller: Naam (sectie DNS/MDNS) + static IP aanpassen!
07jan26 15:00 Version 4 - Circuit-centric met thermostaat inputs

Component       Wijziging
Data model      Circuit struct met alle properties
MCP pins        7=feedback, 10-12=thermostaten
Polling         Prioriteit: 1) Tstat, 2) HTTP
Settings        Circuit boxen met alle velden
Homepage        7 kolommen + totalen + status
NVS             6 keys per circuit (x16)

*/


// DEEL 1: Headers, Structs & Globals

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

// Circuit struct
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
};

// Runtime vars
String room_id = "HVAC";
String wifi_ssid = "";
String wifi_pass = "";
String static_ip_str = "";
IPAddress static_ip;
int circuits_num = 7;
Circuit circuits[16];
String sensor_nicknames[6];
float eco_threshold = 12.0;
float eco_hysteresis = 2.0;
int poll_interval = 20;

#define RELAY_PUMP_SCH 8
#define RELAY_PUMP_WON 9

int vent_percent = 0;
float sch_temps[6] = {-127,-127,-127,-127,-127,-127};
float eco_temps[6] = {-127,-127,-127,-127,-127,-127};
bool sensor_ok[6] = {false};
float sch_qtot = 0.0;
float eco_qtot = 0.0;

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


// DEEL 2: Helper Functies

String getFormattedDateTime() {
  time_t now; time(&now);
  if (now < 1700000000) return "tijd niet gesync";
  struct tm tm; localtime_r(&now, &tm);
  char buf[32]; strftime(buf, sizeof(buf), "%d-%m-%Y %H:%M:%S", &tm);
  return String(buf);
}

float calculateQtot(float temps[6]) {
  return ((temps[0]+temps[1])/2.0 + (temps[2]+temps[3])/2.0 + (temps[4]+temps[5])/2.0) * 0.1;
}

void readBoilerTemps() {
  if (millis() - last_temp_read < 2000) return;
  last_temp_read = millis();
  for (int i = 0; i < 6; i++) {
    ow.reset(); ow.writeByte(0x55);
    for (int j = 0; j < 8; j++) ow.writeByte(sensor_addresses[i][j]);
    ow.writeByte(0x44); delay(750);
    ow.reset(); ow.writeByte(0x55);
    for (int j = 0; j < 8; j++) ow.writeByte(sensor_addresses[i][j]);
    ow.writeByte(0xBE);
    uint8_t data[9];
    for (int j = 0; j < 9; j++) data[j] = ow.readByte();
    uint8_t crc = 0;
    for (int j = 0; j < 8; j++) {
      uint8_t inbyte = data[j];
      for (int k = 0; k < 8; k++) {
        uint8_t mix = (crc ^ inbyte) & 0x01;
        crc >>= 1; if (mix) crc ^= 0x8C; inbyte >>= 1;
      }
    }
    if (crc == data[8]) {
      int16_t raw = (data[1] << 8) | data[0];
      sch_temps[i] = raw / 16.0; sensor_ok[i] = true;
    } else {
      sch_temps[i] = -127.0; sensor_ok[i] = false;
    }
    eco_temps[i] = sch_temps[i];
  }
  sch_qtot = calculateQtot(sch_temps);
  eco_qtot = calculateQtot(eco_temps);
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

void pollRooms() {
  if (millis() - last_poll < (unsigned long)poll_interval * 1000) return;
  last_poll = millis();
  vent_percent = 0;
  float total_power = 0.0;

  for (int i = 0; i < circuits_num; i++) {
    bool heating_demand = false;
    
    // PRIORITEIT 1: Hardwired thermostaat
    if (circuits[i].has_tstat && mcp_available && circuits[i].tstat_pin < 13) {
      bool tstat_on = (mcp.digitalRead(circuits[i].tstat_pin) == LOW);
      if (tstat_on) heating_demand = true;
    }
    
    // PRIORITEIT 2: HTTP poll
    if (circuits[i].ip.length() > 0 || circuits[i].mdns.length() > 0) {
      HTTPClient http;
      String url = circuits[i].ip.length() > 0 
        ? "http://" + circuits[i].ip + "/json"
        : "http://" + circuits[i].mdns + ".local/json";
      http.begin(url);
      http.setTimeout(5000);
      if (http.GET() == 200) {
        circuits[i].online = true;
        circuits[i].last_seen = millis();
        DynamicJsonDocument doc(2048);
        deserializeJson(doc, http.getString());
        bool room_heating = doc["y"] | false;
        int room_vent = doc["z"] | 0;
        if (room_heating) heating_demand = true;
        if (room_vent > vent_percent) vent_percent = room_vent;
        circuits[i].vent_request = room_vent;
      } else {
        circuits[i].online = false;
        circuits[i].vent_request = 0;
      }
      http.end();
    }
    
    // Relay schakelen + duty-cycle
    if (heating_demand != circuits[i].heating_on) {
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
    
    if (circuits[i].heating_on) {
      total_power += circuits[i].power_kw;
    }
  }
  
  analogWrite(VENT_FAN_PIN, map(vent_percent, 0, 100, 0, 255));
  checkPumpFeedback(total_power);
}

void ecoPumpLogic() {
  static bool pumping = false;
  static unsigned long pump_start = 0;
  bool demand = false;
  for (int i = 0; i < circuits_num; i++) if (circuits[i].heating_on) { demand = true; break; }
  if (!demand) {
    pumping = false;
    if (mcp_available) {
      mcp.digitalWrite(RELAY_PUMP_SCH, HIGH);
      mcp.digitalWrite(RELAY_PUMP_WON, HIGH);
    }
    return;
  }
  if (eco_qtot > eco_threshold && !pumping) {
    pumping = true; pump_start = millis();
    if (mcp_available) mcp.digitalWrite(RELAY_PUMP_SCH, LOW);
  }
  if (pumping && (eco_qtot < eco_threshold - eco_hysteresis || millis() - pump_start > 300000)) {
    pumping = false;
    if (mcp_available) mcp.digitalWrite(RELAY_PUMP_SCH, HIGH);
  }
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

String getLogData() {
  DynamicJsonDocument doc(4096);
  doc["timestamp"] = millis() / 1000;
  doc["eco_qtot"] = eco_qtot;
  doc["sch_qtot"] = sch_qtot;
  doc["vent_percent"] = vent_percent;
  JsonArray temps = doc.createNestedArray("temperatures");
  for (int i = 0; i < 6; i++) {
    JsonObject t = temps.createNestedObject();
    t["name"] = sensor_nicknames[i];
    t["temp"] = sch_temps[i];
    t["ok"] = sensor_ok[i];
  }
  JsonArray circs = doc.createNestedArray("circuits");
  for (int i = 0; i < circuits_num; i++) {
    JsonObject c = circs.createNestedObject();
    c["id"] = i + 1;
    c["name"] = circuits[i].name;
    c["on"] = circuits[i].heating_on;
    c["power"] = circuits[i].power_kw;
    c["dc"] = circuits[i].duty_cycle;
    c["online"] = circuits[i].online;
  }
  String json;
  serializeJson(doc, json);
  return json;
}


// DEEL 3 - 1) Web Server functies)

String getMainPage() {
  float total_power = 0.0;
  for (int i = 0; i < circuits_num; i++) {
    if (circuits[i].heating_on) total_power += circuits[i].power_kw;
  }
  
  String html;
  html.reserve(14000);
  html = R"rawliteral(
<!DOCTYPE html>
<html lang="nl">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta http-equiv="refresh" content="5">
  <title>)rawliteral" + room_id + R"rawliteral( Status</title>
  <style>
    body {font-family:Arial,Helvetica,sans-serif;background:#ffffff;margin:0;padding:0;}
    .header {display:flex;background:#ffcc00;color:black;padding:10px 15px;font-size:18px;font-weight:bold;align-items:center;}
    .header-left {flex:1;text-align:left;}
    .header-right {flex:1;text-align:right;font-size:15px;}
    .container {display:flex;flex-direction:row;min-height:calc(100vh - 60px);}
    .sidebar {width:80px;padding:10px 5px;background:#ffffff;border-right:3px solid #cc0000;flex-shrink:0;}
    .sidebar a {display:block;background:#336699;color:white;padding:8px;margin:8px 0;text-decoration:none;font-weight:bold;font-size:12px;border-radius:6px;text-align:center;line-height:1.3;width:60px;margin-left:auto;margin-right:auto;}
    .sidebar a:hover {background:#003366;}
    .sidebar a.active {background:#cc0000;}
    .main {flex:1;padding:15px;overflow-y:auto;}
    .group-title {font-size:17px;font-style:italic;font-weight:bold;color:#336699;margin:20px 0 8px 0;}
    table {width:100%;border-collapse:collapse;margin-bottom:15px;}
    td.label {color:#336699;font-size:13px;padding:8px 5px;border-bottom:1px solid #ddd;text-align:left;vertical-align:middle;}
    td.value {background:#e6f0ff;font-size:13px;padding:8px 5px;border-bottom:1px solid #ddd;text-align:center;vertical-align:middle;}
    tr.header-row {background:#336699;color:white;}
    tr.header-row td {color:white;background:#336699;}
    .status-ok {color:#00aa00;font-weight:bold;}
    .status-na {color:#cc0000;font-weight:bold;}
    @media (max-width: 600px) {
      .container {flex-direction:column;}
      .sidebar {width:100%;border-right:none;border-bottom:3px solid #cc0000;padding:10px 0;display:flex;justify-content:center;}
      .sidebar a {width:70px;margin:0 5px;}
      .main {padding:10px;}
      .group-title {font-size:16px;margin:15px 0 6px 0;}
      td.label {font-size:12px;padding:6px 4px;}
      td.value {font-size:12px;padding:6px 4px;}
    }
  </style>
</head>
<body>
  <div class="header">
    <div class="header-left">)rawliteral" + room_id + R"rawliteral(</div>
    <div class="header-right">)rawliteral" + String(uptime_sec) + " s &nbsp;&nbsp; " + getFormattedDateTime() + R"rawliteral(</div>
  </div>
  <div class="container">
    <div class="sidebar">
      <a href="/" class="active">Status</a>
      <a href="/update">OTA</a>
      <a href="/json">JSON</a>
      <a href="/settings">Settings</a>
    </div>
    <div class="main">
      <div class="group-title">CONTROLLER STATUS</div>
      <table>
        <tr><td class="label">MCP23017</td><td class="value">)rawliteral" + String(mcp_available ? "Verbonden" : "Niet gevonden") + R"rawliteral(</td></tr>
        <tr><td class="label">WiFi</td><td class="value">)rawliteral" + String(WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "Niet verbonden") + R"rawliteral(</td></tr>
        <tr><td class="label">WiFi RSSI</td><td class="value">)rawliteral" + String(WiFi.RSSI()) + " dBm" + R"rawliteral(</td></tr>
        <tr><td class="label">Free heap</td><td class="value">)rawliteral" + String((ESP.getFreeHeap() * 100) / ESP.getHeapSize()) + " %" + R"rawliteral(</td></tr>
      </table>

      <div class="group-title">BOILER TEMPERATUREN</div>
      <table>
        <tr class="header-row"><td class="label">Sensor</td><td class="value">Temperatuur</td><td class="value">Status</td></tr>
)rawliteral";
  
  for (int i = 0; i < 6; i++) {
    String status = sensor_ok[i] ? "OK" : "Error";
    String temp_str = sensor_ok[i] ? String(sch_temps[i], 1) + " °C" : "--";
    html += "<tr><td class=\"label\">" + sensor_nicknames[i] + "</td><td class=\"value\">" + temp_str + "</td><td class=\"value\">" + status + "</td></tr>";
  }
  
  html += R"rawliteral(
      </table>
      <table>
        <tr><td class="label">SCH Qtot</td><td class="value">)rawliteral" + String(sch_qtot, 2) + " kWh" + R"rawliteral(</td></tr>
        <tr><td class="label">ECO Qtot</td><td class="value">)rawliteral" + String(eco_qtot, 2) + " kWh" + R"rawliteral(</td></tr>
      </table>

      <div class="group-title">VENTILATIE</div>
      <table>
        <tr><td class="label">Max request</td><td class="value">)rawliteral" + String(vent_percent) + " %" + R"rawliteral(</td></tr>
      </table>

      <div class="group-title">VERWARMINGSCIRCUITS</div>
      <table>
        <tr class="header-row"><td class="label">#</td><td class="value">Naam</td><td class="value">Status</td><td class="value">Pomp</td><td class="value">Vermogen</td><td class="value">Vent %</td><td class="value">Duty %</td></tr>
)rawliteral";
  
  for (int i = 0; i < circuits_num; i++) {
    String status_icon = circuits[i].online ? "✓ OK" : "✗ NA!";
    String status_class = circuits[i].online ? "status-ok" : "status-na";
    String pump = circuits[i].heating_on ? "AAN" : "UIT";
    String power = circuits[i].heating_on ? String(circuits[i].power_kw, 1) + " kW" : "0 kW";
    
    html += "<tr><td class=\"label\">" + String(i + 1) + "</td>";
    html += "<td class=\"value\">" + circuits[i].name + "</td>";
    html += "<td class=\"value " + status_class + "\">" + status_icon + "</td>";
    html += "<td class=\"value\">" + pump + "</td>";
    html += "<td class=\"value\">" + power + "</td>";
    html += "<td class=\"value\">" + String(circuits[i].vent_request) + " %</td>";
    html += "<td class=\"value\">" + String(circuits[i].duty_cycle, 1) + " %</td></tr>";
  }
  
  html += R"rawliteral(
        <tr style="border-top:2px solid #336699;"><td colspan="4" class="label"><b>TOTAAL</b></td>
        <td class="value"><b>)rawliteral" + String(total_power, 1) + " kW" + R"rawliteral(</b></td>
        <td class="value"><b>)rawliteral" + String(vent_percent) + " %" + R"rawliteral(</b></td>
        <td class="value"></td></tr>
      </table>
      <p style="text-align:center;margin-top:30px;"><a href="/settings" style="color:#336699;text-decoration:underline;font-size:16px;">→ Instellingen</a></p>
    </div>
  </div>
</body>
</html>
)rawliteral";
  return html;
}

void setupWebServer() {
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
<html lang="nl">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>)rawliteral" + room_id + R"rawliteral( - OTA</title>
  <style>
    body {font-family:Arial,Helvetica,sans-serif;background:#ffffff;margin:0;padding:0;}
    .header {display:flex;background:#ffcc00;color:black;padding:10px 15px;font-size:18px;font-weight:bold;align-items:center;}
    .header-left {flex:1;text-align:left;}
    .header-right {flex:1;text-align:right;font-size:15px;}
    .container {display:flex;min-height:calc(100vh - 60px);}
    .sidebar {width:80px;padding:10px 5px;background:#ffffff;border-right:3px solid #cc0000;}
    .sidebar a {display:block;background:#336699;color:white;padding:8px;margin:8px 0;text-decoration:none;font-weight:bold;font-size:12px;border-radius:6px;text-align:center;line-height:1.3;width:60px;margin-left:auto;margin-right:auto;}
    .sidebar a:hover {background:#003366;}
    .sidebar a.active {background:#cc0000;}
    .main {flex:1;padding:40px;text-align:center;}
    .button {background:#336699;color:white;padding:12px 24px;border:none;border-radius:8px;cursor:pointer;font-size:16px;margin:10px;}
    .button:hover {background:#003366;}
    .reboot {background:#cc0000;}
    .reboot:hover {background:#990000;}
    @media (max-width: 600px) {
      .container {flex-direction:column;}
      .sidebar {width:100%;border-right:none;border-bottom:3px solid #cc0000;padding:10px 0;display:flex;justify-content:center;}
      .sidebar a {width:70px;margin:0 5px;}
      .main {padding:20px;}
    }
  </style>
</head>
<body>
  <div class="header">
    <div class="header-left">)rawliteral" + room_id + R"rawliteral(</div>
    <div class="header-right">)rawliteral" + String(uptime_sec) + " s &nbsp;&nbsp; " + getFormattedDateTime() + R"rawliteral(</div>
  </div>
  <div class="container">
    <div class="sidebar">
      <a href="/">Status</a>
      <a href="/update" class="active">OTA</a>
      <a href="/json">JSON</a>
      <a href="/settings">Settings</a>
    </div>
    <div class="main">
      <h1 style="color:#336699;">OTA Firmware Update</h1>
      <p>Selecteer een .bin bestand</p>
      <form method="POST" action="/update" enctype="multipart/form-data">
        <input type="file" name="update" accept=".bin"><br><br>
        <button class="button" type="submit">Upload Firmware</button>
      </form>
      <br>
      <button class="button reboot" onclick="if(confirm('ESP32 rebooten?')) location.href='/reboot'">Reboot</button>
      <br><br><a href="/" style="color:#336699;text-decoration:underline;">← Terug</a>
    </div>
  </div>
</body>
</html>
)rawliteral";
    request->send(200, "text/html; charset=utf-8", html);
  });

  server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request) {
    bool success = !Update.hasError();
    request->send(200, "text/html", success 
      ? "<h2 style='color:#0f0'>Update succesvol!</h2><p>Rebooting...</p>" 
      : "<h2 style='color:#f00'>Update mislukt!</h2>");
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


// DEEL 3 - 2) Settings page

server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *request){
    String sensorNamesHtml = "";
    for (int i = 0; i < 6; i++) {
      sensorNamesHtml += "<label style=\"display:block;margin:6px 0;\">Sensor " + String(i + 1) + ": ";
      sensorNamesHtml += "<input type=\"text\" name=\"sensor_nick_" + String(i) + "\" value=\"" + sensor_nicknames[i] + "\" style=\"width:220px;\"></label>";
    }
    
    String circuitsHtml = "";
    for (int i = 0; i < circuits_num; i++) {
      circuitsHtml += "<div class=\"circuit-box\">";
      circuitsHtml += "<h4>Circuit " + String(i + 1) + "</h4>";
      circuitsHtml += "<table class=\"form-table\">";
      circuitsHtml += "<tr><td class=\"label\">Naam</td><td class=\"input\"><input type=\"text\" name=\"circuit_name_" + String(i) + "\" value=\"" + circuits[i].name + "\"></td></tr>";
      circuitsHtml += "<tr><td class=\"label\">IP adres</td><td class=\"input\"><input type=\"text\" name=\"circuit_ip_" + String(i) + "\" value=\"" + circuits[i].ip + "\" placeholder=\"192.168.1.50\"></td></tr>";
      circuitsHtml += "<tr><td class=\"label\">mDNS naam</td><td class=\"input\"><input type=\"text\" name=\"circuit_mdns_" + String(i) + "\" value=\"" + circuits[i].mdns + "\" placeholder=\"keuken\"></td></tr>";
      circuitsHtml += "<tr><td class=\"label\">Vermogen (kW)</td><td class=\"input\"><input type=\"number\" step=\"0.001\" name=\"circuit_power_" + String(i) + "\" value=\"" + String(circuits[i].power_kw, 3) + "\"></td></tr>";
      circuitsHtml += "<tr><td class=\"label\">Hardwired thermostaat</td><td class=\"input\">";
      circuitsHtml += "<input type=\"checkbox\" name=\"circuit_tstat_" + String(i) + "\" value=\"1\"" + String(circuits[i].has_tstat ? " checked" : "") + "> ";
      circuitsHtml += "Pin: <select name=\"circuit_tstat_pin_" + String(i) + "\">";
      circuitsHtml += "<option value=\"255\"" + String(circuits[i].tstat_pin == 255 ? " selected" : "") + ">Geen</option>";
      circuitsHtml += "<option value=\"10\"" + String(circuits[i].tstat_pin == 10 ? " selected" : "") + ">Pin 10 (ZP)</option>";
      circuitsHtml += "<option value=\"11\"" + String(circuits[i].tstat_pin == 11 ? " selected" : "") + ">Pin 11 (EP)</option>";
      circuitsHtml += "<option value=\"12\"" + String(circuits[i].tstat_pin == 12 ? " selected" : "") + ">Pin 12 (KK)</option>";
      circuitsHtml += "</select></td></tr>";
      circuitsHtml += "</table></div>";
    }

    String html;
    html.reserve(20000);
    html = R"rawliteral(
<!DOCTYPE html>
<html lang="nl">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>)rawliteral" + room_id + R"rawliteral( - Instellingen</title>
  <style>
    body {font-family:Arial,Helvetica,sans-serif;background:#ffffff;margin:0;padding:0;}
    .header {display:flex;background:#ffcc00;color:black;padding:10px 15px;font-size:18px;font-weight:bold;align-items:center;}
    .header-left {flex:1;text-align:left;}
    .header-right {flex:1;text-align:right;font-size:15px;}
    .container {display:flex;flex-direction:row;min-height:calc(100vh - 60px);}
    .sidebar {width:80px;padding:10px 5px;background:#ffffff;border-right:3px solid #cc0000;flex-shrink:0;}
    .sidebar a {display:block;background:#336699;color:white;padding:8px;margin:8px 0;text-decoration:none;font-weight:bold;font-size:12px;border-radius:6px;text-align:center;line-height:1.3;width:60px;margin-left:auto;margin-right:auto;}
    .sidebar a:hover {background:#003366;}
    .sidebar a.active {background:#cc0000;}
    .main {flex:1;padding:20px;overflow-y:auto;}
    .warning {background:#ffe6e6;border:2px solid #cc0000;padding:15px;margin:20px 0;border-radius:8px;text-align:center;font-weight:bold;color:#990000;}
    .section-title {font-size:18px;font-weight:bold;color:#336699;margin:25px 0 10px 0;padding-bottom:5px;border-bottom:2px solid #336699;}
    .form-table {width:100%;border-collapse:collapse;margin:15px 0;}
    .form-table td.label {width:35%;padding:12px 8px;vertical-align:middle;font-weight:bold;color:#336699;}
    .form-table td.input {width:65%;padding:12px 8px;vertical-align:middle;}
    .form-table input[type=text], .form-table input[type=password], .form-table input[type=number], .form-table select {width:100%;padding:8px;border:1px solid #ccc;border-radius:4px;box-sizing:border-box;}
    .form-table tr {border-bottom:1px solid #eee;}
    .circuit-box {background:#f5f5f5;border:1px solid #ccc;border-radius:8px;padding:15px;margin:15px 0;}
    .circuit-box h4 {margin:0 0 10px 0;color:#336699;}
    .submit-btn {background:#336699;color:white;padding:12px 30px;border:none;border-radius:6px;font-size:16px;cursor:pointer;margin:20px 10px;}
    .submit-btn:hover {background:#003366;}
    .scan-btn {background:#336699;color:white;padding:8px 20px;border:none;border-radius:6px;font-size:14px;cursor:pointer;margin:10px 0;}
    .scan-btn:hover {background:#003366;}
    @media (max-width: 800px) {
      .container {flex-direction:column;}
      .sidebar {width:100%;border-right:none;border-bottom:3px solid #cc0000;padding:10px 0;display:flex;justify-content:center;}
      .sidebar a {width:70px;margin:0 5px;}
      .form-table td.label, .form-table td.input {width:50%;}
    }
  </style>
</head>
<body>
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
      <div class="warning">
        OPGEPAST: Wijzigt permanente instellingen!<br>
        Verkeerde WiFi kan controller onbereikbaar maken!<br><br>
        <strong>Geen WiFi?</strong> Controller start AP: HVAC-Setup<br>
        Ga naar http://192.168.4.1/settings
      </div>

      <form action="/save_settings" method="get">
        
        <div class="section-title">WiFi Configuratie</div>
        <button type="button" class="scan-btn" onclick="scanWifi()">Scan netwerken</button>
        <table class="form-table">
          <tr>
            <td class="label">WiFi SSID</td>
            <td class="input">
              <select id="ssid_select" name="wifi_ssid" onchange="document.getElementById('ssid_manual').value=this.value">
                <option value="">Selecteer...</option>
              </select><br>
              Of handmatig: <input id="ssid_manual" type="text" name="wifi_ssid_manual" value=")rawliteral" + wifi_ssid + R"rawliteral(">
            </td>
          </tr>
          <tr>
            <td class="label">WiFi Password</td>
            <td class="input"><input type="password" name="wifi_pass" value=")rawliteral" + wifi_pass + R"rawliteral("></td>
          </tr>
          <tr>
            <td class="label">Static IP</td>
            <td class="input"><input type="text" name="static_ip" value=")rawliteral" + static_ip_str + R"rawliteral(" placeholder="leeg = DHCP"></td>
          </tr>
        </table>

        <div class="section-title">Basis Instellingen</div>
        <table class="form-table">
          <tr>
            <td class="label">Room naam</td>
            <td class="input"><input type="text" name="room_id" value=")rawliteral" + room_id + R"rawliteral(" required></td>
          </tr>
          <tr>
            <td class="label">Aantal circuits</td>
            <td class="input"><input type="number" name="circuits_num" min="1" max="16" value=")rawliteral" + String(circuits_num) + R"rawliteral("></td>
          </tr>
          <tr>
            <td class="label">ECO threshold (kWh)</td>
            <td class="input"><input type="number" step="0.1" name="eco_thresh" value=")rawliteral" + String(eco_threshold) + R"rawliteral("></td>
          </tr>
          <tr>
            <td class="label">ECO hysteresis (kWh)</td>
            <td class="input"><input type="number" step="0.1" name="eco_hyst" value=")rawliteral" + String(eco_hysteresis) + R"rawliteral("></td>
          </tr>
          <tr>
            <td class="label">Poll interval (sec)</td>
            <td class="input"><input type="number" min="5" name="poll_interval" value=")rawliteral" + String(poll_interval) + R"rawliteral("></td>
          </tr>
        </table>

        <div class="section-title">Sensor Nicknames</div>
        <div style="padding:10px;">)rawliteral" + sensorNamesHtml + R"rawliteral(</div>

        <div class="section-title">Verwarmingscircuits</div>
        )rawliteral" + circuitsHtml + R"rawliteral(

        <div style="text-align:center;">
          <button type="submit" class="submit-btn">Opslaan & Reboot</button>
        </div>
      </form>

      <script>
        function scanWifi() {
          fetch('/scan').then(r => r.json()).then(data => {
            let sel = document.getElementById('ssid_select');
            sel.innerHTML = '<option value="">Selecteer...</option>';
            data.networks.forEach(n => {
              sel.innerHTML += '<option value="' + n.ssid + '">' + n.ssid + ' (' + n.rssi + ' dBm)</option>';
            });
          });
        }
      </script>

    </div>
  </div>
</body>
</html>
)rawliteral";
    request->send(200, "text/html; charset=utf-8", html);
  });

  server.on("/save_settings", HTTP_GET, [](AsyncWebServerRequest *request){
    // WiFi
    String ssid_sel = request->arg("wifi_ssid");
    String ssid_manual = request->arg("wifi_ssid_manual");
    if (ssid_manual.length() > 0) preferences.putString(NVS_WIFI_SSID, ssid_manual);
    else if (ssid_sel.length() > 0) preferences.putString(NVS_WIFI_SSID, ssid_sel);
    if (request->hasArg("wifi_pass")) preferences.putString(NVS_WIFI_PASS, request->arg("wifi_pass"));
    preferences.putString(NVS_STATIC_IP, request->arg("static_ip"));
    
    // Basis
    preferences.putString(NVS_ROOM_ID, request->arg("room_id"));
    preferences.putInt(NVS_CIRCUITS_NUM, request->arg("circuits_num").toInt());
    preferences.putFloat(NVS_ECO_THRESHOLD, request->arg("eco_thresh").toFloat());
    preferences.putFloat(NVS_ECO_HYSTERESIS, request->arg("eco_hyst").toFloat());
    preferences.putInt(NVS_POLL_INTERVAL, request->arg("poll_interval").toInt());
    
    // Sensors
    for (int i = 0; i < 6; i++) {
      String param = "sensor_nick_" + String(i);
      if (request->hasArg(param.c_str())) {
        String nick = request->arg(param.c_str());
        nick.trim();
        if (nick.length() == 0) nick = "Sensor " + String(i + 1);
        preferences.putString((String(NVS_SENSOR_NICK_BASE) + i).c_str(), nick);
      }
    }
    
    // Circuits
    for (int i = 0; i < 16; i++) {
      String prefix = "circuit_";
      if (request->hasArg((prefix + "name_" + i).c_str())) {
        preferences.putString((prefix + "name_" + i).c_str(), request->arg((prefix + "name_" + i).c_str()));
      }
      if (request->hasArg((prefix + "ip_" + i).c_str())) {
        preferences.putString((prefix + "ip_" + i).c_str(), request->arg((prefix + "ip_" + i).c_str()));
      }
      if (request->hasArg((prefix + "mdns_" + i).c_str())) {
        preferences.putString((prefix + "mdns_" + i).c_str(), request->arg((prefix + "mdns_" + i).c_str()));
      }
      if (request->hasArg((prefix + "power_" + i).c_str())) {
        preferences.putFloat((prefix + "power_" + i).c_str(), request->arg((prefix + "power_" + i).c_str()).toFloat());
      }
      bool has_tstat = request->hasArg((prefix + "tstat_" + i).c_str());
      preferences.putBool((prefix + "tstat_" + i).c_str(), has_tstat);
      if (request->hasArg((prefix + "tstat_pin_" + i).c_str())) {
        preferences.putInt((prefix + "tstat_pin_" + i).c_str(), request->arg((prefix + "tstat_pin_" + i).c_str()).toInt());
      }
    }
    
    request->send(200, "text/html", "<h2 style='text-align:center;color:#336699;'>Opgeslagen! Rebooting...</h2>");
    delay(800);
    ESP.restart();
  });

  server.begin();
}


// DEEL 4: Setup & Loop


void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== HVAC Controller boot ===");

  Wire.begin(I2C_SDA, I2C_SCL);
  
  if (mcp.begin_I2C(0x20)) {
    Serial.println("MCP23017 gevonden!");
    mcp_available = true;
    
    // Pins 0-6: Outputs (relays circuits)
    for (int i = 0; i < 7; i++) {
      mcp.pinMode(i, OUTPUT);
      mcp.digitalWrite(i, HIGH);
    }
    
    // Pin 7: Input (pump feedback)
    mcp.pinMode(7, INPUT_PULLUP);
    
    // Pins 8-9: Outputs (ECO pompen)
    mcp.pinMode(8, OUTPUT);
    mcp.digitalWrite(8, HIGH);
    mcp.pinMode(9, OUTPUT);
    mcp.digitalWrite(9, HIGH);
    
    // Pins 10-12: Inputs (thermostaten)
    mcp.pinMode(10, INPUT_PULLUP);
    mcp.pinMode(11, INPUT_PULLUP);
    mcp.pinMode(12, INPUT_PULLUP);
    
    // Pins 13-15: Reserve
    mcp.pinMode(13, INPUT_PULLUP);
    mcp.pinMode(14, INPUT_PULLUP);
    mcp.pinMode(15, INPUT_PULLUP);
  } else {
    Serial.println("MCP23017 niet gevonden");
    mcp_available = false;
  }

  preferences.begin("hvac-config", false);

  // Laad basis settings
  room_id = preferences.getString(NVS_ROOM_ID, "HVAC");
  wifi_ssid = preferences.getString(NVS_WIFI_SSID, "");
  wifi_pass = preferences.getString(NVS_WIFI_PASS, "");
  static_ip_str = preferences.getString(NVS_STATIC_IP, "");
  circuits_num = preferences.getInt(NVS_CIRCUITS_NUM, 7);
  circuits_num = constrain(circuits_num, 1, 16);
  eco_threshold = preferences.getFloat(NVS_ECO_THRESHOLD, 12.0);
  eco_hysteresis = preferences.getFloat(NVS_ECO_HYSTERESIS, 2.0);
  poll_interval = preferences.getInt(NVS_POLL_INTERVAL, 20);

  // Laad sensor nicknames
  for (int i = 0; i < 6; i++) {
    sensor_nicknames[i] = preferences.getString(
      (String(NVS_SENSOR_NICK_BASE) + i).c_str(), 
      "Sensor " + String(i + 1)
    );
  }

  // Laad circuit data
  for (int i = 0; i < 16; i++) {
    String prefix = "circuit_";
    circuits[i].name = preferences.getString((prefix + "name_" + i).c_str(), "Circuit " + String(i + 1));
    circuits[i].ip = preferences.getString((prefix + "ip_" + i).c_str(), "");
    circuits[i].mdns = preferences.getString((prefix + "mdns_" + i).c_str(), "");
    circuits[i].power_kw = preferences.getFloat((prefix + "power_" + i).c_str(), 0.0);
    circuits[i].has_tstat = preferences.getBool((prefix + "tstat_" + i).c_str(), false);
    circuits[i].tstat_pin = preferences.getInt((prefix + "tstat_pin_" + i).c_str(), 255);
    
    // Runtime init
    circuits[i].online = false;
    circuits[i].heating_on = false;
    circuits[i].vent_request = 0;
    circuits[i].on_time = 1;
    circuits[i].off_time = 100;
    circuits[i].last_change = millis();
    circuits[i].last_seen = 0;
    circuits[i].duty_cycle = 0.0;
  }

  Serial.println("6 DS18B20 sensoren verwacht");

  // WiFi setup
  WiFi.mode(WIFI_STA);

  if (static_ip_str.length() > 0 && static_ip.fromString(static_ip_str)) {
    IPAddress gateway(192, 168, 1, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.config(static_ip, gateway, subnet, gateway);
    Serial.println("Static IP: " + static_ip_str);
  }

  if (wifi_ssid.length() > 0) {
    WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str());
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
      delay(500);
      Serial.print(".");
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi mislukt -> AP mode");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("HVAC-Setup");
    ap_mode_active = true;
    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
    Serial.println("AP IP: " + WiFi.softAPIP().toString());
  } else {
    Serial.println("\nWiFi verbonden: " + WiFi.localIP().toString());
  }

  if (MDNS.begin(room_id.c_str())) {
    Serial.println("mDNS: http://" + room_id + ".local");
  }

  setenv("TZ", "CET-1CEST,M3.5.0/02,M10.5.0/03", 1);
  tzset();
  configTzTime("CET-1CEST,M3.5.0/02,M10.5.0/03", "pool.ntp.org", "time.nist.gov");

  setupWebServer();
  Serial.println("Webserver gestart");
  
  // Print circuit configuratie
  Serial.println("\n=== Circuit Configuratie ===");
  for (int i = 0; i < circuits_num; i++) {
    Serial.printf("Circuit %d: %s", i + 1, circuits[i].name.c_str());
    if (circuits[i].has_tstat) {
      Serial.printf(" [TSTAT pin %d]", circuits[i].tstat_pin);
    }
    if (circuits[i].ip.length() > 0) {
      Serial.printf(" [IP: %s]", circuits[i].ip.c_str());
    }
    if (circuits[i].mdns.length() > 0) {
      Serial.printf(" [mDNS: %s]", circuits[i].mdns.c_str());
    }
    Serial.printf(" [%.3f kW]\n", circuits[i].power_kw);
  }
  Serial.println("============================\n");
}

void loop() {
  if (ap_mode_active) dnsServer.processNextRequest();

  uptime_sec = millis() / 1000;
  readBoilerTemps();
  pollRooms();
  ecoPumpLogic();

  delay(100);
}

