/* ESP32C6_HVACTEST.ino = Centrale HVAC controller voor kelder (ESP32-C6) op basis van particle sketch voor Flobecq
Transition from Photon based to ESP32 based Home automation system. Developed together with ChatGPT & Grok in januari '26.
Thuis bereikbaar op http://hvactest.local of http://192.168.1.36 => Andere controller: Naam (sectie DNS/MDNS) + static IP aanpassen!
06jan26 23:00 Version 3 - Complete Testroom styling + OTA + WiFi scan. Developed with Claude - January 2026

✅ Testroom look & feel (geel/blauw/rood kleurenschema)
✅ OTA Update pagina
✅ WiFi scan functionaliteit
✅ Sensor & circuit nicknames
✅ Responsive design
✅ Auto-refresh homepage (5 sec)
*/


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
const char* NVS_CIRCUIT_NICK_BASE = "circuit_nick_";
const char* NVS_SENSOR_NICK_BASE = "sensor_nick_";
const char* NVS_ROOM_IP_BASE = "room_ip_";
const char* NVS_ROOM_NAME_BASE = "room_name_";
const char* NVS_ECO_THRESHOLD = "eco_thresh";
const char* NVS_ECO_HYSTERESIS = "eco_hyst";
const char* NVS_POLL_INTERVAL = "poll_interval";

// Runtime vars
String room_id = "HVAC";
String wifi_ssid = "";
String wifi_pass = "";
String static_ip_str = "";
IPAddress static_ip;
int circuits_num = 7;
String circuit_nicknames[16];
String sensor_nicknames[6];
String room_ips[10];
String room_names[10];
float eco_threshold = 12.0;
float eco_hysteresis = 2.0;
int poll_interval = 20;

#define RELAY_PUMP_SCH 8
#define RELAY_PUMP_WON 9

bool circuit_on[16] = {false};
int vent_percent = 0;
unsigned long circuit_on_time[16] = {0};
unsigned long circuit_off_time[16] = {0};
unsigned long circuit_last_change[16] = {0};
float circuit_dc[16] = {0.0};

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

void pollRooms() {
  if (millis() - last_poll < (unsigned long)poll_interval * 1000) return;
  last_poll = millis();
  vent_percent = 0;
  for (int i = 0; i < 10; i++) {
    if (room_ips[i].length() == 0) continue;
    HTTPClient http;
    http.begin("http://" + room_ips[i] + "/json");
    http.setTimeout(5000);
    if (http.GET() == 200) {
      DynamicJsonDocument doc(2048);
      deserializeJson(doc, http.getString());
      bool heating = doc["y"] | false;
      int vent = doc["z"] | 0;
      if (i < circuits_num && heating != circuit_on[i]) {
        if (mcp_available) mcp.digitalWrite(i, heating ? LOW : HIGH);
        if (heating) circuit_off_time[i] += millis() - circuit_last_change[i];
        else circuit_on_time[i] += millis() - circuit_last_change[i];
        circuit_last_change[i] = millis();
        circuit_on[i] = heating;
      }
      if (vent > vent_percent) vent_percent = vent;
    }
    http.end();
  }
  analogWrite(VENT_FAN_PIN, map(vent_percent, 0, 100, 0, 255));
  for (int i = 0; i < circuits_num; i++) {
    unsigned long total = circuit_on_time[i] + circuit_off_time[i];
    if (total > 0) circuit_dc[i] = 100.0 * circuit_on_time[i] / total;
  }
}

void ecoPumpLogic() {
  static bool pumping = false;
  static unsigned long pump_start = 0;
  bool demand = false;
  for (int i = 0; i < circuits_num; i++) if (circuit_on[i]) { demand = true; break; }
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


String getMainPage() {
  String html;
  html.reserve(12000);
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
    .header {display:flex;background:#ffcc00;color:black;padding:10px 15px;font-size:18px;font-weight:bold;align-items:center;box-sizing:border-box;}
    .header-left {flex:1;text-align:left;}
    .header-right {flex:1;text-align:right;font-size:15px;}
    .container {display:flex;flex-direction:row;min-height:calc(100vh - 60px);}
    .sidebar {width:80px;padding:10px 5px;background:#ffffff;border-right:3px solid #cc0000;box-sizing:border-box;flex-shrink:0;}
    .sidebar a {display:block;background:#336699;color:white;padding:8px;margin:8px 0;text-decoration:none;font-weight:bold;font-size:12px;border-radius:6px;text-align:center;line-height:1.3;width:60px;box-sizing:border-box;margin-left:auto;margin-right:auto;}
    .sidebar a:hover {background:#003366;}
    .sidebar a.active {background:#cc0000;}
    .main {flex:1;padding:15px;overflow-y:auto;box-sizing:border-box;}
    .group-title {font-size:17px;font-style:italic;font-weight:bold;color:#336699;margin:20px 0 8px 0;}
    table {width:100%;border-collapse:collapse;margin-bottom:15px;}
    td.label {color:#336699;font-size:13px;padding:8px 5px;width:40%;border-bottom:1px solid #ddd;text-align:left;vertical-align:middle;}
    td.value {background:#e6f0ff;font-size:13px;padding:8px 5px;border-bottom:1px solid #ddd;text-align:center;vertical-align:middle;}
    tr.header-row {background:#336699;color:white;}
    tr.header-row td {color:white;background:#336699;}
    @media (max-width: 600px) {
      .container {flex-direction:column;}
      .sidebar {width:100%;border-right:none;border-bottom:3px solid #cc0000;padding:10px 0;display:flex;justify-content:center;}
      .sidebar a {width:70px;margin:0 5px;}
      .main {padding:10px;}
      .group-title {font-size:16px;margin:15px 0 6px 0;}
      td.label {font-size:12px;padding:6px 4px;width:50%;}
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
        <tr class="header-row"><td class="label">#</td><td class="value">Naam</td><td class="value">State</td><td class="value">Duty-cycle</td></tr>
)rawliteral";
  
  for (int i = 0; i < circuits_num; i++) {
    html += "<tr><td class=\"label\">" + String(i + 1) + "</td><td class=\"value\">" + circuit_nicknames[i] + "</td><td class=\"value\">" + String(circuit_on[i] ? "AAN" : "UIT") + "</td><td class=\"value\">" + String(circuit_dc[i], 1) + " %</td></tr>";
  }
  
  html += R"rawliteral(
      </table>
      <p style="text-align:center;margin-top:30px;"><a href="/settings" style="color:#336699;text-decoration:underline;font-size:16px;">→ Instellingen</a></p>
    </div>
  </div>
</body>
</html>
)rawliteral";
  return html;
}

String getLogData() {
  DynamicJsonDocument doc(4096);
  doc["timestamp"] = millis() / 1000;
  doc["eco_qtot"] = eco_qtot;
  doc["sch_qtot"] = sch_qtot;
  doc["vent_percent"] = vent_percent;
  doc["mcp_available"] = mcp_available;
  doc["wifi_rssi"] = WiFi.RSSI();
  doc["free_heap"] = (ESP.getFreeHeap() * 100) / ESP.getHeapSize();
  
  JsonArray temps = doc.createNestedArray("temperatures");
  for (int i = 0; i < 6; i++) {
    JsonObject t = temps.createNestedObject();
    t["name"] = sensor_nicknames[i];
    t["temp"] = sch_temps[i];
    t["ok"] = sensor_ok[i];
  }
  
  JsonArray circuits = doc.createNestedArray("circuits");
  for (int i = 0; i < circuits_num; i++) {
    JsonObject c = circuits.createNestedObject();
    c["id"] = i + 1;
    c["name"] = circuit_nicknames[i];
    c["on"] = circuit_on[i];
    c["dc"] = circuit_dc[i];
  }
  String json;
  serializeJson(doc, json);
  return json;
}

String getWifiScanJson() {
  DynamicJsonDocument doc(4096);
  int n = WiFi.scanNetworks();
  JsonArray networks = doc.createNestedArray("networks");
  for (int i = 0; i < n; i++) {
    JsonObject net = networks.createNestedObject();
    net["ssid"] = WiFi.SSID(i);
    net["rssi"] = WiFi.RSSI(i);
    net["secure"] = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
  }
  String json;
  serializeJson(doc, json);
  return json;
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

  // OTA Update page
  server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = R"rawliteral(
<!DOCTYPE html>
<html lang="nl">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>)rawliteral" + room_id + R"rawliteral( - OTA Update</title>
  <style>
    body {font-family:Arial,Helvetica,sans-serif;background:#ffffff;margin:0;padding:0;}
    .header {display:flex;background:#ffcc00;color:black;padding:10px 15px;font-size:18px;font-weight:bold;align-items:center;}
    .header-left {flex:1;text-align:left;}
    .header-right {flex:1;text-align:right;font-size:15px;}
    .container {display:flex;min-height:calc(100vh - 60px);}
    .sidebar {width:80px;padding:10px 5px;background:#ffffff;border-right:3px solid #cc0000;box-sizing:border-box;}
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
      <br><br><a href="/" style="color:#336699;text-decoration:underline;">← Terug naar Status</a>
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
      : "<h2 style='color:#f00'>Update mislukt!</h2><p>Probeer opnieuw.</p>");
    if (success) { delay(1000); ESP.restart(); }
  }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
    if (!index) {
      Serial.println("\n=== OTA UPDATE GESTART ===");
      Update.begin(UPDATE_SIZE_UNKNOWN);
    }
    Update.write(data, len);
    if (final) {
      if (Update.end(true)) Serial.println("OTA succesvol");
      else Serial.println("OTA fout");
    }
  });

  server.on("/reboot", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", "<h2>Rebooting...</h2>");
    delay(500);
    ESP.restart();
  });

  // Settings page - DEEL 1
  server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *request){
    String sensorNamesHtml = "";
    for (int i = 0; i < 6; i++) {
      sensorNamesHtml += "<label style=\"display:block;margin:6px 0;\">Sensor " + String(i + 1) + ": ";
      sensorNamesHtml += "<input type=\"text\" name=\"sensor_nick_" + String(i) + "\" value=\"" + sensor_nicknames[i] + "\" style=\"width:220px;\"></label>";
    }
    
    String circuitNamesHtml = "";
    for (int i = 0; i < 16; i++) {
      circuitNamesHtml += "<label style=\"display:block;margin:6px 0;\">Circuit " + String(i + 1) + ": ";
      circuitNamesHtml += "<input type=\"text\" name=\"circuit_nick_" + String(i) + "\" value=\"" + circuit_nicknames[i] + "\" style=\"width:220px;\"></label>";
    }

    String roomsHtml = "";
    for (int i = 0; i < 10; i++) {
      roomsHtml += "<tr><td class=\"label\">Room " + String(i + 1) + " IP</td>";
      roomsHtml += "<td class=\"input\"><input type=\"text\" name=\"room_ip_" + String(i) + "\" value=\"" + room_ips[i] + "\"></td></tr>";
      roomsHtml += "<tr><td class=\"label\">Room " + String(i + 1) + " Naam</td>";
      roomsHtml += "<td class=\"input\"><input type=\"text\" name=\"room_name_" + String(i) + "\" value=\"" + room_names[i] + "\"></td></tr>";
    }

    String html;
    html.reserve(16000);
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
    .sidebar {width:80px;padding:10px 5px;background:#ffffff;border-right:3px solid #cc0000;box-sizing:border-box;flex-shrink:0;}
    .sidebar a {display:block;background:#336699;color:white;padding:8px;margin:8px 0;text-decoration:none;font-weight:bold;font-size:12px;border-radius:6px;text-align:center;line-height:1.3;width:60px;margin-left:auto;margin-right:auto;}
    .sidebar a:hover {background:#003366;}
    .sidebar a.active {background:#cc0000;}
    .main {flex:1;padding:20px;overflow-y:auto;box-sizing:border-box;}
    .warning {background:#ffe6e6;border:2px solid #cc0000;padding:15px;margin:20px 0;border-radius:8px;text-align:center;font-weight:bold;color:#990000;}
    .section-title {font-size:18px;font-weight:bold;color:#336699;margin:25px 0 10px 0;padding-bottom:5px;border-bottom:2px solid #336699;}
    .form-table {width:100%;border-collapse:collapse;margin:15px 0;}
    .form-table td.label {width:35%;padding:12px 8px;vertical-align:middle;font-weight:bold;color:#336699;}
    .form-table td.input {width:65%;padding:12px 8px;vertical-align:middle;}
    .form-table input[type=text], .form-table input[type=password], .form-table input[type=number], .form-table select {width:100%;padding:8px;border:1px solid #ccc;border-radius:4px;box-sizing:border-box;}
    .form-table tr {border-bottom:1px solid #eee;}
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
        Verkeerde WiFi-instellingen kunnen de controller onbereikbaar maken!<br><br>
        <strong>Geen WiFi verbinding?</strong> De controller start automatisch een AP:<br>
        • Naam: HVAC-Setup<br>
        • Ga naar http://192.168.4.1/settings om WiFi in te stellen
      </div>

      <form action="/save_settings" method="get" id="settingsForm">
        
        <div class="section-title">WiFi Configuratie</div>
        <button type="button" class="scan-btn" onclick="scanWifi()">Scan netwerken</button>
        <table class="form-table">
          <tr>
            <td class="label">WiFi SSID</td>
            <td class="input">
              <select id="ssid_select" name="wifi_ssid" onchange="document.getElementById('ssid_manual').value=this.value">
                <option value="">Selecteer netwerk...</option>
              </select><br>
              Of handmatig: <input id="ssid_manual" type="text" name="wifi_ssid_manual" value=")rawliteral" + wifi_ssid + R"rawliteral(">
            </td>
          </tr>
          <tr>
            <td class="label">WiFi Password</td>
            <td class="input"><input type="password" name="wifi_pass" value=")rawliteral" + wifi_pass + R"rawliteral("></td>
          </tr>
          <tr>
            <td class="label">Static IP (optioneel)</td>
            <td class="input"><input type="text" name="static_ip" value=")rawliteral" + static_ip_str + R"rawliteral(" placeholder="192.168.1.50 (leeg = DHCP)"></td>
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

        <div class="section-title">Circuit Nicknames</div>
        <div style="padding:10px;">)rawliteral" + circuitNamesHtml + R"rawliteral(</div>

        <div class="section-title">Room Controllers</div>
        <table class="form-table">
          )rawliteral" + roomsHtml + R"rawliteral(
        </table>

        <div style="text-align:center;">
          <button type="submit" class="submit-btn">Opslaan & Reboot</button>
        </div>
      </form>

      <script>
        function scanWifi() {
          fetch('/scan').then(r => r.json()).then(data => {
            let sel = document.getElementById('ssid_select');
            sel.innerHTML = '<option value="">Selecteer netwerk...</option>';
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

  // Save settings handler
  server.on("/save_settings", HTTP_GET, [](AsyncWebServerRequest *request){
    auto arg = [&](const char* n, const String& d="") {
      return request->hasArg(n) ? request->arg(n) : d;
    };

    String ssid_sel = arg("wifi_ssid");
    String ssid_manual = arg("wifi_ssid_manual");
    if (ssid_manual.length() > 0) preferences.putString(NVS_WIFI_SSID, ssid_manual);
    else if (ssid_sel.length() > 0) preferences.putString(NVS_WIFI_SSID, ssid_sel);
    
    if (request->hasArg("wifi_pass")) preferences.putString(NVS_WIFI_PASS, arg("wifi_pass"));
    preferences.putString(NVS_STATIC_IP, arg("static_ip", ""));
    preferences.putString(NVS_ROOM_ID, arg("room_id", room_id));
    preferences.putInt(NVS_CIRCUITS_NUM, arg("circuits_num", "7").toInt());
    preferences.putFloat(NVS_ECO_THRESHOLD, arg("eco_thresh", "12.0").toFloat());
    preferences.putFloat(NVS_ECO_HYSTERESIS, arg("eco_hyst", "2.0").toFloat());
    preferences.putInt(NVS_POLL_INTERVAL, arg("poll_interval", "20").toInt());

    for (int i = 0; i < 6; i++) {
      String param = "sensor_nick_" + String(i);
      if (request->hasArg(param.c_str())) {
        String nick = request->arg(param.c_str());
        nick.trim();
        if (nick.length() == 0) nick = "Sensor " + String(i + 1);
        preferences.putString((String(NVS_SENSOR_NICK_BASE) + i).c_str(), nick);
      }
    }

    for (int i = 0; i < 16; i++) {
      String param = "circuit_nick_" + String(i);
      if (request->hasArg(param.c_str())) {
        String nick = request->arg(param.c_str());
        nick.trim();
        if (nick.length() == 0) nick = "Circuit " + String(i + 1);
        preferences.putString((String(NVS_CIRCUIT_NICK_BASE) + i).c_str(), nick);
      }
    }

    for (int i = 0; i < 10; i++) {
      String ip_param = "room_ip_" + String(i);
      String name_param = "room_name_" + String(i);
      if (request->hasArg(ip_param.c_str())) {
        preferences.putString((String(NVS_ROOM_IP_BASE) + i).c_str(), request->arg(ip_param.c_str()));
      }
      if (request->hasArg(name_param.c_str())) {
        preferences.putString((String(NVS_ROOM_NAME_BASE) + i).c_str(), request->arg(name_param.c_str()));
      }
    }

    request->send(200, "text/html", "<h2 style='text-align:center;color:#336699;'>Opgeslagen! Rebooting...</h2>");
    delay(800);
    ESP.restart();
  });

  server.begin();
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== HVAC Controller boot ===");

  Wire.begin(I2C_SDA, I2C_SCL);
  
  if (mcp.begin_I2C(0x20)) {
    Serial.println("MCP23017 gevonden!");
    mcp_available = true;
    for (int i = 0; i < 16; i++) {
      mcp.pinMode(i, OUTPUT);
      mcp.digitalWrite(i, HIGH);
    }
  } else {
    Serial.println("MCP23017 niet gevonden");
    mcp_available = false;
  }

  preferences.begin("hvac-config", false);

  room_id = preferences.getString(NVS_ROOM_ID, "HVAC");
  wifi_ssid = preferences.getString(NVS_WIFI_SSID, "");
  wifi_pass = preferences.getString(NVS_WIFI_PASS, "");
static_ip_str = preferences.getString(NVS_STATIC_IP, "");
circuits_num = preferences.getInt(NVS_CIRCUITS_NUM, 7);
circuits_num = constrain(circuits_num, 1, 16);
for (int i = 0; i < 6; i++) {
sensor_nicknames[i] = preferences.getString((String(NVS_SENSOR_NICK_BASE) + i).c_str(), "Sensor " + String(i + 1));
}
for (int i = 0; i < 16; i++) {
circuit_nicknames[i] = preferences.getString((String(NVS_CIRCUIT_NICK_BASE) + i).c_str(), "Circuit " + String(i + 1));
}
for (int i = 0; i < 10; i++) {
room_ips[i] = preferences.getString((String(NVS_ROOM_IP_BASE) + i).c_str(), "");
room_names[i] = preferences.getString((String(NVS_ROOM_NAME_BASE) + i).c_str(), "Room " + String(i + 1));
}
eco_threshold = preferences.getFloat(NVS_ECO_THRESHOLD, 12.0);
eco_hysteresis = preferences.getFloat(NVS_ECO_HYSTERESIS, 2.0);
poll_interval = preferences.getInt(NVS_POLL_INTERVAL, 20);
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
}
void loop() {
if (ap_mode_active) dnsServer.processNextRequest();
uptime_sec = millis() / 1000;
readBoilerTemps();
pollRooms();
ecoPumpLogic();
delay(100);
}





