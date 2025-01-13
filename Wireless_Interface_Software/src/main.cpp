#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Arduino.h>

// AP credentials
const char *ssid = "ESP32_Telemetry";
const char *password = "12345678";

// Server object
AsyncWebServer server(80);

// Telemetry data (dummy values for demonstration)
float temperature = 25.3;
float pressure = 1013.25;
int arm_status = 0;
int ematch_status = 0;

// HTML for the web page
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32 Telemetry</title>
  <style>
    body { font-family: Arial, sans-serif; text-align: center; }
    .container { margin-top: 50px; }
    button { padding: 15px 30px; margin: 10px; font-size: 16px; }
  </style>
</head>
<body>
  <div class="container">
    <h1>ESP32 Telemetry Dashboard</h1>
    <p>Temperature: <span id="temperature">--</span> °C</p>
    <p>Pressure: <span id="pressure">--</span> hPa</p>
    <p>Arm Status: <span id="arm_status">--</span></p>
    <p>eMatch Test Status: <span id="ematch_status">--</span></p>
    <button onclick="sendCommand('arm')">Arm</button>
    <button onclick="sendCommand('ematch')">eMatch Test</button>
  </div>

  <script>
    // Fetch telemetry data every second
    setInterval(() => {
      fetch('/telemetry')
        .then(response => response.json())
        .then(data => {
          document.getElementById('temperature').innerText = data.temperature;
          document.getElementById('pressure').innerText = data.pressure;
          document.getElementById('arm_status').innerText = data.arm_status;
          document.getElementById('ematch_status').innerText = data.ematch_status;
        });
    }, 1000);

    // Send command to ESP32
    function sendCommand(action) {
      fetch('/' + action, { method: 'POST' })
        .then(response => response.text())
        .then(message => alert(message));
    }
  </script>
</body>
</html>
)rawliteral";

// Handle telemetry data requests
void handleTelemetry(AsyncWebServerRequest *request)
{
  String json = "{";
  json += "\"temperature\":" + String(temperature) + ",";
  json += "\"pressure\":" + String(pressure) + ",";
  json += "\"arm_status\":" + String(arm_status) + ",";
  json += "\"ematch_status\":" + String(ematch_status);
  json += "}";
  request->send(200, "application/json", json);
}

// Handle arm button press
void handleArm(AsyncWebServerRequest *request)
{
  arm_status = 1; // Simulate arming the system
  request->send(200, "text/plain", "System Armed");
}

// Handle eMatch test button press
void handleEmatch(AsyncWebServerRequest *request)
{
  ematch_status = 1; // Simulate eMatch test
  delay(500);        // Simulated test delay
  ematch_status = 0; // Reset status
  request->send(200, "text/plain", "eMatch Test Completed");
}

// Setup Access Point and server routes
void setup()
{
  Serial.begin(115200);

  // Setup ESP32 as Access Point
  WiFi.softAP(ssid, password);

  // Print the AP IP address
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Access Point IP Address: ");
  Serial.println(IP);

  // Serve the HTML page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/html", index_html); });

  // Handle telemetry requests
  server.on("/telemetry", HTTP_GET, handleTelemetry);

  // Handle arm button press
  server.on("/arm", HTTP_POST, handleArm);

  // Handle eMatch test button press
  server.on("/ematch", HTTP_POST, handleEmatch);

  // Start server
  server.begin();
}

// Simulate telemetry updates
void loop()
{
  temperature += 0.1 * (random(-5, 5));
  pressure += 0.1 * (random(-2, 2));
  delay(1000);
}