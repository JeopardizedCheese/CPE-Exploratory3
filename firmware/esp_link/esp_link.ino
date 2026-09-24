#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>  // ArduinoJson 7
#include <math.h>
#include "secrets.h"     // Copy secrets.example.h and enter local credentials.

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif
WiFiUDP udp;
unsigned long lastRx = 0;
uint32_t lastSeq = 0;
String activeSession;
bool targetValid = false;
float targetX = 0, targetY = 0;
int targetColor = 0;

void clearTarget() {
  targetValid = false;
  targetColor = 0;
  digitalWrite(LED_BUILTIN, LOW);
  // This receiver does not drive motors. Future motor code MUST stop here.
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  clearTarget();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis()-start < 15000) delay(100);
  WiFi.setAutoReconnect(true);
  udp.begin(4210);
  Serial.printf("ESP32 IP: %s udp/4210\n", WiFi.localIP().toString().c_str());
}

void loop() {
  if (WiFi.status() != WL_CONNECTED || millis()-lastRx > 300) clearTarget();
  int len = udp.parsePacket();
  if (!len) return;
  if (len >= 1400) {
    while (udp.available()) udp.read();
    clearTarget();
    return;
  }
  char buf[1400];
  int n = udp.read(buf, sizeof(buf)-1);
  if (n <= 0) { clearTarget(); return; }
  buf[n] = 0;
  JsonDocument doc;
  if (deserializeJson(doc, buf) || doc["version"] != 2 ||
      !doc["seq"].is<uint32_t>() || !doc["session"].is<const char*>() ||
      doc["units"] != "mm" || doc["ttl_ms"] != 300 || !doc["targets"].is<JsonArray>()) {
    clearTarget();
    return;
  }
  String session = doc["session"].as<String>();
  uint32_t seq = doc["seq"].as<uint32_t>();
  if (session.length() != 12) { clearTarget(); return; }
  // A new sender session is accepted only after the previous one times out.
  if (session != activeSession) {
    if (activeSession.length() && millis()-lastRx <= 300) return;
    activeSession = session;
    lastSeq = 0;
  }
  if (seq <= lastSeq) return; // Duplicates/reordering cannot refresh watchdog.
  lastSeq = seq;
  lastRx = millis();
  clearTarget(); // Empty, stopped or invalid messages revoke the previous target.
  if (doc["status"] != "ok") return;
  JsonArray targets = doc["targets"].as<JsonArray>();
  for (JsonObject t : targets) {
    if (!t["color"].is<int>() || !t["x"].is<float>() || !t["y"].is<float>() ||
        !t["confidence"].is<float>()) continue;
    int color = t["color"];
    float x = t["x"], y = t["y"], confidence = t["confidence"];
    if (color < 1 || color > 6 || !isfinite(x) || !isfinite(y) ||
        !isfinite(confidence) || x < 0 || x > 2100 || y < 0 || y > 1200 ||
        confidence < .25 || confidence > 1) continue;
    targetX = x; targetY = y; targetColor = color; targetValid = true;
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.printf("seq %lu color %d at %.1f, %.1f mm\n", (unsigned long)seq, color, x, y);
    break;
  }
}
