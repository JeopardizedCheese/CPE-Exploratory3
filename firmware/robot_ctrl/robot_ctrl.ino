// Robot controller: wheels + 2 servos + all safety logic, driven by UDP protocol v3.
// Requires ESP32 Arduino core 3.x and ArduinoJson 7.
//
// States:  IDLE --start--> RUNNING --5 min--> DONE
//            ^                 | stop / button      |
//            +------reset------ ESTOP <-------------+ (reset only if button released)
// Wheels move only in RUNNING and only while drive packets keep arriving.
// Servos may move in IDLE (for calibration) and RUNNING; they freeze in DONE/ESTOP
// (holding the stone instead of dropping it).
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <math.h>
#include "secrets.h"   // copy secrets.example.h -> secrets.h
#include "config.h"

#if !defined(ESP_ARDUINO_VERSION_MAJOR) || ESP_ARDUINO_VERSION_MAJOR < 3
#error "Use ESP32 Arduino core 3.x (Boards Manager: esp32 by Espressif >= 3.0)"
#endif

enum State { IDLE, RUNNING, DONE, ESTOP };
const char *STATE_NAME[] = {"IDLE", "RUNNING", "DONE", "ESTOP"};
State state = IDLE;
const char *reason = "boot";

WiFiUDP udp;
String activeSession;
uint32_t lastSeq = 0;
unsigned long lastRx = 0, lastDrive = 0, runStart = 0, lastStatus = 0, lastLoop = 0;
IPAddress peerIp;
uint16_t peerPort = 0;

float cmdL = 0, cmdR = 0, outL = 0, outR = 0;

const uint8_t servoPin[SERVO_COUNT] = SERVO_PINS;
const float servoMin[SERVO_COUNT] = SERVO_MIN_DEG;
const float servoMax[SERVO_COUNT] = SERVO_MAX_DEG;
const float servoStart[SERVO_COUNT] = SERVO_START_DEG;
float servoPos[SERVO_COUNT], servoTarget[SERVO_COUNT];

const uint32_t MOTOR_FULL = (1u << MOTOR_PWM_BITS) - 1;

// ---------------------------------------------------------------- motors
void writeMotor(int in1, int in2, int pwm, float v, bool invert) {
  if (invert) v = -v;
  v = constrain(v, -1.0f, 1.0f);
  float mag = fabsf(v);
  uint32_t duty = mag < 0.01f ? 0 : (uint32_t)((MIN_DUTY + mag * (MAX_DUTY - MIN_DUTY)) * MOTOR_FULL);
#if MOTOR_DRIVER == DRIVER_IN_IN_PWM
  digitalWrite(in1, v > 0.01f);
  digitalWrite(in2, v < -0.01f);
  ledcWrite(pwm, duty);
#else
  (void)pwm;
  ledcWrite(in1, v > 0.01f ? duty : 0);
  ledcWrite(in2, v < -0.01f ? duty : 0);
#endif
}

void applyMotors() {
  writeMotor(L_IN1, L_IN2, L_PWM, outL, L_INVERT);
  writeMotor(R_IN1, R_IN2, R_PWM, outR, R_INVERT);
}

void motorsOff() {
  cmdL = cmdR = outL = outR = 0;
  applyMotors();
}

void setupMotors() {
#if MOTOR_DRIVER == DRIVER_IN_IN_PWM
  int dirPins[] = {L_IN1, L_IN2, R_IN1, R_IN2};
  for (int p : dirPins) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  ledcAttach(L_PWM, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  ledcAttach(R_PWM, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
#else
  int pwmPins[] = {L_IN1, L_IN2, R_IN1, R_IN2};
  for (int p : pwmPins) ledcAttach(p, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
#endif
  if (MOTOR_STBY >= 0) { pinMode(MOTOR_STBY, OUTPUT); digitalWrite(MOTOR_STBY, HIGH); }
  motorsOff();
}

// Speed up gradually, but slow down / reverse-to-stop instantly.
float ramp(float out, float cmd, float dt) {
  if (fabsf(cmd) < fabsf(out) || (cmd * out) < 0) return (cmd * out < 0) ? 0 : cmd;
  float step = RAMP_PER_SEC * dt;
  if (cmd > out) return fminf(cmd, out + step);
  return fmaxf(cmd, out - step);
}

// ---------------------------------------------------------------- servos
void servoWrite(int i, float deg) {
  float us = SERVO_US_MIN + (SERVO_US_MAX - SERVO_US_MIN) * deg / 180.0f;
  ledcWrite(servoPin[i], (uint32_t)(us * 65535.0f / 20000.0f));   // 50 Hz, 16-bit
}

void setupServos() {
  for (int i = 0; i < SERVO_COUNT; i++) {
    ledcAttach(servoPin[i], 50, 16);
    servoPos[i] = servoTarget[i] = constrain(servoStart[i], servoMin[i], servoMax[i]);
    servoWrite(i, servoPos[i]);
  }
}

bool setServo(int i, float deg) {
  if (i < 0 || i >= SERVO_COUNT || !isfinite(deg)) return false;
  if (state != IDLE && state != RUNNING) return false;
  servoTarget[i] = constrain(deg, servoMin[i], servoMax[i]);
  return true;
}

void updateServos(float dt) {
  float step = SERVO_DEG_PER_SEC * dt;
  for (int i = 0; i < SERVO_COUNT; i++) {
    if (state == DONE || state == ESTOP) servoTarget[i] = servoPos[i];   // freeze
    float d = servoTarget[i] - servoPos[i];
    if (fabsf(d) < 0.01f) continue;
    servoPos[i] += constrain(d, -step, step);
    servoWrite(i, servoPos[i]);
  }
}

// ---------------------------------------------------------------- state
bool buttonPressed() { return ESTOP_PIN >= 0 && digitalRead(ESTOP_PIN) == LOW; }

void enter(State s, const char *why) {
  if (state == s) return;
  state = s;
  reason = why;
  if (s != RUNNING) motorsOff();
  if (s == RUNNING) { runStart = millis(); lastDrive = 0; cmdL = cmdR = 0; }
  Serial.printf("[%lu] -> %s (%s)\n", millis(), STATE_NAME[s], why);
}

// ---------------------------------------------------------------- network
void handlePacket(char *buf, unsigned long now) {
  JsonDocument doc;
  if (deserializeJson(doc, buf) || doc["v"] != 3 || !doc["s"].is<const char *>() ||
      !doc["q"].is<uint32_t>() || !doc["c"].is<const char *>()) {
    cmdL = cmdR = 0;
    return;
  }
  String session = doc["s"].as<String>();
  uint32_t seq = doc["q"].as<uint32_t>();
  if (session.length() != 12) return;
  if (session != activeSession) {
    // A new controller may take over only after the old one has been silent.
    if (activeSession.length() && now - lastRx <= DRIVE_TIMEOUT_MS) return;
    activeSession = session;
    lastSeq = 0;
  }
  if (seq <= lastSeq) return;             // duplicate / reordered: ignore
  lastSeq = seq;
  lastRx = now;
  peerIp = udp.remoteIP();
  peerPort = udp.remotePort();

  const char *c = doc["c"];
  if (!strcmp(c, "drive")) {
    float l = doc["l"] | NAN, r = doc["r"] | NAN;
    if (state != RUNNING || !isfinite(l) || !isfinite(r) || fabsf(l) > 1 || fabsf(r) > 1) {
      cmdL = cmdR = 0;
      return;
    }
    cmdL = l; cmdR = r; lastDrive = now;
  } else if (!strcmp(c, "start")) {
    if (state == IDLE && !buttonPressed()) enter(RUNNING, "start");
  } else if (!strcmp(c, "stop")) {
    enter(ESTOP, "remote stop");
  } else if (!strcmp(c, "reset")) {
    if ((state == DONE || state == ESTOP) && !buttonPressed()) enter(IDLE, "reset");
  } else if (!strcmp(c, "servo")) {
    setServo(doc["i"] | -1, doc["deg"] | NAN);
  } else if (!strcmp(c, "grip")) {
    const char *p = doc["p"] | "";
    if (!strcmp(p, "open")) setServo(GRIP_SERVO, GRIP_OPEN_DEG);
    if (!strcmp(p, "close")) setServo(GRIP_SERVO, GRIP_CLOSE_DEG);
  } else if (!strcmp(c, "lift")) {
    const char *p = doc["p"] | "";
    if (!strcmp(p, "up")) setServo(LIFT_SERVO, LIFT_UP_DEG);
    if (!strcmp(p, "down")) setServo(LIFT_SERVO, LIFT_DOWN_DEG);
  }
  // "ping" and unknown commands only refresh the link.
}

void pollUdp(unsigned long now) {
  for (int k = 0; k < 8; k++) {                 // drain a few packets per loop
    int len = udp.parsePacket();
    if (!len) return;
    if (len >= 512) { while (udp.available()) udp.read(); cmdL = cmdR = 0; continue; }
    char buf[512];
    int n = udp.read(buf, sizeof(buf) - 1);
    if (n <= 0) continue;
    buf[n] = 0;
    handlePacket(buf, now);
  }
}

void sendStatus(unsigned long now) {
  if (!peerPort || now - lastStatus < STATUS_PERIOD_MS) return;
  lastStatus = now;
  JsonDocument doc;
  doc["state"] = STATE_NAME[state];
  doc["why"] = reason;
  doc["t_left_ms"] = state == RUNNING ? (long)(RUN_TIME_MS - (now - runStart)) : (state == DONE ? 0 : (long)RUN_TIME_MS);
  doc["l"] = outL;
  doc["r"] = outR;
  doc["rx_age_ms"] = now - lastRx;
  doc["button"] = buttonPressed();
  doc["rssi"] = WiFi.RSSI();
  JsonArray s = doc["servo"].to<JsonArray>();
  for (int i = 0; i < SERVO_COUNT; i++) s.add(roundf(servoPos[i]));
  char out[256];
  size_t n = serializeJson(doc, out, sizeof(out));
  udp.beginPacket(peerIp, peerPort);
  udp.write((const uint8_t *)out, n);
  udp.endPacket();
}

// ---------------------------------------------------------------- main
void setup() {
  setupMotors();                       // first: make sure wheels are off
  Serial.begin(115200);
  pinMode(STATUS_LED, OUTPUT);
  if (ESTOP_PIN >= 0) pinMode(ESTOP_PIN, INPUT_PULLUP);
  setupServos();
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                // lower latency
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(100);
  WiFi.setAutoReconnect(true);
  udp.begin(UDP_PORT);
  Serial.printf("robot_ctrl ready. IP %s udp/%d\n", WiFi.localIP().toString().c_str(), UDP_PORT);
  lastLoop = millis();
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastLoop) / 1000.0f;
  lastLoop = now;

  if (buttonPressed()) enter(ESTOP, "button");
  pollUdp(now);

  if (state == RUNNING) {
    if (now - runStart >= RUN_TIME_MS) enter(DONE, "time up");
    else if (now - lastDrive > DRIVE_TIMEOUT_MS || WiFi.status() != WL_CONNECTED) cmdL = cmdR = 0;
  } else {
    cmdL = cmdR = 0;
  }
  outL = ramp(outL, cmdL, dt);
  outR = ramp(outR, cmdR, dt);
  applyMotors();
  updateServos(dt);
  sendStatus(now);

  bool blink = (now / 150) % 2;
  digitalWrite(STATUS_LED, state == RUNNING ? HIGH : (state == IDLE ? LOW : blink));
  delay(2);
}
