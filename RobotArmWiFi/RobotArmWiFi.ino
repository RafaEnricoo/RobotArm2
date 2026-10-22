/*
  =====================================================================
   ROBOT ARM + CONVEYOR — VERSI WIFI (WebSocket ke server laptop)
  =====================================================================
   Board   : ESP32 Dev Module
   Library : ESP32Servo (Kevin Harrington), ArduinoJson 7.x (Benoit Blanchon),
             WebSockets (Markus Sattler)

   Arsitektur:
     Browser  <--WebSocket-->  server.py (laptop)  <--WebSocket/WiFi-->  ESP32

   Cara pakai:
     1. Isi WIFI_SSID & WIFI_PASS di bawah (laptop & ESP harus di WiFi yang sama).
     2. Di laptop jalankan:  python server.py
     3. Upload sketch ini. ESP otomatis mencari laptop (tidak perlu isi IP).
     4. Buka http://localhost:8765 di browser laptop.
   Serial Monitor (115200) boleh dibuka untuk melihat log.
  =====================================================================
*/

#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebSocketsClient.h>
#include <esp_system.h>

// =====================================================================
//                            KONFIGURASI
// =====================================================================

// ---- WiFi & server laptop ----
const char* WIFI_SSID = "NAMA_WIFI";      // ganti dengan nama WiFi kamu
const char* WIFI_PASS = "PASSWORD_WIFI";  // ganti dengan password WiFi kamu
// Kosongkan ("") supaya ESP mencari laptop otomatis lewat broadcast UDP.
// Kalau auto-cari gagal (WiFi kampus sering memblokir broadcast),
// isi dengan IP laptop yang tampil saat server.py dijalankan, mis. "192.168.1.10".
const char*    SERVER_HOST    = "";
const uint16_t SERVER_PORT    = 8765;
const uint16_t DISCOVERY_PORT = 8766;

// ---- Servo ----
// Urutan: 0 = Base, 1 = Siku, 2 = Lengan Atas, 3 = Gripper
const uint8_t NUM_SERVO = 4;
const uint8_t SERVO_PIN[NUM_SERVO]  = {13, 14, 27, 26};
const char*   SERVO_NAME[NUM_SERVO] = {"Base", "Siku", "Lengan Atas", "Gripper"};

// Batas sudut per servo. KALIBRASI ini sesuai rangka kamu supaya servo
// tidak memaksa mentok (rangka stik es krim bisa patah / servo panas).
//                                 Base Siku LnAtas Gripper
const int SERVO_MIN[NUM_SERVO]  = {  0,   0,   0,   0};
const int SERVO_MAX[NUM_SERVO]  = {180, 180, 180, 180};
const int SERVO_HOME[NUM_SERVO] = { 90,  0,  180,  180};

// Lebar pulsa SG90 (mikrodetik) untuk 0° dan 180°
const int SERVO_US_MIN = 500;
const int SERVO_US_MAX = 2400;

// ---- Motor DC conveyor via L298N ----
// Lepas jumper ENA di L298N, lalu sambungkan ENA ke pin di bawah (PWM).
const uint8_t PIN_ENA = 25;
const uint8_t PIN_IN1 = 33;
const uint8_t PIN_IN2 = 32;
const bool    CONVEYOR_INVERT = false; // true kalau arah maju/mundur terbalik
const int     MOTOR_MIN_DUTY  = 110;   // duty minimal (0-255) agar motor mulai berputar

// ---- Mode hemat daya (untuk servo yang ambil daya dari USB laptop) ----
// true  = servo bergerak lebih pelan, maksimal 2 servo bergerak bersamaan
//         -> mencegah ESP restart (brownout).
// false = pakai kalau servo sudah diberi supply 5V terpisah.
const bool LOW_POWER_MODE      = true;
const int  LP_MAX_SPEED        = 45;   // batas kecepatan servo (derajat/detik)
const int  LP_MAX_MOVING       = 2;    // jumlah servo yang boleh bergerak bersamaan
const int  LP_STARTUP_DELAY_MS = 900;  // jeda menyalakan tiap servo saat boot

// ---- Program ----
const int MAX_STEPS = 60;
const char* PROG_FILE = "/program.json";

// =====================================================================
//                              STATE
// =====================================================================

Servo     servo[NUM_SERVO];
ESP32PWM  motorPwm;
WebSocketsClient ws;
WiFiUDP  udp;
bool     wifiUp      = false;
bool     udpOn       = false;
bool     wsStarted   = false;     // ws.begin() sudah dipanggil
bool     wsConnected = false;     // tersambung ke server laptop
uint32_t wsDownSince = 0;
String   bootInfo;                // alasan restart, dikirim ke web sekali
bool     bootBad     = false;
bool     bootSent    = false;
bool     hostOn   = false;        // web sudah terhubung (kirim "hello")
uint32_t lastRx   = 0;            // waktu pesan terakhir dari web
const uint32_t HOST_TIMEOUT_MS = 3000;

float curPos[NUM_SERVO];        // posisi aktual (dengan smoothing)
int   tgtPos[NUM_SERVO];        // posisi tujuan
int   lastUs[NUM_SERVO];        // pulsa terakhir yang ditulis
float moveSpeed   = 60;         // kecepatan gerak aktif (derajat/detik)
int   manualSpeed = 60;         // kecepatan untuk mode manual
int   convVal     = 0;          // conveyor: -100 (mundur) .. 0 .. 100 (maju)

struct Step {
  uint8_t  a[NUM_SERVO];  // sudut tiap servo
  int8_t   conv;          // conveyor -100..100
  uint16_t wait;          // tunggu setelah pose tercapai (ms)
  uint8_t  spd;           // kecepatan gerak ke pose ini (°/s)
};
Step prog[MAX_STEPS];
int  progLen  = 0;
bool loopMode = true;

enum Mode { MODE_MANUAL, MODE_AUTO, MODE_ESTOP };
Mode     mode      = MODE_MANUAL;
int      stepIdx   = -1;
bool     waiting   = false;
uint32_t waitStart = 0;
uint32_t cycles    = 0;

uint32_t lastServoTick = 0;
uint32_t lastStatus    = 0;

// =====================================================================
//                           UTILITAS
// =====================================================================

int clampAngle(int i, int v) { return constrain(v, SERVO_MIN[i], SERVO_MAX[i]); }

const char* modeName() {
  switch (mode) {
    case MODE_AUTO:  return "AUTO";
    case MODE_ESTOP: return "ESTOP";
    default:         return "MANUAL";
  }
}

void writeServo(int i, float deg) {
  int us = SERVO_US_MIN + (int)((deg / 180.0f) * (SERVO_US_MAX - SERVO_US_MIN) + 0.5f);
  if (us != lastUs[i]) {
    servo[i].writeMicroseconds(us);
    lastUs[i] = us;
  }
}

// Gerak servo bertahap (non-blocking) supaya halus & rangka tidak tersentak
void updateServos() {
  uint32_t now = millis();
  if (now - lastServoTick < 20) return;
  float dt = (now - lastServoTick) / 1000.0f;
  lastServoTick = now;
  float spd = LOW_POWER_MODE ? min(moveSpeed, (float)LP_MAX_SPEED) : moveSpeed;
  float maxStep = spd * dt;
  int moving = 0;
  for (int i = 0; i < NUM_SERVO; i++) {
    float d = tgtPos[i] - curPos[i];
    if (fabsf(d) > 0.01f) {
      // Hemat daya: servo lain menunggu giliran supaya arus tidak melonjak
      if (LOW_POWER_MODE && moving >= LP_MAX_MOVING) { writeServo(i, curPos[i]); continue; }
      moving++;
    }
    if (fabsf(d) <= maxStep) curPos[i] = tgtPos[i];
    else                     curPos[i] += (d > 0 ? maxStep : -maxStep);
    writeServo(i, curPos[i]);
  }
}

bool allReached() {
  for (int i = 0; i < NUM_SERVO; i++)
    if (fabsf(tgtPos[i] - curPos[i]) > 0.5f) return false;
  return true;
}

void holdPosition() {
  for (int i = 0; i < NUM_SERVO; i++) tgtPos[i] = clampAngle(i, (int)roundf(curPos[i]));
}

void setConveyor(int v) {
  v = constrain(v, -100, 100);
  convVal = v;
  if (v == 0) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    motorPwm.write(0);
    return;
  }
  bool fwd = (v > 0) != CONVEYOR_INVERT;
  digitalWrite(PIN_IN1, fwd ? HIGH : LOW);
  digitalWrite(PIN_IN2, fwd ? LOW : HIGH);
  int duty = map(abs(v), 1, 100, MOTOR_MIN_DUTY, 255);
  motorPwm.write(duty);
}

// =====================================================================
//                        KIRIM DATA KE WEB
// =====================================================================

void sendJson(int num, JsonDocument& doc) {
  (void)num;
  if (!wsConnected) return;
  String out;
  serializeJson(doc, out);
  ws.sendTXT(out);
}

void sendMsg(int num, const char* level, const String& txt) {
  JsonDocument doc;
  doc["t"]   = "msg";
  doc["lv"]  = level;   // "info" | "ok" | "err"
  doc["txt"] = txt;
  sendJson(num, doc);
}

void sendConfig(int num) {
  JsonDocument doc;
  doc["t"] = "cfg";
  JsonArray n = doc["names"].to<JsonArray>();
  JsonArray mn = doc["min"].to<JsonArray>();
  JsonArray mx = doc["max"].to<JsonArray>();
  JsonArray hm = doc["home"].to<JsonArray>();
  for (int i = 0; i < NUM_SERVO; i++) {
    n.add(SERVO_NAME[i]); mn.add(SERVO_MIN[i]); mx.add(SERVO_MAX[i]); hm.add(SERVO_HOME[i]);
  }
  doc["maxSteps"] = MAX_STEPS;
  sendJson(num, doc);
}

void programToJson(JsonDocument& doc) {
  doc["loop"] = loopMode;
  doc["mspd"] = manualSpeed;
  JsonArray arr = doc["steps"].to<JsonArray>();
  for (int i = 0; i < progLen; i++) {
    JsonObject o = arr.add<JsonObject>();
    JsonArray a = o["a"].to<JsonArray>();
    for (int j = 0; j < NUM_SERVO; j++) a.add(prog[i].a[j]);
    o["c"] = prog[i].conv;
    o["d"] = prog[i].wait;
    o["s"] = prog[i].spd;
  }
}

void sendProgram(int num) {
  JsonDocument doc;
  doc["t"] = "prog";
  programToJson(doc);
  sendJson(num, doc);
}

void sendStatus() {
  JsonDocument doc;
  doc["t"]    = "st";
  doc["mode"] = modeName();
  JsonArray p = doc["pos"].to<JsonArray>();
  JsonArray t = doc["tgt"].to<JsonArray>();
  for (int i = 0; i < NUM_SERVO; i++) { p.add((int)roundf(curPos[i])); t.add(tgtPos[i]); }
  doc["conv"] = convVal;
  doc["step"] = stepIdx;
  doc["n"]    = progLen;
  doc["wait"] = waiting;
  doc["cyc"]  = cycles;
  doc["spd"]  = LOW_POWER_MODE ? min((int)moveSpeed, LP_MAX_SPEED) : (int)moveSpeed;
  doc["up"]   = millis() / 1000;
  doc["heap"] = ESP.getFreeHeap();
  doc["cl"]   = hostOn ? 1 : 0;
  sendJson(-1, doc);
}

// =====================================================================
//                     SIMPAN / MUAT PROGRAM (LittleFS)
// =====================================================================

void saveProgram() {
  JsonDocument doc;
  programToJson(doc);
  File f = LittleFS.open(PROG_FILE, "w");
  if (!f) { Serial.println("Gagal menyimpan program"); return; }
  serializeJson(doc, f);
  f.close();
}

bool loadProgramFromJson(JsonVariantConst root) {
  JsonArrayConst arr = root["steps"];
  if (arr.isNull()) return false;
  int n = 0;
  for (JsonObjectConst o : arr) {
    if (n >= MAX_STEPS) break;
    JsonArrayConst a = o["a"];
    for (int j = 0; j < NUM_SERVO; j++) prog[n].a[j] = clampAngle(j, a[j] | SERVO_HOME[j]);
    prog[n].conv = constrain((int)(o["c"] | 0), -100, 100);
    prog[n].wait = constrain((long)(o["d"] | 500), 0L, 60000L);
    prog[n].spd  = constrain((int)(o["s"] | 60), 5, 240);
    n++;
  }
  progLen     = n;
  loopMode    = root["loop"] | loopMode;
  manualSpeed = constrain((int)(root["mspd"] | manualSpeed), 5, 240);
  return true;
}

void loadProgram() {
  if (!LittleFS.exists(PROG_FILE)) { Serial.println("Belum ada program tersimpan"); return; }
  File f = LittleFS.open(PROG_FILE, "r");
  if (!f) return;
  JsonDocument doc;
  if (deserializeJson(doc, f) == DeserializationError::Ok && loadProgramFromJson(doc.as<JsonVariantConst>()))
    Serial.printf("Program dimuat: %d step\n", progLen);
  f.close();
}

bool insertStep(int at, const Step& s) {
  if (progLen >= MAX_STEPS) return false;
  at = constrain(at, 0, progLen);
  for (int k = progLen; k > at; k--) prog[k] = prog[k - 1];
  prog[at] = s;
  progLen++;
  return true;
}

void removeStep(int i) {
  for (int k = i; k < progLen - 1; k++) prog[k] = prog[k + 1];
  progLen--;
}

Step captureStep(int waitMs, int spd) {
  Step s;
  for (int j = 0; j < NUM_SERVO; j++) s.a[j] = tgtPos[j];
  s.conv = convVal;
  s.wait = constrain(waitMs, 0, 60000);
  s.spd  = constrain(spd, 5, 240);
  return s;
}

// =====================================================================
//                         MODE OTOMATIS
// =====================================================================

void startStep(int i) {
  stepIdx = i;
  const Step& s = prog[i];
  for (int j = 0; j < NUM_SERVO; j++) tgtPos[j] = clampAngle(j, s.a[j]);
  moveSpeed = s.spd;
  setConveyor(s.conv);   // conveyor langsung diterapkan saat step dimulai
  waiting = false;
}

void startAuto(int from) {
  if (progLen == 0) { sendMsg(-1, "err", "Program masih kosong. Rekam step dulu di tab Manual."); return; }
  mode   = MODE_AUTO;
  cycles = 0;
  startStep(constrain(from, 0, progLen - 1));
  sendMsg(-1, "ok", "Program dijalankan dari step #" + String(stepIdx + 1));
}

void stopAuto(const String& why) {
  mode = MODE_MANUAL;
  stepIdx = -1;
  waiting = false;
  setConveyor(0);
  holdPosition();
  moveSpeed = manualSpeed;
  sendMsg(-1, "info", why);
}

void emergencyStop() {
  mode = MODE_ESTOP;
  stepIdx = -1;
  waiting = false;
  setConveyor(0);
  holdPosition();
  sendMsg(-1, "err", "EMERGENCY STOP! Semua gerakan dihentikan.");
}

// Alur tiap step: conveyor diset -> servo bergerak ke pose -> tunggu 'wait' ms -> step berikutnya
void runAuto() {
  if (mode != MODE_AUTO) return;
  if (!waiting) {
    if (allReached()) { waiting = true; waitStart = millis(); }
    return;
  }
  if (millis() - waitStart < prog[stepIdx].wait) return;
  int next = stepIdx + 1;
  if (next >= progLen) {
    cycles++;
    if (!loopMode) { stopAuto("Program selesai (" + String(cycles) + " siklus)."); return; }
    next = 0;
  }
  startStep(next);
}

// =====================================================================
//                    PERINTAH DARI WEB (WebSocket)
// =====================================================================

bool validIdx(int i) { return i >= 0 && i < progLen; }

void handleCommand(uint8_t num, JsonDocument& doc) {
  String c = doc["cmd"] | "";
  if (c == "ping") return;
  if (c == "hello") { hostOn = true; sendConfig(-1); sendProgram(-1); sendStatus(); return; }

  // --- Perintah yang selalu diizinkan ---
  if (c == "estop") { emergencyStop(); return; }
  if (c == "reset") {
    if (mode == MODE_ESTOP) {
      mode = MODE_MANUAL; moveSpeed = manualSpeed;
      sendMsg(-1, "ok", "E-STOP direset. Kembali ke mode MANUAL.");
    }
    return;
  }
  if (c == "stop") {
    if (mode == MODE_AUTO) stopAuto("Program dihentikan.");
    else setConveyor(0);
    return;
  }
  if (c == "loop") {
    loopMode = doc["val"] | true;
    saveProgram(); sendProgram(-1);
    return;
  }

  if (mode == MODE_ESTOP) { sendMsg(num, "err", "E-STOP aktif. Tekan RESET dulu."); return; }
  if (mode == MODE_AUTO)  { sendMsg(num, "err", "Robot sedang AUTO. Tekan STOP dulu."); return; }

  // --- Perintah mode MANUAL ---
  bool changed = false;
  int i = doc["i"] | -1;

  if (c == "servo") {
    int id = doc["id"] | -1;
    if (id >= 0 && id < NUM_SERVO) tgtPos[id] = clampAngle(id, doc["val"] | tgtPos[id]);
  }
  else if (c == "conv") {
    setConveyor(doc["val"] | 0);
  }
  else if (c == "home") {
    for (int j = 0; j < NUM_SERVO; j++) tgtPos[j] = SERVO_HOME[j];
  }
  else if (c == "mspeed") {
    manualSpeed = constrain((int)(doc["val"] | manualSpeed), 5, 240);
    moveSpeed = manualSpeed;
    changed = true;
  }
  else if (c == "add") {
    Step s = captureStep(doc["d"] | 500, doc["s"] | manualSpeed);
    int at = doc["at"] | progLen;
    if (insertStep(at, s)) {
      changed = true;
      sendMsg(-1, "ok", "Step #" + String(constrain(at, 0, progLen - 1) + 1) + " disimpan.");
    } else sendMsg(num, "err", "Maksimal " + String(MAX_STEPS) + " step.");
  }
  else if (c == "upd" && validIdx(i)) {
    Step s = captureStep(prog[i].wait, prog[i].spd);
    prog[i] = s;
    changed = true;
    sendMsg(-1, "ok", "Step #" + String(i + 1) + " ditimpa dengan pose sekarang.");
  }
  else if (c == "edit" && validIdx(i)) {
    if (!doc["c"].isNull()) prog[i].conv = constrain((int)doc["c"], -100, 100);
    if (!doc["d"].isNull()) prog[i].wait = constrain((long)doc["d"], 0L, 60000L);
    if (!doc["s"].isNull()) prog[i].spd  = constrain((int)doc["s"], 5, 240);
    changed = true;
  }
  else if (c == "del" && validIdx(i)) {
    removeStep(i);
    changed = true;
  }
  else if (c == "dup" && validIdx(i)) {
    Step copy = prog[i];
    if (insertStep(i + 1, copy)) changed = true;
    else sendMsg(num, "err", "Maksimal " + String(MAX_STEPS) + " step.");
  }
  else if (c == "move" && validIdx(i)) {
    int j = i + (int)(doc["dir"] | 0);
    if (validIdx(j) && j != i) { Step t = prog[i]; prog[i] = prog[j]; prog[j] = t; changed = true; }
  }
  else if (c == "goto" && validIdx(i)) {
    for (int j = 0; j < NUM_SERVO; j++) tgtPos[j] = clampAngle(j, prog[i].a[j]);
  }
  else if (c == "clear") {
    progLen = 0;
    changed = true;
    sendMsg(-1, "info", "Semua step dihapus.");
  }
  else if (c == "play") {
    startAuto(doc["from"] | 0);
  }
  else if (c == "import") {
    if (loadProgramFromJson(doc["data"])) {
      moveSpeed = manualSpeed;
      changed = true;
      sendMsg(-1, "ok", "Program diimport: " + String(progLen) + " step.");
    } else sendMsg(num, "err", "File program tidak valid.");
  }

  if (changed) { saveProgram(); sendProgram(-1); }
}

// Web dianggap terputus -> conveyor manual dihentikan demi keamanan
void hostLost() {
  hostOn = false;
  if (mode == MODE_MANUAL && convVal != 0) setConveyor(0);
}

// Event WebSocket dari server laptop (satu pesan = satu perintah JSON)
void onWsEvent(WStype_t type, uint8_t* payload, size_t len) {
  switch (type) {
    case WStype_CONNECTED:
      wsConnected = true;
      Serial.println("Terhubung ke server laptop.");
      if (!bootSent) { sendMsg(-1, bootBad ? "err" : "info", "ESP: " + bootInfo); bootSent = true; }
      sendConfig(-1);
      sendProgram(-1);
      break;
    case WStype_DISCONNECTED:
      if (wsConnected) { Serial.println("Terputus dari server laptop."); wsDownSince = millis(); }
      wsConnected = false;
      hostLost();
      break;
    case WStype_TEXT: {
      JsonDocument doc;
      if (deserializeJson(doc, payload, len) == DeserializationError::Ok) {
        lastRx = millis();
        handleCommand(0, doc);
      }
      break;
    }
    default: break;
  }
}

void startWs(const String& host, uint16_t port) {
  Serial.printf("Menghubungi server ws://%s:%u/esp ...\n", host.c_str(), port);
  ws.begin(host.c_str(), port, "/esp");
  ws.onEvent(onWsEvent);
  ws.setReconnectInterval(2000);
  ws.enableHeartbeat(5000, 3000, 2);   // deteksi server mati walau TCP tidak tertutup
  wsStarted   = true;
  wsDownSince = millis();
}

// WiFi -> cari server laptop (UDP) -> WebSocket. Semua non-blocking.
void handleNetwork() {
  if (wsStarted) ws.loop();

  if (WiFi.status() != WL_CONNECTED) {
    if (wifiUp) { wifiUp = false; Serial.println("WiFi terputus, mencoba lagi..."); }
    return;
  }
  if (!wifiUp) {
    wifiUp = true;
    Serial.printf("WiFi OK. IP ESP: %s\n", WiFi.localIP().toString().c_str());
  }

  if (!wsStarted) {
    if (strlen(SERVER_HOST)) { startWs(SERVER_HOST, SERVER_PORT); return; }
    if (!udpOn) {
      udp.begin(DISCOVERY_PORT);
      udpOn = true;
      Serial.println("Mencari server laptop (jalankan server.py)...");
    }
    if (udp.parsePacket() > 0) {
      char buf[32];
      int n = udp.read(buf, sizeof(buf) - 1);
      buf[n > 0 ? n : 0] = 0;
      if (strncmp(buf, "ROBOTARM ", 9) == 0) {
        int port = atoi(buf + 9);
        String ip = udp.remoteIP().toString();
        udp.stop();
        udpOn = false;
        startWs(ip, port > 0 ? port : SERVER_PORT);
      }
    }
    return;
  }

  // Mode auto-cari: kalau 15 detik gagal tersambung, cari ulang (IP laptop mungkin berubah)
  if (!strlen(SERVER_HOST) && !wsConnected && millis() - wsDownSince > 15000) {
    ws.disconnect();
    wsStarted = false;
  }
}

// =====================================================================
//                       DIAGNOSA RESET
// =====================================================================

void readResetReason() {
  bootBad = true;
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  bootInfo = "Menyala normal (power on)."; bootBad = false; break;
    case ESP_RST_BROWNOUT:
      bootInfo = "!! ESP RESTART KARENA TEGANGAN DROP (brownout). "
                 "Servo kemungkinan ambil daya dari ESP/USB. Pakai supply 5V terpisah, GND disatukan.";
      break;
    case ESP_RST_PANIC:    bootInfo = "!! ESP restart karena program crash (panic)."; break;
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:      bootInfo = "!! ESP restart karena watchdog (program macet)."; break;
    case ESP_RST_SW:       bootInfo = "Restart oleh software / setelah upload."; bootBad = false; break;
    case ESP_RST_EXT:      bootInfo = "Restart lewat tombol EN/RST."; bootBad = false; break;
    default:               bootInfo = "Alasan restart: " + String((int)esp_reset_reason()); break;
  }
  Serial.println(bootInfo);
}

// =====================================================================
//                          SETUP & LOOP
// =====================================================================

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Robot Arm + Conveyor (WiFi) ===");
  readResetReason();

  // Motor conveyor (L298N)
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Penyimpanan & program
  if (!LittleFS.begin(true)) Serial.println("LittleFS gagal di-mount");
  loadProgram();
  moveSpeed = manualSpeed;

  for (int i = 0; i < NUM_SERVO; i++) { curPos[i] = tgtPos[i] = SERVO_HOME[i]; lastUs[i] = -1; }
  if (LOW_POWER_MODE) setCpuFrequencyMhz(80);   // CPU lebih pelan = arus lebih kecil

  motorPwm.attachPin(PIN_ENA, 1000, 8);   // 1 kHz, 8-bit (0-255)
  setConveyor(0);

  // Servo dinyalakan bergantian supaya lonjakan arus tidak bersamaan
  for (int i = 0; i < NUM_SERVO; i++) {
    Serial.printf("Menyalakan servo %s ...\n", SERVO_NAME[i]);
    servo[i].setPeriodHertz(50);
    servo[i].attach(SERVO_PIN[i], SERVO_US_MIN, SERVO_US_MAX);
    writeServo(i, curPos[i]);
    delay(LOW_POWER_MODE ? LP_STARTUP_DELAY_MS : 150);
  }
  Serial.println("Semua servo aktif.");

  // WiFi dinyalakan SETELAH servo, supaya lonjakan arus tidak bersamaan
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  if (LOW_POWER_MODE) {
    WiFi.setSleep(true);                    // hemat arus (respon sedikit lebih lambat)
    WiFi.setTxPower(WIFI_POWER_8_5dBm);     // daya pancar kecil -> lonjakan arus kecil
  } else {
    WiFi.setSleep(false);                   // respon paling cepat
  }
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Menyambung ke WiFi \"%s\" ...\n", WIFI_SSID);

  lastServoTick = millis();
}

void loop() {
  handleNetwork();
  updateServos();
  runAuto();

  // Pengaman: kalau web tidak mengirim apa pun selama 3 detik, anggap terputus
  if (hostOn && millis() - lastRx > HOST_TIMEOUT_MS) hostLost();

  if (hostOn && millis() - lastStatus >= 100) {   // status 10x per detik
    lastStatus = millis();
    sendStatus();
  }
}
