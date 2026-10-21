/*
 * ESP32 4WD Line Follower + Manual Control + RFID Delivery (5-Array IR Version)
 * -----------------------------------------------------------------------------
 * Motor driver: L298N board (Direct PWM on INA/INB/INC/IND)
 * Sensor: 5-channel IR array (digital out) + HC-SR04 ultrasonic + RC522 RFID
 * Buzzer: active HIGH, 3.3 V compatible
 * WiFi LED: blinks while waiting, solid ON when a client is connected
 * WiFi AP: LineBot_ESP32 / 12345678 -> http://192.168.4.1
 * Requires ESP32 Arduino core 3.x + "MFRC522" library
 *
 * RFID DELIVERY FLOW
 *   STANDBY             motors off, waiting for CALL ROBOT
 *     | CALL ROBOT
 *   GOING_TO_SENDER     line follow --(J1 tag)--> stop 3 s -> ROUTES.j1ToSender (S)
 *     | sender tag (X)
 *   AT_SENDER           motors off, waiting for ITEM LOADED
 *     | ITEM LOADED -> 180 deg turn
 *   GOING_TO_RECEIVER   line follow --(J1 tag)--> stop 3 s -> ROUTES.j1ToReceiver (L)
 *     | receiver tag (Y)
 *   AT_RECEIVER         motors off, waiting for RECEIVED
 *     | RECEIVED -> 180 deg turn
 *   RETURNING_TO_BASE   line follow --(J1 tag)--> stop 3 s -> ROUTES.j1ToBase (R)
 *     | base RFID tag
 *     | stop 1 s -> 180 deg turn
 *   endDelivery() -> STANDBY (robot stays at base)
 */

#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <MFRC522.h>

/* ================= WIFI ================= */
const char* AP_SSID = "LineBott_ESP32";
const char* AP_PASS = "12345678";
WebServer server(80);

/* ============== WIFI STATUS LED ============== */
#define WIFI_LED_PIN 4            // GPIO 2 is used by RFID SDA, so LED is on GPIO 4
const unsigned long LED_BLINK_MS = 500;
const unsigned long CLIENT_CHECK_MS = 250;
unsigned long lastLedMs = 0;
unsigned long lastClientCheckMs = 0;
bool ledState = false;
bool clientConnected = false;

/* ============== RFID (RC522) ============== */
#define RFID_SCK  21
#define RFID_MOSI 12
#define RFID_MISO 36
#define RFID_SDA  2               // SS / chip select
#define RFID_RST  255             // RST not connected to ESP32 (wire RST to 3.3V on the module!)
#define RFID_FAST_TIMEOUT 1       // 1 = shorter wait when no tag (less loop delay)
#define RFID_MAX_GAIN     1       // 1 = maximum receiver gain (easier to read while moving)

MFRC522 rfid(RFID_SDA, RFID_RST);
bool rfidOk = false;
String lastUid = "";
const char* lastTag = "";
unsigned long lastRfidPollMs = 0;
const unsigned long RFID_POLL_MS = 30;

/* RC522 can glitch when motors run (power dip / noise). Detect it and recover quickly. */
const unsigned long RFID_HEALTH_MS        = 150;    // health check interval
const unsigned long RFID_FULL_INIT_GAP_MS = 2000;   // slow full re-init at most this often
const unsigned long RFID_LOG_GAP_MS       = 1000;   // Serial glitch message at most this often
unsigned long lastRfidHealthMs   = 0;
unsigned long lastRfidFullInitMs = 0;
unsigned long lastRfidLogMs      = 0;
unsigned long rfidResets = 0;                       // number of glitches seen

/* ---- Tag UIDs: scan a tag, read its UID on the DELIVERY tab / Serial Monitor, paste it here ---- */
struct RfidUid {
  byte size;
  byte bytes[10];
};
const RfidUid UID_J1       = {4, {0xBC, 0x74, 0xC1, 0x01}};   // key ring -> junction J1
const RfidUid UID_SENDER   = {4, {0xAC, 0x71, 0x8F, 0x02}};   // card 1   -> sender   (X)
const RfidUid UID_RECEIVER = {4, {0x09, 0xB2, 0x30, 0x03}};   // card 2   -> receiver (Y)
const RfidUid UID_BASE     = {4, {0x64, 0xD5, 0x30, 0x03}};   // card 3   -> base

/* What to do at J1 for each trip: 'S' = straight, 'L' = left, 'R' = right */
struct Routes {
  char j1ToSender;
  char j1ToReceiver;
  char j1ToBase;
};
const Routes ROUTES = { 'S', 'L', 'R' };

/* ============== DELIVERY TIMING (tune these) ============== */
const unsigned long JUNCTION_STOP_MS    = 3000;  // stop at J1
const unsigned long STRAIGHT_CLEAR_MS   = 400;   // drive straight to clear J1
const int           TURN_SPEED          = 150;   // spin speed for turns
const unsigned long ROUTE_TURN_MIN_MS   = 250;   // min spin time for L/R at J1
const unsigned long UTURN_MIN_MS        = 350;   // min spin time for 180 deg turn
const unsigned long TURN_MAX_MS         = 3500;  // give up waiting for the line
const char          UTURN_DIR           = 'R';   // 180 deg turn direction

/* Base (RFID tag): stop, wait, 180 deg turn, then STANDBY */
const unsigned long BASE_STOP_MS        = 1000;  // wait at base before turning
const int           BASE_TURN_SPEED     = 180;   // spin speed at base
const char          BASE_TURN_DIR       = 'R';   // base turn direction
const bool          BASE_TURN_TIMED     = false; // false = stop turning when line is found
                                                 // true  = spin exactly BASE_TURN_MS
const unsigned long BASE_TURN_MS        = 700;   // used only when BASE_TURN_TIMED = true

/* ============== 5-CHANNEL IR SENSOR PINS (Left -> Right) ============== */
const int irPin[5] = {32, 33, 34, 35, 39};
int irVal[5];

/* Black line on white floor -> true ; White line on black floor -> false */
bool BLACK_LINE = true;

/* ============== MOTOR DRIVER PINS ============== */
#define INA 18   // Left  forward
#define INB 19   // Left  backward
#define INC 22   // Right forward
#define IND 23   // Right backward

/* ============== ULTRASONIC + BUZZER PINS ============== */
#define TRIG_PIN   26
#define ECHO_PIN   27  // HC-SR04 ECHO needs a 5 V to 3.3 V divider
#define BUZZER_PIN 25  // Active buzzer, HIGH = sound

const float STOP_DISTANCE_CM = 13.0;
const unsigned long ULTRASONIC_INTERVAL_MS = 60;
const unsigned long ECHO_WAIT_MS = 30;     // max wait for one echo
const int MISS_LIMIT = 3;                  // consecutive misses before "clear"
float distanceCm = -1.0;       // -1 means no echo / out of range
bool obstacleDetected = false;
unsigned long lastUltrasonicMs = 0;

/* Echo is measured by interrupt, so loop() never blocks */
volatile unsigned long echoStartUs = 0;
volatile unsigned long echoWidthUs = 0;
volatile bool echoReady = false;

void IRAM_ATTR echoISR() {
  if (digitalRead(ECHO_PIN)) {
    echoStartUs = micros();
  } else {
    echoWidthUs = micros() - echoStartUs;
    echoReady = true;
  }
}

#define PWM_FREQ 5000
#define PWM_RES  8      // 0-255

/* If one side spins the wrong way, flip its flag to true */
bool INVERT_LEFT  = false;
bool INVERT_RIGHT = false;

/* ============== CONTROL VARIABLES ============== */
String mode = "STOP";          // LINE / MANUAL / DELIVERY / STOP
String manualCmd = "S";        // F B L R S
int baseSpeed   = 150;
int manualSpeed = 180;

// PID constants tuned for 5-channel array
float Kp = 35.0, Ki = 0.0, Kd = 20.0;
float lastError = 0, integral = 0;
float lastKnownError = 0;

/* ============== DELIVERY STATE ============== */
enum DState { D_STANDBY, D_TO_SENDER, D_AT_SENDER, D_TO_RECEIVER, D_AT_RECEIVER, D_RETURNING };
enum Phase  { PH_IDLE, PH_LINE, PH_JWAIT, PH_ROUTE, PH_UTURN, PH_BASEWAIT, PH_BASETURN };

const char* const DSTATE_NAME[] = {
  "STANDBY", "GOING_TO_SENDER", "AT_SENDER",
  "GOING_TO_RECEIVER", "AT_RECEIVER", "RETURNING_TO_BASE"
};
const char* const PHASE_NAME[] = {
  "IDLE", "LINE", "JUNCTION_WAIT", "ROUTE_TURN", "U_TURN", "BASE_WAIT", "BASE_TURN"
};

DState dState = D_STANDBY;
Phase  phase  = PH_IDLE;

unsigned long phaseStartMs = 0;     // start of junction / base wait
char pendingRoute = 'S';            // route to take after junction wait

char turnDir = 'S';                 // 'L' / 'R' / 'S'
int  turnStage = 0;                 // 0 = leaving old line, 1 = looking for line
unsigned long turnStartMs = 0;
unsigned long turnMinMs = 0;

/* ================= WIFI LED ================= */
void updateWifiLed() {
  // Ask the AP how many clients are connected (not every loop)
  if (millis() - lastClientCheckMs >= CLIENT_CHECK_MS) {
    lastClientCheckMs = millis();
    clientConnected = (WiFi.softAPgetStationNum() > 0);
  }

  if (clientConnected) {
    digitalWrite(WIFI_LED_PIN, HIGH);          // Connected: solid ON
  } else if (millis() - lastLedMs >= LED_BLINK_MS) {
    lastLedMs = millis();                      // Waiting: blink
    ledState = !ledState;
    digitalWrite(WIFI_LED_PIN, ledState ? HIGH : LOW);
  }
}

/* ================= MOTOR HELPERS ================= */
void leftMotor(int spd) {
  spd = constrain(spd, -255, 255);
  if (INVERT_LEFT) spd = -spd;
  if (spd >= 0) { ledcWrite(INA, spd);   ledcWrite(INB, 0); }
  else          { ledcWrite(INA, 0);     ledcWrite(INB, -spd); }
}

void rightMotor(int spd) {
  spd = constrain(spd, -255, 255);
  if (INVERT_RIGHT) spd = -spd;
  if (spd >= 0) { ledcWrite(INC, spd);   ledcWrite(IND, 0); }
  else          { ledcWrite(INC, 0);     ledcWrite(IND, -spd); }
}

void driveStop() {
  leftMotor(0);
  rightMotor(0);
}

/* ================= OBSTACLE CHECK (non-blocking) ================= */
void updateUltrasonic() {
  static bool waiting = false;
  static unsigned long trigMs = 0;
  static int missCount = 0;

  if (waiting) {
    if (echoReady) {
      // Echo received: update distance and obstacle state
      echoReady = false;
      waiting = false;
      missCount = 0;
      distanceCm = echoWidthUs * 0.0343 / 2.0;
      obstacleDetected = (distanceCm <= STOP_DISTANCE_CM);
    } else if (millis() - trigMs > ECHO_WAIT_MS) {
      // No echo this time: keep the old state until misses repeat
      waiting = false;
      if (++missCount >= MISS_LIMIT) {
        distanceCm = -1.0;
        obstacleDetected = false;
      }
    }
    digitalWrite(BUZZER_PIN, obstacleDetected ? HIGH : LOW);
    if (obstacleDetected) driveStop();
    return;
  }

  if (millis() - lastUltrasonicMs < ULTRASONIC_INTERVAL_MS) return;
  lastUltrasonicMs = millis();

  echoReady = false;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  waiting = true;
  trigMs = millis();
}

/* ================= SENSOR READ (5-ARRAY) ================= */
// S1: -4 (far left), S2: -2 (mid left), S3: 0 (center),
// S4: +2 (mid right), S5: +4 (far right)
const float weight[5] = {-4.0, -2.0, 0.0, 2.0, 4.0};

bool readSensors(float &error) {
  float sum = 0;
  int count = 0;
  for (int i = 0; i < 5; i++) {
    int raw = digitalRead(irPin[i]);
    // Most modules output LOW on black line
    irVal[i] = BLACK_LINE ? (raw == LOW ? 1 : 0) : (raw == HIGH ? 1 : 0);
    if (irVal[i]) {
      sum += weight[i];
      count++;
    }
  }
  if (count == 0) return false; // Line lost
  error = sum / count;
  return true;
}

/* ================= LINE FOLLOW ================= */
void lineFollow() {
  float error;
  if (!readSensors(error)) {
    // Line lost: spin back in direction of last known position
    int s = baseSpeed * 0.8;
    if (lastKnownError < 0) { leftMotor(s); rightMotor(-s); }
    else                    { leftMotor(-s); rightMotor(s); }
    return;
  }
  lastKnownError = error;

  integral += error;
  integral = constrain(integral, -50, 50);
  float derivative = error - lastError;
  float correction = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  leftMotor(-baseSpeed - correction);
  rightMotor(-baseSpeed + correction);
}

/* ================= MANUAL DRIVE ================= */
void manualDrive() {
  int s = manualSpeed;
  if      (manualCmd == "B") { leftMotor(s);  rightMotor(s);  }
  else if (manualCmd == "F") { leftMotor(-s); rightMotor(-s); }
  else if (manualCmd == "R") { leftMotor(-s); rightMotor(s);  }
  else if (manualCmd == "L") { leftMotor(s);  rightMotor(-s); }
  else driveStop();
}

/* ================= RFID HELPERS ================= */
String uidToString(const MFRC522::Uid &u) {
  String s = "";
  for (byte i = 0; i < u.size; i++) {
    if (i) s += ':';
    if (u.uidByte[i] < 0x10) s += '0';
    s += String(u.uidByte[i], HEX);
  }
  s.toUpperCase();
  return s;
}

bool uidMatches(const MFRC522::Uid &u, const RfidUid &t) {
  if (u.size != t.size) return false;
  for (byte i = 0; i < u.size; i++) {
    if (u.uidByte[i] != t.bytes[i]) return false;
  }
  return true;
}

const char* tagName(const MFRC522::Uid &u) {
  if (uidMatches(u, UID_J1))       return "J1";
  if (uidMatches(u, UID_SENDER))   return "SENDER";
  if (uidMatches(u, UID_RECEIVER)) return "RECEIVER";
  if (uidMatches(u, UID_BASE))     return "BASE";
  return "UNKNOWN";
}

// Same register setup as MFRC522::PCD_Init(), but WITHOUT the 50+ ms soft reset delay.
// Used for fast recovery when the chip lost its settings (reset by noise / power dip).
void rfidConfig() {
  rfid.PCD_WriteRegister(MFRC522::TModeReg, 0x80);        // timer starts automatically
  rfid.PCD_WriteRegister(MFRC522::TPrescalerReg, 0xA9);
#if RFID_FAST_TIMEOUT
  rfid.PCD_WriteRegister(MFRC522::TReloadRegH, 0x01);     // reload 400 -> ~10 ms no-tag timeout
  rfid.PCD_WriteRegister(MFRC522::TReloadRegL, 0x90);
#else
  rfid.PCD_WriteRegister(MFRC522::TReloadRegH, 0x03);     // reload 1000 -> ~25 ms (library default)
  rfid.PCD_WriteRegister(MFRC522::TReloadRegL, 0xE8);
#endif
  rfid.PCD_WriteRegister(MFRC522::TxASKReg, 0x40);        // 100 % ASK modulation
  rfid.PCD_WriteRegister(MFRC522::ModeReg, 0x3D);         // CRC preset 0x6363
#if RFID_MAX_GAIN
  rfid.PCD_SetAntennaGain(MFRC522::RxGain_max);
#endif
  rfid.PCD_AntennaOn();
}

// Does the chip answer on SPI at all? (0x00 / 0xFF means no answer)
bool rfidChipAnswers() {
  byte ver = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  return (ver != 0x00 && ver != 0xFF);
}

// Is the antenna ON? (after a chip reset the antenna drivers are OFF)
bool rfidConfigured() {
  return (rfid.PCD_ReadRegister(MFRC522::TxControlReg) & 0x03) == 0x03;
}

// Full start: soft reset (slow, ~50 ms) + our settings
void rfidStart(bool verbose) {
  rfid.PCD_Init();
  rfidConfig();
  lastRfidFullInitMs = millis();
  byte ver = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  rfidOk = (ver != 0x00 && ver != 0xFF) && rfidConfigured();
  if (verbose) {
    Serial.print("RFID version register: 0x");
    Serial.println(ver, HEX);
    Serial.println(rfidOk ? "RFID reader OK"
                          : "RFID reader NOT FOUND - check wiring (3.3V, GND, SCK, MOSI, MISO, SDA)");
  }
}

/* ================= DELIVERY ================= */
void resetDelivery() {
  driveStop();
  dState = D_STANDBY;
  phase = PH_IDLE;
}

void endDelivery() {
  resetDelivery();
  Serial.println("[DELIVERY] Base turn done -> STANDBY (waiting for CALL ROBOT)");
}

// Start a spin (or straight run for 'S') ; newPhase is PH_ROUTE, PH_UTURN or PH_BASETURN
void startTurn(char dir, unsigned long minMs, int newPhase) {
  turnDir = dir;
  turnStage = 0;
  turnStartMs = millis();
  turnMinMs = minMs;
  phase = (Phase)newPhase;
}

void finishTurn() {
  // Base turn finished: stay at base and go back to STANDBY
  if (phase == PH_BASETURN) {
    endDelivery();
    return;
  }
  integral = 0;
  lastError = 0;
  lastKnownError = 0;
  phase = PH_LINE;
  Serial.println("[DELIVERY] Turn done -> line follow");
}

void runTurn() {
  float e;
  readSensors(e);
  unsigned long el = millis() - turnStartMs;

  if (turnDir != 'L' && turnDir != 'R') {
    // 'S': drive straight to clear the junction
    leftMotor(-baseSpeed);
    rightMotor(-baseSpeed);
    if (el >= STRAIGHT_CLEAR_MS) finishTurn();
    return;
  }

  int spd = (phase == PH_BASETURN) ? BASE_TURN_SPEED : TURN_SPEED;
  if (turnDir == 'L') { leftMotor(spd);  rightMotor(-spd); }
  else                { leftMotor(-spd); rightMotor(spd);  }

  // Optional purely timed base turn
  if (phase == PH_BASETURN && BASE_TURN_TIMED) {
    if (el >= BASE_TURN_MS) finishTurn();
    return;
  }

  if (turnStage == 0) {
    if (el >= turnMinMs) turnStage = 1;      // spun long enough to leave the old line
  } else if (irVal[2]) {
    finishTurn();                            // centre sensor found the line again
    return;
  }
  if (el >= TURN_MAX_MS) finishTurn();       // safety timeout
}

void startJunction(char route) {
  driveStop();
  pendingRoute = route;
  phase = PH_JWAIT;
  phaseStartMs = millis();
  Serial.printf("[DELIVERY] J1 reached -> stop %lu ms, then route '%c'\n", JUNCTION_STOP_MS, route);
}

void startBaseStop() {
  driveStop();
  phase = PH_BASEWAIT;
  phaseStartMs = millis();
  Serial.printf("[DELIVERY] Base tag -> stop %lu ms, then 180 deg turn\n", BASE_STOP_MS);
}

// Called for every new RFID tag (only acts while following the line in DELIVERY mode)
void handleTag(const MFRC522::Uid &u) {
  if (mode != "DELIVERY" || phase != PH_LINE) return;

  bool isJ1       = uidMatches(u, UID_J1);
  bool isSender   = uidMatches(u, UID_SENDER);
  bool isReceiver = uidMatches(u, UID_RECEIVER);
  bool isBase     = uidMatches(u, UID_BASE);

  if (dState == D_TO_SENDER) {
    if (isJ1) startJunction(ROUTES.j1ToSender);
    else if (isSender) {
      dState = D_AT_SENDER; phase = PH_IDLE; driveStop();
      Serial.println("[DELIVERY] Sender tag -> AT_SENDER (press ITEM LOADED)");
    }
  } else if (dState == D_TO_RECEIVER) {
    if (isJ1) startJunction(ROUTES.j1ToReceiver);
    else if (isReceiver) {
      dState = D_AT_RECEIVER; phase = PH_IDLE; driveStop();
      Serial.println("[DELIVERY] Receiver tag -> AT_RECEIVER (press RECEIVED)");
    }
  } else if (dState == D_RETURNING) {
    if (isJ1) startJunction(ROUTES.j1ToBase);
    else if (isBase) startBaseStop();
  }
}

void updateDelivery() {
  float e;
  switch (phase) {
    case PH_IDLE:
      readSensors(e);          // keep IR dots live
      driveStop();
      break;

    case PH_LINE:
      lineFollow();
      break;

    case PH_JWAIT:
      readSensors(e);
      driveStop();
      if (millis() - phaseStartMs >= JUNCTION_STOP_MS) {
        startTurn(pendingRoute, ROUTE_TURN_MIN_MS, PH_ROUTE);
      }
      break;

    case PH_BASEWAIT:
      readSensors(e);
      driveStop();
      if (millis() - phaseStartMs >= BASE_STOP_MS) {
        Serial.println("[DELIVERY] Base wait done -> 180 deg turn");
        startTurn(BASE_TURN_DIR, UTURN_MIN_MS, PH_BASETURN);
      }
      break;

    case PH_ROUTE:
    case PH_UTURN:
    case PH_BASETURN:
      runTurn();
      break;
  }
}

/* ================= RFID POLL ================= */
void updateRfid() {
  unsigned long now = millis();

  // Health check: chip answering and antenna ON?
  if (now - lastRfidHealthMs >= RFID_HEALTH_MS) {
    lastRfidHealthMs = now;
    bool wasOk = rfidOk;
    bool answers = rfidChipAnswers();

    if (answers && rfidConfigured()) {
      rfidOk = true;
    } else {
      if (wasOk || answers) rfidResets++;          // a real glitch (not "reader never connected")

      if (answers) {
        rfidConfig();                              // chip lost its settings: reload (fast, no delay)
      } else if (now - lastRfidFullInitMs >= RFID_FULL_INIT_GAP_MS) {
        rfidStart(false);                          // chip silent: full re-init (slow, rate limited)
      }
      rfidOk = rfidChipAnswers() && rfidConfigured();

      if ((wasOk || answers) && now - lastRfidLogMs >= RFID_LOG_GAP_MS) {
        lastRfidLogMs = now;
        Serial.printf("[RFID] reader glitch -> recovered=%d (total glitches: %lu)\n",
                      rfidOk ? 1 : 0, rfidResets);
      }
    }
  }

  if (!rfidOk) return;
  if (now - lastRfidPollMs < RFID_POLL_MS) return;
  lastRfidPollMs = now;

  if (!rfid.PICC_IsNewCardPresent()) return;
  if (!rfid.PICC_ReadCardSerial()) return;

  lastUid = uidToString(rfid.uid);
  lastTag = tagName(rfid.uid);
  rfid.PICC_HaltA();           // so the same tag is not read again while in the field
  rfid.PCD_StopCrypto1();

  Serial.print("[RFID] UID: ");
  Serial.print(lastUid);
  Serial.print("  (");
  Serial.print(lastTag);
  Serial.println(")");
  handleTag(rfid.uid);
}

/* ================= WEB PAGE ================= */
const char PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>ESP32 LineBot (5-Array)</title>
<style>
 *{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
 body{margin:0;font-family:system-ui,Arial;background:#101418;color:#e8eef2}
 header{padding:14px;text-align:center;background:#161d24;font-weight:600;letter-spacing:1px}
 .tabs{display:flex}
 .tab{flex:1;padding:14px 4px;text-align:center;background:#1b242d;cursor:pointer;font-weight:600;font-size:13px}
 .tab.on{background:#0aa06e;color:#fff}
 .panel{display:none;padding:18px}
 .panel.on{display:block}
 button{border:0;border-radius:14px;font-size:17px;font-weight:700;color:#fff;padding:18px;width:100%}
 button:disabled{opacity:.35}
 .go{background:#0aa06e}.stop{background:#d0342c}.dir{background:#2b6cb0;height:78px}
 .pad{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;max-width:340px;margin:auto}
 .sp{margin:18px 0}
 input[type=range]{width:100%}
 .sens{display:flex;gap:10px;justify-content:center;margin-top:16px}
 .dot{width:32px;height:32px;border-radius:8px;background:#2a3540}
 .dot.hit{background:#0aa06e}
 .st{text-align:center;margin-top:12px;font-size:14px;color:#8fa3b0}
 .range{text-align:center;padding:12px;background:#1b242d}
 .range.blocked{background:#942d2d;color:#fff}
 .rfid{text-align:center;padding:8px;background:#161d24;font-size:13px;color:#8fa3b0}
 .dbox{text-align:center;background:#1b242d;border-radius:14px;padding:14px;margin-bottom:18px}
 .dv{font-size:22px;font-weight:700;margin:6px 0;color:#0fd18f}
 .dl{font-size:13px;color:#8fa3b0}
</style></head><body>
<header>ESP32 4WD LINE BOT (5-ARRAY)</header>
<div class="tabs">
  <div class="tab on" id="t1" onclick="tab(1)">LINE FOLLOW</div>
  <div class="tab" id="t2" onclick="tab(2)">MANUAL</div>
  <div class="tab" id="t3" onclick="tab(3)">DELIVERY</div>
</div>
<div class="range" id="range">Ultrasonic: <b id="distance">--</b> cm | <b id="obstacle">CLEAR</b></div>
<div class="rfid">RFID: <b id="uid">--</b> <span id="rfidok"></span></div>

<div class="panel on" id="p1">
  <button class="go" onclick="send('/mode?m=LINE')">START LINE FOLLOWING</button>
  <div class="sp"></div>
  <button class="stop" onclick="send('/mode?m=STOP')">STOP</button>
  <div class="sp">Base Speed: <b id="bsv">150</b>
    <input type="range" min="60" max="255" value="150" oninput="bsv.innerText=this.value" onchange="send('/set?base='+this.value)">
  </div>
  <div class="sp">Kp: <b id="kpv">35</b>
    <input type="range" min="0" max="100" value="35" oninput="kpv.innerText=this.value" onchange="send('/set?kp='+this.value)">
  </div>
  <div class="sp">Kd: <b id="kdv">20</b>
    <input type="range" min="0" max="100" value="20" oninput="kdv.innerText=this.value" onchange="send('/set?kd='+this.value)">
  </div>
  <div class="sens" id="sens"></div>
  <div class="st" id="st">mode: --</div>
</div>

<div class="panel" id="p2">
  <div class="pad">
    <div></div>
    <button class="dir" onmousedown="mv('F')" ontouchstart="mv('F')" onmouseup="mv('S')" ontouchend="mv('S')">&#9650;</button>
    <div></div>
    <button class="dir" onmousedown="mv('L')" ontouchstart="mv('L')" onmouseup="mv('S')" ontouchend="mv('S')">&#9664;</button>
    <button class="stop" style="height:78px" onclick="mv('S')">&#9632;</button>
    <button class="dir" onmousedown="mv('R')" ontouchstart="mv('R')" onmouseup="mv('S')" ontouchend="mv('S')">&#9654;</button>
    <div></div>
    <button class="dir" onmousedown="mv('B')" ontouchstart="mv('B')" onmouseup="mv('S')" ontouchend="mv('S')">&#9660;</button>
    <div></div>
  </div>
  <div class="sp">Manual Speed: <b id="msv">180</b>
    <input type="range" min="60" max="255" value="180" oninput="msv.innerText=this.value" onchange="send('/set?man='+this.value)">
  </div>
</div>

<div class="panel" id="p3">
  <div class="dbox">
    <div class="dl">DELIVERY STATE</div>
    <div class="dv" id="dstate">--</div>
    <div class="dl" id="dphase">phase: --</div>
  </div>
  <button class="go" id="callBtn" onclick="send('/call')">CALL ROBOT</button>
  <div class="sp"></div>
  <button class="go" id="loadBtn" onclick="send('/loaded')">ITEM LOADED</button>
  <div class="sp"></div>
  <button class="go" id="recvBtn" onclick="send('/received')">RECEIVED</button>
  <div class="sp"></div>
  <button class="stop" onclick="send('/cancel')">CANCEL / RESET</button>
  <div class="sens" id="sens3"></div>
</div>

<script>
const D=id=>document.getElementById(id);
let box=D('sens'), box3=D('sens3');
// Generates exactly 5 visual sensor dots (line tab + delivery tab)
for(let i=0;i<5;i++){
  let d=document.createElement('div');d.className='dot';d.id='d'+i;box.appendChild(d);
  let d3=document.createElement('div');d3.className='dot';d3.id='e'+i;box3.appendChild(d3);
}
function tab(n){
  for(let i=1;i<=3;i++){
    D('t'+i).className='tab'+(n==i?' on':'');
    D('p'+i).className='panel'+(n==i?' on':'');
  }
  send('/mode?m='+['STOP','MANUAL','DELIVERY'][n-1]);
}
function send(u){fetch(u);}
function mv(c){fetch('/manual?c='+c);}
document.addEventListener('keydown',e=>{const k={ArrowUp:'F',ArrowDown:'B',ArrowLeft:'L',ArrowRight:'R'}[e.key];if(k)mv(k);});
document.addEventListener('keyup',e=>{if(e.key.startsWith('Arrow'))mv('S');});
setInterval(()=>{fetch('/status').then(r=>r.json()).then(j=>{
  for(let i=0;i<5;i++){
    const c='dot'+(j.s[i]?' hit':'');
    D('d'+i).className=c; D('e'+i).className=c;
  }
  D('st').innerText='mode: '+j.mode;
  D('distance').textContent=j.distanceCm===null?'--':j.distanceCm;
  D('obstacle').textContent=j.obstacle?'OBSTACLE - STOP':'CLEAR';
  D('range').classList.toggle('blocked',j.obstacle);
  D('uid').textContent=j.uid?(j.uid+' ('+j.tag+')'):'--';
  D('rfidok').textContent=(j.rfid?'':'(READER NOT RESPONDING) ')+(j.rfidResets>0?'glitches: '+j.rfidResets:'');
  D('dstate').textContent=j.dstate;
  D('dphase').textContent='phase: '+j.phase;
  const dm=(j.mode==='DELIVERY');
  D('callBtn').disabled=!(dm&&j.dstate==='STANDBY');
  D('loadBtn').disabled=!(dm&&j.dstate==='AT_SENDER');
  D('recvBtn').disabled=!(dm&&j.dstate==='AT_RECEIVER');
});},200);
</script></body></html>
)HTML";

/* ================= HANDLERS ================= */
void handleRoot() { server.send_P(200, "text/html", PAGE); }

void handleMode() {
  mode = server.arg("m");
  integral = 0; lastError = 0;
  resetDelivery();
  if (mode != "LINE" && mode != "MANUAL" && mode != "DELIVERY") { mode = "STOP"; driveStop(); }
  server.send(200, "text/plain", "ok");
}

void handleManual() {
  if (mode == "DELIVERY") {                 // manual keys must not break a delivery
    server.send(200, "text/plain", "ignored");
    return;
  }
  manualCmd = server.arg("c");
  mode = "MANUAL";
  server.send(200, "text/plain", "ok");
}

void handleSet() {
  if (server.hasArg("base")) baseSpeed   = server.arg("base").toInt();
  if (server.hasArg("man"))  manualSpeed = server.arg("man").toInt();
  if (server.hasArg("kp"))   Kp = server.arg("kp").toFloat();
  if (server.hasArg("kd"))   Kd = server.arg("kd").toFloat();
  server.send(200, "text/plain", "ok");
}

void handleCall() {
  if (mode == "DELIVERY" && dState == D_STANDBY) {
    integral = 0; lastError = 0; lastKnownError = 0;
    dState = D_TO_SENDER;
    phase = PH_LINE;
    Serial.println("[DELIVERY] CALL ROBOT -> GOING_TO_SENDER");
    server.send(200, "text/plain", "ok");
  } else {
    server.send(200, "text/plain", "ignored");
  }
}

void handleLoaded() {
  if (mode == "DELIVERY" && dState == D_AT_SENDER) {
    dState = D_TO_RECEIVER;
    startTurn(UTURN_DIR, UTURN_MIN_MS, PH_UTURN);
    Serial.println("[DELIVERY] ITEM LOADED -> 180 deg turn -> GOING_TO_RECEIVER");
    server.send(200, "text/plain", "ok");
  } else {
    server.send(200, "text/plain", "ignored");
  }
}

void handleReceived() {
  if (mode == "DELIVERY" && dState == D_AT_RECEIVER) {
    dState = D_RETURNING;
    startTurn(UTURN_DIR, UTURN_MIN_MS, PH_UTURN);
    Serial.println("[DELIVERY] RECEIVED -> 180 deg turn -> RETURNING_TO_BASE");
    server.send(200, "text/plain", "ok");
  } else {
    server.send(200, "text/plain", "ignored");
  }
}

void handleCancel() {
  resetDelivery();
  Serial.println("[DELIVERY] CANCEL -> STANDBY");
  server.send(200, "text/plain", "ok");
}

void handleStatus() {
  String j = "{\"mode\":\"" + mode + "\",\"s\":[";
  for (int i = 0; i < 5; i++) {
    j += String(irVal[i]);
    if (i < 4) j += ",";
  }
  j += "],\"distanceCm\":";
  if (distanceCm < 0) j += "null";
  else j += String(distanceCm, 1);
  j += ",\"obstacle\":";
  j += (obstacleDetected ? "true" : "false");
  j += ",\"dstate\":\"";
  j += DSTATE_NAME[dState];
  j += "\",\"phase\":\"";
  j += PHASE_NAME[phase];
  j += "\",\"uid\":\"";
  j += lastUid;
  j += "\",\"tag\":\"";
  j += lastTag;
  j += "\",\"rfid\":";
  j += (rfidOk ? "true" : "false");
  j += ",\"rfidResets\":";
  j += String(rfidResets);
  j += "}";
  server.send(200, "application/json", j);
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);

  pinMode(WIFI_LED_PIN, OUTPUT);
  digitalWrite(WIFI_LED_PIN, LOW);

  // Configure 5 IR pins as inputs
  for (int i = 0; i < 5; i++) pinMode(irPin[i], INPUT);

  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoISR, CHANGE);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  lastUltrasonicMs = millis() - ULTRASONIC_INTERVAL_MS;

  // Core 3.x PWM configuration
  ledcAttach(INA, PWM_FREQ, PWM_RES);
  ledcAttach(INB, PWM_FREQ, PWM_RES);
  ledcAttach(INC, PWM_FREQ, PWM_RES);
  ledcAttach(IND, PWM_FREQ, PWM_RES);
  driveStop();

  // RFID on custom SPI pins
  SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_SDA);
  rfidStart(true);

  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("Access Point started! IP: ");
  Serial.println(WiFi.softAPIP()); // 192.168.4.1

  server.on("/", handleRoot);
  server.on("/mode", handleMode);
  server.on("/manual", handleManual);
  server.on("/set", handleSet);
  server.on("/status", handleStatus);
  server.on("/call", handleCall);
  server.on("/loaded", handleLoaded);
  server.on("/received", handleReceived);
  server.on("/cancel", handleCancel);
  server.begin();
}

/* ================= LOOP ================= */
void loop() {
  server.handleClient();
  updateUltrasonic();
  updateWifiLed();
  updateRfid();

  if (obstacleDetected) {
    float e;
    readSensors(e); // Keep the existing IR dots live while stopped
    driveStop();
    return;
  }

  if (mode == "LINE")          lineFollow();
  else if (mode == "MANUAL")   manualDrive();
  else if (mode == "DELIVERY") updateDelivery();
  else {
    float e;
    readSensors(e); // Updates live UI dots while robot is stopped
    driveStop();
  }
}