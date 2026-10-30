/*
 * ESP32 4WD Line Follower + Manual Control + RFID Luggage Delivery (5-Array IR Version)
 * -----------------------------------------------------------------------------
 * Motor driver: L298N board (Direct PWM on INA/INB/INC/IND)
 * Sensor: 5-channel IR array (digital out) + HC-SR04 ultrasonic + 2 x RC522 RFID
 * Buzzer: active HIGH, 3.3 V compatible
 * WiFi LED: blinks while waiting, solid ON when a client is connected
 * WiFi AP: LineBott_ESP32 / 12345678 -> http://192.168.4.1
 * Requires ESP32 Arduino core 3.x + "MFRC522" library
 *
 * TWO RFID READERS share SCK / MOSI / MISO, only SDA (SS) differs:
 *   Reader 1 SDA -> GPIO 2
 *   Reader 2 SDA -> GPIO 13
 *   Both readers do exactly the same job (a tag read by either one is handled the same way)
 *   RFID_ALTERNATE = 1: only ONE antenna is ON at a time (readers take turns)
 *
 * 6 RFID TAGS
 *   Floor : BASE, J1 (junction), ROOM1 (in front of Room 1), ROOM2 (in front of Room 2)
 *   Customer cards : CARD_ROOM1, CARD_ROOM2
 *
 * DELIVERY FLOW (DELIVERY tab must be open)
 *   STANDBY            robot at base, waiting for a customer card
 *     | CARD_ROOM1 or CARD_ROOM2 touched
 *   GOING_TO_ROOMx     line follow --(J1 tag)--> stop 3 s -> turn (Room1 = R, Room2 = L)
 *     | ROOMx floor tag
 *   AT_ROOMx           motors off, waiting
 *     | same customer card touched again  OR  CALL ROBOT on web page
 *     | 180 deg turn
 *   RETURNING_TO_BASE  line follow --(J1 tag)--> stop 3 s -> turn (from Room1 = L, from Room2 = R)
 *     | BASE floor tag
 *     | stop 1 s -> 180 deg turn
 *   STANDBY            (robot faces the junction again, ready for next customer)
 *
 *   Ignored: unknown card, wrong room card, any card while the robot is moving.
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

/* ============== RFID (2 x RC522) ============== */
#define RFID_SCK  21
#define RFID_MOSI 12
#define RFID_MISO 36
#define RFID_SDA  2               // Reader 1: SS / chip select
#define RFID_SDA2 13              // Reader 2: SS / chip select
#define RFID_RST  255             // RST not connected to ESP32 (wire RST to 3.3V on both modules!)
#define RFID_FAST_TIMEOUT 1       // 1 = shorter wait when no tag (less loop delay)
#define RFID_MAX_GAIN     1       // 1 = maximum receiver gain (easier to read while moving)
#define RFID_COUNT        2
#define RFID_DIAG         1       // 1 = print reader status to Serial every 3 s
#define RFID_ALTERNATE    1       // 1 = readers take turns (one antenna ON at a time)
                                  // 0 = both antennas always ON (old behaviour)

const unsigned long RFID_SLOT_MS   = 20;     // how long one reader owns the field
const unsigned long RFID_SETTLE_MS = 4;      // wait after antenna ON before reading
const unsigned long RFID_REPEAT_MS = 1500;   // same tag seen again within this gap = same visit

/* Everything one reader needs (its own chip object + its own health/poll state) */
struct RfidReader {
  MFRC522 dev;
  bool ok;
  byte ver;                       // last VersionReg value (0x00 / 0xFF = chip not answering)
  unsigned long reads;            // tags fully read by this reader
  unsigned long seen;             // times a tag answered (even if UID read failed)
  unsigned long fails;            // tag answered but UID read failed
  unsigned long lastPollMs;
  unsigned long lastHealthMs;
  unsigned long lastFullInitMs;
  unsigned long lastLogMs;
  unsigned long resets;           // number of glitches seen on this reader
  RfidReader(byte ss, byte rst)
    : dev(ss, rst), ok(false), ver(0), reads(0), seen(0), fails(0), lastPollMs(0),
      lastHealthMs(0), lastFullInitMs(0), lastLogMs(0), resets(0) {}
};

RfidReader readers[RFID_COUNT] = {
  RfidReader(RFID_SDA,  RFID_RST),
  RfidReader(RFID_SDA2, RFID_RST)
};

String lastUid = "";
const char* lastTag = "";
int lastReader = 0;                                 // 1 or 2 (0 = no tag read yet)
const unsigned long RFID_POLL_MS = 30;
const unsigned long RFID_DIAG_MS = 3000;
unsigned long lastRfidDiagMs = 0;

/* Alternate-mode bookkeeping */
int rfidActive = 0;                                 // reader that owns the field right now
unsigned long rfidSlotMs = 0;
bool rfidPolled = true;
String rfidSeenUid = "";                            // repeat filter
unsigned long rfidSeenMs = 0;

/* RC522 can glitch when motors run (power dip / noise). Detect it and recover quickly. */
const unsigned long RFID_HEALTH_MS        = 150;    // health check interval
const unsigned long RFID_FULL_INIT_GAP_MS = 2000;   // slow full re-init at most this often
const unsigned long RFID_LOG_GAP_MS       = 1000;   // Serial glitch message at most this often

/* ---- Tag UIDs ---- */
struct RfidUid {
  byte size;
  byte bytes[10];
};
// Floor tags
const RfidUid UID_BASE       = {4, {0xC1, 0x0D, 0xBB, 0x0A}};   // base station
const RfidUid UID_J1         = {4, {0x64, 0xD5, 0x30, 0x03}};   // junction J1
const RfidUid UID_ROOM1      = {4, {0xAC, 0x71, 0x8F, 0x02}};   // in front of Room 1
const RfidUid UID_ROOM2      = {4, {0x09, 0xB2, 0x30, 0x03}};   // in front of Room 2
// Customer cards
const RfidUid UID_CARD_ROOM1 = {4, {0xE9, 0x8F, 0x36, 0x03}};   // customer card -> Room 1
const RfidUid UID_CARD_ROOM2 = {4, {0xBC, 0x74, 0xC1, 0x01}};   // customer card -> Room 2

/* What to do at J1: 'S' = straight, 'L' = left, 'R' = right
   If the robot turns the wrong way on your track, swap 'L' and 'R' here. */
struct Routes {
  char baseToRoom1;
  char baseToRoom2;
  char room1ToBase;
  char room2ToBase;
};
const Routes ROUTES = { 'R', 'L', 'L', 'R' };

/* ============== DELIVERY TIMING (tune these) ============== */
const unsigned long JUNCTION_STOP_MS    = 3000;  // stop at J1
const unsigned long STRAIGHT_CLEAR_MS   = 400;   // drive straight to clear J1
const int           TURN_SPEED          = 255;   // spin speed for turns (MAX)
const unsigned long ROUTE_TURN_MIN_MS   = 250;   // min spin time for L/R at J1
const unsigned long UTURN_MIN_MS        = 350;   // min spin time for 180 deg turn
const unsigned long TURN_MAX_MS         = 3500;  // give up waiting for the line
const char          UTURN_DIR           = 'R';   // 180 deg turn direction

/* Base (RFID tag): stop, wait, 180 deg turn, then STANDBY */
const unsigned long BASE_STOP_MS        = 1000;  // wait at base before turning
const int           BASE_TURN_SPEED     = 255;   // spin speed at base (MAX)
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
int baseSpeed   = 255;         // line follow + delivery speed (MAX)
int manualSpeed = 255;         // manual mode speed (MAX)

// PID constants tuned for 5-channel array
float Kp = 35.0, Ki = 0.0, Kd = 20.0;
float lastError = 0, integral = 0;
float lastKnownError = 0;

/* ============== DELIVERY STATE ============== */
enum DState { D_STANDBY, D_GOING, D_AT_ROOM, D_RETURNING };
enum Phase  { PH_IDLE, PH_LINE, PH_JWAIT, PH_ROUTE, PH_UTURN, PH_BASEWAIT, PH_BASETURN };

const char* const PHASE_NAME[] = {
  "IDLE", "LINE", "JUNCTION_WAIT", "ROUTE_TURN", "U_TURN", "BASE_WAIT", "BASE_TURN"
};

DState dState = D_STANDBY;
Phase  phase  = PH_IDLE;
int    targetRoom = 0;              // 0 = none, 1 = Room 1, 2 = Room 2
bool   j1Done = false;              // J1 already handled on this trip leg

unsigned long phaseStartMs = 0;     // start of junction / base wait
char pendingRoute = 'S';            // route to take after junction wait

char turnDir = 'S';                 // 'L' / 'R' / 'S'
int  turnStage = 0;                 // 0 = leaving old line, 1 = looking for line
unsigned long turnStartMs = 0;
unsigned long turnMinMs = 0;

String dStateName() {
  switch (dState) {
    case D_STANDBY:   return String("STANDBY");
    case D_GOING:     return String("GOING_TO_ROOM") + targetRoom;
    case D_AT_ROOM:   return String("AT_ROOM") + targetRoom;
    case D_RETURNING: return String("RETURNING_TO_BASE");
  }
  return String("?");
}

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
  if (uidMatches(u, UID_BASE))       return "BASE";
  if (uidMatches(u, UID_J1))         return "J1";
  if (uidMatches(u, UID_ROOM1))      return "ROOM1";
  if (uidMatches(u, UID_ROOM2))      return "ROOM2";
  if (uidMatches(u, UID_CARD_ROOM1)) return "CARD_ROOM1";
  if (uidMatches(u, UID_CARD_ROOM2)) return "CARD_ROOM2";
  return "UNKNOWN";
}

// Should this reader's antenna be ON right now?
bool rfidAntennaWanted(int idx) {
#if RFID_ALTERNATE
  return idx == rfidActive;
#else
  (void)idx;
  return true;
#endif
}

// Same register setup as MFRC522::PCD_Init(), but WITHOUT the 50+ ms soft reset delay.
// Used for fast recovery when the chip lost its settings (reset by noise / power dip).
void rfidConfig(MFRC522 &d) {
  d.PCD_WriteRegister(MFRC522::TModeReg, 0x80);        // timer starts automatically
  d.PCD_WriteRegister(MFRC522::TPrescalerReg, 0xA9);
#if RFID_FAST_TIMEOUT
  d.PCD_WriteRegister(MFRC522::TReloadRegH, 0x01);     // reload 400 -> ~10 ms no-tag timeout
  d.PCD_WriteRegister(MFRC522::TReloadRegL, 0x90);
#else
  d.PCD_WriteRegister(MFRC522::TReloadRegH, 0x03);     // reload 1000 -> ~25 ms (library default)
  d.PCD_WriteRegister(MFRC522::TReloadRegL, 0xE8);
#endif
  d.PCD_WriteRegister(MFRC522::TxASKReg, 0x40);        // 100 % ASK modulation
  d.PCD_WriteRegister(MFRC522::ModeReg, 0x3D);         // CRC preset 0x6363
#if RFID_MAX_GAIN
  d.PCD_SetAntennaGain(MFRC522::RxGain_max);
#endif
  d.PCD_AntennaOn();
}

// Does the chip answer on SPI at all? (0x00 / 0xFF means no answer)
bool rfidChipAnswers(MFRC522 &d) {
  byte ver = d.PCD_ReadRegister(MFRC522::VersionReg);
  return (ver != 0x00 && ver != 0xFF);
}

// Is the antenna ON? (after a chip reset the antenna drivers are OFF)
bool rfidConfigured(MFRC522 &d) {
  return (d.PCD_ReadRegister(MFRC522::TxControlReg) & 0x03) == 0x03;
}

// Full start: soft reset (slow, ~50 ms) + our settings
void rfidStart(int idx, bool verbose) {
  RfidReader &r = readers[idx];
  bool wanted = rfidAntennaWanted(idx);
  r.dev.PCD_Init();
  rfidConfig(r.dev);
  if (!wanted) r.dev.PCD_AntennaOff();
  r.lastFullInitMs = millis();
  byte ver = r.dev.PCD_ReadRegister(MFRC522::VersionReg);
  r.ver = ver;
  r.ok = (ver != 0x00 && ver != 0xFF) && (rfidConfigured(r.dev) || !wanted);
  if (verbose) {
    Serial.printf("RFID reader %d version register: 0x%02X\n", idx + 1, ver);
    if (r.ok) Serial.printf("RFID reader %d OK\n", idx + 1);
    else      Serial.printf("RFID reader %d NOT FOUND - check wiring (3.3V, GND, SCK, MOSI, MISO, SDA, RST->3.3V)\n", idx + 1);
  }
}

// Give the field to one reader, switch the others OFF
void rfidSwitchTo(int idx) {
  rfidActive = idx;
  for (int i = 0; i < RFID_COUNT; i++) {
    if (i == idx) readers[i].dev.PCD_AntennaOn();
    else          readers[i].dev.PCD_AntennaOff();
  }
}

unsigned long rfidTotalResets() {
  unsigned long t = 0;
  for (int i = 0; i < RFID_COUNT; i++) t += readers[i].resets;
  return t;
}

/* ================= DELIVERY ================= */
void resetDelivery() {
  driveStop();
  dState = D_STANDBY;
  phase = PH_IDLE;
  targetRoom = 0;
  j1Done = false;
}

void endDelivery() {
  resetDelivery();
  Serial.println("[DELIVERY] Base turn done -> STANDBY (waiting for customer card)");
}

void clearPid() {
  integral = 0;
  lastError = 0;
  lastKnownError = 0;
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
  clearPid();
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
  j1Done = true;
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

// Customer card at base: go to that room
void startGoing(int room) {
  targetRoom = room;
  dState = D_GOING;
  j1Done = false;
  clearPid();
  phase = PH_LINE;
  Serial.printf("[DELIVERY] Customer card -> GOING_TO_ROOM%d\n", room);
}

// Same card again or CALL ROBOT: 180 deg turn, then back to base
void startReturn(const char* why) {
  dState = D_RETURNING;
  j1Done = false;
  startTurn(UTURN_DIR, UTURN_MIN_MS, PH_UTURN);
  Serial.printf("[DELIVERY] %s -> 180 deg turn -> RETURNING_TO_BASE\n", why);
}

// Called for every new RFID tag
void handleTag(const MFRC522::Uid &u) {
  if (mode != "DELIVERY") return;

  bool isCard1 = uidMatches(u, UID_CARD_ROOM1);
  bool isCard2 = uidMatches(u, UID_CARD_ROOM2);

  /* ---- Customer cards ---- */
  if (isCard1 || isCard2) {
    int room = isCard1 ? 1 : 2;
    if (dState == D_STANDBY && phase == PH_IDLE) {
      startGoing(room);
    } else if (dState == D_AT_ROOM && phase == PH_IDLE) {
      if (room == targetRoom) startReturn("Same customer card");
      else Serial.println("[DELIVERY] Wrong room card -> ignored");
    } else {
      Serial.println("[DELIVERY] Robot is moving -> card ignored");
    }
    return;
  }

  /* ---- Floor tags (only while following the line) ---- */
  if (phase != PH_LINE) return;

  bool isJ1    = uidMatches(u, UID_J1);
  bool isBase  = uidMatches(u, UID_BASE);
  bool isRoom1 = uidMatches(u, UID_ROOM1);
  bool isRoom2 = uidMatches(u, UID_ROOM2);

  if (dState == D_GOING) {
    if (isJ1 && !j1Done) {
      startJunction(targetRoom == 1 ? ROUTES.baseToRoom1 : ROUTES.baseToRoom2);
    } else if ((targetRoom == 1 && isRoom1) || (targetRoom == 2 && isRoom2)) {
      driveStop();
      dState = D_AT_ROOM;
      phase = PH_IDLE;
      Serial.printf("[DELIVERY] Room %d tag -> AT_ROOM%d (touch same card or CALL ROBOT)\n",
                    targetRoom, targetRoom);
    }
  } else if (dState == D_RETURNING) {
    if (isJ1 && !j1Done) {
      startJunction(targetRoom == 1 ? ROUTES.room1ToBase : ROUTES.room2ToBase);
    } else if (isBase) {
      startBaseStop();
    }
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

/* ================= RFID HEALTH (one reader) ================= */
void rfidHealth(int idx) {
  RfidReader &r = readers[idx];
  unsigned long now = millis();
  if (now - r.lastHealthMs < RFID_HEALTH_MS) return;
  r.lastHealthMs = now;

  bool wasOk = r.ok;
  bool wanted = rfidAntennaWanted(idx);
  r.ver = r.dev.PCD_ReadRegister(MFRC522::VersionReg);
  bool answers = (r.ver != 0x00 && r.ver != 0xFF);

  // Healthy: chip answers and (antenna ON, or it is not this reader's turn)
  if (answers && (rfidConfigured(r.dev) || !wanted)) {
    r.ok = true;
    return;
  }

  if (wasOk || answers) r.resets++;              // a real glitch (not "reader never connected")

  if (answers) {
    rfidConfig(r.dev);                           // chip lost its settings: reload (fast, no delay)
    if (!wanted) r.dev.PCD_AntennaOff();
  } else if (now - r.lastFullInitMs >= RFID_FULL_INIT_GAP_MS) {
    rfidStart(idx, false);                       // chip silent: full re-init (slow, rate limited)
  }
  r.ok = rfidChipAnswers(r.dev) && (rfidConfigured(r.dev) || !wanted);

  if ((wasOk || answers) && now - r.lastLogMs >= RFID_LOG_GAP_MS) {
    r.lastLogMs = now;
    Serial.printf("[RFID %d] reader glitch -> recovered=%d (glitches on this reader: %lu)\n",
                  idx + 1, r.ok ? 1 : 0, r.resets);
  }
}

/* ================= RFID POLL (one reader) ================= */
void rfidPoll(int idx, bool rateLimit) {
  RfidReader &r = readers[idx];
  unsigned long now = millis();

  if (!r.ok) return;
  if (rateLimit) {
    if (now - r.lastPollMs < RFID_POLL_MS) return;
    r.lastPollMs = now;
  }

  if (!r.dev.PICC_IsNewCardPresent()) return;
  r.seen++;
  if (!r.dev.PICC_ReadCardSerial()) { r.fails++; return; }

  r.reads++;
  String uidStr = uidToString(r.dev.uid);
  const char* name = tagName(r.dev.uid);
  lastUid = uidStr;
  lastTag = name;
  lastReader = idx + 1;

  // Copy the UID before the tag is halted (handleTag needs it)
  MFRC522::Uid uidCopy = r.dev.uid;
  r.dev.PICC_HaltA();           // so the same tag is not read again while in the field
  r.dev.PCD_StopCrypto1();

#if RFID_ALTERNATE
  // The field is switched OFF/ON between turns, so a tag sitting on a reader is "new" again each time.
  // Ignore the same tag while it keeps being seen (gap < RFID_REPEAT_MS) = same visit.
  bool sameVisit = (uidStr == rfidSeenUid) && (now - rfidSeenMs < RFID_REPEAT_MS);
  rfidSeenUid = uidStr;
  rfidSeenMs = now;
  if (sameVisit) return;
#endif

  Serial.printf("[RFID %d] UID: ", idx + 1);
  Serial.print(uidStr);
  Serial.print("  (");
  Serial.print(name);
  Serial.println(")");
  handleTag(uidCopy);
}

/* ================= RFID UPDATE (all readers) ================= */
void updateRfid() {
  for (int i = 0; i < RFID_COUNT; i++) rfidHealth(i);

#if RFID_ALTERNATE
  unsigned long now = millis();
  // Hand the field to the next reader every RFID_SLOT_MS
  if (now - rfidSlotMs >= RFID_SLOT_MS) {
    rfidSlotMs = now;
    rfidSwitchTo((rfidActive + 1) % RFID_COUNT);
    rfidPolled = false;
  }
  // Let the antenna settle, then read once per turn
  if (!rfidPolled && now - rfidSlotMs >= RFID_SETTLE_MS) {
    rfidPolled = true;
    rfidPoll(rfidActive, false);
  }
#else
  for (int i = 0; i < RFID_COUNT; i++) rfidPoll(i, true);
#endif

#if RFID_DIAG
  // ver 0x91/0x92 (or clone 0x82 etc.) = chip talking, 0x00/0xFF = no answer on SPI
  // seen = a tag answered, reads = UID read OK, fail = tag answered but UID read failed
  if (millis() - lastRfidDiagMs >= RFID_DIAG_MS) {
    lastRfidDiagMs = millis();
    Serial.printf("[RFID DIAG] R1 ok=%d ver=0x%02X seen=%lu reads=%lu fail=%lu | R2 ok=%d ver=0x%02X seen=%lu reads=%lu fail=%lu\n",
                  readers[0].ok ? 1 : 0, readers[0].ver, readers[0].seen, readers[0].reads, readers[0].fails,
                  readers[1].ok ? 1 : 0, readers[1].ver, readers[1].seen, readers[1].reads, readers[1].fails);
  }
#endif
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
 .hint{text-align:center;margin-top:14px;font-size:14px;color:#8fa3b0}
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
  <div class="sp">Base Speed: <b id="bsv">255</b>
    <input type="range" min="60" max="255" value="255" oninput="bsv.innerText=this.value" onchange="send('/set?base='+this.value)">
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
  <div class="sp">Manual Speed: <b id="msv">255</b>
    <input type="range" min="60" max="255" value="255" oninput="msv.innerText=this.value" onchange="send('/set?man='+this.value)">
  </div>
</div>

<div class="panel" id="p3">
  <div class="dbox">
    <div class="dl">DELIVERY STATE</div>
    <div class="dv" id="dstate">--</div>
    <div class="dl" id="dphase">phase: --</div>
    <div class="dl" id="dtarget">target: --</div>
  </div>
  <button class="go" id="callBtn" onclick="send('/call')">CALL ROBOT</button>
  <div class="sp"></div>
  <button class="stop" onclick="send('/cancel')">CANCEL / RESET</button>
  <div class="hint" id="dhint">--</div>
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
  D('uid').textContent=j.uid?(j.uid+' ('+j.tag+' / R'+j.reader+')'):'--';
  D('rfidok').textContent='R1:'+(j.rfid?'OK':'NO')+'('+j.reads1+'/'+j.seen1+') R2:'+(j.rfid2?'OK':'NO')+'('+j.reads2+'/'+j.seen2+') '+(j.rfidResets>0?'glitches: '+j.rfidResets:'');
  D('dstate').textContent=j.dstate;
  D('dphase').textContent='phase: '+j.phase;
  D('dtarget').textContent='target: '+(j.target?'Room '+j.target:'--');
  const dm=(j.mode==='DELIVERY');
  const atRoom=dm&&j.dstate.startsWith('AT_ROOM')&&j.phase==='IDLE';
  D('callBtn').disabled=!atRoom;
  let h='Open DELIVERY tab to start';
  if(dm){
    if(j.dstate==='STANDBY') h='Customer: touch your room card';
    else if(j.dstate.startsWith('GOING')) h='Going to Room '+j.target;
    else if(j.dstate.startsWith('AT_ROOM')) h='Touch same card or press CALL ROBOT';
    else if(j.dstate==='RETURNING_TO_BASE') h='Returning to base';
  }
  D('dhint').textContent=h;
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

// CALL ROBOT: bring the robot back from the room (if customer forgot the card)
void handleCall() {
  if (mode == "DELIVERY" && dState == D_AT_ROOM && phase == PH_IDLE) {
    startReturn("CALL ROBOT");
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
  j += dStateName();
  j += "\",\"phase\":\"";
  j += PHASE_NAME[phase];
  j += "\",\"target\":";
  j += String(targetRoom);
  j += ",\"uid\":\"";
  j += lastUid;
  j += "\",\"tag\":\"";
  j += lastTag;
  j += "\",\"reader\":";
  j += String(lastReader);
  j += ",\"rfid\":";
  j += (readers[0].ok ? "true" : "false");
  j += ",\"rfid2\":";
  j += (readers[1].ok ? "true" : "false");
  j += ",\"reads1\":";
  j += String(readers[0].reads);
  j += ",\"reads2\":";
  j += String(readers[1].reads);
  j += ",\"seen1\":";
  j += String(readers[0].seen);
  j += ",\"seen2\":";
  j += String(readers[1].seen);
  j += ",\"rfidResets\":";
  j += String(rfidTotalResets());
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

  // Both RFID chip-select lines HIGH (deselected) BEFORE the bus is used,
  // so only one reader ever talks on the shared MISO line
  pinMode(RFID_SDA, OUTPUT);
  digitalWrite(RFID_SDA, HIGH);
  pinMode(RFID_SDA2, OUTPUT);
  digitalWrite(RFID_SDA2, HIGH);

  // RFID on custom SPI pins (shared by both readers).
  // SS = -1: neither SDA pin is claimed as a hardware chip-select, the library controls both manually.
  SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, -1);
  rfidActive = 0;
  for (int i = 0; i < RFID_COUNT; i++) rfidStart(i, true);
#if RFID_ALTERNATE
  rfidSwitchTo(0);
  rfidSlotMs = millis();
  rfidPolled = true;
#endif

  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("Access Point started! IP: ");
  Serial.println(WiFi.softAPIP()); // 192.168.4.1

  server.on("/", handleRoot);
  server.on("/mode", handleMode);
  server.on("/manual", handleManual);
  server.on("/set", handleSet);
  server.on("/status", handleStatus);
  server.on("/call", handleCall);
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