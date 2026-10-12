/*
 * ESP32 4WD Line Follower + Manual Control (5-Array IR Version)
 * -----------------------------------------------------------
 * Motor driver: L298N board (Direct PWM on INA/INB/INC/IND)
 * Sensor: 5-channel IR array (digital out)
 * WiFi AP: LineBot_ESP32 / 12345678 -> http://192.168.4.1
 * Requires ESP32 Arduino core 3.x
 */

#include <WiFi.h>
#include <WebServer.h>

/* ================= WIFI ================= */
const char* AP_SSID = "LineBot_ESP32";
const char* AP_PASS = "12345678";
WebServer server(80);

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

#define PWM_FREQ 5000
#define PWM_RES  8      // 0-255

/* If one side spins the wrong way, flip its flag to true */
bool INVERT_LEFT  = false;
bool INVERT_RIGHT = false;

/* ============== CONTROL VARIABLES ============== */
String mode = "STOP";          // LINE / MANUAL / STOP
String manualCmd = "S";        // F B L R S
int baseSpeed   = 150;
int manualSpeed = 180;

// PID constants tuned for 5-channel array
float Kp = 35.0, Ki = 0.0, Kd = 20.0;
float lastError = 0, integral = 0;
float lastKnownError = 0;

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

/* ================= SENSOR READ (5-ARRAY) ================= */
// S1: -4 (far left), S2: -2 (mid left), S3: 0 (center), S4: +2 (mid right), S5: +4 (far right)
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
    else                    { leftMotor(-s);  rightMotor(s); }
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
 .tab{flex:1;padding:14px;text-align:center;background:#1b242d;cursor:pointer;font-weight:600}
 .tab.on{background:#0aa06e;color:#fff}
 .panel{display:none;padding:18px}
 .panel.on{display:block}
 button{border:0;border-radius:14px;font-size:17px;font-weight:700;color:#fff;padding:18px;width:100%}
 .go{background:#0aa06e}.stop{background:#d0342c}.dir{background:#2b6cb0;height:78px}
 .pad{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;max-width:340px;margin:auto}
 .sp{margin:18px 0}
 input[type=range]{width:100%}
 .sens{display:flex;gap:10px;justify-content:center;margin-top:16px}
 .dot{width:32px;height:32px;border-radius:8px;background:#2a3540}
 .dot.hit{background:#0aa06e}
 .st{text-align:center;margin-top:12px;font-size:14px;color:#8fa3b0}
</style></head><body>
<header>ESP32 4WD LINE BOT (5-ARRAY)</header>
<div class="tabs">
  <div class="tab on" id="t1" onclick="tab(1)">LINE FOLLOW</div>
  <div class="tab" id="t2" onclick="tab(2)">MANUAL</div>
</div>

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

<script>
let box=document.getElementById('sens');
// Generates exactly 5 visual sensor dots
for(let i=0;i<5;i++){let d=document.createElement('div');d.className='dot';d.id='d'+i;box.appendChild(d);}
function tab(n){
  t1.className='tab'+(n==1?' on':'');t2.className='tab'+(n==2?' on':'');
  p1.className='panel'+(n==1?' on':'');p2.className='panel'+(n==2?' on':'');
  send('/mode?m='+(n==1?'STOP':'MANUAL'));
}
function send(u){fetch(u);}
function mv(c){fetch('/manual?c='+c);}
document.addEventListener('keydown',e=>{const k={ArrowUp:'F',ArrowDown:'B',ArrowLeft:'L',ArrowRight:'R'}[e.key];if(k)mv(k);});
document.addEventListener('keyup',e=>{if(e.key.startsWith('Arrow'))mv('S');});
setInterval(()=>{fetch('/status').then(r=>r.json()).then(j=>{
  for(let i=0;i<5;i++)document.getElementById('d'+i).className='dot'+(j.s[i]?' hit':'');
  st.innerText='mode: '+j.mode;
});},200);
</script></body></html>
)HTML";

/* ================= HANDLERS ================= */
void handleRoot() { server.send_P(200, "text/html", PAGE); }

void handleMode() {
  mode = server.arg("m");
  integral = 0; lastError = 0;
  if (mode != "LINE" && mode != "MANUAL") { mode = "STOP"; driveStop(); }
  server.send(200, "text/plain", "ok");
}

void handleManual() {
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

void handleStatus() {
  String j = "{\"mode\":\"" + mode + "\",\"s\":[";
  for (int i = 0; i < 5; i++) { 
    j += String(irVal[i]); 
    if (i < 4) j += ","; 
  }
  j += "]}";
  server.send(200, "application/json", j);
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);

  // Configure 5 IR pins as inputs
  for (int i = 0; i < 5; i++) pinMode(irPin[i], INPUT);

  // Core 3.x PWM configuration
  ledcAttach(INA, PWM_FREQ, PWM_RES);
  ledcAttach(INB, PWM_FREQ, PWM_RES);
  ledcAttach(INC, PWM_FREQ, PWM_RES);
  ledcAttach(IND, PWM_FREQ, PWM_RES);
  driveStop();

  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("Access Point started! IP: ");
  Serial.println(WiFi.softAPIP()); // 192.168.4.1

  server.on("/", handleRoot);
  server.on("/mode", handleMode);
  server.on("/manual", handleManual);
  server.on("/set", handleSet);
  server.on("/status", handleStatus);
  server.begin();
}

/* ================= LOOP ================= */
void loop() {
  server.handleClient();

  if (mode == "LINE")        lineFollow();
  else if (mode == "MANUAL") manualDrive();
  else {
    float e; 
    readSensors(e); // Updates live UI dots while robot is stopped
    driveStop();
  }
}