// -------------------------------------------------------------
// ESP32 Buzzer Test Code
// -------------------------------------------------------------

const int BUZZER_PIN = 23; // Connected to GPIO 23

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  Serial.println("Buzzer test starting...");
}

void loop() {
  // Beep 1: Short beep
  Serial.println("Beep!");
  digitalWrite(BUZZER_PIN, HIGH); // Turn buzzer ON
  delay(200);                     // Wait 200 ms
  digitalWrite(BUZZER_PIN, LOW);  // Turn buzzer OFF
  delay(200);

  // Beep 2: Short beep
  Serial.println("Beep!");
  digitalWrite(BUZZER_PIN, HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN, LOW);
  delay(200);

  // Beep 3: Long beep
  Serial.println("Long Beep!");
  digitalWrite(BUZZER_PIN, HIGH);
  delay(800);                     // Wait 800 ms
  digitalWrite(BUZZER_PIN, LOW);

  // Pause before repeating
  Serial.println("Waiting 2 seconds...\n");
  delay(2000);
}