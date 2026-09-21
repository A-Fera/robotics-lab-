// -------------------------------------------------------------
// 5-Array IR Sensor Test Code for ESP32
// -------------------------------------------------------------

// Define the 5 sensor pins (Left to Right)
const int IR_PINS[5] = {14, 27, 26, 25, 33};

void setup() {
  Serial.begin(115200);

  // Set all 5 sensor pins as INPUT
  for (int i = 0; i < 5; i++) {
    pinMode(IR_PINS[i], INPUT);
  }

  Serial.println("=================================================");
  Serial.println("     5-Array IR Sensor Serial Monitor Test       ");
  Serial.println("=================================================");
  Serial.println(" [S1]    [S2]    [S3]    [S4]    [S5]   --> Visual");
  Serial.println(" (FarL)  (MidL)  (Cent)  (MidR)  (FarR)           ");
  Serial.println("-------------------------------------------------");
  delay(1000);
}

void loop() {
  int sensorValues[5];

  // 1. Read all 5 sensors
  for (int i = 0; i < 5; i++) {
    sensorValues[i] = digitalRead(IR_PINS[i]);
  }

  // 2. Print numeric values
  for (int i = 0; i < 5; i++) {
    Serial.print("  ");
    Serial.print(sensorValues[i]);
    Serial.print("    ");
  }

  // 3. Print a quick visual representation
  Serial.print(" --> [ ");
  for (int i = 0; i < 5; i++) {
    if (sensorValues[i] == 1) {
      Serial.print("█ "); // Detected Black (or White depending on module)
    } else {
      Serial.print(". "); // Opposite surface
    }
  }
  Serial.println("]");

  delay(200); // Update 5 times per second
}