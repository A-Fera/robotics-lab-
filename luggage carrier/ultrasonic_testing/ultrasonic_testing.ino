const int TRIG_PIN = 5;   // GPIO 5
const int ECHO_PIN = 18;  // GPIO 18

const float SPEED_OF_SOUND = 0.0343; // cm/microsecond

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.println("ESP32 Ultrasonic Distance Sensor Ready!");
}

void loop() {
  // Clear the trigger
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // Send 10µs pulse to start measurement
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read the echo return time (timeout 30ms)
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) {
    Serial.println("Out of range or no echo!");
  } else {
    float distanceCm = (duration * SPEED_OF_SOUND) / 2.0;
    Serial.print("Distance: ");
    Serial.print(distanceCm, 1);
    Serial.println(" cm");
  }

  delay(500); // Read twice per second
}