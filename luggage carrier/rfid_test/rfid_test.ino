#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN    5   // SDA
#define RST_PIN   22  // RST
#define SCK_PIN   18  // SCK
#define MISO_PIN  19  // MISO
#define MOSI_PIN  23  // MOSI

MFRC522 rfid(SS_PIN, RST_PIN);

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Initialize SPI
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  rfid.PCD_Init();
  delay(100);

  // 🔥 BOOST ANTENNA GAIN TO MAXIMUM (Fix for 0x82 chips)
  rfid.PCD_SetAntennaGain(rfid.RxGain_max);

  Serial.println("=========================================");
  Serial.println("Antenna Gain set to MAXIMUM (48dB)");
  Serial.println("Ready! Hold card directly on the antenna.");
  Serial.println("=========================================");
}

void loop() {
  // Check for card
  if (!rfid.PICC_IsNewCardPresent()) {
    return;
  }

  // Read card data
  if (!rfid.PICC_ReadCardSerial()) {
    return;
  }

  // Print detected UID
  Serial.print("🎉 CARD FOUND! UID: ");
  for (byte i = 0; i < rfid.uid.size; i++) {
    Serial.print(rfid.uid.uidByte[i] < 0x10 ? " 0" : " ");
    Serial.print(rfid.uid.uidByte[i], HEX);
  }
  Serial.println();

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  delay(1000);
}