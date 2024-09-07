#include "BluetoothSerial.h" // Library for Bluetooth

BluetoothSerial ESP_BT; // Object for Bluetooth

void setup() {
  Serial.begin(115200); // Start the Serial Monitor
  ESP_BT.begin("ESP32_Test"); // Bluetooth device name
  Serial.println("Bluetooth Device is Ready to Pair");
}

void loop() {
  if (ESP_BT.connected()) {
    ESP_BT.println("Hello, this is a message from ESP32!");
    delay(2000); // Wait for 2 seconds
  }
}
