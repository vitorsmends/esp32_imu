#include <Wire.h>
#include <MPU6050.h>
#include <BluetoothSerial.h>

MPU6050 mpu;
BluetoothSerial SerialBT;

#define I2C_SDA 21
#define I2C_SCL 22

void setup() {
  Serial.begin(115200);

  SerialBT.begin("ESP32_IMU");
  Serial.println("Bluetooth inicializado. Agora você pode se conectar!");

  Wire.begin(I2C_SDA, I2C_SCL);

  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU6050 conectado!");
  } else {
    Serial.println("Erro ao conectar com o MPU6050.");
    while (1);
  }
}

void loop() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  String imuData = "A";
  imuData += String(ax) + ",";
  imuData += String(ay) + ",";
  imuData += String(az) + ",G";
  imuData += String(gx) + ",";
  imuData += String(gy) + ",";
  imuData += String(gz);

  Serial.println(imuData);

  SerialBT.println(imuData);

  delay(100);
}
