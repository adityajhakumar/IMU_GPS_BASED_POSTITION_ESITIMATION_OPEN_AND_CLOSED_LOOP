#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// BNO055 setup
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // Default I2C address

// Timing variables
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 1000;  // 1 second

void setup() {
  Serial.begin(115200);

  // Initialize I2C for BNO055 (SDA = 21, SCL = 22)
  Wire.begin(21, 22);

  if (!bno.begin()) {
    Serial.println("BNO055 not detected!");
    while (1);
  }

  bno.setExtCrystalUse(true);

  Serial.println("Initializing IMU...");
}

void loop() {
  unsigned long now = millis();

  // Print IMU data at 1 Hz
  if (now - lastPrintTime >= printInterval) {
    lastPrintTime = now;

    // --- Read sensor data ---
    imu::Vector<3> linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

    uint8_t calSys, calGyro, calAccel, calMag;
    bno.getCalibration(&calSys, &calGyro, &calAccel, &calMag);

    // --- Output ---
    Serial.println("\n========== IMU DATA ==========");
    
    Serial.print("CALIB: SYS="); Serial.print(calSys);
    Serial.print(" GYR="); Serial.print(calGyro);
    Serial.print(" ACC="); Serial.print(calAccel);
    Serial.print(" MAG="); Serial.println(calMag);


Serial.print("Linear Accel (m/s^2): ");
Serial.print(linAccel.x(), 2); Serial.print(", ");
Serial.print(linAccel.y(), 2); Serial.print(", ");
Serial.println(linAccel.z(), 2);


    Serial.print("Gyro (rad/s): ");
    Serial.print(gyro.x(), 2); Serial.print(", ");
    Serial.print(gyro.y(), 2); Serial.print(", ");
    Serial.println(gyro.z(), 2);

    Serial.print("Mag (uT): ");
    Serial.print(mag.x(), 2); Serial.print(", ");
    Serial.print(mag.y(), 2); Serial.print(", ");
    Serial.println(mag.z(), 2);
  }
}
