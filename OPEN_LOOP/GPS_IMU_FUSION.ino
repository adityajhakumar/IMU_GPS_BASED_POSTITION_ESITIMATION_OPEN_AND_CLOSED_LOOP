#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <TinyGPSPlus.h>
#include <EEPROM.h>
#include <utility/imumaths.h>

// ---------- Definitions ----------
#define EEPROM_SIZE      512
#define CALIB_FLAG_ADDR   0
#define OFFSETS_ADDR      1
#define GPS_RX_PIN       16
#define GPS_TX_PIN       17
#define GPS_BAUD        9600

const float dt = 0.20f;
const float R_gps = 6.0f;
const float Q_base = 0.05f;
const float velocityDamp = 0.90f;
const float accelThreshold = 0.1f;
const float speedThreshold = 0.5f;
const unsigned long gpsTimeout = 3000;

const double EARTH_RADIUS_M = 6371000.0;
const double DEG2RAD = 3.14159265358979323846 / 180.0;

// ---------- Global Variables ----------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
TinyGPSPlus gps;
HardwareSerial GPSSerial(1);

float X[4] = {0};
float P[4][4] = {
  {10, 0, 0, 0},
  {0, 10, 0, 0},
  {0, 0, 1, 0},
  {0, 0, 0, 1}
};

double originLat = 0.0, originLon = 0.0;
bool originSet = false, gpsAvailable = false;
unsigned long lastGPSFixTime = 0;
String nmeaLine = "";

// ---------- Function Declarations ----------
void loadCalibration();
void saveCalibration();
void printCalibrationStatus();
void convertLatLonToXY(double lat, double lon, float &outX, float &outY);
void convertXYToLatLon(float x, float y, double &outLat, double &outLon);
void kalmanPredict(float accelX, float accelY);
void kalmanUpdate(float measX, float measY);
float distanceBetween(float x1, float y1, float x2, float y2);

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(100);
  EEPROM.begin(EEPROM_SIZE);

  Wire.begin(21, 22);  // SDA, SCL
  if (!bno.begin()) {
    Serial.println("BNO055 not detected. Check wiring.");
    while (1);
  }

  loadCalibration();
  bno.setExtCrystalUse(true);
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  lastGPSFixTime = millis();

  Serial.println("Setup complete.");
}

// ---------- Loop ----------
void loop() {
  // Read GPS and build NMEA line
  while (GPSSerial.available() > 0) {
    char c = GPSSerial.read();
    gps.encode(c);
    if (c == '\n') {
      nmeaLine.trim();
      if (nmeaLine.startsWith("$GPRMC")) {
        Serial.print("GPRMC: ");
        Serial.println(nmeaLine);
      }
      nmeaLine = "";
    } else {
      nmeaLine += c;
    }
  }

  printCalibrationStatus();

  static bool offsetsSaved = false;
  uint8_t sysCal, gyroCal, accelCal, magCal;
  bno.getCalibration(&sysCal, &gyroCal, &accelCal, &magCal);
  if (!offsetsSaved && sysCal == 3 && gyroCal == 3 && accelCal == 3 && magCal == 3) {
    saveCalibration();
    offsetsSaved = true;
  }

  bool newGPSFix = false;
  double lat = 0.0, lon = 0.0, gpsSpeed = 0.0;

  if (gps.location.isUpdated() && gps.location.isValid()) {
    lat = gps.location.lat();
    lon = gps.location.lng();
    gpsSpeed = gps.speed.mps();
    newGPSFix = true;
    gpsAvailable = true;
    lastGPSFixTime = millis();

    if (!originSet) {
      originLat = lat;
      originLon = lon;
      originSet = true;
      Serial.println("✅ Origin set.");
    }
  }

  // ---------- GPS Data ----------
  Serial.print("GPS Lat/Lon: ");
  Serial.print(gps.location.lat(), 6);
  Serial.print(", ");
  Serial.print(gps.location.lng(), 6);
  Serial.print(" (Valid? ");
  Serial.print(gps.location.isValid() ? "Yes" : "No");
  Serial.println(")");

  Serial.print("GPS Speed (m/s): ");
  Serial.print(gps.speed.mps(), 3);
  Serial.print(" (Valid? ");
  Serial.print(gps.speed.isValid() ? "Yes" : "No");
  Serial.println(")");

  // ---------- IMU Data ----------
  imu::Vector<3> linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  Serial.println("\n========== IMU DATA ==========");
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

  // ---------- Sensor Fusion ----------
  float ax = linAccel.x(), ay = linAccel.y();
  float accelMag = sqrt(ax * ax + ay * ay);
  bool isMoving = (accelMag > accelThreshold) || (newGPSFix && gpsSpeed > speedThreshold);

  imu::Quaternion quat = bno.getQuat();
  imu::Vector<3> accelEarth = quat.rotateVector(linAccel);
  float ax_e = accelEarth.x(), ay_e = accelEarth.y();

  float prevX = X[0], prevY = X[1];

  if (isMoving) kalmanPredict(ax_e, ay_e);
  else {
    X[0] += X[2] * dt;
    X[1] += X[3] * dt;
    X[2] *= velocityDamp;
    X[3] *= velocityDamp;
    for (int i = 0; i < 4; i++) P[i][i] += Q_base;
  }

  if (newGPSFix && originSet && isMoving) {
    float measX, measY;
    convertLatLonToXY(lat, lon, measX, measY);
    kalmanUpdate(measX, measY);
  }

  if ((millis() - lastGPSFixTime) > gpsTimeout) gpsAvailable = false;

  // ---------- Position Output ----------
  if (originSet) {
    double fusedLat, fusedLon, estLat, estLon;
    convertXYToLatLon(X[0], X[1], fusedLat, fusedLon);
    convertXYToLatLon(prevX, prevY, estLat, estLon);

    Serial.println("\n--- Position ---");
    if (gpsAvailable) {
      float posError = distanceBetween(X[0], X[1], prevX, prevY);

      Serial.print("GPS only: ");
      Serial.print(fusedLat, 6); Serial.print(", ");
      Serial.println(fusedLon, 6);

      Serial.print("Estimated (IMU only): ");
      Serial.print(estLat, 6); Serial.print(", ");
      Serial.println(estLon, 6);

      Serial.print("Position Error (m): ");
      Serial.println(posError, 3);
    } else {
      Serial.print("Estimated (GPS lost): ");
      Serial.print(estLat, 6); Serial.print(", ");
      Serial.println(estLon, 6);
    }

    // Heading
    Serial.println("\n--- Orientation ---");
    float heading = atan2(mag.y(), mag.x()) * 180.0 / PI;
    if (heading < 0) heading += 360.0;
    Serial.print("Heading (deg): ");
    Serial.println(heading, 2);
  }

  Serial.println("\n--------------------------------\n");
  delay((int)(dt * 1000));
}

// ---------- Calibration ----------
void loadCalibration() {
  byte flag = EEPROM.read(CALIB_FLAG_ADDR);
  if (flag == 0x55) {
    adafruit_bno055_offsets_t storedOffsets;
    EEPROM.get(OFFSETS_ADDR, storedOffsets);
    bno.setSensorOffsets(storedOffsets);
    delay(10);
    Serial.println("✅ Calibration loaded from EEPROM.");
  } else {
    Serial.println("❌ No calibration data in EEPROM.");
  }
}

void saveCalibration() {
  adafruit_bno055_offsets_t saveOffsets;
  bno.getSensorOffsets(saveOffsets);
  EEPROM.put(OFFSETS_ADDR, saveOffsets);
  EEPROM.write(CALIB_FLAG_ADDR, 0x55);
  EEPROM.commit();
  Serial.println("💾 Calibration saved to EEPROM.");
}

void printCalibrationStatus() {
  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  Serial.print("Calib: SYS="); Serial.print(sys);
  Serial.print(" GYR="); Serial.print(gyro);
  Serial.print(" ACC="); Serial.print(accel);
  Serial.print(" MAG="); Serial.println(mag);
}

// ---------- Coordinate Conversion ----------
void convertLatLonToXY(double lat, double lon, float &outX, float &outY) {
  if (!originSet) { outX = 0; outY = 0; return; }
  double dLat = (lat - originLat) * DEG2RAD;
  double dLon = (lon - originLon) * DEG2RAD;
  double cosLat0 = cos(originLat * DEG2RAD);
  outY = dLat * EARTH_RADIUS_M;
  outX = dLon * EARTH_RADIUS_M * cosLat0;
}

void convertXYToLatLon(float x, float y, double &outLat, double &outLon) {
  if (!originSet) { outLat = 0; outLon = 0; return; }
  double cosLat0 = cos(originLat * DEG2RAD);
  outLat = originLat + (y / EARTH_RADIUS_M) * (180.0 / M_PI);
  outLon = originLon + (x / (EARTH_RADIUS_M * cosLat0)) * (180.0 / M_PI);
}

// ---------- Kalman Filter ----------
void kalmanPredict(float accelX, float accelY) {
  X[0] += X[2] * dt + 0.5f * accelX * dt * dt;
  X[1] += X[3] * dt + 0.5f * accelY * dt * dt;
  X[2] = velocityDamp * (X[2] + accelX * dt);
  X[3] = velocityDamp * (X[3] + accelY * dt);
  for (int i = 0; i < 4; i++) P[i][i] += Q_base;
}

void kalmanUpdate(float zX, float zY) {
  float residX = zX - X[0];
  float residY = zY - X[1];
  float Kx = P[0][0] / (P[0][0] + R_gps);
  float Ky = P[1][1] / (P[1][1] + R_gps);
  X[0] += Kx * residX;
  X[1] += Ky * residY;
  P[0][0] *= (1.0f - Kx);
  P[1][1] *= (1.0f - Ky); 
}

float distanceBetween(float x1, float y1, float x2, float y2) {
  float dx = x1 - x2, dy = y1 - y2;
  return sqrt(dx * dx + dy * dy);
}
