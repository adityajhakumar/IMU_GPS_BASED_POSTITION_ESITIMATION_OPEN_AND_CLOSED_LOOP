#include <TinyGPSPlus.h>

#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GPS_BAUD   9600

TinyGPSPlus gps;
HardwareSerial GPSSerial(1);  // Use UART1 on ESP32

void setup() {
  Serial.begin(115200);
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("Starting GPS data read...");
}

void loop() {
  // Read and decode incoming GPS data
  while (GPSSerial.available() > 0) {
    char c = GPSSerial.read();
    gps.encode(c);
  }

  if (gps.location.isValid()) {
    Serial.println("===== GPS Data =====");
    Serial.print("Latitude  : ");
    Serial.println(gps.location.lat(), 6);

    Serial.print("Longitude : ");
    Serial.println(gps.location.lng(), 6);

    Serial.print("Speed     : ");
    Serial.print(gps.speed.mps(), 2);
    Serial.println(" m/s");

    if (gps.altitude.isValid()) {
      Serial.print("Altitude  : ");
      Serial.print(gps.altitude.meters(), 2);
      Serial.println(" m");
    } else {
      Serial.println("Altitude  : Not Available");
    }

    Serial.println();
  } else {
    Serial.println("Waiting for valid GPS fix...");
  }

  delay(1000);  // Delay for readability
}
