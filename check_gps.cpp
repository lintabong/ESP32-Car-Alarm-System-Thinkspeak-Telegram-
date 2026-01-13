#include <Arduino.h>
#include <HardwareSerial.h>

HardwareSerial GPS(1); // RX=16, TX=17

String nmea = "";
double lat = 0;
double lon = 0;

double convertNMEA(String raw, char dir) {
  if (raw.length() < 4) return 0;
  double val = raw.toFloat();
  int degrees = int(val / 100);
  double minutes = val - degrees * 100;
  double decimal = degrees + minutes / 60.0;
  if (dir == 'S' || dir == 'W') decimal *= -1;
  return decimal;
}

bool readGPS(double &lat, double &lon) {
  while (GPS.available()) {
    char c = GPS.read();

    if (c == '\n') {
      if (nmea.startsWith("$GNRMC") || nmea.startsWith("$GPRMC")) {

        String parts[12];
        int idx = 0;

        for (char ch : nmea) {
          if (ch == ',') {
            if (idx < 11) idx++;
          } else {
            if (idx < 12) parts[idx] += ch;
          }
        }

        if (parts[2] != "A") {
          nmea = "";
          return false;
        }

        if (parts[3].length() > 0 && parts[4].length() > 0 &&
            parts[5].length() > 0 && parts[6].length() > 0) {

          lat = convertNMEA(parts[3], parts[4][0]);
          lon = convertNMEA(parts[5], parts[6][0]);
        }

        nmea = "";
        return true;
      }

      nmea = "";
    } 
    else {
      nmea += c;
    }
  }

  return false;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  GPS.begin(9600, SERIAL_8N1, 16, 17);

  Serial.println("GPS Monitor Started");
  Serial.println("Waiting for GPS signal...");
}

void loop() {
  if (readGPS(lat, lon)) {
    Serial.print("Latitude: ");
    Serial.print(lat, 6);
    Serial.print(" | Longitude: ");
    Serial.print(lon, 6);
    Serial.print(" | Google Maps: https://maps.google.com/?q=");
    Serial.print(lat, 6);
    Serial.print(",");
    Serial.println(lon, 6);
  }

  delay(100);
}