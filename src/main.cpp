
#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <HardwareSerial.h>
#include "DFRobotDFPlayerMini.h"

#include "config.h"

const int AX_PIN = 6;
const int AY_PIN = 7;
const int AZ_PIN = 8;
const int TRIG_PIN = 5;
const int ECHO_PIN = 4;
const int GAS1_PIN = 1;
const int GAS2_PIN = 2;
const int CAM_PIN = 10;
const unsigned long MAX_ECHO_TIME = 38000UL;
const unsigned long sendInterval = 20000;

HardwareSerial GPS(1); // RX=16, TX=17
HardwareSerial mp3Serial(2);

DFRobotDFPlayerMini dfplayer;

WiFiClientSecure secured_client;
UniversalTelegramBot bot(botToken, secured_client);

String nmea = "";
String currentLocation = "";
double lat = 0;
double lon = 0;
int gas1 = 0;
int gas2 = 0;

float ax = 0;
float ay = 0;
float az = 0;

float dax = 0;
float day = 0;
float daz = 0;

float distance = 0;
unsigned long lastSend = 0;
unsigned long lastCheck = 0;
const long pollingInterval = 1500;

unsigned long velocityInterval = 500;
unsigned long lastVelocityTime = 0;

double velocity = 0.0;
double lastLat = 0.0;
double lastLon = 0.0;
bool hasLast = false;

double haversine(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0; // Earth radius in meters
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);

  lat1 = radians(lat1);
  lat2 = radians(lat2);

  double a = sin(dLat/2) * sin(dLat/2) +
             sin(dLon/2) * sin(dLon/2) * cos(lat1) * cos(lat2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

double calculateVelocity(double currentLat, double currentLon) {
  unsigned long now = millis();

  if (now - lastVelocityTime < velocityInterval)
    return -1;  // -1 means "not updated yet"

  lastVelocityTime = now;

  // If this is the first reading
  if (!hasLast) {
    lastLat = currentLat;
    lastLon = currentLon;
    hasLast = true;
    return 0.0;   // first measurement → velocity = 0
  }

  double distance = haversine(lastLat, lastLon, currentLat, currentLon); // meters
  double dt = velocityInterval / 1000.0; // seconds

  lastLat = currentLat;
  lastLon = currentLon;

  double velocity = distance / dt;

  return velocity * 3.6;
}

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

float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, MAX_ECHO_TIME);
  if (duration == 0) return -1;

  return duration / 58.0;  // cm
}

void sendToThingSpeak() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi lost — reconnecting...");
    WiFi.reconnect();
    return;
  }

  String url = tsBaseURL +
    "?api_key=" + tsApiKey +
    "&field1=" + String(distance, 2) +
    "&field2=" + String(lat, 6) +
    "&field3=" + String(lon, 6) +
    "&field4=" + String(gas1) +
    "&field5=" + String(gas2);

  Serial.println("Request: " + url);

  HTTPClient http;
  http.begin(url);

  int code = http.GET();

  if (code > 0) {
    Serial.print("ThingSpeak Response: ");
    Serial.println(http.getString());
  } else {
    Serial.println("HTTP error");
  }

  http.end();
}

void sendTelegram(String message) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi lost — reconnecting...");
    WiFi.reconnect();
    return;
  }

  String url = tgBaseURL + botToken +
      "/sendMessage?chat_id=" + chatID +
      "&text=" + message;

  HTTPClient http;
  http.begin(url);
  http.GET();
  http.end();
}

float readAndConvertADXL335(int pin) {
  int raw = analogRead(pin);
  float voltage = (raw / 4095.0) * 3.3;
  return (voltage - 1.65) / 0.300;
}

void handleMessage(int numMessages) {
  for (int i = 0; i < numMessages; i++) {
    String chat_id = bot.messages[i].chat_id;
    String text = bot.messages[i].text;

    if (text == "/location") {
      String message = "https://maps.google.com/?q=" 
          + String(lat, 6) + "," + String(lon, 6);
      sendTelegram(message);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(800);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
  }
  sendTelegram("[DEBUG] WIFI connected");

  secured_client.setInsecure();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(AX_PIN, INPUT);
  pinMode(AY_PIN, INPUT);
  pinMode(AZ_PIN, INPUT);

  GPS.begin(9600, SERIAL_8N1, 16, 17);

  mp3Serial.begin(9600, SERIAL_8N1, 15, 14);

  if (!dfplayer.begin(mp3Serial)) {
    sendTelegram("[DEBUG] Unable to begin DFPlayer");
    while (true) {
      Serial.println("DFPlayer init failed.");
      delay(2000);
    }
  }

  dfplayer.setTimeOut(500);
  dfplayer.volume(20); // 0...30
  dfplayer.EQ(DFPLAYER_EQ_NORMAL); // equalizer
  dfplayer.outputDevice(DFPLAYER_DEVICE_SD); // SD card as media

  ax = readAndConvertADXL335(AX_PIN);
  ay = readAndConvertADXL335(AY_PIN);
  az = readAndConvertADXL335(AZ_PIN);

  sendTelegram("[DEBUG] System Ready");

  const String commands = F("["
                            "{\"command\":\"location\",  \"description\":\"melihat update lokasi\"}"
                            "]");
  bot.setMyCommands(commands);
  dfplayer.play(2);
}

void loop() {
  distance = readUltrasonic();
  gas1 = analogRead(GAS1_PIN);
  gas2 = analogRead(GAS2_PIN);
  dax = abs(readAndConvertADXL335(AX_PIN) - ax);
  day = abs(readAndConvertADXL335(AY_PIN) - ay);
  daz = abs(readAndConvertADXL335(AZ_PIN) - az);

  velocity = calculateVelocity(lat, lon);

  if (readGPS(lat, lon)) {
    Serial.print(distance);
    Serial.print(" || ");
    Serial.print(gas1);
    Serial.print("  ");
    Serial.print(gas2);
    Serial.print(" || ");
    Serial.print(dax, 2); Serial.print(",");
    Serial.print(day, 2); Serial.print(",");
    Serial.print(daz, 2);
    Serial.print(" || ");
    Serial.print(velocity);                                                                                                                                                                                                                                                                                                 
    Serial.print(" || https://maps.google.com/?q=");
    Serial.print(lat, 6); Serial.print(",");
    Serial.println(lon, 6);

    currentLocation = String(lat, 6) + "," + String(lon, 6);
  }

  if (dax > maxAx || day > maxAy || daz > maxAz) {
    sendTelegram(carRolledOverMessage + currentLocation);
    delay(1000);
  }

  if (gas1 > maxGas1){
    sendTelegram(gas1Message);
    delay(1000);
  }

  if (gas2 > maxGas2){
    sendTelegram(gas2Message);
    delay(1000);
  }

  if (distance >= maxDistance && distance <= minDistance){
    sendTelegram(distanceMessage);
    delay(1000);
  }

  if (velocity > maxVelocity){
    sendTelegram(velocityMessage);
    delay(1000);
  }

  unsigned long now = millis();
  if (now - lastSend >= sendInterval) {
    lastSend = now;
    sendToThingSpeak();
  }

  if (millis() - lastCheck > 1500) {
    int newMsg = bot.getUpdates(bot.last_message_received + 1);

    while (newMsg) {
      handleMessage(newMsg);
      newMsg = bot.getUpdates(bot.last_message_received + 1);
    }

    lastCheck = millis();
  }
  
  delay(50);
}
