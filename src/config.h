
#include <Arduino.h>

const char* ssid = "Default";
const char* password = "default234";

String tsApiKey = "ULKHU5TK22XGKN51";
String tsBaseURL = "http://api.thingspeak.com/update";

String botToken  = "8587477001:AAHYqDfI1g5Qi7Yd6ewZYy7srH-mXWogu9s";
String chatID = "5089825587";
String chatIDtarget = "8587477001";
String tgBaseURL = "https://api.telegram.org/bot";

float maxDistance = 100;
float midDistance = 5;
float minDistance = 2;

int maxGas1 = 1500;
int minGas1 = 0;
int maxGas2 = 1500;
int minGas2 = 0;

float maxAx = 0.6;
float maxAy = 0.6;
float maxAz = 1.2;

double maxVelocity = 70;

String carRolledOverMessage = "Mobil terguling di https://maps.google.com/?q=";
String gas1Message = "Kandungan gas 1 sangat tinggi";
String gas2Message = "Kandungan gas 2 sangat tinggi";
String distanceMessage = "Jarak diluar batas";
String minDistanceMessage = "Jarak terlalu dekat";
String midDistanceMessage = "Jarak agak bahaya";
String drowsinessMessage = "Terdeteksi ngantuk";
String velocityMessage = "Kecepatan melebihi batas";
