
#include <Arduino.h>

const char* ssid = "";
const char* password = "";

String tsApiKey = "";
String tsBaseURL = "http://api.thingspeak.com/update";

String botToken  = "";
String chatID = "";
String tgBaseURL = "https://api.telegram.org/bot";

float maxDistance = 100;
float minDistance = 20;

int maxGas1 = 4000;
int minGas1 = 0;
int maxGas2 = 4000;
int minGas2 = 0;

float maxAx = 0.6;
float maxAy = 0.6;
float maxAz = 0.6;

double maxVelocity = 90.0;

String carRolledOverMessage = "Mobil terguling di https://maps.google.com/?q=";
String gas1Message = "Kandungan gas 1 sangat tinggi";
String gas2Message = "Kandungan gas 2 sangat tinggi";
String distanceMessage = "Jarak diluar batas";
String velocityMessage = "Kecepatan melebihi batas";
