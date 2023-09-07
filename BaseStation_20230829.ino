#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <math.h>

#define SCK 42
#define MISO 41
#define MOSI 2
#define SS 1
#define rst 5
#define dio0 4

const int triggerPin = 14;

TinyGPSPlus gps;
#define hs Serial1

// Declare a global variable
unsigned long calibrateStartTime = 0;
unsigned long roundTripTime = 0;  // To store the round-trip time
unsigned long oneWayTime = 0;
unsigned long shotStartTime;          //used for shot timer
unsigned long adjustedRoundTripTime;  //this will be factored into the time of flight
unsigned long shotTriggerTime;        //Time from trigger to response from target
unsigned long elapsedTime;
const long debounceDelay = 1000;      // the debounce time in milliseconds
float baseLat;
float baseLon;

const int numTargets = 10;
long impactTimes[numTargets] = { 0 };
int hitCounts[numTargets];  // where numTargets is the number of targets you have


void setup() {
  Serial.begin(115200);
  hs.begin(9600);
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, rst, dio0);
  LoRa.begin(915E6);
  delay(2000);
  Serial.println("Type 'calibrate' to start calibration, 'shot' to simulate shot trigger,")
  Serial.println("reset1 to reset hit count on target 1 or 'gps' to request target location.");
  pinMode(triggerPin, INPUT_PULLUP);
  digitalWrite(triggerPin, LOW);

  // Initialize hitCounts to zero
  for (int i = 0; i < numTargets; i++) {
    hitCounts[i] = 0;
  }
}

void loop() {

  // Check for input from the serial monitor
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    if (input == "calibrate") {
      Serial.println("Calibrate started");
      sendCalibrateMessage();
      //also need to run function to start timer and calculate transmit round trip time
    } else if (input == "shot") {
      Serial.println("Shot trigger started");
      sendWaitForImpact();
      //also need to run function to start timer and calculate time of flight
    } else if (input == "gps") {
      Serial.println("Getting target location for range");
      sendPositionRequest(1);
      //also need to run function to start calculate the range between target and base station
    } else if (input == "reset1") {
      hitCounts[1] = 0;  // Reset the hit count for Target 1
      Serial.println("Hit count for Target 1 has been reset.");
    }
  }

  // Set trigger state and debounce
  static bool shotTrigger = false;
  static unsigned long lastDebounceTime = 0;
  bool currentTriggerState = digitalRead(triggerPin);

  if (currentTriggerState == LOW) {
    if (!shotTrigger && (millis() - lastDebounceTime) > debounceDelay) {
      sendWaitForImpact();
      shotTrigger = true;
      lastDebounceTime = millis();  // Reset debounce time on press
      Serial.println("Shot Detected.");
    }
  } else {
    shotTrigger = false;
    lastDebounceTime = millis();  // Reset debounce time on release
  }

  // Look for LoRa packet to be received
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    onReceive(packetSize);
  }

  // GPS handling
  if (hs.available()) {
    gps.encode(hs.read());
    smartDelay(10);
  }
}

void handlePositionInfo(double baseLat, double baseLon, double targetLat, double targetLon, int targetID) {
  double distance = calculateDistance(baseLat, baseLon, targetLat, targetLon);
  double bearing = calculateBearing(baseLat, baseLon, targetLat, targetLon);

  // Do something with the distance and bearing
  // For example, print them out
  Serial.print("Target ID: ");
  Serial.println(targetID);
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" meters");
  Serial.print("Bearing: ");
  Serial.print(bearing);
  Serial.println(" degrees");
}


// Convert degrees to radians
double toRadians(double degree) {
  return degree * (PI / 180);
}

// Convert radians to degrees
double toDegrees(double radians) {
  return radians * (180 / PI);
}

// Calculate distance between two points in meters
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000;  // Earth's radius in meters
  double phi1 = toRadians(lat1);
  double phi2 = toRadians(lat2);
  double deltaPhi = toRadians(lat2 - lat1);
  double deltaLambda = toRadians(lon2 - lon1);

  double a = sin(deltaPhi / 2) * sin(deltaPhi / 2) + cos(phi1) * cos(phi2) * sin(deltaLambda / 2) * sin(deltaLambda / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return R * c;
}

// Calculate bearing between two points in degrees
double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
  double phi1 = toRadians(lat1);
  double phi2 = toRadians(lat2);
  double deltaLambda = toRadians(lon2 - lon1);

  double x = sin(deltaLambda) * cos(phi2);
  double y = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(deltaLambda);

  double bearing = atan2(x, y);
  bearing = fmod((toDegrees(bearing) + 360.0), 360.0);  // Normalize to 0°..360°

  return bearing;
}

// Wrapper function to calculate both range and bearing
void calculateRangeAndBearing(float baseLat, float baseLon, float targetLat, float targetLon, int targetID) {
  double distance = calculateDistance(baseLat, baseLon, targetLat, targetLon);
  double bearing = calculateBearing(baseLat, baseLon, targetLat, targetLon);

  // Output or store the calculated distance and bearing
  Serial.print("Target ");
  Serial.print(targetID);
  Serial.print(", Distance: ");
  Serial.print(distance, 2);  // Display up to 2 decimal places
  Serial.print(" meters, Bearing: ");
  Serial.print(bearing, 2);  // Display up to 2 decimal places
  Serial.println(" degrees");
}

void onReceive(int packetSize) {
  char incoming[512];  // Buffer to store incoming messages
  int index = 0;
  int targetID;
  float targetLat, targetLon;

  if (packetSize == 0) return;

  // Read the incoming message into the buffer
  while (LoRa.available()) {
    incoming[index] = (char)LoRa.read();
    index++;
  }
  incoming[index] = '\0';  // Null-terminate the C-string

  // Log the received message
  Serial.print("Received message: ");
  Serial.println(incoming);

  // Handle calibration response
  if (strncmp(incoming, "calibrateResponse", 17) == 0) {
    // Calculate the round-trip time
    roundTripTime = millis() - calibrateStartTime;
    oneWayTime = roundTripTime / 2;  // Calculate the one-way time
    Serial.print("One-way time calculated: ");
    Serial.println(oneWayTime);

    // Handle hit message (TargetID)
  } else if (sscanf(incoming, "Target%d", &targetID) == 1) {
    if (targetID >= 0 && targetID < numTargets) {
      hitCounts[targetID]++;  // Increment the hit count for this target
      Serial.print("Target ");
      Serial.print(targetID);
      Serial.print(" has been hit ");
      Serial.print(hitCounts[targetID]);
      Serial.println(" times.");
    }

    // Handle position response (TargetID, Latitude, Longitude)
  } else if (sscanf(incoming, "Target%d,%f,%f", &targetID, &targetLat, &targetLon) == 3) {
    char baseLocationBuffer[128];
    float baseLat, baseLon;
    if (getLocation(baseLocationBuffer, sizeof(baseLocationBuffer))) {
      sscanf(baseLocationBuffer, "%f,%f", &baseLat, &baseLon);
      calculateRangeAndBearing(baseLat, baseLon, targetLat, targetLon, targetID);
    }

    // Handle time elapsed message (TargetID, elapsedTime)
  } else if (sscanf(incoming, "Target%d,Time:%lu", &targetID, &elapsedTime) == 2) {
    Serial.print("Time elapsed received from Target: ");
    Serial.print(targetID);
    Serial.print(", Time: ");
    Serial.println(elapsedTime);
    calculateTotalTime(elapsedTime);  // Pass the elapsedTime to the function

  } else {
    Serial.println("Unrecognized message received.");
  }
}


void calculateTotalTime(unsigned long receivedElapsedTime) {
  unsigned long totalTime = receivedElapsedTime + oneWayTime;
  Serial.print("Total time of flight: ");
  Serial.println(totalTime);
}


void sendWaitForImpact() {
  char waitForImpactMessage[20] = "waitForImpact";
  sendMessage(waitForImpactMessage);
}

// For sending the calibrate message from the base station
void sendCalibrateMessage() {
  char calibrateMessage[20] = "calibrateMessage ";  // Note the extra space to make it 17 characters
  sendFixedLengthMessage(calibrateMessage, 17);     // Send exactly 17 bytes
  calibrateStartTime = millis();                    // Store the time when the message is sent
}

void sendPositionRequest(int targetID) {
  char requestMessage[50];
  snprintf(requestMessage, sizeof(requestMessage), "positionRequest,Target%d", targetID);
  sendMessage(requestMessage);
}

// Function to send fixed-length messages
void sendFixedLengthMessage(const char *message, int length) {
  LoRa.beginPacket();
  LoRa.write((uint8_t *)message, length);  // Send exactly 'length' bytes
  LoRa.endPacket();
}

void sendMessage(const char *message) {
  // Check if there's enough space for the message
  if (strlen(message) > 250) {
    Serial.println("Message too long.");
    return;
  }

  LoRa.beginPacket();
  LoRa.write((uint8_t *)message, strlen(message));
  LoRa.endPacket();
}

//GPS Functions Here

//probably need to fix this for base station location
bool getLocation(char *locationBuffer, int bufferSize) {
  smartDelay(10);  // Wait for a short time to collect GPS data

  float baseLatitude;
  float baseLongitude;

  if (gps.location.isValid()) {
    baseLatitude = gps.location.lat();
    baseLongitude = gps.location.lng();
    Serial.print("Lat ");
    Serial.println(baseLatitude, 6);
    Serial.print("Lon ");
    Serial.println(baseLongitude, 6);

    // Use snprintf to populate the locationBuffer
    snprintf(locationBuffer, bufferSize, "%f,%f", baseLatitude, baseLongitude);
    return true;  // Return true to indicate success
  } else {
    Serial.println("GPS location is not valid.");
    strncpy(locationBuffer, "INVALID", bufferSize);
    return false;  // Return false to indicate failure
  }
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (hs.available())
      gps.encode(hs.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec) {
  if (!valid) {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  } else {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1);  // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3
                           : vi >= 10  ? 2
                                       : 1;
    for (int i = flen; i < len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len) {
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i = strlen(sz); i < len; ++i)
    sz[i] = ' ';
  if (len > 0)
    sz[len - 1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t) {
  if (!d.isValid()) {
    Serial.print(F("********** "));
  } else {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }

  if (!t.isValid()) {
    Serial.print(F("******** "));
  } else {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len) {
  int slen = strlen(str);
  for (int i = 0; i < len; ++i)
    Serial.print(i < slen ? str[i] : ' ');
  smartDelay(0);
}