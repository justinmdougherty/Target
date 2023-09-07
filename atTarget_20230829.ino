#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>

#define ss 8
#define rst 4
#define dio0 7

const int targetID = 1;
const int triggerPin = 14;
const int led = 13;
float targetLatitude;
float targetLongitude;
unsigned long impactStartTime = 0;
const long debounceDelay = 1000;  // the debounce time in milliseconds

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
#define hs Serial1

bool waitForTrigger = true;
char messageToSend[512] = "";       // The message to send
unsigned long nextMessageTime = 0;  // The time when the next message should be sent
bool shouldSendResponse = false;    //Track whether a response should be sent
static bool waitForImpactMessageReceived = false;

void setup() {
  Serial.begin(115200);
  hs.begin(9600);
  Serial.println("LoRa Receiver");
  LoRa.setPins(ss, rst, dio0);
  //gps.init();

  while (!LoRa.begin(915E6)) {
    Serial.println(".");
    delay(500);
  }
  Serial.println("LoRa Initializing OK!");

  pinMode(triggerPin, INPUT_PULLUP);
  digitalWrite(triggerPin, LOW);
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);
}


void loop() {

  // Set trigger state and debounce
  static bool impactTrigger = false;
  static unsigned long lastDebounceTime = 0;
  bool currentTriggerState = digitalRead(triggerPin);

  // Timeout mechanism
  static unsigned long impactTimeout = 0;
  const unsigned long timeoutDuration = 30000;  // 30 seconds

  if (waitForImpactMessageReceived && millis() - impactStartTime > timeoutDuration) {
    Serial.println("Impact wait timed out. Resetting state.");
    impactStartTime = 0;                   // Reset the timer
    impactTrigger = false;                 // Reset the impact trigger
    waitForImpactMessageReceived = false;  // Reset the flag
    impactTimeout = millis();              // Update the timeout timestamp
  }

  // Check if the timer has started and the trigger has occurred
  if (impactStartTime != 0 && impactTrigger) {
    unsigned long elapsedTime = millis() - impactStartTime;
    impactStartTime = 0;                   // Reset the timer
    impactTrigger = false;                 // Reset the impact trigger
    waitForImpactMessageReceived = false;  // Reset the flag
    impactTimer(elapsedTime);              // Function to send the elapsed time back to the base station
  }

  // Check for impact trigger
  if (currentTriggerState == LOW) {
    if (!impactTrigger && (millis() - lastDebounceTime) > debounceDelay) {
      impactTrigger = true;
      lastDebounceTime = millis();  // Reset debounce time on press

      if (waitForImpactMessageReceived) {
        // If "waitForImpact" message was received, do nothing here.
        // The timer logic above will handle it.
      } else {
        Serial.println("Impact Detected.");
        sendHitMessage();
      }
    }
  } else {
    lastDebounceTime = millis();  // Reset debounce time on release
  }

  // This is for testing without requiring a trigger
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    if (input == "hit") {
      Serial.println("Hit!");
      sendHitMessage();
    }
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


void onReceive(int packetSize) {
  if (packetSize == 0) return;

  // Declare a C-string to hold the incoming message
  char incoming[256];
  int index = 0;

  // Read the incoming message
  while (LoRa.available()) {
    incoming[index] = (char)LoRa.read();
    index++;
  }

  // Don't forget to terminate the C-string with a null character
  incoming[index] = '\0';

  // Log the received message
  Serial.print("Received message: ");
  Serial.println(incoming);

  // Check for specific commands in the incoming message
  if (strncmp(incoming, "calibrateMessage", 16) == 0) {
    sendCalibrateResponse();
  } else if (strncmp(incoming, "waitForImpact", 13) == 0) {
    impactStartTime = millis();  // Start the timer
    waitForImpactMessageReceived = true;
  } else if (strncmp(incoming, "positionRequest", 15) == 0) {
    sendPositionResponse();
  } else {
    Serial.println("Unrecognized message received.");
  }
}


// For sending the calibrate response from the remote device
void sendCalibrateResponse() {
  char calibrateResponse[20] = "calibrateResponse";  // 17 characters
  sendFixedLengthMessage(calibrateResponse, 17);     // Send exactly 17 bytes
}

void sendHitMessage() {
  char hitMessage[20];                        // Declare the buffer without initializing
  memset(hitMessage, 0, sizeof(hitMessage));  // Zero out the memory

  // Use snprintf to concatenate "Target" and target ID
  snprintf(hitMessage, sizeof(hitMessage), "Target%d", targetID);

  sendMessage(hitMessage);
}

void sendPositionResponse() {
  char sendPositionResponse[512];  // Buffer to hold the response message
  char locationBuffer[128];        // Buffer to hold the location information

  if (getLocation(locationBuffer, sizeof(locationBuffer))) {
    // Format the complete response message
    snprintf(sendPositionResponse, sizeof(sendPositionResponse), "Target%d,%s", targetID, locationBuffer);

    // Send the complete response message
    sendMessage(sendPositionResponse);
  } else {
    // GPS fix is not valid, can either skip sending or send a default message
    Serial.println("Waiting for a good GPS fix to respond.");
  }
}

void impactTimer(unsigned long elapsedTime) {
  char timerMessage[50];  // Declare the buffer
  snprintf(timerMessage, sizeof(timerMessage), "Target%d,Time:%lu", targetID, elapsedTime);
  sendMessage(timerMessage);  // Send the message
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

bool getLocation(char *locationBuffer, int bufferSize) {
  smartDelay(10);  // Wait for a short time to collect GPS data

  float targetLatitude;
  float targetLongitude;

  if (gps.location.isValid()) {
    targetLatitude = gps.location.lat();
    targetLongitude = gps.location.lng();
    Serial.print("Lat ");
    Serial.println(targetLatitude, 6);
    Serial.print("Lon ");
    Serial.println(targetLongitude, 6);

    // Use snprintf to populate the locationBuffer
    snprintf(locationBuffer, bufferSize, "%f,%f,%d", targetLatitude, targetLongitude, targetID);
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