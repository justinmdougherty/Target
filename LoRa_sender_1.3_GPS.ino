#include <TinyGPSPlus.h>
#include <SPI.h>
#include <LoRa.h>

/*
   This sample code demonstrates the normal use of a TinyGPSPlus (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/


// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
#define hs Serial1

//define the pins used by the transceiver module
#define cs 8
#define rst 4
#define dio0 7

int counter = 0;

int triggerPin = 14;
const int target = 1;

void setup() {
  Serial.begin(9600);
  Serial.print("LoRa_Sender_1.3_GPS");
  Serial1.begin(9600);

  pinMode(triggerPin, INPUT_PULLUP);
  digitalWrite(13, LOW);

  LoRa.setPins(cs, rst, dio0);
  
}

void loop() {


  // Serial.print("Sending packet: ");
  // Serial.println(counter);

  // // send packet
  // LoRa.beginPacket();
  // LoRa.print("hello ");
  // LoRa.print(counter);
  // LoRa.endPacket();

  // counter++;

  // delay(5000);


  smartDelay(10);

  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    LoRa.beginPacket();
    LoRa.print("Target: ");
    LoRa.print(target);
    LoRa.print("Lat: ");
    LoRa.print(gps.location.lat(), 6);
    LoRa.print("Lon: ");
    LoRa.print(gps.location.lng(), 6);
    LoRa.endPacket();
  }

  if (digitalRead(triggerPin) == LOW) {
    Serial.println("Trigger");
    digitalWrite(13, HIGH);
    if (gps.location.isValid()) {
        LoRa.beginPacket();
        LoRa.print("Target: ");
        LoRa.print(target);
        LoRa.print("Lat: ");
        LoRa.print(gps.location.lat(), 6);
        LoRa.print("Lon: ");
        LoRa.print(gps.location.lng(), 6);
        LoRa.endPacket();
    } else {
        Serial.println("GPS location is not valid");
    }
    digitalWrite(13, LOW);
    Serial.println("END");
}

/*
  if (digitalRead(triggerPin) == LOW) {
    Serial.println("Trigger");
    digitalWrite(13, HIGH);
    LoRa.beginPacket();
    LoRa.print("Target: ");
    LoRa.print(target);
    LoRa.print("Lat: ");
    LoRa.print(gps.location.lat(), 6);
    LoRa.print("Lon: ");
    LoRa.print(gps.location.lng(), 6);
    
    LoRa.endPacket();

    digitalWrite(13, LOW);

    Serial.println("END");
  }
  */


if (millis() > 5000 && gps.charsProcessed() < 10)
Serial.println(F("No GPS data received: check wiring"));


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
