// include libraries essential
#include <ArduinoJson.h>
#include "BluetoothSerial.h"
#include <TinyGPS++.h>
#include <stdlib.h>

// enable serial bluetooth ESP32
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
TinyGPSPlus gps;

int byteReceived;
int sts = 0;
float lati = -7.017587971917217;
float longi = 108.63739431304614;

void setup() {
  // Initialize "debug" serial port
  // The data rate must be much higher than the "link" serial port
  Serial.begin(115200);
  //  Serial1.begin(9600);
  SerialBT.begin("ESP32"); //Bluetooth device name
  while (!Serial) continue;

  // Initialize the "link" serial port
  // Use the lowest possible data rate to reduce error ratio
  Serial2.begin(9600);
  Serial.println("ESP32 to Android");
}

void loop() {
  // variable millis
  unsigned long interval = 3000; // the time we need to wait
  unsigned long previousMillis = 0; // millis() returns an unsigned long.
  unsigned long currentMillis = millis(); // grab current time

  // check if "interval" time has passed (1000 milliseconds)
  if ((unsigned long)(currentMillis - previousMillis) >= interval) {
    // Bluetooth read
  if (SerialBT.available()) {
    sts = SerialBT.read();

    Serial.print("data from bluetooth : ");
    Serial.println(sts);
  }

    StaticJsonDocument<200> doc;
    doc["sts"] = sts;
    doc["lat"] = lati;
    doc["lon"] = longi;
    delay(100);
    Serial.print("status : "); Serial.println(sts);
    Serial.print("lat : "); Serial.println(lati);
    Serial.print("lon : "); Serial.println(longi);
    SerialBT.println(sts);
    SerialBT.println(lati);
    SerialBT.println(longi);
    delay(500);

    serializeJsonPretty(doc, Serial2);

    // save the "current" time
    previousMillis = millis();
  }

  // GPS read
  while (Serial1.available() > 0)
    (gps.encode(Serial1.read()));
  
  //Seria2 read
  if (Serial2.available()) {
    byteReceived = Serial2.read(); // Read the byte
    //    Serial.println(byteReceived);
    if (byteReceived == 0) {
      //      digitalWrite(LED, LOW);
    }
    if (byteReceived == 1) {
      //      digitalWrite(LED, HIGH);
    }
    if (byteReceived == 2) {
    }
  }
  if (byteReceived == 3) {

  }
}
