/*this sketch is for the Veggietronix sensors:
 * Soil humidity: https://www.vegetronix.com/Products/VH400/
 * Soil Temp:https://www.vegetronix.com/Products/THERM200/
 * For ESP32: https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
 ESP-WROOM-32 
  */

#include "EspNowMQTTClient.h"
#include <WiFi.h>

#define GATEWAY_NAME "00Name-Gateway" // this must be the same as in the gateway sketch

EspNowMQTTClient client;

unsigned long lastMillis = 0;
const int soiHumidPin = A2; //https://www.vegetronix.com/Products/VH400/
int soilHumidPinValue = 0;
const int soilTempPin = A1; //https://www.vegetronix.com/Products/THERM200/
int soilTempPinValue = 0;
int soilTempAdjustValue = 0;
const int sensorOn = D2; // this is the mosfet that pulls the sensor to ground


void setup() {
  Serial.begin(115200);
  pinMode(soiHumidPin, INPUT);
  pinMode(soilTempPin, INPUT);
  pinMode(sensorOn, OUTPUT);
  Serial.println("Hello ESP-NOW Sensor");
  Serial.println("Using gateway " GATEWAY_NAME);
  digitalWrite(sensorOn, HIGH);

  // Use the same name as in the MQTT gateway sketch
  client.begin(GATEWAY_NAME);
  
}

void loop() {
  client.loop();

  if (millis() - lastMillis > 10000) { // once an hour once in deep sleep
    lastMillis = millis();
    
    soilHumidPinValue =  analogRead (soiHumidPin);
    soilTempPinValue =  analogRead (soilTempPin);
    Serial.println (soilTempPinValue);
    
    // adjust according to https://www.vegetronix.com/Products/THERM200/
    soilTempAdjustValue = ((soilTempPinValue*.00081)*41.67-40); 
    
    client.publish("Topic-Name/soilHumid", String(soilHumidPinValue));
    client.publish("Topic-Name/soilTemp", String(soilTempAdjustValue));
  } 
}
