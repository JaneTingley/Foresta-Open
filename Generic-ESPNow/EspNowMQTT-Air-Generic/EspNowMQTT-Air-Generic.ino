 /*

** This sketch uses the following two sensors:

* True VOC sensor: https://learn.adafruit.com/adafruit-ccs811-air-quality-sensor
(the newer models require that the ake pin is pulled to ground.)

* *  Adafruit SCD-40 - True CO2, Temperature and Humidity Sensor 
//https://www.adafruit.com/product/5187

  */

#include "EspNowMQTTClient.h"
#include <Arduino.h> //CO2
#include <SensirionI2CScd4x.h> //CO2
#include <Wire.h> //CO2
#include "Adafruit_CCS811.h" // VOC
            
Adafruit_CCS811 ccs; //VOC
SensirionI2CScd4x scd4x; //CO2
EspNowMQTTClient client;

#define GATEWAY_NAME "00Name-Gateway" // this must be the same as in the gateway sketch

unsigned long lastMillis = 0;
uint16_t error; //CO2
char errorMessage[256]; //CO2
int sensorControl = D2; // Mosfet to turn on/off board - for deep sleep

void setup() {
  Serial.begin(115200);
  pinMode(sensorControl, OUTPUT);
  digitalWrite(sensorControl, HIGH);
  delay(100);

  Serial.println("Hello ESP-NOW Sensor");
  Serial.println("Using gateway " GATEWAY_NAME);
  Serial.println("CCS811 test");

  while (!Serial) {
    delay(100);
  }
// Use the same name as in the MQTT gateway sketch
  client.begin(GATEWAY_NAME);
  
  
  Wire.begin();
  scd4x.begin(Wire);


 // stop potentially previously started measurement
  error = scd4x.stopPeriodicMeasurement();
  if (error) {
      Serial.print("Error trying to execute stopPeriodicMeasurement(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
  }

  // Start Measurement
  error = scd4x.startPeriodicMeasurement();
  if (error) {
      Serial.print("Error trying to execute startPeriodicMeasurement(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
  }

  Serial.println("Waiting for first measurement... (5 sec)");

  if(!ccs.begin()){
    Serial.println("Failed to start sensor! Please check your wiring.");
    while(1);
  }
  // Wait for the sensor to be ready
  while(!ccs.available());
}

void loop() {
  client.loop();

  if (millis() - lastMillis > 10000) { // this should send once an hour
    lastMillis = millis();
    // Read Measurement
    uint16_t co2;
    float temperature;
    float humidity;
    error = scd4x.readMeasurement(co2, temperature, humidity);
    if (error) {
        Serial.print("Error trying to execute readMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else if (co2 == 0) {
        Serial.println("Invalid sample detected, skipping.");
    } else {
        Serial.print("Co2:");
        Serial.print(co2);
        client.publish("Topic-Name/co2", String(co2));  // C02 Readings
        Serial.print("\t");
        Serial.print("Temperature:");
        Serial.print(temperature);
        client.publish("Topic-Name/temperature", String(temperature));
        Serial.print("\t");
        Serial.print("Humidity:");
        Serial.println(humidity);
        client.publish("Topic-Name/humidity", String(humidity));
    }
        
// only get the True VOCs from this sensor
  if(ccs.available()){
    if(!ccs.readData()){
      //Serial.print("CO2: ");
      //Serial.print(ccs.geteCO2());
      Serial.print("ppm, TVOC: ");
      Serial.println(ccs.getTVOC());
      client.publish("Topic-Name/TVOC", String(ccs.getTVOC())); // VOC Readings
    }
    else{
      Serial.println("ERROR!");
      while(1);
    }
  }
  delay(500); 
  }
}
