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
#include <WiFi.h>
#include "battery.h"

Adafruit_CCS811 ccs; //VOC
SensirionI2CScd4x scd4x; //CO2
EspNowMQTTClient client;

#define GATEWAY_NAME "00Name-Gateway" // this must be the same as in the gateway sketch
#define POD_NAME "air"

// How long the ESP will sleep for (measured from entering sleep to waking up)
#define DEEP_SLEEP_SECONDS   59*60
// How long the ESP will stay awake for (measured from time of wakeup to time of sleep)
#define AWAKE_SECONDS        60
// Delay between each sensor reading while the sensor is awake
#define SENSOR_DELAY_SECONDS 10

RTC_DATA_ATTR int bootCount = 0;

unsigned long lastMillis = 0;

const int batteryPin = A0;
uint16_t error; //CO2
char errorMessage[256]; //CO2
int sensorOn = D2; // Mosfet to turn on/off board - for deep sleep
bool sentMAC = false;

void print_wakeup_reason()
{
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

void setup() {
  bootCount++;

  Serial.begin(115200);
  Serial.println("Hello ESP-NOW " POD_NAME " pod (boot #" + String(bootCount) + ")");
  print_wakeup_reason();
  Serial.println("Using gateway " GATEWAY_NAME);

  pinMode(sensorOn, OUTPUT);
  digitalWrite(sensorOn, HIGH);
  delay(100);

  // Try connecting for 10 seconds, then go back to sleep
  while (millis() < 10000) {
    if (client.begin(GATEWAY_NAME)) break;
  }
  if (!client.isConnected()) sleep();
  
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

void sleep() {
  Serial.println("Entering Deep Sleep for " + String(DEEP_SLEEP_SECONDS) + " seconds");
  ESP.deepSleep(DEEP_SLEEP_SECONDS * 1000000ULL);
}

void loop() {
  client.loop();

  unsigned long currentMillis = millis();
  // Publish our MAC address to the Gateway's MQTT topic once per wakeup
  if (client.isConnected() && !sentMAC) {
    client.publish(GATEWAY_NAME "/" POD_NAME "/MAC", WiFi.macAddress(), 1);
    client.publish(GATEWAY_NAME "/" POD_NAME "/wakeups", String(bootCount), 1);
    sentMAC = true;
  }
  
  if (client.isConnected() && currentMillis - lastMillis >= SENSOR_DELAY_SECONDS * 1000) {
    lastMillis = millis();
    
    const auto batteryReading = analogRead(batteryPin);
    // TODO: Calibrate (2.1 is calibrated on the DevSet gateway)
    const auto batteryVoltage = readingToBatteryVoltage(batteryReading, 2.1);
    client.publish(GATEWAY_NAME "/" POD_NAME "/batteryVoltage", String(batteryVoltage), 1);
    client.publish(GATEWAY_NAME "/" POD_NAME "/batteryPercent", String(batteryVoltageToPercent(batteryVoltage)), 1);

    // Read Measurement
    uint16_t co2;
    float temperature;
    float humidity;
    error = scd4x.readMeasurement(co2, temperature, humidity);
    if (error) {
        Serial.print("Error trying to execute CO2 readMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else if (co2 == 0) {
        Serial.println("Invalid sample detected, skipping.");
    } else {
        Serial.print("Co2:");
        Serial.print(co2);
        client.publish("Topic-Name/co2", String(co2), 1); // Publishes CO2 - the #1 makes the value persistent in Shiftr
        Serial.print("\t");
        Serial.print("Temperature:");
        Serial.print(temperature);
        client.publish("Topic-Name/temperature", String(temperature), 1); // Publishes air temp - the #1 makes the value persistent in Shiftr
        Serial.print("\t");
        Serial.print("Humidity:");
        Serial.println(humidity);
        client.publish("Topic-Name/humidity", String(humidity), 1); // Publishes air humidity - the #1 makes the value persistent in Shiftr
    }
        
    // only get the True VOCs from this sensor
    if (ccs.available()) {
      if (!ccs.readData()) {
        //Serial.print("CO2: ");
        //Serial.print(ccs.geteCO2());
        Serial.print("ppm, TVOC: ");
        Serial.println(ccs.getTVOC());
        client.publish("Topic-Name/TVOC", String(ccs.getTVOC()), 1); // Publishes VOCs - the #1 makes the value persistent in Shiftr
      }
      else{
        Serial.println("ERROR!");
      }
    }
  }

  // Go to sleep after AWAKE_SECOND seconds
  if (millis() >= AWAKE_SECONDS * 1000) {
    // TODO: This may be unnecessary, we don't quite know how GPIO works during deep sleep
    digitalWrite(sensorOn, LOW);
    sleep();    
  }
}
