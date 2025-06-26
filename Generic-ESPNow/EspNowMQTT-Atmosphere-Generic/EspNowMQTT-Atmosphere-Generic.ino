/*

This sketch uses the following sensors:

* PMS5003 Particulate sensor 
* https://learn.adafruit.com/pm25-air-quality-sensor/arduino-code
* Currently requires a custom branch of "PMS Library" Arduino library:
* https://github.com/kintel/PMS/tree/particles

AND

* * The Rain Drop Detection Module:
* * https://docs.sunfounder.com/projects/umsk/en/latest/01_components_basic/15-component_raindrop.html

*/


#include <PMS.h>

#include "EspNowMQTTClient.h"
#include <WiFi.h>
#include <SensirionI2CScd4x.h>
#include <Wire.h>

SensirionI2CScd4x scd4x;
PMS pms(Serial2);
PMS::DATA data;

#define GATEWAY_NAME "00Name-Gateway" // this must be the same as in the gateway sketch

EspNowMQTTClient client;

//This manages the time
unsigned long fastDelayTime = 1000;
unsigned long slowDelayTime = 10000; // THIS SHOULD PUBLISH ONCE AN HOUR
unsigned long previousFastMillis = 0;
unsigned long previousSlowMillis = 0;
const int rainPin = A1;
int rainPinValue = 0;
int sensorControl = D2;

void initParticulateSensor(bool firstBoot)
{
  // Serial2 is connected to GPIO16/17 (FireBeetle)
  Serial2.begin(9600);

  if (firstBoot) {
    pms.passiveMode();
    delay(1000); // FIXME: Do we need this?
  }

  // Somehow passive mode seems to send the sensor to sleep, so we always have to wake it
  pms.wakeUp();
  Serial2.flush();
  // After wakeup, it takes 10 readings for particles to become available
  for (int i=0;i<10;i++) {
    pms.requestRead();
    pms.readUntil(data);
  }  
}

void setup() {
  Serial.begin(115200);
  Serial.println("Hello ESP-NOW Sensor");
  Serial.println("Using gateway " GATEWAY_NAME);
  pinMode(rainPin, INPUT);
  pinMode(sensorControl, OUTPUT);
  digitalWrite(sensorControl, HIGH);
  
  while (!Serial) {
   delay(100);
  }

  // Use the same name as in the MQTT gateway sketch
  client.begin(GATEWAY_NAME);

  initParticulateSensor(true);
}

void loop() {
  client.loop();
  
  unsigned long currentMillis = millis();
  if (currentMillis - previousFastMillis >= fastDelayTime) {
    previousFastMillis = currentMillis;
  }
  if (currentMillis - previousSlowMillis >= slowDelayTime) {
    previousSlowMillis = millis();
      
    rainPinValue =  analogRead (rainPin);
    client.publish("Topic-Name/Rain", String(rainPinValue));

    pms.requestRead();
    if (pms.readUntil(data)) {
      if (data.hasParticles) {
        //Particulate
        client.publish("Topic-Name/Particle-.3", String(data.PM_TOTALPARTICLES_0_3), 1); 
        client.publish("Topic-Name/Particle-.5", String(data.PM_TOTALPARTICLES_0_5), 1);
        client.publish("Topic-Name/Particle-1", String(data.PM_TOTALPARTICLES_1_0), 1);
        client.publish("Topic-Name/Particle-2.5", String(data.PM_TOTALPARTICLES_2_5), 1);
        client.publish("Topic-Name/Particle-5", String(data.PM_TOTALPARTICLES_5_0), 1);
        client.publish("Topic-Name/Particle-10", String(data.PM_TOTALPARTICLES_10_0), 1);

        Serial.println();
        Serial.println("---------------------------------------");
        Serial.print("Particles > 0.3um / 0.1L air:"); Serial.println(data.PM_TOTALPARTICLES_0_3);
        Serial.print("Particles > 0.5um / 0.1L air:"); Serial.println(data.PM_TOTALPARTICLES_0_5);
        Serial.print("Particles > 1.0um / 0.1L air:"); Serial.println(data.PM_TOTALPARTICLES_1_0);
        Serial.print("Particles > 2.5um / 0.1L air:"); Serial.println(data.PM_TOTALPARTICLES_2_5);
        Serial.print("Particles > 5.0um / 0.1L air:"); Serial.println(data.PM_TOTALPARTICLES_5_0);
        Serial.print("Particles > 10.0 um / 0.1L air:"); Serial.println(data.PM_TOTALPARTICLES_10_0);
        Serial.print("Rain : "); Serial.println(rainPinValue);
        Serial.println("---------------------------------------");
        
      }
      else {
        Serial.println("No particles in data.");
      }
      Serial.println();
    }
    else {
      Serial.println("No particulate data");
    }
  }
}
