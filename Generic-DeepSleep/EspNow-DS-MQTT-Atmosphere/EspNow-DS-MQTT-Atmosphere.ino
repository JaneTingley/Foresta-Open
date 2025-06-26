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
#include <Wire.h>
#include "battery.h"

#define GATEWAY_NAME "00Name-Gateway" // this must be the same as in the gateway sketch
#define POD_NAME "atmosphere"
// How long the ESP will sleep for (measured from entering sleep to waking up)
#define DEEP_SLEEP_SECONDS   59*60
// How long the ESP will stay awake for (measured from time of wakeup to time of sleep)
// Note: This should not be less than "sensorWarmupMillis", otherwise the Particle sensor won't be read
#define AWAKE_SECONDS        60
// Delay between each sensor reading while the sensor is awake
#define SENSOR_DELAY_SECONDS 1

RTC_DATA_ATTR int bootCount = 0;

EspNowMQTTClient client;

PMS pms(Serial2);
PMS::DATA data;

// The Particulate sensor needs 30 seconds to allow the fan to pull in enough air
const int sensorWarmupMillis = 30000;
unsigned long lastMillis = 0;
uint16_t error;
char errorMessage[256];

const int batteryPin = A0;
const int rainPin = A1;
int rainPinValue = 0;
int sensorOn = D2;
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
  bootCount++;
  
  pinMode(batteryPin, INPUT);
  pinMode(sensorOn, OUTPUT);
  digitalWrite(sensorOn, HIGH);

  Serial.begin(115200);
  Serial.println("Hello ESP-NOW " POD_NAME " pod (boot #" + String(bootCount) + ")");
  print_wakeup_reason();
  Serial.println("Using gateway " GATEWAY_NAME);
  
  // Try connecting for 10 seconds, then go back to sleep
  while (millis() < 10000) {
    if (client.begin(GATEWAY_NAME)) break;
  }
  if (!client.isConnected()) sleep();

  initParticulateSensor(bootCount == 1);
  pinMode(rainPin, INPUT);
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
    lastMillis = currentMillis;
    
    const auto batteryReading = analogRead(batteryPin);
    // TODO: Calibrate (2.1 is calibrated on the DevSet gateway)
    const auto batteryVoltage = readingToBatteryVoltage(batteryReading, 2.1);
    client.publish(GATEWAY_NAME "/" POD_NAME "/batteryVoltage", String(batteryVoltage), 1);
    client.publish(GATEWAY_NAME "/" POD_NAME "/batteryPercent", String(batteryVoltageToPercent(batteryVoltage)), 1);

    // Don't read Particles until the sensor has warmed up
    if (millis() >= sensorWarmupMillis) {
      pms.requestRead();
      if (pms.readUntil(data)) {
        if (data.hasParticles) {
          //Particulate
          // All topis publish values - th #1 makess the value persistent in Shiftr
          client.publish("Topic-Name/Particle-.3", String(data.PM_TOTALPARTICLES_0_3), 1); 
          client.publish("Topic-Name/Particle-.5", String(data.PM_TOTALPARTICLES_0_5), 1);
          client.publish("Topic-Name/Particle-1", String(data.PM_TOTALPARTICLES_1_0), 1);
          client.publish("Topic-Name/Particle-2.5", String(data.PM_TOTALPARTICLES_2_5), 1);
          client.publish("Topic-Name/Particle-5", String(data.PM_TOTALPARTICLES_5_0), 1);
          client.publish("Topic-Name/Particle-10", String(data.PM_TOTALPARTICLES_10_0), 1);
        }
        else {
          Serial.println("No particles in data.");
        }
        Serial.println();
      }
      else {
        Serial.println("No particulate data");
      }

      rainPinValue =  analogRead (rainPin);
      client.publish("Topic-Name/Rain", String(rainPinValue), 1);
    }
  }

  // Go to sleep after AWAKE_SECOND seconds
  if (millis() >= AWAKE_SECONDS * 1000) {
    pms.sleep();
    sleep();
  }
}
