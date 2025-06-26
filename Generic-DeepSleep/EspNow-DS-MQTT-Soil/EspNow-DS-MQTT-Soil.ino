/*this sketch is for the Veggietronix sensors:
 * Soil humidity: https://www.vegetronix.com/Products/VH400/
 * Soil Temp:https://www.vegetronix.com/Products/THERM200/
 * For ESP32: https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
 ESP-WROOM-32 
  */

#include "EspNowMQTTClient.h"
#include <WiFi.h>
#include "battery.h"

// this must be the same as in the gateway sketch
#define GATEWAY_NAME "00Name-Gateway"
#define POD_NAME "soil"

// How long the ESP will sleep for (measured from entering sleep to waking up)
#define DEEP_SLEEP_SECONDS   59*60
// How long the ESP will stay awake for (measured from time of wakeup to time of sleep)
#define AWAKE_SECONDS        60
// Delay between each sensor reading while the sensor is awake
#define SENSOR_DELAY_SECONDS 10

RTC_DATA_ATTR int bootCount = 0;

EspNowMQTTClient client;

unsigned long lastMillis = 0;

const int batteryPin = A0;
const int soiHumidPin = A2; //https://www.vegetronix.com/Products/VH400/
int soilHumidPinValue = 0;
const int soilTempPin = A1; //https://www.vegetronix.com/Products/THERM200/
int soilTempPinValue = 0;
int soilTempAdjustValue = 0;
const int sensorOn = D2; // this is the mosfet that pulls the sensor to ground
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

  pinMode (batteryPin, INPUT);

  // Try connecting for 10 seconds, then go back to sleep
  while (millis() < 10000) {
    if (client.begin(GATEWAY_NAME)) break;
  }
  if (!client.isConnected()) sleep();

  pinMode(soiHumidPin, INPUT);
  pinMode(soilTempPin, INPUT);
  
  pinMode(sensorOn, OUTPUT);
  digitalWrite(sensorOn, HIGH);
}

void sleep() {
  Serial.println("Entering Deep Sleep for " + String(DEEP_SLEEP_SECONDS) + " seconds");
  ESP.deepSleep(DEEP_SLEEP_SECONDS * 1000000ULL);
}

void loop() {
  client.loop();

  // Take and publish sensor readings every SENSOR_DELAY_SECONDS seconds.
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

    soilHumidPinValue =  analogRead(soiHumidPin);
    soilTempPinValue =  analogRead(soilTempPin);
    // adjust according to https://www.vegetronix.com/Products/THERM200/
    soilTempAdjustValue = ((soilTempPinValue*.00081)*41.67+5);  // changed until sensor is fixed - should be -40
    
    client.publish("Topic-Name/soilHumid", String(soilHumidPinValue), 1); //Publishes soil humidity and #1 makes the value persistant in Shiftr
    client.publish("Topic-Name/soilTemp", String(soilTempAdjustValue), 1); // Publishes soil temperature and #1 makes the value persistant in Shiftr
  }

  // Go to sleep after AWAKE_SECOND seconds
  if (millis() >= AWAKE_SECONDS * 1000) sleep();
}
