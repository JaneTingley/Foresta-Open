/*
 * This sketch requires the modified EspNowMQTTServer .....
 * 
 * 
 * 
 * 
 * This sketch reads the Anemometer: https://www.adafruit.com/product/1733
 * and the TEMT6000 Ambient Light Sensor: https://learn.sparkfun.com/tutorials/temt6000-ambient-light-sensor-hookup-guide/all
 * 
 */


#include <EspMQTTClient.h>
#include "EspNowMQTTServer.h"
#include "battery.h"

// This is used both as a MQTT client name and as a WiFi network name
// for gateway discovery. Make sure it's unique, in case there are multiple MQTT
// gateways running.

//Program this sketch as a firebeetle - not dev kit. The dev kit version does not see A1
#define GATEWAY_NAME "00Name-Gateway"


EspMQTTClient client(
  "wifi-SSID", //SSID
  "wifi-password", //wifi password
  "public.cloud.shiftr.io", //Instance location, 
  "public", // Username,
  "public", // secret password,
  GATEWAY_NAME, // don't change
  1883  // port - don't change
);

EspNowMQTTServer server;

// These are used to keep track of WiFi and MQTT disconnects
int prevWifiConnections = 0;
int currWifiConnections = 1;
int prevMqttConnections = 0;
int currMqttConnections = 0;

unsigned long lastMillis = 0;
int windReading = 0;
int lightReading = 0;

const int batteryPin = A0;
const int windPin = A1;
const int luxPin = A2;
  
void setup() {
  Serial.begin(115200);
  Serial.println("Hello " GATEWAY_NAME);
  pinMode (batteryPin, INPUT);
  pinMode (luxPin, INPUT);
  pinMode (windPin, INPUT);

  client.enableDebuggingMessages(); // Enable debugging messages sent to serial output
  WiFi.onEvent(onWiFiDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
}

void onWiFiDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  if (WiFi.status() == WL_DISCONNECTED) {
    currWifiConnections++;
  }
}

void onConnectionEstablished() {
  Serial.println("Router MAC address: " + WiFi.BSSIDstr());
  server.begin(GATEWAY_NAME);
}

void onMessageReceived(const String &topic, const String &value, bool retain) {
  client.publish(topic, value, retain);
}

// WiFi and MQTT disconnect tracking
void trackDisconnects() {
  currMqttConnections = client.getConnectionEstablishedCount();

  if (client.isConnected()) {
    if (currMqttConnections != prevMqttConnections) {
      if (client.publish(GATEWAY_NAME "/mqttConnections", String(currMqttConnections), 1)) {
  prevMqttConnections = currMqttConnections;
      }
    }
    if (currWifiConnections != prevWifiConnections) {
      if (client.publish(GATEWAY_NAME "/wifiConnections", String(currWifiConnections), 1)) {
  prevWifiConnections = currWifiConnections;
      }
    }
  }
}

void loop() {
  client.loop();
  server.loop();
  trackDisconnects();
  
  if (client.isConnected() && millis() - lastMillis > 1000) {
    lastMillis = millis();

    // TODO: Only send updated values, to maximize MQTT Explorer history
    client.publish(GATEWAY_NAME "/uptime", String(millis()/1000), 1);
    client.publish(GATEWAY_NAME "/MAC", WiFi.macAddress(), 1);
    client.publish(GATEWAY_NAME "/RSSI", String(WiFi.RSSI()), 1);
    client.publish(GATEWAY_NAME "/channel", String(WiFi.channel()), 1);
    const auto batteryReading = analogRead(batteryPin);
    // 2.1 is calibrated on the DevSet gateway
    const auto batteryVoltage = readingToBatteryVoltage(batteryReading, 2.1);
    client.publish(GATEWAY_NAME "/batteryVoltage", String(batteryVoltage), 1);
    client.publish(GATEWAY_NAME "/batteryPercent", String(batteryVoltageToPercent(batteryVoltage)), 1);
    lightReading = analogRead(luxPin);
    client.publish("Topic-Name/Light-ESP", String(lightReading), 1);  // this publishes Light data and the #1 makes the value persistant in Shiftr
    windReading = analogRead(windPin);
    client.publish("Topic-Name/Wind-ESP", String(windReading), 1);  // this publishes Wind data and the #1 makes the value persistant in Shiftr
  }
}
