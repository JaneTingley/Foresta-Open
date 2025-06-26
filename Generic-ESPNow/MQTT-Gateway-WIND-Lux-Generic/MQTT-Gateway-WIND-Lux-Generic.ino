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

const int windPin = A1;
const int luxPin = A2;

void setup() {
  Serial.begin(115200);
  Serial.println("Hello " GATEWAY_NAME);
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
  
  if (millis() - lastMillis > 1000) {
    lastMillis = millis();

    client.publish(GATEWAY_NAME "/uptime", String(millis()/1000));
    client.publish(GATEWAY_NAME "/MAC", WiFi.macAddress());
    client.publish(GATEWAY_NAME "/RSSI", String(WiFi.RSSI()));
    client.publish(GATEWAY_NAME "/channel", String(WiFi.channel()));
    lightReading = analogRead(luxPin); // This is the light sensor
    client.publish("Topic-Name/Light-ESP", String(lightReading)); // this publishes Light data
    windReading = analogRead(windPin); // This is the wind sensor
    client.publish("Topic-Name/Wind-ESP", String(windReading)); //this publishes Wind data
  }
}
