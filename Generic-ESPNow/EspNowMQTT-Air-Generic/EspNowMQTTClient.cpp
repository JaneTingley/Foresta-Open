#include "EspNowMQTTClient.h"
#include <esp_wifi.h>
#include <WiFi.h>

EspNowMQTTClient::EspNowMQTTClient()
{
}

// Init ESP Now with fallback
void EspNowMQTTClient::InitESPNow()
{
  WiFi.disconnect();
  esp_wifi_set_channel(this->gateway.channel, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init OK");
  }
  else {
    Serial.println("ESPNow Init failed");
    ESP.restart();
  }
}

// Scan for a Gateway.
// The gateway is a soft Access Point with
// a pre-shared SSID and password.
bool EspNowMQTTClient::ScanForGateway(const String& ssid)
{
  memset(&this->gateway, 0, sizeof(this->gateway));

  bool found = false;
  int8_t scanResults = WiFi.scanNetworks();
  if (scanResults == 0) {
    Serial.println("No networks found");
  } else {
    Serial.print("Found "); Serial.print(scanResults); Serial.println(" devices ");
    for (int i = 0; i < scanResults; i++) {
      if (WiFi.SSID(i) == ssid) {
        Serial.print("Found gateway on channel ");
        Serial.print(WiFi.channel(i));
        Serial.print(": ");
        Serial.print(WiFi.SSID(i));
        Serial.print("   ");
        Serial.println(WiFi.BSSIDstr(i));
        int mac[6];
        if (sscanf(WiFi.BSSIDstr(i).c_str(), "%x:%x:%x:%x:%x:%x",
                   &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]) == 6) {
          for (int j = 0; j < 6; j++) {
            this->gateway.peer_addr[j] = (uint8_t)mac[j];
          }
          this->gateway.channel = WiFi.channel(i);
          this->gateway.encrypt = 0;
          found = true;
          break;
        }
      }
    }
  }
  WiFi.scanDelete();
  if (!found) {
    Serial.println("No gateway found");
  }

  return found;
}

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
}

void EspNowMQTTClient::begin(const String& ssid)
{
  WiFi.mode(WIFI_STA);

  if (this->ScanForGateway(ssid)) {
    InitESPNow();
    esp_now_register_send_cb(OnDataSent);

    esp_err_t addStatus = esp_now_add_peer(&gateway);
    if (addStatus == ESP_OK) {
      Serial.println("Paired with gateway");
    } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
      Serial.println("ESP-NOW not initialized");
    } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
      Serial.println("Invalid Argument");
    } else {
      Serial.println("Failed to peer with gateway");
    }
  }
}

// Publish an MQTT value over ESP-NOW
void EspNowMQTTClient::publish(const String &topic, const String &payload, bool retain)
{
  // ESP-NOW message format:
  // Regular messages: "topicstring=valuestring\0"
  // Retained messages: "topicstring:=valuestring\0"
  String packet = topic + (retain?":":"") + "=" + payload;
  Serial.print("Publish: " + packet);
  esp_err_t result = esp_now_send(gateway.peer_addr, (uint8_t *)packet.c_str(), packet.length()+1);

  // FIXME: If we experience excessive errors, try to rescan the gateway
  // o Scan can probably be done asynchronously
  // o If channel changed, update channel and continue
  // o If gateway is not found, report error and keep retrying
  
  Serial.print("Send Status: ");
  if (result == ESP_OK) {
    Serial.println("Success");
  } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
    Serial.println("ESP-NOW not initialized");
  } else if (result == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
    Serial.println("Internal Error");
  } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Unknown error");
  }
}

void EspNowMQTTClient::loop()
{
  // TODO: Here we could monitor the messages sent to listen for persistent transmission errors.
  // If a transmission error occurs, it could be a sign that the gateway has changed channels, or that we
  // in general need to reconnect.
}
