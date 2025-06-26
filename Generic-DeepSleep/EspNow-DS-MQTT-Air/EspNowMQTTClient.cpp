#include "EspNowMQTTClient.h"
#include <esp_wifi.h>
#include <WiFi.h>

EspNowMQTTClient::EspNowMQTTClient()
{
}

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
}

// Init ESP Now with fallback
bool EspNowMQTTClient::initESPNow()
{
  WiFi.disconnect();
  esp_wifi_set_channel(this->gateway.channel, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init OK");
  }
  else {
    Serial.println("ESPNow Init failed");
    return false;
  }

  esp_now_register_send_cb(OnDataSent);

  esp_err_t addStatus = esp_now_add_peer(&gateway);
  if (addStatus == ESP_OK) {
    Serial.println("Paired with gateway");
    return true;
  } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
    Serial.println("ESP-NOW not initialized");
  } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else {
    Serial.println("Failed to peer with gateway");
  }
  return false;
}

// Scan for a Gateway.
// The gateway is a soft Access Point with
// a pre-shared SSID and password.
bool EspNowMQTTClient::scanForGateway(const String& ssid)
{
  bool found = false;
  int8_t scanResults = WiFi.scanNetworks();
  if (scanResults == 0) {
    Serial.println("No networks found");
  } else {
    for (int i = 0; i < scanResults; i++) {
      if (WiFi.SSID(i) == ssid) {
        Serial.println("Found gateway on channel " + String(WiFi.channel(i)) + ": " + 
                     String(WiFi.SSID(i)) + " [" + WiFi.BSSIDstr(i) + "]");
        uint8_t mac[6];
        if (sscanf(WiFi.BSSIDstr(i).c_str(), "%2" SCNx8 ":%2" SCNx8 ":%2" SCNx8 ":%2" SCNx8 ":%2" SCNx8 ":%2" SCNx8,
                   &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]) == 6) {
      	  this->registerGateway(WiFi.channel(i), mac);
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

void EspNowMQTTClient::registerGateway(uint8_t wifiChannel, uint8_t gatewayMAC[6])
{
  this->gateway = esp_now_peer_info_t{
    .peer_addr = {gatewayMAC[0],gatewayMAC[1],gatewayMAC[2],gatewayMAC[3],gatewayMAC[4],gatewayMAC[5]},
    .channel = wifiChannel,
  };
}

/*
 * Scan for gateway. Will return false if something went wrong; 
 * gateway not found, unable to initialize ESP-NOW or add gateway as a peer.
 */
bool EspNowMQTTClient::begin(const String& ssid, uint8_t wifiChannel, uint8_t gatewayMAC[6])
{
  WiFi.mode(WIFI_STA);

  if (wifiChannel == 0 || !gatewayMAC|| gatewayMAC[0] == 0) {
    if (!this->scanForGateway(ssid)) {
      Serial.print("Unable to locate gateway ");
      Serial.println(ssid);
      return false;
    }
  }

  this->connected = initESPNow();
  return this->connected;
}

// Publish an MQTT value over ESP-NOW
void EspNowMQTTClient::publish(const String &topic, const String &payload, bool retain)
{
  // ESP-NOW message format:
  // Regular messages: "topicstring=valuestring\0"
  // Retained messages: "topicstring:=valuestring\0"
  String packet = topic + (retain?":":"") + "=" + payload;
  Serial.println("Publish: " + packet);
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
