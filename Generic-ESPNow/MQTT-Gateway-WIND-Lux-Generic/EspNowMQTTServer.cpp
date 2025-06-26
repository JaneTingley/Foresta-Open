#include "EspNowMQTTServer.h"
#include <esp_now.h>
#include <WiFi.h>

// FIXME: We could move these into the class and access the class instance
// through a singleton.
QueueHandle_t queue;
bool debug;

// Called asynchronously every time a message arrives over ESP-NOW.
// FIXME: What is the context in which this is called? Interrupt?
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *packet, int packet_len)
{
  // Since this is called asynchronously, and can interrupt the main loop,
  // we need this to be interrupt-safe. We're using an xQueue to queue
  // all incoming messages for processing in the main loop.

  // Copy data into a block matching the queue item size and add to queue
  // We add one byte to the max. ESP-NOW message size for zero-termination purposes.
  uint8_t data[ESP_NOW_MAX_DATA_LEN+1];
  memcpy(data, packet, packet_len);
  data[packet_len] = '\0';
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (xQueueSendFromISR(queue, data, &xHigherPriorityTaskWoken) != pdTRUE) {
    Serial.println("ESP-NOW queue full. Dropping message");
  }
  else if (debug) {
    Serial.println("ESP-NOW message queued. " + String(uxQueueMessagesWaitingFromISR(queue)) + " messages queued.");
  }

  if (debug) {
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
	     mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    Serial.println("");
    Serial.print("Recv from: "); Serial.println(macStr);
    Serial.print("["); Serial.print(packet_len); Serial.print("]: ");
    Serial.println(String((const char *)packet));
  }
}

// Init ESP Now with fallback
void InitESPNow() {
//  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init OK");
  }
  else {
    Serial.println("ESPNow Init failed");
  }
}

void EspNowMQTTServer::configSoftAP(const String& ssid)
{
  int hidden = 0;
  bool result = WiFi.softAP(ssid.c_str(), "jSFJ^38^dsj", this->channel, hidden);
  if (!result) {
    Serial.println("AP Config failed");
  } else {
    Serial.println("AP Config OK");
  }
}

EspNowMQTTServer::EspNowMQTTServer()
{
  // Support 10 ESP-NOW messages in flight
  queue = xQueueCreate(10, ESP_NOW_MAX_DATA_LEN+1);
}

void EspNowMQTTServer::enableDebuggingMessages()
{
  this->debug = true;
  debug = true;
}

void EspNowMQTTServer::begin(const String& ssid)
{
  this->channel = WiFi.channel();
  Serial.print("Connected on WiFi channel ");
  Serial.println(this->channel);

  this->configSoftAP(ssid);
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());

  InitESPNow();
  esp_now_register_recv_cb(OnDataRecv);
}

// Call this as often as possible, to avoid ESP-NOW messages accumulating past the size of the queue
void EspNowMQTTServer::loop()
{
  uint8_t packet[ESP_NOW_MAX_DATA_LEN+1];

  if (this->debug) {
    auto pending = uxQueueMessagesWaiting(queue);
    if (pending > 0) {
      Serial.println("loop: " + String(pending) + " ESP-NOW messages pending. Processing...");
    }
  }

  // Process all pending messages
  while (xQueueReceive(queue, packet, 0)) {
    // ESP-NOW message format:
    // Regular messages: "topicstring=valuestring\0"
    // Retained messages: "topicstring:=valuestring\0"
    String payload((const char *)packet);
  
    bool retain = false;
    int eq = payload.indexOf('=');
    if (eq == 0 || eq == -1) {
      Serial.println("Unexpected ESP-NOW packet: " + payload);
      return;
    }
    String value = payload.substring(eq+1);
    if (payload.charAt(eq-1) == ':') {
      retain = true;
      eq--;
    }
    String topic = payload.substring(0, eq);
  
    onMessageReceived(topic, value, retain);
  }
}
