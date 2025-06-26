#include <Arduino.h>
#include <esp_now.h>

class EspNowMQTTClient
{
  esp_now_peer_info_t gateway;
  bool connected = false;

  bool initESPNow();
  bool scanForGateway(const String& ssid);
  void registerGateway(uint8_t wifiChannel, uint8_t gatewayMAC[6]);
public:
  EspNowMQTTClient();

  bool begin(const String& ssid, uint8_t wifiChannel = 0, uint8_t gatewayMAC[6] = nullptr);
  void loop();
  bool isConnected() const { return this->connected; }
  uint8_t getChannel() const { return this->gateway.channel; }
  const uint8_t *getGatewayMAC() const { return this->gateway.peer_addr; }
  void publish(const String &topic, const String &payload, bool retain=false);
};
