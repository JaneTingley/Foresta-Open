#include <Arduino.h>
#include <esp_now.h>

class EspNowMQTTClient
{
  esp_now_peer_info_t gateway;

  void InitESPNow();
  bool ScanForGateway(const String& ssid);
public:
  EspNowMQTTClient();

  void begin(const String& ssid);
  void loop();
  void publish(const String &topic, const String &payload, bool retain=false);
    // if the third argument is true then the sensor value is retained on the Shiftr
};
