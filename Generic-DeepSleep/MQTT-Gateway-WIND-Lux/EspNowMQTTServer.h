#include <Arduino.h>

// MUST be implemented in your sketch. Called when an MQTT message arrives over ESP-NOW
void onMessageReceived(const String &topic, const String &value, bool retain);

class EspNowMQTTServer
{
  int channel = 0;
  bool debug = false;

  void configSoftAP(const String& ssid);
public:
  EspNowMQTTServer();

  void enableDebuggingMessages();
  void begin(const String& ssid);
  void loop();
};
