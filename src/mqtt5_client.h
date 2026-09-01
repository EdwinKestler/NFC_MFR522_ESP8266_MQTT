#pragma once

#include <Arduino.h>
#include <WiFiClient.h>
#include <stdint.h>

// MQTT 5.0 client built on mqtt5nano's packet codec (mqttPacketPieces).
class Mqtt5Client {
 public:
  typedef void (*MessageCallback)(const char* topic, const uint8_t* payload, uint16_t length);

  void setServer(const char* host, uint16_t port);
  void setKeepAlive(uint16_t seconds);
  void setCallback(MessageCallback cb);

  bool connect(const char* clientId, const char* username = nullptr, const char* password = nullptr);
  void disconnect();
  bool connected() const;

  bool publish(const char* topic, const char* payload, bool retain = false);
  bool subscribe(const char* topic, uint8_t qos = 1);

  void loop();

 private:
  static const uint16_t kSinkSize = 768;
  static const uint16_t kAssemblySize = 640;

  bool sendConnect(const char* clientId, const char* username, const char* password);
  bool sendPing();
  void ingest();
  void handlePacket(uint8_t rawType, int bodyLen, int bodyStart);

  WiFiClient tcp_;
  const char* host_ = nullptr;
  uint16_t port_ = 1883;
  uint16_t keepAliveSec_ = 60;
  MessageCallback callback_ = nullptr;
  bool sessionUp_ = false;
  uint16_t packetId_ = 1;
  uint32_t lastOutMs_ = 0;
  uint32_t lastInMs_ = 0;

  char sinkBuf_[kSinkSize];
  uint16_t sinkLen_ = 0;
};
