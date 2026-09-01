#include "mqtt5_client.h"

#include <string.h>
#include "mqtt5nanoParse.h"

namespace {

struct TcpDestination : mqtt5nano::Destination {
  WiFiClient* client = nullptr;
  bool writeByte(char c) override {
    if (client == nullptr) {
      return false;
    }
    return client->write((uint8_t)c) == 1;
  }
};

void copySlice(const mqtt5nano::slice& s, char* out, size_t outLen) {
  if (outLen == 0) {
    return;
  }
  const int n = (s.base == nullptr || s.start >= s.end) ? 0 : (int)(s.end - s.start);
  const int copy = (n + 1 > (int)outLen) ? (int)outLen - 1 : n;
  for (int i = 0; i < copy; i++) {
    out[i] = s.base[s.start + i];
  }
  out[copy] = '\0';
}

}  // namespace

void Mqtt5Client::setServer(const char* host, uint16_t port) {
  host_ = host;
  port_ = port;
}

void Mqtt5Client::setKeepAlive(uint16_t seconds) {
  keepAliveSec_ = seconds == 0 ? 60 : seconds;
}

void Mqtt5Client::setCallback(MessageCallback cb) {
  callback_ = cb;
}

bool Mqtt5Client::connected() {
  return sessionUp_ && tcp_.connected();
}

void Mqtt5Client::disconnect() {
  if (tcp_.connected()) {
    const uint8_t disc[] = {0xE0, 0x00};
    tcp_.write(disc, sizeof(disc));
    tcp_.stop();
  }
  sessionUp_ = false;
  sinkLen_ = 0;
}

bool Mqtt5Client::connect(const char* clientId, const char* username, const char* password) {
  disconnect();
  if (host_ == nullptr || clientId == nullptr) {
    return false;
  }
  if (!tcp_.connect(host_, port_)) {
    return false;
  }
  tcp_.setNoDelay(true);
  if (!sendConnect(clientId, username, password)) {
    tcp_.stop();
    return false;
  }
  sessionUp_ = true;
  lastOutMs_ = millis();
  lastInMs_ = millis();
  sinkLen_ = 0;
  return true;
}

bool Mqtt5Client::sendConnect(const char* clientId, const char* username, const char* password) {
  char assembly[kAssemblySize];
  mqtt5nano::ByteCollector sink(assembly, sizeof(assembly));
  mqtt5nano::mqttPacketPieces pkt;
  TcpDestination dest;
  dest.client = &tcp_;

  const char* user = (username != nullptr) ? username : "";
  const char* pass = (password != nullptr) ? password : "";
  // mqtt5nano CONNECT flags always include username+password (0xC2, MQTT 5, clean start).
  const bool ok = pkt.outputConnect(sink, &dest, mqtt5nano::slice(clientId),
                                    mqtt5nano::slice(user), mqtt5nano::slice(pass));
  return ok && tcp_.connected();
}

bool Mqtt5Client::publish(const char* topic, const char* payload, bool retain) {
  if (!connected() || topic == nullptr || payload == nullptr) {
    return false;
  }
  (void)retain;  // mqtt5nano encoder does not expose the retain bit

  char assembly[kAssemblySize];
  mqtt5nano::ByteCollector sink(assembly, sizeof(assembly));
  mqtt5nano::mqttPacketPieces pkt;
  pkt.packetType = mqtt5nano::CtrlPublish;
  pkt.QoS = 1;
  pkt.PacketID = ++packetId_;
  if (packetId_ == 0) {
    packetId_ = 1;
  }
  pkt.TopicName = mqtt5nano::slice(topic);
  pkt.Payload = mqtt5nano::slice(payload);

  TcpDestination dest;
  dest.client = &tcp_;
  const bool ok = pkt.outputPubOrSub(sink, &dest);
  if (ok) {
    lastOutMs_ = millis();
  }
  return ok;
}

bool Mqtt5Client::subscribe(const char* topic, uint8_t qos) {
  if (!connected() || topic == nullptr) {
    return false;
  }
  char assembly[256];
  mqtt5nano::ByteCollector sink(assembly, sizeof(assembly));
  mqtt5nano::mqttPacketPieces pkt;
  pkt.packetType = mqtt5nano::CtrlSubscribe;
  pkt.QoS = (qos == 0) ? 1 : qos;  // encoder writes a packet id; QoS 1 is the mqtt5nano default
  pkt.PacketID = ++packetId_;
  if (packetId_ == 0) {
    packetId_ = 1;
  }
  pkt.TopicName = mqtt5nano::slice(topic);

  TcpDestination dest;
  dest.client = &tcp_;
  const bool ok = pkt.outputPubOrSub(sink, &dest);
  if (ok) {
    lastOutMs_ = millis();
  }
  return ok;
}

bool Mqtt5Client::sendPing() {
  const uint8_t ping[] = {0xC0, 0x00};
  if (tcp_.write(ping, sizeof(ping)) != sizeof(ping)) {
    return false;
  }
  lastOutMs_ = millis();
  return true;
}

void Mqtt5Client::loop() {
  if (!sessionUp_) {
    return;
  }
  if (!tcp_.connected()) {
    sessionUp_ = false;
    return;
  }

  ingest();

  const uint32_t now = millis();
  const uint32_t keepMs = (uint32_t)keepAliveSec_ * 1000UL;
  if (now - lastOutMs_ >= keepMs / 2UL) {
    if (!sendPing()) {
      disconnect();
    }
  }
  if (now - lastInMs_ >= keepMs + 10000UL) {
    disconnect();
  }
}

void Mqtt5Client::ingest() {
  while (tcp_.available() > 0 && sinkLen_ + 1U < kSinkSize) {
    sinkBuf_[sinkLen_++] = (char)tcp_.read();
    lastInMs_ = millis();
  }
  if (sinkLen_ >= kSinkSize - 1) {
    disconnect();
    return;
  }

  while (sinkLen_ >= 2) {
    mqtt5nano::slice pos(sinkBuf_, 0, sinkLen_);
    const uint8_t rawType = pos.readByte();
    const int bodyLen = pos.getLittleEndianVarLenInt();
    if (bodyLen < 0) {
      return;
    }
    const int hdr = pos.start;
    if (sinkLen_ < hdr + bodyLen) {
      return;
    }
    handlePacket(rawType, bodyLen, hdr);
    const int consumed = hdr + bodyLen;
    if (consumed < sinkLen_) {
      memmove(sinkBuf_, sinkBuf_ + consumed, sinkLen_ - consumed);
    }
    sinkLen_ = (uint16_t)(sinkLen_ - consumed);
  }
}

void Mqtt5Client::handlePacket(uint8_t rawType, int bodyLen, int bodyStart) {
  const int packetType = rawType >> 4;
  mqtt5nano::slice body(sinkBuf_, bodyStart, bodyStart + bodyLen);
  mqtt5nano::mqttPacketPieces parser;

  if (packetType == mqtt5nano::CtrlPublish) {
    if (!parser.parse(body, rawType, bodyLen)) {
      return;
    }
    if (callback_ != nullptr && !parser.Payload.empty()) {
      char topic[128];
      copySlice(parser.TopicName, topic, sizeof(topic));
      const uint8_t* payload =
          (const uint8_t*)(parser.Payload.base + parser.Payload.start);
      callback_(topic, payload, (uint16_t)parser.Payload.size());
    }
    if (parser.QoS == 1) {
      uint8_t ack[6];
      ack[0] = 0x40;
      ack[1] = 4;
      ack[2] = (uint8_t)(parser.PacketID >> 8);
      ack[3] = (uint8_t)(parser.PacketID & 0xFF);
      ack[4] = 0x00;
      ack[5] = 0x00;
      tcp_.write(ack, sizeof(ack));
      lastOutMs_ = millis();
    }
    return;
  }

  if (packetType == mqtt5nano::CtrlPingReq) {
    const uint8_t resp[] = {0xD0, 0x00};
    tcp_.write(resp, sizeof(resp));
    lastOutMs_ = millis();
  }
}
