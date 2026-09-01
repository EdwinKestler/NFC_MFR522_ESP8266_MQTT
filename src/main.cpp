/*
  NFC RFID node — ESP8266 (Wemos D1 mini)

  Created by Edwin Kestler & Dennis Revolorio, 2018.
  Refactored to a non-blocking OOP state machine.

  Flow: Boot → WiFi → NTP → MQTT subscribe → Idle
  Idle publishes tag reads, periodic health, and queued remote alarms.
*/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include "mqtt5_client.h"
#include <TimeLibEsp.h>
#include <WiFiUdp.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <WiFiManager.h>
#include <SoftwareSerial.h>
#include <string.h>
#include <stdio.h>

#include "settings.h"
#include "hardware.h"
#include "node_debug.h"

SoftwareSerial DebugSerial(PIN_DBG_RX, PIN_DBG_TX, false);

static const char kWifiApName[] = "flatwifi";
static const char kMqttUser[]   = "flatboxadmin";
static const char kMqttPass[]   = "FBx_admin2012";
static const uint8_t kNtpPacketSize = 48;

enum class AppState : uint8_t {
  WaitNtp,
  MqttConnect,
  Idle,
  TransmitCard,
  UpdateDevice,
  TransmitDevice,
  TransmitAlarm,
  SyncTime,
  MqttReconnect
};

static const char* stateName(AppState s) {
  switch (s) {
    case AppState::WaitNtp:        return "WAIT_NTP";
    case AppState::MqttConnect:    return "MQTT_CONNECT";
    case AppState::Idle:           return "IDLE";
    case AppState::TransmitCard:   return "TX_CARD";
    case AppState::UpdateDevice:   return "UPDATE_DEV";
    case AppState::TransmitDevice: return "TX_DEVICE";
    case AppState::TransmitAlarm:  return "TX_ALARM";
    case AppState::SyncTime:       return "SYNC_TIME";
    case AppState::MqttReconnect:  return "MQTT_RECONNECT";
    default:                       return "?";
  }
}

class Feedback {
 public:
  void begin() {
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
    pinMode(PIN_LED_R, OUTPUT);
    pinMode(PIN_LED_G, OUTPUT);
    pinMode(PIN_LED_B, OUTPUT);
    leds(false, false, false);
  }

  void leds(bool r, bool g, bool b) {
    digitalWrite(PIN_LED_R, r ? HIGH : LOW);
    digitalWrite(PIN_LED_G, g ? HIGH : LOW);
    digitalWrite(PIN_LED_B, b ? HIGH : LOW);
  }

  void off() {
    leds(false, false, false);
    digitalWrite(PIN_BUZZER, LOW);
    ledUntilMs_ = 0;
    buzzUntilMs_ = 0;
    remainingBeeps_ = 0;
  }

  void solid(bool r, bool g, bool b) {
    leds(r, g, b);
    ledUntilMs_ = 0;
  }

  void flash(bool r, bool g, bool b, uint16_t onMs = LED_FLASH_MS) {
    leds(r, g, b);
    ledUntilMs_ = millis() + onMs;
  }

  void beep(uint16_t onMs, uint8_t count = 1, uint16_t gapMs = BEEP_GAP_MS) {
    onMs_ = onMs;
    gapMs_ = gapMs;
    remainingBeeps_ = count;
    startBeep();
  }

  bool busy() const {
    return ledUntilMs_ != 0 || buzzUntilMs_ != 0 || remainingBeeps_ > 0;
  }

  void playAlarm(uint8_t state) {
    switch (state) {
      case 1:
        flash(true, false, false);
        beep(BEEP_LONG_MS, 2, BEEP_GAP_MS);
        break;
      case 2:
        flash(false, true, false);
        beep(BEEP_SHORT_MS);
        break;
      case 3:
        flash(false, false, true);
        beep(BEEP_SHORT_MS);
        break;
      case 4:
        flash(true, true, true);
        beep(BEEP_SHORT_MS);
        break;
      case 5:
        flash(true, false, true);
        beep(BEEP_SHORT_MS);
        break;
      default:
        break;
    }
  }

  void tick() {
    const uint32_t now = millis();
    if (ledUntilMs_ != 0 && (int32_t)(now - ledUntilMs_) >= 0) {
      leds(false, false, false);
      ledUntilMs_ = 0;
    }
    if (buzzUntilMs_ == 0) {
      return;
    }
    if ((int32_t)(now - buzzUntilMs_) < 0) {
      return;
    }
    if (buzzOn_) {
      digitalWrite(PIN_BUZZER, LOW);
      buzzOn_ = false;
      remainingBeeps_--;
      if (remainingBeeps_ > 0) {
        buzzUntilMs_ = now + gapMs_;
      } else {
        buzzUntilMs_ = 0;
      }
    } else if (remainingBeeps_ > 0) {
      startBeep();
    } else {
      buzzUntilMs_ = 0;
    }
  }

 private:
  void startBeep() {
    digitalWrite(PIN_BUZZER, HIGH);
    buzzOn_ = true;
    buzzUntilMs_ = millis() + onMs_;
  }

  uint32_t ledUntilMs_ = 0;
  uint32_t buzzUntilMs_ = 0;
  uint16_t onMs_ = BEEP_SHORT_MS;
  uint16_t gapMs_ = BEEP_GAP_MS;
  uint8_t remainingBeeps_ = 0;
  bool buzzOn_ = false;
};

class RfidReader {
 public:
  void begin() {
    Serial.begin(RFID_BAUD);
    winLen_ = 0;
    cardLen_ = 0;
    recording_ = false;
    tagReady_ = false;
    lastPokeMs_ = 0;
    rearmAtMs_ = 0;
    poke();
  }

  bool takeTag(char* out, size_t outLen) {
    if (!tagReady_ || out == nullptr || outLen == 0) {
      return false;
    }
    strncpy(out, readyTag_, outLen - 1);
    out[outLen - 1] = '\0';
    tagReady_ = false;
    return true;
  }

  void tick() {
    const uint32_t now = millis();
    while (Serial.available() > 0) {
      push((char)Serial.read());
    }
    if (rearmAtMs_ != 0 && (int32_t)(now - rearmAtMs_) >= 0) {
      poke();
      rearmAtMs_ = 0;
    }
    if (!recording_ && Serial.available() == 0 &&
        (now - lastPokeMs_ >= RFID_POKE_MS)) {
      poke();
    }
  }

 private:
  void poke() {
    Serial.write(RFID_CMD_WAKE);
    lastPokeMs_ = millis();
  }

  void push(char c) {
    if (recording_) {
      if (c == '\n' || c == '\r') {
        if (cardLen_ > 0) {
          card_[cardLen_] = '\0';
          strncpy(readyTag_, card_, sizeof(readyTag_) - 1);
          readyTag_[sizeof(readyTag_) - 1] = '\0';
          tagReady_ = true;
          rearmAtMs_ = millis() + RFID_REARM_MS;
        }
        recording_ = false;
        cardLen_ = 0;
      } else if (cardLen_ + 1U < sizeof(card_)) {
        card_[cardLen_++] = c;
      }
      return;
    }

    if (winLen_ + 1U >= sizeof(win_)) {
      memmove(win_, win_ + 1, sizeof(win_) - 2);
      winLen_ = sizeof(win_) - 2;
    }
    win_[winLen_++] = c;
    win_[winLen_] = '\0';
    if (strstr(win_, "Series Number:") != nullptr) {
      winLen_ = 0;
      win_[0] = '\0';
      recording_ = true;
      cardLen_ = 0;
    }
  }

  char win_[64];
  char card_[32];
  char readyTag_[32];
  uint8_t winLen_ = 0;
  uint8_t cardLen_ = 0;
  bool recording_ = false;
  bool tagReady_ = false;
  uint32_t lastPokeMs_ = 0;
  uint32_t rearmAtMs_ = 0;
};

class NtpClient {
 public:
  void begin(uint16_t localPort) {
    udp_.begin(localPort);
    waiting_ = false;
    synced_ = false;
  }

  void request() {
    while (udp_.parsePacket() > 0) {
      udp_.read(packet_, kNtpPacketSize);
    }
    memset(packet_, 0, kNtpPacketSize);
    packet_[0] = 0b11100011;
    packet_[1] = 0;
    packet_[2] = 6;
    packet_[3] = 0xEC;
    packet_[12] = 49;
    packet_[13] = 0x4E;
    packet_[14] = 49;
    packet_[15] = 52;
    udp_.beginPacket(timeServer, 123);
    udp_.write(packet_, kNtpPacketSize);
    udp_.endPacket();
    sentAtMs_ = millis();
    waiting_ = true;
    DBG_PRINTLN(F("NTP request sent"));
  }

  bool tick() {
    if (!waiting_) {
      return false;
    }
    const int size = udp_.parsePacket();
    if (size >= kNtpPacketSize) {
      udp_.read(packet_, kNtpPacketSize);
      unsigned long secsSince1900 =
          ((unsigned long)packet_[40] << 24) |
          ((unsigned long)packet_[41] << 16) |
          ((unsigned long)packet_[42] << 8) |
          ((unsigned long)packet_[43]);
      const time_t epoch = (time_t)(secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR);
      setTime(epoch);
      waiting_ = false;
      synced_ = true;
      DBG_PRINTLN(F("NTP response OK"));
      return true;
    }
    if (millis() - sentAtMs_ >= NTP_WAIT_MS) {
      waiting_ = false;
      DBG_PRINTLN(F("NTP timeout"));
    }
    return false;
  }

  bool waiting() const { return waiting_; }
  bool synced() const { return synced_; }

 private:
  WiFiUDP udp_;
  uint8_t packet_[kNtpPacketSize];
  uint32_t sentAtMs_ = 0;
  bool waiting_ = false;
  bool synced_ = false;
};

class NodeFirmware {
 public:
  NodeFirmware() {
    self_ = this;
  }

  void setup() {
    fb_.begin();
    fb_.solid(false, true, false);

    DebugSerial.begin(DEBUG_BAUD, SWSERIAL_8N1, PIN_DBG_RX, PIN_DBG_TX, false, 256);
    snprintf(nodeId_, sizeof(nodeId_), "%lu", (unsigned long)ESP.getChipId());

    DBG_PRINTLN(F(""));
    DBG_PRINTLN(F("RFID node boot"));
    DBG_PRINT(F("CHIPID: "));
    DBG_PRINTLN(nodeId_);
    DBG_PRINT(F("HW: "));
    DBG_PRINTLN(HardwareVersion);
    DBG_PRINT(F("FW: "));
    DBG_PRINTLN(FirmwareVersion);
    DBG_PRINT(F("NTP: "));
    DBG_PRINTLN(ntpServerName);
    DBG_PRINT(F("MQTT: "));
    DBG_PRINTLN(MQTTServer);

    fb_.solid(false, false, true);
    ensureWifi();
    cacheNetIdentity();

    rfid_.begin();
    ntp_.begin(localPort);
    buildMqttClientId();
    mqtt_.setServer(MQTTServer, 1883);
    mqtt_.setKeepAlive(60);
    mqtt_.setCallback(mqttThunk);

    ntpRetries_ = 0;
    ntp_.request();
    setState(AppState::WaitNtp);
  }

  void loop() {
    fb_.tick();
    mqtt_.loop();

    if (rebootRequested_) {
      ESP.restart();
    }

    switch (state_) {
      case AppState::WaitNtp:        handleWaitNtp(); break;
      case AppState::MqttConnect:    handleMqttConnect(); break;
      case AppState::Idle:           handleIdle(); break;
      case AppState::TransmitCard:   handleTransmitCard(); break;
      case AppState::UpdateDevice:   handleUpdateDevice(); break;
      case AppState::TransmitDevice: handleTransmitDevice(); break;
      case AppState::TransmitAlarm:  handleTransmitAlarm(); break;
      case AppState::SyncTime:       handleSyncTime(); break;
      case AppState::MqttReconnect:  handleMqttReconnect(); break;
    }

    yield();
  }

 private:
  static NodeFirmware* self_;
  static void mqttThunk(const char* topic, const uint8_t* payload, uint16_t length) {
    if (self_ != nullptr) {
      self_->onMqtt(topic, payload, length);
    }
  }

  void setState(AppState next) {
    if (next == state_) {
      return;
    }
    DBG_PRINT(F("FSM "));
    DBG_PRINT(stateName(state_));
    DBG_PRINT(F(" -> "));
    DBG_PRINTLN(stateName(next));
    state_ = next;
  }

  void ensureWifi() {
    if (WiFi.status() == WL_CONNECTED) {
      return;
    }
    // WiFiManager's portal has no async API; this is the only blocking wait left.
    WiFiManager wifiManager;
    fb_.flash(false, false, true);
    if (!wifiManager.autoConnect(kWifiApName)) {
      fb_.solid(true, false, true);
      if (!wifiManager.startConfigPortal(kWifiApName)) {
        ESP.restart();
      }
    }
  }

  void cacheNetIdentity() {
    const String ip = WiFi.localIP().toString();
    const String mac = WiFi.macAddress();
    strncpy(ip_, ip.c_str(), sizeof(ip_) - 1);
    strncpy(mac_, mac.c_str(), sizeof(mac_) - 1);
    ip_[sizeof(ip_) - 1] = '\0';
    mac_[sizeof(mac_) - 1] = '\0';
    DBG_PRINTLN(WiFi.SSID());
    DBG_PRINTLN(WiFi.RSSI());
    DBG_PRINTLN(ip_);
    DBG_PRINTLN(mac_);
  }

  void buildMqttClientId() {
    snprintf(mqttClientId_, sizeof(mqttClientId_), "d:%s:%s:%s%s",
             ORG, DEVICE_TYPE, DEVICE_ID, nodeId_);
  }

  bool mqttConnectOnce() {
    DBG_PRINT(F("MQTT connect "));
    DBG_PRINTLN(MQTTServer);
    fb_.flash(true, true, true);
#if defined(internetS)
    return mqtt_.connect(mqttClientId_, kMqttUser, kMqttPass);
#else
    return mqtt_.connect(mqttClientId_, "", "");
#endif
  }

  void subscribeAll() {
    if (!mqtt_.subscribe(responseTopic)) {
      DBG_PRINTLN(F("sub response FAIL"));
    }
    if (!mqtt_.subscribe(rebootTopic)) {
      DBG_PRINTLN(F("sub reboot FAIL"));
    }
    if (!mqtt_.subscribe(updateTopic)) {
      DBG_PRINTLN(F("sub update FAIL"));
    }
    publishManageMetadata();
  }

  void publishManageMetadata() {
    JsonDocument doc;
    JsonObject d = doc["d"].to<JsonObject>();
    JsonObject metadata = d["metadata"].to<JsonObject>();
    metadata["UInterval"] = UInterval;
    metadata["UPDATETIME"] = 60 * UInterval;
    metadata["NResetTIME"] = 60 * 60 * UInterval;
    metadata["timeZone"] = timeZone;
    JsonObject supports = d["supports"].to<JsonObject>();
    supports["deviceActions"] = true;
    JsonObject deviceInfo = d["deviceInfo"].to<JsonObject>();
    deviceInfo["ntpServerName"] = ntpServerName;
    deviceInfo["server"] = MQTTServer;
    deviceInfo["MacAddress"] = mac_;
    deviceInfo["IPAddress"] = ip_;
    char buff[500];
    serializeJson(doc, buff, sizeof(buff));
    sent_++;
    if (mqtt_.publish(manageTopic, buff)) {
      published_++;
      failed_ = 0;
    } else {
      failed_++;
      DBG_PRINTLN(F("manage publish FAIL"));
    }
  }

  void onMqtt(const char* topic, const uint8_t* payload, uint16_t length) {
    if (strcmp(rebootTopic, topic) == 0) {
      DBG_PRINTLN(F("remote reboot queued"));
      rebootRequested_ = true;
      return;
    }

    JsonDocument doc;
    const DeserializationError err = deserializeJson(doc, payload, length);
    if (err) {
      DBG_PRINT(F("JSON parse FAIL: "));
      DBG_PRINTLN(err.c_str());
      return;
    }

    if (strcmp(updateTopic, topic) == 0) {
      return;
    }
    if (strcmp(responseTopic, topic) == 0) {
      handleAlarmJson(doc);
    }
  }

  void handleAlarmJson(JsonDocument& doc) {
    if (!chipIdMatches(doc)) {
      return;
    }
    if (doc["ALARM_STATE"].isNull()) {
      return;
    }
    const int alarm = doc["ALARM_STATE"].as<int>();
    if (alarm < 1 || alarm > 5) {
      DBG_PRINTLN(F("ALARM_STATE out of range"));
      return;
    }
    pendingAlarm_ = (uint8_t)alarm;
    alarmQueued_ = true;
  }

  bool chipIdMatches(JsonDocument& doc) {
    JsonVariant v = doc["CHIPID"];
    if (v.isNull()) {
      v = doc["ChipID"];
    }
    if (v.isNull()) {
      return false;
    }
    if (v.is<const char*>()) {
      const char* asText = v.as<const char*>();
      return asText != nullptr && strcmp(asText, nodeId_) == 0;
    }
    char tmp[16];
    snprintf(tmp, sizeof(tmp), "%ld", v.as<long>());
    return strcmp(tmp, nodeId_) == 0;
  }

  void formatIso8601(char* out, size_t outLen) {
    if (timeStatus() == timeNotSet) {
      strncpy(out, "unsynced", outLen - 1);
      out[outLen - 1] = '\0';
      return;
    }
    snprintf(out, outLen, "%04d-%02d-%02dT%02d:%02d:%02d",
             year(), month(), day(), hour(), minute(), second());
  }

  float readBatteryVolts() {
    const uint16_t raw = (uint16_t)analogRead(PIN_VBAT);
    const float volt = (float)raw / 221.93f;
    if (volt < 0.5f || volt > 5.5f) {
      DBG_PRINTLN(F("VBAT reading rejected"));
      return -1.0f;
    }
    return volt;
  }

  bool mqttReady() {
    if (mqtt_.connected()) {
      mqttRetries_ = 0;
      return true;
    }
    resumeAfterMqtt_ = state_;
    setState(AppState::MqttReconnect);
    return false;
  }

  void publishDevice(const char* msg) {
    char iso[24];
    formatIso8601(iso, sizeof(iso));
    const int16_t rssi = (int16_t)WiFi.RSSI();
    float vbat = vbat_;
    if (vbat < 0.0f) {
      vbat = 0.0f;
    }

    JsonDocument doc;
    JsonObject d = doc["d"].to<JsonObject>();
    JsonObject Ddata = d["Ddata"].to<JsonObject>();
    Ddata["ChipID"] = nodeId_;
    Ddata["Msg"] = msg;
    Ddata["batt"] = vbat;
    Ddata["RSSI"] = rssi;
    Ddata["publicados"] = published_;
    Ddata["enviados"] = sent_;
    Ddata["fallidos"] = failed_;
    Ddata["Tstamp"] = iso;
    Ddata["Mac"] = mac_;
    Ddata["Ip"] = ip_;
    char payload[300];
    serializeJson(doc, payload, sizeof(payload));
    sent_++;
    if (mqtt_.publish(manageTopic, payload)) {
      published_++;
      failed_ = 0;
    } else {
      failed_++;
      fb_.flash(true, false, false);
      DBG_PRINTLN(F("device publish FAIL"));
    }
  }

  void publishTag(const char* tag) {
    if (strcmp(oldTag_, tag) == 0) {
      DBG_PRINTLN(F("duplicate tag ignored"));
      return;
    }
    strncpy(oldTag_, tag, sizeof(oldTag_) - 1);
    oldTag_[sizeof(oldTag_) - 1] = '\0';
    eventId_++;

    char iso[24];
    formatIso8601(iso, sizeof(iso));
    char eventKey[24];
    snprintf(eventKey, sizeof(eventKey), "%s%u", nodeId_, (unsigned)eventId_);

    JsonDocument doc;
    JsonObject d = doc["d"].to<JsonObject>();
    JsonObject tagdata = d["tagdata"].to<JsonObject>();
    tagdata["ChipID"] = nodeId_;
    tagdata["IDeventoTag"] = eventKey;
    tagdata["Tstamp"] = iso;
    tagdata["Tag"] = tag;
    char payload[250];
    serializeJson(doc, payload, sizeof(payload));
    sent_++;
    if (mqtt_.publish(publishTopic, payload)) {
      published_++;
      failed_ = 0;
      fb_.flash(false, true, false);
      fb_.beep(BEEP_SHORT_MS);
    } else {
      failed_++;
      oldTag_[0] = '1';
      oldTag_[1] = '\0';
      fb_.flash(true, false, false);
      DBG_PRINTLN(F("tag publish FAIL"));
    }
  }

  void handleWaitNtp() {
    if (ntp_.tick()) {
      ntpRetries_ = 0;
      lastNtpMs_ = millis();
      setState(AppState::MqttConnect);
      return;
    }
    if (ntp_.waiting()) {
      return;
    }
    ntpRetries_++;
    fb_.flash(true, true, false);
    if (ntpRetries_ >= NTP_RETRY_MAX) {
      DBG_PRINTLN(F("NTP giving up; continuing"));
      setState(AppState::MqttConnect);
      return;
    }
    ntp_.request();
  }

  void handleMqttConnect() {
    if (mqtt_.connected()) {
      mqttRetries_ = 0;
      subscribeAll();
      fb_.off();
      lastDeviceMs_ = millis();
      lastNtpMs_ = millis();
      lastHourMs_ = millis();
      lastDedupeMs_ = millis();
      setState(AppState::Idle);
      return;
    }
    if (millis() - lastMqttAttemptMs_ < MQTT_RETRY_MS && lastMqttAttemptMs_ != 0) {
      return;
    }
    lastMqttAttemptMs_ = millis();
    if (mqttConnectOnce()) {
      return;
    }
    mqttRetries_++;
    fb_.flash(true, false, false);
    if (mqttRetries_ > MQTT_RETRY_MAX) {
      ESP.restart();
    }
  }

  void handleIdle() {
    if (WiFi.status() != WL_CONNECTED) {
      if (millis() - lastWifiMs_ >= 5000UL) {
        lastWifiMs_ = millis();
        WiFi.reconnect();
      }
    }
    if (!mqtt_.connected()) {
      resumeAfterMqtt_ = AppState::Idle;
      setState(AppState::MqttReconnect);
      return;
    }

    rfid_.tick();
    char tag[32];
    if (rfid_.takeTag(tag, sizeof(tag))) {
      strncpy(pendingTag_, tag, sizeof(pendingTag_) - 1);
      pendingTag_[sizeof(pendingTag_) - 1] = '\0';
      fb_.flash(false, false, true);
      setState(AppState::TransmitCard);
      return;
    }

    if (alarmQueued_) {
      fb_.playAlarm(pendingAlarm_);
      alarmQueued_ = false;
    }

    if (!fb_.busy()) {
      tickHealth();
    }
    tickHourlyReset();

    if (failed_ >= FAILTRESHOLD) {
      DBG_PRINTLN(F("publish fail threshold; restart"));
      ESP.restart();
    }

    if (millis() - lastDedupeMs_ >= 5UL * UInterval) {
      lastDedupeMs_ = millis();
      oldTag_[0] = '1';
      oldTag_[1] = '\0';
    }
    if (millis() - lastDeviceMs_ >= 30UL * 60UL * UInterval) {
      lastDeviceMs_ = millis();
      setState(AppState::UpdateDevice);
      return;
    }
    if (millis() - lastNtpMs_ >= 60UL * 60UL * UInterval) {
      lastNtpMs_ = millis();
      ntpRetries_ = 0;
      ntp_.request();
      setState(AppState::SyncTime);
    }
  }

  void tickHealth() {
    if (millis() - lastWarningMs_ < UInterval) {
      return;
    }
    lastWarningMs_ = millis();

    const int16_t rssi = (int16_t)WiFi.RSSI();
    if (rssi < RSSI_ALARM_DBM) {
      fb_.flash(true, true, true);
      if (signalBeeps_ < SIGNAL_BEEP_MAX) {
        fb_.beep(BEEP_SHORT_MS);
        signalBeeps_++;
      }
    } else {
      signalBeeps_ = 0;
    }

    vbat_ = readBatteryVolts();
    if (vbat_ >= 0.0f && vbat_ < BATTRESHHOLD) {
      flashWarning_ = true;
      if (battBeeps_ < BATT_BEEP_MAX) {
        fb_.beep(BEEP_SHORT_MS);
        battBeeps_++;
      }
      fb_.flash(true, false, false);
    } else if (vbat_ >= BATTRESHHOLD) {
      flashWarning_ = false;
      battBeeps_ = 0;
      batWarningSent_ = false;
    }
    if (flashWarning_) {
      fb_.flash(true, false, false);
    }
  }

  void tickHourlyReset() {
    if (millis() - lastHourMs_ < 60UL * 60UL * UInterval) {
      return;
    }
    lastHourMs_ = millis();
    hourCount_++;
    if (hourCount_ <= 24) {
      return;
    }
    strncpy(statusMsg_, "24h NReset", sizeof(statusMsg_) - 1);
    vbat_ = readBatteryVolts();
    if (mqtt_.connected()) {
      publishDevice(statusMsg_);
    }
    ESP.restart();
  }

  void handleTransmitCard() {
    if (!mqttReady()) {
      return;
    }
    publishTag(pendingTag_);
    setState(AppState::Idle);
  }

  void handleUpdateDevice() {
    vbat_ = readBatteryVolts();
    const int16_t rssi = (int16_t)WiFi.RSSI();
    strncpy(statusMsg_, "on", sizeof(statusMsg_) - 1);

    if (rssi < RSSI_WEAK_DBM) {
      strncpy(statusMsg_, "LOWiFi", sizeof(statusMsg_) - 1);
      fb_.flash(true, false, false);
      fb_.beep(BEEP_SHORT_MS);
      setState(AppState::TransmitAlarm);
      return;
    }
    if (vbat_ >= 0.0f && vbat_ < BATTRESHHOLD) {
      flashWarning_ = true;
      fb_.beep(BEEP_SHORT_MS);
      strncpy(statusMsg_, "LowBat", sizeof(statusMsg_) - 1);
      if (!batWarningSent_) {
        batWarningSent_ = true;
        setState(AppState::TransmitAlarm);
        return;
      }
      setState(AppState::Idle);
      return;
    }
    if (vbat_ >= BATTRESHHOLD) {
      batWarningSent_ = false;
      flashWarning_ = false;
    }
    setState(AppState::TransmitDevice);
  }

  void handleTransmitDevice() {
    if (!mqttReady()) {
      return;
    }
    publishDevice(statusMsg_);
    setState(AppState::Idle);
  }

  void handleTransmitAlarm() {
    if (!mqttReady()) {
      return;
    }
    publishDevice(statusMsg_);
    setState(AppState::Idle);
  }

  void handleSyncTime() {
    if (ntp_.tick()) {
      ntpRetries_ = 0;
      setState(AppState::Idle);
      return;
    }
    if (ntp_.waiting()) {
      rfid_.tick();
      return;
    }
    ntpRetries_++;
    if (ntpRetries_ >= NTP_RETRY_MAX) {
      DBG_PRINTLN(F("periodic NTP failed"));
      setState(AppState::Idle);
      return;
    }
    ntp_.request();
  }

  void handleMqttReconnect() {
    if (mqtt_.connected()) {
      mqttRetries_ = 0;
      subscribeAll();
      const AppState resume = resumeAfterMqtt_;
      resumeAfterMqtt_ = AppState::Idle;
      setState(resume);
      return;
    }
    if (millis() - lastMqttAttemptMs_ < MQTT_RETRY_MS) {
      rfid_.tick();
      return;
    }
    lastMqttAttemptMs_ = millis();
    fb_.beep(BEEP_SHORT_MS);
    if (mqttConnectOnce()) {
      return;
    }
    mqttRetries_++;
    fb_.flash(true, false, false);
    if (mqttRetries_ > MQTT_RETRY_MAX) {
      ESP.restart();
    }
  }

  AppState state_ = AppState::WaitNtp;
  AppState resumeAfterMqtt_ = AppState::Idle;
  Feedback fb_;
  RfidReader rfid_;
  NtpClient ntp_;
  Mqtt5Client mqtt_;

  char nodeId_[16] = {};
  char mqttClientId_[40] = {};
  char mac_[18] = {};
  char ip_[16] = {};
  char pendingTag_[32] = {};
  char oldTag_[32] = {'1', '\0'};
  char statusMsg_[16] = {'o', 'n', '\0'};

  uint32_t lastDeviceMs_ = 0;
  uint32_t lastNtpMs_ = 0;
  uint32_t lastHourMs_ = 0;
  uint32_t lastDedupeMs_ = 0;
  uint32_t lastWarningMs_ = 0;
  uint32_t lastMqttAttemptMs_ = 0;
  uint32_t lastWifiMs_ = 0;

  uint16_t sent_ = 0;
  uint16_t published_ = 0;
  uint16_t failed_ = 0;
  uint16_t eventId_ = 0;
  uint8_t hourCount_ = 0;
  uint8_t ntpRetries_ = 0;
  uint8_t mqttRetries_ = 0;
  uint8_t signalBeeps_ = 0;
  uint8_t battBeeps_ = 0;
  uint8_t pendingAlarm_ = 0;

  float vbat_ = 0.0f;
  bool alarmQueued_ = false;
  bool rebootRequested_ = false;
  bool flashWarning_ = false;
  bool batWarningSent_ = false;
};

NodeFirmware* NodeFirmware::self_ = nullptr;

/*
  Wiring pinout (see hardware.h for GPIO notes)

  RFID reader  -> UART0 (TX GPIO1 / RX GPIO3) @ 115200
  Debug UART   -> D1 RX / D2 TX @ 9600
  Buzzer       -> D5 (idle LOW)
  LED R/G/B    -> D8 / D7 / D6 (active HIGH)
  Battery      -> A0
*/

static NodeFirmware g_node;

void setup() {
  g_node.setup();
}

void loop() {
  g_node.loop();
}
