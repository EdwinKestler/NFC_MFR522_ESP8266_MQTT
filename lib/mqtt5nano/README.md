MQTT 5 packet codec only, vendored from https://github.com/awootton/mqtt5nano (v0.7.0).

The upstream library also ships EEPROM, WiFi, HTTP, and a command shell. Those are omitted here so this node can keep WiFiManager and its own FSM. Encode/decode uses `mqtt5nano::mqttPacketPieces` (CONNECT, PUBLISH, SUBSCRIBE).
