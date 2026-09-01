#pragma once

#include <Arduino.h>
#include <stdint.h>

/*
  Wiring (Wemos D1 mini / ESP8266)

  Function            ESP pin    GPIO    Notes
  ------------------  ---------  ------  --------------------------------
  RFID UART TX/RX     UART0      1 / 3   Hardware Serial, 115200 8N1
  Debug UART RX       D1         5       SoftwareSerial 9600
  Debug UART TX       D2         4       SoftwareSerial 9600
  Buzzer (+)          D5         14      Active-high; idle LOW
  RGB Blue            D6         12      Active-high LED
  RGB Green           D7         13      Active-high LED
  RGB Red             D8         15      Active-high LED; keep LOW at boot
  Battery sense       A0         ADC0    Divider calibrated in firmware
  GND                 GND

  No mechanical user button is wired. D4/GPIO2 is left unused (boot / onboard LED).
*/

static const uint8_t PIN_DBG_RX   = D1;
static const uint8_t PIN_DBG_TX   = D2;
static const uint8_t PIN_BUZZER   = D5;
static const uint8_t PIN_LED_B    = D6;
static const uint8_t PIN_LED_G    = D7;
static const uint8_t PIN_LED_R    = D8;
static const uint8_t PIN_VBAT     = A0;

static const uint32_t RFID_BAUD  = 115200UL;
static const uint32_t DEBUG_BAUD = 9600UL;

static const uint16_t LED_FLASH_MS     = 250;
static const uint16_t BEEP_SHORT_MS    = 200;
static const uint16_t BEEP_LONG_MS     = 500;
static const uint16_t BEEP_GAP_MS      = 50;
static const uint16_t RFID_REARM_MS    = 55;
static const uint16_t NTP_WAIT_MS      = 1500;
static const uint16_t MQTT_RETRY_MS    = 1000;
static const uint8_t  MQTT_RETRY_MAX   = 5;
static const uint8_t  NTP_RETRY_MAX    = 10;
static const uint8_t  SIGNAL_BEEP_MAX  = 4;
static const uint8_t  BATT_BEEP_MAX    = 4;
static const int16_t  RSSI_ALARM_DBM   = -85;
static const int16_t  RSSI_WEAK_DBM    = -75;
static const uint32_t RFID_POKE_MS     = 9000UL;

static const uint8_t RFID_CMD_WAKE     = 0x02;
