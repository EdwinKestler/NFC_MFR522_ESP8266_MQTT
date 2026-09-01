#pragma once

#include "settings.h"

#ifndef DEBUG_SERIAL
#define DEBUG_SERIAL 1
#endif

#include <SoftwareSerial.h>
extern SoftwareSerial DebugSerial;

#if DEBUG_SERIAL
#define DBG_PRINT(...)    DebugSerial.print(__VA_ARGS__)
#define DBG_PRINTLN(...)  DebugSerial.println(__VA_ARGS__)
#else
#define DBG_PRINT(...)    ((void)0)
#define DBG_PRINTLN(...)  ((void)0)
#endif
