#pragma once
#include <Arduino.h>

// Set to 0 to compile out logs
#ifndef LOG_ENABLED
#define LOG_ENABLED 1
#endif

enum class LogLevel : uint8_t { DEBUG=0, INFO=1, WARN=2, ERROR=3 };

#ifndef LOG_LEVEL
#define LOG_LEVEL static_cast<uint8_t>(LogLevel::INFO)
#endif

namespace logger {

inline void begin(uint32_t baud = 115200) {
#if LOG_ENABLED
  Serial.begin(baud);
  // Optional small delay for USB serial attach
  delay(50);
#endif
}

inline void printPrefix(LogLevel lvl) {
#if LOG_ENABLED
  switch (lvl) {
    case LogLevel::DEBUG: Serial.print("[D] "); break;
    case LogLevel::INFO:  Serial.print("[I] "); break;
    case LogLevel::WARN:  Serial.print("[W] "); break;
    case LogLevel::ERROR: Serial.print("[E] "); break;
  }
#endif
}

inline void log(LogLevel lvl, const __FlashStringHelper* msg) {
#if LOG_ENABLED
  if (static_cast<uint8_t>(lvl) < LOG_LEVEL) return;
  printPrefix(lvl);
  Serial.println(msg);
#endif
}

inline void log(LogLevel lvl, const String& msg) {
#if LOG_ENABLED
  if (static_cast<uint8_t>(lvl) < LOG_LEVEL) return;
  printPrefix(lvl);
  Serial.println(msg);
#endif
}

} // namespace logger

// Convenience macros (use F() to keep RAM low)
#define LOGD(msg) do { logger::log(LogLevel::DEBUG, msg); } while(0)
#define LOGI(msg) do { logger::log(LogLevel::INFO,  msg); } while(0)
#define LOGW(msg) do { logger::log(LogLevel::WARN,  msg); } while(0)
#define LOGE(msg) do { logger::log(LogLevel::ERROR, msg); } while(0)