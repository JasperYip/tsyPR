#pragma once
#include <Arduino.h>
#include <stdint.h>

// We built a non-blocking BmsUart transport layer that 
// initializes Serial2 at 115200 8N1, buffers incoming bytes 
// in a ring buffer, provides a raw frame extraction interface, 
// and includes a CRC16 helper compatible with TinyBMS. This 
// driver strictly handles UART transport and buffering only, 
// leaving protocol parsing, polling logic, and fault handling 
// for a higher-level BMS manager in later phases.

class BmsUart {
public:
  struct Config {
    HardwareSerial* serial;
    uint32_t baud;
    uint16_t rx_buffer_size = 256;
  };

  explicit BmsUart(const Config& cfg);

  void begin();

  // Send raw bytes
  void write(const uint8_t* data, uint16_t len);

  // Call frequently in loop()
  void update();

  // Check if a full frame is available
  bool frameAvailable() const;

  // Copy out received frame
  uint16_t readFrame(uint8_t* out, uint16_t max_len);

  // CRC16 (TinyBMS uses CRC-16-IBM typically)
  static uint16_t crc16(const uint8_t* data, uint16_t len);

private:
  Config cfg_;

  static constexpr uint16_t INTERNAL_BUF_SIZE = 512;
  uint8_t buffer_[INTERNAL_BUF_SIZE];
  uint16_t head_ = 0;
  uint16_t tail_ = 0;

  bool frame_ready_ = false;
};