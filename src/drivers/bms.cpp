#include "drivers/bms.hpp"
#include <Arduino.h>

BmsUart::BmsUart(const Config& cfg) : cfg_(cfg) {}

void BmsUart::begin() {
  cfg_.serial->begin(cfg_.baud);
}

void BmsUart::write(const uint8_t* data, uint16_t len) {
  cfg_.serial->write(data, len);
}

void BmsUart::update() {
  while (cfg_.serial->available()) {
    uint8_t byte = cfg_.serial->read();

    buffer_[head_] = byte;
    head_ = (head_ + 1) % INTERNAL_BUF_SIZE;

    // Simple overflow protection
    if (head_ == tail_) {
      tail_ = (tail_ + 1) % INTERNAL_BUF_SIZE;
    }

    // In Phase 1 we just mark that data exists.
    frame_ready_ = true;
  }
}

bool BmsUart::frameAvailable() const {
  return frame_ready_;
}

uint16_t BmsUart::readFrame(uint8_t* out, uint16_t max_len) {
  uint16_t count = 0;

  while (tail_ != head_ && count < max_len) {
    out[count++] = buffer_[tail_];
    tail_ = (tail_ + 1) % INTERNAL_BUF_SIZE;
  }

  frame_ready_ = false;
  return count;
}

uint16_t BmsUart::crc16(const uint8_t* data, uint16_t len) {
  uint16_t crc = 0xFFFF;

  for (uint16_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 1)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }
  return crc;
}