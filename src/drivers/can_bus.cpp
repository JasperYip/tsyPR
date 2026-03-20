#include "drivers/can_bus.hpp"
#include <FlexCAN_T4.h>

// Use CAN2 with moderate queue sizes
static FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> g_can;

bool CanBus::begin(uint32_t bitrate, uint8_t rxPin, uint8_t txPin) {
  (void)rxPin;
  (void)txPin;

  g_can.begin();
  g_can.setBaudRate(bitrate);

  // Optional: reject/accept filters later (Phase 2)
  // For now, accept all frames.
  g_can.setMaxMB(16);
  //g_can.enableFIFO();
  //g_can.enableFIFOInterrupt();

  started_ = true;
  return true;
}

bool CanBus::available() {
  if (!started_) return false;
  CAN_message_t msg;
  return g_can.read(msg); // peek-style not available; we'll do read() pattern below instead
}

bool CanBus::read(CanFrame& out) {
  if (!started_) return false;

  CAN_message_t msg;
  if (!g_can.read(msg)) return false;

  out.id = msg.id;
  out.len = msg.len;
  out.extended = msg.flags.extended;
  out.rtr = msg.flags.remote;
  for (uint8_t i = 0; i < msg.len && i < 8; i++) out.data[i] = msg.buf[i];
  return true;
}

bool CanBus::write(const CanFrame& f) {
  if (!started_) return false;

  CAN_message_t msg;
  msg.id = f.id;
  msg.len = (f.len > 8) ? 8 : f.len;
  msg.flags.extended = f.extended;
  msg.flags.remote = f.rtr;

  for (uint8_t i = 0; i < msg.len; i++) msg.buf[i] = f.data[i];

  return g_can.write(msg);
}