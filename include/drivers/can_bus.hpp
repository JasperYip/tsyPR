#pragma once
#include <stdint.h>

// We separated the CAN implementation into two clear layers: 
// the `can_bus` driver handles only low-level transport 
// (initializing CAN2, sending and receiving raw frames), 
// while all message definitions, IDs, and pack/unpack logic 
// will live under `protocols/can_messages/`. This keeps 
// hardware communication completely independent from 
// application-level message structure, making the system 
//cleaner, easier to modify, and safer to maintain as the CAN 
// protocol evolves.


struct CanFrame {
  uint32_t id = 0;
  uint8_t  len = 0;
  uint8_t  data[8] = {0};
  bool     extended = false;
  bool     rtr = false;
};

class CanBus {
public:
  // CAN2 is fixed for this project
  bool begin(uint32_t bitrate, uint8_t rxPin, uint8_t txPin);

  bool write(const CanFrame& f);
  bool read(CanFrame& out);
  bool available();

private:
  bool started_ = false;
};