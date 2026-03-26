#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "config/constants.hpp"
#include "drivers/bms.hpp"

class BmsManager {
public:
    struct Config {
        BmsUart* transport = nullptr;

        // Poll one command every interval. Seven commands -> full snapshot.
        uint32_t poll_interval_ms = 100;

        // Response wait before retrying / giving up this poll slot.
        uint32_t response_timeout_ms = 120;
        uint8_t max_retries = 1;

        // Recoverable event must persist this long (no recovery event) before flagging.
        uint32_t fault_confirm_ms = 2000;

        // Silence-based link timeouts.
        uint32_t timeout_small_ms = config::BMS_TIMEOUT_SMALL_MS;
        uint32_t timeout_large_ms = config::BMS_TIMEOUT_LARGE_MS;
    };

    struct Telemetry {
        bool any_response = false;
        bool full_snapshot_ready = false;

        float pack_voltage_v = 0.0f;
        float pack_current_a = 0.0f;

        uint16_t max_cell_mv = 0;
        uint16_t min_cell_mv = 0;
        uint16_t online_status = 0;

        // 0.1 degC units from TinyBMS INT16 register.
        int16_t pack_temp_dC = 0;
        int16_t ext1_temp_dC = -32768;
        int16_t ext2_temp_dC = -32768;

        uint8_t last_event_id = 0;
        uint32_t bms_timestamp_s = 0;

        uint32_t last_rx_ms = 0;
        uint32_t last_full_snapshot_ms = 0;
    };

    struct Flags {
        bool bms_temp_high = false;
        bool bms_overcurrent = false;
        bool bms_switch_fault = false;
        bool bms_timeout_small = false;
        bool bms_timeout_large = false;
        bool bms_low_voltage = false;
        bool bms_high_voltage = false;
    };

    BmsManager();
    explicit BmsManager(const Config& cfg);

    void configure(const Config& cfg);

    void begin(uint32_t now_ms = millis());
    void reset(uint32_t now_ms = millis());

    // Call from main loop with current millis().
    void update(uint32_t now_ms = millis());

    const Telemetry& telemetry() const { return telemetry_; }
    const Flags& flags() const { return flags_; }

private:
    struct RecoverableState {
        bool active = false;
        bool pending = false;
        uint32_t pending_since_ms = 0;
    };

    static constexpr uint8_t CMD_READ_NEWEST_EVENTS = 0x11;
    static constexpr uint8_t CMD_READ_PACK_VOLTAGE = 0x14;
    static constexpr uint8_t CMD_READ_PACK_CURRENT = 0x15;
    static constexpr uint8_t CMD_READ_MAX_CELL = 0x16;
    static constexpr uint8_t CMD_READ_MIN_CELL = 0x17;
    static constexpr uint8_t CMD_READ_ONLINE_STATUS = 0x18;
    static constexpr uint8_t CMD_READ_TEMPERATURES = 0x1B;

    static constexpr uint8_t kPollCommands[7] = {
        CMD_READ_PACK_VOLTAGE,
        CMD_READ_PACK_CURRENT,
        CMD_READ_MAX_CELL,
        CMD_READ_MIN_CELL,
        CMD_READ_ONLINE_STATUS,
        CMD_READ_TEMPERATURES,
        CMD_READ_NEWEST_EVENTS
    };

    enum SnapshotBit : uint8_t {
        SNAP_VOLTAGE = 1 << 0,
        SNAP_CURRENT = 1 << 1,
        SNAP_MAX_CELL = 1 << 2,
        SNAP_MIN_CELL = 1 << 3,
        SNAP_STATUS = 1 << 4,
        SNAP_TEMPS = 1 << 5,
        SNAP_EVENTS = 1 << 6
    };

    static constexpr uint8_t SNAP_REQUIRED_MASK =
        SNAP_VOLTAGE | SNAP_CURRENT | SNAP_MAX_CELL |
        SNAP_MIN_CELL | SNAP_STATUS | SNAP_TEMPS | SNAP_EVENTS;

    static constexpr uint16_t RX_BUF_SIZE = 384;
    static constexpr uint8_t EVENT_CACHE_SIZE = 16;

    void pullTransport();
    void appendRx(const uint8_t* data, uint16_t len);
    void parseRx(uint32_t now_ms);
    void handleFrame(const uint8_t* frame, uint16_t len, uint32_t now_ms);

    void handleEventsResponse(const uint8_t* frame, uint16_t len, uint32_t now_ms);
    void processEvent(uint32_t event_ts_s, uint8_t event_id, uint32_t now_ms);

    void setRecoverablePending(RecoverableState& state, uint32_t now_ms);
    void clearRecoverable(RecoverableState& state);
    void updateRecoverableFlags(uint32_t now_ms);

    void trySendNext(uint32_t now_ms);
    void handlePendingTimeout(uint32_t now_ms);
    void sendCommand(uint8_t cmd);
    void completePending(uint8_t cmd);
    void advancePoll();

    void markSnapshotBit(uint8_t cmd, uint32_t now_ms);

    bool eventSeen(uint32_t signature) const;
    void rememberEvent(uint32_t signature);

    static uint32_t elapsedMs(uint32_t now, uint32_t since);
    static uint16_t readU16LE(const uint8_t* p);
    static int16_t readI16LE(const uint8_t* p);
    static uint32_t readU24LE(const uint8_t* p);
    static uint32_t readU32LE(const uint8_t* p);
    static float readFloatLE(const uint8_t* p);

    Config cfg_{};
    Telemetry telemetry_{};
    Flags flags_{};

    RecoverableState temp_high_{};
    RecoverableState overcurrent_{};
    RecoverableState low_voltage_{};
    RecoverableState high_voltage_{};

    uint8_t poll_index_ = 0;
    uint8_t snapshot_mask_ = 0;

    bool pending_active_ = false;
    uint8_t pending_cmd_ = 0;
    uint8_t pending_retries_ = 0;
    uint32_t pending_sent_ms_ = 0;
    uint32_t last_poll_ms_ = 0;

    uint32_t last_good_rx_ms_ = 0;

    uint8_t rx_buf_[RX_BUF_SIZE] = {0};
    uint16_t rx_len_ = 0;

    uint32_t event_cache_[EVENT_CACHE_SIZE] = {0};
    uint8_t event_cache_count_ = 0;
    uint8_t event_cache_head_ = 0;
};
