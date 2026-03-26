#include "controllers/bms_manager.hpp"

#include <cmath>
#include <cstring>

constexpr uint8_t BmsManager::kPollCommands[7];

BmsManager::BmsManager()
{
    configure(Config{});
}

BmsManager::BmsManager(const Config& cfg)
{
    configure(cfg);
}

void BmsManager::configure(const Config& cfg)
{
    cfg_ = cfg;

    if (cfg_.poll_interval_ms == 0) {
        cfg_.poll_interval_ms = 100;
    }

    if (cfg_.response_timeout_ms == 0) {
        cfg_.response_timeout_ms = 120;
    }

    if (cfg_.max_retries > 5) {
        cfg_.max_retries = 5;
    }

    if (cfg_.fault_confirm_ms == 0) {
        cfg_.fault_confirm_ms = 2000;
    }

    if (cfg_.timeout_small_ms == 0) {
        cfg_.timeout_small_ms = config::BMS_TIMEOUT_SMALL_MS;
    }

    if (cfg_.timeout_large_ms < cfg_.timeout_small_ms) {
        cfg_.timeout_large_ms = cfg_.timeout_small_ms;
    }
}

void BmsManager::begin(uint32_t now_ms)
{
    if (cfg_.transport != nullptr) {
        cfg_.transport->begin();
    }

    reset(now_ms);
}

void BmsManager::reset(uint32_t now_ms)
{
    telemetry_ = Telemetry{};
    flags_ = Flags{};

    temp_high_ = RecoverableState{};
    overcurrent_ = RecoverableState{};
    low_voltage_ = RecoverableState{};
    high_voltage_ = RecoverableState{};

    poll_index_ = 0;
    snapshot_mask_ = 0;

    pending_active_ = false;
    pending_cmd_ = 0;
    pending_retries_ = 0;
    pending_sent_ms_ = 0;
    last_poll_ms_ = now_ms - cfg_.poll_interval_ms;

    last_good_rx_ms_ = now_ms;

    rx_len_ = 0;

    event_cache_count_ = 0;
    event_cache_head_ = 0;
    std::memset(event_cache_, 0, sizeof(event_cache_));
}

void BmsManager::update(uint32_t now_ms)
{
    if (cfg_.transport == nullptr) {
        flags_.bms_timeout_small = true;
        flags_.bms_timeout_large = true;
        return;
    }

    pullTransport();
    parseRx(now_ms);

    handlePendingTimeout(now_ms);
    trySendNext(now_ms);

    updateRecoverableFlags(now_ms);

    const uint32_t silence_ms = elapsedMs(now_ms, last_good_rx_ms_);
    flags_.bms_timeout_small = (silence_ms >= cfg_.timeout_small_ms);
    flags_.bms_timeout_large = (silence_ms >= cfg_.timeout_large_ms);
}

void BmsManager::pullTransport()
{
    cfg_.transport->update();

    while (cfg_.transport->frameAvailable()) {
        uint8_t chunk[256];
        const uint16_t len = cfg_.transport->readFrame(chunk, sizeof(chunk));

        if (len == 0) {
            break;
        }

        appendRx(chunk, len);
        cfg_.transport->update();
    }
}

void BmsManager::appendRx(const uint8_t* data, uint16_t len)
{
    if (data == nullptr || len == 0) {
        return;
    }

    if (len >= RX_BUF_SIZE) {
        std::memcpy(rx_buf_, data + (len - RX_BUF_SIZE), RX_BUF_SIZE);
        rx_len_ = RX_BUF_SIZE;
        return;
    }

    if (static_cast<uint32_t>(rx_len_) + len > RX_BUF_SIZE) {
        const uint16_t overflow =
            static_cast<uint16_t>(static_cast<uint32_t>(rx_len_) + len - RX_BUF_SIZE);

        if (overflow >= rx_len_) {
            rx_len_ = 0;
        } else {
            std::memmove(rx_buf_, rx_buf_ + overflow, rx_len_ - overflow);
            rx_len_ = static_cast<uint16_t>(rx_len_ - overflow);
        }
    }

    std::memcpy(rx_buf_ + rx_len_, data, len);
    rx_len_ = static_cast<uint16_t>(rx_len_ + len);
}

void BmsManager::parseRx(uint32_t now_ms)
{
    while (rx_len_ >= 5) {
        // Align on SOF.
        uint16_t start = 0;
        while (start < rx_len_ && rx_buf_[start] != 0xAA) {
            start++;
        }

        if (start == rx_len_) {
            rx_len_ = 0;
            break;
        }

        if (start > 0) {
            std::memmove(rx_buf_, rx_buf_ + start, rx_len_ - start);
            rx_len_ = static_cast<uint16_t>(rx_len_ - start);
        }

        if (rx_len_ < 5) {
            break;
        }

        const uint8_t type = rx_buf_[1];
        uint16_t frame_len = 0;

        if (type == 0x01) {
            frame_len = 5; // ACK
        } else if (type == 0x00) {
            frame_len = 6; // NACK
        } else if (type == CMD_READ_PACK_VOLTAGE || type == CMD_READ_PACK_CURRENT) {
            frame_len = 8;
        } else if (type == CMD_READ_MAX_CELL ||
                   type == CMD_READ_MIN_CELL ||
                   type == CMD_READ_ONLINE_STATUS) {
            frame_len = 6;
        } else if (type == CMD_READ_TEMPERATURES ||
                   type == CMD_READ_NEWEST_EVENTS) {
            if (rx_len_ < 3) {
                break;
            }
            frame_len = static_cast<uint16_t>(rx_buf_[2] + 5);
        } else {
            // Unknown type, shift by one and resync.
            std::memmove(rx_buf_, rx_buf_ + 1, rx_len_ - 1);
            rx_len_ = static_cast<uint16_t>(rx_len_ - 1);
            continue;
        }

        if (frame_len < 5) {
            std::memmove(rx_buf_, rx_buf_ + 1, rx_len_ - 1);
            rx_len_ = static_cast<uint16_t>(rx_len_ - 1);
            continue;
        }

        if (rx_len_ < frame_len) {
            break;
        }

        const uint16_t crc_rx = readU16LE(&rx_buf_[frame_len - 2]);
        const uint16_t crc_calc = BmsUart::crc16(rx_buf_, frame_len - 2);

        if (crc_rx != crc_calc) {
            // Bad CRC, shift by one and resync.
            std::memmove(rx_buf_, rx_buf_ + 1, rx_len_ - 1);
            rx_len_ = static_cast<uint16_t>(rx_len_ - 1);
            continue;
        }

        handleFrame(rx_buf_, frame_len, now_ms);

        if (frame_len < rx_len_) {
            std::memmove(rx_buf_, rx_buf_ + frame_len, rx_len_ - frame_len);
        }
        rx_len_ = static_cast<uint16_t>(rx_len_ - frame_len);
    }
}

void BmsManager::handleFrame(const uint8_t* frame, uint16_t len, uint32_t now_ms)
{
    if (frame == nullptr || len < 5) {
        return;
    }

    telemetry_.any_response = true;
    telemetry_.last_rx_ms = now_ms;
    last_good_rx_ms_ = now_ms;

    const uint8_t type = frame[1];

    if (type == 0x00) {
        // NACK: [AA, 00, CMD, ERROR, CRC_L, CRC_H]
        const uint8_t cmd = frame[2];
        completePending(cmd);
        return;
    }

    if (type == 0x01) {
        // ACK: [AA, 01, CMD, CRC_L, CRC_H]
        const uint8_t cmd = frame[2];
        completePending(cmd);
        return;
    }

    switch (type) {
    case CMD_READ_PACK_VOLTAGE:
        if (len == 8) {
            const float v = readFloatLE(&frame[2]);
            if (std::isfinite(v)) {
                telemetry_.pack_voltage_v = v;
            }
            markSnapshotBit(type, now_ms);
        }
        completePending(type);
        break;

    case CMD_READ_PACK_CURRENT:
        if (len == 8) {
            const float a = readFloatLE(&frame[2]);
            if (std::isfinite(a)) {
                telemetry_.pack_current_a = a;
            }
            markSnapshotBit(type, now_ms);
        }
        completePending(type);
        break;

    case CMD_READ_MAX_CELL:
        if (len == 6) {
            telemetry_.max_cell_mv = readU16LE(&frame[2]);
            markSnapshotBit(type, now_ms);
        }
        completePending(type);
        break;

    case CMD_READ_MIN_CELL:
        if (len == 6) {
            telemetry_.min_cell_mv = readU16LE(&frame[2]);
            markSnapshotBit(type, now_ms);
        }
        completePending(type);
        break;

    case CMD_READ_ONLINE_STATUS:
        if (len == 6) {
            telemetry_.online_status = readU16LE(&frame[2]);
            markSnapshotBit(type, now_ms);
        }
        completePending(type);
        break;

    case CMD_READ_TEMPERATURES:
        // [AA, 1B, PL, t_int_l, t_int_h, t1_l, t1_h, t2_l, t2_h, crc_l, crc_h]
        if (len >= 11) {
            telemetry_.pack_temp_dC = readI16LE(&frame[3]);
            telemetry_.ext1_temp_dC = readI16LE(&frame[5]);
            telemetry_.ext2_temp_dC = readI16LE(&frame[7]);
            markSnapshotBit(type, now_ms);
        }
        completePending(type);
        break;

    case CMD_READ_NEWEST_EVENTS:
        if (len >= 9) {
            handleEventsResponse(frame, len, now_ms);
            markSnapshotBit(type, now_ms);
        }
        completePending(type);
        break;

    default:
        break;
    }
}

void BmsManager::handleEventsResponse(const uint8_t* frame, uint16_t len, uint32_t now_ms)
{
    if (frame == nullptr || len < 9) {
        return;
    }

    const uint8_t pl = frame[2];
    if (static_cast<uint16_t>(pl) + 5 != len) {
        return;
    }

    if (pl < 4) {
        return;
    }

    telemetry_.bms_timestamp_s = readU32LE(&frame[3]);

    // Payload: [BTSP(4 bytes)] + N * [TSP(3 bytes), EVENT(1 byte)]
    const uint16_t payload_end = static_cast<uint16_t>(3 + pl);
    uint16_t idx = 7;

    while (idx + 3 < payload_end) {
        const uint32_t tsp_s = readU24LE(&frame[idx]);
        const uint8_t event_id = frame[idx + 3];

        processEvent(tsp_s, event_id, now_ms);

        idx = static_cast<uint16_t>(idx + 4);
    }
}

void BmsManager::processEvent(uint32_t event_ts_s, uint8_t event_id, uint32_t now_ms)
{
    telemetry_.last_event_id = event_id;

    const uint32_t signature =
        ((event_ts_s & 0x00FFFFFFu) << 8) | static_cast<uint32_t>(event_id);

    if (eventSeen(signature)) {
        return;
    }
    rememberEvent(signature);

    switch (event_id) {
    // Non-recoverable switch faults -> report immediately.
    case 0x0B:
    case 0x0C:
    case 0x0D:
        flags_.bms_switch_fault = true;
        break;

    // Overtemperature fault / recovery.
    case 0x04:
        setRecoverablePending(temp_high_, now_ms);
        break;
    case 0x73:
        clearRecoverable(temp_high_);
        break;

    // Overcurrent faults / recoveries.
    case 0x05: // discharging OC
    case 0x06: // charging OC
    case 0x07: // regeneration OC
        setRecoverablePending(overcurrent_, now_ms);
        break;
    case 0x76:
    case 0x77:
    case 0x78:
        clearRecoverable(overcurrent_);
        break;

    // Low voltage fault / recovery.
    case 0x02:
        setRecoverablePending(low_voltage_, now_ms);
        break;
    case 0x7B:
        clearRecoverable(low_voltage_);
        break;

    // High voltage fault / recovery.
    case 0x03:
        setRecoverablePending(high_voltage_, now_ms);
        break;
    case 0x79:
        clearRecoverable(high_voltage_);
        break;

    default:
        break;
    }
}

void BmsManager::setRecoverablePending(RecoverableState& state, uint32_t now_ms)
{
    if (state.active || state.pending) {
        return;
    }

    state.pending = true;
    state.pending_since_ms = now_ms;
}

void BmsManager::clearRecoverable(RecoverableState& state)
{
    state.active = false;
    state.pending = false;
    state.pending_since_ms = 0;
}

void BmsManager::updateRecoverableFlags(uint32_t now_ms)
{
    auto promoteIfConfirmed = [&](RecoverableState& s) {
        if (s.pending && !s.active &&
            elapsedMs(now_ms, s.pending_since_ms) >= cfg_.fault_confirm_ms) {
            s.pending = false;
            s.active = true;
        }
    };

    promoteIfConfirmed(temp_high_);
    promoteIfConfirmed(overcurrent_);
    promoteIfConfirmed(low_voltage_);
    promoteIfConfirmed(high_voltage_);

    flags_.bms_temp_high = temp_high_.active;
    flags_.bms_overcurrent = overcurrent_.active;
    flags_.bms_low_voltage = low_voltage_.active;
    flags_.bms_high_voltage = high_voltage_.active;
}

void BmsManager::trySendNext(uint32_t now_ms)
{
    if (pending_active_) {
        return;
    }

    if (elapsedMs(now_ms, last_poll_ms_) < cfg_.poll_interval_ms) {
        return;
    }

    const uint8_t cmd = kPollCommands[poll_index_];
    sendCommand(cmd);

    pending_active_ = true;
    pending_cmd_ = cmd;
    pending_retries_ = 0;
    pending_sent_ms_ = now_ms;
    last_poll_ms_ = now_ms;
}

void BmsManager::handlePendingTimeout(uint32_t now_ms)
{
    if (!pending_active_) {
        return;
    }

    if (elapsedMs(now_ms, pending_sent_ms_) < cfg_.response_timeout_ms) {
        return;
    }

    if (pending_retries_ < cfg_.max_retries) {
        pending_retries_++;
        sendCommand(pending_cmd_);
        pending_sent_ms_ = now_ms;
        return;
    }

    // Give up this slot and continue polling cycle.
    pending_active_ = false;
    pending_retries_ = 0;
    advancePoll();
}

void BmsManager::sendCommand(uint8_t cmd)
{
    if (cfg_.transport == nullptr) {
        return;
    }

    uint8_t req[4] = {0xAA, cmd, 0x00, 0x00};
    const uint16_t crc = BmsUart::crc16(req, 2);
    req[2] = static_cast<uint8_t>(crc & 0xFFu);
    req[3] = static_cast<uint8_t>((crc >> 8) & 0xFFu);

    cfg_.transport->write(req, sizeof(req));
}

void BmsManager::completePending(uint8_t cmd)
{
    if (!pending_active_) {
        return;
    }

    if (pending_cmd_ != cmd) {
        return;
    }

    pending_active_ = false;
    pending_retries_ = 0;
    advancePoll();
}

void BmsManager::advancePoll()
{
    const uint8_t count = static_cast<uint8_t>(sizeof(kPollCommands) / sizeof(kPollCommands[0]));
    poll_index_ = static_cast<uint8_t>((poll_index_ + 1) % count);
}

void BmsManager::markSnapshotBit(uint8_t cmd, uint32_t now_ms)
{
    switch (cmd) {
    case CMD_READ_PACK_VOLTAGE:  snapshot_mask_ |= SNAP_VOLTAGE; break;
    case CMD_READ_PACK_CURRENT:  snapshot_mask_ |= SNAP_CURRENT; break;
    case CMD_READ_MAX_CELL:      snapshot_mask_ |= SNAP_MAX_CELL; break;
    case CMD_READ_MIN_CELL:      snapshot_mask_ |= SNAP_MIN_CELL; break;
    case CMD_READ_ONLINE_STATUS: snapshot_mask_ |= SNAP_STATUS; break;
    case CMD_READ_TEMPERATURES:  snapshot_mask_ |= SNAP_TEMPS; break;
    case CMD_READ_NEWEST_EVENTS: snapshot_mask_ |= SNAP_EVENTS; break;
    default: break;
    }

    if ((snapshot_mask_ & SNAP_REQUIRED_MASK) == SNAP_REQUIRED_MASK) {
        telemetry_.full_snapshot_ready = true;
        telemetry_.last_full_snapshot_ms = now_ms;
        snapshot_mask_ = 0;
    }
}

bool BmsManager::eventSeen(uint32_t signature) const
{
    for (uint8_t i = 0; i < event_cache_count_; i++) {
        if (event_cache_[i] == signature) {
            return true;
        }
    }
    return false;
}

void BmsManager::rememberEvent(uint32_t signature)
{
    if (event_cache_count_ < EVENT_CACHE_SIZE) {
        event_cache_[event_cache_count_] = signature;
        event_cache_count_++;
        return;
    }

    event_cache_[event_cache_head_] = signature;
    event_cache_head_ = static_cast<uint8_t>((event_cache_head_ + 1) % EVENT_CACHE_SIZE);
}

uint32_t BmsManager::elapsedMs(uint32_t now, uint32_t since)
{
    return now - since;
}

uint16_t BmsManager::readU16LE(const uint8_t* p)
{
    return static_cast<uint16_t>(
        static_cast<uint16_t>(p[0]) |
        (static_cast<uint16_t>(p[1]) << 8));
}

int16_t BmsManager::readI16LE(const uint8_t* p)
{
    return static_cast<int16_t>(readU16LE(p));
}

uint32_t BmsManager::readU24LE(const uint8_t* p)
{
    return
        static_cast<uint32_t>(p[0]) |
        (static_cast<uint32_t>(p[1]) << 8) |
        (static_cast<uint32_t>(p[2]) << 16);
}

uint32_t BmsManager::readU32LE(const uint8_t* p)
{
    return
        static_cast<uint32_t>(p[0]) |
        (static_cast<uint32_t>(p[1]) << 8) |
        (static_cast<uint32_t>(p[2]) << 16) |
        (static_cast<uint32_t>(p[3]) << 24);
}

float BmsManager::readFloatLE(const uint8_t* p)
{
    const uint32_t raw = readU32LE(p);
    float out = 0.0f;
    std::memcpy(&out, &raw, sizeof(float));
    return out;
}
