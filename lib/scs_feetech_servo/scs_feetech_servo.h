// scs_feetech_servo.h — Feetech SCS protocol packet builder (shared library)
//
// Supports: STS3215, STS3032, ST3235, SMS_STS, SCS15 and all Feetech serial bus servos
// Protocol: SCS (Serial Control Servo) — half-duplex TTL or RS-485
// Packet:   [0xFF][0xFF][ID][LEN][INST][PARAMS...][CHECKSUM]
//
// Usage:
//   #include <scs_feetech_servo.h>
//   uint8_t buf[64];
//   int len = scs_build_ping(buf, 1);
//   uart_write_bytes(UART_NUM, buf, len);

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// ---- SCS Protocol Constants ----

#define SCS_HEADER_0       0xFF
#define SCS_HEADER_1       0xFF
#define SCS_BROADCAST_ID   0xFE

// Instructions
#define SCS_INST_PING        0x01
#define SCS_INST_READ        0x02
#define SCS_INST_WRITE       0x03
#define SCS_INST_REG_WRITE   0x04
#define SCS_INST_ACTION      0x05
#define SCS_INST_RESET       0x06
#define SCS_INST_SYNC_WRITE  0x83

// Common register addresses (STS3215 / STS series)
#define SCS_ADDR_ID                 0x05
#define SCS_ADDR_BAUD_RATE          0x06
#define SCS_ADDR_TORQUE_ENABLE      0x28
#define SCS_ADDR_ACCELERATION       0x29
#define SCS_ADDR_GOAL_POSITION      0x2A  // 2B LE
#define SCS_ADDR_GOAL_TIME          0x2C  // 2B LE
#define SCS_ADDR_MOVING_SPEED       0x2E  // 2B LE
#define SCS_ADDR_PRESENT_POSITION   0x38  // 2B LE
#define SCS_ADDR_PRESENT_SPEED      0x3A  // 2B LE
#define SCS_ADDR_PRESENT_LOAD       0x3C  // 2B LE
#define SCS_ADDR_PRESENT_VOLTAGE    0x3E  // 1B
#define SCS_ADDR_PRESENT_TEMP       0x3F  // 1B
#define SCS_ADDR_PRESENT_CURRENT    0x41  // 2B
#define SCS_ADDR_MOVING             0x42  // 1B

// Position range
#define SCS_POS_MIN  0
#define SCS_POS_MAX  4095

// ---- Checksum ----

static inline uint8_t scs_checksum(const uint8_t *pkt, int total_len) {
    uint8_t sum = 0;
    for (int i = 2; i < total_len - 1; i++) sum += pkt[i];
    return (~sum) & 0xFF;
}

// ---- Packet Builders ----

// PING: [FF FF ID 02 01 CHKSUM] = 6 bytes
static inline int scs_build_ping(uint8_t *buf, uint8_t id) {
    buf[0] = 0xFF; buf[1] = 0xFF;
    buf[2] = id; buf[3] = 0x02; buf[4] = SCS_INST_PING;
    buf[5] = scs_checksum(buf, 6);
    return 6;
}

// READ: [FF FF ID 04 02 ADDR LEN CHKSUM] = 8 bytes
static inline int scs_build_read(uint8_t *buf, uint8_t id, uint8_t addr, uint8_t data_len) {
    buf[0] = 0xFF; buf[1] = 0xFF;
    buf[2] = id; buf[3] = 0x04; buf[4] = SCS_INST_READ;
    buf[5] = addr; buf[6] = data_len;
    buf[7] = scs_checksum(buf, 8);
    return 8;
}

// WRITE byte: [FF FF ID 04 03 ADDR VAL CHKSUM] = 8 bytes
static inline int scs_build_write_byte(uint8_t *buf, uint8_t id, uint8_t addr, uint8_t val) {
    buf[0] = 0xFF; buf[1] = 0xFF;
    buf[2] = id; buf[3] = 0x04; buf[4] = SCS_INST_WRITE;
    buf[5] = addr; buf[6] = val;
    buf[7] = scs_checksum(buf, 8);
    return 8;
}

// WRITE Goal Position + Goal Time + Moving Speed:
// [FF FF ID 09 03 2A posL posH timeL timeH spdL spdH CHKSUM] = 13 bytes
static inline int scs_build_write_pos(uint8_t *buf, uint8_t id,
                                       uint16_t pos, uint16_t time_ms, uint16_t speed) {
    buf[0] = 0xFF; buf[1] = 0xFF;
    buf[2] = id; buf[3] = 0x09; buf[4] = SCS_INST_WRITE;
    buf[5] = SCS_ADDR_GOAL_POSITION;
    buf[6] = pos & 0xFF; buf[7] = (pos >> 8) & 0xFF;
    buf[8] = time_ms & 0xFF; buf[9] = (time_ms >> 8) & 0xFF;
    buf[10] = speed & 0xFF; buf[11] = (speed >> 8) & 0xFF;
    buf[12] = scs_checksum(buf, 13);
    return 13;
}

// SYNC_WRITE Goal Position (multiple servos, 1 frame):
// [FF FF FE LEN 83 ADDR DATA_LEN_PER_SERVO [ID1 posL posH] [ID2 posL posH] ... CHKSUM]
// data_per_servo = 2 (Position only, 2 bytes LE)
// Returns total packet length
static inline int scs_build_sync_write_pos(uint8_t *buf, const uint8_t *ids,
                                            const uint16_t *positions, int n_axes) {
    buf[0] = 0xFF; buf[1] = 0xFF;
    buf[2] = SCS_BROADCAST_ID;  // 0xFE
    // buf[3] = LEN (filled later)
    buf[4] = SCS_INST_SYNC_WRITE;
    buf[5] = SCS_ADDR_GOAL_POSITION;  // start address
    buf[6] = 2;  // data length per servo (Goal Position = 2 bytes)
    int pos = 7;
    for (int i = 0; i < n_axes; i++) {
        buf[pos++] = ids[i];
        buf[pos++] = positions[i] & 0xFF;
        buf[pos++] = (positions[i] >> 8) & 0xFF;
    }
    buf[3] = (uint8_t)(pos - 3);  // LEN = everything after LEN field, including checksum
    buf[pos] = scs_checksum(buf, pos + 1);
    return pos + 1;
}

// SYNC_WRITE Goal Position + Goal Time + Moving Speed (6 bytes per servo):
// Some STS3215 servos require Time+Speed fields to move.
// [FF FF FE LEN 83 0x2A 6 [ID posL posH timeL timeH spdL spdH] ... CHKSUM]
static inline int scs_build_sync_write_pos_full(uint8_t *buf, const uint8_t *ids,
                                                 const uint16_t *positions, int n_axes,
                                                 uint16_t time_ms, uint16_t speed) {
    buf[0] = 0xFF; buf[1] = 0xFF;
    buf[2] = SCS_BROADCAST_ID;
    buf[4] = SCS_INST_SYNC_WRITE;
    buf[5] = SCS_ADDR_GOAL_POSITION;
    buf[6] = 6;  // data length per servo: pos(2) + time(2) + speed(2)
    int pos = 7;
    for (int i = 0; i < n_axes; i++) {
        buf[pos++] = ids[i];
        buf[pos++] = positions[i] & 0xFF;
        buf[pos++] = (positions[i] >> 8) & 0xFF;
        buf[pos++] = time_ms & 0xFF;
        buf[pos++] = (time_ms >> 8) & 0xFF;
        buf[pos++] = speed & 0xFF;
        buf[pos++] = (speed >> 8) & 0xFF;
    }
    buf[3] = (uint8_t)(pos - 3);
    buf[pos] = scs_checksum(buf, pos + 1);
    return pos + 1;
}

// Torque Enable/Disable for single servo
static inline int scs_build_torque(uint8_t *buf, uint8_t id, bool enable) {
    return scs_build_write_byte(buf, id, SCS_ADDR_TORQUE_ENABLE, enable ? 1 : 0);
}

// ---- Response Parser ----

// Receive SCS status packet from UART with tight polling.
// Requires: uart_read_bytes() from ESP-IDF driver/uart.h
// Returns: number of bytes received (0 = timeout)
// response format: [FF FF ID LEN ERR [PARAMS...] CHKSUM]
#if defined(ARDUINO) || defined(ESP_IDF_VERSION)
#include "driver/uart.h"
#include "esp_timer.h"

static inline int scs_recv_response(uart_port_t uart_num, uint8_t *buf, int max_len, uint32_t timeout_us) {
    int pos = 0;
    int64_t deadline = esp_timer_get_time() + timeout_us;
    while (esp_timer_get_time() < deadline) {
        int n = uart_read_bytes(uart_num, buf + pos, max_len - pos, 0);
        if (n > 0) {
            pos += n;
            // Check for complete SCS packet: [FF FF ID LEN ...]
            if (pos >= 6) {
                int idx = -1;
                for (int i = 0; i <= pos - 2; i++) {
                    if (buf[i] == 0xFF && buf[i + 1] == 0xFF) { idx = i; break; }
                }
                if (idx >= 0 && idx + 3 < pos) {
                    int pkt_len = buf[idx + 3] + 4;  // header(2) + id(1) + len_field(1) + len bytes
                    if (idx + pkt_len <= pos) return pos;  // complete packet
                }
            }
        }
    }
    return pos;
}
// Scan all IDs (0-253) with PING. Returns number of found servos.
// found_ids[]: filled with responding IDs (up to max_found)
// timeout_per_id_us: PING timeout per ID (recommend 1500-3000 us)
static inline int scs_scan_ids(uart_port_t uart_num, uint8_t *found_ids, int max_found,
                                uint32_t timeout_per_id_us) {
    uint8_t ping_buf[8];
    uint8_t resp_buf[16];
    int found = 0;
    for (int id = 0; id < 254 && found < max_found; id++) {
        // Flush RX
        { uint8_t tmp[64]; uart_read_bytes(uart_num, tmp, sizeof(tmp), 0); }

        int len = scs_build_ping(ping_buf, (uint8_t)id);
        uart_write_bytes(uart_num, (const char *)ping_buf, len);
        uart_wait_tx_done(uart_num, pdMS_TO_TICKS(5));

        int resp_len = scs_recv_response(uart_num, resp_buf, sizeof(resp_buf), timeout_per_id_us);
        if (resp_len >= 6) {
            // Valid PING response: check header
            int idx = -1;
            for (int i = 0; i <= resp_len - 2; i++) {
                if (resp_buf[i] == 0xFF && resp_buf[i + 1] == 0xFF) { idx = i; break; }
            }
            if (idx >= 0 && idx + 4 < resp_len) {
                uint8_t resp_id = resp_buf[idx + 2];
                uint8_t resp_err = resp_buf[idx + 4];
                if (resp_id == (uint8_t)id && resp_err == 0) {
                    found_ids[found++] = (uint8_t)id;
                }
            }
        }
    }
    return found;
}
#endif

// Return true if position is within single-turn range (SCS_POS_MIN .. SCS_POS_MAX = 0..4095).
// Values outside this range indicate multi-turn mode (EEPROM Min_Angle_Limit = Max_Angle_Limit = 0)
// where the servo reports a signed 16-bit cumulative position (1 turn = 4096 counts).
// Writing such values to another servo's Goal Position causes sudden multi-rotation movement
// (observed symptom: "Joint suddenly spins in reverse"). Application layer should filter
// with this helper before propagating positions between servos.
// Root-cause fix: set EEPROM Min=0, Max=4095 via so101_calibrate.py --reset.
static inline bool scs_is_single_turn_position(int pos) {
    return pos >= SCS_POS_MIN && pos <= SCS_POS_MAX;
}

// Extract 16-bit position from READ response
// Response: [FF FF ID LEN ERR posL posH CHKSUM]
// Returns raw position value or -1 on error.
// NOTE: returned value is the raw 16-bit register value. On multi-turn-mode servos
// (EEPROM Min=Max=0) this can exceed 4095. Combine with scs_is_single_turn_position()
// before using as a Goal Position target to prevent multi-rotation runaway.
static inline int scs_parse_position(const uint8_t *resp, int resp_len) {
    if (resp_len < 8) return -1;
    int idx = -1;
    for (int i = 0; i <= resp_len - 2; i++) {
        if (resp[i] == 0xFF && resp[i + 1] == 0xFF) { idx = i; break; }
    }
    if (idx < 0 || idx + 7 > resp_len) return -1;
    uint8_t err = resp[idx + 4];
    if (err != 0) return -1;
    return (int)(resp[idx + 5] | (resp[idx + 6] << 8));
}

// SYNC_READ packet builder (instruction 0x82)
// Format: [FF FF FE LEN 0x82 addr rd_len ID1...IDn CHK]
//   LEN = 4 + n
//   CHK = ~(0xFE + LEN + 0x82 + addr + rd_len + ID1 + ... + IDn) & 0xFF
// Returns total packet length (8 + n)
//
// Verified on STS3215 (2026-04-16): all 6 servos respond back-to-back
// with [FF FF ID 04 ERR posL posH CHK] = 8B each = 48B continuous stream.
static inline int scs_build_sync_read(uint8_t *buf, uint8_t addr, uint8_t rd_len,
                                       const uint8_t *ids, int n) {
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    buf[2] = 0xFE;                    // broadcast ID
    buf[3] = (uint8_t)(4 + n);        // LEN
    buf[4] = 0x82;                    // SYNC_READ instruction
    buf[5] = addr;
    buf[6] = rd_len;
    uint8_t sum = 0xFE + buf[3] + 0x82 + addr + rd_len;
    for (int i = 0; i < n; i++) {
        buf[7 + i] = ids[i];
        sum += ids[i];
    }
    buf[7 + n] = (uint8_t)(~sum);
    return 8 + n;
}

// Parse SYNC_READ response stream of N x 8B packets.
// Walks the buffer for [FF FF ID 04 ERR posL posH CHK] packets and extracts
// positions for matched IDs (in order of expected_ids[]).
// Returns # of successfully parsed positions.
//
// out_positions[i] is set only if expected_ids[i] is found and ERR==0.
// Indices not found are left untouched.
// NOTE: out_positions[] receive raw 16-bit register values. Multi-turn-mode
// servos (EEPROM Min=Max=0) can return values >4095. Combine with
// scs_is_single_turn_position() before using as Goal Position targets.
static inline int scs_parse_sync_read_positions(const uint8_t *resp, int resp_len,
                                                  const uint8_t *expected_ids, int n_expected,
                                                  uint16_t *out_positions) {
    int found = 0;
    int i = 0;
    while (i + 8 <= resp_len) {
        // Look for SOF
        if (resp[i] != 0xFF || resp[i + 1] != 0xFF) {
            i++;
            continue;
        }
        // Check it's a valid 8B response: LEN=4 (means 4 bytes after LEN: ERR posL posH CHK)
        uint8_t id = resp[i + 2];
        uint8_t len = resp[i + 3];
        if (len != 4) { i++; continue; }
        uint8_t err = resp[i + 4];
        if (err != 0) { i += 8; continue; }
        // Match against expected_ids
        for (int k = 0; k < n_expected; k++) {
            if (expected_ids[k] == id) {
                out_positions[k] = (uint16_t)(resp[i + 5] | (resp[i + 6] << 8));
                found++;
                break;
            }
        }
        i += 8;
    }
    return found;
}
