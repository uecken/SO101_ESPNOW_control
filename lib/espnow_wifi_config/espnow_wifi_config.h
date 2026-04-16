// espnow_wifi_config.h — ESP-NOW WiFi tuning shared library
//
// Extracted from rs485_espnow_bridge for reuse across projects.
// Provides runtime-configurable PHY rate, power save, AMPDU, channel, TX power.
//
// Serial command integration:
//   R<n>     : PHY rate idx 0..13
//   S<0|1|2> : Power Save (0=NONE, 1=MIN, 2=MAX)  — NOTE: conflicts with 's' (start)
//   A<0|1>   : AMPDU off/on
//   C<ch>    : WiFi channel 1..13
//   P<dBm>   : TX power 2..21
//   U<0|1>   : peer mode (0=broadcast, 1=unicast)
//   W        : show current config
//
// Usage in main.cpp:
//   #include <espnow_wifi_config.h>
//   // In setup(): wifi_apply_defaults();
//   // In command handler: wifi_handle_command(cmd);

#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ---- State variables ----
static int8_t cur_phy_rate_idx = -1;
static int8_t cur_wifi_ch = -1;
static int8_t cur_tx_pwr_q025 = -1;
static int8_t cur_ps_mode = -1;
static int8_t cur_protocol = -1;

// ---- PHY rate table ----
struct PhyRateEntry {
    const char *name;
    wifi_phy_rate_t rate;
};

static const PhyRateEntry PHY_RATE_TABLE[] = {
    {"1M_L",      WIFI_PHY_RATE_1M_L},      // 0
    {"2M_L",      WIFI_PHY_RATE_2M_L},      // 1
    {"5M_L",      WIFI_PHY_RATE_5M_L},      // 2
    {"11M_L",     WIFI_PHY_RATE_11M_L},     // 3
    {"6M",        WIFI_PHY_RATE_6M},        // 4
    {"9M",        WIFI_PHY_RATE_9M},        // 5
    {"12M",       WIFI_PHY_RATE_12M},       // 6
    {"18M",       WIFI_PHY_RATE_18M},       // 7
    {"24M",       WIFI_PHY_RATE_24M},       // 8
    {"36M",       WIFI_PHY_RATE_36M},       // 9
    {"48M",       WIFI_PHY_RATE_48M},       // 10
    {"54M",       WIFI_PHY_RATE_54M},       // 11
    {"MCS0_LGI",  WIFI_PHY_RATE_MCS0_LGI}, // 12
    {"MCS7_SGI",  WIFI_PHY_RATE_MCS7_SGI},  // 13
};
static const int PHY_RATE_TABLE_LEN = sizeof(PHY_RATE_TABLE) / sizeof(PHY_RATE_TABLE[0]);

// ---- WiFi config functions ----

static inline bool wifi_set_phy_rate(int idx) {
    if (idx < 0 || idx >= PHY_RATE_TABLE_LEN) return false;
    esp_err_t r = esp_wifi_config_espnow_rate(WIFI_IF_STA, PHY_RATE_TABLE[idx].rate);
    if (r != ESP_OK) {
        Serial.printf("esp_wifi_config_espnow_rate(%s) failed: %d\n", PHY_RATE_TABLE[idx].name, (int)r);
        return false;
    }
    cur_phy_rate_idx = (int8_t)idx;
    Serial.printf("PHY rate -> %s\n", PHY_RATE_TABLE[idx].name);
    return true;
}

static inline bool wifi_set_ps_mode(int mode) {
    wifi_ps_type_t ps;
    const char *name;
    switch (mode) {
        case 0: ps = WIFI_PS_NONE;      name = "NONE";      break;
        case 1: ps = WIFI_PS_MIN_MODEM; name = "MIN_MODEM"; break;
        case 2: ps = WIFI_PS_MAX_MODEM; name = "MAX_MODEM"; break;
        default: return false;
    }
    esp_err_t r = esp_wifi_set_ps(ps);
    if (r != ESP_OK) {
        Serial.printf("esp_wifi_set_ps(%s) failed: %d\n", name, (int)r);
        return false;
    }
    cur_ps_mode = (int8_t)mode;
    Serial.printf("PS mode -> %s\n", name);
    return true;
}

static inline bool wifi_set_protocol_mask(uint8_t mask) {
    esp_err_t r = esp_wifi_set_protocol(WIFI_IF_STA, mask);
    if (r != ESP_OK) {
        Serial.printf("esp_wifi_set_protocol(0x%02X) failed: %d\n", mask, (int)r);
        return false;
    }
    cur_protocol = (int8_t)mask;
    Serial.printf("Protocol -> 0x%02X (b=%d g=%d n=%d)\n", mask,
                  !!(mask & WIFI_PROTOCOL_11B),
                  !!(mask & WIFI_PROTOCOL_11G),
                  !!(mask & WIFI_PROTOCOL_11N));
    return true;
}

static inline bool wifi_set_channel_n(int ch) {
    if (ch < 1 || ch > 13) return false;
    esp_err_t r = esp_wifi_set_channel((uint8_t)ch, WIFI_SECOND_CHAN_NONE);
    if (r != ESP_OK) {
        Serial.printf("esp_wifi_set_channel(%d) failed: %d\n", ch, (int)r);
        return false;
    }
    cur_wifi_ch = (int8_t)ch;
    Serial.printf("Channel -> %d\n", ch);
    return true;
}

static inline bool wifi_set_tx_power_dbm(int dbm) {
    if (dbm < 2 || dbm > 21) return false;
    int8_t q = (int8_t)(dbm * 4);
    esp_err_t r = esp_wifi_set_max_tx_power(q);
    if (r != ESP_OK) {
        Serial.printf("esp_wifi_set_max_tx_power(%d=%ddBm) failed: %d\n", q, dbm, (int)r);
        return false;
    }
    cur_tx_pwr_q025 = q;
    Serial.printf("TX power -> %d (= %d dBm)\n", q, dbm);
    return true;
}

static inline void wifi_show_config() {
    int8_t pwr = -1;
    esp_wifi_get_max_tx_power(&pwr);
    uint8_t ch_now = 0;
    wifi_second_chan_t sec;
    esp_wifi_get_channel(&ch_now, &sec);
    uint8_t proto = 0;
    esp_wifi_get_protocol(WIFI_IF_STA, &proto);
    wifi_ps_type_t ps;
    esp_wifi_get_ps(&ps);
    const char *ps_name = (ps == WIFI_PS_NONE) ? "NONE"
                        : (ps == WIFI_PS_MIN_MODEM) ? "MIN_MODEM"
                        : (ps == WIFI_PS_MAX_MODEM) ? "MAX_MODEM" : "?";
    const char *rate_name = (cur_phy_rate_idx >= 0 && cur_phy_rate_idx < PHY_RATE_TABLE_LEN)
                            ? PHY_RATE_TABLE[cur_phy_rate_idx].name : "default";
    Serial.printf("WIFI: rate=%s ch=%u power=%d (%.1fdBm) ps=%s proto=0x%02X (b=%d g=%d n=%d)\n",
                  rate_name, ch_now, pwr, pwr * 0.25, ps_name, proto,
                  !!(proto & WIFI_PROTOCOL_11B),
                  !!(proto & WIFI_PROTOCOL_11G),
                  !!(proto & WIFI_PROTOCOL_11N));
}

// ---- Default setup (call after WiFi.mode + esp_now_init) ----

static inline void wifi_apply_defaults() {
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_11M_L);
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G);
    cur_phy_rate_idx = 3;  // 11M_L
    cur_ps_mode = 0;       // NONE
    cur_protocol = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G;
}

// ---- Print help for WiFi commands ----

static inline void wifi_print_help() {
    Serial.println("--- WiFi / ESP-NOW tuning ---");
    Serial.println("  R<n>     : PHY rate idx 0..13");
    Serial.println("             0=1M_L 1=2M_L 2=5M_L 3=11M_L");
    Serial.println("             4=6M 5=9M 6=12M 7=18M 8=24M 9=36M 10=48M 11=54M");
    Serial.println("             12=MCS0_LGI 13=MCS7_SGI");
    Serial.println("  A<0|1>   : AMPDU off/on (0=11b+g, 1=11b+g+n)");
    Serial.println("  C<ch>    : WiFi channel 1..13");
    Serial.println("  P<dBm>   : TX power (2..21 dBm)");
    Serial.println("  W        : show current WiFi config");
}

// ---- Handle WiFi-related serial command (returns true if handled) ----

static inline bool wifi_handle_command(const char *cmd) {
    switch (cmd[0]) {
    case 'R': {
        int idx = atoi(cmd + 1);
        if (!wifi_set_phy_rate(idx)) Serial.printf("ERR: rate idx %d invalid\n", idx);
        return true;
    }
    case 'A': {
        int on = atoi(cmd + 1);
        uint8_t mask = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G;
        if (on) mask |= WIFI_PROTOCOL_11N;
        wifi_set_protocol_mask(mask);
        return true;
    }
    case 'C': {
        int ch = atoi(cmd + 1);
        if (!wifi_set_channel_n(ch)) Serial.printf("ERR: channel %d invalid (1..13)\n", ch);
        return true;
    }
    case 'P': {
        int dbm = atoi(cmd + 1);
        if (!wifi_set_tx_power_dbm(dbm)) Serial.printf("ERR: power %d invalid (2..21)\n", dbm);
        return true;
    }
    case 'W':
        wifi_show_config();
        return true;
    default:
        return false;  // not a WiFi command
    }
}
