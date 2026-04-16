// SO-101 Leader-Follower wireless teleoperation (PC-less, ESP-NOW)
//
// 4 states: INIT → SCANNING → RUNNING → STOPPED
// Power ON → auto-scan → auto-start when all servo IDs found
//
// 3 modes:
//   m0 = LEADER        — UART1: reads positions from STS3215, sends via ESP-NOW
//   m1 = FOLLOWER      — UART1: receives via ESP-NOW, writes SYNC_WRITE to STS3215
//   m2 = BRIDGE_LEADER — UART2(G26/G36): reads Leader arm, UART1(G32/G33): writes SYNC_WRITE to bridge
//
// Hardware (m0/m1):
//   M5StickC → Grove UART1 (GPIO 32/33) → Waveshare Adapter (URT-1 mode) → STS3215 ×6
// Hardware (m2):
//   Leader Arm → Waveshare → Grove UART2 (G26=TX, G36=RX) → M5StickC
//   M5StickC → Grove UART1 (G32/G33) → ISO485 → RS-485 → bridge_L → ESP-NOW → bridge_R → Follower Arm
//
// Serial commands:
//   m<0|1|2> : mode (0=LEADER, 1=FOLLOWER, 2=BRIDGE_LEADER) [STOPPED only]
//   J<ids>   : servo IDs comma-separated (e.g. J1,2,3,4,5,6) [STOPPED only]
//   I<n>     : axes count 1-6 [STOPPED only]
//   G<us>    : control period us (default 20000=50Hz) [STOPPED only]
//   S        : manual scan (→ SCANNING)
//   s        : start (→ SCANNING → RUNNING)
//   x        : stop (→ STOPPED, Follower: Torque OFF)
//   d        : toggle debug position output (1Hz)
//   r        : reset stats
//   ?        : status
//   v        : verbose
//   h        : help
//   R/A/C/P/W : WiFi tuning (see espnow_wifi_config.h)

#if defined(BOARD_M5STICKC)
  #include <M5StickC.h>
#elif defined(BOARD_ATOMS3)
  #include <Arduino.h>
#else
  #include <Arduino.h>
#endif

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <esp_idf_version.h>
#include "driver/uart.h"
#include "soc/soc_caps.h"

#include <scs_feetech_servo.h>
#include <espnow_wifi_config.h>

// ---- GPIO ----
#ifndef PIN_TXD
#define PIN_TXD 32
#endif
#ifndef PIN_RXD
#define PIN_RXD 33
#endif
#ifndef PIN_TXD2
#define PIN_TXD2 26
#endif
#ifndef PIN_RXD2
#define PIN_RXD2 36
#endif

static const uart_port_t UART_NUM = UART_NUM_1;   // UART1: Follower WRITE / bridge output
#if defined(BOARD_M5STICKC) || defined(BOARD_ATOMS3)
static const uart_port_t UART_LEADER = UART_NUM_2; // UART2: Leader READ (m2, ESP32 only)
#else
// ESP32-C6 etc: only 2 UARTs. UART_LEADER shares UART_NUM (m2 not supported on this board)
static const uart_port_t UART_LEADER = UART_NUM_1;
#endif
static const int UART_BUF_SIZE = 4096;

// ---- ESP-NOW ----
static const uint8_t BROADCAST_ADDR[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint8_t my_mac[6];

// ---- State Machine ----
enum DeviceState { ST_INIT = 0, ST_SCANNING = 1, ST_RUNNING = 2, ST_STOPPED = 3 };
static const char *STATE_NAMES[] = {"INIT", "SCANNING", "RUNNING", "STOPPED"};
static DeviceState state = ST_INIT;

enum DeviceMode { MODE_LEADER = 0, MODE_FOLLOWER = 1, MODE_BRIDGE_LEADER = 2 };
static const char *MODE_NAMES[] = {"LEADER", "FOLLOWER", "BRIDGE_LEADER"};
static DeviceMode device_mode = MODE_LEADER;

// ---- Config ----
static uint32_t servo_baud = 1000000;
static int n_axes = 6;
static uint8_t servo_ids[6] = {1, 2, 3, 4, 5, 6};
static uint32_t control_period_us = 20000;  // 50Hz
static bool ids_manually_set = false;
static bool debug_pos_output = false;

static bool uart_installed = false;
static bool uart2_installed = false;

// ---- NVS (persistent settings) ----
static Preferences prefs;

static void nvs_save() {
    prefs.begin("so101", false);
    prefs.putUChar("mode", (uint8_t)device_mode);
    prefs.putUChar("n_axes", (uint8_t)n_axes);
    prefs.putUInt("period", control_period_us);
    prefs.putBytes("ids", servo_ids, 6);
    prefs.end();
    Serial.println("Settings saved to NVS.");
}

static void nvs_load() {
    prefs.begin("so101", true);  // read-only
    if (prefs.isKey("mode")) {
        device_mode = (DeviceMode)prefs.getUChar("mode", 0);
        n_axes = prefs.getUChar("n_axes", 6);
        control_period_us = prefs.getUInt("period", 20000);
        prefs.getBytes("ids", servo_ids, 6);
        Serial.printf("NVS loaded: mode=%s axes=%d period=%luus IDs=[",
                      MODE_NAMES[device_mode], n_axes, (unsigned long)control_period_us);
        for (int i = 0; i < n_axes; i++)
            Serial.printf("%d%s", servo_ids[i], i < n_axes - 1 ? "," : "");
        Serial.println("]");
    } else {
        Serial.println("NVS: no saved settings, using defaults.");
    }
    prefs.end();
}

// ---- Position Packet (ESP-NOW) ----
struct __attribute__((packed)) pos_packet_t {
    uint8_t src_mac[6];
    uint8_t n_axes;
    uint8_t ids[6];
    uint16_t pos[6];
};  // 25B

// ---- Stats ----
static uint64_t leader_read_ok = 0;
static uint64_t leader_read_fail = 0;
static uint64_t leader_send_ok = 0;
static uint64_t leader_send_fail = 0;
static uint64_t follower_recv = 0;
static uint64_t follower_write = 0;
static uint32_t last_cycle_us = 0;
static uint32_t consecutive_fail = 0;

// ---- Positions ----
static uint16_t leader_positions[6] = {};
static uint16_t follower_last_pos[6] = {};
static uint8_t  follower_last_n_axes = 0;

// ---- Scan ----
static uint32_t last_scan_ms = 0;
static bool first_scan = true;
static const uint32_t SCAN_INTERVAL_MS = 5000;

// ---- Timing ----
static int64_t next_cycle_us = 0;
static uint32_t lcd_update_ms = 0;

// ---- Serial ----
static char cmd_buf[48];
static int cmd_pos = 0;

// ============================================================================
// UART
// ============================================================================

static void init_uart() {
    if (uart_installed) {
        uart_driver_delete(UART_NUM);
        uart_installed = false;
    }
    uart_config_t cfg = {};
    cfg.baud_rate = (int)servo_baud;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
#if defined(UART_SCLK_APB)
    cfg.source_clk = UART_SCLK_APB;
#elif defined(UART_SCLK_DEFAULT)
    cfg.source_clk = UART_SCLK_DEFAULT;
#else
    cfg.source_clk = (uart_sclk_t)0;  // use default clock
#endif
    uart_driver_install(UART_NUM, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0);
    uart_param_config(UART_NUM, &cfg);
    uart_set_pin(UART_NUM, PIN_TXD, PIN_RXD,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_installed = true;
}

static void init_uart2() {
    if (uart2_installed) {
        uart_driver_delete(UART_LEADER);
        uart2_installed = false;
    }
    uart_config_t cfg = {};
    cfg.baud_rate = (int)servo_baud;  // 1Mbaud (Leader servo)
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
#if defined(UART_SCLK_APB)
    cfg.source_clk = UART_SCLK_APB;
#elif defined(UART_SCLK_DEFAULT)
    cfg.source_clk = UART_SCLK_DEFAULT;
#else
    cfg.source_clk = (uart_sclk_t)0;  // use default clock
#endif
    uart_driver_install(UART_LEADER, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0);
    uart_param_config(UART_LEADER, &cfg);
    uart_set_pin(UART_LEADER, PIN_TXD2, PIN_RXD2,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart2_installed = true;
}

// ============================================================================
// ESP-NOW callbacks
// ============================================================================

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static void on_espnow_send(const wifi_tx_info_t *info, esp_now_send_status_t status) {
#else
static void on_espnow_send(const uint8_t *mac, esp_now_send_status_t status) {
#endif
    if (status == ESP_NOW_SEND_SUCCESS) leader_send_ok++;
    else leader_send_fail++;
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static void on_espnow_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    const uint8_t *mac = info->src_addr;
#else
static void on_espnow_recv(const uint8_t *mac, const uint8_t *data, int len) {
#endif
    if (device_mode != MODE_FOLLOWER || state != ST_RUNNING) return;
    if (len < (int)sizeof(pos_packet_t)) return;

    const pos_packet_t *pkt = (const pos_packet_t *)data;
    if (memcmp(pkt->src_mac, my_mac, 6) == 0) return;  // loop guard

    follower_recv++;

    uint8_t tx_buf[64];
    int axes = pkt->n_axes;
    if (axes > 6) axes = 6;

    // Save received positions for display
    follower_last_n_axes = (uint8_t)axes;
    for (int i = 0; i < axes; i++) follower_last_pos[i] = pkt->pos[i];

    int pkt_len = scs_build_sync_write_pos_full(tx_buf, pkt->ids, pkt->pos, axes, 0, 0);
    if (uart_installed) {
        uart_write_bytes(UART_NUM, (const char *)tx_buf, pkt_len);
        uart_wait_tx_done(UART_NUM, pdMS_TO_TICKS(10));
        follower_write++;
    }
}

static bool espnow_setup() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    wifi_apply_defaults();  // 11M_L, PS_NONE, AMPDU off
    WiFi.macAddress(my_mac);

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        return false;
    }
    esp_now_register_send_cb(on_espnow_send);
    esp_now_register_recv_cb(on_espnow_recv);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, BROADCAST_ADDR, 6);
    peer.channel = 0;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("ESP-NOW peer add failed");
        return false;
    }
    return true;
}

// ============================================================================
// Scan: check all expected servo IDs are present
// ============================================================================

static bool all_ids_found(const uint8_t *found, int n_found,
                          const uint8_t *expected, int n_expected) {
    for (int i = 0; i < n_expected; i++) {
        bool ok = false;
        for (int j = 0; j < n_found; j++) {
            if (found[j] == expected[i]) { ok = true; break; }
        }
        if (!ok) return false;
    }
    return true;
}

static int do_scan() {
    uint8_t found[16];
    int n = scs_scan_ids(UART_NUM, found, 16, 2000);

    Serial.printf("SCAN: found %d servo(s):", n);
    for (int i = 0; i < n; i++) Serial.printf(" %d", found[i]);
    Serial.println();

    if (all_ids_found(found, n, servo_ids, n_axes)) {
        Serial.printf("All %d expected IDs found. Ready.\n", n_axes);
        return n_axes;
    } else {
        // Show which are missing
        Serial.printf("Missing IDs:");
        for (int i = 0; i < n_axes; i++) {
            bool ok = false;
            for (int j = 0; j < n; j++) {
                if (found[j] == servo_ids[i]) { ok = true; break; }
            }
            if (!ok) Serial.printf(" %d", servo_ids[i]);
        }
        Serial.println();
        return -1;
    }
}

// ============================================================================
// Leader: read + send
// ============================================================================

static void leader_read_and_send() {
    uint8_t buf[16];
    uint8_t resp[32];
    int ok_count = 0;

    for (int i = 0; i < n_axes; i++) {
        { uint8_t tmp[256]; uart_read_bytes(UART_NUM, tmp, sizeof(tmp), 0); }

        int pkt_len = scs_build_read(buf, servo_ids[i], SCS_ADDR_PRESENT_POSITION, 2);
        uart_write_bytes(UART_NUM, (const char *)buf, pkt_len);
        uart_wait_tx_done(UART_NUM, pdMS_TO_TICKS(5));

        int resp_len = scs_recv_response(UART_NUM, resp, sizeof(resp), 3000);
        int pos = scs_parse_position(resp, resp_len);
        if (pos >= 0) {
            leader_positions[i] = (uint16_t)pos;
            leader_read_ok++;
            ok_count++;
        } else {
            leader_read_fail++;
        }
    }

    // Error tracking
    if (ok_count == 0) consecutive_fail++;
    else consecutive_fail = 0;

    // Send position packet
    pos_packet_t pkt;
    memcpy(pkt.src_mac, my_mac, 6);
    pkt.n_axes = (uint8_t)n_axes;
    for (int i = 0; i < 6; i++) {
        pkt.ids[i] = (i < n_axes) ? servo_ids[i] : 0;
        pkt.pos[i] = (i < n_axes) ? leader_positions[i] : 0;
    }
    esp_now_send(BROADCAST_ADDR, (uint8_t *)&pkt, sizeof(pkt));
}

// ============================================================================
// Bridge Leader: UART2 read + UART1 write (transparent bridge)
// ============================================================================

static void bridge_leader_read_and_write() {
    uint8_t buf[16];
    uint8_t resp[32];
    int ok_count = 0;

    // Read all axes from Leader via UART2
    for (int i = 0; i < n_axes; i++) {
        { uint8_t tmp[256]; uart_read_bytes(UART_LEADER, tmp, sizeof(tmp), 0); }

        int pkt_len = scs_build_read(buf, servo_ids[i], SCS_ADDR_PRESENT_POSITION, 2);
        uart_write_bytes(UART_LEADER, (const char *)buf, pkt_len);
        uart_wait_tx_done(UART_LEADER, pdMS_TO_TICKS(5));

        int resp_len = scs_recv_response(UART_LEADER, resp, sizeof(resp), 3000);
        int pos = scs_parse_position(resp, resp_len);
        if (pos >= 0) {
            leader_positions[i] = (uint16_t)pos;
            leader_read_ok++;
            ok_count++;
        } else {
            leader_read_fail++;
        }
    }

    if (ok_count == 0) consecutive_fail++;
    else consecutive_fail = 0;

    // Send SYNC_WRITE to Follower via UART1 → ISO485 → bridge_L → ESP-NOW → bridge_R
    if (ok_count > 0) {
        uint8_t tx_buf[64];
        int pkt_len = scs_build_sync_write_pos(tx_buf, servo_ids, leader_positions, n_axes);
        uart_write_bytes(UART_NUM, (const char *)tx_buf, pkt_len);
        uart_wait_tx_done(UART_NUM, pdMS_TO_TICKS(10));
        leader_send_ok++;
    }
}

// ============================================================================
// Follower: torque control
// ============================================================================

static void follower_torque(bool enable) {
    uint8_t buf[16];
    for (int i = 0; i < n_axes; i++) {
        int len = scs_build_torque(buf, servo_ids[i], enable);
        uart_write_bytes(UART_NUM, (const char *)buf, len);
        uart_wait_tx_done(UART_NUM, pdMS_TO_TICKS(5));
        delay(10);
    }
    Serial.printf("Torque %s (axes %d)\n", enable ? "ON" : "OFF", n_axes);
}

// ============================================================================
// Stats
// ============================================================================

static void reset_stats() {
    leader_read_ok = 0;
    leader_read_fail = 0;
    leader_send_ok = 0;
    leader_send_fail = 0;
    follower_recv = 0;
    follower_write = 0;
    last_cycle_us = 0;
    consecutive_fail = 0;
}

static void show_status() {
    Serial.printf("STATUS state=%s mode=%s baud=%lu axes=%d period=%luus\n",
                  STATE_NAMES[state], MODE_NAMES[device_mode],
                  (unsigned long)servo_baud, n_axes,
                  (unsigned long)control_period_us);
    if (device_mode == MODE_LEADER) {
        Serial.printf("  read_ok=%llu read_fail=%llu send_ok=%llu send_fail=%llu cycle_us=%lu\n",
                      (unsigned long long)leader_read_ok,
                      (unsigned long long)leader_read_fail,
                      (unsigned long long)leader_send_ok,
                      (unsigned long long)leader_send_fail,
                      (unsigned long)last_cycle_us);
    } else {
        Serial.printf("  recv=%llu write=%llu\n",
                      (unsigned long long)follower_recv,
                      (unsigned long long)follower_write);
    }
}

static void show_verbose() {
    show_status();
    Serial.printf("  MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  my_mac[0], my_mac[1], my_mac[2],
                  my_mac[3], my_mac[4], my_mac[5]);
    Serial.printf("  IDs: [");
    for (int i = 0; i < n_axes; i++)
        Serial.printf("%d%s", servo_ids[i], i < n_axes - 1 ? "," : "");
    Serial.printf("]\n");
    if (device_mode == MODE_LEADER && state == ST_RUNNING) {
        Serial.printf("  pos: ");
        for (int i = 0; i < n_axes; i++)
            Serial.printf("[%d]=%d ", servo_ids[i], leader_positions[i]);
        Serial.println();
    }
    wifi_show_config();
}

// ============================================================================
// LCD
// ============================================================================

#if defined(BOARD_M5STICKC)
static void lcd_update() {
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.setTextSize(2);
    char prefix = (device_mode == MODE_LEADER) ? 'L' : 'F';

    switch (state) {
    case ST_INIT:
        M5.Lcd.printf("%c INIT", prefix);
        break;
    case ST_SCANNING:
        M5.Lcd.printf("%c SCAN", prefix);
        break;
    case ST_RUNNING:
        if (device_mode == MODE_LEADER) {
            M5.Lcd.printf("L %dHz\n", (int)(1000000.0f / control_period_us));
            M5.Lcd.setTextSize(1);
            M5.Lcd.printf("ok=%llu f=%llu\n",
                          (unsigned long long)leader_read_ok,
                          (unsigned long long)leader_read_fail);
            M5.Lcd.printf("cyc=%luus\n", (unsigned long)last_cycle_us);
        } else {
            M5.Lcd.printf("F RUN\n");
            M5.Lcd.setTextSize(1);
            M5.Lcd.printf("rx=%llu wr=%llu\n",
                          (unsigned long long)follower_recv,
                          (unsigned long long)follower_write);
        }
        break;
    case ST_STOPPED:
        M5.Lcd.printf("%c STOP", prefix);
        break;
    }
}
#else
static void lcd_update() {}
#endif

// ============================================================================
// Serial commands
// ============================================================================

static void print_help() {
    Serial.println("=== SO-101 Leader-Follower ===");
    Serial.println("--- Servo control ---");
    Serial.println("  m<0|1|2> : mode (0=LEADER, 1=FOLLOWER, 2=BRIDGE_LEADER) [STOPPED only]");
    Serial.println("  J<ids>   : servo IDs (e.g. J1,2,3,4,5,6) [STOPPED only]");
    Serial.println("  I<n>     : axes 1-6 [STOPPED only]");
    Serial.println("  G<us>    : period (default 20000=50Hz) [STOPPED only]");
    Serial.printf( "  S        : scan servo IDs on bus\n");
    Serial.println("  s        : start (→ SCANNING → RUNNING)");
    Serial.println("  x        : stop (→ STOPPED)");
    Serial.println("  d        : toggle debug position output (1Hz)");
    Serial.println("  r        : reset stats");
    Serial.println("  ?        : status");
    Serial.println("  v        : verbose");
    Serial.println("  h        : help");
    wifi_print_help();
}

static void handle_command(const char *cmd) {
    // WiFi commands (R/A/C/P/W) handled by shared library
    if (wifi_handle_command(cmd)) return;

    switch (cmd[0]) {
    case 'm': {
        if (state != ST_STOPPED && state != ST_INIT) {
            Serial.println("ERR: stop first (x)");
            break;
        }
        int m = atoi(cmd + 1);
        if (m >= 0 && m <= 2) {
            device_mode = (DeviceMode)m;
            Serial.printf("Mode=%s\n", MODE_NAMES[device_mode]);
            nvs_save();
        } else {
            Serial.println("ERR: m expects 0 (LEADER), 1 (FOLLOWER), or 2 (BRIDGE_LEADER)");
        }
        break;
    }
    case 'J': {
        if (state == ST_RUNNING) { Serial.println("ERR: stop first"); break; }
        int idx = 0;
        const char *p = cmd + 1;
        while (*p && idx < 6) {
            servo_ids[idx++] = (uint8_t)atoi(p);
            while (*p && *p != ',') p++;
            if (*p == ',') p++;
        }
        if (idx > 0) {
            n_axes = idx;
            ids_manually_set = true;
            Serial.printf("IDs=[");
            for (int i = 0; i < n_axes; i++)
                Serial.printf("%d%s", servo_ids[i], i < n_axes - 1 ? "," : "");
            Serial.printf("] axes=%d\n", n_axes);
            nvs_save();
        }
        break;
    }
    case 'I': {
        if (state == ST_RUNNING) { Serial.println("ERR: stop first"); break; }
        int n = atoi(cmd + 1);
        if (n >= 1 && n <= 6) {
            n_axes = n;
            Serial.printf("Axes=%d\n", n_axes);
            nvs_save();
        }
        break;
    }
    case 'G':
        if (state == ST_RUNNING) { Serial.println("ERR: stop first"); break; }
        control_period_us = strtoul(cmd + 1, NULL, 10);
        Serial.printf("Period=%luus (%.1fHz)\n",
                      (unsigned long)control_period_us,
                      1000000.0f / control_period_us);
        nvs_save();
        break;
    case 'S':  // manual scan
        if (!uart_installed) init_uart();
        state = ST_SCANNING;
        first_scan = true;
        last_scan_ms = 0;
        Serial.println("→ SCANNING");
        break;
    case 's':  // start
        if (!uart_installed) init_uart();
        if (device_mode == MODE_BRIDGE_LEADER && !uart2_installed) init_uart2();
        reset_stats();
        state = ST_SCANNING;
        first_scan = true;
        last_scan_ms = 0;
        Serial.printf("START mode=%s axes=%d period=%luus\n",
                      MODE_NAMES[device_mode], n_axes,
                      (unsigned long)control_period_us);
        break;
    case 'x':  // stop
        if (state == ST_RUNNING && device_mode == MODE_FOLLOWER && uart_installed) {
            follower_torque(false);
        }
        state = ST_STOPPED;
        consecutive_fail = 0;
        Serial.println("→ STOPPED");
        break;
    case 'r':
        reset_stats();
        Serial.println("Stats reset.");
        break;
    case 'd':
        debug_pos_output = !debug_pos_output;
        Serial.printf("Debug output %s\n", debug_pos_output ? "ON" : "OFF");
        break;
    case '?':
        show_status();
        break;
    case 'v':
        show_verbose();
        break;
    case 'h':
        print_help();
        break;
    default:
        Serial.printf("Unknown: %s\n", cmd);
        print_help();
        break;
    }
}

static void poll_serial() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (cmd_pos > 0) {
                cmd_buf[cmd_pos] = 0;
                handle_command(cmd_buf);
                cmd_pos = 0;
            }
        } else if (cmd_pos < (int)sizeof(cmd_buf) - 1) {
            cmd_buf[cmd_pos++] = c;
        }
    }
}

// ============================================================================
// Setup / Loop
// ============================================================================

void setup() {
#if defined(BOARD_M5STICKC)
    M5.begin();
    M5.Axp.ScreenBreath(7);
    M5.Lcd.setRotation(3);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.println("SO-101 LF");
#endif

    Serial.begin(115200);
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    esp_task_wdt_config_t wdt_cfg = { .timeout_ms = 60000, .idle_core_mask = 0, .trigger_panic = false };
    esp_task_wdt_init(&wdt_cfg);
#else
    esp_task_wdt_init(60, false);
#endif

    // Load saved settings from NVS (mode, axes, IDs, period)
    nvs_load();

    if (!espnow_setup()) {
        Serial.println("FATAL: ESP-NOW setup failed");
        while (1) delay(1000);
    }

    init_uart();
    if (device_mode == MODE_BRIDGE_LEADER) {
        init_uart2();
        Serial.println("UART2 initialized (G26=TX, G36=RX, Leader READ)");
    }

    Serial.println("\n=== SO-101 Leader-Follower ===");
    Serial.printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  my_mac[0], my_mac[1], my_mac[2],
                  my_mac[3], my_mac[4], my_mac[5]);
    Serial.printf("Default: mode=%s axes=%d IDs=[", MODE_NAMES[device_mode], n_axes);
    for (int i = 0; i < n_axes; i++)
        Serial.printf("%d%s", servo_ids[i], i < n_axes - 1 ? "," : "");
    Serial.printf("] period=%luus (%.1fHz)\n",
                  (unsigned long)control_period_us,
                  1000000.0f / control_period_us);
    print_help();

    // Auto-start scanning
    state = ST_SCANNING;
    first_scan = true;
    last_scan_ms = 0;
    Serial.println("→ SCANNING (auto-start)");
}

void loop() {
#if defined(BOARD_M5STICKC)
    if (state != ST_RUNNING) M5.update();
#endif
    poll_serial();

    uint32_t now = millis();

    switch (state) {
    case ST_INIT:
        // Should not reach here (setup transitions to SCANNING)
        break;

    case ST_SCANNING: {
        if (first_scan || (now - last_scan_ms >= SCAN_INTERVAL_MS)) {
            first_scan = false;
            last_scan_ms = now;

            int result;
            if (device_mode == MODE_BRIDGE_LEADER) {
                // Scan Leader arm on UART2
                uint8_t found[16];
                int n = scs_scan_ids(UART_LEADER, found, 16, 2000);
                Serial.printf("SCAN(UART2): found %d servo(s):", n);
                for (int i = 0; i < n; i++) Serial.printf(" %d", found[i]);
                Serial.println();
                result = all_ids_found(found, n, servo_ids, n_axes) ? n_axes : -1;
            } else {
                result = do_scan();  // scan on UART1
            }

            if (result > 0) {
                // All IDs found → transition to RUNNING
                if (device_mode == MODE_LEADER || device_mode == MODE_BRIDGE_LEADER) {
                    follower_torque(false);   // Leader: Torque OFF so arm can be moved by hand
                } else {
                    follower_torque(true);    // Follower: Torque ON to hold position
                }
                reset_stats();
                next_cycle_us = esp_timer_get_time();
                state = ST_RUNNING;
                Serial.printf("→ RUNNING (%s, %d axes, %.1f Hz)\n",
                              MODE_NAMES[device_mode], n_axes,
                              1000000.0f / control_period_us);
                lcd_update();
            } else {
                lcd_update();
            }
        }
        break;
    }

    case ST_RUNNING: {
        if (device_mode == MODE_LEADER || device_mode == MODE_BRIDGE_LEADER) {
            int64_t now_us = esp_timer_get_time();
            if (now_us >= next_cycle_us) {
                int64_t t0 = now_us;
                if (device_mode == MODE_BRIDGE_LEADER) {
                    bridge_leader_read_and_write();
                } else {
                    leader_read_and_send();
                }
                last_cycle_us = (uint32_t)(esp_timer_get_time() - t0);
                next_cycle_us = now_us + control_period_us;
            }
        }
        // Follower logic is in on_espnow_recv (interrupt-driven)

        // Error detection: 10 consecutive cycles with 0 successful reads
        if ((device_mode == MODE_LEADER || device_mode == MODE_BRIDGE_LEADER) && consecutive_fail >= 10) {
            Serial.println("ERR: 10 consecutive read failures → STOPPED");
            state = ST_STOPPED;
            lcd_update();
        }

        // LCD + position print every 1 second
        if (now - lcd_update_ms >= 1000) {
            lcd_update_ms = now;
            lcd_update();
            // Debug: print positions every 1 second (toggle with 'd' command)
            if (debug_pos_output) {
                if (device_mode == MODE_LEADER || device_mode == MODE_BRIDGE_LEADER) {
                    Serial.printf("POS:");
                    for (int i = 0; i < n_axes; i++)
                        Serial.printf(" [%d]=%d", servo_ids[i], leader_positions[i]);
                    Serial.printf("  (cycle=%luus ok=%llu fail=%llu)\n",
                                  (unsigned long)last_cycle_us,
                                  (unsigned long long)leader_read_ok,
                                  (unsigned long long)leader_read_fail);
                } else if (device_mode == MODE_FOLLOWER && follower_last_n_axes > 0) {
                    Serial.printf("RCV:");
                    for (int i = 0; i < follower_last_n_axes; i++)
                        Serial.printf(" [%d]=%d", servo_ids[i], follower_last_pos[i]);
                    Serial.printf("  (recv=%llu write=%llu)\n",
                                  (unsigned long long)follower_recv,
                                  (unsigned long long)follower_write);
                    // Read actual servo positions from Follower's bus
                    uint8_t rbuf[16], resp[32];
                    Serial.printf("ACT:");
                    for (int i = 0; i < n_axes; i++) {
                        { uint8_t tmp[256]; uart_read_bytes(UART_NUM, tmp, sizeof(tmp), 0); }
                        int plen = scs_build_read(rbuf, servo_ids[i], SCS_ADDR_PRESENT_POSITION, 2);
                        uart_write_bytes(UART_NUM, (const char *)rbuf, plen);
                        uart_wait_tx_done(UART_NUM, pdMS_TO_TICKS(5));
                        int rlen = scs_recv_response(UART_NUM, resp, sizeof(resp), 3000);
                        int pos = scs_parse_position(resp, rlen);
                        Serial.printf(" [%d]=%d", servo_ids[i], pos);
                    }
                    Serial.println();
                }
            }
        }
        break;
    }

    case ST_STOPPED:
        // Idle. Wait for 's' command.
        break;
    }
}
