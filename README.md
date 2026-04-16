# SO-101 Leader-Follower (ESP-NOW)

ESP-NOW 経由で SO-101 ロボットアームの Leader-Follower テレオペを行うファームウェア。
WiFi AP 不要の直接 peer-to-peer 通信で、WiFi UDP 版 (SO101_wifi_ap_control) より低遅延。

## 構成

```
Leader MCU ──UART 1Mbaud──> STS3215 x6 (READ)
     │
     │ ESP-NOW broadcast (14B, 50Hz, RTT 1-3ms)
     │
     └──> Follower MCU ──UART 1Mbaud──> STS3215 x6 (SYNC_WRITE)
```

WiFi AP は不要。ESP-NOW は broadcast (FF:FF:FF:FF:FF:FF) で自動接続。

## 対応ボード

| env | MCU | サーボ TX/RX | 備考 |
|-----|-----|------------|------|
| `m5stick-c` | M5StickC (ESP32) | GPIO32 / GPIO33 | UART2 (bridge 用): GPIO26 / GPIO36 |
| `m5stack-atoms3` | M5Stack AtomS3 (ESP32-S3) | GPIO2 / GPIO1 | USB-CDC |
| `xiao-esp32c3-scs-recognize` | XIAO ESP32-C3 | GPIO21 / GPIO20 | シングルコア |

## ビルド・アップロード

同一バイナリを 2 台にアップロード。役割はシリアルで設定。

```bash
cd SO101_leader_follower

# M5StickC
pio run -e m5stick-c -t upload --upload-port COMxx

# AtomS3
pio run -e m5stack-atoms3 -t upload --upload-port COMxx
```

## 初回設定 (書込み後 device 毎に 1 回)

シリアルモニタ (115200 baud) で各デバイスに接続し、役割を設定:

### Leader 側

```
m0                 ← LEADER モード
s                  ← 開始
```

### Follower 側

```
m1                 ← FOLLOWER モード
s                  ← 開始
```

モードは NVS 保存されるため、次回起動は電源 ON + `s` で動作。

### Bridge Leader モード (オプション)

```
m2                 ← BRIDGE_LEADER (UART2 から Leader アーム読取 → UART1 出力)
```

M5StickC の UART2 (GPIO26/36) に接続した Leader アームの位置を読み、UART1 経由で bridge 出力する特殊モード。

## シリアルコマンド一覧

| コマンド | 説明 | NVS |
|---------|------|-----|
| `m<0\|1\|2>` | モード (0=LEADER, 1=FOLLOWER, 2=BRIDGE_LEADER) | ✓ |
| `s` | 開始 (ESP-NOW 初期化 → サーボスキャン → RUNNING) | - |
| `x` | 停止 (Follower は torque OFF) | - |
| `J<id1,id2,...>` | サーボ ID (例: `J1,2,3,4,5,6`) | ✓ |
| `I<n>` | 軸数 (1-6) | ✓ |
| `b<baud>` | サーボ baud (デフォルト 1000000) | ✓ |
| `G<us>` | 制御周期 us (デフォルト 20000 = 50Hz) | ✓ |
| `d` | デバッグ出力 ON/OFF | - |
| `?` | ステータス表示 | - |
| `h` | ヘルプ | - |

## 動作確認

```
?
```

正常時の表示 (Leader):
```
STATUS state=RUNNING mode=LEADER baud=1000000 axes=6 period=20000us
  espnow: send_ok=150 send_fail=0 recv=0
  read_ok=900 fail=0 cycle=3800us
  pos: [1]=1812 [2]=2040 [3]=2050 [4]=2020 [5]=1980 [6]=1382
```

正常時の表示 (Follower):
```
STATUS state=RUNNING mode=FOLLOWER baud=1000000 axes=6 period=20000us
  espnow: send_ok=0 send_fail=0 recv=150
  write_ok=150
```

## SCS プロトコル (STS3215)

共通ライブラリ [lib/scs_feetech_servo/scs_feetech_servo.h](lib/scs_feetech_servo/scs_feetech_servo.h) を使用:

| 操作 | 関数 | アドレス |
|------|------|---------|
| Position READ | `scs_build_read()` + `scs_parse_position()` | 0x38 (PRESENT_POSITION, 2B) |
| Position WRITE | `scs_build_sync_write_pos_full(.., 0, 0)` | 0x2A (GOAL_POSITION + TIME + SPEED, 6B/servo) |
| Torque ON/OFF | `scs_build_torque()` | - |
| PING | `scs_build_ping()` | - |

パケットフォーマット: `[0xFF][0xFF][ID][LEN][INST][PARAMS...][CHECKSUM]`

## 性能

| 項目 | 値 |
|------|---|
| 通信方式 | ESP-NOW broadcast |
| RTT | **1-3 ms** |
| 制御レート | **50 Hz** (デフォルト) |
| サーボ baud | 1 Mbaud |
| UART READ 時間 | ~3.8 ms / 6 軸 |
| WiFi AP | 不要 |
| PC 接続 | 不可 (ESP-NOW は直接通信) |

## WiFi UDP 版との比較

| | SO101_leader_follower (ESP-NOW) | SO101_wifi_ap_control (WiFi UDP) |
|---|---|---|
| RTT | **1-3ms** | 8-11ms |
| 制御レート | **50Hz** | 10-15Hz |
| WiFi AP | 不要 | 必要 |
| PC からの制御 | 不可 (直接) | UDP で可能 |
| 線形補間 | なし (ジッタ小さく不要) | あり |
| 設定変更 | シリアルのみ | シリアル + UDP |

## トラブルシューティング

| 症状 | 対処 |
|------|------|
| Follower が受信しない (recv=0) | 両方が同じ WiFi channel にいるか確認。`x` → `s` で再初期化 |
| READ 失敗が連続 | サーボ配線 (TX/RX バスバー + DATA) / 5V 電源 / GND 共通を確認 |
| 10 回連続 READ 失敗で自動停止 | `s` で再開始 |
| AtomS3 でシリアルモニタ開くと再起動 | USB-CDC の DTR リセット。`--raw` フラグで抑制 |
| サーボが遅い | `scs_build_sync_write_pos_full(.., 0, 0)` で速度 max 強制済。EEPROM Moving Speed は無視される |

## 関連プロジェクト

- [RS-485_wireless](https://github.com/uecken/RS-485_wireless) -- 元リポジトリ (WiFi UDP 版、RS-485 bridge 等)
