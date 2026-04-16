# SO-101 Leader-Follower (ESP-NOW)

![SO-101 ESP-NOW Standalone](image/SO101_ESPNOW_Standalone.png)

ESP-NOW 経由で SO-101 ロボットアームの Leader-Follower テレオペを行うファームウェア。
WiFi AP 不要の直接 peer-to-peer 通信で、WiFi UDP 版 (SO101_wifi_ap_control) より低遅延。

**デモ動画**: [Twitter / X @robostadion](https://x.com/robostadion/status/2043876691672793417)

## 構成

```
Leader MCU ──UART 1Mbaud──> STS3215 x6 (READ)
     │
     │ ESP-NOW broadcast (25B/pkt, 50Hz, 片方向遅延 1-3ms)
     │
     └──> Follower MCU ──UART 1Mbaud──> STS3215 x6 (SYNC_WRITE)
```

WiFi AP は不要。ESP-NOW は broadcast (FF:FF:FF:FF:FF:FF) で自動接続。

## ハードウェア構成

### MCU (teleop 用) — 以下のいずれか

Leader/Follower 各 1 台で計 2 台必要。同じ MCU 推奨。異種混在は未検証。

| パーツ | リンク |
|-------|-------|
| M5StickC (ESP32-PICO) | [M5Stack 公式](https://shop.m5stack.com/products/stick-c) |
| M5Stack AtomS3 | [M5Stack 公式](https://shop.m5stack.com/products/atoms3-dev-kit-w-0-85-inch-screen) |
| XIAO ESP32-C3 | [Seeed Studio 公式](https://www.seeedstudio.com/Seeed-XIAO-ESP32C3-p-5431.html) |

### サーボバスドライバ基板 (MCU / PC 共通のサーボ接続ハブ)

![XIAO ESP32-C3 on Waveshare UART Serial Driver](image/XIAO-ESP32C3_on_waveshare-UART-Serial-Driver.png)

| パーツ | リンク |
|-------|-------|
| **Waveshare シリアルバスサーボドライバ** | [秋月電子 M-17068](https://akizukidenshi.com/catalog/g/g131227/) |

MCU (XIAO / AtomS3 等) を差し込んで使う **物理プラットフォーム**:
- サーボ connector × 複数 (STS3215 デイジーチェーン対応)
- 5V / 7.4V 電源分配
- 半二重 UART の RX/TX 合流配線
- USB-UART ブリッジも内蔵 (PC 直結も可)

teleop 時は MCU が Waveshare に乗って駆動。**キャリブレーション (`so101_calibrate.py`) 実行時は現状、Waveshare 本体の USB ポートに PC を直接接続する必要がある** (MCU 経由の透過モードは未対応)。

### ロボットアーム

| パーツ | リンク |
|-------|-------|
| **SO-ARM100 / SO-101** (3D print + BOM) | [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) |
| STS3215 サーボ (Feetech) | [Feetech 公式](http://www.feetechrc.com/) |
| LeRobot (学習フレームワーク) | [huggingface/lerobot](https://github.com/huggingface/lerobot) -- **模倣学習は対応検討中** |

## 対応ボード

| env | MCU | サーボ TX/RX | 備考 |
|-----|-----|------------|------|
| `m5stick-c` | M5StickC (ESP32) | GPIO32 / GPIO33 | UART2 (bridge 用): GPIO26 / GPIO36、**動作確認済** |
| `m5stack-atoms3` | M5Stack AtomS3 (ESP32-S3) | GPIO2 / GPIO1 | USB-CDC、**動作確認済** |
| `xiao-esp32c3-scs-recognize` | XIAO ESP32-C3 | GPIO21 / GPIO20 | シングルコア、ESP-NOW 50Hz で動作確認済 |
| `xiao-esp32s3` | XIAO ESP32-S3 | GPIO43 / GPIO44 | ビルド成功、実機動作未検証 |
| `xiao-esp32c6` | XIAO ESP32-C6 | GPIO16 / GPIO17 | **未検証** (Arduino v3/pioarduino 必要) |

## 開発環境

| 項目 | バージョン / 内容 |
|------|----------------|
| **ビルドシステム** | [PlatformIO](https://platformio.org/) |
| **フレームワーク** | **Arduino framework v2** (ESP-IDF 4.4 ベース) |
| **Platform** | `espressif32@6.9.0` |
| UART driver | ESP-IDF `driver/uart.h` (Arduino `Serial` ではなく直接使用、1Mbaud 安定化のため) |
| WiFi stack | ESP-NOW (`esp_now.h`) broadcast, AP 不要 |
| NVS | Arduino `Preferences` (ESP-IDF NVS ラッパ) |

ESP32-C6 は **未検証**。Arduino v3 (pioarduino) が必要 (ESP-IDF 4.4 で C6 サポートなし)。env 定義は [platformio.ini](platformio.ini) に用意済だが実機動作未確認。

## ビルド・アップロード

同一バイナリを 2 台にアップロード。役割はシリアルで設定。

```bash
cd SO101_leader_follower

# M5StickC
pio run -e m5stick-c -t upload --upload-port COMxx

# AtomS3
pio run -e m5stack-atoms3 -t upload --upload-port COMxx
```

## 起動フロー (自動)

```
電源 ON → NVS load (mode/servo/WiFi 設定復元)
       → ESP-NOW init + WiFi 設定適用 (PHY rate / channel / AMPDU)
       → ST_SCANNING (自動開始、5 秒毎にリトライ)
       → 全サーボ ID 応答 → ST_RUNNING (自動遷移)
       → Leader: READ + ESP-NOW broadcast 開始 (50Hz)
       → Follower: ESP-NOW 受信 → SYNC_WRITE 開始
```

**`s` コマンドは不要** (起動時に自動で SCANNING → RUNNING)。サーボ未接続時は SCANNING で待機。

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

モード・サーボ ID・WiFi 設定等はすべて NVS 永続化されるため、**次回起動は電源 ON のみで自動動作開始** (`s` 不要)。
setup() 内で ST_SCANNING に自動遷移し、全サーボ ID 応答確認後に ST_RUNNING へ。

手動停止 (`x`) 後の再開時のみ `s` が必要。

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
| **WiFi / ESP-NOW** | | |
| `R<n>` | PHY rate (0=1M_L, 3=11M_L, 8=24M, ...) | **✓** |
| `C<ch>` | WiFi channel (1-13, Leader/Follower で一致必須) | **✓** |
| `A<0\|1>` | AMPDU off/on (0=11b+g推奨, 1=11b+g+n) | **✓** |
| `P<dBm>` | TX power (2-21 dBm) | - |
| `W` | 現在の WiFi 設定表示 | - |

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
| 片方向遅延 (Leader→Follower) | **1-3 ms** |
| 制御レート | **50 Hz** (デフォルト) |
| サーボ baud | 1 Mbaud |
| UART READ 時間 | ~3.8 ms / 6 軸 |
| WiFi AP | 不要 |
| PC 接続 | 不可 (ESP-NOW は直接通信) |

## トラブルシューティング

| 症状 | 対処 |
|------|------|
| Follower が受信しない (recv=0) | 両方が同じ WiFi channel にいるか確認。`x` → `s` で再初期化 |
| READ 失敗が連続 | サーボ配線 (TX/RX バスバー + DATA) / 5V 電源 / GND 共通を確認 |
| 10 回連続 READ 失敗で自動停止 | `s` で再開始 |
| AtomS3 でシリアルモニタ開くと再起動 | USB-CDC の DTR リセット。`--raw` フラグで抑制 |
| サーボが遅い | `scs_build_sync_write_pos_full(.., 0, 0)` で速度 max 強制済。EEPROM Moving Speed は無視される |

## Python ツール

PC からの設定・テストスクリプトは [python/](python/) を参照。
- `so101_setup.py` -- mode / servo ID / **WiFi channel・PHY rate (NVS)** 設定
- `so101_calibrate.py` -- STS3215 キャリブレーション
- `so101_leader_follower_test.py` -- 追従精度テスト

詳細: [python/README.md](python/README.md)

## 関連プロジェクト

- [RS-485_wireless](https://github.com/uecken/RS-485_wireless) -- 元リポジトリ (WiFi UDP 版、RS-485 bridge 等)

## License

[MIT License](LICENSE)

---

## Appendix: Bridge Leader モード (m2、M5StickC 専用・旧検証用)

本体機能とは **独立した旧検証モード**。通常の teleop 構成 (m0/m1) では使用しません。

UART2 (GPIO26/36) で Leader アームを READ し、組み立てた SYNC_WRITE を UART1 経由で
[rs485_espnow_bridge](https://github.com/uecken/RS-485_wireless/tree/master/rs485_espnow_bridge) の bridge_L に渡すハイブリッド中継モード。
既存の「PC/コントローラ → RS-485 → bridge → ESP-NOW → bridge → Follower」構成に
ESP32 teleop を挟み込む検証用。

- M5StickC 限定 (UART2 用ピンが必要)
- シリアル `m2` で選択
- FW 側コードは維持、README 本文から独立させただけ
