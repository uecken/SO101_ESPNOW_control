# SO-101 Leader-Follower Python スクリプト

SO-101 ESP-NOW Leader-Follower テレオペ制御のテスト・設定・キャリブレーション用スクリプト集。

FW 側の使い方は [../README.md](../README.md) を参照。

## スクリプト一覧 (現行)

| スクリプト | 用途 |
|-----------|------|
| `so101_setup.py` | NVS 設定 (Leader/Follower モード、軸数、ID、制御周期) |
| `so101_calibrate.py` | STS3215 キャリブレーション (LeRobot 互換、サーボ直結) |
| `so101_leader_follower_test.py` | ESP-NOW Leader-Follower 追従精度テスト (メイン) |

### old/ (旧アーキテクチャ用、参考保管)

| スクリプト | 旧用途 |
|-----------|-------|
| `old/so101_single_servo_test.py` | 単体サーボ debug (bridge FW 前提) |
| `old/so101_bridge_test.py` | PC 経由透過 bridge テスト (rs485_espnow_bridge) |
| `old/so101_uart_bridge_test.py` | UART 透過 bridge 両側テスト (rs485_espnow_bridge) |
| `old/test_results_20260413.txt` | 2026-04-13 追従精度測定ログ (95.2% / 15s) |

現構成は consolidated FW (SO101_leader_follower) を 2 台で動かすため、`old/` の bridge 系スクリプトは不要。

## 依存関係

```
pip install pyserial
```

## 使い方

### 1. NVS 設定 (so101_setup.py)

M5StickC / XIAO ESP32C3 の Leader/Follower モードを設定。NVS に保存されるため、次回起動時に自動復帰。

```bash
# Leader に設定
python so101_setup.py --port COM28 --leader

# Follower に設定
python so101_setup.py --port COM29 --follower

# ステータス確認
python so101_setup.py --port COM28 --status

# 開始 / 停止
python so101_setup.py --port COM28 --start
python so101_setup.py --port COM28 --stop

# デバッグ出力 ON/OFF
python so101_setup.py --port COM28 --debug

# カスタム設定
python so101_setup.py --port COM28 --mode 0 --axes 6 --ids 1,2,3,4,5,6 --period 20000

# 任意コマンド送信
python so101_setup.py --port COM28 --cmd "m0"

# デバイスのヘルプ表示
python so101_setup.py --port COM28 --help-device
```

### 2. キャリブレーション (so101_calibrate.py)

STS3215 サーボの Min/Max Position と Homing Offset を設定。LeRobot と同じ手順。

```bash
# フルキャリブレーション (EEPROM 書き込み + JSON 保存)
python so101_calibrate.py --port COM24 --save calibration.json

# 現在の値を確認
python so101_calibrate.py --port COM24 --read-only

# 全サーボを 0-4095 にリセット
python so101_calibrate.py --port COM24 --reset
```

**注意**: キャリブレーションは USB-UART アダプタ (1Mbaud) でサーボに直結して実行。M5StickC 経由ではない。

### 2b. WiFi / ESP-NOW 設定 (NVS 保存)

so101_setup.py の WiFi フラグで channel / PHY rate / AMPDU を runtime 変更・NVS 保存できる。

```bash
python so101_setup.py --port COM28 --wifi-channel 6      # ch 1-13
python so101_setup.py --port COM28 --wifi-phy 3          # 3=11M_L (default)
python so101_setup.py --port COM28 --wifi-ampdu 0        # 0=off (推奨), 1=on
python so101_setup.py --port COM28 --wifi-power 12       # 2-21 dBm (揮発)
python so101_setup.py --port COM28 --wifi-show           # 現在設定表示

# 複合
python so101_setup.py --port COM28 --wifi-channel 6 --wifi-phy 3 --wifi-ampdu 0
```

**Leader と Follower は同じ channel / protocol (AMPDU) が必須**。
両機に順次実行 (channel 不一致中は通信停止):

```bash
for com in COM28 COM29; do
  python so101_setup.py --port $com --wifi-channel 6
done
```

### 3. Leader-Follower 追従テスト (so101_leader_follower_test.py)

ESP-NOW 経由の追従精度を計測。Leader の POS と Follower の ACT (実際の位置) を比較。

```bash
# 15 秒テスト
python so101_leader_follower_test.py --leader COM22 --follower COM8

# 30 秒テスト
python so101_leader_follower_test.py --leader COM22 --follower COM8 --duration 30
```

**注意**: Leader/Follower の USB Serial を開くため、M5StickC で DTR リセットが発生する。

(旧アーキテクチャ向け `old/so101_bridge_test.py` / `old/so101_uart_bridge_test.py` / `old/so101_single_servo_test.py` は参考用として残存。rs485_espnow_bridge FW が別途必要)

## COM ポート (参考)

COM 番号は USB 再接続で変わるため、都度確認が必要。

```powershell
# Windows
Get-CimInstance Win32_PnPEntity | Where-Object { $_.Name -match 'COM\d' } | Select Name, DeviceID | Sort Name
```

### M5StickC 構成

| COM | 役割 | USB DeviceID |
|-----|------|-------------|
| COM22 | Leader (m0) | `FTDIBUS\...\B&3A26F7AC` |
| COM8 | Follower (m1) / bridge_L | `FTDIBUS\...\0000` |
| COM14 | bridge_R | `FTDIBUS\...\A&EE8C903` |

### XIAO ESP32C3 構成

| COM | 役割 | USB DeviceID |
|-----|------|-------------|
| COM28 | Leader (m0) | `USB\VID_303A&PID_1001\9&200C7791` |
| COM29 | Follower (m1) | `USB\VID_303A&PID_1001\A&29682071` |

## ファームウェアモード

| モード | コマンド | 用途 |
|--------|---------|------|
| LEADER (m0) | `m0` | Leader: READ → ESP-NOW 送信 |
| FOLLOWER (m1) | `m1` | Follower: ESP-NOW 受信 → SYNC_WRITE |
| BRIDGE_LEADER (m2) | `m2` | UART2 で Leader READ → UART1 で bridge 経由 WRITE |
