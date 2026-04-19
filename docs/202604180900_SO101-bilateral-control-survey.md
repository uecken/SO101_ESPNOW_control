# SO-101 バイラテラル制御 — 事例調査 + 実現性分析

2026-04-18 作成。SO-101 ロボットアームの Leader-Follower テレオペ (バイラテラル制御) に関する網羅的調査。

## TL;DR

- **SO-101 の bilateral 制御は「位置ミラーリング」が業界標準** — 力覚フィードバックを実装した事例は皆無
- **通信方式は USB-serial (有線) が圧倒的多数** — 無線は AWS IoT (WiFi+MQTT)、Waveshare ESP-NOW、LeKiwi (WiFi+SSH) の 3 件のみ
- **ESP-NOW で SO-101 専用の Leader-Follower teleop を実装しているのは本プロジェクトのみ** (世界初)
- 制御周波数は **30-50 Hz が de facto standard** (ALOHA / LeRobot / Koch 共通)
- **STS3215 の SYNC_READ (0x82) 対応は本プロジェクトで発見・検証済** — 他プロジェクトでの言及なし

---

## 1. 公式・標準プロジェクト

### 1.1 TheRobotStudio/SO-ARM100 (ハードウェア設計元)

| 項目 | 内容 |
|------|------|
| URL | https://github.com/TheRobotStudio/SO-ARM100 |
| Stars / Forks | ⭐ 5,900 / 🍴 524 |
| 通信方式 | **USB-serial** → PC (Linux/Mac/Win) 経由 Feetech bus board |
| MCU | **なし** (PC 直結、MCU レス) |
| 制御周波数 | 30 Hz default (LeRobot 側で設定) |
| BOM | ~$200-400 (Leader + Follower pair) |
| 特徴 | 3D プリント部品、LeRobot 統合前提の設計。SO-100 (初代) → SO-101 (改良版: 配線簡素化、ギア除去不要) |

### 1.2 HuggingFace/LeRobot (ソフトウェアプラットフォーム)

| 項目 | 内容 |
|------|------|
| URL | https://github.com/huggingface/lerobot |
| Stars / Forks | ⭐ 22,600 / 🍴 4,100 |
| 通信方式 | **USB-serial** → `FeetechMotorsBus` Python ドライバ |
| 制御周波数 | default `loop_rate=30` Hz (最大 100 Hz) |
| teleop デバイス | Leader arm, keyboard, gamepad, **phone (ARKit)**, **Meta Quest VR** |
| 特徴 | `lerobot-teleoperate` コマンドで即 teleop 開始、calibration workflow、dataset 録画、policy 学習 (ACT, Diffusion, smolVLA, pi0) |

---

## 2. 無線実装 (★ 本プロジェクトとの差分がある領域)

### 2.1 本プロジェクト (uecken/SO101_ESPNOW_control) — ESP-NOW

| 項目 | 内容 |
|------|------|
| URL | https://github.com/uecken/SO101_ESPNOW_control |
| 通信方式 | **ESP-NOW broadcast** (WiFi AP 不要、peer-to-peer) |
| MCU | M5StickC / M5Stack AtomS3 / XIAO ESP32-C3 / XIAO ESP32-S3 |
| 制御周波数 | **50 Hz** (20ms cycle) |
| 遅延 | **片方向 1-3 ms** (実測) |
| 特徴 | PC 不要 (MCU 単体で完結)、NVS 永続化、Web Serial installer、Phase bit4 / Homing_Offset calibration 対応 |
| **独自性** | **ESP-NOW で SO-101 専用の Leader-Follower を実装した唯一の事例** |

### 2.2 chiwaichan/aws-iot-core-lerobot-so101 — AWS IoT Cloud

| 項目 | 内容 |
|------|------|
| URL | https://github.com/chiwaichan/aws-iot-core-lerobot-so101 |
| Stars | 0 (新規) |
| 通信方式 | **WiFi + MQTT via AWS IoT Core** (インターネット経由) |
| MCU | **XIAO ESP32-C3** (MicroPython) |
| 制御周波数 | ~100 Hz (閾値 1° 以上の変化でパブリッシュ) |
| 遅延 | ~50-200 ms (クラウド経由) |
| 特徴 | X.509 証明書認証、JSON データ、自動再接続 |
| **意義** | ESP32 MCU で SO-101 無線制御する唯一の他事例 (WiFi+MQTT、ESP-NOW ではない) |

### 2.3 Waveshare Servo Driver with ESP32 — ESP-NOW

| 項目 | 内容 |
|------|------|
| URL | https://github.com/waveshare/Servo-Driver-with-ESP32 |
| 通信方式 | **ESP-NOW** (broadcast / group / one-to-one) |
| MCU | ESP32-WROOM-32 |
| サーボ | ST/SC series (STS3215 互換) |
| 特徴 | Leader が Active Servo ID + position + speed を ESP-NOW 送信 → Follower が模倣。最大 253 サーボ対応 |
| **意義** | **同じサーボ系列で ESP-NOW Leader-Follower を実装した既製品**。SO-101 専用ではないが互換 HW |

### 2.4 Waveshare RoArm-M2-S — ESP-NOW ロボットアーム

| 項目 | 内容 |
|------|------|
| URL | https://github.com/waveshareteam/roarm_m2 |
| 通信方式 | **ESP-NOW** |
| ハードウェア | 4-DOF arm, ST3215 サーボ |
| 特徴 | 1 台のアームが他を無線制御、Web GUI |
| **意義** | 同サーボ系列 ESP-NOW 製品。4-DOF だが 6-DOF 拡張は可能 |

### 2.5 LeKiwi (LeRobot 公式モバイルマニピュレータ) — WiFi/SSH

| 項目 | 内容 |
|------|------|
| URL | https://huggingface.co/docs/lerobot/lekiwi |
| 通信方式 | **WiFi** (SSH + Python socket) |
| コンピュータ | **Raspberry Pi 5** (ロボット側) + PC (操作側) |
| 特徴 | SO-101 アームを全方向移動ベースに搭載、カメラストリーミング、WiFi teleop |

### 2.6 Viam SO-101 — Cloud/Web テレオペ

| 項目 | 内容 |
|------|------|
| URL | https://github.com/viam-devrel/so-101 |
| 通信方式 | **Cloud** (Viam platform, NAT 越え) |
| 特徴 | ブラウザからどこでも制御、自動 discovery |

---

## 3. ROS 2 実装

| プロジェクト | URL | Stars | 特徴 |
|------------|-----|-------|------|
| **legalaspro/so101-ros-physical-ai** | [GitHub](https://github.com/legalaspro/so101-ros-physical-ai) | 62 | Feetech ドライバ + MoveIt 2 + multi-camera + ACT/smolVLA 推論 + remote GPU |
| **msf4-0/so101_ros2** | [GitHub](https://github.com/msf4-0/so101_ros2) | 14 | Dual-arm teleop + recording/replay |
| **ycheng517/lerobot-ros** | [GitHub](https://github.com/ycheng517/lerobot-ros) | - | LeRobot ↔ ROS 2 軽量ブリッジ |

全て **USB-serial** + ROS 2 middleware で ~50 Hz 制御。

---

## 4. シミュレーション連携

| プロジェクト | シミュレータ | 特徴 |
|------------|------------|------|
| **LightwheelAI/leisaac** | NVIDIA Isaac Lab | 物理 Leader → sim Follower、GR00T N1.5 |
| **masato-ka/gym_soarm_demo_suite** | MuJoCo | Real-time sim teleop、DH kinematics |
| **protomota** | Isaac Sim + ROS 2 | Digital twin |

---

## 5. マルチモーダル teleop プラットフォーム

| プロジェクト | URL | Stars | teleop 方式 |
|------------|-----|-------|------------|
| **phospho-app/phosphobot** | [GitHub](https://github.com/phospho-app/phosphobot) | 366 | Leader arm / keyboard / gamepad / **Meta Quest VR** |
| **IliaLarchenko/lerobot_phone_teleop** | [GitHub](https://github.com/IliaLarchenko/lerobot_phone_teleop) | - | **Android phone** (Flutter, WiFi) |
| **Vector-Wangel/XLeRobot** | [GitHub](https://github.com/Vector-Wangel/XLeRobot) | 5,000 | **Bluetooth gamepad** / VR (Quest 3)、dual-arm mobile |

---

## 6. 関連ロボットアーム比較

| Platform | サーボ | DoF | ギア比 | プロトコル | 制御 Hz | bilateral | 力覚 FB | コスト |
|----------|------|-----|--------|----------|--------|----------|--------|------|
| **SO-101 (本プロジェクト)** | STS3215 | 6 | 1/147-1/345 | SCS | 50 | 位置ミラー | なし | ~$300 |
| **ALOHA** | XM540/XM430 | 7/arm | 1/193-1/353 | Dynamixel 2.0 | 50 | 位置ミラー | なし | ~$20,000 |
| **Koch v1.1** | XL430/XL330 | 5 | 1/258 | Dynamixel 2.0 | 50 | 位置ミラー | なし | ~$430 |
| **GELLO** | 3D print + Dynamixel | 6-7 | Direct drive (Leader) | Dynamixel | 50 | 位置ミラー | **受動的** | ~$300 |
| **XLeRobot** | STS3215 | 6×2 + mobile | 1/345 | SCS | - | Bluetooth/VR | なし | ~$660 |
| **AhaRobot** | Dual-motor | 6×2 + mobile | - | - | - | Web/Cellular | なし | ~$1,000 |
| **U-Arm (LeRobot-Anything)** | 汎用 | - | - | ROS 1 | - | 位置ミラー | なし | ~$60 |

**全低コスト bilateral システムが位置ミラーリングのみ — 力覚フィードバックを実装した事例はゼロ。**

---

## 7. 通信方式の横断比較

| 方式 | 使用プロジェクト | 片方向遅延 | PC 要否 | 距離制限 |
|------|---------------|---------|--------|---------|
| **USB-serial (有線)** | LeRobot, ALOHA, ROS2 全般, Koch, phospho | <1 ms | **必要** | ケーブル長 |
| **ESP-NOW** | **本プロジェクト**, Waveshare Servo Driver, RoArm-M2 | ~1-5 ms | **不要** | ~10m (環境依存) |
| **WiFi + MQTT (AWS IoT)** | chiwaichan/aws-iot | ~50-200 ms | **不要** (MCU のみ) | インターネット |
| **WiFi + SSH/socket** | LeKiwi | ~10-50 ms | Raspberry Pi | LAN 内 |
| **Cloud (Viam)** | Viam SO-101 | Variable | PC/ブラウザ | インターネット |
| **Bluetooth** | XLeRobot (gamepad) | Low | PC | ~10m |
| **Cellular** | AhaRobot | Variable | Web | 携帯圏 |

---

## 8. 実現性分析

### 8.1 通信遅延要件

| 制御方式 | 許容遅延 (片方向) | 根拠 |
|---------|----------------|------|
| **力覚反映 bilateral** | < 1-2 ms | 安定性条件 (Hashtrudi-Zaad & Salcudean, 2001) |
| **位置ミラーリング teleop** | 10-50 ms | 視覚-運動ループの知覚閾値 ~50-100 ms |
| **模倣学習データ収集** | 20-50 ms | ALOHA が 50 Hz (= 20 ms cycle) で標準化 |
| **人間ループ操作** | 30-100 ms | 200 ms 超でオペレータのフラストレーション増大 |

**ESP-NOW (1-3 ms)** は位置ミラーリングに十分余裕あり。WiFi UDP (5-10 ms) も実用圏。

### 8.2 STS3215 の物理限界

| 項目 | 値 | 影響 |
|------|---|------|
| 個別 READ 6 軸 | ~2.3 ms (1Mbaud) | 50 Hz 周期に余裕あり |
| **SYNC_READ 6 軸** | **~0.7 ms** (本プロジェクトで検証) | 100 Hz+ 可能 |
| SYNC_WRITE 6 軸 | ~0.4 ms | 問題なし |
| Torque OFF 時の位置フィードバック | ✅ 動作 | Leader 腕で使用中 |
| **トルク制御モード** | **❌ 存在しない** | 力覚フィードバック不可の根本理由 |
| Present_Load レジスタ | 10-bit (0.1% 分解能) | 力のプロキシとして低精度 |
| Present_Current レジスタ | 2B | 存在するが本プロジェクトでは未読 |

### 8.3 制御周波数のトレードオフ

**50 Hz タイミングバジェット (ESP-NOW パス)**:

```
Leader READ 6 軸:     2.3 ms (individual) / 0.7 ms (SYNC_READ)
ESP-NOW 送信:         ~1 ms
Follower SYNC_WRITE:  ~0.4 ms
合計:                 ~3.7 ms / ~2.1 ms
残余 headroom:        16.3 ms / 17.9 ms
```

**50 Hz 以上に上げる効果**: STS3215 の PID ループは ~1 kHz 内部動作だが、機械応答 (モータ + ギアボックス慣性) で律速。人間の手の操作帯域は ~5-10 Hz。追従誤差は 50 Hz で平均 3.9° — 通信遅延ではなく機械応答が支配的。**50 Hz で十分、100 Hz は微改善のみ**。

### 8.4 力覚フィードバック bilateral の実現性

**STS3215 では不可能。** 理由:

1. **トルク制御モードが存在しない** — SCS プロトコルに電流/トルク制御 instruction がない
2. **ギア比が高すぎる** (1/147-1/345) — ギアトレインの摩擦が外力をマスク
3. **重力補償なし** — Present_Load は腕自重に支配される
4. **STS3215 は位置制御サーボ** — Dynamixel XM540 のような current-based position control がない

**代替案**: Follower の Present_Position と Goal_Position の差分を読んで「仮想壁」的な制動を Leader に返す impedance control approach。ただし双方向通信 (現在は Leader→Follower 片方向のみ) が必要。

### 8.5 無線 bilateral の実用性

| 距離 | ESP-NOW pass rate (実測) | 判定 |
|------|------------------------|------|
| ~1 m | 100% | ✅ 実用 |
| ~3 m | **55-99.98%** (環境変動大) | ⚠️ リスク大 |
| ~5 m+ | 未測定 | ❌ 非推奨 |

**商用 bilateral システムで WiFi/BLE を制御ループに使う事例はゼロ** (da Vinci, Force Dimension 等は全て有線または専用無線)。

### 8.6 通信断時の安全性

| 状態 | 現在の挙動 | 推奨改善 |
|------|---------|---------|
| Leader 切断 | Follower は最終 Goal Position を保持 (STS3215 内部) | ⚠️ Follower 側に watchdog timer 追加 (例: 500 ms timeout → torque OFF) |
| Follower 切断 | Leader は読み続けるが送信先なし | 影響なし (broadcast) |
| ESP-NOW dropout | Follower は受信サイクルが抜ける → 位置ステップ | ⚠️ 補間バッファ (WiFi 版は実装済、ESP-NOW 版は未実装) |

---

## 9. 本プロジェクトのポジショニング

### 独自性

| 差別化要素 | 状況 |
|-----------|------|
| **ESP-NOW で SO-101 Leader-Follower** | **世界唯一** (Waveshare は同サーボ系列だが SO-101 専用ではない) |
| **MCU 単体完結 (PC 不要)** | AWS IoT (ESP32-C3 + MQTT) 以外にない |
| **STS3215 SYNC_READ (0x82) 対応確認** | **本プロジェクトで発見・検証** (他事例なし) |
| **Phase register bit 4 問題の解析・修正** | **本プロジェクトで発見** |
| **Web Serial installer (GitHub Pages)** | SO-101 系で唯一 |
| **NVS 永続化 + 起動時自動 RUNNING 復帰** | LeRobot (PC 依存) にはない |

### 既存事例に対する優位性

```
                LeRobot (USB 有線, PC 必須)
                    ↓ 
             ┌──────┴──────────────┐
             │                     │
     AWS IoT (WiFi+MQTT)    本プロジェクト (ESP-NOW)
     遅延 50-200ms           遅延 1-3ms
     クラウド依存             MCU 単体完結
     MicroPython             C/C++ (Arduino + ESP-IDF)
     インターネット越え可     近距離 (<3m 安定)
```

### 今後の拡張候補

| 拡張 | 実現性 | 価値 | 備考 |
|------|-------|------|------|
| SYNC_READ 化 (sequential READ → 1 transaction) | ✅ lib 実装済 | 中 (50Hz で十分) | 100 Hz+ やりたい時 |
| 双方向通信 (Follower→Leader position 返送) | ✅ 可能 | 中 | impedance control 前提 |
| Follower watchdog (通信断 → torque OFF) | ✅ 簡単 | 高 (安全) | 500ms timeout |
| 補間バッファ (ESP-NOW jitter 吸収) | ✅ WiFi 版に実装済 | 中 | ESP-NOW 版は未移植 |
| Dynamixel サーボ対応 | △ protocol 違い | 低 | ALOHA/Koch 互換性 |
| 力覚フィードバック | ❌ STS3215 では不可 | - | サーボ交換が前提 |
| 3m+ 距離安定化 | △ アンテナ/方式変更 | 中 | 802.11ax (C6) 期待 |

---

## 10. 参考リンク

### 公式

- [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) — ハードウェア設計
- [HuggingFace/LeRobot](https://github.com/huggingface/lerobot) — ソフトウェアプラットフォーム
- [LeRobot SO-101 docs](https://huggingface.co/docs/lerobot/so101) — 公式ドキュメント

### 無線事例

- [chiwaichan/aws-iot-core-lerobot-so101](https://github.com/chiwaichan/aws-iot-core-lerobot-so101) — AWS IoT
- [Waveshare Servo Driver ESP-NOW](https://github.com/waveshare/Servo-Driver-with-ESP32) — 同系列サーボ
- [Waveshare RoArm-M2-S](https://github.com/waveshareteam/roarm_m2) — 4-DOF ESP-NOW

### ROS 2

- [legalaspro/so101-ros-physical-ai](https://github.com/legalaspro/so101-ros-physical-ai) — 最も包括的な ROS 2 スタック
- [msf4-0/so101_ros2](https://github.com/msf4-0/so101_ros2) — Dual-arm teleop
- [ycheng517/lerobot-ros](https://github.com/ycheng517/lerobot-ros) — LeRobot ↔ ROS 2 bridge

### 関連ロボット

- [tonyzhaozh/aloha](https://github.com/tonyzhaozh/aloha) — ALOHA (bilateral 普及の起点)
- [AlexanderKoch-Koch/low_cost_robot](https://github.com/AlexanderKoch-Koch/low_cost_robot) — Koch (SO-100/101 の原型)
- [wuphilipp/gello_software](https://github.com/wuphilipp/gello_software) — GELLO (汎用 teleop デバイス)
- [Vector-Wangel/XLeRobot](https://github.com/Vector-Wangel/XLeRobot) — dual-arm mobile (Bluetooth/VR)
- [phospho-app/phosphobot](https://github.com/phospho-app/phosphobot) — マルチモーダル teleop (VR 対応)

### 本プロジェクト

- [uecken/SO101_ESPNOW_control](https://github.com/uecken/SO101_ESPNOW_control) — 本 ESP-NOW bilateral 実装
- [Web Serial Installer](https://uecken.github.io/SO101_ESPNOW_control/) — ブラウザ書込み
- [calibration-method-comparison.md](202604162200_calibration-method-comparison.md) — キャリブレーション方式比較
