# STS3215 Calibration 方法の比較 — Feetech / LeRobot / SO101_leader_follower

2026-04-16 セッションで SO-101 teleop の 2 大バグ (Follower ID=2 突然反転 / Leader ID=3 Range 2089 vs Follower 3965 の大差) を根治する過程で判明した、**各実装層の責務分担 + 正しい calibration 手順** をまとめる。

## TL;DR

| 実装 | 責務 | ソース |
|------|------|--------|
| **Feetech STS3215 ファームウェア** | `Homing_Offset` を内部で自動適用 (Read / Write 両方向) + Operating_Mode / Min/Max Limit / Phase register の物理挙動 | サーボ内部 firmware |
| **LeRobot** | EEPROM に `Homing_Offset` / `Min_Position_Limit` / `Max_Position_Limit` を書込み + ホスト側で clamp & normalize ([-100, +100] など) | [`feetech.py`](https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/feetech/feetech.py), [`motors_bus.py`](https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/motors_bus.py) |
| **SO101_leader_follower FW** | **Homing_Offset を一切意識しない pass-through**。Leader の Present_Position をそのまま Follower の Goal_Position へ書込み。offset/clamp 処理なし | [`src/main.cpp`](../src/main.cpp) |
| **[so101_calibrate.py](../python/so101_calibrate.py)** | LeRobot 流儀を Python で再現。EEPROM 書込みまで担当 (ホスト normalize は不要、teleop FW が薄いため) | 本 repo |

## 1. なぜ compatibility が壊れやすかったか

### 1-1. 「中間ポーズ」の定義が曖昧

LeRobot の `set_half_turn_homings()` は `Homing_Offset = 現在 raw - 2047` を書き込む。**ユーザーが「中間」と思う姿勢の raw encoder 値が Offset を決定** するため:

- **機械中点** (両端の物理ストッパの中間) に置いた場合 → Present_Position が `[0, 4095]` 内の両側均等に展開 → wrap 発生しない
- **視覚的に「真っ直ぐ」な姿勢** に置いた場合 → 機械中点からズレて偏った位置になり、Offset が不適切 → 物理可動域の一方が 0 や 4095 を跨ぎ、**Present_Position が wrap** して teleop 中に急激な値ジャンプ発生

実測: Leader ID=3 で視覚中央 (raw=2128) で Step 2 実行すると Offset=+85、Range 計測で Min=39・Max=2126 だが、elbow をもう一方向に動かすと raw≈19 まで行き `Present = 19 - 85 = -66 → wrap to 4030` となって teleop 破綻。

→ **docs/[image/s0101_calibration_position.png](../image/s0101_calibration_position.png) の姿勢 (mechanical middle)** で Step 2 実行するのが正解。

### 1-2. STS3215 Phase register bit 4 の個体差

Feetech の STS3215 は **`Phase` レジスタ (EEPROM 0x12) bit 4** が angle-feedback mode を決める:

| bit 4 | 挙動 | Present_Position 報告値 |
|-------|------|------------------------|
| **= 0** | 正規 `[0, resolution-1]` wrap モード | **0-4095 の uint16** (raw 0 付近で wrap) |
| **= 1** | sign-magnitude 符号拡張モード | **bit15=符号 + bits0-14=絶対値** (raw 零跨ぎで 0x8000+ の値) |

**SO-101 Leader の非 1/345-gear サーボ (ID=1, 3, 4, 5, 6) は出荷時 bit 4 = 1** (実測)、Follower の全 1/345-gear サーボは bit 4 = 0 (実測)。この個体差が以下の症状を生む:

- Leader サーボから Present_Position=`32826` (= 0x807A) が返る → ホスト側で二進二の補数と誤解釈すると `-32710` (見掛け上の multi-turn 異常値) に見える
- 以前の我々の FW filter `scs_is_single_turn_position()` が `>4095` を弾く実装だったため、境界跨ぎ時に Leader が Follower への値更新を停止 → Follower 停止 → ユーザーが腕を戻すと急追従 → **「突然動く」誤動作**

LeRobot は `configure_motors()` で全 STS3215 の Phase bit 4 を強制 0 にクリア:

```python
# LeRobot feetech.py より抜粋
if self.motors[motor].model == "sts3215":
    phase = self.read("Phase", motor, normalize=False)
    if phase & 0x10:
        self.write("Phase", motor, phase & ~0x10)
```

SO-101 でこれを行わないと、**同じ SO-101 組立でも Leader / Follower で Present_Position の符号化が異なる** という罠に落ちる。

### 1-3. Operating_Mode (EEPROM 0x21) の意味

```
0: Position (Servo) mode       ← 単回転、Min/Max Limit でクランプ
1: Wheel (continuous rotation) ← 連続回転 (速度制御)
2: PWM mode
3: Step Servo mode             ← 多回転位置制御 (signed 16-bit 累積)
```

工場出荷は 0 が多いが、以前のユーザーが 3 (Step) や 1 (Wheel) に書き換えていた個体があると、Present_Position が 0-4095 ではなく signed 16-bit 累積 (`±32768` 領域) で返ってくる → 見掛け上の multi-turn。

## 2. 各実装の処理フロー詳細

### 2-1. Feetech STS3215 firmware (サーボ内部)

EEPROM 0x09-0x0B (`Min/Max Angle Limit`), 0x12 (`Phase`), 0x1F (`Homing_Offset`), 0x21 (`Operating Mode`), 0x38 (`Present Position`), 0x2A (`Goal Position`) をサーボ自身が解釈。

```
┌─ 物理 encoder (12-bit absolute, 0-4095/turn) ─┐
           │
           │ (Phase bit 4 により 2 種類に分岐)
           │
     bit4=0│                              bit4=1
           ├──────────┐              ┌────┤
     raw 0-4095        │              │   signed 16-bit 累積
                        ↓              ↓
               Present_Position = raw - Homing_Offset
                        │               (Wrap mod 65536 → サーボ内部で uint16 として報告)
                        ↓
                    ホスト Read
                        ↓
                    ホストが Goal_Position 書込 (同座標系)
                        ↓
               actual_target_raw = Goal + Homing_Offset
                        ↓
                    サーボが PID でその raw へ追従
                        ↓
                    Min/Max Angle Limit でクランプ
```

**Homing_Offset は Read / Write 両方向に自動適用**。ホストが毎パケット offset 演算をする必要は無い。

### 2-2. LeRobot (host side)

EEPROM 書込み + ホスト側 2 段階処理。

#### 書込み (一度きり、calibration 時)

```python
# src/lerobot/motors/feetech/feetech.py (verbatim)
def _get_half_turn_homings(self, positions):
    """Present_Position = Actual_Position - Homing_Offset"""
    for motor, pos in positions.items():
        max_res = self.model_resolution_table[model] - 1  # STS3215: 4095
        half_turn_homings[motor] = pos - int(max_res / 2)  # pos - 2047
    return half_turn_homings

def set_half_turn_homings(self, motors=...):
    self.reset_calibration(motor_names)  # Homing_Offset=0, Min=0, Max=4095
    actual_positions = self.sync_read("Present_Position", motor_names, normalize=False)
    homing_offsets = self._get_half_turn_homings(actual_positions)
    for motor, offset in homing_offsets.items():
        self.write("Homing_Offset", motor, offset)
```

#### teleop (毎サイクル、ホストで正規化)

```python
# src/lerobot/motors/motors_bus.py (verbatim)
def _normalize(self, ids_values):
    for id_, val in ids_values.items():
        min_ = self.calibration[motor].range_min
        max_ = self.calibration[motor].range_max
        bounded_val = min(max_, max(min_, val))   # ★ clamp
        if norm_mode is MotorNormMode.RANGE_M100_100:
            norm = (((bounded_val - min_) / (max_ - min_)) * 200) - 100
            normalized_values[id_] = -norm if drive_mode else norm
```

LeRobot は **teleop / imitation learning 用に値を ±100% 等に正規化** してから AI モデルや Follower へ渡す。Leader と Follower の Min/Max が違っても、正規化値の共通座標で 1:1 対応 + clamp による境界安全が保証される。

### 2-3. SO101_leader_follower FW (pass-through)

```cpp
// Leader side - src/main.cpp:339-361
int pos = scs_parse_position(resp, resp_len);  // サーボが返す Present (offset 適用済)
if (pos >= 0) {
    leader_positions[i] = (uint16_t)pos;       // そのまま格納
}
// ... ESP-NOW broadcast ...
```

```cpp
// Follower side - src/main.cpp:254-256
int pkt_len = scs_build_sync_write_pos_full(
    tx_buf, pkt->ids, pkt->pos,   // ← Leader からの値をそのまま Goal に書込
    axes, 0, 0);
uart_write_bytes(UART_NUM, (const char *)tx_buf, pkt_len);
```

**Homing_Offset も Min/Max も normalize も意識しない**。両サーボの EEPROM 設定を信頼した **薄い pass-through**。

この設計の前提:
- 両腕で calibration が同一物理姿勢で実施されている (両者の Present=2047 が同じ物理姿勢を指す)
- サーボ側の Min/Max Limit で Goal クランプされる (hardware 境界は firmware が守る)

メリット: コードが極小、ESP-NOW パケットが軽い (`pos_packet_t` は 24B)、サーボ交換時は EEPROM 再書込みのみで FW 変更不要。

デメリット: EEPROM 設定ミスや calibration ズレで即破綻 (今回の 2 大バグがまさにこれ)。

## 3. 正しい Calibration 手順 (LeRobot 互換)

[so101_calibrate.py](../python/so101_calibrate.py) に実装済。Waveshare USB-TTL をサーボバスに直結した状態で:

```bash
# 全軸リセット + Phase bit 4 = 0 + Operating_Mode = 0
python so101_calibrate.py --port COMxx --reset

# インタラクティブ calibration
python so101_calibrate.py --port COMxx --save leader_calibration.json
```

`calibrate()` フロー内部:

```
Step 1: reset_calibration(sid) を全軸実行
  ├ disable_torque()
  ├ configure_phase(sid)                       ← Phase bit 4 = 0 強制 (LeRobot 相当)
  ├ write_byte(ADDR_OPERATING_MODE, 0)         ← Position mode 強制
  ├ write_sword(ADDR_HOMING_OFFSET, 0)
  ├ write_word(ADDR_MIN_POSITION, 0)
  └ write_word(ADDR_MAX_POSITION, 4095)

Step 2: Homing_Offset を書込み
  ├ "Move ALL joints to MECHANICAL MIDDLE" 表示 (基本姿勢画像参照)
  │  ⚠️ 視覚的中央ではなく両端ストッパの中点！
  ├ ユーザー ENTER 待ち
  ├ 各軸の Present_Position を read (retry 3回、out-of-range 弾き)
  └ 各軸に Offset = pos - 2047 を書込み

Step 3: 可動域記録
  ├ "Move each joint through its FULL range" 表示
  ├ ENTER で記録開始、ユーザーが全可動域をスイープ
  ├ 6 軸の Present_Position を live sampling
  │  ⚠️ [0, 4095] 外の transient 値は除外 (wrap 等の誤値を防止)
  └ ENTER で停止

Step 4: EEPROM へ確定書込み
  ├ 各軸に Homing_Offset, Min_Position_Limit, Max_Position_Limit 書込み
  ├ Lock = 1 で EEPROM 再ロック
  └ JSON ファイル保存 (per-device backup)
```

### 我々のスクリプト固有の追加機能

LeRobot の `feetech.py` に無い / 我々が追加した防御:

| 機能 | 目的 |
|------|------|
| `configure_phase()` 呼び出しを `reset_calibration()` 内に統合 | `--reset` だけで Phase bit 4 と Operating_Mode と Min/Max と Offset を一括正規化 |
| `--configure` フラグ | Phase bit 4 だけ clear (calibration 保持) |
| Step 2 の retry 3 回 + out-of-range 弾き | Phase 未調整サーボが transient で sign-mag 値を返しても Offset が狂わない |
| Step 3 の sweep filter `0 <= pos <= 4095` | wrap transient 値を Min/Max に焼き付けない |
| `read_all_calibration()` の Phase 列 + 警告フラグ (`***PHASE_BIT4_SET`, `***NOT_POSITION_MODE`, `***MULTI_TURN`) | EEPROM 診断で問題個体を即発見 |
| 確認 prompt の default Y (`[Y/n]`) | 書込み確定時の ENTER 一発 |

## 4. Teleop 中の Present_Position 流れ (calibration 後)

```
【中点姿勢】
  Leader raw=2200, Offset=+153  → Present=2047 ── ESP-NOW ──→ Goal=2047 → raw_target=2047+(-747)=1300 → Follower raw=1300 (中点)
                                                                                        │
  両腕とも物理中点で Present=2047 (calibration のお陰)                                    │
                                                                                        ↓
                                                                                 Follower Offset=-747

【+10°】
  Leader raw=2314 (中点+114) → Present=2161 ── ESP-NOW ──→ Goal=2161 → raw_target=2161+(-747)=1414 → Follower raw=1414 (+10°)

【-10°】
  Leader raw=2086 → Present=1933 → Goal=1933 → raw_target=1186 → Follower raw=1186 (-10°)
```

Leader と Follower の raw 値は **個体差の分だけ常にオフセット** しているが、**Present 座標では共通**。FW は Present をそのまま Forward するだけで物理姿勢が 1:1 追従。

## 5. 実測データ (本セッション calibration 後)

| ID | Name | Leader Offset | Follower Offset | ギア比 | Phase register |
|----|------|--------------|-----------------|-------|---------------|
| 1 | shoulder_pan | +152 | -353 | Leader 1/191 / Follower 1/345 | Leader 0x4C / Follower 0x0C |
| 2 | shoulder_lift | -1301 | +1642 | 両方 1/345 | 両方 0x0C |
| 3 | elbow_flex | -844 | -1371 | Leader 1/191 / Follower 1/345 | Leader 0x4C / Follower 0x0C |
| 4 | wrist_flex | -1154 | +889 | Leader 1/147 / Follower 1/345 | Leader 0x4C / Follower 0x0C |
| 5 | wrist_roll | +457 | +2009 | Leader 1/147 / Follower 1/345 | Leader 0x4C / Follower 0x0C |
| 6 | gripper | +475 | -1572 | Leader 1/147 / Follower 1/345 | Leader 0x4C / Follower 0x0C |

Phase register 0x4C (Leader) = 0b01001100, 0x0C (Follower) = 0b00001100 — **bit 4 が両者とも 0** に揃っている。

Offset 差 (Leader-Follower) が 2000 付近の軸 (ID=4, 6) は **ホーン 180° 取付違い** を示す (正常、サーボ内部で吸収)。

### Range 完全一致

| ID | Leader Range | Follower Range | 差 |
|----|--------------|----------------|---|
| 1 shoulder_pan | 2722 | 2724 | 2 (0.07%) |
| 2 shoulder_lift | 2409 | 2413 | 4 (0.17%) |
| 3 elbow_flex | **2239** | **2237** | 2 (0.09%) ← 前回 1876 差 → 2 差に改善 |
| 4 wrist_flex | 2345 | 2346 | 1 (0.04%) |
| 5 wrist_roll | 4095 | 4095 | 0 |
| 6 gripper | 1280 | 1452 | 172 (13%) |

## 6. 今回のバグの再発防止チェックリスト

### キャリブレーション実施時

1. ✅ `so101_calibrate.py --reset` で Phase bit 4 / Operating_Mode / Min/Max / Offset を一括クリーン
2. ✅ Step 2 では **mechanical middle** (両端ストッパの真ん中) に配置 — [image/s0101_calibration_position.png](../image/s0101_calibration_position.png) 参照
3. ✅ Leader / Follower を同一物理姿勢に揃えて Step 2 実行
4. ✅ Step 3 sweep で全軸を物理限界まで動かす (elbow / wrist / gripper 特に)
5. ✅ 完了後 `--read-only` で Phase = 0x0C か 0x4C (bit 4 = 0) を確認、Min/Max/Offset が妥当範囲であることを目視

### トラブル時の診断

- **Follower が特定軸で突然反転** → 該当軸の Leader Offset と Follower Offset の差が極端に大きい (>2500) + calibration 姿勢が両腕で違っていた可能性 → 両腕同一姿勢で再 calibration
- **特定軸の Range が Max 近くで頭打ち** → 該当軸の Phase register 確認 (bit 4 = 1 なら wrap で偽の Max を記録)、または Step 2 の配置が機械中点から外れていた可能性
- **ポジション値が 32000 付近で返る** → Phase bit 4 = 1 状態 (sign-magnitude) 。`--configure` で修正
- **ポジション値が数千単位で跳ねる** → Operating_Mode が Step Servo (3) の可能性。`--reset` で Position mode へ戻す

## 7. 参照実装

- LeRobot feetech driver: https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/feetech/feetech.py
- LeRobot motors bus base class: https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/motors_bus.py
- LeRobot STS3215 register table: https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/feetech/tables.py
- LeRobot SO-101 follower calibration flow: https://github.com/huggingface/lerobot/blob/main/src/lerobot/robots/so_follower/so_follower.py
- 本 repo の calibration スクリプト: [python/so101_calibrate.py](../python/so101_calibrate.py)
- 本 repo の診断モニター: [python/so101_monitor.py](../python/so101_monitor.py)
- 本 repo の teleop FW: [src/main.cpp](../src/main.cpp)

## 8. 本セッションの関連 commit

- `e0636d5` — so101: fix multi-turn runaway via Phase bit4 + mechanical-middle calibration
- `b3f9be8` — so101: add calibration base-pose image + README guidance to all 3 FW
