#!/usr/bin/env python3
"""
SO-101 STS3215 Calibration Script (LeRobot compatible procedure)

Follows the exact same calibration flow as huggingface/lerobot:
  1. Torque OFF, Lock=0, Operating_Mode=POSITION
  2. Reset: Min=0, Max=4095, Homing_Offset=0
  3. User moves all joints to middle -> compute & write Homing_Offset
  4. User sweeps joints through full range -> record min/max
     (wrist_roll/ID=5 is auto-set to 0-4095)
  5. Write Min_Position_Limit and Max_Position_Limit to servo EEPROM
  6. Save calibration JSON

Usage:
  python so101_calibrate.py --port COM24                  # full calibration
  python so101_calibrate.py --port COM24 --read-only      # show current values
  python so101_calibrate.py --port COM24 --reset          # reset to full range
"""

import sys
import os
import json
import time
import argparse
import threading
import serial

BAUD = 1000000
SERVO_IDS = [1, 2, 3, 4, 5, 6]
SERVO_NAMES = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}
WRIST_ROLL_ID = 5  # full rotation, skip range recording

# STS3215 register addresses
ADDR_MIN_POSITION    = 0x09  # 2B EEPROM
ADDR_MAX_POSITION    = 0x0B  # 2B EEPROM
ADDR_PHASE           = 0x12  # 1B EEPROM (bit4 = angle feedback mode; LeRobot clears it)
ADDR_MAX_TORQUE      = 0x10  # 2B EEPROM
ADDR_HOMING_OFFSET   = 0x1F  # 2B EEPROM (signed, two's complement)
ADDR_OPERATING_MODE  = 0x21  # 1B
ADDR_TORQUE_ENABLE   = 0x28  # 1B
ADDR_ACCELERATION    = 0x29  # 1B
ADDR_LOCK            = 0x37  # 1B
ADDR_PRESENT_POSITION = 0x38 # 2B
ADDR_MAX_ACCELERATION = 0x55 # 1B EEPROM


# ============================================================================
# SCS Protocol helpers
# ============================================================================

def scs_packet(sid, inst, params):
    length = len(params) + 2
    chk = (~(sid + length + inst + sum(params))) & 0xFF
    return bytes([0xFF, 0xFF, sid, length, inst]) + bytes(params) + bytes([chk])


def send_recv(ser, pkt, timeout=0.05):
    ser.reset_input_buffer()
    ser.write(pkt)
    ser.flush()
    time.sleep(timeout)
    return ser.read(ser.in_waiting or 0)


def parse_params(resp):
    idx = resp.find(b"\xff\xff")
    if idx < 0 or idx + 4 >= len(resp):
        return None
    slen = resp[idx + 3]
    err = resp[idx + 4]
    params = list(resp[idx + 5 : idx + 3 + slen])
    return params if err == 0 else None


def read_byte(ser, sid, addr):
    p = parse_params(send_recv(ser, scs_packet(sid, 0x02, [addr, 1])))
    return p[0] if p and len(p) >= 1 else None


def read_word(ser, sid, addr):
    p = parse_params(send_recv(ser, scs_packet(sid, 0x02, [addr, 2])))
    return (p[0] | (p[1] << 8)) if p and len(p) >= 2 else None


def read_sword(ser, sid, addr):
    v = read_word(ser, sid, addr)
    if v is None:
        return None
    return v if v < 0x8000 else v - 0x10000


def write_byte(ser, sid, addr, val):
    send_recv(ser, scs_packet(sid, 0x03, [addr, val & 0xFF]))


def write_word(ser, sid, addr, val):
    send_recv(ser, scs_packet(sid, 0x03, [addr, val & 0xFF, (val >> 8) & 0xFF]))


def write_sword(ser, sid, addr, val):
    if val < 0:
        val = val + 0x10000
    write_word(ser, sid, addr, val)


# ============================================================================
# Servo control
# ============================================================================

def disable_torque(ser, sid):
    write_byte(ser, sid, ADDR_TORQUE_ENABLE, 0)
    time.sleep(0.01)
    write_byte(ser, sid, ADDR_LOCK, 0)
    time.sleep(0.01)


def configure_phase(ser, sid):
    """Clear bit 4 (0x10) of the Phase register (0x12).
    Matches LeRobot's configure_motors(). Without this, STS3215 units shipped
    with Phase bit 4 = 1 report Present_Position in sign-magnitude encoding
    (bit15 = sign) when the shaft crosses raw=0, producing values like 32826
    (= -58 sign-mag) instead of a clean 0-4095 wrap. Clearing bit 4 forces all
    units into the [0, 4095] wrap behavior so that Present_Position is always
    an unsigned 12-bit value — making the teleop datapath consistent.
    """
    phase = read_byte(ser, sid, ADDR_PHASE)
    if phase is None:
        return None
    if phase & 0x10:
        write_byte(ser, sid, ADDR_PHASE, phase & ~0x10)
    return phase


def reset_calibration(ser, sid):
    """Reset to factory-like single-turn state.
    Writes Operating_Mode=0 (Position/Servo mode) explicitly to prevent the
    servo from staying in Step Servo mode (0x21=3) or Wheel mode (0x21=1).
    Also clears Phase register bit 4 via configure_phase() so Present_Position
    stays in [0, 4095] range instead of spilling into sign-magnitude negative.
    Matches the LeRobot / Feetech standard reset+configure sequence.
    """
    disable_torque(ser, sid)
    configure_phase(ser, sid)                      # Phase bit 4 -> 0 (consistent wrap)
    write_byte(ser, sid, ADDR_OPERATING_MODE, 0)   # Position (single-turn) mode
    write_sword(ser, sid, ADDR_HOMING_OFFSET, 0)
    write_word(ser, sid, ADDR_MIN_POSITION, 0)
    write_word(ser, sid, ADDR_MAX_POSITION, 4095)


# ============================================================================
# Read-only: show current calibration
# ============================================================================

def read_all_calibration(ser):
    print()
    mode_names = {0: "Pos", 1: "Wheel", 2: "PWM", 3: "Step*"}  # * = multi-turn risk
    fmt = "%3s  %-15s %4s %5s %6s %6s %6s %7s %5s %6s %6s %6s"
    print(fmt % ("ID", "Name", "Mode", "Phase", "Min", "Max", "Range", "Offset", "Lock", "Pos", "Accel", "MaxAcc"))
    print("-" * 92)
    for sid in SERVO_IDS:
        name = SERVO_NAMES.get(sid, "?")
        op_mode = read_byte(ser, sid, ADDR_OPERATING_MODE)
        phase = read_byte(ser, sid, ADDR_PHASE)
        mn = read_word(ser, sid, ADDR_MIN_POSITION)
        mx = read_word(ser, sid, ADDR_MAX_POSITION)
        offset = read_sword(ser, sid, ADDR_HOMING_OFFSET)
        lock = read_byte(ser, sid, ADDR_LOCK)
        pos = read_word(ser, sid, ADDR_PRESENT_POSITION)
        accel = read_byte(ser, sid, ADDR_ACCELERATION)
        max_accel = read_byte(ser, sid, ADDR_MAX_ACCELERATION)
        rng = (mx - mn) if mn is not None and mx is not None else -1
        flags = ""
        if 0 <= rng < 100: flags += " ***NARROW"
        if op_mode is not None and op_mode != 0: flags += " ***NOT_POSITION_MODE"
        if mn == 0 and mx == 0: flags += " ***MULTI_TURN"
        if phase is not None and (phase & 0x10): flags += " ***PHASE_BIT4_SET"  # sign-mag mode
        mode_str = mode_names.get(op_mode, "?") if op_mode is not None else "ERR"
        phase_str = ("0x%02X" % phase) if phase is not None else "ERR"
        print(("%3d  %-15s %4s %5s %6s %6s %6s %7s %5s %6s %6s %6s%s") % (
            sid, name,
            mode_str,
            phase_str,
            mn if mn is not None else "ERR",
            mx if mx is not None else "ERR",
            rng if rng >= 0 else "ERR",
            offset if offset is not None else "ERR",
            lock if lock is not None else "ERR",
            pos if pos is not None else "ERR",
            accel if accel is not None else "ERR",
            max_accel if max_accel is not None else "ERR",
            flags))
    print()


# ============================================================================
# Reset to full range
# ============================================================================

def reset_all(ser):
    print("Resetting all servos to single-turn Position mode (Phase bit4=0)...")
    for sid in SERVO_IDS:
        reset_calibration(ser, sid)
        name = SERVO_NAMES.get(sid, "?")
        print("  ID=%d (%s): Phase_bit4=0, OpMode=0(Pos), Min=0, Max=4095, Offset=0" % (sid, name))
    print("Done.\n")


def configure_phase_all(ser):
    """Clear Phase bit4 on all servos WITHOUT touching Min/Max/Offset.
    Use this to fix Phase without losing calibration data.
    """
    print("Configuring Phase register on all servos (clearing bit 4)...")
    for sid in SERVO_IDS:
        # Phase bit requires Torque OFF + Lock=0 before write (EEPROM).
        disable_torque(ser, sid)
        prev = configure_phase(ser, sid)
        name = SERVO_NAMES.get(sid, "?")
        if prev is None:
            print("  ID=%d (%s): read fail" % (sid, name))
        else:
            new = read_byte(ser, sid, ADDR_PHASE)
            changed = "CHANGED" if (prev & 0x10) and not (new & 0x10 if new is not None else True) else "unchanged"
            print("  ID=%d (%-15s) Phase: 0x%02X -> 0x%02X  [%s]" %
                  (sid, name, prev, new if new is not None else 0, changed))
        # Re-lock EEPROM (nice-to-have; does nothing functionally for Phase)
        write_byte(ser, sid, ADDR_LOCK, 1)
    print("Done.\n")


# ============================================================================
# Full calibration (LeRobot compatible)
# ============================================================================

def calibrate(ser, save_path=None):
    print("=" * 60)
    print("SO-101 Calibration (LeRobot compatible)")
    print("=" * 60)
    print()

    # --- Step 1: Reset all ---
    print("[Step 1/4] Reset & prepare")
    for sid in SERVO_IDS:
        reset_calibration(ser, sid)   # torque off, Phase bit4=0, Mode=0, Offset=0, Min=0, Max=4095
    print("  All servos: Torque OFF, Phase bit4=0, Mode=POSITION, Min=0, Max=4095, Offset=0")
    print()

    # --- Step 2: Homing offset ---
    print("[Step 2/4] Set homing offset")
    print("  Move ALL joints to the MIDDLE of their MECHANICAL range.")
    print("  The 'middle' means HALFWAY between each joint's two physical stops,")
    print("  NOT the pose that visually looks straight.")
    print()
    print("  Reference posture: see image/s0101_calibration_position.png")
    print("    (path: SO101_leader_follower/image/s0101_calibration_position.png)")
    print()
    print("  If placed off-center here, Homing_Offset will be wrong and")
    print("  Present_Position may wrap through 0/4095 during teleop (= sudden motion).")
    print()
    input("  Press ENTER when ready...")
    print()

    offsets = {}
    print("  %-15s %6s %7s" % ("Name", "Pos", "Offset"))
    print("  " + "-" * 32)
    for sid in SERVO_IDS:
        # Read up to 3 times to avoid a transient out-of-range reading being
        # captured as the middle-pose position and burned into a huge Offset.
        pos = None
        for _ in range(3):
            p = read_word(ser, sid, ADDR_PRESENT_POSITION)
            if p is not None and 0 <= p <= 4095:
                pos = p
                break
            time.sleep(0.02)
        if pos is None:
            print("  ERROR: Cannot read valid Pos for ID=%d (got out-of-range or no response)." % sid)
            print("         The servo may still be in a wrapped state from previous EEPROM values.")
            print("         Re-run with --reset first to fully clear the EEPROM, then retry calibration.")
            return
        offset = pos - 2047
        offsets[sid] = offset
        name = SERVO_NAMES.get(sid, "?")
        print("  %-15s %6d %+7d" % (name, pos, offset))

    print()
    print("  Writing homing offsets to EEPROM...")
    for sid in SERVO_IDS:
        write_sword(ser, sid, ADDR_HOMING_OFFSET, offsets[sid])
    print("  Done.")
    print()

    # --- Step 3: Record range of motion ---
    print("[Step 3/4] Record range of motion")
    print("  Move each joint (except wrist_roll) through its FULL range.")
    print("  wrist_roll (ID=5) = 0-4095 (full rotation, auto-set)")
    print()
    input("  Press ENTER to START recording...")
    print()
    print("  Recording... move all joints now! Press ENTER to stop.")
    print()

    # Initialize with current positions
    range_min = {}
    range_max = {}
    for sid in SERVO_IDS:
        pos = read_word(ser, sid, ADDR_PRESENT_POSITION)
        if pos is not None:
            range_min[sid] = pos
            range_max[sid] = pos
        else:
            range_min[sid] = 4095
            range_max[sid] = 0

    # Background thread waits for ENTER
    stop_event = threading.Event()

    def wait_enter():
        input()
        stop_event.set()

    t = threading.Thread(target=wait_enter, daemon=True)
    t.start()

    # Print header for live table
    header = "  %-15s | %6s | %6s | %6s" % ("NAME", "MIN", "POS", "MAX")
    separator = "  " + "-" * len(header)
    n_lines = len(SERVO_IDS) + 2  # header + separator + rows

    sample_count = 0
    while not stop_event.is_set():
        current_pos = {}
        for sid in SERVO_IDS:
            if sid == WRIST_ROLL_ID:
                current_pos[sid] = read_word(ser, sid, ADDR_PRESENT_POSITION) or 0
                continue
            pos = read_word(ser, sid, ADDR_PRESENT_POSITION)
            # Filter out-of-range transient reads (>4095 or <0).
            # These occur when Homing_Offset causes 16-bit overflow, or if a
            # servo briefly reports a multi-turn value. Writing such a value
            # as Max_Position_Limit would bake a corrupted range into EEPROM.
            if pos is not None and 0 <= pos <= 4095:
                current_pos[sid] = pos
                if pos < range_min[sid]:
                    range_min[sid] = pos
                if pos > range_max[sid]:
                    range_max[sid] = pos
            else:
                current_pos[sid] = -1
        sample_count += 1

        # Redraw table every 5 samples
        if sample_count % 5 == 0:
            # Move cursor up to overwrite
            if sample_count > 5:
                sys.stdout.write("\033[%dA" % n_lines)
            print(header)
            print(separator)
            for sid in SERVO_IDS:
                name = SERVO_NAMES.get(sid, "?")
                if sid == WRIST_ROLL_ID:
                    print("  %-15s | %6d | %6d | %6d  (auto)" %
                          (name, 0, current_pos[sid], 4095))
                else:
                    print("  %-15s | %6d | %6d | %6d" %
                          (name, range_min[sid], current_pos[sid], range_max[sid]))

        time.sleep(0.02)

    # wrist_roll: always full range
    range_min[WRIST_ROLL_ID] = 0
    range_max[WRIST_ROLL_ID] = 4095

    print()
    print("  Recording stopped. Final results:")
    print()
    print("  %3s  %-15s %6s %6s %6s" % ("ID", "Name", "Min", "Max", "Range"))
    print("  " + "-" * 48)
    any_narrow = False
    for sid in SERVO_IDS:
        name = SERVO_NAMES.get(sid, "?")
        rng = range_max[sid] - range_min[sid]
        flag = ""
        if sid != WRIST_ROLL_ID and rng < 100:
            flag = " *** TOO NARROW - move this joint more!"
            any_narrow = True
        print("  %3d  %-15s %6d %6d %6d%s" %
              (sid, name, range_min[sid], range_max[sid], rng, flag))

    if any_narrow:
        print()
        print("  WARNING: Some joints have very narrow range.")
        print("  Consider re-running calibration and moving those joints fully.")

    print()
    confirm = input("  Write these values to servo EEPROM? [Y/n]: ").strip().lower()
    if confirm not in ("", "y", "yes"):
        print("  Cancelled.")
        return

    # --- Step 4: Write to EEPROM ---
    print()
    print("[Step 4/4] Writing calibration to EEPROM...")
    calibration = {}
    for sid in SERVO_IDS:
        disable_torque(ser, sid)
        write_sword(ser, sid, ADDR_HOMING_OFFSET, offsets[sid])
        write_word(ser, sid, ADDR_MIN_POSITION, range_min[sid])
        write_word(ser, sid, ADDR_MAX_POSITION, range_max[sid])
        write_byte(ser, sid, ADDR_LOCK, 1)
        name = SERVO_NAMES.get(sid, "?")
        print("  ID=%d (%s): Offset=%+d, Min=%d, Max=%d" %
              (sid, name, offsets[sid], range_min[sid], range_max[sid]))
        calibration[name] = {
            "id": sid,
            "drive_mode": 0,
            "homing_offset": offsets[sid],
            "range_min": range_min[sid],
            "range_max": range_max[sid],
        }

    # Save JSON
    if save_path:
        dir_name = os.path.dirname(save_path)
        if dir_name:  # only mkdir if path has a directory component (e.g. "sub/cal.json")
            os.makedirs(dir_name, exist_ok=True)
        with open(save_path, "w") as f:
            json.dump(calibration, f, indent=2)
        print()
        print("  Calibration saved to: %s" % save_path)

    print()
    print("Calibration complete!")
    print()

    # Verify
    read_all_calibration(ser)


# ============================================================================
# Main
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="SO-101 STS3215 Calibration (LeRobot compatible)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python so101_calibrate.py --port COM24                   # full calibration
  python so101_calibrate.py --port COM24 --read-only       # show current values
  python so101_calibrate.py --port COM24 --reset           # reset to 0-4095 + Phase bit4=0
  python so101_calibrate.py --port COM24 --configure       # clear Phase bit4 only (keep calibration)
  python so101_calibrate.py --port COM24 --save cal.json   # calibrate + save JSON
""")
    parser.add_argument("--port", type=str, required=True,
                        help="COM port (e.g. COM24)")
    parser.add_argument("--baud", type=int, default=BAUD,
                        help="Baud rate (default: 1000000)")
    parser.add_argument("--read-only", action="store_true",
                        help="Just show current calibration values")
    parser.add_argument("--reset", action="store_true",
                        help="Reset all servos to full range (0-4095)")
    parser.add_argument("--configure", action="store_true",
                        help="Clear Phase register bit 4 only (preserves Min/Max/Offset)")
    parser.add_argument("--save", type=str, default=None,
                        help="Save calibration JSON to this path")
    args = parser.parse_args()

    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    time.sleep(0.1)
    ser.reset_input_buffer()

    try:
        if args.read_only:
            read_all_calibration(ser)
        elif args.reset:
            reset_all(ser)
            read_all_calibration(ser)
        elif args.configure:
            configure_phase_all(ser)
            read_all_calibration(ser)
        else:
            calibrate(ser, save_path=args.save)
    finally:
        ser.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
