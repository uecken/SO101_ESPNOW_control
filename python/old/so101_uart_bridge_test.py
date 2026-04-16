#!/usr/bin/env python3
"""
SO-101 True UART Transparent Bridge Leader-Follower Test

PC reads Leader arm positions directly via USB-UART, then sends
SYNC_WRITE to Follower arm via true UART transparent bridge:
  COM26 (USB-RS485) -> ISO485 -> bridge_L (UART_THROUGH m1) -> ESP-NOW -> bridge_R (UART_THROUGH m1) -> Follower

Both bridges use UART_THROUGH (m1) mode. No USB_THROUGH.

Architecture:
  PC --COM25(1Mbaud)----> Leader Arm (SCS READ, direct USB-UART)
  PC --COM26(500kbaud)--> USB-RS485(CH340) --RS485--> ISO485 --UART--> bridge_L(m1,500k) ==ESP-NOW==> bridge_R(m1,1M) --UART--> Waveshare --> Follower Arm

Note: USB-RS485 (CH340) is limited to 500kbaud max (1Mbaud causes bit errors).
      bridge_L=500kbaud, bridge_R=1Mbaud (asymmetric baud configuration).

Usage:
  python so101_uart_bridge_test.py
  python so101_uart_bridge_test.py --leader COM25 --bridge-usb COM26 --bridge-l COM8 --bridge-r COM14
  python so101_uart_bridge_test.py --duration 30 --hz 50
  python so101_uart_bridge_test.py --duration 0        # run until Ctrl+C
  python so101_uart_bridge_test.py --ping-only         # PING/READ test only, no tracking
  python so101_uart_bridge_test.py --skip-setup         # skip bridge setup (already configured)
"""

import sys
import time
import argparse
import serial

SERVO_IDS = [1, 2, 3, 4, 5, 6]
SERVO_NAMES = {
    1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex",
    4: "wrist_flex", 5: "wrist_roll", 6: "gripper",
}


# ============================================================================
# SCS Protocol
# ============================================================================

def scs_packet(sid, inst, params):
    length = len(params) + 2
    chk = (~(sid + length + inst + sum(params))) & 0xFF
    return bytes([0xFF, 0xFF, sid, length, inst]) + bytes(params) + bytes([chk])


def scs_read_pos(sid):
    return scs_packet(sid, 0x02, [0x38, 2])


def scs_sync_write_pos(ids, positions):
    buf = [0xFF, 0xFF, 0xFE, 0, 0x83, 0x2A, 2]
    for i in range(len(ids)):
        buf.append(ids[i])
        buf.append(positions[i] & 0xFF)
        buf.append((positions[i] >> 8) & 0xFF)
    buf[3] = len(buf) - 3
    buf.append((~sum(buf[2:])) & 0xFF)
    return bytes(buf)


def scs_torque(sid, on):
    return scs_packet(sid, 0x03, [0x28, 1 if on else 0])


def parse_pos(resp):
    idx = resp.find(b"\xff\xff")
    if idx < 0 or idx + 7 > len(resp):
        return -1
    if resp[idx + 4] != 0:
        return -1
    return resp[idx + 5] | (resp[idx + 6] << 8)


def hex_str(data):
    return " ".join(f"{b:02X}" for b in data)


# ============================================================================
# Bridge setup
# ============================================================================

def setup_bridge(port, baud, name):
    """Configure bridge via USB serial (keep open to avoid DTR reset)."""
    s = serial.Serial(port, 115200, timeout=1.0)
    s.dtr = False
    s.rts = False
    time.sleep(1.5)
    s.reset_input_buffer()
    s.write(b"x\n"); s.flush(); time.sleep(0.2); s.read(s.in_waiting)
    s.write(b"r\n"); s.flush(); time.sleep(0.2); s.read(s.in_waiting)
    s.write(b"m1\n"); s.flush(); time.sleep(0.2); s.read(s.in_waiting)
    s.write(("b%d\n" % baud).encode()); s.flush(); time.sleep(0.2); s.read(s.in_waiting)
    s.write(b"s\n"); s.flush(); time.sleep(0.5); s.read(s.in_waiting)
    # Verify
    s.write(b"?\n"); s.flush(); time.sleep(0.5)
    r = s.read(s.in_waiting).decode(errors="replace")
    for line in r.split("\n"):
        if "STATUS" in line:
            print("  %s: %s" % (name, line.strip()[:100]))
            break
    return s  # keep open


# ============================================================================
# PING / READ test
# ============================================================================

def test_ping_read(bridge_usb_port, bridge_baud):
    """Test SCS PING and READ through the transparent bridge."""
    s = serial.Serial(bridge_usb_port, bridge_baud, timeout=0.3)
    time.sleep(0.1)
    s.reset_input_buffer()

    print()
    print("PING (via bridge):")
    ping_ok = 0
    for sid in SERVO_IDS:
        s.reset_input_buffer()
        s.write(scs_packet(sid, 0x01, []))
        s.flush()
        time.sleep(0.2)
        resp = s.read(s.in_waiting or 0)
        idx = resp.find(b"\xff\xff")
        if idx >= 0 and idx + 5 <= len(resp):
            rid = resp[idx + 2]
            err = resp[idx + 4]
            ok = (rid == sid and err == 0)
            if ok:
                ping_ok += 1
            print("  ID=%d: %s" % (sid, "OK" if ok else "BAD (id=%d err=%d)" % (rid, err)))
        else:
            print("  ID=%d: NO RESPONSE (%dB: %s)" % (sid, len(resp), hex_str(resp[:16])))

    print()
    print("READ Present_Position (via bridge):")
    read_ok = 0
    for sid in SERVO_IDS:
        s.reset_input_buffer()
        s.write(scs_read_pos(sid))
        s.flush()
        time.sleep(0.2)
        resp = s.read(s.in_waiting or 0)
        pos = parse_pos(resp)
        if pos >= 0:
            read_ok += 1
        name = SERVO_NAMES.get(sid, "?")
        print("  ID=%d (%s): pos=%d" % (sid, name, pos))

    s.close()
    print()
    print("PING: %d/6, READ: %d/6" % (ping_ok, read_ok))
    return ping_ok, read_ok


# ============================================================================
# Leader-Follower tracking test
# ============================================================================

def read_follower_positions(bridge_ser):
    """Read actual positions from Follower servos via bridge (bidirectional)."""
    positions = {}
    for sid in SERVO_IDS:
        bridge_ser.reset_input_buffer()
        bridge_ser.write(scs_read_pos(sid))
        bridge_ser.flush()
        time.sleep(0.01)  # wait for round trip via bridge
        # Read with longer timeout for bridge RTT
        deadline = time.time() + 0.1
        resp = b""
        while time.time() < deadline:
            if bridge_ser.in_waiting:
                resp += bridge_ser.read(bridge_ser.in_waiting)
                if len(resp) >= 8 and b"\xff\xff" in resp:
                    break
            time.sleep(0.002)
        pos = parse_pos(resp)
        if pos >= 0:
            positions[sid] = pos
    return positions


def test_tracking(leader_port, leader_baud, bridge_usb_port, bridge_baud,
                  duration, target_hz, read_hz=1.0):
    """Read Leader positions and send SYNC_WRITE to Follower via bridge.
    Periodically reads Follower actual positions for verification."""
    interval = 1.0 / target_hz
    read_interval = 1.0 / read_hz if read_hz > 0 else float("inf")

    leader = serial.Serial(leader_port, leader_baud, timeout=0.05)
    time.sleep(0.1)

    bridge_in = serial.Serial(bridge_usb_port, bridge_baud, timeout=0.1)
    time.sleep(0.1)

    # Torque ON
    print("Torque ON Follower (via bridge)...")
    for sid in SERVO_IDS:
        bridge_in.reset_input_buffer()
        bridge_in.write(scs_torque(sid, True))
        bridge_in.flush()
        time.sleep(0.05)

    dur_str = "%ds" % duration if duration > 0 else "unlimited (Ctrl+C)"
    print()
    print("Running: WRITE ~%dHz, READ ~%.1fHz, duration %s" %
          (target_hz, read_hz, dur_str))
    print("Move the Leader arm!")
    print()

    unlimited = (duration <= 0)
    end_time = time.time() + duration if not unlimited else float("inf")
    start_time = time.time()
    cycles = 0
    leader_read_ok = 0
    write_ok = 0
    follower_read_count = 0
    follower_read_ok = 0
    last_print = time.time()
    last_follower_read = 0
    last_leader_pos = {}
    last_follower_pos = {}

    try:
        while time.time() < end_time:
            t0 = time.time()

            # Read all 6 axes from Leader (direct)
            positions = {}
            for sid in SERVO_IDS:
                leader.reset_input_buffer()
                leader.write(scs_read_pos(sid))
                leader.flush()
                time.sleep(0.003)
                resp = leader.read(leader.in_waiting or 0)
                pos = parse_pos(resp)
                if pos >= 0:
                    positions[sid] = pos

            if len(positions) >= 6:
                leader_read_ok += 1
                ids = sorted(positions.keys())
                pos_list = [positions[i] for i in ids]

                pkt = scs_sync_write_pos(ids, pos_list)
                bridge_in.write(pkt)
                bridge_in.flush()
                write_ok += 1
                last_leader_pos = dict(positions)

            cycles += 1

            now = time.time()

            # Periodically read Follower actual positions via bridge
            if now - last_follower_read >= read_interval:
                last_follower_read = now
                follower_read_count += 1
                fpos = read_follower_positions(bridge_in)
                if len(fpos) >= 6:
                    follower_read_ok += 1
                    last_follower_pos = dict(fpos)

                # Print Leader POS + Follower ACT
                elapsed = now - start_time
                hz = cycles / elapsed if elapsed > 0 else 0
                if last_leader_pos:
                    lstr = " ".join("[%d]=%d" % (i, last_leader_pos[i])
                                   for i in sorted(last_leader_pos.keys()))
                    print("  POS: %s  (%.0fHz, wr=%d)" % (lstr, hz, write_ok))
                if last_follower_pos:
                    astr = " ".join("[%d]=%d" % (i, last_follower_pos[i])
                                   for i in sorted(last_follower_pos.keys()))
                    print("  ACT: %s  (fread=%d/%d)" %
                          (astr, follower_read_ok, follower_read_count))
                elif follower_read_count > 0:
                    print("  ACT: READ FAILED (fread=%d/%d)" %
                          (follower_read_ok, follower_read_count))
            elif now - last_print >= 1.0:
                # Print POS only if no follower read this second
                last_print = now
                elapsed = now - start_time
                hz = cycles / elapsed if elapsed > 0 else 0
                if positions:
                    pos_str = " ".join(
                        "[%d]=%d" % (i, positions[i]) for i in sorted(positions.keys()))
                    print("  [%4d] %s  (%.0fHz)" % (write_ok, pos_str, hz))

            elapsed = time.time() - t0
            if elapsed < interval:
                time.sleep(interval - elapsed)

    except KeyboardInterrupt:
        print("\nInterrupted.")

    # Torque OFF
    print()
    print("Torque OFF Follower...")
    for sid in SERVO_IDS:
        bridge_in.reset_input_buffer()
        bridge_in.write(scs_torque(sid, False))
        bridge_in.flush()
        time.sleep(0.05)

    leader.close()
    bridge_in.close()

    duration_actual = time.time() - start_time
    rate = write_ok / duration_actual if duration_actual > 0 else 0

    print()
    print("=" * 60)
    print("Result:")
    print("  Duration       : %.1f s" % duration_actual)
    print("  Cycles         : %d" % cycles)
    print("  Leader READ ok : %d/%d (%.1f%%)" %
          (leader_read_ok, cycles, leader_read_ok / cycles * 100 if cycles else 0))
    print("  WRITE ok       : %d" % write_ok)
    print("  Write Rate     : %.1f Hz" % rate)
    print("  Follower READ  : %d/%d (%.1fHz)" %
          (follower_read_ok, follower_read_count,
           follower_read_count / duration_actual if duration_actual > 0 else 0))
    print()
    print("  Leader    : %s @ %d baud (direct USB-UART)" % (leader_port, leader_baud))
    print("  Follower  : %s @ %d baud -> USB-RS485 -> ISO485 -> bridge_L(%dk) -> ESP-NOW -> bridge_R(1M)" %
          (bridge_usb_port, bridge_baud, bridge_baud // 1000))
    print("=" * 60)

    return write_ok, cycles, rate


# ============================================================================
# Main
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="SO-101 true UART transparent bridge Leader-Follower test",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--leader", type=str, default="COM25",
                        help="Leader USB-UART port (default: COM25)")
    parser.add_argument("--leader-baud", type=int, default=1000000,
                        help="Leader servo baud (default: 1000000)")
    parser.add_argument("--bridge-usb", type=str, default="COM26",
                        help="USB-RS485 port to bridge_L (default: COM26)")
    parser.add_argument("--bridge-baud", type=int, default=500000,
                        help="Bridge_L / USB-RS485 baud (default: 500000)")
    parser.add_argument("--bridge-l", type=str, default="COM8",
                        help="bridge_L USB port for setup (default: COM8)")
    parser.add_argument("--bridge-r", type=str, default="COM14",
                        help="bridge_R USB port for setup (default: COM14)")
    parser.add_argument("--servo-baud", type=int, default=1000000,
                        help="Follower servo baud = bridge_R baud (default: 1000000)")
    parser.add_argument("--duration", type=int, default=15,
                        help="Test duration seconds (default: 15, 0=unlimited/Ctrl+C)")
    parser.add_argument("--hz", type=int, default=30,
                        help="Target WRITE rate Hz (default: 30)")
    parser.add_argument("--read-hz", type=float, default=1.0,
                        help="Follower READ rate Hz (default: 1.0, 0=disable)")
    parser.add_argument("--ping-only", action="store_true",
                        help="PING/READ test only, no tracking")
    parser.add_argument("--skip-setup", action="store_true",
                        help="Skip bridge setup (already configured)")
    args = parser.parse_args()

    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    print("=" * 60)
    print("SO-101 True UART Transparent Bridge Test")
    print("=" * 60)
    print()
    print("Configuration:")
    print("  Leader     : %s @ %d baud (direct)" % (args.leader, args.leader_baud))
    print("  USB-RS485  : %s @ %d baud (CH340)" % (args.bridge_usb, args.bridge_baud))
    print("  bridge_L   : %s (UART_THROUGH m1, %d baud)" % (args.bridge_l, args.bridge_baud))
    print("  bridge_R   : %s (UART_THROUGH m1, %d baud)" % (args.bridge_r, args.servo_baud))
    print()

    # Setup bridges
    bl_handle = None
    br_handle = None
    if not args.skip_setup:
        print("Setting up bridges...")
        br_handle = setup_bridge(args.bridge_r, args.servo_baud, "bridge_R")
        bl_handle = setup_bridge(args.bridge_l, args.bridge_baud, "bridge_L")
        print()

    try:
        if args.ping_only:
            test_ping_read(args.bridge_usb, args.bridge_baud)
        else:
            # PING first
            ping_ok, read_ok = test_ping_read(args.bridge_usb, args.bridge_baud)
            if ping_ok < 6:
                print("WARNING: Not all servos responded. Continue anyway.")
            print()

            # Tracking
            test_tracking(args.leader, args.leader_baud,
                          args.bridge_usb, args.bridge_baud,
                          args.duration, args.hz, args.read_hz)
    finally:
        if bl_handle:
            bl_handle.close()
        if br_handle:
            br_handle.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
