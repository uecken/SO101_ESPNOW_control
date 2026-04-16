#!/usr/bin/env python3
"""
SO-101 Transparent Bridge Leader-Follower Test

PC reads Leader arm positions directly via USB-UART, then sends
SYNC_WRITE to Follower arm via rs485_espnow_bridge (USB_THROUGH + UART_THROUGH).

Architecture:
  PC --USB-UART(1Mbaud)--> Leader Arm (SCS READ)
  PC --USB(115200)-------> bridge_L (USB_THROUGH m2) ==ESP-NOW==> bridge_R (UART_THROUGH m1) --Grove--> Waveshare --> Follower Arm

Usage:
  python so101_bridge_test.py --leader COM25 --bridge COM8
  python so101_bridge_test.py --leader COM25 --bridge COM8 --duration 30 --hz 50
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


def setup_bridge(port):
    """Setup bridge_L as USB_THROUGH and start."""
    s = serial.Serial(port, 115200, timeout=1.0)
    s.dtr = False
    s.rts = False
    time.sleep(0.5)
    s.reset_input_buffer()
    # Exit USB_THROUGH if active
    s.write(b"\x18\x18\x18")
    s.flush()
    time.sleep(0.3)
    s.read(s.in_waiting)
    # Stop, set USB_THROUGH, start
    s.write(b"x\n")
    s.flush()
    time.sleep(0.2)
    s.read(s.in_waiting)
    s.write(b"m2\n")
    s.flush()
    time.sleep(0.2)
    s.read(s.in_waiting)
    s.write(b"s\n")
    s.flush()
    time.sleep(1.5)
    s.read(s.in_waiting)
    return s


def main():
    parser = argparse.ArgumentParser(
        description="SO-101 transparent bridge Leader-Follower test")
    parser.add_argument("--leader", type=str, default="COM25",
                        help="Leader USB-UART port (default: COM25)")
    parser.add_argument("--bridge", type=str, default="COM8",
                        help="bridge_L USB port (default: COM8)")
    parser.add_argument("--baud", type=int, default=1000000,
                        help="Leader servo baud (default: 1000000)")
    parser.add_argument("--duration", type=int, default=15,
                        help="Test duration seconds (default: 15)")
    parser.add_argument("--hz", type=int, default=50,
                        help="Target control rate Hz (default: 50)")
    args = parser.parse_args()

    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    interval = 1.0 / args.hz

    # Open Leader direct
    leader = serial.Serial(args.leader, args.baud, timeout=0.05)
    time.sleep(0.1)

    # Setup bridge_L
    print("Setting up bridge_L (%s) as USB_THROUGH..." % args.bridge)
    bridge = setup_bridge(args.bridge)

    # Torque ON on Follower
    print("Torque ON Follower (via bridge)...")
    for sid in SERVO_IDS:
        bridge.reset_input_buffer()
        bridge.write(scs_torque(sid, True))
        bridge.flush()
        time.sleep(0.05)

    print()
    print("Running at ~%dHz for %ds. Move the Leader arm!" % (args.hz, args.duration))
    print()

    end_time = time.time() + args.duration
    start_time = time.time()
    cycles = 0
    read_ok = 0
    write_ok = 0
    last_print = time.time()

    try:
        while time.time() < end_time:
            t0 = time.time()

            # Read all 6 axes from Leader
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
                read_ok += 1
                ids = sorted(positions.keys())
                pos_list = [positions[i] for i in ids]

                pkt = scs_sync_write_pos(ids, pos_list)
                bridge.write(pkt)
                bridge.flush()
                write_ok += 1

            cycles += 1

            # Print every second
            now = time.time()
            if now - last_print >= 1.0:
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
        bridge.reset_input_buffer()
        bridge.write(scs_torque(sid, False))
        bridge.flush()
        time.sleep(0.05)

    # Exit USB_THROUGH
    bridge.write(b"\x18\x18\x18")
    bridge.flush()
    time.sleep(0.3)
    bridge.read(bridge.in_waiting)

    leader.close()
    bridge.close()

    duration_actual = time.time() - start_time
    rate = write_ok / duration_actual if duration_actual > 0 else 0
    print()
    print("=" * 60)
    print("Result:")
    print("  Duration  : %.1f s" % duration_actual)
    print("  Cycles    : %d" % cycles)
    print("  READ ok   : %d/%d (%.1f%%)" % (read_ok, cycles, read_ok / cycles * 100 if cycles else 0))
    print("  WRITE ok  : %d" % write_ok)
    print("  Rate      : %.1f Hz" % rate)
    print("=" * 60)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
