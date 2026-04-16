#!/usr/bin/env python3
"""
SO-101 Leader-Follower: 1台の STS3215 を ESP-NOW 経由で制御するテスト。

構成:
  [M5StickC Leader] ══ESP-NOW══> [M5StickC Follower] ──Waveshare──> STS3215 (ID=2)

Leader 側は firmware の Leader モード (m0) で、Python から USB で制御指令を送る代わりに
firmware 内部で SCS READ → ESP-NOW 送信を自動実行する。

このスクリプトは:
  1. Leader と Follower の両方をセットアップ
  2. Leader にサーボを繋いで手で動かす → Follower のサーボが追従するかテスト
  3. または Leader なしで Follower を直接制御するモードも提供

Usage:
  # Leader-Follower テスト (2台)
  python so101_single_servo_test.py --leader COM22 --follower COM23 --id 2

  # Follower のみ直接制御 (1台、USB_THROUGH 経由)
  python so101_single_servo_test.py --follower COM23 --id 2 --direct

  # スキャンのみ
  python so101_single_servo_test.py --follower COM23 --scan

  # Follower の位置を読む
  python so101_single_servo_test.py --follower COM23 --id 2 --read-pos

  # Follower でスイープ
  python so101_single_servo_test.py --follower COM23 --id 2 --sweep
"""
import sys
import time
import argparse
import serial


MONITOR_BAUD = 115200


def open_port(name):
    ser = serial.Serial(name, MONITOR_BAUD, timeout=0.3)
    ser.dtr = False
    ser.rts = False
    time.sleep(0.1)
    ser.reset_input_buffer()
    return ser


def send_cmd(ser, cmd, wait=0.3):
    ser.write((cmd + "\n").encode())
    ser.flush()
    time.sleep(wait)
    r = b""
    while ser.in_waiting:
        r += ser.read(ser.in_waiting)
        time.sleep(0.05)
    return r.decode("utf-8", errors="replace")


def drain(ser, dur=0.3):
    end = time.time() + dur
    out = b""
    while time.time() < end:
        if ser.in_waiting:
            out += ser.read(ser.in_waiting)
        else:
            time.sleep(0.02)
    return out.decode("utf-8", errors="replace")


def setup_device(ser, mode, servo_id, n_axes=1):
    """Stop, set mode/ID/axes, then start (→ SCANNING → RUNNING)."""
    send_cmd(ser, "x", 0.3)
    send_cmd(ser, f"m{mode}", 0.2)
    send_cmd(ser, f"J{servo_id}", 0.2)
    send_cmd(ser, f"I{n_axes}", 0.2)
    resp = send_cmd(ser, "s", 0.5)
    return resp


def wait_for_running(ser, timeout_s=15):
    """Wait until device reaches RUNNING state."""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        time.sleep(1)
        drain(ser, 0.2)
        resp = send_cmd(ser, "?", 0.3)
        if "RUNNING" in resp:
            return True
        # Print scan progress
        for line in resp.split("\n"):
            if "SCAN" in line or "Missing" in line or "found" in line:
                print(f"  {line.strip()}")
    return False


def get_status(ser):
    """Get verbose status."""
    drain(ser, 0.1)
    return send_cmd(ser, "v", 0.5)


# ============================================================================
# SCS protocol (for --direct mode, USB_THROUGH bypass)
# ============================================================================

def scs_packet(sid, inst, params):
    length = len(params) + 2
    chk = (~(sid + length + inst + sum(params))) & 0xFF
    return bytes([0xFF, 0xFF, sid, length, inst]) + bytes(params) + bytes([chk])


def scs_ping(sid):
    return scs_packet(sid, 0x01, [])


def scs_read_pos(sid):
    return scs_packet(sid, 0x02, [0x38, 2])


def scs_write_pos(sid, pos, speed=500):
    return scs_packet(sid, 0x03, [0x2A, pos & 0xFF, (pos >> 8) & 0xFF,
                                   0, 0, speed & 0xFF, (speed >> 8) & 0xFF])


def scs_torque(sid, on):
    return scs_packet(sid, 0x03, [0x28, 1 if on else 0])


def parse_status(resp):
    if not resp or len(resp) < 6:
        return None
    idx = resp.find(b"\xff\xff")
    if idx < 0 or idx + 4 >= len(resp):
        return None
    sid = resp[idx + 2]
    slen = resp[idx + 3]
    err = resp[idx + 4]
    params = list(resp[idx + 5:idx + 3 + slen]) if idx + 3 + slen <= len(resp) else []
    return {"id": sid, "error": err, "params": params}


def hex_str(data):
    return " ".join(f"{b:02X}" for b in data)


# ============================================================================
# Test modes
# ============================================================================

def test_leader_follower(leader_port, follower_port, servo_id, duration_s=30):
    """Leader-Follower test: Leader reads servo, Follower writes via ESP-NOW."""
    print("=" * 70)
    print("Leader-Follower Test (ESP-NOW)")
    print(f"  Leader:   {leader_port}")
    print(f"  Follower: {follower_port}")
    print(f"  Servo ID: {servo_id}")
    print(f"  Duration: {duration_s}s")
    print("=" * 70)

    leader = open_port(leader_port)
    follower = open_port(follower_port)

    try:
        # Setup Follower first (needs time to scan)
        print("\n[1/4] Setting up Follower...")
        setup_device(follower, mode=1, servo_id=servo_id)
        print("  Waiting for Follower RUNNING...")
        if not wait_for_running(follower, 15):
            print("  ERROR: Follower did not reach RUNNING. Check servo connection.")
            return

        # Setup Leader
        print("\n[2/4] Setting up Leader...")
        setup_device(leader, mode=0, servo_id=servo_id)
        print("  Waiting for Leader RUNNING...")
        if not wait_for_running(leader, 15):
            print("  ERROR: Leader did not reach RUNNING. Check servo connection.")
            return

        # Run
        print(f"\n[3/4] Running for {duration_s}s — move the Leader servo by hand!")
        print("  Leader reads position → ESP-NOW → Follower writes position")
        time.sleep(duration_s)

        # Collect stats
        print("\n[4/4] Collecting stats...")
        drain(leader, 0.2)
        drain(follower, 0.2)
        leader_v = get_status(leader)
        follower_v = get_status(follower)

        print("\n--- Leader ---")
        for line in leader_v.split("\n"):
            l = line.strip()
            if l and ("STATUS" in l or "read" in l or "pos" in l or "WIFI" in l):
                print(f"  {l}")

        print("\n--- Follower ---")
        for line in follower_v.split("\n"):
            l = line.strip()
            if l and ("STATUS" in l or "recv" in l or "write" in l or "WIFI" in l):
                print(f"  {l}")

        # Stop
        send_cmd(leader, "x", 0.3)
        send_cmd(follower, "x", 0.3)
        print("\nDone. Both stopped.")

    finally:
        leader.close()
        follower.close()


def test_scan(port, servo_id=None):
    """Scan servo IDs on the bus."""
    print("=" * 70)
    print(f"Servo Scan on {port}")
    print("=" * 70)

    ser = open_port(port)
    try:
        send_cmd(ser, "x", 0.3)
        if servo_id:
            send_cmd(ser, f"J{servo_id}", 0.2)
            send_cmd(ser, "I1", 0.2)
        resp = send_cmd(ser, "S", 2.0)
        print(resp)
    finally:
        ser.close()


def test_read_pos(port, servo_id):
    """Read current position via Leader mode."""
    print(f"Reading position of ID={servo_id} on {port}...")

    ser = open_port(port)
    try:
        setup_device(ser, mode=0, servo_id=servo_id)
        if not wait_for_running(ser, 10):
            print("ERROR: not RUNNING")
            return
        time.sleep(1)
        drain(ser, 0.2)
        v = get_status(ser)
        for line in v.split("\n"):
            l = line.strip()
            if "pos" in l:
                print(f"  {l}")
        send_cmd(ser, "x", 0.3)
    finally:
        ser.close()


def test_sweep(port, servo_id, speed=500):
    """Sweep test: move servo through positions via Leader-Follower."""
    print("=" * 70)
    print(f"Sweep Test (Follower mode, ID={servo_id}) on {port}")
    print("=" * 70)

    ser = open_port(port)
    try:
        # Use Follower mode: we'll manually send pos_packets
        # Actually simpler: use the firmware's built-in Leader mode
        # and physically move the servo. But for automated sweep,
        # we need direct SCS control.
        # → Use bridge's USB_THROUGH if available, or directly set mode.

        # For now, use Leader mode to READ current pos, then print
        setup_device(ser, mode=0, servo_id=servo_id)
        if not wait_for_running(ser, 10):
            print("ERROR: not RUNNING")
            return

        print("Reading positions for 5 seconds...")
        for i in range(5):
            time.sleep(1)
            drain(ser, 0.1)
            v = send_cmd(ser, "?", 0.3)
            for line in v.split("\n"):
                l = line.strip()
                if "read_ok" in l or "pos" in l:
                    print(f"  [{i+1}s] {l}")

        send_cmd(ser, "x", 0.3)
        print("Done.")
    finally:
        ser.close()


# ============================================================================
# Main
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="SO-101 single servo test via ESP-NOW Leader-Follower",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Leader-Follower テスト (2台 + サーボ2台)
  python so101_single_servo_test.py --leader COM22 --follower COM23 --id 2

  # スキャン
  python so101_single_servo_test.py --follower COM23 --scan

  # 位置読取
  python so101_single_servo_test.py --follower COM23 --id 2 --read-pos

  # 1台で Leader として位置モニタ
  python so101_single_servo_test.py --follower COM23 --id 2 --sweep
""")
    parser.add_argument("--leader", type=str, default=None,
                        help="Leader COM port (e.g. COM22)")
    parser.add_argument("--follower", type=str, required=True,
                        help="Follower COM port (e.g. COM23)")
    parser.add_argument("--id", type=int, default=2,
                        help="Servo ID (default: 2)")
    parser.add_argument("--duration", type=int, default=30,
                        help="Test duration in seconds (default: 30)")
    parser.add_argument("--scan", action="store_true",
                        help="Scan servo IDs only")
    parser.add_argument("--read-pos", action="store_true",
                        help="Read current position")
    parser.add_argument("--sweep", action="store_true",
                        help="Sweep/monitor test")
    args = parser.parse_args()
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    if args.scan:
        test_scan(args.follower, args.id)
    elif args.read_pos:
        test_read_pos(args.follower, args.id)
    elif args.sweep:
        test_sweep(args.follower, args.id)
    elif args.leader:
        test_leader_follower(args.leader, args.follower, args.id, args.duration)
    else:
        # Default: scan + read position
        test_scan(args.follower, args.id)
        print()
        test_read_pos(args.follower, args.id)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
