#!/usr/bin/env python3
"""
SO-101 Leader-Follower Position Tracking Test

Collects position data from both Leader and Follower via USB serial,
then analyzes tracking accuracy.

Leader outputs:  POS: [1]=xxxx [2]=xxxx ... (read from servos)
Follower outputs: RCV: [1]=xxxx ... (received via ESP-NOW)
                  ACT: [1]=xxxx ... (actual servo position)

Usage:
  python so101_leader_follower_test.py --leader COM22 --follower COM8
  python so101_leader_follower_test.py --leader COM22 --follower COM8 --duration 30
"""

import sys
import re
import time
import argparse
import threading
import serial

SERVO_NAMES = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}


def collect_lines(port, duration, prefixes):
    """Collect lines matching prefixes from a serial port."""
    s = serial.Serial(port, 115200, timeout=1.0)
    s.dtr = False
    s.rts = False
    time.sleep(0.5)
    s.reset_input_buffer()
    # Enable debug output
    s.write(b"d\n")
    s.flush()
    time.sleep(0.3)
    s.read(s.in_waiting)

    lines = []
    end = time.time() + duration
    while time.time() < end:
        if s.in_waiting:
            data = s.read(s.in_waiting).decode(errors="replace")
            for line in data.split("\n"):
                l = line.strip()
                if any(l.startswith(p) for p in prefixes):
                    lines.append(l)
        else:
            time.sleep(0.1)

    # Disable debug output
    s.write(b"d\n")
    s.flush()
    time.sleep(0.2)
    s.read(s.in_waiting)
    s.close()
    return lines


def parse_positions(line):
    """Extract positions from a POS/RCV/ACT line. Returns dict {id: pos}."""
    matches = re.findall(r"\[(\d)\]=(-?\d+)", line)
    return {int(sid): int(pos) for sid, pos in matches}


def parse_stats(line):
    """Extract stats from a POS/RCV line."""
    stats = {}
    for key in ("cycle", "ok", "fail", "recv", "write"):
        m = re.search(r"%s=(\d+)" % key, line)
        if m:
            stats[key] = int(m.group(1))
    return stats


def analyze(leader_lines, follower_rcv, follower_act, duration):
    """Analyze tracking accuracy and print results."""

    # Parse leader positions
    leader_pos = []
    leader_stats = {}
    for line in leader_lines:
        p = parse_positions(line)
        if p:
            leader_pos.append(p)
        s = parse_stats(line)
        if s:
            leader_stats = s

    # Parse follower actual positions
    act_pos = []
    for line in follower_act:
        p = parse_positions(line)
        if p:
            act_pos.append(p)

    # Parse follower receive stats
    follower_stats = {}
    for line in follower_rcv:
        s = parse_stats(line)
        if s:
            follower_stats = s

    n_samples = min(len(leader_pos), len(act_pos))
    if n_samples == 0:
        print("ERROR: No matching samples collected.")
        return

    # Compute per-axis errors
    servo_ids = sorted(leader_pos[0].keys()) if leader_pos else list(range(1, 7))
    diffs = {sid: [] for sid in servo_ids}

    for i in range(n_samples):
        lp = leader_pos[i]
        ap = act_pos[i]
        for sid in servo_ids:
            if sid in lp and sid in ap and ap[sid] >= 0:
                diffs[sid].append(abs(lp[sid] - ap[sid]))

    # Print results
    print()
    print("=" * 70)
    print("SO-101 Leader-Follower Wireless Tracking Test Results")
    print("=" * 70)

    print()
    print("Test Conditions:")
    print("  Communication : ESP-NOW broadcast (11M_L CCK)")
    print("  Control rate  : 50 Hz (20 ms)")
    print("  Axes          : 6 (STS3215 x6)")
    print("  Duration      : %d seconds" % duration)
    print("  Samples       : %d (1 Hz position log)" % n_samples)
    if leader_stats:
        print("  Leader cycle  : %d us" % leader_stats.get("cycle", 0))
        print("  Leader READ   : ok=%d, fail=%d" %
              (leader_stats.get("ok", 0), leader_stats.get("fail", 0)))
    if follower_stats:
        print("  Follower      : recv=%d, write=%d" %
              (follower_stats.get("recv", 0), follower_stats.get("write", 0)))

    print()
    print("Per-Axis Tracking Accuracy (Leader POS vs Follower ACT):")
    print()
    print("  ID  %-15s  %8s  %8s  %8s  %11s" %
          ("Name", "Avg Err", "Max Err", "Min Err", "Follow(<200)"))
    print("  " + "-" * 60)

    total_good = 0
    total_count = 0
    threshold = 200

    for sid in servo_ids:
        d = diffs[sid]
        if not d:
            print("  %2d  %-15s  %8s" % (sid, SERVO_NAMES.get(sid, "?"), "NO DATA"))
            continue
        avg = sum(d) / len(d)
        mx = max(d)
        mn = min(d)
        good = sum(1 for x in d if x < threshold)
        rate = good / len(d) * 100
        total_good += good
        total_count += len(d)
        print("  %2d  %-15s  %8.1f  %8d  %8d  %5.1f%% (%d/%d)" %
              (sid, SERVO_NAMES.get(sid, "?"), avg, mx, mn, rate, good, len(d)))

    overall = total_good / total_count * 100 if total_count > 0 else 0
    print()
    print("  Overall: %d/%d within %d steps = %.1f%%" %
          (total_good, total_count, threshold, overall))

    # ESP-NOW loss
    if follower_stats and leader_stats:
        sent = leader_stats.get("ok", 0) // 6  # 6 reads per send
        recv = follower_stats.get("recv", 0)
        if sent > 0:
            loss = max(0, sent - recv)
            loss_pct = loss / sent * 100
            print("  ESP-NOW loss  : %d/%d = %.2f%%" % (loss, sent, loss_pct))

    print()
    print("=" * 70)
    print()


def main():
    parser = argparse.ArgumentParser(
        description="SO-101 Leader-Follower position tracking test",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--leader", type=str, default="COM22",
                        help="Leader COM port (default: COM22)")
    parser.add_argument("--follower", type=str, default="COM8",
                        help="Follower COM port (default: COM8)")
    parser.add_argument("--duration", type=int, default=15,
                        help="Test duration in seconds (default: 15)")
    args = parser.parse_args()

    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    print("SO-101 Leader-Follower Test")
    print("  Leader:   %s" % args.leader)
    print("  Follower: %s" % args.follower)
    print("  Duration: %d s" % args.duration)
    print()
    print("Waiting for devices to enter RUNNING state...")

    time.sleep(5)

    print("Collecting data for %d seconds..." % args.duration)
    print("Move the Leader arm now!")
    print()

    results = {}

    def worker(name, port, prefixes):
        results[name] = collect_lines(port, args.duration, prefixes)

    t1 = threading.Thread(target=worker, args=("leader", args.leader, ["POS:"]))
    t2 = threading.Thread(target=worker,
                          args=("follower", args.follower, ["RCV:", "ACT:"]))
    t1.start()
    t2.start()
    t1.join()
    t2.join()

    # Separate follower RCV and ACT
    follower_rcv = [l for l in results["follower"] if l.startswith("RCV:")]
    follower_act = [l for l in results["follower"] if l.startswith("ACT:")]

    # Print raw data
    print("--- Raw Data ---")
    print()
    print("Leader (%s): %d lines" % (args.leader, len(results["leader"])))
    for l in results["leader"]:
        print("  " + l)
    print()
    print("Follower (%s): %d RCV + %d ACT lines" %
          (args.follower, len(follower_rcv), len(follower_act)))
    for l in results["follower"]:
        print("  " + l)

    # Analyze
    analyze(results["leader"], follower_rcv, follower_act, args.duration)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
