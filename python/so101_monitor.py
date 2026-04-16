#!/usr/bin/env python3
"""Live monitor SO-101 STS3215 Present_Position.

Default: polls all 6 servos and redraws a live table every period.
  python so101_monitor.py --port COM25

Single-axis focus mode (timestamped scrolling log, finer period):
  python so101_monitor.py --port COM25 --id 3 --period 0.05

The script handles both Feetech sign-magnitude (bit15=sign, bits0-14=value)
and two's-complement decodings so you can spot Phase register bit 4
differences between units.
"""

import argparse
import time
import serial
import sys

SERVO_IDS = [1, 2, 3, 4, 5, 6]
SERVO_NAMES = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}

ADDR_PRESENT_POSITION = 0x38
ADDR_HOMING_OFFSET    = 0x1F
ADDR_MIN_POSITION     = 0x09
ADDR_MAX_POSITION     = 0x0B
ADDR_PHASE            = 0x12


# --------------------------------------------------------------------------
# SCS protocol helpers (same convention as so101_calibrate.py)
# --------------------------------------------------------------------------

def scs_packet(sid, inst, params):
    length = len(params) + 2
    chk = (~(sid + length + inst + sum(params))) & 0xFF
    return bytes([0xFF, 0xFF, sid, length, inst]) + bytes(params) + bytes([chk])


def send_recv(ser, pkt, timeout=0.03):
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
    params = list(resp[idx + 5: idx + 3 + slen])
    return params if err == 0 else None


def read_byte(ser, sid, addr):
    p = parse_params(send_recv(ser, scs_packet(sid, 0x02, [addr, 1])))
    return p[0] if p and len(p) >= 1 else None


def read_word(ser, sid, addr):
    p = parse_params(send_recv(ser, scs_packet(sid, 0x02, [addr, 2])))
    return (p[0] | (p[1] << 8)) if p and len(p) >= 2 else None


def to_signed_twos(u):
    """Two's-complement signed interpretation. Used for Homing_Offset."""
    return u - 0x10000 if u >= 0x8000 else u


def to_signed_magnitude(u):
    """Feetech Present_Position uses sign-magnitude (bit15=sign, bits0-14=value).
    Example: 0x807A (32826) -> -(32826-32768) = -58.
    """
    if u >= 0x8000:
        return -(u - 0x8000)
    return u


def decode_pres(u, mode):
    """Decode Present_Position per `mode` in {'twos', 'signmag'}."""
    if mode == "twos":
        return to_signed_twos(u)
    return to_signed_magnitude(u)


# --------------------------------------------------------------------------
# EEPROM summary (printed once at startup)
# --------------------------------------------------------------------------

def read_eeprom_summary(ser, ids):
    """Return dict[id -> {min, max, offset, phase}]. None values on read fail."""
    summary = {}
    for sid in ids:
        mn = read_word(ser, sid, ADDR_MIN_POSITION)
        mx = read_word(ser, sid, ADDR_MAX_POSITION)
        off_u = read_word(ser, sid, ADDR_HOMING_OFFSET)
        phase = read_byte(ser, sid, ADDR_PHASE)
        summary[sid] = {
            "min":    mn,
            "max":    mx,
            "offset": to_signed_twos(off_u) if off_u is not None else None,
            "phase":  phase,
        }
    return summary


def print_eeprom_summary(summary):
    print()
    print("EEPROM summary (read once at startup):")
    print(f"  {'ID':>3}  {'Name':<15} {'Phase':>5}  {'Min':>6}  {'Max':>6}  {'Range':>6}  {'Offset':>7}")
    print(f"  {'-'*3}  {'-'*15} {'-'*5}  {'-'*6}  {'-'*6}  {'-'*6}  {'-'*7}")
    for sid, d in summary.items():
        name = SERVO_NAMES.get(sid, "?")
        rng = (d["max"] - d["min"]) if d["min"] is not None and d["max"] is not None else None
        ph_str = ("0x%02X" % d["phase"]) if d["phase"] is not None else "ERR"
        mn_str = d["min"] if d["min"] is not None else "ERR"
        mx_str = d["max"] if d["max"] is not None else "ERR"
        rng_str = rng if rng is not None else "ERR"
        off_str = ("%+d" % d["offset"]) if d["offset"] is not None else "ERR"
        flag = ""
        if d["phase"] is not None and (d["phase"] & 0x10):
            flag = "  ***PHASE_BIT4_SET (sign-mag mode)"
        print(f"  {sid:>3}  {name:<15} {ph_str:>5}  {mn_str!s:>6}  {mx_str!s:>6}  {rng_str!s:>6}  {off_str:>7}{flag}")
    print()


# --------------------------------------------------------------------------
# All-axes live table mode (default)
# --------------------------------------------------------------------------

def monitor_all(ser, summary, period, decode_mode):
    """Redraw a 6-axis table every `period` seconds."""
    print(f"Polling all 6 axes every {period*1000:.0f}ms. Ctrl+C to stop.")
    print(f"Pres_s decoded as: {decode_mode}")
    print()
    # Track min/max per axis (signed, decoded)
    mins = {sid: 32767 for sid in SERVO_IDS}
    maxes = {sid: -32768 for sid in SERVO_IDS}

    header = f"  {'ID':>3}  {'Name':<15}  {'Pres_u':>7}  {'Pres_s':>7}  {'raw':>6}  {'min':>6}  {'max':>6}  note"
    sep = "  " + "-" * (len(header) - 2)
    n_lines = len(SERVO_IDS) + 2  # header + sep + rows
    first_draw = True
    try:
        t0 = time.time()
        while True:
            # Move cursor up to overwrite previous block (skip first draw)
            if not first_draw:
                sys.stdout.write(f"\033[{n_lines}A")
            print(header)
            print(sep)
            for sid in SERVO_IDS:
                name = SERVO_NAMES.get(sid, "?")
                pres_u = read_word(ser, sid, ADDR_PRESENT_POSITION)
                if pres_u is None:
                    print(f"  {sid:>3}  {name:<15}  {'ERR':>7}  {'ERR':>7}  {'n/a':>6}  {mins[sid]!s:>6}  {maxes[sid]!s:>6}")
                    continue
                pres_s = decode_pres(pres_u, decode_mode)
                off = summary[sid]["offset"]
                raw = (pres_s + off) if off is not None else None
                if pres_s > maxes[sid]: maxes[sid] = pres_s
                if pres_s < mins[sid]: mins[sid] = pres_s
                note_parts = []
                mx = summary[sid]["max"]
                if mx is not None and pres_s >= mx - 5:
                    note_parts.append(f"AT_MAX({mx})")
                if pres_s < 0:
                    note_parts.append("BELOW_ZERO")
                if summary[sid]["phase"] is not None and (summary[sid]["phase"] & 0x10):
                    note_parts.append("phase_bit4=1")
                note = " ".join(note_parts)
                raw_str = str(raw) if raw is not None else "n/a"
                print(f"  {sid:>3}  {name:<15}  {pres_u:>7d}  {pres_s:>7d}  {raw_str:>6}  {mins[sid]:>6d}  {maxes[sid]:>6d}  {note}")
            first_draw = False
            time.sleep(period)
    except KeyboardInterrupt:
        print()
        print(f"Session ended after {time.time()-t0:.1f}s")
        print()
        print(f"  {'ID':>3}  {'Name':<15}  {'Pres_s min':>10}  {'Pres_s max':>10}  {'Range':>6}")
        for sid in SERVO_IDS:
            rng = maxes[sid] - mins[sid]
            name = SERVO_NAMES.get(sid, "?")
            print(f"  {sid:>3}  {name:<15}  {mins[sid]:>10d}  {maxes[sid]:>10d}  {rng:>6d}")


# --------------------------------------------------------------------------
# Single-axis focus mode (scrolling log)
# --------------------------------------------------------------------------

def monitor_single(ser, sid, summary, period, decode_mode):
    d = summary[sid]
    name = SERVO_NAMES.get(sid, "?")
    print(f"Focus mode: ID={sid} ({name}), polling every {period*1000:.0f}ms.")
    print(f"Pres_s decoded as: {decode_mode}")
    print(f"  Min={d['min']} Max={d['max']} Offset={d['offset']} Phase={d['phase'] if d['phase'] is None else ('0x%02X' % d['phase'])}")
    print()
    print(f"  {'time':>6} | {'Pres_u':>7} | {'Pres_s':>7} | {'raw=Pres+Off':>14} | note")
    print(f"  {'-'*6}-+-{'-'*7}-+-{'-'*7}-+-{'-'*14}-+-{'-'*30}")

    hit_max = 0
    below_zero = 0
    min_seen = 32767
    max_seen = -32768

    try:
        t0 = time.time()
        while True:
            pres_u = read_word(ser, sid, ADDR_PRESENT_POSITION)
            if pres_u is None:
                print(f"  {time.time()-t0:6.2f} | (read fail)")
                time.sleep(period)
                continue
            pres_s = decode_pres(pres_u, decode_mode)
            raw = (pres_s + d["offset"]) if d["offset"] is not None else None
            note_parts = []
            if d["max"] is not None and pres_s >= d["max"] - 5:
                note_parts.append(f"AT_MAX({d['max']})")
                hit_max += 1
            if pres_s < 0:
                note_parts.append("BELOW_ZERO")
                below_zero += 1
            if pres_s > max_seen: max_seen = pres_s
            if pres_s < min_seen: min_seen = pres_s
            raw_str = str(raw) if raw is not None else "n/a"
            print(f"  {time.time()-t0:6.2f} | {pres_u:>7d} | {pres_s:>7d} | {raw_str:>14} | {' '.join(note_parts)}")
            time.sleep(period)
    except KeyboardInterrupt:
        print()
        print(f"Session ended after {time.time()-t0:.1f}s")
        print(f"  Pres_s range:     [{min_seen}, {max_seen}]")
        if d["max"] is not None:
            print(f"  EEPROM Max:       {d['max']}  {'(exceeded)' if max_seen > d['max'] else '(within)'}")
        print(f"  AT_MAX samples:    {hit_max}")
        print(f"  BELOW_ZERO samples:{below_zero}")


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="SO-101 Present_Position live monitor")
    parser.add_argument("--port", required=True, help="COM port (e.g. COM25)")
    parser.add_argument("--baud", type=int, default=1000000)
    parser.add_argument("--id", type=int, default=None,
                        help="servo ID for single-axis focus mode (default: monitor all 6)")
    parser.add_argument("--period", type=float, default=0.2,
                        help="poll period in seconds (default 0.2 = 5Hz all-axes, recommend 0.05 for single-axis)")
    parser.add_argument("--decode", choices=["signmag", "twos", "both"], default="signmag",
                        help="Present_Position signed decoding (default: signmag — Feetech convention). "
                             "'both' shows two columns")
    args = parser.parse_args()

    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    time.sleep(0.2)
    ser.reset_input_buffer()

    try:
        ids = SERVO_IDS if args.id is None else [args.id]
        summary = read_eeprom_summary(ser, ids)
        print_eeprom_summary(summary)

        if args.decode == "both":
            print("NOTE: --decode both not yet implemented. Falling back to signmag.")
            decode_mode = "signmag"
        else:
            decode_mode = args.decode

        if args.id is None:
            monitor_all(ser, summary, args.period, decode_mode)
        else:
            monitor_single(ser, args.id, summary, args.period, decode_mode)
    finally:
        ser.close()


if __name__ == "__main__":
    sys.exit(main())
