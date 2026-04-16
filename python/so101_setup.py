#!/usr/bin/env python3
"""
SO-101 Leader-Follower NVS Setup Script

USB Serial 経由で M5StickC / XIAO ESP32C3 の NVS 設定を行う。

Usage:
  # Leader に設定
  python so101_setup.py --port COM28 --leader

  # Follower に設定
  python so101_setup.py --port COM29 --follower

  # ステータス確認のみ
  python so101_setup.py --port COM28 --status

  # カスタム設定
  python so101_setup.py --port COM28 --mode 0 --axes 6 --ids 1,2,3,4,5,6 --period 20000

  # 全コマンドを手動送信
  python so101_setup.py --port COM28 --cmd "m0"
  python so101_setup.py --port COM28 --cmd "s"
"""

import sys
import time
import argparse
import serial


def send_cmd(ser, cmd, wait=0.5):
    """Send command and return response."""
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())
    ser.flush()
    time.sleep(wait)
    resp = b""
    while ser.in_waiting:
        resp += ser.read(ser.in_waiting)
        time.sleep(0.05)
    return resp.decode("utf-8", errors="replace")


def main():
    parser = argparse.ArgumentParser(
        description="SO-101 Leader-Follower NVS Setup",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python so101_setup.py --port COM28 --leader      # Leader に設定
  python so101_setup.py --port COM29 --follower     # Follower に設定
  python so101_setup.py --port COM28 --status       # ステータス確認
  python so101_setup.py --port COM28 --cmd "m0"     # 任意コマンド送信
  python so101_setup.py --port COM28 --start        # 開始 (s コマンド)
  python so101_setup.py --port COM28 --stop         # 停止 (x コマンド)

  # WiFi / ESP-NOW 設定 (NVS 保存、Leader/Follower で一致必須)
  python so101_setup.py --port COM28 --wifi-channel 6
  python so101_setup.py --port COM28 --wifi-phy 3
  python so101_setup.py --port COM28 --wifi-ampdu 0
  python so101_setup.py --port COM28 --wifi-show
""")
    parser.add_argument("--port", type=str, required=True,
                        help="COM port (e.g. COM28)")
    parser.add_argument("--baud", type=int, default=115200,
                        help="USB serial baud (default: 115200)")
    parser.add_argument("--leader", action="store_true",
                        help="Set as Leader (m0)")
    parser.add_argument("--follower", action="store_true",
                        help="Set as Follower (m1)")
    parser.add_argument("--mode", type=int, default=None,
                        help="Mode: 0=LEADER, 1=FOLLOWER, 2=BRIDGE_LEADER")
    parser.add_argument("--axes", type=int, default=None,
                        help="Number of axes (1-6)")
    parser.add_argument("--ids", type=str, default=None,
                        help="Servo IDs comma-separated (e.g. 1,2,3,4,5,6)")
    parser.add_argument("--period", type=int, default=None,
                        help="Control period in us (e.g. 20000=50Hz)")
    parser.add_argument("--status", action="store_true",
                        help="Show status only")
    parser.add_argument("--start", action="store_true",
                        help="Send start command (s)")
    parser.add_argument("--stop", action="store_true",
                        help="Send stop command (x)")
    parser.add_argument("--debug", action="store_true",
                        help="Toggle debug output (d)")
    parser.add_argument("--cmd", type=str, default=None,
                        help="Send raw command")
    parser.add_argument("--help-device", action="store_true",
                        help="Show device help (h command)")
    # WiFi / ESP-NOW settings (NVS-persisted via espnow_wifi_config.h)
    parser.add_argument("--wifi-channel", type=int, default=None,
                        help="WiFi channel 1-13 (Leader/Follower must match)")
    parser.add_argument("--wifi-phy", type=int, default=None,
                        help="PHY rate idx 0-13 (3=11M_L default)")
    parser.add_argument("--wifi-ampdu", type=int, default=None,
                        help="AMPDU 0=off(11b+g) / 1=on(11b+g+n)")
    parser.add_argument("--wifi-power", type=int, default=None,
                        help="TX power 2-21 dBm (volatile, not NVS-saved)")
    parser.add_argument("--wifi-show", action="store_true",
                        help="Show current WiFi config (W)")
    args = parser.parse_args()

    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    print("Opening %s..." % args.port)
    # Set dtr/rts BEFORE open so the ESP32 isn't reset on port-open
    # (required for XIAO ESP32C3 / AtomS3 native USB-CDC; harmless on FTDI)
    ser = serial.Serial()
    ser.port = args.port
    ser.baudrate = args.baud
    ser.timeout = 2.0
    ser.dtr = False
    ser.rts = False
    ser.open()
    time.sleep(3)  # wait for USB CDC + boot
    ser.reset_input_buffer()
    # Drain boot messages
    time.sleep(1)
    boot = ser.read(ser.in_waiting or 0).decode(errors="replace")
    if boot.strip():
        print("Boot:")
        for line in boot.split("\n"):
            l = line.strip()
            if l:
                print("  %s" % l[:120])
        print()

    try:
        if args.help_device:
            r = send_cmd(ser, "h", 0.5)
            print(r)
            return 0

        if args.status:
            r = send_cmd(ser, "?", 0.5)
            print(r)
            r = send_cmd(ser, "v", 0.5)
            print(r)
            return 0

        if args.cmd:
            r = send_cmd(ser, args.cmd, 0.5)
            print(r)
            return 0

        if args.stop:
            r = send_cmd(ser, "x", 0.5)
            print(r)
            return 0

        if args.start:
            r = send_cmd(ser, "s", 3.0)
            print(r)
            return 0

        if args.debug:
            r = send_cmd(ser, "d", 0.3)
            print(r)
            return 0

        # WiFi / ESP-NOW commands (standalone; no stop/start wrapping)
        wifi_cmds = []
        if args.wifi_channel is not None:
            if not 1 <= args.wifi_channel <= 13:
                print("ERR: --wifi-channel must be 1-13"); return 2
            wifi_cmds.append(("C%d" % args.wifi_channel,
                              "Channel=%d" % args.wifi_channel))
        if args.wifi_phy is not None:
            if not 0 <= args.wifi_phy <= 13:
                print("ERR: --wifi-phy must be 0-13"); return 2
            wifi_cmds.append(("R%d" % args.wifi_phy,
                              "PhyRate=%d" % args.wifi_phy))
        if args.wifi_ampdu is not None:
            if args.wifi_ampdu not in (0, 1):
                print("ERR: --wifi-ampdu must be 0 or 1"); return 2
            wifi_cmds.append(("A%d" % args.wifi_ampdu,
                              "AMPDU=%d" % args.wifi_ampdu))
        if args.wifi_power is not None:
            if not 2 <= args.wifi_power <= 21:
                print("ERR: --wifi-power must be 2-21"); return 2
            wifi_cmds.append(("P%d" % args.wifi_power,
                              "TxPower=%ddBm (volatile)" % args.wifi_power))

        if wifi_cmds:
            for cmd, desc in wifi_cmds:
                r = send_cmd(ser, cmd, 0.4)
                print("-> %s" % desc)
                print(r.strip())
            # Always show resulting config after a WiFi change
            r = send_cmd(ser, "W", 0.4)
            print(r.strip())
            return 0

        if args.wifi_show:
            r = send_cmd(ser, "W", 0.4)
            print(r.strip())
            return 0

        # Setup commands
        cmds = []

        # Always stop first for config changes
        need_config = (args.leader or args.follower or args.mode is not None
                       or args.axes or args.ids or args.period)
        if need_config:
            cmds.append(("x", "Stop"))

        if args.leader:
            cmds.append(("m0", "Mode=LEADER"))
        elif args.follower:
            cmds.append(("m1", "Mode=FOLLOWER"))
        elif args.mode is not None:
            cmds.append(("m%d" % args.mode, "Mode=%d" % args.mode))

        if args.ids:
            cmds.append(("J%s" % args.ids, "IDs=%s" % args.ids))

        if args.axes:
            cmds.append(("I%d" % args.axes, "Axes=%d" % args.axes))

        if args.period:
            cmds.append(("G%d" % args.period, "Period=%dus" % args.period))

        if not cmds and not args.status:
            parser.print_help()
            return 1

        for cmd, desc in cmds:
            r = send_cmd(ser, cmd, 0.5)
            lines = [l.strip() for l in r.split("\n") if l.strip()]
            print("%s: %s" % (desc, lines[0] if lines else "(no response)"))

        # Show final status
        print()
        r = send_cmd(ser, "?", 0.5)
        print(r.strip())

    finally:
        ser.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
