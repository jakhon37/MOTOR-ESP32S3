#!/usr/bin/env python3
"""Wheel and encoder tests for this firmware. See docs/FIRST_DRIVE.md.

Stop the bridge first: only one program can use the port. Needs pyserial, which the robot's
container already has.

  watch      counts, speeds and encoder pin activity (no motor commands)
  turns      hand test: counts per wheel turn, direction, encoder_cpr (no motor commands)
  direction  drives each wheel briefly, checks motor and encoder direction
  ff         open-loop PWM sweep, prints the deadband, slope and kick values for config.h
  speed      requests 0.02 to 0.25 m/s on each wheel, prints the speed measured
  timeout    drives, stops sending commands, measures how long the wheels take to stop
  hold       holds a speed or PWM for a time, and prints what the controller did
  lines      prints the raw lines from the board (no motor commands)
  send       sends lines to the board as written and prints the replies, like "g cpr 1496" or "d"
"""
import argparse
import math
import re
import statistics
import sys
import threading
import time

import serial

WHEEL_RADIUS = 0.034
# A wheel moving the robot forward gets a positive request on the left and a negative one on the
# right, because the bridge negates the right wheel's command and count. Its count follows.
FORWARD_SIGN = {"L": 1, "R": -1}
OLD_LEFT_PINS = ("13", "17")
REQUESTS = (0.02, 0.03, 0.05, 0.08, 0.10, 0.15, 0.20, 0.25)
COMMAND_PERIOD_S = 0.05  # 20 Hz, like the bridge at its slowest
MAX_START_PWM = 600  # the sweep from rest gives up here: nothing should need more, in the air
DEFAULT_KICK = 550  # the old firmware's 500, the only level known to work on the floor, plus a margin
MIN_DRIVE_COUNTS = 50  # a 2 s drive at 0.1 m/s is about 1,400 counts, and one dead line only jitters
MIN_COUNTS_PER_TURN = 100  # about 1,500 is expected, and the old one-edge count was about 350


class Board:
    def __init__(self, port):
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = 115200
        self.ser.timeout = 0.01
        self.ser.dtr = True  # same line states as the bridge: this doesn't reset the board
        self.ser.rts = False
        self.ser.open()
        self.counts = None
        self.diag = {}
        self.pins = {}
        self.tuning = {}
        self.line_counts = {"e": 0, "i": 0}
        self.echo = None  # "all" or "replies" prints lines as they arrive
        self.saw_ready = False
        self.running = True
        threading.Thread(target=self.read_lines, daemon=True).start()
        time.sleep(0.5)
        if self.counts is None:
            raise SystemExit(f"No 'e' lines from {port}. Is this firmware on the board, and the bridge stopped?")

    def read_lines(self):
        """Runs all the time, so the counts stay live while waiting for the person at the robot."""
        buf = b""
        while self.running:
            try:
                buf += self.ser.read(self.ser.in_waiting or 1)
            except (OSError, TypeError, serial.SerialException):
                if self.running:
                    print("Lost the serial port. Did the board reset or the cable move?")
                return
            *lines, buf = re.split(rb"[\r\n]+", buf)
            for raw in lines:
                try:
                    self.handle(raw.decode("ascii", errors="ignore"))
                except (ValueError, IndexError):
                    pass  # a damaged line: the next one will do

    def close(self):
        try:
            self.stop()
        except (OSError, serial.SerialException):
            pass
        self.running = False
        time.sleep(0.05)
        self.ser.close()

    def handle(self, line):
        parts = line.split()
        telemetry = bool(parts) and parts[0] in self.line_counts
        if telemetry:
            self.line_counts[parts[0]] += 1
        if self.echo == "all" or (self.echo == "replies" and not telemetry):
            print(line)
        if len(parts) == 3 and parts[0] == "e":
            try:
                self.counts = {"L": int(parts[1]), "R": int(parts[2])}
            except ValueError:
                pass
        elif len(parts) > 1 and parts[0] == "d":
            if parts[1] == "pins":
                for item in parts[2:]:
                    pin, value = item.split("=")
                    level, transitions = value.split("/")
                    self.pins[pin] = (int(level), int(transitions))
            elif parts[1] == "fault":
                print(f"  board says: {parts[2]} encoder fault {parts[3]}")
            elif parts[1] in ("L", "R"):
                self.diag[parts[1]] = dict(item.split("=") for item in parts[2:])
            else:
                self.diag["board"] = dict(item.split("=") for item in parts[1:])
        elif len(parts) > 2 and parts[0] == "g":
            self.tuning = dict(zip(parts[1::2], parts[2::2]))
        elif "READY" in line:
            self.saw_ready = True
            if not self.echo:
                print(f"  board rebooted: {line}")

    def send(self, text):
        self.ser.write(text.encode() + b"\r")
        self.ser.flush()

    def velocity(self, left, right):
        self.send(f"m {left:.3f} {right:.3f}")

    def force(self, left, right):
        self.send(f"f {int(left)} {int(right)}")

    def stop(self):
        self.send("m 0 0")

    def diagnostics(self):
        self.send("d")
        time.sleep(0.1)

    def get_tuning(self):
        self.send("g")
        time.sleep(0.1)
        return self.tuning

    def counts_per_turn(self):
        cpr = float(self.get_tuning().get("cpr", 0))
        if cpr <= 0:
            raise SystemExit("The board did not report its tuning ('g').")
        return cpr


def drive_for(seconds, send):
    """Calls send() at 20 Hz for this long."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        send()
        time.sleep(COMMAND_PERIOD_S)


def rest(board, seconds=0.5):
    board.stop()
    time.sleep(seconds)


def wheel_command(wheel, value):
    """Left and right arguments for a command on one wheel, forward positive."""
    signed = FORWARD_SIGN[wheel] * value
    return (signed, 0) if wheel == "L" else (0, signed)


def forward_speed(board, wheel, start, seconds, cpr):
    """Speed over a time, forward positive, from the count at its start."""
    moved = FORWARD_SIGN[wheel] * (board.counts[wheel] - start)
    return moved / cpr * 2 * math.pi * WHEEL_RADIUS / seconds


def confirm_drive(on_floor=False):
    if on_floor:
        print("This drives the robot on the floor. Clear the way ahead, with a hand near the motor power.")
    else:
        print("This drives the motors. Both wheels must be off the ground, with someone at the robot.")
    if input("Type yes to go on: ").strip().lower() != "yes":
        raise SystemExit("Stopped. Nothing was driven.")


def pin_changes(before, after):
    return {pin: after[pin][1] - before.get(pin, (0, 0))[1] for pin in after}


def cmd_watch(board, args):
    print("Once a second: counts (change), lines per second, pin=level/changes. Ctrl-C to stop.")
    last_counts = dict(board.counts)
    last_lines = dict(board.line_counts)
    last_pins = {}
    last_time = time.monotonic()
    end = last_time + args.seconds if args.seconds else None
    while end is None or time.monotonic() < end:
        board.diagnostics()
        time.sleep(0.9)
        now = time.monotonic()
        rate = {k: (board.line_counts[k] - last_lines[k]) / (now - last_time) for k in last_lines}
        changes = pin_changes(last_pins, board.pins)
        pins = " ".join(f"{p}={board.pins[p][0]}/{changes[p]}" for p in board.pins)
        print(f"L {board.counts['L']:>9} ({board.counts['L'] - last_counts['L']:+6d})   "
              f"R {board.counts['R']:>9} ({board.counts['R'] - last_counts['R']:+6d})   "
              f"e {rate['e']:3.0f}/s  i {rate['i']:3.0f}/s   {pins}")
        last_counts = dict(board.counts)
        last_lines = dict(board.line_counts)
        last_pins = dict(board.pins)
        last_time = now


def hand_turns(board, wheel, name, direction, turns):
    """One step of the hand test: returns the count change, forward positive, and which pins moved."""
    input(f"Mark the {name} wheel. Press Enter, then turn it {turns} full turns {direction} by hand.")
    board.diagnostics()
    start = dict(board.counts)
    pins_before = dict(board.pins)
    input(f"Turning the {name} wheel {direction}. Press Enter when done.")
    board.diagnostics()
    moved = {w: board.counts[w] - start[w] for w in start}
    other = "R" if wheel == "L" else "L"
    print(f"  {name}: {moved[wheel]:+d} counts ({moved[wheel] / turns:+.1f} per turn). Other wheel: {moved[other]:+d}.")
    if abs(moved[other]) > abs(moved[wheel]):
        print("  The other wheel's count moved more. Are the left and right encoders swapped?")
    return moved[wheel], pin_changes(pins_before, board.pins)


def cmd_turns(board, args):
    print("Hand test, no motor commands. Turn the wheels the way that moves the robot forward, then back.")
    sizes = {}
    for wheel, name, pin_a, pin_b in (("L", "left", "18", "14"), ("R", "right", "4", "5")):
        forward, pins = hand_turns(board, wheel, name, "forward", args.turns)
        backward, pins_back = hand_turns(board, wheel, name, "backward", args.turns)
        changes = {p: pins[p] + pins_back[p] for p in pins}

        moved_a, moved_b = changes[pin_a], changes[pin_b]
        print(f"  Pin changes seen: A (GPIO {pin_a}) {moved_a}, B (GPIO {pin_b}) {moved_b}")
        if wheel == "L" and any(changes[p] for p in OLD_LEFT_PINS):
            print(f"  Activity on old left pins: {[(p, changes[p]) for p in OLD_LEFT_PINS]}."
                  " A left encoder wire is on GPIO 13 or 17, not on 18 or 14.")
        if not moved_a or not moved_b:
            dead = pin_a if not moved_a else pin_b
            print(f"  GPIO {dead} never changed: check that wire. Quadrature can't count with one line dead.")
            continue
        if min(abs(forward), abs(backward)) < MIN_COUNTS_PER_TURN * args.turns:
            print("  Both pins changed, but the count hardly moved. Check that A and B aren't the same signal"
                  " or the same wire, then repeat.")
            continue

        sign = FORWARD_SIGN[wheel]
        if forward * sign > 0 > backward * sign:
            print("  Direction OK.")
        elif forward * sign < 0 < backward * sign:
            define = "LEFT_ENCODER_SIGN" if wheel == "L" else "RIGHT_ENCODER_SIGN"
            print(f"  Counts run the wrong way. Flip {define} in config.h.")
        else:
            print("  The count did not reverse when the wheel did. Check the A and B wires, then repeat.")
            continue
        sizes[wheel] = [abs(forward) / args.turns, abs(backward) / args.turns]

    if not sizes:
        print("\nNo usable count from either wheel, so no encoder_cpr yet. Fix what's listed above and repeat.")
        return
    all_sizes = [size for pair in sizes.values() for size in pair]
    spread = (max(all_sizes) - min(all_sizes)) / statistics.mean(all_sizes) * 100
    print(f"\nCounts per turn: {[round(s) for s in all_sizes]}, spread {spread:.0f}%.")
    print(f"Set ENCODER_CPR in include/config.h and encoder_cpr in the bridge to {statistics.mean(all_sizes):.0f}.")
    if len(sizes) == 1:
        print("That's from one wheel only. Repeat when the other works: they should match.")
    if spread > 5:
        print("That spread is high for two wheels of the same type: recount with marks, or check the wires.")


def cmd_direction(board, args):
    confirm_drive()
    for wheel, name in (("L", "left"), ("R", "right")):
        input(f"Watch the {name} wheel. Press Enter to drive it for 2 s.")
        start = board.counts[wheel]
        left, right = wheel_command(wheel, 0.1)
        drive_for(2.0, lambda: board.velocity(left, right))
        board.diagnostics()
        info = dict(board.diag.get(wheel, {}))
        rest(board)
        moved = board.counts[wheel] - start
        print(f"  Sent 'm {left:.3f} {right:.3f}'. Count changed by {moved:+d}. Board: "
              f"pwm={info.get('pwm')} feedback={info.get('feedback')} fault={info.get('fault')}")
        answer = ""
        while answer not in ("f", "b", "n"):
            answer = input(f"  Which way did the {name} wheel turn: forward, backward, or not at all? [f/b/n] ").strip().lower()[:1]
        if answer == "n":
            print("  It didn't turn, so this says nothing about direction. Check the motor power and the driver's"
                  " wiring. If those are fine, raise the kick.")
            continue
        forward = answer == "f"
        if not forward:
            print(f"  Wrong way. Swap the numbers of {name.upper()}_MOTOR_IN1 and {name.upper()}_MOTOR_IN2 in config.h.")
        if abs(moved) < MIN_DRIVE_COUNTS:
            print("  The count hardly moved. Run 'turns' to find the dead wire. This wheel runs on feed-forward alone.")
        elif (moved * FORWARD_SIGN[wheel] > 0) != forward:
            define = "LEFT_ENCODER_SIGN" if wheel == "L" else "RIGHT_ENCODER_SIGN"
            print(f"  The count ran against the wheel. Flip {define} in config.h.")
        else:
            print("  Count direction OK.")


def sweep_pwm(board, wheel, cpr):
    """Measures speed against PWM, returns (start_pwm, table). Forward positive."""
    start_pwm = None
    for pwm in range(100, MAX_START_PWM + 1, 25):  # going up from rest: the first PWM that starts the wheel
        rest(board, 0.4)
        start = board.counts[wheel]
        drive_for(0.6, lambda: board.force(*wheel_command(wheel, pwm)))
        speed = forward_speed(board, wheel, start, 0.6, cpr)
        print(f"  from rest, PWM {pwm:4d}: {speed:+.3f} m/s")
        if abs(speed) > 0.01:
            start_pwm = pwm
            break
    rest(board, 0.4)
    if start_pwm is None:
        return None, []

    table = []
    for pwm in range(start_pwm + 300, 40, -25):  # going down while turning: where it stops
        pwm = min(pwm, 1023)
        drive_for(0.5, lambda: board.force(*wheel_command(wheel, pwm)))
        start = board.counts[wheel]
        drive_for(0.5, lambda: board.force(*wheel_command(wheel, pwm)))
        speed = forward_speed(board, wheel, start, 0.5, cpr)
        table.append((pwm, speed))
        print(f"  turning, PWM {pwm:4d}: {speed:+.3f} m/s")
        if abs(speed) < 0.005:
            break
    rest(board, 0.4)
    return start_pwm, table


def fit_line(points):
    """Least squares y = a + b x."""
    n = len(points)
    mean_x = sum(p[0] for p in points) / n
    mean_y = sum(p[1] for p in points) / n
    b = sum((p[0] - mean_x) * (p[1] - mean_y) for p in points) / sum((p[0] - mean_x) ** 2 for p in points)
    return mean_y - b * mean_x, b


def cmd_ff(board, args):
    confirm_drive()
    cpr = board.counts_per_turn()
    print(f"Speeds use the {cpr:.0f} counts per turn on the board, so run 'turns' and set that first.")
    lines = []
    for wheel, name in (("L", "left"), ("R", "right")):
        print(f"{name} wheel:")
        start_pwm, table = sweep_pwm(board, wheel, cpr)
        if start_pwm is None:
            print(f"  The encoder saw no movement up to PWM {MAX_START_PWM}. Find out why with 'turns' and 'direction'.")
            continue
        running = [(speed, pwm) for pwm, speed in table if 0.01 < speed <= 0.40]
        if len(running) < 3:
            print("  Too few running points to fit. Are the counts going the right way? Run 'direction'.")
            continue
        deadband, slope = fit_line(running)
        kick = min(max(start_pwm + 100, DEFAULT_KICK), 1023)  # starting on the floor takes more than this
        print(f"  deadband {deadband:.0f}, slope {slope:.0f} PWM per m/s, starts from rest at PWM {start_pwm}")
        lines.append((name.upper(), name[0], round(deadband), round(slope), kick))
    if lines:
        print("\nIn include/config.h, in the block for measured values:")
        for upper, _, deadband, slope, kick in lines:
            print(f"#define FF_DEADBAND_{upper}  {deadband}\n#define FF_SLOPE_{upper}  {slope}\n#define KICK_PWM_{upper}  {kick}")
        print("\nOr to try them without flashing, until the next reboot:")
        settings = " ".join(f'"g dead_{letter} {deadband}" "g slope_{letter} {slope}" "g kick_{letter} {kick}"'
                            for _, letter, deadband, slope, kick in lines)
        print(f"python3 {sys.argv[0]} send {settings}")
        print("\nThese are for wheels in the air. The floor needs more: see the floor step in docs/FIRST_DRIVE.md.")


def cmd_speed(board, args):
    confirm_drive()
    cpr = board.counts_per_turn()
    closed_before = board.tuning.get("closed", "1")
    board.send(f"g closed {0 if args.open_loop else 1}")
    print(f"{'Feed-forward only' if args.open_loop else 'Closed loop'}, {cpr:.0f} counts per turn. Forward is positive.")
    print("wheel  request  measured  ratio   pwm    ff  trim  fault")
    try:
        for wheel in ("L", "R"):
            for sign in (1, -1):
                for request in REQUESTS:
                    left, right = wheel_command(wheel, sign * request)
                    drive_for(1.5, lambda: board.velocity(left, right))
                    start = board.counts[wheel]
                    drive_for(2.0, lambda: board.velocity(left, right))
                    speed = forward_speed(board, wheel, start, 2.0, cpr)
                    board.velocity(left, right)
                    board.diagnostics()
                    info = board.diag.get(wheel, {})
                    print(f"{wheel:^5}  {sign * request:+7.3f}  {speed:+8.3f}  {speed / (sign * request):5.2f}  "
                          f"{info.get('pwm', '?'):>4}  {info.get('ff', '?'):>4}  {info.get('trim', '?'):>4}  {info.get('fault', '?')}")
                    rest(board, 0.3)

        print("\nBoth wheels together, as a gentle arc: left 0.05, right 0.20 m/s forward.")
        drive_for(1.5, lambda: board.velocity(0.05, -0.20))
        start = dict(board.counts)
        drive_for(2.0, lambda: board.velocity(0.05, -0.20))
        print(f"  left {forward_speed(board, 'L', start['L'], 2.0, cpr):+.3f}  right {forward_speed(board, 'R', start['R'], 2.0, cpr):+.3f} m/s")
        rest(board, 0.5)

        print("A request under 0.01 m/s means stop: 0.005 m/s for 1 s.")
        start = dict(board.counts)
        drive_for(1.0, lambda: board.velocity(0.005, -0.005))
        print(f"  count changes: L {board.counts['L'] - start['L']:+d}  R {board.counts['R'] - start['R']:+d} (both should be about 0)")
    finally:
        board.send(f"g closed {closed_before}")
        rest(board)
    print("Measured against request should be close to 1 at every speed, and the rows should differ.")


def cmd_hold(board, args):
    confirm_drive(args.on_floor)
    unit = "PWM" if args.pwm else "m/s"
    print(f"Holding left {args.left} and right {args.right} {unit}, forward positive, for {args.seconds:.0f} s.")
    if args.pwm:
        send = lambda: board.force(args.left, -args.right)
    else:
        send = lambda: board.velocity(args.left, -args.right)
    start = dict(board.counts)
    drive_for(args.seconds, send)
    send()
    board.diagnostics()
    state = dict(board.diag)
    rest(board)
    print(f"Count changes: L {board.counts['L'] - start['L']:+d}  R {board.counts['R'] - start['R']:+d}")
    if not args.pwm:
        print("At the end, with forward positive, so a positive trim means the wheel needed more than the feed-forward:")
        for wheel in ("L", "R"):
            info = state.get(wheel, {})
            forward = {k: FORWARD_SIGN[wheel] * float(info.get(k, 0)) for k in ("speed", "pwm", "ff", "trim")}
            print(f"  {wheel}: speed={forward['speed']:+.3f} pwm={forward['pwm']:.0f} ff={forward['ff']:.0f} "
                  f"trim={forward['trim']:+.0f} fault={info.get('fault')}")


def cmd_timeout(board, args):
    confirm_drive()
    start = dict(board.counts)
    drive_for(1.5, lambda: board.velocity(0.15, -0.15))
    last_command = time.monotonic()
    encoders_moved = any(abs(board.counts[w] - start[w]) >= MIN_DRIVE_COUNTS for w in start)
    counts = dict(board.counts)
    last_move = last_command
    while time.monotonic() - last_move < 1.0 and time.monotonic() - last_command < 5.0:
        time.sleep(0.02)
        if board.counts != counts:
            counts = dict(board.counts)
            last_move = time.monotonic()
    if encoders_moved:
        print(f"Last count change {last_move - last_command:.2f} s after the last command.")
        print("Expect the 0.5 s timeout plus a little coasting. Over about 1 s is a failure.")
    else:
        print("The encoders showed no movement, so this can't time the stop. Watch the wheels instead: run it"
              " again, and they should stop about 2 s after you start it, half a second after the commands end.")
    board.diagnostics()
    print(f"Board: L pwm={board.diag['L'].get('pwm')} R pwm={board.diag['R'].get('pwm')} (both must be 0)")


def cmd_lines(board, args):
    board.echo = "all"
    if args.reset:
        # A plain reset through the USB port's control lines, the way esptool ends a flash.
        board.ser.dtr = False
        time.sleep(0.1)
        board.ser.rts = True
        time.sleep(0.1)
        board.ser.rts = False
        time.sleep(0.1)
        board.ser.dtr = True
    time.sleep(args.seconds)
    board.echo = None
    if args.reset:
        print(f"Boot line with READY seen: {'yes' if board.saw_ready else 'no'}")


def cmd_send(board, args):
    board.echo = "replies"
    for text in args.lines:
        board.send(text)
        time.sleep(0.2)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="/dev/ttyACM0")
    sub = parser.add_subparsers(dest="command", required=True)
    watch = sub.add_parser("watch")
    watch.add_argument("--seconds", type=float, default=0, help="stop after this long (default: never)")
    turns = sub.add_parser("turns")
    turns.add_argument("--turns", type=int, default=10, help="full turns in each direction")
    sub.add_parser("direction")
    sub.add_parser("ff")
    speed = sub.add_parser("speed")
    speed.add_argument("--open-loop", action="store_true", help="feed-forward only, to see how good it is alone")
    sub.add_parser("timeout")
    hold = sub.add_parser("hold")
    hold.add_argument("left", type=float, help="forward positive, m/s (or PWM with --pwm)")
    hold.add_argument("right", type=float, help="forward positive, m/s (or PWM with --pwm)")
    hold.add_argument("--seconds", type=float, default=20)
    hold.add_argument("--pwm", action="store_true", help="open-loop PWM instead of a speed")
    hold.add_argument("--on-floor", action="store_true", help="the robot is on the floor, not on its stand")
    lines = sub.add_parser("lines")
    lines.add_argument("--seconds", type=float, default=1)
    lines.add_argument("--reset", action="store_true", help="reset the board first, to see its boot line")
    send = sub.add_parser("send")
    send.add_argument("lines", nargs="+", help="this can drive the motors: it asks nothing first")
    args = parser.parse_args()
    sys.stdout.reconfigure(line_buffering=True)  # so that piping into tee shows lines as they come

    board = Board(args.port)
    try:
        globals()[f"cmd_{args.command}"](board, args)
    except KeyboardInterrupt:
        print("\nStopped.")
    except (OSError, serial.SerialException) as err:
        print(f"Serial port error: {err}")
    finally:
        board.close()


if __name__ == "__main__":
    main()
