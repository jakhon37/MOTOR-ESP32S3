# First flash and first drive

A checklist for putting this firmware on the robot's ESP32-S3, with someone at the robot and the wheels off the ground.

**None of this firmware has run on hardware.** It has been compiled, and its control logic and serial protocol have been tested on a PC against a model of a wheel. Motor direction, encoder direction, the encoder wiring, the pulse counter setup, the IMU on these pins, USB behaviour and every number in the measured block of `config.h` are all unconfirmed until you do the steps below.

Take the steps in order, and stop at any step that fails. Commands say whether they run on the **PC** or the **Jetson**, and `container` means inside `auto_ros_humble` (`docker exec -it auto_ros_humble bash`). The container has root access to the port and `pyserial`. The Jetson's own `nvidia` user isn't in `dialout`.

## 0. Before you start

- [ ] Robot on a stand with **both wheels off the ground**, from here until the last step. Nothing touching the wheels.
- [ ] Someone at the robot, with a way to cut motor power.
- [ ] Decide which way is **forward**: the way the robot drives with the caster ahead of the axle.

## 1. Stop the bridge

Only one program can use the serial port, and if the bridge holds it during a flash, the board can come back as `ttyACM1`.

**Jetson:**

```bash
docker exec auto_ros_humble pkill -f diffdrive_node
sudo fuser -v /dev/ttyACM0
```

- [ ] `fuser` lists nothing. If you started the stack with `./robot.sh robot`, run `./robot.sh stop` as well.

The bridge running on 2026-11-07 was started by hand as `/tmp/diffdrive_node_patched.py`, because the Jetson's checkout is older than the PC's and lacks the first-reading baseline fix. Step 9 restarts it.

## 2. Back up the current firmware

The firmware on the board was never committed, so this backup is the only copy. It may hold Wi-Fi details, so keep it out of git.

**Jetson, container:**

```bash
pip3 install -q --target /tmp/esptool/pip pip
PYTHONPATH=/tmp/esptool/pip python3 -m pip install -q --no-deps --target /tmp/esptool/lib 'esptool==4.9.0' intelhex
mkdir -p /tmp/motor-fw && cd /tmp/motor-fw
export PYTHONPATH=/tmp/esptool/lib
python3 -m esptool --chip esp32s3 --port /dev/ttyACM0 --baud 460800 flash_id
python3 -m esptool --chip esp32s3 --port /dev/ttyACM0 --baud 460800 read_flash 0 ALL backup-old-firmware.bin
python3 -m esptool --chip esp32s3 --port /dev/ttyACM0 --baud 460800 verify_flash 0 backup-old-firmware.bin
```

The container's own pip can't build esptool, so the first line gets a newer one. `--target` keeps both out of the container's ROS Python. These two install lines were tried in the container on 2026-11-07, without opening the port. A new shell needs that `export` again.

- [ ] `flash_id` says `8MB` or more. This build assumes 8 MB of flash, so stop here if it's less.
- [ ] `verify_flash` ends with `verify OK (digest matched)`. The backup's size is the flash size `flash_id` printed.
- [ ] Copy it off the robot. **Jetson**, then **PC**:

```bash
docker cp auto_ros_humble:/tmp/motor-fw/backup-old-firmware.bin ~/backup-old-firmware.bin
```

```bash
scp jetson:backup-old-firmware.bin ~/backup-old-firmware.bin
```

These commands reset the board, so its encoder counts restart from 0. It boots back into the old firmware.

If esptool can't connect: hold BOOT, tap RESET, release BOOT, and repeat the command with `--before no_reset` added after `--baud 460800`. Tap RESET again when it's finished.

**To go back at any time** (**Jetson, container**, with the bridge stopped):

```bash
PYTHONPATH=/tmp/esptool/lib python3 -m esptool --chip esp32s3 --port /dev/ttyACM0 --baud 460800 write_flash 0 /tmp/motor-fw/backup-old-firmware.bin
```

## 3. Flash

`flash_bundle/` holds the built files. It's ignored by git. If it isn't there or is out of date, rebuild it on this branch. That needs PlatformIO, from its VS Code extension or `pip install platformio`, and about 1.5 GB of downloads the first time. **PC**, in `MOTOR-ESP32S3`:

```bash
pio run && tools/make_flash_bundle.sh
```

Copy it to the robot. **PC**, in `MOTOR-ESP32S3`:

```bash
ssh jetson mkdir -p motor-fw && scp flash_bundle/* jetson:motor-fw/
```

**Jetson:**

```bash
docker cp ~/motor-fw/. auto_ros_humble:/tmp/motor-fw/
```

**Jetson, container:**

```bash
cd /tmp/motor-fw && sha256sum -c SHA256SUMS
PYTHONPATH=/tmp/esptool/lib python3 -m esptool --chip esp32s3 --port /dev/ttyACM0 --baud 460800 write_flash -z 0x0 bootloader.bin 0x8000 partitions.bin 0xe000 boot_app0.bin 0x10000 firmware.bin
```

- [ ] Ends with `Hard resetting`. If the LED stays off, press the board's reset button.
- [ ] The port is back: `ls /dev/ttyACM*` shows `ttyACM0`, on the Jetson and in the container. If it's only on the Jetson, restart the container.

## 4. First boot: no motion

**Jetson, container:**

```bash
python3 /tmp/motor-fw/wheel_test.py watch --seconds 10
```

- [ ] `e` and `i` both read about `50/s`. This shows the same streams the bridge reads. The rate isn't exact in the first line.
- [ ] LED is green. Red means the IMU wasn't found, and `i` shows `0/s`: check GPIO 1 and 21.
- [ ] The wheels don't move.

See the raw lines and the boot line. This resets the board through the USB control lines, the way esptool does. The motors stay off:

```bash
python3 /tmp/motor-fw/wheel_test.py lines --reset --seconds 2 | awk 'NR<=4 || /READY/ || /Lost/'
```

- [ ] Lines look like `e 0 0` and `i 0.20 0.14 -9.94 -0.01 0.01 0.01`, the same as before.
- [ ] `Boot line with READY seen: yes`. If the USB link dropped instead and it says `Lost the serial port`, that isn't a failure: the bridge reconnects and takes a new baseline when that happens. Check the port is back as `ttyACM0`.

## 5. Encoder wiring and counts per turn: by hand, no motor commands

This also settles what the 2026-11-06 hand test couldn't: whether the counts reverse when a wheel does.

**Jetson, container:**

```bash
python3 /tmp/motor-fw/wheel_test.py turns | tee /tmp/motor-fw/turns.log
```

Follow the prompts: mark each wheel, turn it 10 full turns forward, then 10 back.

- [ ] Both wheels report `Direction OK`.
- [ ] Left forward counts up, right forward counts down. The bridge negates the right wheel, so that's correct.
- [ ] The two wheels' counts per turn are within about 5%. It prints the number to use.

What it might say instead:

| Message | What to do |
| :--- | :--- |
| `GPIO n never changed` | Dead line. With the power off, check that wire at both ends. Quadrature can't count without it, so that wheel's count stays near 0. Then measure the line while turning the wheel slowly: it should swing between about 0 V and at most 3.3 V. If it goes to 5 V, stop and fix that first, because the ESP32-S3's inputs aren't rated for it. |
| `Activity on old left pins` | A left encoder wire is on GPIO 13 or 17. Move it to 18 (A) or 14 (B), or change `LEFT_ENCODER_A/B` in `config.h`. |
| `Counts run the wrong way` | Flip the named `*_ENCODER_SIGN` in `config.h`, and rebuild. |
| `The count did not reverse` | A and B aren't both working as a pair. Check the wires, then repeat. |

If an encoder can't be fixed now, go on. That wheel will run on feed-forward alone, and `/odom` will be wrong for it. Use `hold` in step 7 to measure its speed by eye.

Then watch for the left channel's unexplained jumps:

```bash
python3 /tmp/motor-fw/wheel_test.py watch --seconds 300
```

- [ ] At rest, for 5 minutes, neither count moves by more than a few. Then wiggle the encoder wires gently: no jumps of hundreds or thousands.

Tell the board the number from `turns` until it's in `config.h`, because step 7's speeds use it. A `g` setting lasts until the next reboot, and opening the port doesn't reboot the board:

```bash
python3 /tmp/motor-fw/wheel_test.py send "g cpr NUMBER_FROM_TURNS"
```

## 6. First drive: direction and timeout

Each drive command asks you to confirm the wheels are off the ground. The first PWM values are estimates, so a wheel may jerk or spin faster than asked.

**Jetson, container:**

```bash
python3 /tmp/motor-fw/wheel_test.py direction | tee /tmp/motor-fw/direction.log
```

- [ ] Each wheel turns **forward** when driven alone, for 2 s, then stops. If not, swap the two numbers it names in `config.h` (`*_MOTOR_IN1`, `*_MOTOR_IN2`), rebuild and reflash. Don't change the sign in the bridge.
- [ ] It reports `Count direction OK` for each wheel. If it says to flip a `*_ENCODER_SIGN`, do that, and rebuild and reflash.
- [ ] LED was blue while driving and green after. Orange or a `board says` line means that wheel fell back to feed-forward alone: see "If something goes wrong" at the end.

A reflash or reset loses `g` settings. If you reflash here, put `ENCODER_CPR` in `config.h` at the same time, or send it again.

```bash
python3 /tmp/motor-fw/wheel_test.py timeout
```

- [ ] Prints a last count change about 0.5 to 0.8 s after the last command, and `pwm=0` on both. **Over about 1 s is a failure: don't go on.**

## 7. Measure and set the gains

```bash
python3 /tmp/motor-fw/wheel_test.py ff | tee /tmp/motor-fw/ff.log
```

This sweeps raw PWM on each wheel and prints deadband, slope and kick values. It goes up to about 300 above where the wheel starts, so it spins quite fast. If the encoder shows nothing, it gives up at PWM 600 for that wheel.

- [ ] Try them now: run the `send` line it prints. Or put the `config.h` lines it prints straight into step 8, and flash before going on.

```bash
python3 /tmp/motor-fw/wheel_test.py speed --open-loop | tee /tmp/motor-fw/speed-open.log
```

```bash
python3 /tmp/motor-fw/wheel_test.py speed | tee /tmp/motor-fw/speed-closed.log
```

Each takes about 2 minutes. `speed` also runs an arc (left 0.05, right 0.20 m/s) and the stop rule (0.005 m/s).

- [ ] Closed loop: `ratio` within about 0.85 to 1.15 from 0.02 to 0.25 m/s, both directions, both wheels, with `fault` `none`. Different requests give clearly different speeds.
- [ ] The arc's two wheel speeds differ by about 4 to 1.
- [ ] 0.005 m/s moves neither wheel.
- [ ] Open loop: ratios may be further off but should rise evenly. A wheel whose encoder has failed behaves about like this, except that it gets no further kicks if it stalls, so it may stop at the lowest speeds.
- [ ] No hunting: no audible pulsing at a steady request. If a wheel hunts at low speed, halve `K_P` and `K_I`.

If `trim` sits near its limit (250) or `fault` says `trim_sat`, the feed-forward is far off. Redo `ff`.

If an encoder is dead on a wheel, `ff` and `speed` can't measure it. Count turns of the mark by eye instead, for several speeds. Forward is positive, and `--pwm` holds a raw PWM instead of a speed:

```bash
python3 /tmp/motor-fw/wheel_test.py hold 0.10 0.10 --seconds 20
```

One wheel turn is 0.2136 m. Fit deadband and slope to PWM against speed by hand, with `hold --pwm`.

- [ ] Keep the logs: **Jetson**, `docker cp auto_ros_humble:/tmp/motor-fw/ ~/motor-fw-logs/`. They're the first measurements of these motors.

## 8. Put the numbers in the firmware

**PC**, in `include/config.h`, in the block marked "SET THESE FROM A MEASUREMENT":

- [ ] `ENCODER_CPR` from step 5
- [ ] `FF_DEADBAND_*`, `FF_SLOPE_*`, `KICK_PWM_*` from step 7
- [ ] Rebuild and flash as in step 3
- [ ] Commit `config.h`. These are the first measured values, so note the battery voltage in the commit message: the slope depends on it.

## 9. `encoder_cpr` in autoJetsonBot, and back to ROS

The old firmware counted one edge of channel A. This one counts all 4 edges of A and B, so the count per turn is **4 times as high**: from the hand test's 344 to 387, expect about **1,376 to 1,546**. (1,496 would fit a 34:1 gearbox with 11 pulses per motor turn, but the gearbox isn't documented.) **Use the number from step 5**, not an estimate. Not 3,436, which may have been 10 turns recorded as one, and not 4 x 3,436. If the bridge keeps 3,436, `/odom` will read 0.43 of real distance.

- [ ] In `autoJetsonBot`, set it in both places, and commit:
  - `src/jetson_bot_bringup/launch/main.launch.py`, the `'encoder_cpr': 3436` in the bridge's parameters
  - `src/jetson_bot_diffdrive/jetson_bot_diffdrive/diffdrive_node.py`, the default in `declare_parameter('encoder_cpr', 3436)`
- [ ] `GEMINI.md` also says 3436, for documentation only.
- [ ] Check nothing else publishes `/cmd_vel` before the bridge starts, because from now on it can drive the wheels. Test runs in the `jbl_baseline` container use their own domain, but `test_safety_gate_launch.py` keeps one that's already set.

**Jetson**, restart the bridge. By hand, as it was, with your number:

```bash
docker exec -d auto_ros_humble bash -c '. /opt/ros/humble/setup.bash; exec python3 /tmp/diffdrive_node_patched.py --ros-args -r __node:=diffdrive_bridge -p port:=/dev/ttyACM0 -p baud:=115200 -p wheel_separation:=0.212 -p wheel_radius:=0.034 -p encoder_cpr:=NUMBER_FROM_TURNS -p publish_tf:=false'
```

**Jetson, container**, with the wheels still off the ground:

```bash
. /opt/ros/humble/setup.bash
cat /tmp/esp32_status.json
timeout 5 ros2 topic hz /serial_imu_raw
ros2 topic info /cmd_vel
```

- [ ] `enc_ok` and `imu_ok` rise, and the last lines look right. `/serial_imu_raw` runs at about 50 Hz. `/cmd_vel` has 0 publishers.

In one terminal, watch odometry. Use the position, not the speed: see the note below.

```bash
ros2 topic echo /odom --field pose.pose
```

In a second, drive forward for a few seconds, then Ctrl-C. The wheels stop within 0.5 s of that:

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```

- [ ] Both wheels turn forward. `position.x` rises by about 0.1 a second, and `orientation.z` stays about 0.

Then turn left:

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"
```

- [ ] Left wheel back, right wheel forward. `orientation.z` rises steadily from where it was.

**Expect spikes in `/odom`'s speed while driving, from the bridge rather than this firmware.** This comes from reading its code, because nothing has moved under it yet. When two `e` lines arrive in one 50 Hz bridge tick, `diffdrive_node.py` divides the second one's distance by a tiny time. The EKF fuses that speed, and `dock_judge` uses it to decide "stopped", so fix it in the bridge before trusting them.

## 10. On the floor

Only after all of the above. Stop the bridge again as in step 1, keep each run short, clear about 1 m ahead, and keep a hand near the motor power. Set the values for the floor: it takes more PWM than the air, and the closed loop is slow to find it at low speed.

**Jetson, container:**

```bash
python3 /tmp/motor-fw/wheel_test.py hold 0.03 0.03 --seconds 4 --on-floor
```

```bash
python3 /tmp/motor-fw/wheel_test.py hold 0.15 0.15 --seconds 3 --on-floor
```

Each prints the `trim` each wheel ended on, positive if the wheel needed more than the feed-forward gave.

- [ ] Both wheels started, with `fault` `none`, and the robot went about straight. If a wheel didn't start, raise its kick: `send "g kick_l 650"` or `kick_r`.
- [ ] For each wheel, add (trim at 0.15 - trim at 0.03) / 0.12 to `FF_SLOPE_*`, call that increase S, and add (trim at 0.03 - 0.03 x S) to `FF_DEADBAND_*`. Try them with `send "g dead_l ..." "g slope_l ..."`, repeat until the trims are small, then put them in `config.h` as in step 8.
- [ ] If starts are too jerky at low speed, lower `KICK_MS` first, then the kick. Too low and a wheel stays stopped.

## If something goes wrong

| What you see | What it means |
| :--- | :--- |
| Orange LED, `fault=no_counts`, wheel didn't move | The kick didn't start it, or there's no motor power, or it's blocked. It gets no more kicks until the request next goes under 0.01 m/s. |
| Orange LED, `fault=no_counts`, wheel turning | A dead encoder line. Run `turns`. The wheel keeps running on feed-forward alone. |
| `fault=wrong_dir` | The count runs against the request. Run `direction`. |
| `fault=implausible` | Encoder noise. Check the wires, and how they run near the motor leads. |
| `fault=trim_sat` | The closed loop couldn't reach the speed with `TRIM_MAX_PWM` more power. The feed-forward is too low, or counts are being lost. Redo `ff`. |
| Red LED | The IMU isn't answering. The rest keeps working, and it retries every 2 s while idle. `send d` shows `imu_bad_reads`. |
| Board silent, no `e` lines, motors off, after a brownout or crash | It may be waiting in download mode, because the bridge and this tool hold DTR high (from the esptool source and memory of the data sheet, not tried). Stop whatever has the port open and press RESET. |
| A wheel keeps turning after a crash | From reading the restart code: a crash doesn't reset the PWM hardware, so it holds until the firmware restarts, an estimated 1 s. That's not measured, and no crash is known. Cut the motor power if it lasts. |

## What this firmware changes from the old one

| | Old (`MOTOR-ESP32S3` main plus IMU, as flashed) | This branch |
| :--- | :--- | :--- |
| Power for 0.01 to 0.25 m/s | 500/1023 for all of it | Deadband + slope x speed, plus a closed-loop trim |
| Start from rest | Same 500 | Short kick, then the above |
| Encoder decoding | Rising edge of A only, B read for direction | All 4 edges of A and B, in hardware, with a 10 us glitch filter |
| Counts per turn | About 344 to 387 | About 4 times that |
| `m` parsing | Blocking `parseFloat` | Whole lines, malformed ones ignored |
| Command timeout | Checked in `loop()` | Checked in the control task, independent of serial |
| `f` test | Blocks everything for 2 s | Runs alongside telemetry, and any `m` ends it |

The formats of `e`, `i` and `m`, the `READY` boot line and the sign convention are unchanged. The old firmware's source was never committed, so its column comes from the repo's `main` and June's notes.
