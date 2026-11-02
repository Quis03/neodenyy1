# Setup

First power-up of an assembled machine and the OpenPnP configuration for this
firmware.

Before you start:

 - Back up the stock firmware (see README) and keep two copies, 524288 bytes each.
 - Flash a binary built from this tree with `make`.
 - Open the serial console on the main display UART, 115200 8N1.

> **There is no E-stop.** `M112` is not implemented and a move in progress cannot be
> interrupted. Keep a hand on the power switch until the machine is configured.

## First power-up

Do not connect OpenPnP yet, and keep the nozzles clear: homing does Z first, and the
Z search swings the cam back and forth over the whole stroke if it starts far from
home.

The firmware waits 2 seconds and homes on its own. `M114` should then read all zeros.
If homing fails you get an `error:` line naming the axis; fix it and send `G28`.

Y=0 is at the end opposite the Y home switch: homing references against the switch,
then moves the origin to the far end and inverts the direction.

### Re-homing

Start from the middle of the bed, so that a wrong direction cannot hit the stop:

    G0 Y180 F1000
    G28

Y must travel toward the home switch. Send `G28` a couple more times and check that
each one ends with `COMPLETE` and `M114` reads zero.

### Directions

Move slowly, `F600` is about 4 mm/s.

    G0 X50 F600
    G0 Y50 F600
    G0 Z2 F300
    G0 Z0 F300
    G0 I90 F600
    G0 I0 F600

On the reference machine X grows to the right, Y toward the back, and **nozzle 2 goes
down for positive Z**. Z is one cam shared by the two nozzles: one goes down as the
other goes up. The firmware allows about ±20.4 mm, but find the depth where the
nozzle touches the bed in 1 mm steps and use that, minus a margin, as the soft limit.

`G0 X400` must answer `error: X out of range` without moving.

### Actuators

    M800 P1 V1     pump and nozzle 1 vacuum
    M105 N1        ok V:1 with a part held on nozzle 1
    M800 P0 V0
    M800 D1        needle down
    M800 D0        needle up
    M800 L1        left peel bank, then L0, R1, R0
    M800 S1 Q400   400 ms of vibrator
    M800 T1 B1     both LED rings, then T0 B0

Check that the nozzle rotation power works: after `G0 I0` nozzle 1 resists by hand,
after `M800 E0` it turns freely. Mark a nozzle, rotate it to an angle that is not a
full step (`G0 I1`), drop the power with `M800 E0` and command the same angle again a
few times: the mark must not move. If it does, hold `M800 E1` for the whole job.

### Feed rate

Time a few long moves (`G0 X0 F600`, `G0 X300 F1200`, ...), then increase `F` and
send `G28` after each one: where `M114` and the real position stop agreeing, the
steppers are losing steps. Use 70% of that as Max Feed Rate. Y moves the whole gantry
and gives up first; 4000 mm/min was the limit on the reference machine. Start at
3000.

## OpenPnP configuration

Issues and Solutions cannot auto-configure this firmware, everything is manual.

### Driver

`GcodeDriver`, serial, 115200, Millimeters.

| setting | value |
| ------- | ----- |
| `COMMAND_CONFIRM_REGEX` | `^COMPLETE.*` |
| `COMMAND_ERROR_REGEX` | `^error:.*` |
| `HOME_COMMAND` | `G28` |
| `MOVE_TO_COMMAND` | `G0 {X:X%.4f} {Y:Y%.4f} {Z:Z%.4f} {I:I%.4f} {J:J%.4f} {FeedRate:F%.0f}` |
| `GET_POSITION_COMMAND` | `M114` |
| `POSITION_REPORT_REGEX` | `^ok X:(?<X>-?\d+\.\d+) Y:(?<Y>-?\d+\.\d+) Z:(?<Z>-?\d+\.\d+) I:(?<I>-?\d+\.\d+) J:(?<J>-?\d+\.\d+).*` |
| `DISABLE_COMMAND` | `M800 E0` |
| `CONNECT_COMMAND`, `ENABLE_COMMAND`, `MOVE_TO_COMPLETE_COMMAND` | empty |

Do not leave `COMMAND_CONFIRM_REGEX` at `^ok.*`: `ok` is the prefix of the `M105` and
`M114` replies, not a confirmation.

### Axes

```
ReferenceControllerAxis  X   letter X   soft limits 0 … 364 mm
ReferenceControllerAxis  Y   letter Y   soft limits 0 … 368 mm
ReferenceControllerAxis  Z   letter Z   soft limits from the first power-up
ReferenceControllerAxis  C1  letter I   -180 … +180 deg
ReferenceControllerAxis  C2  letter J   -180 … +180 deg

ReferenceMappedAxis  Z_N1: input Z, A=(0,0), B=(1,+1)  -> nozzle 1
ReferenceMappedAxis  Z_N2: input Z, A=(0,0), B=(1,-1)  -> nozzle 2
```

The nozzle that goes down for positive Z gets the inverted mapping. Z must be a plain
`ReferenceControllerAxis`: the firmware already converts millimetres into cam degrees.

### Actuators

| actuator | `ACTUATE_BOOLEAN_COMMAND` |
| -------- | ------------------------- |
| `PUMP` | `M800 P{True:1}{False:0}` |
| `VAC1` | `M800 V{True:1}{False:0}` |
| `VAC2` | `M800 W{True:1}{False:0}` |
| `NEEDLE` | `M800 D{True:1}{False:0}` |
| `PEEL_L` | `M800 L{True:1}{False:0}` |
| `PEEL_R` | `M800 R{True:1}{False:0}` |
| `VIBRATOR` | `M800 S{True:1}{False:0} Q400` |
| `LED_TOP` | `M800 T{True:1}{False:0}` |
| `LED_BOTTOM` | `M800 B{True:1}{False:0}` |
| `HOLD_ROT` | `M800 E{True:1}{False:0}` |

| actuator | `ACTUATOR_READ_COMMAND` | `ACTUATOR_READ_REGEX` |
| -------- | ----------------------- | --------------------- |
| `PART_SENSE_1` | `M105 N1` | `^ok V:(?<Value>-?\d+).*` |
| `PART_SENSE_2` | `M105 N2` | `^ok W:(?<Value>-?\d+).*` |

The part sensors are digital: use them as part-on / part-off with 0/1 thresholds.

`HOLD_ROT` goes on the head, next to `PUMP` and the valves. For it and the two LED
actuators set the machine states to Enabled `ActuateOFF`, Homed `LeaveAsIs`, Disabled
`ActuateOFF`.

### Scripts

Copy [openpnp/scripts](openpnp/scripts) into `~/.openpnp2/scripts/`:

 - `Nozzle.BeforePlace.js` turns `HOLD_ROT` on before every place. OpenPnP closes the
   valve while the nozzle is still down on the part, and without power the shaft
   would turn with the part on the way up.
 - `Job.Placement.Complete.js` turns it off once both nozzles are free, after
   rotating them back to 0°, which is on a full-step detent.

### Cameras

Both are `OpenPnpCaptureCamera`. Set the light actuator under **Cameras -> \<camera\>
-> Camera Settings -> Light Actuator**: `LED_TOP` for the head camera, `LED_BOTTOM`
for the bed one.

Then focus, exposure, lens calibration, bottom camera calibration and the vision
pipelines for your packages.

### Nozzle tips and the changer

The rack is mechanical, so a nozzle change is only an OpenPnP sequence:
`ReferenceNozzleTip` -> **Nozzle Tip Changer**, with the three Load and Unload
locations taught on the machine. Use a low feed rate for the changer moves.

### Feeders

The tape feeders are driven by the needle and the head's own movement:
`ReferenceDragFeeder` or `ReferencePushPullFeeder` with `actuatorName` `NEEDLE` and
`peelOffActuatorName` `PEEL_L` or `PEEL_R` for the bank. Push-pull handles 2 mm pitch
tape (0402 / 0201), which only advances every other feed.

For stick and bulk parts use a `ReferenceLoosePartFeeder` with the `VIBRATOR`
actuator.

## Placement accuracy

 - Set the board Z to the height where the bare nozzle tip touches the board, measured
   by jogging. Every 0.1 mm lower pushes the part into the board with the spring.
 - Measure that height with both nozzles. If they differ, put the difference in the
   nozzle `head-offsets` Z, not in the board.
 - The Z axis is not perfectly vertical: the nozzle moves about 0.04 mm in X/Y per mm
   of Z, the same way on both nozzles. Focus the bottom camera at the placement
   height, so that alignment and placement happen at the same Z, then recalibrate
   units per pixel, the nozzle runout and the vision pipelines.

## Known limitations

 - No coordinated motion: each axis runs at `F` on its own, and Z moves after X and Y.
 - No gantry squaring: both Y motors share one timer.
 - The nozzle rotations have no home switch. `G28` returns them to the counted zero,
   but a lost step is never recovered.
 - Fixed actuator delays: 250 ms for pump, valves and peel, 150 ms for the needle.
 - No E-stop.

## Rollback

    openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg \
        -c "adapter speed 5000" \
        -c "program firmware_YY1.bin reset 0x08000000 exit"

and refit the original camera modules.
