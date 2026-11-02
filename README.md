# Firmware for Neoden YY1 Pick-And-Place machine

This is a conversion kit for your YY1 to make it OpenPnP-compatible.

This fork of [mdepx/neodenyy1](https://github.com/mdepx/neodenyy1) adds homing on
`G28`, feed rate control, error reporting, more actuators (peel banks, LED rings,
vibrator) and power management for the nozzle rotation drivers. See
[SETUP.md](SETUP.md) for the first power-up and the OpenPnP configuration.

Project status: working, managed to assemble a few boards with no issues.

| part              | status  |  notes |
| ----------------- | ------- | ------ |
| Motion controller | functional              | `G28` homing, `F` speed control and error reporting; motion is per axis, not coordinated |
| Vision            | functional              | openpnp's vision pipeline needs tuning |
| LED ring bottom   | firmware done, HW prototyping in progress | on/off only |
| LED ring top      | firmware done, HW prototyping in progress | on/off only |
| Nozzle changer    | no firmware needed      | mechanical rack, an OpenPnP nozzle tip changer sequence |
| Vibrator          | functional              | `M800 S`, for stick / bulk feeders |

Note that the information below is not complete, I am keep updating it.

### Hardware overview.

YY1 features several electronics parts: main board (motion controller), 3x displays and camera boards. All parts are connected to each other using either UART or SPI, there is no USB in entire system.

Main board is build around STM32F407VET6 micro-controller and LV8729 steppers.

The vision is done on the edge, i.e. directly on the PCBs the camera sensors are located on. Each features STM32H7. The camera sensor handled by DCMI (Digital Camera Interface) stm32's peripheral device. Firmware on both cameras are identical. The result of machine vision is then passed over low speed serial interface to the main board. The stm32 pins exposed to connectors on the camera modules could not be remapped to USB.

Each of camera module is connected to its display for HMI over low speed as well.

Main display is connected to the main board over UART.
We will use this UART for both communication to OpenPnP over GCode (input/output) and developer console to the stm32f407 (output only).

![Stepper board](https://raw.githubusercontent.com/mdepx/neodenyy1/master/images/stepper_board.jpg)

### What you need to do

 1) Replace both cameras (for OpenPnP we need USB cameras)
 2) Update firmware
 3) Setup OpenPnP

### What this project gives to you

 1) New firmware for motion controller written from scratch. The firmware uses real-time operating system [MDEPX](https://github.com/mdepx/mdepx) that I wrote from scratch as well.
 2) CAD models so you can assemble your own USB camera modules and LED rings (TODO)
 3) No warranty of any kind. Do that on your own risk!

### Debug tools

Locate SWD pins (Data / Clock / Ground) on the stepper board and solder down 2.54mm header.

Any standard Cortex SWD debugger should work with those pins. Instructions below are for [Debug Probe](https://www.raspberrypi.com/products/debug-probe/).

### Download and install OpenOCD

I use [OpenOCD fork from RPI](https://github.com/raspberrypi/openocd.git).
(Clone and build using instructions provided at the link)

### Build this firmware under Linux/FreeBSD
    $ export CROSS_COMPILE=arm-none-eabi-
    $ git clone --recursive https://github.com/Quis03/neodenyy1
    $ cd neodenyy1
    $ make

Note there is no support for build under Windows.

### Read and backup your current firmware

Replace tcl path to your openocd installation directory

    $ sudo openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg \
        -s /home/br/dev/openocd-rpi/tcl -c "adapter speed 5000" \
        -c 'cmsis_dap_vid_pid 0x2e8a 0x000c' -c init -c "reset halt" \
        -c 'flash read_bank 0 firmware_YY1.bin 0 0x80000' \
        -c "reset" -c shutdown

### Program firmware
    $ sudo openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg \
        -s /home/br/dev/openocd-rpi/tcl -c "adapter speed 5000" \
        -c 'cmsis_dap_vid_pid 0x2e8a 0x000c' -c init -c "reset halt" \
        -c 'program obj/neodenyy1.bin reset 0x08000000 exit'

### Operation

The firmware homes the machine on startup, and `G28` homes it again on demand. If
homing fails the board still enters the GCode loop, and moves are refused with
`error: machine not homed` until a `G28` succeeds.

Note that this firmware converts linear motion of Z coordinate into rotational. When you setup Z axis in the OpenPnP use ReferenceControllerAxis (linear motion).

### GCode interface

The console is the main display UART, 115200 8N1. Lines end with LF (CRLF is
accepted), `;` starts a comment, and lines longer than 256 characters are rejected.

| command | action |
| ------- | ------ |
| `G0 X Y Z I J F` | linear move; `I` / `J` are the nozzle rotations, `F` is mm/min |
| `G28` | home every axis |
| `M114` | position, replies `ok X:.. Y:.. Z:.. I:.. J:..` |
| `M105 N1` / `M105 N2` | part on nozzle 1 / 2, replies `ok V:<0/1>` / `ok W:<0/1>` |
| `M800 <target><0/1> ...` | actuators, several targets per line allowed |

| `M800` target | actuator | pin |
| ------------- | -------- | --- |
| `P` | pump | PB13 |
| `V` | vacuum valve, nozzle 1 | PE2 |
| `W` | vacuum valve, nozzle 2 | PE1 |
| `D` | needle | PE0 |
| `L` | peel, left bank | PB12 |
| `R` | peel, right bank | PB11 |
| `S` | vibrator, `Q<ms>` switches it off again after that time | PA3 |
| `T` | top (down-looking) camera LED ring | PD5 |
| `B` | bottom (up-looking) camera LED ring | PC10 |
| `E` | nozzle rotation driver power, see below | PD12, PD3, PA15 |

Every command answers `COMPLETE` on success, or a single `error: <text>` line and no
`COMPLETE` on failure. A multi axis `G0` is range checked on every axis before any of
them moves.

`F` is applied per axis, so a diagonal is not a straight line and the vector speed
can be up to sqrt(2) times `F`. Without `F` a move runs at about 14000 mm/min on X
and Y. `F` is ignored for `I` and `J`.

#### Nozzle rotation power

Left energised, the nozzle rotation drivers overheat the head, so they are only
powered while one of these holds:

 - a vacuum valve is open (`M800 V1` / `W1`),
 - a rotation was commanded (`G0 I` / `J`), until `G28`, `M800 E0` or one minute of
   idle console,
 - `M800 E1` was sent, until `M800 E0` or five minutes of idle console.

`E1` is what keeps the angle while the nozzle is still down on a part after the valve
closed. The OpenPnP scripts in [openpnp/scripts](openpnp/scripts) send it before
every place and release it once both nozzles are free.

The rotations have no home switch, so an angle lost while the drivers are off is
never recovered. If the shafts do not keep their angle across a power cycle on your
machine, hold `M800 E1` for the whole job instead.

### LED rings

The LED outputs are push-pull GPIOs, active high and low at reset. Do not drive a
ring straight off the pin: use it as the gate of a low-side N-channel MOSFET and feed
the ring from the machine supply. There is no PWM on PC10 or PD5, so brightness is
set in hardware.

### Camera modules

You need these parts

    - M2 standoff round spacers https://www.aliexpress.com/item/4001270070683.html
    - M3 standoff hex spacers https://www.aliexpress.com/item/4001242054845.html
    - Wera 118124 Kraftform Micro Nut Spinner 5mm (for M3 standoffs)
    - Sony IMX415 Top and Bottom cameras https://www.aliexpress.com/item/1005005481109987.html
    - Some M12 lens on your choice (I'm still experimenting with different)

### Contribution

Any improvements are welcome! Note that your patches and pull-requests (if any) use FreeBSD's [style(9) guide](https://man.freebsd.org/cgi/man.cgi?style(9)) and BSD 2-clause license.

### LQFP128 vision result

![LQFP](https://raw.githubusercontent.com/mdepx/neodenyy1/master/images/lqfp_vision.gif)

### Help

If you are seeking for help for this project or need firmware for another PnP / other types of equipment, then you can reach me using br@bsdpad.com.
Also my telegram channel [@machinedependent](https://t.me/machinedependent) and the chat linked to it are available.

![NeoDen YY1](https://raw.githubusercontent.com/mdepx/neodenyy1/master/images/neodenyy1.jpg)
