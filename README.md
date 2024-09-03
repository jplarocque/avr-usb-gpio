AVR USB GPIO
============

Firmware for making USB GPIO adapters with AVR microcontrollers, built upon the
[V-USB library](https://www.obdev.at/products/vusb/index.html).

Also includes a Linux driver, a Debian DKMS package, and accompanying udev
rules for nice symlinks.

Building a Board
================

Connect one of the supported microcontrollers (see below) to USB D- and D+
lines.  Use the AVR I/O pins listed for your AVR model below.

Use a fixed 1.5 kΩ pull-up resistor for D-.

Refer to the
[V-USB wiki Hardware Considerations article](http://vusb.wikidot.com/hardware)
for examples of how to connect I/O lines.  You should use the voltage clamping
variant (called "level conversion" in the wiki article).  If you use the
"reduced supply voltage" option, you'll need to customize the fuse values in
`usbconfig.h` to adjust the Brown-Out Detector threshold for 3.3 V operation.
(Currently there's not a good process of customizing fuse values, besides just
tweaking them in source code for a one-off copy/fork of this project.)

Building Firmware
=================

The philosophy of this project is to provide firmware you can use to make your
own board using a supported model of AVR microcontroller.  There are some fixed
design decisions, like which I/O lines are used for USB D- and D+, and that a
crystal oscillator is to be used.  However, the firmware is flexible in your
choice of oscillator frequency, supporting all the options provided by V-USB:

  - 12.0 MHz
  - 12.8 MHz
  - 15.0 MHz
  - 16.0 MHz
  - 16.5 MHz
  - 18.0 MHz
  - 20.0 MHz

Because of this flexibility, when running `make`, you should always specify
your microcontroller model with `DEVICE=...`, and your crystal oscillator
frequency with `F_CPU=...`.  Some example `make` commands are given for each
supported AVR model below, but you are free to (and must) adjust the frequency
to match your crystal.

RC Oscillator Support
---------------------

While V-USB supports the AVR's internal RC oscillator (only at 12.8 MHz and
16.5 MHz), you'd have to extend this project with runtime oscillator
calibration support.  This shouldn't be too difficult if you refer to the
[V-USB examples](http://vusb.wikidot.com/examples).  If you choose to do this,
you'll probably also want to free up the pins which were previously reserved
for the crystal to make them available as GPIO lines to the USB host.

Supported AVRs
==============

ATmega8
-------

Example command to build and flash:

```sh
cd firmware
make DEVICE=atmega8 F_CPU=16000000 flash
```

| AVR pin       | Function                          |
| ------------- | --------------------------------- |
| PB6, PB7      | Crystal oscillator                |
| PD2           | USB D+ (chosen for INT0 function) |
| PD4           | USB D-                            |
| PC6           | /Reset                            |
| 18 other I/Os | Available for GPIO over USB       |

ATtiny24, ATtiny44, ATtiny84, ATtiny24A, ATtiny44A, ATtiny84A
-------------------------------------------------------------

Example command to build and flash:

```sh
cd firmware
make DEVICE=attiny24a F_CPU=16000000 flash
```

| AVR pin      | Function                    |
| ------------ | --------------------------- |
| PB0, PB1     | Crystal oscillator          |
| PA0          | USB D+                      |
| PA1          | USB D-                      |
| PB3          | /Reset                      |
| 7 other I/Os | Available for GPIO over USB |

Other AVR Models
----------------

It should be straightforward to add support for most other AVR microcontroller
models.

The model will need at least 2 KiB flash program storage, or 4 KiB if you want
to use V-USB's CRC checking support (18 MHz crystal only).

Currently AVRs which define a `PUEx` I/O port register for enabling pull-ups
are *not* supported.  Per the `avr/io*.h` headers in avr-libc 2.0.0, this
includes ATtiny4/5/9/10, ATtiny441/841, ATtiny828, and ATtiny1634.

Choose which pins to use for USB D- and D+.  Ideally try to put D+ on INT0 or
INT1, but since no other interrupts are used in the firmware, resorting to a
pin-change interrupt is fine (as is done for ATtiny24/44/84[A]).  I think V-USB
also making D- trigger the interrupt rather than D+, but I haven't tested this.

Edit `usbconfig.h` to add support for the model using C preprocessor
conditionals.  Refer to the existing code to see how to specify:

  - Which I/O ports your new AVR model has, using the `DEFPORTREGS()` and
    `DEFPORTLINECOUNT()` macros.
  
  - Which pins on those ports are allowed to be used as GPIO by the USB host,
    using the `DEFPORTVALID()` macro.
  
  - The fuse bits appropriate for your AVR model, using the `DEFFUSES()` macro.

Linux Driver
============

Module name: `gpio-avr-usb`  
Minimum required kernel version: 6.8  
Minimum recommended: 6.9

Manual Build
------------

Example to build for the currently-running kernel version:

```sh
cd linux
make modules
sudo make modules_install
```

DKMS Package
------------

To arrange for your system to automatically build the kernel module for any
kernel version you install in the future (i.e. for kernel updates), you can use
the included DKMS configuration.

For Debian and Debian-based distributions, this is made easy with the included
Debian source package `avr-usb-gpio-dkms`.  First, make sure you have an
appropriate `linux-headers-*` metapackage installed which matches the kernel
`linux-image-*` metapackage you run.  For example, if you use the
`kernel-image-amd64` metapackage, then install `kernel-headers-amd64`.  If you
are using a kernel image metapackage from Debian Backports, then be sure to
double-check that you've selected a version of the matching headers metapackage
from Backports.

Install additional packages required for building and running the DKMS package:

```sh
sudo apt install build-essential dh-dkms dkms
```

Build the package:

```sh
fakeroot debian/rules binary
```

Install the package:

```sh
sudo dpkg -i ../avr-usb-gpio-dkms_*_all.deb
```

Now try the module:

```sh
modprobe gpio-avr-usb
```

If all goes well, you should see something like this in
[`dmesg(1)`](https://manpages.debian.org/stable/util-linux/dmesg.1.en.html):

    usb 1-2.4.2: New USB device found, idVendor=16c0, idProduct=05dc, bcdDevice= 1.00
    usb 1-2.4.2: New USB device strings: Mfr=1, Product=2, SerialNumber=0
    usb 1-2.4.2: Product: AVR USB GPIO
    usb 1-2.4.2: Manufacturer: ftlvisual.com
    gpio-avr-usb 1-2.4.2:1.0: adding board:
    gpio-avr-usb 1-2.4.2:1.0:   gpiochip0 "PA": _ _ PA2 PA3 PA4 PA5 PA6 PA7
    gpio-avr-usb 1-2.4.2:1.0:   gpiochip1 "PB": _ _ PB2 _
    gpio-avr-usb 1-2.4.2:1.0: 2 port(s), 7 line(s) available, 5 line(s) reserved

And you should see some GPIO character devices:

    $ ls /dev/gpiochip*
    /dev/gpiochip0  /dev/gpiochip1

Using GPIO from Userland
------------------------

The libgpiod utilities (Debian:
[`gpiod`](https://packages.debian.org/stable/gpiod)) allow you to inspect GPIO
lines, and read/write values using
[`gpioget(1)`](https://manpages.debian.org/stable/gpiod/gpioget.1.en.html) and
[`gpioset(1)`](https://manpages.debian.org/stable/gpiod/gpioset.1.en.html):

    $ gpiodetect
    gpiochip0 [PA] (8 lines)
    gpiochip1 [PB] (4 lines)
    $ gpioinfo
    gpiochip0 - 8 lines:
            line   0:        "PA0"       kernel   input  active-high [used]
            line   1:        "PA1"       kernel   input  active-high [used]
            line   2:        "PA2"       unused   input  active-high
            line   3:        "PA3"       unused   input  active-high
            line   4:        "PA4"       unused  output  active-high
            line   5:        "PA5"       unused  output  active-high
            line   6:        "PA6"       unused  output  active-high
            line   7:        "PA7"       unused  output  active-high
    gpiochip1 - 4 lines:
            line   0:        "PB0"       kernel   input  active-high [used]
            line   1:        "PB1"       kernel   input  active-high [used]
            line   2:        "PB2"       unused   input  active-high 
            line   3:        "PB3"       kernel   input  active-high [used]

In this example, I'm using an ATtiny24A, which reserves PA0, PA1, PB0, PB1, and
PB3 for its own use.  Reserved lines are reported by the driver for your
reference, but are not available for use.

For Python work, the
[python-periphery library](https://github.com/vsergeev/python-periphery/)
(Debian:
[`python3-periphery`](https://packages.debian.org/stable/python3-periphery))
offers a nice interface for interfacing with GPIO.

udev Rules
==========

Example udev rules are provided in `udev-rules.d/`.  These rules add persistent
symlinks for `gpiochipN` character devices to subdirectories under
`/dev/gpio/`, similar to `/dev/disk/by-*` and `/dev/serial/by-*` symlinks.

    /dev/gpio
    ├── by-id
    │   ├── usb-ftlvisual.com_AVR_USB_GPIO-if00-PA -> ../../gpiochip0
    │   └── usb-ftlvisual.com_AVR_USB_GPIO-if00-PB -> ../../gpiochip1
    └── by-path
        ├── pci-0000:00:14.0-usb-0:2.4.2:1.0-chip0 -> ../../gpiochip0
        └── pci-0000:00:14.0-usb-0:2.4.2:1.0-chip1 -> ../../gpiochip1

Rationale
---------

This is helpful for reliable script/program integration, since low-level GPIO
character device names like `gpiochip0` can change.  For example, if a cable
gets wiggled or a USB hub just decides to reset a port, then the device is
re-enumerated and new GPIO chips are added.  If some program still had
`/dev/gpiochip0` open at the time of re-enumeration, then the old stale GPIO
chip will hold onto that name, making the name `gpiochip0` unavailable for the
new GPIO chip instance(s).  Eventually, the program will fail when it tries to
read or write to a GPIO line using its file descriptor for `gpiochip0`.

If the program is configured to access `/dev/gpiochip0`, then when it restarts,
it won't be able to open `/dev/gpiochip0`, since a different name was assigned
for the newly-enumerated GPIO chip.  This condition would persist indefinitely
(until another re-enumeration), nomatter how many times the program is
restarted.

By installing these udev rules and using the `/dev/gpio/by-*` symlinks in your
program configuration, the program will still fail upon a disconnected AVR USB
GPIO device, but would reconnect to the same GPIO chip at its new low-level
(`gpiochipN`) name.

Rule Installation
-----------------

Copy the example rules from `udev-rules.d/` to `/etc/udev/rules.d/`, then run:

```sh
sudo udevadm trigger
```
