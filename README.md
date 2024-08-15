### AVR USB GPIO

This is a GPIO over USB board based on the
[V-USB](https://www.obdev.at/products/vusb/index.html) library for AVR and the
Atmel/Microchip ATmega8 microcontroller.  It exposes 18 GPIO lines.

### How to Flash ATmega8

```sh
cd firmware
make hex
avrdude -c usbasp -p m8 -U flash:w:main.hex
```

### GPIO pin mapping

| ATmega8 GPIO |                      Pin number |
| :----------- | ------------------------------: |
| PB0          |                               0 |
| PB1          |                               1 |
| PB2          |                               2 |
| PB3          |                               3 |
| PB4          |                               4 |
| PB5          |                               5 |
| PB6          | Not available (used by crystal) |
| PB7          | Not available (used by crystal) |
| PC0          |                               6 |
| PC1          |                               7 |
| PC2          |                               8 |
| PC3          |                               9 |
| PC4          |                              10 |
| PC5          |                              11 |
| PC6          | Not available (used for /Reset) |
| PD0          |                              12 |
| PD1          |                              13 |
| PD2          |      Not available (used as D+) |
| PD3          |                              14 |
| PD4          |      Not available (used as D-) |
| PD5          |                              15 |
| PD6          |                              16 |
| PD7          |                              17 |

### How to access GPIO port as Linux kernel sysfs

You need to load the kernel driver for this.

`$ cd driver/gpio`  
`$ make`  
`$ sudo insmod avr-usb-gpio.ko`  
`$ cd /sys/class/gpio/ `  
           `gpiochip**N**` where **N** is the value allocated by kernel   
`$ sudo chmod 666 export unexport`  
`$ echo **N** > export`  
`$ cd gpio**N**`  
`$ echo out > direction`  
`$ cat value`  
`$ echo 1 > value`  
`$ echo 0 > value`  

Before unloading the module, do $ echo **N** > unexport  
gpio**N** gets deleted by doing so.  
To unload the module, `$ sudo rmmod avr_usb_gpio`  

## Links

  - VUSB: https://www.obdev.at/products/vusb/index.html
