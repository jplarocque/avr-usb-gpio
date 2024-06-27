### GPIO-12 board

This is a gpio over usb board based on [VUSB](https://www.obdev.at/products/vusb/index.html) stack for AVR atmega8 chip. It exposes 12 GPIO ports.

My current prototype uses [USBPrayog Board](http://rarecomponents.com/store/1411?search=prayog) from
[rarecomponents.com](http://rarecomponents.com).

I shall produce an exclusive kicad schematic for this board soon.
[![board in action](https://img.youtube.com/vi/E6ALEKi3zcU/0.jpg)](https://www.youtube.com/watch?v=E6ALEKi3zcU)


### How to flash mega8
`cd firmware`  
`make hex`  
`avrdude -c usbasp -p m8 -U flash:w:main.hex`  



### PIN mapping
#### GPIO pins
Atmega8 gpio | pin number
-------------| ------------
PD0          |   1
PD1          |   2
PD2          |   Not available (used as D+)
PD3          |   3
PD4          |   Not available (used as D-)
PD5          |   4
PD6          |   5
PD7          |   6
PB0          |   7
PB1          |   8
PB2          |   9
PB3          |   10
PB4          |   11
PB5          |   12
PB6          |   Not available (used by crystal)
PB7          |   Not available (used by crystal)

### how to access gpio port as linux kernel sysfs
You need to load the kernel driver for this.

`$ cd driver/gpio`  
`$ make`  
`$ sudo insmod usb-gpio12.ko`  
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
To unload the module, `$ sudo rmmod usb_gpio12`  

### How to control gpio pins via libusb
In case, you want to control gpio ports via libusb, follow this guide.

include following code in your application code or include the common.h file.


```C
typedef struct __attribute__((__packed__)) _gpio_info
{
   uint8_t no;
   uint8_t data;
} gpio_info;

typedef struct __attribute__((__packed__)) _gpiopktheader
{
   uint8_t command;
   gpio_info gpio;
} gpiopktheader;

typedef enum _command
{
   BOARD_INIT, // This does the init of board
   GPIO_INPUT, // Set GPIO as input
   GPIO_OUTPUT, // Set GPIO as output
   GPIO_READ,   // Read GPIO
   GPIO_WRITE, // Write to GPIO
} command;

```

- init the board
```C
uint8_t buffer[3];
usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN,
                        BOARD_INIT, 0, 0, buffer, 3, 1000);
gpiopktheader *reply = (gpiopktheader *)buffer;
```
- set GPIO as output
```C
uint8_t buffer[3];
usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN,
                        GPIO_OUTPUT, gpio_number, 0, buffer, 3, 1000);
gpiopktheader *reply = (gpiopktheader *)buffer;
```

- write to GPIO

*high*  
```C
uint8_t buffer[3];
usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN,
                        GPIO_WRITE, gpio_number | (1 << 8), 0, buffer, 3, 1000);
gpiopktheader *reply = (gpiopktheader *)buffer;
```

*low*  
```C
uint8_t buffer[3];
usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN,
                        GPIO_WRITE, gpio_number | (0 << 8), 0, buffer, 3, 1000);
gpiopktheader *reply = (gpiopktheader *)buffer;
```

- Read GPIO
```C
uint8_t buffer[3];

usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN,
                        GPIO_INPUT, gpio_number, 0, buffer, 3, 1000);
usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN,
                        GPIO_READ, gpio_number, 0, buffer, 3, 1000);
gpiopktheader *reply = (gpiopktheader *)buffer;

uint8_t gpio_read_val = reply->val;
```

you can look into [ubstest](https://raw.githubusercontent.com/amitesh-singh/usb-gpio-board/master/firmware/usbtest/usbtest.c) example for more details.
#### example
- [on-off](https://raw.githubusercontent.com/amitesh-singh/usb-gpio-board/master/examples/on-off/on-off.c)  

### GPIO write speed

GPIO write speed is close to 1Khz.
![GPIO write speed - logic analyzer](./photos/gpio_write_speed.png)

### TODOs
 - ~~write firmware~~ **DONE**
 - ~~write basic gpio driver~~ **DONE**
 - ~~Add support of spin locking in gpio driver.~~ **DONE**
 - Add PWM pins support.
 - Add PWM driver?.
 - Add UART support.
 - Add UART ttyUSB driver support.
 - design schematic.

## Links
 - how to make nice pinouts: http://www.pighixxx.com/test/?s=made+a+pinout
 http://www.pighixxx.com/test/2016/06/how-its-made-a-pinout-part-1/
- VUSB: https://www.obdev.at/products/vusb/index.html
