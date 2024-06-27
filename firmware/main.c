/*
 *  GPIO-12 board firmware
 *
 *  (C) Amitesh Singh <singh.amitesh@gmail.com>, 2016
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  D2(D+) and D4(D-) are used for vusb  and PB6 and PB6 are used for 16MHz crystal
 *  then 12 GPIOs are only left for general purpose.
 *  PD0  - PD7 (excluding PD2/D+ and PD4/D - ) = 6
 *	PB0  - PB5 (excluding PB6 and PB7)  = 6
 */

#include <stdbool.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <avr/pgmspace.h>
#include "usbdrv.h"

#define AVR_USB_FIRMWARE
#include "common.h"

#define ARRAYLEN(array) (sizeof ((array)) / sizeof ((array)[0]))

static uint8_t replybuf[5];

static inline volatile uint8_t *
PORTx(volatile uint8_t *base) {
    return base - 0;
}

static inline volatile uint8_t *
DDRx(volatile uint8_t *base) {
    return base - 1;
}

static inline volatile uint8_t *
PINx(volatile uint8_t *base) {
    return base - 2;
}

#define TABENT(port, bit) (((port) << 3) | (bit))
#define PORT_A 0
#define PORT_B 1
#define PORT_C 2
#define PORT_D 3

static const PROGMEM uint8_t gpiotab[] = {
    /* List the defined entries contiguously to easily find the index of an
       entry by its line number: */
    TABENT(PORT_B, 0),
    TABENT(PORT_B, 1),
    TABENT(PORT_B, 2),
    TABENT(PORT_B, 3),
    TABENT(PORT_B, 4),
    TABENT(PORT_B, 5),
    TABENT(PORT_C, 0),
    TABENT(PORT_C, 1),
    TABENT(PORT_C, 2),
    TABENT(PORT_C, 3),
    TABENT(PORT_C, 4),
    TABENT(PORT_C, 5),
    TABENT(PORT_D, 0),
    TABENT(PORT_D, 1),
    TABENT(PORT_D, 3),
    TABENT(PORT_D, 5),
    TABENT(PORT_D, 6),
    TABENT(PORT_D, 7),
    /* Undefined entries due to pin conflicts or other reasons: */
    //TABENT(PORT_B, 6), // XTAL1
    //TABENT(PORT_B, 7), // XTAL2
    //TABENT(PORT_C, 6), // /Reset
    //TABENT(PORT_D, 2), // INT0, D+
    //TABENT(PORT_D, 4), // D-
};

static bool
gpio_base_and_mask(volatile uint8_t **base, uint8_t *mask, uint8_t no) {
    if (no < ARRAYLEN(gpiotab)) {
        uint8_t tabent = pgm_read_byte(&gpiotab[no]);
        uint8_t port_no = tabent >> 3, bit_no = tabent & 0b111;
        *base = &PORTB + 3 - 3 * port_no;
        *mask = 1 << bit_no;
        return true;
    } else {
        return false;
    }
}

static void
_gpio_init(uint8_t no, uint8_t input)
{
    volatile uint8_t *base;
    uint8_t mask;
    if (! gpio_base_and_mask(&base, &mask, no)) return;
    if (input) *DDRx(base) &= ~mask;
    else *DDRx(base) |= mask;
}

static void
_gpio_access(uint8_t no, uint8_t write, uint8_t *val)
{
    volatile uint8_t *base;
    uint8_t mask;
    if (! gpio_base_and_mask(&base, &mask, no)) return;
    if (write) {
        if (*val) *PORTx(base) |= mask;
        else *PORTx(base) &= ~mask;
    } else {
        *val = !!(*PINx(base) & mask);
    }
}

/*
#define MOSI PB3
#define MISO PB4
#define SCK PB5
#define SS PB3
 */

usbMsgLen_t
usbFunctionSetup(uchar data[8])
{
   usbRequest_t *rq = (void *)data;
   uint8_t len = 0;

   replybuf[0] = rq->bRequest;

   switch(rq->bRequest)
     {
      case BOARD_INIT:

         //do board init stuffs,
         len = 2;
         replybuf[1] = ARRAYLEN(gpiotab);
         //blink leds etcs ? we could use some port for blinking? not sure?
         break;

      case GPIO_INPUT:
         replybuf[1] = rq->wValue.bytes[0]; // gpio no
         _gpio_init(replybuf[1], 1);

         len = 2;
         break;

      case GPIO_OUTPUT:
         replybuf[1] = rq->wValue.bytes[0]; //gpio no
         _gpio_init(replybuf[1], 0);

         len = 2;
         break;

      case GPIO_READ:
         replybuf[1] = rq->wValue.bytes[0]; // gpio no
         _gpio_access(replybuf[1], 0, &replybuf[2]); //this populates gpio value

         len = 3;
         break;

      case GPIO_WRITE:
         replybuf[1] = rq->wValue.bytes[0]; //gpio no.
         replybuf[2] = rq->wValue.bytes[1]; // gpio value
         _gpio_access(replybuf[1], 1, &replybuf[2]);

         len = 3;
         break;

      default:
         break;
     }

   usbMsgPtr = (unsigned char *) replybuf;

   return len; // should not get here
}

int __attribute__((noreturn))
main(void)
{
   uchar i = 0;

   wdt_enable(WDTO_2S);
   usbInit();
   usbDeviceDisconnect();

   while(--i)
     {
        wdt_reset();
        _delay_ms(1);
     }

   usbDeviceConnect();
   sei();

   while(1)
     {
        wdt_reset();
        usbPoll();
     }
}
