/* 
 * AVR-GPIO
 * 
 * (C) Amitesh Singh <singh.amitesh@gmail.com>, 2016
 * © 2024 Jean-Paul Larocque
 * 
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 * 
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 * 
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

// Support macros for usbconfig.h, included by usbdrv.h:
#define DEFFUSES(...)                           \
    FUSES = {__VA_ARGS__};
#define DEFGPIOTAB(...)                                         \
    static const PROGMEM uint8_t gpiotab[] = {__VA_ARGS__};
#define TABENT(port, bit) (((port) << 3) | (bit))
#define PORT_A 0
#define PORT_B 1
#define PORT_C 2
#define PORT_D 3
#include "usbdrv.h"
#include "common.h"

#define ARRAYLEN(array) (sizeof ((array)) / sizeof ((array)[0]))

static bool
gpio_base_and_mask(volatile uint8_t **base, uint8_t *mask, uint8_t gpio_num);
static inline volatile uint8_t *
PORTx(volatile uint8_t *base);
static inline volatile uint8_t *
DDRx(volatile uint8_t *base);
static inline volatile uint8_t *
PINx(volatile uint8_t *base);

int __attribute__((noreturn))
main(void) {
    wdt_enable(WDTO_2S);
    usbInit();
    usbDeviceDisconnect();
    wdt_reset();
    _delay_ms(250);
    usbDeviceConnect();
    sei();

    while (true) {
        wdt_reset();
        usbPoll();
    }
}

usbMsgLen_t
usbFunctionSetup(uchar data[8]) {
    usbRequest_t *rq = (void *) data;
    enum proto_cmd cmd = rq->bRequest;
    static uint8_t replybuf[1];
    usbMsgPtr = replybuf;
   
    if (cmd == GET_INFO) {
        replybuf[0] = ARRAYLEN(gpiotab);
        return 1;
    }
   
    // All other requests encode a GPIO number in wIndex.
    uint8_t gpio_num = rq->wIndex.bytes[0];
    volatile uint8_t *base;
    uint8_t mask;
    if (! gpio_base_and_mask(&base, &mask, gpio_num)) return 0;
   
    switch(cmd) {
    case GPIO_INPUT:
        *DDRx(base) &= ~mask;
        return 0;
       
    case GPIO_OUTPUT:
        *DDRx(base) |= mask;
        return 0;
       
    case GPIO_READ:
        replybuf[0] = *PINx(base) & mask;
        return 1;

    case GPIO_WRITE:
        if (rq->wValue.bytes[0]) *PORTx(base) |= mask;
        else *PORTx(base) &= ~mask;
        return 0;
       
    default:
        return 0;
    }
}

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
