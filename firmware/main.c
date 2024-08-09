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
#include <stddef.h>
#include <string.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>

#define FROM_MAIN_FIRMWARE
#include "usbdrv.h"
#include "common.h"

#define ARRAYLEN(array) (sizeof ((array)) / sizeof ((array)[0]))
#define MAX(x, y) ((x) >= (y) ? (x) : (y))

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
    /* Urge GCC to allocate `data` in one of the base pointer register pairs (Y
       or Z), which are eligible for dereferencing w/ displacement.  Otherwise,
       it puts `data` in X, which doesn't have displacement, wasting 18 bytes
       on ADIW & SBIW twiddling to access the desired address. */
    asm volatile ("" : : "b" (data));
    enum proto_cmd cmd = rq->bRequest;
    if ((rq->bmRequestType & USBRQ_TYPE_MASK) != USBRQ_TYPE_VENDOR) return 0;
    bool dir_in = rq->bmRequestType & USBRQ_DIR_DEVICE_TO_HOST;
    static uint8_t replybuf[MAX(2U, ARRAYLEN(io_ports))];
    usbMsgPtr = replybuf;
   
    if (cmd == MSG_VALID_MASK && dir_in) {
        for (uint8_t i = 0; i < ARRAYLEN(io_ports); i++) {
            replybuf[i] = io_ports[i].valid_mask;
        }
        return ARRAYLEN(io_ports);
    }
   
    // All other requests encode a port number in wIndex.
    if (rq->wIndex.bytes[1] > 0) return 0;
    uint8_t port_num = rq->wIndex.bytes[0];
    if (port_num >= ARRAYLEN(io_ports)) return 0;
    const struct gpio_port *gpio_port = io_ports + port_num;
    /* Same hack to encourage `gpio_port` to be allocated in a base pointer
       register pair, where it's really needed.  GCC already does this for
       `gpio_port`, but that could change at any time. */
    asm volatile ("" : : "b" (gpio_port));
    uint8_t valid_mask = gpio_port->valid_mask;
    
    switch(cmd) {
    case MSG_PORT_DDR:
        if (dir_in) {
            replybuf[0] = *gpio_port->PORTx & valid_mask;
            replybuf[1] = *gpio_port->DDRx & valid_mask;
            return 2;
        } else {
            /* Pre-fetch and precalculate as much as possible before entering
               the atomic section. */
            volatile uint8_t
                *DDRx = gpio_port->DDRx,
                *PINx = gpio_port->PINx;
            uint8_t
                PORTx_req = rq->wValue.bytes[0],
                DDRx_req = rq->wValue.bytes[1],
                DDRx_val,
                DDRx_clear = DDRx_req | ~valid_mask,
                DDRx_set = DDRx_req & valid_mask,
                PINx_new = (*gpio_port->PORTx ^ PORTx_req) & valid_mask;
            /* Hack to convince GCC to really pre-calculate everything we
               precalculated above, instead of inside the ATOMIC_BLOCK.
               Minimizes time spent with interrupts disabled (12 cycles
               total). */
            asm volatile ("" : :
                          "e" (DDRx),
                          "e" (PINx),
                          "r" (DDRx_clear),
                          "r" (DDRx_set),
                          "r" (PINx_new));
            ATOMIC_BLOCK(ATOMIC_FORCEON) {
                /* Until desired output levels are set, only disable any lines
                   that are now inputs.  We don't want to begin driving any
                   newly-enabled output lines at the wrong level. */
                *DDRx = (DDRx_val = *DDRx & DDRx_clear);
                /* Toggle PORTx bits to get the correct level.  Toggling saves
                   a boolean operation, compared to reading and updating PORTx
                   in a masked fashion like we do DDRx. */
                *PINx = PINx_new;
                // Safe to enable new outputs now.
                *DDRx = DDRx_val | DDRx_set;
            }
            return 0;
        }
    case MSG_PIN:
        if (dir_in) {
            replybuf[0] = *gpio_port->PINx & valid_mask;
            return 1;
        }
        break;
    default:
        break;
    }
    
    return 0;
}
