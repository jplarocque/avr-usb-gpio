/* 
 * AVR USB GPIO
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

#ifndef _AVR_USB_GPIO_PROTOCOL_H
#define _AVR_USB_GPIO_PROTOCOL_H

enum proto_cmd {
    // IN: get array of valid I/O line masks for each port
    MSG_VALID_MASK,
    
    // IN: get array of physically-present line counts for each port
    MSG_LINE_COUNT,
    
    // IN: get [PORT[index], DDR[index]]
    // OUT: set PORT[index] = value[0], DDR[index] = value[1] (little-endian)
    MSG_PORT_DDR,
    
    // IN: get [PIN[index]]
    MSG_PIN,
};

#endif
