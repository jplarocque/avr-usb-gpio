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

#ifndef COMMON_H_
#define COMMON_H_

enum proto_cmd {
   GET_INFO, // Get board information
   GPIO_INPUT, // Set GPIO as input
   GPIO_OUTPUT, // Set GPIO as output
   GPIO_READ, // Read GPIO
   GPIO_WRITE, // Write to GPIO
};

#endif /* COMMON_H_ */
