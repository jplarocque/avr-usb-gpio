/* Name: usbconfig.h
 * Project: V-USB, virtual USB port for Atmel's(r) AVR(r) microcontrollers
 * Author: Christian Starkjohann
 * Creation Date: 2005-04-01
 * Tabsize: 4
 * Copyright: (c) 2005 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
 * This Revision: $Id: usbconfig-prototype.h 785 2010-05-30 17:57:07Z cs $
 */

#ifndef _AVR_USB_GPIO_USBCONFIG_H
#define _AVR_USB_GPIO_USBCONFIG_H

#include <avr/io.h>
#include "usb_ids.h"

/* We unify hardware configuration information required both by v-usb and the
   main firmware code into this single header file.

   Common configuration is defined up-front.  Device-specific v-usb macros are
   left here in the initial part, next to the per-macro documentation, but
   commented out.  Those device-macros are defined at the end of this file,
   conditionally based on the device model. */

/*
General Description:
This file is an example configuration (with inline documentation) for the USB
driver. It configures V-USB for USB D+ connected to Port D bit 2 (which is
also hardware interrupt 0 on many devices) and USB D- to Port D bit 4. You may
wire the lines to any other port, as long as D+ is also wired to INT0 (or any
other hardware interrupt, as long as it is the highest level interrupt, see
section at the end of this file).
*/

/* ---------------------------- Hardware Config ---------------------------- */

//#define USB_CFG_IOPORTNAME	D
/* This is the port where the USB bus is connected. When you configure it to
 * "B", the registers PORTB, PINB and DDRB will be used.
 */
//#define USB_CFG_DMINUS_BIT	4
/* This is the bit number in USB_CFG_IOPORT where the USB D- line is connected.
 * This may be any bit in the port.
 */
//#define USB_CFG_DPLUS_BIT	2
/* This is the bit number in USB_CFG_IOPORT where the USB D+ line is connected.
 * This may be any bit in the port. Please note that D+ must also be connected
 * to interrupt pin INT0! [You can also use other interrupts, see section
 * "Optional MCU Description" below, or you can connect D- to the interrupt, as
 * it is required if you use the USB_COUNT_SOF feature. If you use D- for the
 * interrupt, the USB interrupt will also be triggered at Start-Of-Frame
 * markers every millisecond.]
 */
#define USB_CFG_CLOCK_KHZ	(F_CPU/1000)
/* Clock rate of the AVR in kHz. Legal values are 12000, 12800, 15000, 16000,
 * 16500, 18000 and 20000. The 12.8 MHz and 16.5 MHz versions of the code
 * require no crystal, they tolerate +/- 1% deviation from the nominal
 * frequency. All other rates require a precision of 2000 ppm and thus a
 * crystal!
 * Since F_CPU should be defined to your actual clock rate anyway, you should
 * not need to modify this setting.
 */
#ifndef USB_CFG_CHECK_CRC
#  define USB_CFG_CHECK_CRC	(F_CPU == 18000000 && FLASHEND >= 0xFFF)
#endif
/* Define this to 1 if you want that the driver checks integrity of incoming
 * data packets (CRC checks). CRC checks cost quite a bit of code size and are
 * currently only available for 18 MHz crystal clock. You must choose
 * USB_CFG_CLOCK_KHZ = 18000 if you enable this option.
 */

/* ----------------------- Optional Hardware Config ------------------------ */

//#define USB_CFG_PULLUP_IOPORTNAME	D
/* If you connect the 1.5k pullup resistor from D- to a port pin instead of
 * V+, you can connect and disconnect the device from firmware by calling
 * the macros usbDeviceConnect() and usbDeviceDisconnect() (see usbdrv.h).
 * This constant defines the port on which the pullup resistor is connected.
 */
//#define USB_CFG_PULLUP_BIT		4
/* This constant defines the bit number in USB_CFG_PULLUP_IOPORT (defined
 * above) where the 1.5k pullup resistor is connected. See description
 * above for details.
 */

/* --------------------------- Functional Range ---------------------------- */

#define USB_CFG_HAVE_INTRIN_ENDPOINT	0
/* Define this to 1 if you want to compile a version with two endpoints: The
 * default control endpoint 0 and an interrupt-in endpoint (any other endpoint
 * number).
 */
#define USB_CFG_HAVE_INTRIN_ENDPOINT3	0
/* Define this to 1 if you want to compile a version with three endpoints: The
 * default control endpoint 0, an interrupt-in endpoint 3 (or the number
 * configured below) and a catch-all default interrupt-in endpoint as above.
 * You must also define USB_CFG_HAVE_INTRIN_ENDPOINT to 1 for this feature.
 */
#define USB_CFG_EP3_NUMBER		3
/* If the so-called endpoint 3 is used, it can now be configured to any other
 * endpoint number (except 0) with this macro. Default if undefined is 3.
 */
//#define USB_INITIAL_DATATOKEN		USBPID_DATA1
/* The above macro defines the startup condition for data toggling on the
 * interrupt/bulk endpoints 1 and 3. Defaults to USBPID_DATA1.
 * Since the token is toggled BEFORE sending any data, the first packet is
 * sent with the oposite value of this configuration!
 */
#define USB_CFG_IMPLEMENT_HALT		0
/* Define this to 1 if you also want to implement the ENDPOINT_HALT feature
 * for endpoint 1 (interrupt endpoint). Although you may not need this feature,
 * it is required by the standard. We have made it a config option because it
 * bloats the code considerably.
 */
#define USB_CFG_SUPPRESS_INTR_CODE	0
/* Define this to 1 if you want to declare interrupt-in endpoints, but don't
 * want to send any data over them. If this macro is defined to 1, functions
 * usbSetInterrupt() and usbSetInterrupt3() are omitted. This is useful if
 * you need the interrupt-in endpoints in order to comply to an interface
 * (e.g. HID), but never want to send any data. This option saves a couple
 * of bytes in flash memory and the transmit buffers in RAM.
 */
#define USB_CFG_INTR_POLL_INTERVAL	100
/* If you compile a version with endpoint 1 (interrupt-in), this is the poll
 * interval. The value is in milliseconds and must not be less than 10 ms for
 * low speed devices.
 */
#define USB_CFG_IS_SELF_POWERED		0
/* Define this to 1 if the device has its own power supply. Set it to 0 if the
 * device is powered from the USB bus.
 */
#define USB_CFG_MAX_BUS_POWER		20
/* Set this variable to the maximum USB bus power consumption of your device.
 * The value is in milliamperes. [It will be divided by two since USB
 * communicates power requirements in units of 2 mA.]
 */
#define USB_CFG_IMPLEMENT_FN_WRITE	0
/* Set this to 1 if you want usbFunctionWrite() to be called for control-out
 * transfers. Set it to 0 if you don't need it and want to save a couple of
 * bytes.
 */
#define USB_CFG_IMPLEMENT_FN_READ	0
/* Set this to 1 if you need to send control replies which are generated
 * "on the fly" when usbFunctionRead() is called. If you only want to send
 * data from a static buffer, set it to 0 and return the data from
 * usbFunctionSetup(). This saves a couple of bytes.
 */
#define USB_CFG_IMPLEMENT_FN_WRITEOUT	0
/* Define this to 1 if you want to use interrupt-out (or bulk out) endpoints.
 * You must implement the function usbFunctionWriteOut() which receives all
 * interrupt/bulk data sent to any endpoint other than 0. The endpoint number
 * can be found in 'usbRxToken'.
 */
#define USB_CFG_HAVE_FLOWCONTROL	0
/* Define this to 1 if you want flowcontrol over USB data. See the definition
 * of the macros usbDisableAllRequests() and usbEnableAllRequests() in
 * usbdrv.h.
 */
#define USB_CFG_DRIVER_FLASH_PAGE	0
/* If the device has more than 64 kBytes of flash, define this to the 64 k page
 * where the driver's constants (descriptors) are located. Or in other words:
 * Define this to 1 for boot loaders on the ATMega128.
 */
#define USB_CFG_LONG_TRANSFERS		0
/* Define this to 1 if you want to send/receive blocks of more than 254 bytes
 * in a single control-in or control-out transfer. Note that the capability
 * for long transfers increases the driver size.
 */
//#define USB_RX_USER_HOOK(data, len)	if(usbRxToken == (uchar)USBPID_SETUP) blinkLED();
/* This macro is a hook if you want to do unconventional things. If it is
 * defined, it's inserted at the beginning of received message processing.
 * If you eat the received message and don't want default processing to
 * proceed, do a return after doing your things. One possible application
 * (besides debugging) is to flash a status LED on each packet.
 */
//#define USB_RESET_HOOK(resetStarts)	if(!resetStarts){hadUsbReset();}
/* This macro is a hook if you need to know when an USB RESET occurs. It has
 * one parameter which distinguishes between the start of RESET state and its
 * end.
 */
//#define USB_SET_ADDRESS_HOOK()	hadAddressAssigned();
/* This macro (if defined) is executed when a USB SET_ADDRESS request was
 * received.
 */
#define USB_COUNT_SOF			0
/* define this macro to 1 if you need the global variable "usbSofCount" which
 * counts SOF packets. This feature requires that the hardware interrupt is
 * connected to D- instead of D+.
 */
/*
#ifdef __ASSEMBLER__
macro myAssemblerMacro
    in      YL, TCNT0
    sts     timer0Snapshot, YL
    endm
#endif
#define USB_SOF_HOOK			myAssemblerMacro
*/
/* This macro (if defined) is executed in the assembler module when a
 * Start Of Frame condition is detected. It is recommended to define it to
 * the name of an assembler macro which is defined here as well so that more
 * than one assembler instruction can be used. The macro may use the register
 * YL and modify SREG. If it lasts longer than a couple of cycles, USB messages
 * immediately after an SOF pulse may be lost and must be retried by the host.
 * What can you do with this hook? Since the SOF signal occurs exactly every
 * 1 ms (unless the host is in sleep mode), you can use it to tune OSCCAL in
 * designs running on the internal RC oscillator.
 * Please note that Start Of Frame detection works only if D- is wired to the
 * interrupt, not D+. THIS IS DIFFERENT THAN MOST EXAMPLES!
 */
#define USB_CFG_CHECK_DATA_TOGGLING	0
/* define this macro to 1 if you want to filter out duplicate data packets
 * sent by the host. Duplicates occur only as a consequence of communication
 * errors, when the host does not receive an ACK. Please note that you need to
 * implement the filtering yourself in usbFunctionWriteOut() and
 * usbFunctionWrite(). Use the global usbCurrentDataToken and a static variable
 * for each control- and out-endpoint to check for duplicate packets.
 */

/* define this macro to 1 if you want the function usbMeasureFrameLength()
 * compiled in. This function can be used to calibrate the AVR's RC oscillator.
 */
#define USB_USE_FAST_CRC		0
/* The assembler module has two implementations for the CRC algorithm. One is
 * faster, the other is smaller. This CRC routine is only used for transmitted
 * messages where timing is not critical. The faster routine needs 31 cycles
 * per byte while the smaller one needs 61 to 69 cycles. The faster routine
 * may be worth the 32 bytes bigger code size if you transmit lots of data and
 * run the AVR close to its limit.
 */

/* -------------------------- Device Description --------------------------- */
// See also: usbidconfig.h

#define USB_CFG_DEVICE_CLASS	0xff
#define USB_CFG_DEVICE_SUBCLASS	0
/* See USB specification if you want to conform to an existing device class.
 * Class 0xff is "vendor specific".
 */
#define USB_CFG_INTERFACE_CLASS		0
#define USB_CFG_INTERFACE_SUBCLASS	0
#define USB_CFG_INTERFACE_PROTOCOL	0
/* See USB specification if you want to conform to an existing device class or
 * protocol. The following classes must be set at interface level:
 * HID class is 3, no subclass and protocol required (but may be useful!)
 * CDC class is 2, use subclass 2 and protocol 1 for ACM
 */
// comment this if u r doing custom class.
//#define USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH	52
/* Define this to the length of the HID report descriptor, if you implement
 * an HID device. Otherwise don't define it or define it to 0.
 * If you use this define, you must add a PROGMEM character array named
 * "usbHidReportDescriptor" to your code which contains the report descriptor.
 * Don't forget to keep the array and this define in sync!
 */

//#define USB_PUBLIC static
/* Use the define above if you #include usbdrv.c instead of linking against it.
 * This technique saves a couple of bytes in flash memory.
 */

/* ------------------- Fine Control over USB Descriptors ------------------- */
/* If you don't want to use the driver's default USB descriptors, you can
 * provide our own. These can be provided as (1) fixed length static data in
 * flash memory, (2) fixed length static data in RAM or (3) dynamically at
 * runtime in the function usbFunctionDescriptor(). See usbdrv.h for more
 * information about this function.
 * Descriptor handling is configured through the descriptor's properties. If
 * no properties are defined or if they are 0, the default descriptor is used.
 * Possible properties are:
 *   + USB_PROP_IS_DYNAMIC: The data for the descriptor should be fetched
 *     at runtime via usbFunctionDescriptor(). If the usbMsgPtr mechanism is
 *     used, the data is in FLASH by default. Add property USB_PROP_IS_RAM if
 *     you want RAM pointers.
 *   + USB_PROP_IS_RAM: The data returned by usbFunctionDescriptor() or found
 *     in static memory is in RAM, not in flash memory.
 *   + USB_PROP_LENGTH(len): If the data is in static memory (RAM or flash),
 *     the driver must know the descriptor's length. The descriptor itself is
 *     found at the address of a well known identifier (see below).
 * List of static descriptor names (must be declared PROGMEM if in flash):
 *   char usbDescriptorDevice[];
 *   char usbDescriptorConfiguration[];
 *   char usbDescriptorHidReport[];
 *   char usbDescriptorString0[];
 *   int usbDescriptorStringVendor[];
 *   int usbDescriptorStringDevice[];
 *   int usbDescriptorStringSerialNumber[];
 * Other descriptors can't be provided statically, they must be provided
 * dynamically at runtime.
 *
 * Descriptor properties are or-ed or added together, e.g.:
 * #define USB_CFG_DESCR_PROPS_DEVICE   (USB_PROP_IS_RAM | USB_PROP_LENGTH(18))
 *
 * The following descriptors are defined:
 *   USB_CFG_DESCR_PROPS_DEVICE
 *   USB_CFG_DESCR_PROPS_CONFIGURATION
 *   USB_CFG_DESCR_PROPS_STRINGS
 *   USB_CFG_DESCR_PROPS_STRING_0
 *   USB_CFG_DESCR_PROPS_STRING_VENDOR
 *   USB_CFG_DESCR_PROPS_STRING_PRODUCT
 *   USB_CFG_DESCR_PROPS_STRING_SERIAL_NUMBER
 *   USB_CFG_DESCR_PROPS_HID
 *   USB_CFG_DESCR_PROPS_HID_REPORT
 *   USB_CFG_DESCR_PROPS_UNKNOWN (for all descriptors not handled by the driver)
 *
 * Note about string descriptors: String descriptors are not just strings, they
 * are Unicode strings prefixed with a 2 byte header. Example:
 * int  serialNumberDescriptor[] = {
 *     USB_STRING_DESCRIPTOR_HEADER(6),
 *     'S', 'e', 'r', 'i', 'a', 'l'
 * };
 */

#define USB_CFG_DESCR_PROPS_DEVICE			0
#define USB_CFG_DESCR_PROPS_CONFIGURATION		0
#define USB_CFG_DESCR_PROPS_STRINGS			0
#define USB_CFG_DESCR_PROPS_STRING_0			0
#define USB_CFG_DESCR_PROPS_STRING_VENDOR		0
#define USB_CFG_DESCR_PROPS_STRING_PRODUCT		0
#define USB_CFG_DESCR_PROPS_STRING_SERIAL_NUMBER	0
#define USB_CFG_DESCR_PROPS_HID				0
#define USB_CFG_DESCR_PROPS_HID_REPORT			0
#define USB_CFG_DESCR_PROPS_UNKNOWN			0

/* ----------------------- Optional MCU Description ------------------------ */

/* The following configurations have working defaults in usbdrv.h. You
 * usually don't need to set them explicitly. Only if you want to run
 * the driver on a device which is not yet supported or with a compiler
 * which is not fully supported (such as IAR C) or if you use a differnt
 * interrupt than INT0, you may have to define some of these.
 */
//#define USB_INTR_CFG		MCUCR
//#define USB_INTR_CFG_SET	((1 << ISC00) | (1 << ISC01))
//#define USB_INTR_CFG_CLR	0
//#define USB_INTR_ENABLE	GIMSK
//#define USB_INTR_ENABLE_BIT	INT0
//#define USB_INTR_PENDING	GIFR
//#define USB_INTR_PENDING_BIT	INTF0
//#define USB_INTR_VECTOR	INT0_vect

/* --------------------- Device-Specific Configuration --------------------- */

#ifdef FROM_MAIN_FIRMWARE
/* When this header file is ultimately included from main.c, define a data
   structure (used only by main.c), and define macros which (unfortunately,
   since this is a /header/ file) define objects.

   This is gross, but it allows us to keep ALL MCU-specific configuration
   together.  Since MCU-specific configuration includes macros consumed by
   V-USB, that means defining it here, in this header file. */
struct gpio_port {
    volatile uint8_t *PORTx, *DDRx, *PINx;
#if ! __AVR_HAVE_MUL__
    uint8_t _pad[2];
#endif
};

#  define DEFFUSES(...)                         \
    FUSES = {__VA_ARGS__};
#  define DEFPORTVALID(...)                                     \
    static const uint8_t io_port_valid[] = {__VA_ARGS__};
#  define DEFPORTLINECOUNT(...)                                 \
    static const uint8_t io_port_line_count[] = {__VA_ARGS__};
#  define DEFPORTREGS(...)                                              \
    static const struct gpio_port io_port_regs[] = {__VA_ARGS__};
#else
// No-op macros for V-USB source files:
#  define DEFFUSES(...)
#  define DEFPORTVALID(...)
#  define DEFPORTLINECOUNT(...)
#  define DEFPORTREGS(...)
#endif

/* The DEFFUSES() macro builds on the avr-libc FUSES macro to define which
   fuses to program (see <avr/fuse.h> documentation).  You can use C
   preprocessor conditionals and C constant expressions logic/arithmetic in
   order to tune the fuses as needed for build variations, etc.
   
   The DEFPORTVALID(), DEFPORTLINECOUNT(), and DEFPORTREGS() macros defines
   three parallel arrays, where each index represents a possible I/O port, and
   the different arrays express different information about it.  They represent
   all I/O port names starting from "PA", and continuing in sequence up to the
   last (highest) named I/O port of the target MCU.  If the MCU doesn't have a
   "PA" I/O port (or any other port name interspersed in that sequence), an
   array entry still needs to be defined for that port name with a placeholder
   struct, so that the USB host-side device driver may correctly infer the port
   name of following structs.

   EXCEPTION: Atmel/Microchip don't define a "PI" I/O port for any AVR MCU
   (determined by checking avr-libc <avr/io*.h> headers), maybe to avoid
   confusion with the digit "1".  The driver DOES take this into account, so if
   you have ports numbered higher than PI, do NOT represent a placeholder port
   for "PI" in the macro initializers.

   Specify a comma-separated list of `uint8_t` valid-masks in DEFPORTVALID()
   (or 0 for placeholder ports).

   Specify a comma-separated list of `uint8_t` line counts (in [1,8]) in
   DEFPORTLINECOUNT() (or 0 for placeholder ports).
   
   Specify a comma-separated list of `struct gpio_port` initializers in
   DEFPORTREGS().  To define a real port, use pointers to its registers.  For
   example:
   
       {&PORTC, &DDRC, &PINC}

   To define a placeholder entry in DEFPORTREGS(), use pointers to the correct
   registers of any other single valid I/O port (this will be some other port
   which is represented as a real port in another array index).  For example:
   
       {&PORTB, &DDRB, &PINB},

   WARNING: Do not use a semicolon after any of these macro invocations.  The
   regular macros for main.c expand to include a terminating semicolon, so
   adding your own is not necessary.  In the case of expansion of the no-op
   macros from within V-USB code, an external trailing semicolon could wind up
   in non-C source code and cause problems.
   
   (To elaborate on the rationale for the above requirements:)
   
   When the driver interrogates the MCU firmware, it learns the number of
   defined I/O ports (real or placeholder), and some data about them.  The
   driver then labels the ports in sequence, from "PA" and up.  For example, if
   four ports are advertised, then the driver chooses the names "PA", "PB",
   "PC", and "PD".  If, however, the target MCU has only PB, PC, and PD I/O
   ports (at the hardware level), and you define array entries for only those
   three ports, then the driver sees three ports and names them "PA", "PB", and
   "PC", which is incorrect.  In this case, you would need a placeholder entry
   for the non-existent "PA".  This isn't a deficiency with the driver.  These
   implicit names are a protocol optimization which is intended to save space
   in MCU Flash storage by not having to store port names, and not needing code
   for transmitting names.
   
   When specifying pointers to registers of some real I/O port in a placeholder
   entry, that other port could be accessed if the USB device driver
   misbehaves.  However, its DDRx and PORTx registers will not be altered as
   long as .valid_mask = 0.  NULL would certainly be a more clean and natural
   choice of expression here, but to allow this, we'd need to add code
   (increasing Flash size) to handle and avoid null pointer dereferencing.
   Dereferencing NULL for reading would be fine--it'd be a read from CPU
   register r0, and that read data would be ANDed with 0 before it gets
   transmitted back to the USB host--but writing could corrupt program state if
   GCC generates code which changes r0 and relies on r0 not being modified from
   outside its control. */

#if defined __AVR_ATmega8__

DEFFUSES(.low = (// Brown-out detection, 4.0 V:
                 FUSE_BODLEVEL & FUSE_BODEN &
                 // Crystal oscillator, 1+ MHz:
                 /*FUSE_CKSEL3 & FUSE_CKSEL2 & FUSE_CKSEL1 & FUSE_CKSEL0 &*/
                 // For crystal oscillator w/ BOD enabled:
                 FUSE_SUT1 & /*FUSE_SUT0 &*/
                 0xFF),
         .high = (// Allow serial programming:
                  FUSE_SPIEN &
                  // Use full output swing for crystal oscillator:
                  FUSE_CKOPT &
                  0xFF))

#  define USB_CFG_IOPORTNAME		D
#  define USB_CFG_DMINUS_BIT		4
#  define USB_CFG_DPLUS_BIT		2
//#  define USB_CFG_PULLUP_IOPORTNAME	...
//#  define USB_CFG_PULLUP_BIT		...

DEFPORTVALID(// no PA
             0,
             // PB6/PB7 used for XTAL1/XTAL2.
             0b00111111,
             // PC6 used for /Reset; no PC7
             0b00111111,
             // PD2 used for INT0 & D+; PD4 used for D-
             0b11101011,
             )
DEFPORTLINECOUNT(0, 8, 7, 8)
DEFPORTREGS({&PORTB, &DDRB, &PINB}, // Placeholder
            {&PORTB, &DDRB, &PINB},
            {&PORTC, &DDRC, &PINC},
            {&PORTD, &DDRD, &PIND},
            )

#elif (defined __AVR_ATtiny24__ ||              \
       defined __AVR_ATtiny24A__ ||             \
       defined __AVR_ATtiny44__ ||              \
       defined __AVR_ATtiny44A__ ||             \
       defined __AVR_ATtiny84__ ||              \
       defined __AVR_ATtiny84A__)

DEFFUSES(.low = (// Crystal oscillator, 8+ MHz:
                 /*FUSE_CKSEL3 & FUSE_CKSEL2 & FUSE_CKSEL1 &*/
                 // For crystal oscillator w/ BOD enabled:
                 /*FUSE_CKSEL0 &*/ FUSE_SUT1 & /*FUSE_SUT0 &*/
                 0xFF),
         .high = (// Allow serial programming:
                  FUSE_SPIEN &
                  // Brown-out detection, 4.1/4.3/4.5 V min/typ/max:
                  /*FUSE_BODLEVEL2 &*/ FUSE_BODLEVEL1 & FUSE_BODLEVEL0 &
                  0xFF),
         .extended = 0xFF)

#  define USB_CFG_IOPORTNAME		A
#  define USB_CFG_DMINUS_BIT		1
#  define USB_CFG_DPLUS_BIT		0
//#  define USB_CFG_PULLUP_IOPORTNAME	...
//#  define USB_CFG_PULLUP_BIT		...

/* Not using INT0 pin for D+ or D- because INT0 is on PB, which doesn't have
   enough free pins to fit both D+ and D- (which are required by V-USB to be on
   the same I/O port).  To make it work, we'd have to use the RC oscillator to
   free up XTAL1/XTAL2, or disable /Reset AND hope that the reduced drive
   strength on that pin doesn't harm USB communication.

   Instead, use PCINT0 to trigger an interrupt on any level change (toggle) of
   D+. */ 
#  define USB_INTR_CFG			PCMSK0
#  define USB_INTR_CFG_SET		(1 << PCINT0)
#  define USB_INTR_CFG_CLR		0
#  define USB_INTR_ENABLE		GIMSK
#  define USB_INTR_ENABLE_BIT		PCIE0
#  define USB_INTR_PENDING		GIFR
#  define USB_INTR_PENDING_BIT		PCIF0
#  define USB_INTR_VECTOR		PCINT0_vect

DEFPORTVALID(// PA0 used for D+ and PCINT0; PA1 used for D-
             0b11111100,
             // PB0/PB1 used for XTAL1/XTAL2; PB3 used for /Reset; no PB[4:7]
             0b00000100,
             )
DEFPORTLINECOUNT(8, 4)
DEFPORTREGS({&PORTA, &DDRA, &PINA},
            {&PORTB, &DDRB, &PINB},
            )

#else
#  error "Unsupported device"
#endif

#endif
