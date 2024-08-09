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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/overflow.h>

#include <linux/device.h>
#include <linux/usb.h>
#include <linux/slab.h>
#include <linux/gpio/driver.h>

#include "../../firmware/common.h"
#include "../../firmware/usbidconfig.h"

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Jean-Paul Larocque <jpl@ftlvisual.com>");
MODULE_DESCRIPTION("AVR-GPIO USB driver");
MODULE_VERSION("0.1");

/* Actual maximum is expected to be 11 (PA..PL on e.g. ATmega640, with "PI"
   skipped due to Atmel port numbering). */
#define MAX_PORTS 25

#define TIMEOUT 100

struct avr_gpio_board;
struct avr_gpio_port;

struct avr_gpio_port {
    struct gpio_chip gc;
    struct avr_gpio_board *board;
    /* For identifying this port on the wire (NOT an index into .ports[] of
       `struct avr_gpio_board`). */
    uint8_t id;
    /* Per-line flags/masks.

       To support "total" biasing with simplicity, we manipulate high-level
       desired-state bits instead of low-level hardware register (DDRx, PORTx)
       bits. */
    /* Whether the line is set for output, and the output value (ignored if
       direction is input). */
    uint8_t direction, value;
    /* Whether bias is enabled, and whether the bias is "total" (driving hard
       to VCC or GND; ignored if no bias).  Due to hardware limitations, when
       bias is enabled, the bias is always high (pull-up resistor or total VCC
       pull). */
    uint8_t bias_en, bias_total;
    /* Actual last-known register states, encoded in the on-wire format
       (PORTx | (DDRx << 8)).  May be set to a value outside [0, UINT16_MAX] to
       invalidate any previously-known value upon error. */
    int32_t actual_PORTx_DDRx;
};

struct avr_gpio_board {
    struct mutex lock;
    struct usb_device *udev;
    /* Maps port IDs to masks of usable I/O lines for that port.
       
       Used only at initialization time:
       
         - By usb_probe(); the stack would be a nicer choice for such a small,
           fixed-size buffer, but that isn't suitable for DMA by
           usb_control_msg().
         
         - By init_valid_mask(), where it must be looked up later due to the
           gpiolib API.  There's no way to pass the valid-mask directly during
           initialization.
       
       For maximum reliability, the device is responsible for enforcing the
       valid mask.  Therefore, this driver does not. */
    uint8_t *valid_mask; // length = MAX_PORTS
    uint8_t buf[2]; // Misc USB transfer buffer; can't be stack-allocated
    size_t port_count;
    /* Array of available ports which have a non-zero number of valid I/O
       lines.  This is indexed contiguously up to .port_count (exclusively) in
       initialization order.  NEVER index by port ID. */
    struct avr_gpio_port ports[];
};

static int
usb_probe(struct usb_interface *interface, const struct usb_device_id *id);
static int
fetch_port_state(struct avr_gpio_port *port);
static int
init_valid_mask(struct gpio_chip *gc, unsigned long *valid_mask,
                unsigned int ngpios);
static void
usb_disconnect(struct usb_interface *interface);
static void
free_board(struct avr_gpio_board *board);
static int
direction_input(struct gpio_chip *gc, unsigned int offset);
static int
direction_output(struct gpio_chip *gc, unsigned int offset, int value);
static int
get(struct gpio_chip *gc, unsigned int offset);
static int
get_multiple(struct gpio_chip *gc, unsigned long *mask, unsigned long *bits);
static int
get_raw(struct gpio_chip *gc);
static void
set(struct gpio_chip *gc, unsigned int offset, int value);
static void
set_multiple(struct gpio_chip *gc, unsigned long *mask, unsigned long *bits);
static int
set_config(struct gpio_chip *gc, unsigned int offset, unsigned long config);
static int
update_PORTx_DDRx(struct avr_gpio_port *port);

static struct usb_device_id id_table[] = {
    {USB_DEVICE(USB_VENDOR_ID, USB_DEVICE_ID)},
    {},
};
MODULE_DEVICE_TABLE(usb, id_table);

static struct usb_driver avr_gpio_driver = {
    .name = KBUILD_MODNAME,
    .id_table = id_table,
    .probe = usb_probe,
    .disconnect = usb_disconnect,
};
module_usb_driver(avr_gpio_driver);

static int
usb_probe(struct usb_interface *interface, const struct usb_device_id *id) {
    struct usb_device *udev = interface_to_usbdev(interface);
    int ret;

    static const char
        /* These name macros expand to a sequence of chars (as required by
           V-USB). */
        vendor_name[] = {USB_CFG_VENDOR_NAME, '\0'},
        device_name[] = {USB_CFG_DEVICE_NAME, '\0'};
    if (! (strcmp(udev->manufacturer, vendor_name) == 0 &&
           strcmp(udev->product, device_name) == 0)) {
        return -ENODEV;
    }
    struct avr_gpio_board *board = NULL;
    uint8_t *valid_mask = NULL;
    /* From this point forward, all error paths must go through `goto err` in
       order to ensure matching usb_put_dev() call. */

    size_t valid_mask_size = array_size(MAX_PORTS,
                                        sizeof board->valid_mask[0]);
    valid_mask = devm_kzalloc(&udev->dev, valid_mask_size, GFP_KERNEL);
    if (valid_mask == NULL) {
        ret = -ENOMEM;
        goto err;
    }
    ret = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0),
                          MSG_VALID_MASK,
                          USB_RECIP_DEVICE | USB_TYPE_VENDOR | USB_DIR_IN,
                          0, 0,
                          valid_mask, valid_mask_size,
                          TIMEOUT);
    if (ret < 0) {
        dev_err(&udev->dev, "MSG_VALID_MASK (IN) failed (error %d)\n",
                -ret);
        goto err;
    } else if (ret > valid_mask_size) {
        BUG(); // Buffer overflow!
        ret = -EOVERFLOW;
        goto err;
    }
    /* Number of ports as reported by the device.  port_count may end up being
       less than this value, since we prune empty ports. */
    size_t dev_port_count = ret;
    if (dev_port_count == 0) {
        dev_warn(&udev->dev, "Device has no ports\n");
    } else if (dev_port_count >= MAX_PORTS) {
        dev_warn(&udev->dev,
                 "Device has excessive number of ports; limiting to %d\n",
                 MAX_PORTS);
    }
    size_t empty_ports = 0;
    for (size_t i = 0; i < dev_port_count; i++) {
        if (valid_mask[i] == 0) empty_ports++;
    }
    size_t port_count = size_sub(dev_port_count, empty_ports);
    if (WARN(port_count == SIZE_MAX, "Lost track of number of ports\n")) {
        ret = -ENOTRECOVERABLE;
        goto err;
    }
    
    board = devm_kzalloc(&udev->dev, struct_size(board, ports, port_count),
                         GFP_KERNEL);
    if (board == NULL) {
        ret = -ENOMEM;
        goto err;
    }
    usb_set_intfdata(interface, board);
    mutex_init(&board->lock);
    board->udev = usb_get_dev(udev);
    board->valid_mask = valid_mask;
    board->port_count = port_count;
    struct avr_gpio_port *port = board->ports, *ports_end = port + port_count;
    // FIXME: ensure USB path and board serial number info get printed here
    dev_info(&udev->dev, "%s: adding board:\n", KBUILD_MODNAME);
    for (size_t port_id = 0; port_id < dev_port_count; port_id++) {
        if (valid_mask[port_id] == 0) continue;
        if (WARN(port == ports_end, "Exceeded allocated ports\n")) {
            ret = -ENOTRECOVERABLE;
            goto err;
        }
        
        // FIXME: add serial number to chip name
        char port_letter = 'A' + port_id;
        // "PI" is skipped by Atmel in their port numbering.
        if (port_letter >= 'I') port_letter++;
        port->gc.label = devm_kasprintf(&udev->dev, GFP_KERNEL,
                                        "%s P%c", KBUILD_MODNAME, port_letter);
        if (port->gc.label == NULL) {
            ret = -ENOMEM;
            goto err;
        }
        port->gc.parent = &udev->dev;
        port->gc.owner = THIS_MODULE;
        port->gc.base = -1;
        port->gc.direction_input = direction_input;
        port->gc.direction_output = direction_output;
        port->gc.get = get;
        port->gc.get_multiple = get_multiple;
        port->gc.set = set;
        port->gc.set_multiple = set_multiple;
        port->gc.set_config = set_config;
        port->gc.init_valid_mask = init_valid_mask;
        port->gc.can_sleep = true;
        port->gc.ngpio = 8;
        
        port->board = board;
        port->id = port_id;
        ret = fetch_port_state(port);
        if (ret < 0) goto err;
        
        unsigned int valid_count = hweight8(valid_mask[port_id]);
        dev_info(&udev->dev, "  %s: %u line%s\n",
                 port->gc.label, valid_count, valid_count == 1 ? "" : "s");
        
        ret = devm_gpiochip_add_data(&udev->dev, &port->gc, port);
        if (ret < 0) {
            dev_err(&udev->dev, "devm_gpiochip_add_data() failed (ret %d)\n",
                    ret);
            goto err;
        }
        
        port++;
    }
    return 0;

 err:
    devm_kfree(&udev->dev, valid_mask);
    if (board != NULL) board->valid_mask = NULL;
    free_board(board);
    return ret;
}

/* Must be called from initialization context (no locking is performed). */
static int
fetch_port_state(struct avr_gpio_port *port) {
    struct avr_gpio_board *board = port->board;
    struct usb_device *udev = board->udev;
    struct {
        uint8_t PORTx, DDRx;
    } __packed *buf = (void *) board->buf;
    if (WARN_ON(sizeof *buf != 2) ||
        WARN_ON(sizeof *buf > sizeof board->buf)) {
        return -ENOTRECOVERABLE;
    }
    int ret = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0),
                              MSG_PORT_DDR,
                              USB_RECIP_DEVICE | USB_TYPE_VENDOR | USB_DIR_IN,
                              0, port->id,
                              buf, sizeof *buf,
                              TIMEOUT);
    if (unlikely(ret < sizeof *buf)) {
        dev_err(&udev->dev, "MSG_PORT_DDR[%u] (IN) failed ", port->id);
        if (ret < 0) {
            pr_cont("(error %d)\n", -ret);
            return ret;
        } else {
            pr_cont("(short read, %d byte(s))\n", ret);
            return -EPROTO;
        }
    } else if (unlikely(ret > sizeof *buf)) {
        BUG(); // Buffer overflow!
        return -EOVERFLOW;
    }
    port->direction = buf->DDRx;
    port->value = buf->PORTx;
    port->bias_en = buf->PORTx & ~buf->DDRx;
    port->bias_total = 0;
    port->actual_PORTx_DDRx = buf->PORTx | ((int32_t) buf->DDRx << 8U);
    return 0;
}

static int
init_valid_mask(struct gpio_chip *gc, unsigned long *valid_mask,
                unsigned int ngpios) {
    struct avr_gpio_port *port = gpiochip_get_data(gc);
    *valid_mask = port->board->valid_mask[port->id];
    return 0;
}

static void
usb_disconnect(struct usb_interface *interface) {
    struct avr_gpio_board *board = usb_get_intfdata(interface);
    free_board(board);
}

static void
free_board(struct avr_gpio_board *board) {
    if (board == NULL) return;
    struct usb_device *udev = board->udev;
    for (size_t i = 0; i < board->port_count; i++) {
        if (board->ports[i].gc.gpiodev != NULL) {
            gpiochip_remove(&board->ports[i].gc);
        }
        /* DO NOT FREE board->ports[i].gc.label.

           devm_gpiochip_add_data() copies the label to its own self-managed
           dynamic memory, so it's not our memory to free.  Unfortunately, this
           isn't documented, so we can't take advantage of this behavior by
           passing a static buffer or something at initialization.  The only
           correct solution is to let devm clean up our original memory
           allocation eventually. */
    }
    devm_kfree(&udev->dev, board->valid_mask);
    devm_kfree(&udev->dev, board);
    usb_put_dev(udev);
}

static int
direction_input(struct gpio_chip *gc, unsigned int offset) {
    struct avr_gpio_port *port = gpiochip_get_data(gc);
    struct avr_gpio_board *board = port->board;
    mutex_lock(&board->lock);
    // Note: `offset` has been validated by the API.
    port->direction &= ~(1U << offset);
    int ret = update_PORTx_DDRx(port);
    mutex_unlock(&board->lock);
    return ret;
}

static int
direction_output(struct gpio_chip *gc, unsigned int offset, int value) {
    struct avr_gpio_port *port = gpiochip_get_data(gc);
    struct avr_gpio_board *board = port->board;
    mutex_lock(&board->lock);
    /* Note: `offset` has been validated by the API.
       
       It looks like gpiolib.c always passes a normalized `value` of 0 or 1,
       but that's not assured by the API. */
    port->direction |= 1U << offset;
    port->value &= ~(1U << offset);
    port->value |= !!value << offset;
    int ret = update_PORTx_DDRx(port);
    mutex_unlock(&board->lock);
    return ret;
}

static int
get(struct gpio_chip *gc, unsigned int offset) {
    int ret = get_raw(gc);
    if (ret < 0) return ret;
    return (ret >> offset) & 1;
}

static int
get_multiple(struct gpio_chip *gc, unsigned long *mask, unsigned long *bits) {
    int ret = get_raw(gc);
    if (ret < 0) return ret;
    // Only update bits requested via `*mask`.
    *bits = (*bits & ~*mask) | (ret & *mask);
    return 0;
}

static int
get_raw(struct gpio_chip *gc) {
    struct avr_gpio_port *port = gpiochip_get_data(gc);
    struct avr_gpio_board *board = port->board;
    struct usb_device *udev = board->udev;
    struct {
        uint8_t data;
    } __packed *buf = (void *) board->buf;
    if (WARN_ON(sizeof *buf != 1) ||
        WARN_ON(sizeof *buf > sizeof board->buf)) {
        return -ENOTRECOVERABLE;
    }
    mutex_lock(&board->lock);
    int ret = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0),
                              MSG_PIN,
                              USB_RECIP_DEVICE | USB_TYPE_VENDOR | USB_DIR_IN,
                              0, port->id,
                              buf, sizeof *buf,
                              TIMEOUT);
    mutex_unlock(&board->lock);
    if (unlikely(ret < sizeof *buf)) {
        dev_err(&udev->dev, "MSG_PIN[%u] (IN) failed ", port->id);
        if (ret < 0) {
            pr_cont("(error %d)\n", -ret);
            return ret;
        } else {
            pr_cont("(short read, %d byte(s))\n", ret);
            return -EPROTO;
        }
    } else if (unlikely(ret > sizeof *buf)) {
        BUG(); // Buffer overflow!
        return -EOVERFLOW;
    }
    return buf->data;
}

static void
set(struct gpio_chip *gc, unsigned int offset, int value) {
    unsigned long mask = 1U << offset, bits = (!!value) << offset;
    set_multiple(gc, &mask, &bits);
}

static void
set_multiple(struct gpio_chip *gc, unsigned long *mask, unsigned long *bits) {
    struct avr_gpio_port *port = gpiochip_get_data(gc);
    struct avr_gpio_board *board = port->board;
    uint8_t mask_ = *mask;
    mutex_lock(&board->lock);
    port->value &= ~mask_;
    port->value |= *bits & mask_;
    update_PORTx_DDRx(port);
    mutex_unlock(&board->lock);
}

static int
set_config(struct gpio_chip *gc, unsigned int offset, unsigned long config) {
    struct avr_gpio_port *port = gpiochip_get_data(gc);
    struct avr_gpio_board *board = port->board;
    uint8_t mask = 1U << offset;
    enum pin_config_param param = pinconf_to_config_param(config);
    uint32_t arg = pinconf_to_config_argument(config);
    int ret;
    mutex_lock(&board->lock);
    switch (param) {
    case PIN_CONFIG_BIAS_DISABLE:
        port->bias_en &= ~mask;
        break;
    case PIN_CONFIG_BIAS_HIGH_IMPEDANCE:
        port->bias_en &= ~mask;
        port->direction &= ~mask;
        break;
    case PIN_CONFIG_BIAS_PULL_PIN_DEFAULT:
        if (arg == 0) {
            port->bias_en |= mask;
            port->bias_total &= ~mask;
        } else {
            ret = 0;
            goto end;
        }
        break;
    case PIN_CONFIG_BIAS_PULL_UP:
        port->bias_en |= mask;
        if (arg != 0) port->bias_total &= ~mask;
        else port->bias_total |= mask;
        break;
    default:
        ret = -ENOTSUPP;
        goto end;
    }
    ret = update_PORTx_DDRx(port);
 end:
    mutex_unlock(&board->lock);
    return ret;
}

static int
update_PORTx_DDRx(struct avr_gpio_port *port) {
    struct avr_gpio_board *board = port->board;
    struct usb_device *udev = board->udev;
    uint8_t
        DDRx = port->direction | (port->bias_en & port->bias_total),
        PORTx = (port->direction & port->value) | port->bias_en;
    int32_t PORTx_DDRx = PORTx | ((int32_t) DDRx << 8U);
    if (PORTx_DDRx == port->actual_PORTx_DDRx) return 0;
    int ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
                              MSG_PORT_DDR,
                              USB_RECIP_DEVICE | USB_TYPE_VENDOR | USB_DIR_OUT,
                              PORTx_DDRx,
                              port->id,
                              NULL, 0,
                              TIMEOUT);
    if (unlikely(ret < 0)) {
        dev_err(&udev->dev,
                "MSG_PORT_DDR[%u] = 0x%04X (OUT) failed (error %d)\n",
                port->id, PORTx_DDRx, -ret);
        /* We don't know the device's PORTx and DDRx state, so invalidate our
           local copies to force a future update. */
        port->actual_PORTx_DDRx = -1;
        return ret;
    }
    WARN_ON_ONCE(ret > 0);
    port->actual_PORTx_DDRx = PORTx_DDRx;
    return 0;
}
