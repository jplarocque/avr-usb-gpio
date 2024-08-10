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
    /* Mask of usable I/O lines.  Used only by init_valid_mask(), where it must
       be looked up later due to the gpiolib API.  (There's no way to pass the
       valid-mask directly during gpiochip initialization.)
       
       The device firmware is responsible for enforcing the valid mask (writes
       are no-ops, reads return 0 or some other constant data).  However, it
       looks like gpiolib also enforces it for us. */
    uint8_t valid_mask;
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
       (PORTx | (DDRx << 8)).  May be set to a value outside [0, (uint16_t) -1]
       to invalidate any previously-known value upon error. */
    int32_t actual_PORTx_DDRx;
};

struct avr_gpio_board {
    struct mutex lock;
    struct usb_interface *intf;
    uint8_t buf[2]; // Misc USB transfer buffer; can't be stack-allocated
    size_t port_count;
    /* Array of available ports which have a non-zero number of valid I/O
       lines.  This is indexed contiguously up to .port_count (exclusively) in
       initialization order.  NEVER index by port ID. */
    struct avr_gpio_port ports[];
};

static int
usb_probe(struct usb_interface *intf, const struct usb_device_id *id);
static int
fetch_port_state(struct avr_gpio_port *port);
static int
init_valid_mask(struct gpio_chip *gc, unsigned long *valid_mask,
                unsigned int ngpios);
static void
usb_disconnect(struct usb_interface *intf);
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
static int
avr_gpio_msg(struct usb_interface *intf, uint8_t request,
             const char *request_fmt, uint8_t dir,
             uint16_t value, uint16_t index,
             void *data, size_t min_size, size_t max_size);
static inline int
avr_gpio_msg2_in(struct usb_interface *intf, uint8_t request,
                 const char *request_fmt,
                 uint16_t value, uint16_t index,
                 void *data, size_t min_size, size_t max_size);

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
usb_probe(struct usb_interface *intf, const struct usb_device_id *id) {
    struct usb_device *udev = interface_to_usbdev(intf);
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
    void *devg = devres_open_group(&intf->dev, NULL, GFP_KERNEL);
    if (devg == NULL) return -ENOMEM;
    struct avr_gpio_board *board = NULL;
    uint8_t valid_mask[MAX_PORTS], line_count[MAX_PORTS];
    /* From this point forward, all error handling must go through `goto err`
       in order to ensure matching usb_put_intf() call. */

    ret = avr_gpio_msg2_in(intf, MSG_VALID_MASK, "MSG_VALID_MASK", 0, 0,
                           valid_mask, 0, sizeof valid_mask);
    if (ret < 0) goto err;
    /* Number of ports as reported by the device.  port_count may end up being
       less than this value, since we prune empty ports. */
    size_t dev_port_count = ret;
    if (dev_port_count == 0) {
        dev_warn(&intf->dev, "Device has no ports\n");
    } else if (dev_port_count >= MAX_PORTS) {
        dev_warn(&intf->dev,
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

    ret = avr_gpio_msg2_in(intf, MSG_LINE_COUNT, "MSG_LINE_COUNT", 0, 0,
                           line_count, 0, sizeof line_count);
    if (ret < 0) goto err;
    if (ret != dev_port_count) {
        dev_err(&intf->dev,
                "Port count mismatch: %zu per MSG_VALID_MASK; %d per MSG_LINE_COUNT\n",
                dev_port_count, ret);
        ret = -EPROTO;
        goto err;
    }
    
    board = devm_kzalloc(&intf->dev, struct_size(board, ports, port_count),
                         GFP_KERNEL);
    if (board == NULL) {
        ret = -ENOMEM;
        goto err;
    }
    usb_set_intfdata(intf, board);
    mutex_init(&board->lock);
    board->intf = usb_get_intf(intf);
    board->port_count = port_count;
    struct avr_gpio_port *port = board->ports, *ports_end = port + port_count;
    // FIXME: ensure USB path and board serial number info get printed here
    dev_info(&intf->dev, "adding board:\n");
    for (size_t port_id = 0; port_id < dev_port_count; port_id++) {
        if (line_count[port_id] > 8) {
            dev_err(&intf->dev,
                    "MSG_LINE_COUNT (IN) failed: invalid response %u\n",
                    line_count[port_id]);
            ret = -EPROTO;
            goto err;
        }
        if (valid_mask[port_id] == 0) continue;
        if (valid_mask[port_id] & ~((1U << line_count[port_id]) - 1)) {
            dev_err(&intf->dev,
                    "MSG_VALID_MASK 0x%02X includes bits outside of MSG_LINE_COUNT %u\n",
                    valid_mask[port_id], line_count[port_id]);
            ret = -EPROTO;
            goto err;
        }
        if (WARN(port == ports_end, "Exceeded allocated ports\n")) {
            ret = -ENOTRECOVERABLE;
            goto err;
        }
        
        // FIXME: add serial number to chip name
        char port_letter = 'A' + port_id;
        // "PI" is skipped by Atmel in their port numbering.
        if (port_letter >= 'I') port_letter++;
        port->gc.label = devm_kasprintf(&intf->dev, GFP_KERNEL,
                                        "%s P%c", KBUILD_MODNAME, port_letter);
        if (port->gc.label == NULL) {
            ret = -ENOMEM;
            goto err;
        }
        port->gc.parent = &intf->dev;
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
        port->gc.ngpio = line_count[port_id];
        
        port->board = board;
        port->id = port_id;
        port->valid_mask = valid_mask[port_id];
        ret = fetch_port_state(port);
        if (ret < 0) goto err;
        
        unsigned int valid_count = hweight8(valid_mask[port_id]);
        dev_info(&intf->dev, "  %s: %u line%s\n",
                 port->gc.label, valid_count, valid_count == 1 ? "" : "s");
        
        /* After this function returns successfully, the gpiochip is available,
           so all data required by our gpio functions must be filled into
           *board and *port before this call. */
        ret = devm_gpiochip_add_data(&intf->dev, &port->gc, port);
        if (ret < 0) {
            dev_err(&intf->dev, "devm_gpiochip_add_data() failed (ret %d)\n",
                    ret);
            goto err;
        }
        
        port++;
    }
    devres_remove_group(&intf->dev, devg);
    return 0;

 err:
    if (board != NULL) usb_put_intf(board->intf);
    devres_release_group(&intf->dev, devg);
    return ret;
}

/* Must be called from initialization context (no locking is performed). */
static int
fetch_port_state(struct avr_gpio_port *port) {
    struct avr_gpio_board *board = port->board;
    struct usb_interface *intf = board->intf;
    struct {
        uint8_t PORTx, DDRx;
    } __packed *buf = (void *) board->buf;
    if (WARN_ON(sizeof *buf != 2) ||
        WARN_ON(sizeof *buf > sizeof board->buf)) {
        return -ENOTRECOVERABLE;
    }
    int ret = avr_gpio_msg(intf, MSG_PORT_DDR, "MSG_PORT_DDR[%u]", USB_DIR_IN,
                           0, port->id,
                           buf, sizeof *buf, sizeof *buf);
    if (ret < 0) return ret;
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
    *valid_mask = port->valid_mask;
    return 0;
}

static void
usb_disconnect(struct usb_interface *intf) {
    struct avr_gpio_board *board = usb_get_intfdata(intf);
    dev_info(&intf->dev, "board removed:\n");
    for (size_t i = 0; i < board->port_count; i++) {
        dev_info(&intf->dev, "  %s\n", board->ports[i].gc.label);
    }
    usb_put_intf(board->intf);
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
    struct usb_interface *intf = board->intf;
    struct {
        uint8_t data;
    } __packed *buf = (void *) board->buf;
    if (WARN_ON(sizeof *buf != 1) ||
        WARN_ON(sizeof *buf > sizeof board->buf)) {
        return -ENOTRECOVERABLE;
    }
    mutex_lock(&board->lock);
    int ret = avr_gpio_msg(intf, MSG_PIN, "MSG_PIN[%u]", USB_DIR_IN,
                           0, port->id,
                           buf, sizeof *buf, sizeof *buf);
    mutex_unlock(&board->lock);
    if (ret < 0) return ret;
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
    struct usb_interface *intf = board->intf;
    uint8_t
        DDRx = port->direction | (port->bias_en & port->bias_total),
        PORTx = (port->direction & port->value) | port->bias_en;
    int32_t PORTx_DDRx = PORTx | ((int32_t) DDRx << 8U);
    if (PORTx_DDRx == port->actual_PORTx_DDRx) return 0;
    int ret = avr_gpio_msg(intf, MSG_PORT_DDR, "MSG_PORT_DDR[%u] = 0x%04X",
                           USB_DIR_OUT,
                           PORTx_DDRx, port->id,
                           NULL, 0, 0);
    if (unlikely(ret < 0)) {
        /* We don't know the device's PORTx and DDRx state, so invalidate our
           local copies to force a future update. */
        port->actual_PORTx_DDRx = -1;
        return ret;
    }
    port->actual_PORTx_DDRx = PORTx_DDRx;
    return 0;
}

static int
avr_gpio_msg(struct usb_interface *intf, uint8_t request,
             const char *request_fmt, uint8_t dir,
             uint16_t value, uint16_t index,
             void *data, size_t min_size, size_t max_size) {
    struct usb_device *udev = interface_to_usbdev(intf);
    bool dir_out;
    unsigned int pipe;
    switch (dir) {
    case USB_DIR_OUT:
        dir_out = true;
        pipe = usb_sndctrlpipe(udev, 0);
        break;
    case USB_DIR_IN:
        dir_out = false;
        pipe = usb_rcvctrlpipe(udev, 0);
        break;
    default:
        return -EINVAL;
    }
    uint8_t request_type = dir | USB_RECIP_DEVICE | USB_TYPE_VENDOR;
    int ret = usb_control_msg(udev, pipe, request, request_type,
                              value, index,
                              data, min(max_size, (size_t) (uint16_t) -1),
                              TIMEOUT);
    char request_label[128], detail[128];
    const char *dir_label;
    if (unlikely(ret < 0)) {
        snprintf(detail, sizeof detail, "error %d", -ret);
        goto err;
    } else if (unlikely((unsigned int) ret > max_size)) {
        if (! dir_out) BUG(); // Buffer overflow!
        snprintf(detail, sizeof detail,
                 "%s %d byte(s) past end of buffer (%zu byte(s))",
                 dir_out ? "transmit read" : "receive overflowed",
                 ret, max_size);
        ret = -EOVERFLOW;
        goto err;
    } else if (unlikely((unsigned int) ret < min_size)) {
        snprintf(detail, sizeof detail, "short %s, %d byte(s)",
                 dir_out ? "write" : "read", ret);
        ret = -EPROTO;
        goto err;
    }
    return ret;
    
 err:
    snprintf(request_label, sizeof request_label,
             request_fmt, index, value);
    dir_label = dir_out ? "OUT" : "IN";
    dev_err(&intf->dev, "%s (%s) failed (%s)\n",
            request_label, dir_label, detail);
    return ret;
}

/* Like avr_gpio_msg(), but input only, into any memory (not just DMAable
   memory).  Allocates and frees temporary dynamic memory. */
static inline int
avr_gpio_msg2_in(struct usb_interface *intf, uint8_t request,
                 const char *request_fmt,
                 uint16_t value, uint16_t index,
                 void *data, size_t min_size, size_t max_size) {
    char *buf = kzalloc(max_size, GFP_KERNEL);
    if (buf == NULL) return -ENOMEM;
    int ret = avr_gpio_msg(intf, request, request_fmt, USB_DIR_IN,
                           value, index, buf, min_size, max_size);
    if (ret < 0) {
        // Error; no-op.
    } else {
        memcpy(data, buf, ret);
    }
    kfree(buf);
    return ret;
}
