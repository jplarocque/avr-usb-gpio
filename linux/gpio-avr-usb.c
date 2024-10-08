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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/overflow.h>
#include <linux/version.h>

#include <linux/device.h>
#include <linux/usb.h>
#include <linux/slab.h>
#include <linux/gpio/driver.h>

#include "protocol.h"
#include "usb_ids.h"

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Jean-Paul Larocque <jpl@ftlvisual.com>");
MODULE_DESCRIPTION("AVR USB GPIO driver");
MODULE_VERSION("0.1");

/* Old versions of gpiolib can crash the system when GPIO chips are dynamically
   removed.  Refuse to build on old kernels. */
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 8, 0)
#  error "Linux kernel 6.8 or higher required; 6.9 or higher preferred"
#elif LINUX_VERSION_CODE < KERNEL_VERSION(6, 9, 0)
#  warning "Scary (but benign) warnings are logged upon unplug for kernel versions older than 6.9"
#endif

/* Actual maximum is expected to be 11 (PA..PL on e.g. ATmega640, with "PI"
   skipped due to Atmel port numbering). */
#define MAX_PORTS 25
#define RETRIES 5 // FIXME: make configurable
/* How long to wait between retries.  Only applies after the second attempt
   (the first retry is immediate). */
#define RETRY_DELAY_ms 5 // FIXME: make configurable
#define TIMEOUT_ms 100 // FIXME: make configurable

struct board;
struct port;

struct port {
    struct gpio_chip gc;
    struct board *board;
    /* Sequential port number (index into .ports[] of `struct board`). */
    uint8_t idx;
    /* For identifying this port on the wire (NOT an index into .ports[] of
       `struct board`). */
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

struct board {
    struct mutex lock;
    struct usb_interface *intf;
    uint8_t buf[2]; // Misc USB transfer buffer; can't be stack-allocated
    size_t port_count;
    /* Array of available ports which have a non-zero number of valid I/O
       lines.  This is indexed contiguously up to .port_count (exclusively) in
       initialization order.  NEVER index by port ID. */
    struct port ports[];
};

static int
usb_probe(struct usb_interface *intf, const struct usb_device_id *id);
static int
init_valid_mask(struct gpio_chip *gc, unsigned long *valid_mask,
                unsigned int ngpios);
static ssize_t
show_sysfs_attr(struct device *dev, struct device_attribute *attr,
                char *buf);
static int
gpio_chip_match_device(struct gpio_chip *gc, const void *data);
static void
summarize_gpiochip_info(struct gpio_chip *gc,
                        char *names, size_t names_size,
                        unsigned int *valid_lines,
                        unsigned int *invalid_lines);
static void
usb_disconnect(struct usb_interface *intf);
static int
request(struct gpio_chip *gc, unsigned int offset);
static int
get_direction(struct gpio_chip *gc, unsigned int offset);
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
read_PORTx_DDRx(struct port *port);
static int
write_PORTx_DDRx(struct port *port);
static int
xfer(struct port *port, struct usb_interface *intf,
     uint8_t request, const char *request_fmt,
     uint8_t dir, uint16_t value, uint16_t index,
     void *data, size_t min_size, size_t max_size);
static void
xfer_printk(const char *level, struct port *port, struct usb_interface *intf,
            const char *request_fmt,
            uint8_t dir, uint16_t value, uint16_t index,
            const char *detail_fmt, ...);
static void
xfer_printk_2(const char *level, struct port *port, struct usb_interface *intf,
              uint8_t dir, uint16_t value, uint16_t index,
              struct va_format *detail_vaf, const char *request_fmt, ...);
static inline int
xfer2_in(struct usb_interface *intf, uint8_t request, const char *request_fmt,
         uint16_t value, uint16_t index,
         void *data, size_t min_size, size_t max_size);

#define xfer_err(port, intf, request_fmt, dir, value, index,            \
                 detail_fmt, ...)                                       \
    do {                                                                \
        xfer_printk(KERN_ERR, (port), (intf), (request_fmt), (dir),     \
                    (value), (index), (detail_fmt), ##__VA_ARGS__);     \
    } while (false)
#define xfer_warn(port, intf, request_fmt, dir, value, index,           \
                  detail_fmt, ...)                                      \
    do {                                                                \
        xfer_printk(KERN_WARNING, (port), (intf), (request_fmt), (dir), \
                    (value), (index), (detail_fmt), ##__VA_ARGS__);     \
    } while (false)

static struct usb_device_id id_table[] = {
    {USB_DEVICE(USB_VENDOR_ID, USB_DEVICE_ID)},
    {},
};
MODULE_DEVICE_TABLE(usb, id_table);

static struct usb_driver gpio_avr_usb = {
    .name = "gpio-avr-usb",
    .id_table = id_table,
    .probe = usb_probe,
    .disconnect = usb_disconnect,
};
module_usb_driver(gpio_avr_usb);

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
    struct board *board = NULL;
    uint8_t valid_mask[MAX_PORTS], line_count[MAX_PORTS];
    /* From this point forward, all error handling must go through `goto err`
       in order to ensure matching usb_put_intf() call. */

    ret = xfer2_in(intf, MSG_VALID_MASK, "MSG_VALID_MASK", 0, 0,
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

    ret = xfer2_in(intf, MSG_LINE_COUNT, "MSG_LINE_COUNT", 0, 0,
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
    if (board == NULL) goto nomem;
    usb_set_intfdata(intf, board);
    mutex_init(&board->lock);
    board->intf = usb_get_intf(intf);
    board->port_count = port_count;
    size_t port_idx = 0;
    unsigned int valid_lines = 0, invalid_lines = 0;
    // FIXME: ensure USB path and board serial number info get printed here
    dev_info(&intf->dev, "adding board:\n");
    for (size_t port_id = 0; port_id < dev_port_count; port_id++) {
        const char *names[8];
        if (line_count[port_id] > ARRAY_SIZE(names)) {
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

        if (WARN(port_idx >= port_count, "Exceeded allocated ports\n")) {
            ret = -ENOTRECOVERABLE;
            goto err;
        }
        
        // FIXME: add serial number to chip name
        char port_letter = 'A' + port_id;
        // "PI" is skipped by Atmel in their port numbering.
        if (port_letter >= 'I') port_letter++;
        
        struct port *port = board->ports + port_idx;
        port->gc.label = devm_kasprintf(&intf->dev, GFP_KERNEL,
                                        "P%c", port_letter);
        if (port->gc.label == NULL) goto nomem;
        port->gc.parent = &intf->dev;
        port->gc.owner = THIS_MODULE;
        port->gc.base = -1;
        port->gc.request = request;
        port->gc.get_direction = get_direction;
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
        for (size_t i = 0; i < line_count[port_id]; i++) {
            names[i] = devm_kasprintf(&intf->dev, GFP_KERNEL, "P%c%zu",
                                      port_letter, i);
            if (names[i] == NULL) goto nomem;
        }
        port->gc.names = names;
        
        port->board = board;
        port->idx = port_idx;
        port->id = port_id;
        port->valid_mask = valid_mask[port_id];
        ret = read_PORTx_DDRx(port);
        if (ret < 0) goto err;
        
        /* After this function returns successfully, the gpiochip is available,
           so all data required by our gpio functions must be filled into
           *board and *port before this call. */
        ret = devm_gpiochip_add_data(&intf->dev, &port->gc, port);
        if (ret < 0) {
            dev_err(&intf->dev, "devm_gpiochip_add_data() failed (ret %d)\n",
                    ret);
            goto err;
        }
        
        char names_concat[33];
        summarize_gpiochip_info(&port->gc, names_concat, sizeof names_concat,
                                &valid_lines, &invalid_lines);
        dev_info(&intf->dev, "  %s \"%s\":%s\n",
                 dev_name(gpio_device_to_device(port->gc.gpiodev)),
                 port->gc.label, names_concat);
        
        port_idx++;
    }
    dev_info(&intf->dev,
             "%zu port(s), %u line(s) available, %u line(s) reserved\n",
             port_count, valid_lines, invalid_lines);
    devres_remove_group(&intf->dev, devg);
    return 0;
    
 nomem:
    ret = -ENOMEM;
    goto err;
    
 err:
    if (board != NULL) usb_put_intf(board->intf);
    devres_release_group(&intf->dev, devg);
    return ret;
}

static struct device_attribute
dev_attr_chip_idx = __ATTR(chip_idx, S_IRUGO, show_sysfs_attr, NULL),
dev_attr_chip_id = __ATTR(chip_id, S_IRUGO, show_sysfs_attr, NULL),
dev_attr_chip_name = __ATTR(chip_name, S_IRUGO, show_sysfs_attr, NULL);
static struct attribute *avr_usb_gpio_attrs[] = {
    &dev_attr_chip_idx.attr,
    &dev_attr_chip_id.attr,
    &dev_attr_chip_name.attr,
    NULL,
};
ATTRIBUTE_GROUPS(avr_usb_gpio);

static int
init_valid_mask(struct gpio_chip *gc, unsigned long *valid_mask,
                unsigned int ngpios) {
    struct port *port = gpiochip_get_data(gc);
    *valid_mask = port->valid_mask;

    /* It's kind of a hack to put this here: gc->init_valid_mask() is one of
       the few places we can hook between gc->gpiodev initialization and
       device_initialize().  However, there's not really anything in the API
       that seems to assure this behavior.  It'd be better if `struct
       gpio_chip` had some explicit pre-initialize hook function pointer. */
    struct device *gpiodev_dev = gpio_device_to_device(gc->gpiodev);
    gpiodev_dev->groups = avr_usb_gpio_groups;
    
    return 0;
}

static ssize_t
show_sysfs_attr(struct device *dev, struct device_attribute *attr,
                char *buf) {
    /* We need to use `dev` to look up the correct `struct port` in order to
       find the port number.
       
         - We can't use the (perfect) function to_gpio_device() to get closer
           by finding the owning `struct gpio_device`, since it's statically
           defined in a private header.
         
         - We can't use container_of(dev, struct gpio_device, dev) to get
           closer by finding the owning `struct gpio_device`, since the layout
           of `struct gpio_device` is hidden in a private header.
         
         - We could use dev_set_drvdata()/dev_get_drvdata(), but since we don't
           own the `struct device` (gpiolib does), this would be foolish.
         
         - Therefore, we need to use the pointer value `dev` as a lookup key in
           a global search.
    */
    struct gpio_device *gdev = gpio_device_find(dev, gpio_chip_match_device);
    if (WARN_ON_ONCE(gdev == NULL)) return -ENODEV;
    struct gpio_chip *gc = gpio_device_get_chip(gdev);
    struct port *port = gpiochip_get_data(gc);
    ssize_t out;
    if (attr == &dev_attr_chip_idx) {
        out = sysfs_emit(buf, "%u\n", port->idx);
    } else if (attr == &dev_attr_chip_id) {
        out = sysfs_emit(buf, "%u\n", port->id);
    } else if (attr == &dev_attr_chip_name) {
        out = sysfs_emit(buf, "%s\n", port->gc.label);
    } else {
        out = -EINVAL;
    }
    gpio_device_put(gdev);
    return out;
}

static int
gpio_chip_match_device(struct gpio_chip *gc, const void *data) {
    const struct device *dev = data;
    return gpio_device_to_device(gc->gpiodev) == dev;
}

static void
summarize_gpiochip_info(struct gpio_chip *gc,
                        char *names, size_t names_size,
                        unsigned int *valid_lines,
                        unsigned int *invalid_lines) {
    struct port *port = gpiochip_get_data(gc);
    if (! WARN_ON_ONCE(names_size < 1)) *names = '\0';
    char *names_end = names + names_size;
    for (size_t i = 0; i < gc->ngpio; i++) {
        const char *name;
        if (port->valid_mask & (1U << i)) {
            name = gc->names[i];
            if (valid_lines != NULL) (*valid_lines)++;
        } else {
            name = "_";
            if (invalid_lines != NULL) (*invalid_lines)++;
        }
        /* Don't break if we run out of room, since we want to keep counting
           lines. */
        if (! WARN_ON_ONCE(names_end - names < 2)) *names++ = ' ';
        else names = names_end;
        ssize_t ret = strscpy(names, name, names_end - names);
        if (WARN_ON_ONCE(ret < 0)) names = names_end;
        else names += ret;
    }
}

static void
usb_disconnect(struct usb_interface *intf) {
    struct board *board = usb_get_intfdata(intf);
    dev_info(&intf->dev, "board removed:\n");
    for (size_t i = 0; i < board->port_count; i++) {
        struct gpio_chip *gc = &board->ports[i].gc;
        dev_info(&intf->dev, "  %s \"%s\"\n",
                 dev_name(gpio_device_to_device(gc->gpiodev)),
                 gc->label);
    }
    usb_put_intf(board->intf);
}

static int
request(struct gpio_chip *gc, unsigned int offset) {
    /* No-op.  However, this triggers behavior in gpiolib which differs from
       what would happen if .request were NULL.

       Specifically, the valid_mask is only honored upon open if there is a
       .request function defined for a gpiochip.  This is probably a bug, and
       is undocumented. */
    return 0;
}

static int
get_direction(struct gpio_chip *gc, unsigned int offset) {
    struct port *port = gpiochip_get_data(gc);
    // Note: `offset` has been validated by the API.
    return (port->direction & (1U << offset)
            ? GPIO_LINE_DIRECTION_OUT : GPIO_LINE_DIRECTION_IN);
}

static int
direction_input(struct gpio_chip *gc, unsigned int offset) {
    struct port *port = gpiochip_get_data(gc);
    struct board *board = port->board;
    mutex_lock(&board->lock);
    // Note: `offset` has been validated by the API.
    port->direction &= ~(1U << offset);
    int ret = write_PORTx_DDRx(port);
    mutex_unlock(&board->lock);
    return ret;
}

static int
direction_output(struct gpio_chip *gc, unsigned int offset, int value) {
    struct port *port = gpiochip_get_data(gc);
    struct board *board = port->board;
    mutex_lock(&board->lock);
    /* Note: `offset` has been validated by the API.
       
       It looks like gpiolib.c always passes a normalized `value` of 0 or 1,
       but that's not assured by the API. */
    port->direction |= 1U << offset;
    port->value &= ~(1U << offset);
    port->value |= !!value << offset;
    int ret = write_PORTx_DDRx(port);
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
    struct port *port = gpiochip_get_data(gc);
    struct board *board = port->board;
    struct usb_interface *intf = board->intf;
    struct {
        uint8_t data;
    } __packed *buf = (void *) board->buf;
    if (WARN_ON(sizeof *buf != 1) ||
        WARN_ON(sizeof *buf > sizeof board->buf)) {
        return -ENOTRECOVERABLE;
    }
    mutex_lock(&board->lock);
    int ret = xfer(port, intf, MSG_PIN, "MSG_PIN[%u]", USB_DIR_IN,
                   0, port->id,
                   buf, sizeof *buf, sizeof *buf);
    mutex_unlock(&board->lock);
    if (ret < 0) return ret;
    return buf->data;
}

static void
set(struct gpio_chip *gc, unsigned int offset, int value) {
    unsigned long mask = 1U << offset,
        bits = (unsigned int) (!!value) << offset;
    set_multiple(gc, &mask, &bits);
}

static void
set_multiple(struct gpio_chip *gc, unsigned long *mask, unsigned long *bits) {
    struct port *port = gpiochip_get_data(gc);
    struct board *board = port->board;
    uint8_t mask_ = *mask;
    mutex_lock(&board->lock);
    port->value &= ~mask_;
    port->value |= *bits & mask_;
    write_PORTx_DDRx(port);
    mutex_unlock(&board->lock);
}

static int
set_config(struct gpio_chip *gc, unsigned int offset, unsigned long config) {
    struct port *port = gpiochip_get_data(gc);
    struct board *board = port->board;
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
    ret = write_PORTx_DDRx(port);
 end:
    mutex_unlock(&board->lock);
    return ret;
}

/*
 * Low-level functions.  The caller is responsible for protecting against
 * concurrent access to the `struct port` being operated upon, either by
 * locking the mutex or by calling at probe time.
 */

static int
read_PORTx_DDRx(struct port *port) {
    struct board *board = port->board;
    struct usb_interface *intf = board->intf;
    struct {
        uint8_t PORTx, DDRx;
    } __packed *buf = (void *) board->buf;
    if (WARN_ON(sizeof *buf != 2) ||
        WARN_ON(sizeof *buf > sizeof board->buf)) {
        return -ENOTRECOVERABLE;
    }
    int ret = xfer(port, intf, MSG_PORT_DDR, "MSG_PORT_DDR[%u]", USB_DIR_IN,
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
write_PORTx_DDRx(struct port *port) {
    struct board *board = port->board;
    struct usb_interface *intf = board->intf;
    uint8_t
        DDRx = port->direction | (port->bias_en & port->bias_total),
        PORTx = (port->direction & port->value) | port->bias_en;
    int32_t PORTx_DDRx = PORTx | ((int32_t) DDRx << 8U);
    if (PORTx_DDRx == port->actual_PORTx_DDRx) return 0;
    int ret = xfer(port, intf, MSG_PORT_DDR, "MSG_PORT_DDR[%u] = 0x%04X",
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
xfer(struct port *port, struct usb_interface *intf,
     uint8_t request, const char *request_fmt,
     uint8_t dir,
     uint16_t value, uint16_t index,
     void *data, size_t min_size, size_t max_size) {
    struct usb_device *udev = interface_to_usbdev(intf);
    unsigned int pipe;
    switch (dir) {
    case USB_DIR_OUT:
        pipe = usb_sndctrlpipe(udev, 0);
        break;
    case USB_DIR_IN:
        pipe = usb_rcvctrlpipe(udev, 0);
        break;
    default:
        return -EINVAL;
    }
    uint8_t request_type = dir | USB_RECIP_DEVICE | USB_TYPE_VENDOR;
    int tries = 0, ret;
 retry:
    ret = usb_control_msg(udev, pipe, request, request_type,
                          value, index,
                          data, min(max_size, (size_t) (uint16_t) -1),
                          TIMEOUT_ms);
    tries++;
    if (unlikely(ret < 0)) {
        switch (-ret) {
        case EPROTO: // Bitstuff error, bus turnaround timeout, or unknown
        case EILSEQ: // CRC error, bus turnaround timeout, or unknown
        case ETIME: // Bus turnaround timeout
        case ETIMEDOUT: // High-level timeout
        case EPIPE: /* Endpoint stalled; firmware uses this to signal CRC
                       mismatch outside standard USB handshake packets */
        case ECOMM: // Received too fast
        case ENOSR: // Couldn't send fast enough
            if (tries < RETRIES) {
                xfer_warn(port, intf, request_fmt, dir, value, index,
                          "(error %d, try %d); retrying",
                          -ret, tries);
                if (tries > 1) msleep(RETRY_DELAY_ms);
                goto retry;
            } else {
                xfer_err(port, intf, request_fmt, dir, value, index,
                         "(error %d, try %d); giving up",
                         -ret, tries);
            }
            break;
        default:
            xfer_err(port, intf, request_fmt, dir, value, index,
                             "(error %d)", -ret);
        }
    } else if (unlikely((unsigned int) ret > max_size)) {
        if (dir == USB_DIR_IN) BUG(); // Buffer overflow!
        xfer_err(port, intf, request_fmt, dir, value, index,
                 "(%s %d byte(s) %s %zu-byte buffer)",
                 dir == USB_DIR_OUT ? "transmit read" :
                 dir == USB_DIR_IN ? "receive overflowed" : "???",
                 ret,
                 dir == USB_DIR_OUT ? "from" :
                 dir == USB_DIR_IN ? "into" : "???",
                 max_size);
        ret = -EOVERFLOW;
    } else if (unlikely((unsigned int) ret < min_size)) {
        xfer_err(port, intf, request_fmt, dir, value, index,
                 "(short %s, %d byte(s))",
                 dir == USB_DIR_OUT ? "write" :
                 dir == USB_DIR_IN ? "read" : "???",
                 ret);
        ret = -EPROTO;
    }
    return ret;
}

static void
xfer_printk(const char *level, struct port *port, struct usb_interface *intf,
            const char *request_fmt,
            uint8_t dir, uint16_t value, uint16_t index,
            const char *detail_fmt, ...) {
    va_list ap;
    struct va_format detail_vaf = {
        .fmt = detail_fmt,
        .va = &ap,
    };
    va_start(ap, detail_fmt);
    xfer_printk_2(level, port, intf, dir, value, index,
                  &detail_vaf, request_fmt, index, value);
    va_end(ap);
}

/* Hack to capture the arguments for request_fmt for use with
   va_start()/va_end(). */
static void
xfer_printk_2(const char *level, struct port *port, struct usb_interface *intf,
              uint8_t dir, uint16_t value, uint16_t index,
              struct va_format *detail_vaf, const char *request_fmt, ...) {
    const char *gpiodev_name = NULL;
    if (port != NULL && port->gc.gpiodev != NULL) {
        struct device *gpiodev_dev = gpio_device_to_device(port->gc.gpiodev);
        if (gpiodev_dev != NULL) {
            gpiodev_name = dev_name(gpiodev_dev);
        }
    }
    
    va_list ap;
    struct va_format request_vaf = {
        .fmt = request_fmt,
        .va = &ap,
    };
    va_start(ap, request_fmt);
    dev_printk(level, &intf->dev, "%s%s%pV (%s) failed %pV\n",
               gpiodev_name != NULL ? gpiodev_name : "",
               gpiodev_name != NULL ? ": " : "",
               &request_vaf,
               dir == USB_DIR_OUT ? "OUT" :
               dir == USB_DIR_IN ? "IN" : "???",
               detail_vaf);
    va_end(ap);
}

/* Like xfer(), but input only, during board initialization, into any memory
   (not just DMAable memory).  Allocates and frees temporary dynamic memory. */
static inline int
xfer2_in(struct usb_interface *intf, uint8_t request, const char *request_fmt,
         uint16_t value, uint16_t index,
         void *data, size_t min_size, size_t max_size) {
    char *buf = kzalloc(max_size, GFP_KERNEL);
    if (buf == NULL) return -ENOMEM;
    int ret = xfer(NULL, intf, request, request_fmt, USB_DIR_IN,
                   value, index, buf, min_size, max_size);
    if (ret < 0) {
        // Error; no-op.
    } else {
        memcpy(data, buf, ret);
    }
    kfree(buf);
    return ret;
}
