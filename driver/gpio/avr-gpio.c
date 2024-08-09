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

struct avr_gpio {
    struct mutex lock;
    struct usb_device *udev;
    struct gpio_chip chip; //this is our GPIO chip
    u32 timeout;
    u8 buf[1];
};

static int
usb_probe(struct usb_interface *interface, const struct usb_device_id *id);
static void
usb_disconnect(struct usb_interface *interface);
static int
direction_input(struct gpio_chip *chip, unsigned int offset);
static int
direction_output(struct gpio_chip *chip, unsigned int offset, int value);
static int
get(struct gpio_chip *chip, unsigned int offset);
static void
set(struct gpio_chip *chip, unsigned int offset, int value);

static struct usb_device_id id_table[] = {
    { USB_DEVICE(USB_VENDOR_ID, USB_DEVICE_ID) },
    {},
};

MODULE_DEVICE_TABLE(usb, id_table);

static struct usb_driver avr_gpio_driver = {
    .name = "avr_gpio",
    .id_table = id_table,
    .probe = usb_probe,
    .disconnect = usb_disconnect,
};

static int __init
mod_init(void) {
    return usb_register(&avr_gpio_driver);
}
module_init(mod_init);

static void __exit
mod_exit(void) {
    printk(KERN_INFO "Unloading avr-gpio driver");
    usb_deregister(&avr_gpio_driver);
}
module_exit(mod_exit);

static int
usb_probe(struct usb_interface *interface, const struct usb_device_id *id) {
    struct usb_device *udev = interface_to_usbdev(interface);
    int ret;

    static const char
        vendor_name[USB_CFG_VENDOR_NAME_LEN + 1] = {USB_CFG_VENDOR_NAME, '\0'},
        device_name[USB_CFG_DEVICE_NAME_LEN + 1] = {USB_CFG_DEVICE_NAME, '\0'};
    // FIXME: isn't this printing superfluous?  Don't other USB bits do this too?
    printk(KERN_INFO "manufacturer: %s", udev->manufacturer);
    printk(KERN_INFO "product: %s", udev->product);
    if (! (strcmp(udev->manufacturer, vendor_name) == 0 &&
           strcmp(udev->product, device_name) == 0)) {
        return -ENODEV;
    }

    struct avr_gpio *data = kzalloc(sizeof(struct avr_gpio), GFP_KERNEL);
    if (data == NULL) {
        printk(KERN_ALERT "Failed to allocate data");
        return -ENODEV;
    }

    // Increase ref count.  Make sure you call usb_put_dev() in disconnect().
    data->udev = usb_get_dev(udev);
    data->timeout = 100;
    mutex_init(&data->lock);

    data->chip.label = "avr-gpio";
    // Optional device providing the GPIOs:
    data->chip.parent = &data->udev->dev;
    /* Helps prevent removal of modules exporting active GPIOs.  This is
       required for proper cleanup. */
    data->chip.owner = THIS_MODULE;
    data->chip.base = -1;
    data->chip.direction_input = direction_input;
    data->chip.direction_output = direction_output;
    data->chip.get = get;
    data->chip.set = set;
    data->chip.can_sleep = true;

    usb_set_intfdata(interface, data);

    mutex_lock(&data->lock);
    ret = usb_control_msg(data->udev, usb_rcvctrlpipe(data->udev, 0),
                          GET_INFO,
                          USB_RECIP_DEVICE | USB_TYPE_VENDOR | USB_DIR_IN,
                          0, 0,
                          data->buf, 1,
                          data->timeout);
    mutex_unlock(&data->lock);
    if (ret != 1) {
        dev_err(&data->udev->dev, "GET_INFO failed (ret %d)\n",
                ret);
        ret = -EREMOTEIO;
        goto err;
    }

    data->chip.ngpio = data->buf[0];

    struct usb_host_interface *iface_desc = interface->cur_altsetting;
    printk(KERN_INFO "AVR-GPIO board %d probed: (%04X:%04X)",
           iface_desc->desc.bInterfaceNumber, id->idVendor, id->idProduct);
    printk(KERN_INFO "bNumEndpoints: %d", iface_desc->desc.bNumEndpoints);
    printk(KERN_INFO "ngpio: %d", data->chip.ngpio);
    
    if (gpiochip_add(&data->chip) < 0) {
        printk(KERN_ALERT "Failed to add gpio chip");
        ret = -ENODEV;
        goto err;
    }
    
    return 0;

 err:
    usb_put_dev(data->udev);
    kfree(data);
    return ret;
}

static void
usb_disconnect(struct usb_interface *interface) {
    struct avr_gpio *data = usb_get_intfdata(interface);
    gpiochip_remove(&data->chip);
    usb_put_dev(data->udev);
    kfree(data);
}

static int
direction_input(struct gpio_chip *chip, unsigned int offset) {
    struct avr_gpio *data = container_of(chip, struct avr_gpio, chip);
    int ret;

    mutex_lock(&data->lock);
    ret = usb_control_msg(data->udev, usb_sndctrlpipe(data->udev, 0),
                          GPIO_INPUT,
                          USB_RECIP_DEVICE | USB_TYPE_VENDOR | USB_DIR_OUT,
                          0, offset,
                          NULL, 0,
                          data->timeout);
    mutex_unlock(&data->lock);
    if (ret != 0) {
        dev_err(chip->parent, "GPIO_INPUT %d failed (ret %d)\n",
                offset, ret);
        return -EREMOTEIO;
    }
    
    return 0;
}

static int
direction_output(struct gpio_chip *chip, unsigned int offset, int value) {
    struct avr_gpio *data = container_of(chip, struct avr_gpio, chip);
    int ret;

    mutex_lock(&data->lock);
    ret = usb_control_msg(data->udev, usb_sndctrlpipe(data->udev, 0),
                          GPIO_OUTPUT,
                          USB_RECIP_DEVICE | USB_TYPE_VENDOR | USB_DIR_OUT,
                          !!value, offset,
                          NULL, 0,
                          data->timeout);
    mutex_unlock(&data->lock);
    if (ret != 0) {
        dev_err(chip->parent, "GPIO_OUTPUT %d failed (ret %d)\n",
                offset, ret);
        return -EREMOTEIO;
    }
    
    return 0;
}

static int
get(struct gpio_chip *chip, unsigned int offset) {
    struct avr_gpio *data = container_of(chip, struct avr_gpio, chip);
    int ret;

    mutex_lock(&data->lock);
    ret = usb_control_msg(data->udev, usb_rcvctrlpipe(data->udev, 0),
                          GPIO_READ,
                          USB_RECIP_DEVICE | USB_TYPE_VENDOR | USB_DIR_IN,
                          0, offset,
                          data->buf, 1,
                          data->timeout);
    mutex_unlock(&data->lock);
    if (ret != 1) {
        dev_err(chip->parent, "GPIO_READ %d failed (ret %d)\n",
                offset, ret);
        return -EREMOTEIO;
    }
    
    return !!data->buf[0];
}

static void
set(struct gpio_chip *chip, unsigned int offset, int value) {
    struct avr_gpio *data = container_of(chip, struct avr_gpio, chip);
    int ret;

    mutex_lock(&data->lock);
    ret = usb_control_msg(data->udev, usb_sndctrlpipe(data->udev, 0),
                          GPIO_WRITE,
                          USB_RECIP_DEVICE | USB_TYPE_VENDOR | USB_DIR_OUT,
                          !!value, offset,
                          NULL, 0,
                          data->timeout);
    mutex_unlock(&data->lock);
    if (ret != 0) {
        dev_err(chip->parent, "GPIO_WRITE %d = %d failed (ret %d)\n",
                offset, !!value, ret);
    }
}
