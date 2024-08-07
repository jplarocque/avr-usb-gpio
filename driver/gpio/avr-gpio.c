/*
 *  AVR-GPIO
 *
 *  (C) Amitesh Singh <singh.amitesh@gmail.com>, 2016
 *  © 2024 Jean-Paul Larocque
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/device.h>
#include <linux/usb.h>
#include <linux/slab.h>
#include <linux/gpio/driver.h>
#include <linux/spinlock.h>

#include "../../firmware/common.h"
#include "../../firmware/usbidconfig.h"

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Jean-Paul Larocque <jpl@ftlvisual.com>");
MODULE_DESCRIPTION("AVR-GPIO USB driver");
MODULE_VERSION("0.1");

struct avr_gpio
{
   struct usb_device *udev;
   struct gpio_chip chip; //this is our GPIO chip
   u8 buf[1];
   u32 timeout;
   spinlock_t lock;
};

static void
_gpioa_set(struct gpio_chip *chip,
           unsigned offset, int value)
{
   struct avr_gpio *data = container_of(chip, struct avr_gpio, chip);
   int ret;

   spin_lock(&data->lock);
   ret = usb_control_msg(data->udev, usb_sndctrlpipe(data->udev, 0),
                         GPIO_WRITE,
                         USB_RECIP_DEVICE | USB_TYPE_VENDOR | USB_DIR_OUT,
                         !!value, offset,
                         NULL, 0,
                         data->timeout);
   spin_unlock(&data->lock);

   if (ret != 0) {
     dev_err(chip->parent, "usb error setting pin value\n");
   }
}

static int
_gpioa_get(struct gpio_chip *chip,
           unsigned offset)
{
   struct avr_gpio *data = container_of(chip, struct avr_gpio, chip);
   int ret;

   printk(KERN_INFO "GPIO GET INFO: %d", offset);

   spin_lock(&data->lock);
   ret = usb_control_msg(data->udev, usb_rcvctrlpipe(data->udev, 0),
                         GPIO_READ,
                         USB_RECIP_DEVICE | USB_TYPE_VENDOR | USB_DIR_IN,
                         0, offset,
                         data->buf, 1,
                         data->timeout);
   spin_unlock(&data->lock);

   if (ret != 1) {
        printk(KERN_ALERT "ret = %d Failed to get correct reply", ret);
        return -EREMOTEIO;
   }
   
   return !!data->buf[0];
}

static int
_direction_output(struct gpio_chip *chip,
                  unsigned offset, int value)
{
   struct avr_gpio *data = container_of(chip, struct avr_gpio, chip);
   int ret;

   spin_lock(&data->lock);
   ret = usb_control_msg(data->udev, usb_sndctrlpipe(data->udev, 0),
                         GPIO_OUTPUT,
                         USB_RECIP_DEVICE | USB_TYPE_VENDOR | USB_DIR_OUT,
                         0, offset,
                         NULL, 0,
                         data->timeout);
   spin_unlock(&data->lock);

   if (ret != 0) {
        printk(KERN_ALERT "ret = %d Failed to get correct reply", ret);
        return -EREMOTEIO;
   }
   
   return 0;
}

static int
_direction_input(struct gpio_chip *chip,
                 unsigned offset)
{
   struct avr_gpio *data = container_of(chip, struct avr_gpio, chip);
   int ret;

   spin_lock(&data->lock);
   ret = usb_control_msg(data->udev, usb_sndctrlpipe(data->udev, 0),
                         GPIO_INPUT,
                         USB_RECIP_DEVICE | USB_TYPE_VENDOR | USB_DIR_OUT,
                         0, offset,
                         NULL, 0,
                         data->timeout);
   spin_unlock(&data->lock);

   if (ret != 0) {
        printk(KERN_ALERT "ret= %d Failed to get correct reply", ret);
        return -EREMOTEIO;
   }
   
   return 0;
}

static int
avr_gpio_probe(struct usb_interface *interface,
               const struct usb_device_id *id)
{
   struct usb_device *udev = interface_to_usbdev(interface);
   struct usb_host_interface *iface_desc;
   struct avr_gpio *data;
   int ret;

   static const char
       vendor_name[USB_CFG_VENDOR_NAME_LEN + 1] = {USB_CFG_VENDOR_NAME, '\0'},
       device_name[USB_CFG_DEVICE_NAME_LEN + 1] = {USB_CFG_DEVICE_NAME, '\0'};
   printk(KERN_INFO "manufacturer: %s", udev->manufacturer);
   printk(KERN_INFO "product: %s", udev->product);
   if (! (strcmp(udev->manufacturer, vendor_name) == 0 &&
          strcmp(udev->product, device_name) == 0)) {
       return -ENODEV;
   }

   data = kzalloc(sizeof(struct avr_gpio), GFP_KERNEL);
   if (data == NULL)
     {
        printk(KERN_ALERT "Failed to alloc data");
        return -ENODEV;
     }

   //increase ref count, make sure u call usb_put_dev() in disconnect()
   data->udev = usb_get_dev(udev);

   data->chip.label = "avr-gpio";
   data->chip.parent = &data->udev->dev; // optional device providing the GPIOs
   data->chip.owner = THIS_MODULE; // helps prevent removal of modules exporting active GPIOs, so this is required for proper cleanup
   data->chip.base = -1;
   data->chip.can_sleep = false;

   data->chip.set = _gpioa_set;
   data->chip.get = _gpioa_get;

   data->chip.direction_input = _direction_input;
   data->chip.direction_output = _direction_output;
   data->timeout = 100;

   usb_set_intfdata(interface, data);

   spin_lock_init(&data->lock);

   //init the board
   spin_lock(&data->lock);
   ret = usb_control_msg(data->udev, usb_rcvctrlpipe(data->udev, 0),
                         GET_INFO,
                         USB_RECIP_DEVICE | USB_TYPE_VENDOR | USB_DIR_IN,
                         0, 0,
                         data->buf, 1,
                         data->timeout);
   spin_unlock(&data->lock);

   if (ret != 1) {
        printk(KERN_ALERT "ret = %d, func= %s Failed to get correct reply", ret, __func__);
        return -EREMOTEIO;
   }

   data->chip.ngpio = data->buf[0];

   iface_desc = interface->cur_altsetting;
   printk(KERN_INFO "AVR-GPIO board %d probed: (%04X:%04X)",
          iface_desc->desc.bInterfaceNumber, id->idVendor, id->idProduct);
   printk(KERN_INFO "bNumEndpoints: %d", iface_desc->desc.bNumEndpoints);
   printk(KERN_INFO "ngpio: %d", data->chip.ngpio);
   
   if (gpiochip_add(&data->chip) < 0)
   {
	   printk(KERN_ALERT "Failed to add gpio chip");
	   return -ENODEV;
   }
   
   return 0;
}

static void
avr_gpio_disconnect(struct usb_interface *interface)
{
   struct avr_gpio *data;

   data = usb_get_intfdata(interface);

   gpiochip_remove(&data->chip);

   usb_set_intfdata(interface, NULL);

   //deref the count
   usb_put_dev(data->udev);

   kfree(data);
}

static struct usb_device_id avr_gpio_table[] = {
       { USB_DEVICE(USB_VENDOR_ID, USB_DEVICE_ID) },
       {},
};

MODULE_DEVICE_TABLE(usb, avr_gpio_table);

static struct usb_driver avr_gpio_driver = {
     .name = "avr_gpio",
     .id_table = avr_gpio_table,
     .probe = avr_gpio_probe,
     .disconnect = avr_gpio_disconnect,
};

static int __init
_usb_init(void)
{
  return usb_register(&avr_gpio_driver);
}

static void __exit
_usb_exit(void)
{
   printk(KERN_INFO "usb driver is unloaded");
   usb_deregister(&avr_gpio_driver);
}

module_init(_usb_init);
module_exit(_usb_exit);
