/*
 * usb_config.h
 *
 * CherryUSB device configuration for the CH32L103 candleLight port.
 *
 * Only the stack core is built: the gs_usb class (candle-light compatible
 * vendor interface) lives in the candleLight firmware sources, so there is no
 * CONFIG_USB_DEVICE_<class> here - our class handler is registered from
 * src/usbd_bridge.c.
 */
#ifndef USB_CONFIG_H
#define USB_CONFIG_H

/* No debug output from the stack: it would pull the formatter in. */
#define CONFIG_USB_PRINTF(...)  ((void)0)
#define CONFIG_USB_DBG_LEVEL    0

#define CONFIG_USB_ALIGN_SIZE   4
#define USB_NOCACHE_RAM_SECTION

#define CONFIG_USB_DEVICE       1
#define CONFIG_USBDEV_MAX_BUS   1

/* gs_usb needs no WCID/MS OS descriptors: Windows binds the libusb driver
 * through the interface GUID that candleLight's usbd_desc.c hands out. */
#define CONFIG_USBDEV_ADVANCE_DESC 1

/* EP0 control transfers are 8 byte gs_usb requests plus a 64 byte setup buffer,
 * so the stack's request buffer stays small. */
#define CONFIG_USBDEV_REQUEST_BUFFER_LEN 512

/* EP0 + 2 bulk endpoints (IN/OUT). */
#define CONFIG_USBDEV_EP_NUM    3

/* VID/PID live in candleLight's config.h (USBD_VID / USBD_PID_FS); CherryUSB
 * only serves the descriptor bytes we hand it, so it needs none of them here.
 * Defining them in this header would clash with config.h in the same TU. */

#endif /* USB_CONFIG_H */
