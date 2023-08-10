/* This file is the part of the Lightweight USB device Stack for STM32 microcontrollers
 *
 * Copyright ©2016 Dmitry Filimonchuk <dmitrystu[at]gmail[dot]com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *   http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm32.h"
#include "usb.h"
#include "usb_cdc.h"
#if (ENABLE_HID_WEBUSB_COMM == 1)
#include "usb_hid.h"
#endif
#include "sdk_config.h"
#include "libusb.h"

#define DEVICE_DATA_EP_SIZE   0x40
#define USB_EP0_SIZE    0x08

#if (ENABLE_CDC_COMM == 1)
#define CDC_RXD_EP      0x01
#define CDC_TXD_EP      0x81
#define CDC_DATA_SZ     DEVICE_DATA_EP_SIZE
#define CDC_NTF_EP      0x82
#define CDC_NTF_SZ      0x08

#define DEVICE_DATA_RX_EP   CDC_RXD_EP
#define DEVICE_DATA_TX_EP   CDC_TXD_EP
#define DEVICE_DATA_EP_TYPE USB_EPTYPE_BULK
#elif (ENABLE_HID_WEBUSB_COMM == 1)
#define HID_RXD_EP      0x01
#define HID_TXD_EP      0x81
#define HID_DATA_SZ     DEVICE_DATA_EP_SIZE
#define WEBUSB_RXD_EP   0x02
#define WEBUSB_TXD_EP   0x82
#define WEBUSB_DATA_SZ  DEVICE_DATA_EP_SIZE

#define WEBUSB_REQUEST_GET_ALLOWED_ORIGINS  0x01
#define WEBUSB_REQUEST_GET_URL              0x02

#define DEVICE_DATA_RX_EP   HID_RXD_EP
#define DEVICE_DATA_TX_EP   HID_TXD_EP
#define DEVICE_DATA_EP_TYPE USB_EPTYPE_INTERRUPT
#else
    #error "No USB protocol defined select ENABLE_CDC_COMM or ENABLE_HID_WEBUSB_COMM"
#endif

#define LU_RX_PKT_BUF_COUNT     3
#define LU_TX_PKT_BUF_COUNT     3

// Bootloader takes upto 128 byte payloads while application takes about 88 bytes long payload
#define BYTE_STUFFED_DATA_SIZE                 128

#define LU_RX_BUF_SIZE          (BYTE_STUFFED_DATA_SIZE * LU_RX_PKT_BUF_COUNT)
#define LU_TX_BUF_SIZE          (BYTE_STUFFED_DATA_SIZE * LU_TX_PKT_BUF_COUNT)

//#define SIGNAL_MODEM
#define CDC_USE_IRQ

#if defined(SIGNAL_MODEM)
#define CDC_PROTOCOL USB_CDC_PROTO_V25TER
#else
#define CDC_PROTOCOL USB_PROTO_NONE
#endif

libusb_parser_fptr_t parseFun = NULL;

/* Declaration of the report descriptor */
struct usb_device__config {
    struct usb_config_descriptor        config;
#if (ENABLE_CDC_COMM == 1)
    struct usb_iad_descriptor           comm_iad;
    struct usb_interface_descriptor     comm;
    struct usb_cdc_header_desc          cdc_hdr;
    struct usb_cdc_call_mgmt_desc       cdc_mgmt;
    struct usb_cdc_acm_desc             cdc_acm;
    struct usb_cdc_union_desc           cdc_union;
    struct usb_endpoint_descriptor      comm_ep;
    struct usb_interface_descriptor     data;
    struct usb_endpoint_descriptor      data_eprx;
    struct usb_endpoint_descriptor      data_eptx;
#elif (ENABLE_HID_WEBUSB_COMM == 1)
    struct usb_interface_descriptor     hid;
    struct usb_hid_descriptor           hid_desc;
    struct usb_endpoint_descriptor      data_eprx;
    struct usb_endpoint_descriptor      data_eptx;
    struct usb_interface_descriptor     webdata;
    struct usb_endpoint_descriptor      webdata_eprx;
    struct usb_endpoint_descriptor      webdata_eptx;
#else
    #error "No USB protocol defined select ENABLE_CDC_COMM or ENABLE_HID_WEBUSB_COMM"
#endif

} __attribute__((packed));

/* Device descriptor */
static const struct usb_device_descriptor device_desc = {
    .bLength            = sizeof(struct usb_device_descriptor),
    .bDescriptorType    = USB_DTYPE_DEVICE,
#if (ENABLE_CDC_COMM == 1)
    .bcdUSB             = VERSION_BCD(2,0,0),
#elif (ENABLE_HID_WEBUSB_COMM == 1)
    .bcdUSB             = VERSION_BCD(2,1,0),
#else
    #error "No USB protocol defined select ENABLE_CDC_COMM or ENABLE_HID_WEBUSB_COMM"
#endif
    .bDeviceClass       = USB_CLASS_IAD,
    .bDeviceSubClass    = USB_SUBCLASS_IAD,
    .bDeviceProtocol    = USB_PROTO_IAD,
    .bMaxPacketSize0    = USB_EP0_SIZE,
    .idVendor           = BSP_USB_VID,
    .idProduct          = BSP_USB_PID,
    .bcdDevice          = VERSION_BCD(2,0,0),
    .iManufacturer      = 1,
    .iProduct           = 2,
    .iSerialNumber      = INTSERIALNO_DESCRIPTOR,
    .bNumConfigurations = 1,
};

#if (ENABLE_HID_WEBUSB_COMM == 1)
static const uint8_t hid_report_desc[] = {
  0x06, 0xa0, 0xff,     //USAGE PAGE: Vendor 1
  0x09, 0x01,           //USAGE: Vendor 1
  0xa1, 0x01,           //Collection Application
  0x15, 0x00,           //Logical Minimum
  0x26, 0xff, 0x00,     //Logical Maximum
  0x75, 0x08,           //Report Size
  0x95, 0x40,           //Report Count
  0x09, 0x01,           //Usage: Vendor 1
  0x81, 0x08,           //Input (Data, Arr, Abs, Wrap)
  0x95, 0x40,           //Report Count
  0x09, 0x01,           //Usage: Vendor 1
  0x91, 0x08,           //Output (Data, Arr, Abs, Wrap)
  0x95, 0x01,           //Report Count
  0x09, 0x01,           //Usage: Vendor 1
  0xB1, 0x02,           //Feature
  0xC0                  //End collection
  };
#endif

/* Device configuration descriptor */
static const struct usb_device__config config_desc = {
    .config = {
        .bLength                = sizeof(struct usb_config_descriptor),
        .bDescriptorType        = USB_DTYPE_CONFIGURATION,
        .wTotalLength           = sizeof(struct usb_device__config),
#if (ENABLE_CDC_COMM == 1)
        .bNumInterfaces         = 2,
#elif (ENABLE_HID_WEBUSB_COMM == 1)
        .bNumInterfaces         = 2,
#else
    #error "No USB protocol defined select ENABLE_CDC_COMM or ENABLE_HID_WEBUSB_COMM"
#endif
        .bConfigurationValue    = 1,
        .iConfiguration         = NO_DESCRIPTOR,
        .bmAttributes           = USB_CFG_ATTR_RESERVED | USB_CFG_ATTR_SELFPOWERED,
        .bMaxPower              = USB_CFG_POWER_MA(200),
    },
#if (ENABLE_CDC_COMM == 1)
    .comm_iad = {
        .bLength = sizeof(struct usb_iad_descriptor),
        .bDescriptorType        = USB_DTYPE_INTERFASEASSOC,
        .bFirstInterface        = 0,
        .bInterfaceCount        = 2,
        .bFunctionClass         = USB_CLASS_CDC,
        .bFunctionSubClass      = USB_CDC_SUBCLASS_ACM,
        .bFunctionProtocol      = CDC_PROTOCOL,
        .iFunction              = NO_DESCRIPTOR,
    },
    .comm = {
        .bLength                = sizeof(struct usb_interface_descriptor),
        .bDescriptorType        = USB_DTYPE_INTERFACE,
        .bInterfaceNumber       = 0,
        .bAlternateSetting      = 0,
        .bNumEndpoints          = 1,
        .bInterfaceClass        = USB_CLASS_CDC,
        .bInterfaceSubClass     = USB_CDC_SUBCLASS_ACM,
        .bInterfaceProtocol     = CDC_PROTOCOL,
        .iInterface             = NO_DESCRIPTOR,
    },
    .cdc_hdr = {
        .bFunctionLength        = sizeof(struct usb_cdc_header_desc),
        .bDescriptorType        = USB_DTYPE_CS_INTERFACE,
        .bDescriptorSubType     = USB_DTYPE_CDC_HEADER,
        .bcdCDC                 = VERSION_BCD(1,1,0),
    },
    .cdc_mgmt = {
        .bFunctionLength        = sizeof(struct usb_cdc_call_mgmt_desc),
        .bDescriptorType        = USB_DTYPE_CS_INTERFACE,
        .bDescriptorSubType     = USB_DTYPE_CDC_CALL_MANAGEMENT,
        .bmCapabilities         = 0,
        .bDataInterface         = 1,

    },
    .cdc_acm = {
        .bFunctionLength        = sizeof(struct usb_cdc_acm_desc),
        .bDescriptorType        = USB_DTYPE_CS_INTERFACE,
        .bDescriptorSubType     = USB_DTYPE_CDC_ACM,
        .bmCapabilities         = 0,
    },
    .cdc_union = {
        .bFunctionLength        = sizeof(struct usb_cdc_union_desc),
        .bDescriptorType        = USB_DTYPE_CS_INTERFACE,
        .bDescriptorSubType     = USB_DTYPE_CDC_UNION,
        .bMasterInterface0      = 0,
        .bSlaveInterface0       = 1,
    },
    .comm_ep = {
        .bLength                = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType        = USB_DTYPE_ENDPOINT,
        .bEndpointAddress       = CDC_NTF_EP,
        .bmAttributes           = USB_EPTYPE_INTERRUPT,
        .wMaxPacketSize         = CDC_NTF_SZ,
        .bInterval              = 0xFF,
    },
    .data = {
        .bLength                = sizeof(struct usb_interface_descriptor),
        .bDescriptorType        = USB_DTYPE_INTERFACE,
        .bInterfaceNumber       = 1,
        .bAlternateSetting      = 0,
        .bNumEndpoints          = 2,
        .bInterfaceClass        = USB_CLASS_CDC_DATA,
        .bInterfaceSubClass     = USB_SUBCLASS_NONE,
        .bInterfaceProtocol     = USB_PROTO_NONE,
        .iInterface             = NO_DESCRIPTOR,
    },
    .data_eprx = {
        .bLength                = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType        = USB_DTYPE_ENDPOINT,
        .bEndpointAddress       = CDC_RXD_EP,
        .bmAttributes           = USB_EPTYPE_BULK,
        .wMaxPacketSize         = CDC_DATA_SZ,
        .bInterval              = 0x01,
    },
    .data_eptx = {
        .bLength                = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType        = USB_DTYPE_ENDPOINT,
        .bEndpointAddress       = CDC_TXD_EP,
        .bmAttributes           = USB_EPTYPE_BULK,
        .wMaxPacketSize         = CDC_DATA_SZ,
        .bInterval              = 0x01,
    },
#elif (ENABLE_HID_WEBUSB_COMM == 1)
    .hid = {
        .bLength                = sizeof(struct usb_interface_descriptor),
        .bDescriptorType        = USB_DTYPE_INTERFACE,
        .bInterfaceNumber       = 0,
        .bAlternateSetting      = 0,
        .bNumEndpoints          = 2,
        .bInterfaceClass        = USB_CLASS_HID,
        .bInterfaceSubClass     = USB_SUBCLASS_NONE,
        .bInterfaceProtocol     = USB_PROTO_NONE,
        .iInterface             = 2,
    },
    .hid_desc = {
        .bLength                = sizeof(struct usb_hid_descriptor),
        .bDescriptorType        = USB_DTYPE_HID,
        .bcdHID                 = VERSION_BCD(1,1,1),
        .bCountryCode           = USB_HID_COUNTRY_NONE,
        .bNumDescriptors        = 1,
        .bDescriptorType0       = USB_DTYPE_HID_REPORT,
        .wDescriptorLength0     = sizeof(hid_report_desc),
    },
    .data_eprx = {
        .bLength                = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType        = USB_DTYPE_ENDPOINT,
        .bEndpointAddress       = HID_RXD_EP,
        .bmAttributes           = USB_EPTYPE_INTERRUPT,
        .wMaxPacketSize         = HID_DATA_SZ,
        .bInterval              = 0x01,
    },
    .data_eptx = {
        .bLength                = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType        = USB_DTYPE_ENDPOINT,
        .bEndpointAddress       = HID_TXD_EP,
        .bmAttributes           = USB_EPTYPE_INTERRUPT,
        .wMaxPacketSize         = HID_DATA_SZ,
        .bInterval              = 0x01,
    },
    .webdata = {
        .bLength                = sizeof(struct usb_interface_descriptor),
        .bDescriptorType        = USB_DTYPE_INTERFACE,
        .bInterfaceNumber       = 1,
        .bAlternateSetting      = 0,
        .bNumEndpoints          = 2,
        .bInterfaceClass        = 0xff,
        .bInterfaceSubClass     = 0x00,
        .bInterfaceProtocol     = 0x00,
        .iInterface             = NO_DESCRIPTOR,
    },
    .webdata_eprx = {
        .bLength                = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType        = USB_DTYPE_ENDPOINT,
        .bEndpointAddress       = WEBUSB_RXD_EP,
        .bmAttributes           = USB_EPTYPE_INTERRUPT,
        .wMaxPacketSize         = WEBUSB_DATA_SZ,
        .bInterval              = 0x01,
    },
    .webdata_eptx = {
        .bLength                = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType        = USB_DTYPE_ENDPOINT,
        .bEndpointAddress       = WEBUSB_TXD_EP,
        .bmAttributes           = USB_EPTYPE_INTERRUPT,
        .wMaxPacketSize         = WEBUSB_DATA_SZ,
        .bInterval              = 0x01,
    },
#else
    #error "No USB protocol defined select ENABLE_CDC_COMM or ENABLE_HID_WEBUSB_COMM"
#endif
};

static const struct usb_string_descriptor lang_desc     = USB_ARRAY_DESC(USB_LANGID_ENG_US);
static const struct usb_string_descriptor manuf_desc_en = USB_STRING_DESC("HODL TECH PTE LTD");
static const struct usb_string_descriptor prod_desc_en  = USB_STRING_DESC("CYPHEROCK X1 WALLET");
static const struct usb_string_descriptor *const dtable[] = {
    &lang_desc,
    &manuf_desc_en,
    &prod_desc_en,
};

typedef struct{
    uint8_t  tx_fifo[LU_TX_BUF_SIZE];
    uint8_t  rx_buffer[LU_RX_BUF_SIZE];
    uint32_t fifo_pos;
} rxtx_buffer;

usbd_device libusb_udev;
uint32_t	ubuf[0x20];

#if (ENABLE_HID_WEBUSB_COMM == 1)
rxtx_buffer webusb_buffer = {
    .tx_fifo = {0},
    .rx_buffer = {0},
    .fifo_pos = 0
};
#endif

rxtx_buffer data_ep_buffer = {
    .tx_fifo = {0},
    .rx_buffer = {0},
    .fifo_pos = 0
};

static struct usb_cdc_line_coding cdc_line = {
    .dwDTERate          = 38400,
    .bCharFormat        = USB_CDC_1_STOP_BITS,
    .bParityType        = USB_CDC_NO_PARITY,
    .bDataBits          = 8,
    .blineState         = 0
};
#if (ENABLE_HID_WEBUSB_COMM == 1)
struct usb_bos_cap_descriptor{
    struct usb_bos_descriptor           bos;
    struct usb_webusb_cap_descriptor    webusb_cap;
    struct usb_MSOS2_cap_descriptor     msos2_cap;
} __attribute__((packed));

const struct usb_bos_cap_descriptor bos_desc = {
    .bos = {
        .bLength                = sizeof(struct usb_bos_descriptor),
        .bDescriptorType        = USB_DTYPE_BOS,
        .wTotalLenght           = sizeof(struct usb_bos_cap_descriptor),
        .bNumDeviceCaps         = 2,
    },
    .webusb_cap = {
        .bLength                = sizeof(struct usb_webusb_cap_descriptor),
        .bDescriptorType        = USB_DTYPE_CAPABILITY,
        .bDevCapabilityType     = 0x05,
        .bReserved              = 0x00,
        .PlatformCapabilityUUID = {0x38, 0xB6, 0x08, 0x34, 0xA9, 0x09, 0xA0, 0x47, 0x8B, 0xFD, 0xA0, 0x76, 0x88, 0x15, 0xB6, 0x65},
        .bcdVersion             = VERSION_BCD(1,0,0),
        .bVendorCode            = 0x01,
    },
    .msos2_cap = {
        .bLength                = sizeof(struct usb_MSOS2_cap_descriptor),
        .bDescriptorType        = USB_DTYPE_CAPABILITY,
        .bDevCapabilityType     = 0x05,
        .bReserved              = 0x00,
        .PlatformCapabilityUUID = {0xDF, 0x60, 0xDD, 0xD8, 0x89, 0x45, 0xC7, 0x4C, 0x9C, 0xD2, 0x65, 0x9D, 0x9E, 0x64, 0x8A, 0x9F},
        .dwWindowsVersion       = 0x06030000,
        .wMSOSDescriptorSetTotalLength = 0x00B2,
        .bMS_VendorCode         = 0x02,
        .bAltEnumCode           = 0x00,
    }
};

struct usb_MSOS2_set_descriptors {
    struct usb_MSOS2_desc_set_header_descriptor     desc_set_header;
    struct usb_MSOS2_conf_subset_header_descriptor  conf_subset_header;
    struct usb_MSOS2_func_subset_header_descriptor  func_subset_header;
    struct usb_MSOS2_comp_id_descriptor             comp_id;
    struct usb_MSOS2_reg_prop_descriptor            reg_prop;
};

const struct usb_MSOS2_set_descriptors msos2_desc_set = {
    .desc_set_header = {
        .wLength                    = sizeof(struct usb_MSOS2_desc_set_header_descriptor),
        .wDescriptorType            = 0x0000,
        .dwWindowsVersion           = 0x06030000,
        .wTotalLenght               = 0x00B2,
    },
    .conf_subset_header = {
        .wLength                    = sizeof(struct usb_MSOS2_conf_subset_header_descriptor),
        .wDescriptorType            = 0x0001,
        .bConfigurationValue        = 0x00,
        .bReserved                  = 0x00,
        .wTotalLenght               = 0x00A8,
    },
    .func_subset_header = {
        .wLength                    = sizeof(struct usb_MSOS2_func_subset_header_descriptor),
        .wDescriptorType            = 0x0002,
        .bFirstInterface            = 0x01,
        .bReserved                  = 0x00,
        .wSubsetLenght              = 0x00A0,
    },
    .comp_id = {
        .wLength                    = sizeof(struct usb_MSOS2_comp_id_descriptor),
        .wDescriptorType            = 0x0003,
        .CompatibleID               = {'W', 'I', 'N', 'U', 'S', 'B', '\0', '\0'},
        .SubCompatibleID            = {'\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0'},
    },
    .reg_prop = {
        .wLength                    = sizeof(struct usb_MSOS2_reg_prop_descriptor),
        .wDescriptorType            = 0x0004,
        .wPropertyDataType          = 0x0007,
        .wPropertyNameLength        = 0x002A,
        .PropertyName               = {'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00, 'e', 0x00, 'I', 0x00, 'n', 0x00, 't', 0x00, 'e', 0x00, 
                                        'r', 0x00, 'f', 0x00, 'a', 0x00, 'c', 0x00, 'e', 0x00, 'G', 0x00, 'U', 0x00, 'I', 0x00, 'D', 0x00, 's', 0x00, 0x00, 0x00}, 
        .wPropertyDataLength        = 0x0050,
        .PropertyData               = {'{', 0x00, '9', 0x00, '7', 0x00, '5', 0x00, 'F', 0x00, '4', 0x00, '4', 0x00, 'D', 0x00, '9', 0x00, '-', 0x00, 
                                        '0', 0x00, 'D', 0x00, '0', 0x00, '8', 0x00, '-', 0x00, '4', 0x00, '3', 0x00, 'F', 0x00, 'D', 0x00, '-', 0x00, 
                                        '8', 0x00, 'B', 0x00, '3', 0x00, 'E', 0x00, '-', 0x00, '1', 0x00, '2', 0x00, '7', 0x00, 'C', 0x00, 'A', 0x00, 
                                        '8', 0x00, 'A', 0x00, 'F', 0x00, 'F', 0x00, 'F', 0x00, '9', 0x00, 'D', 0x00, '}', 0x00, 0x00, 0x00, 0x00, 0x00}
    },
};
#endif

static usbd_respond device_getdesc (usbd_ctlreq *req, void **address, uint16_t *length) {
    const uint8_t dtype = req->wValue >> 8;
    const uint8_t dnumber = req->wValue & 0xFF;
    const void* desc;
    uint16_t len = 0;
    switch (dtype) {
    case USB_DTYPE_DEVICE:
        desc = &device_desc;
        break;
    case USB_DTYPE_CONFIGURATION:
        desc = &config_desc;
        len = sizeof(config_desc);
        break;
    case USB_DTYPE_STRING:
        if (dnumber < 3) {
            desc = dtable[dnumber];
        } else {
            return usbd_fail;
        }
        break;
    default:
        return usbd_fail;
    }
    if (len == 0) {
        len = ((struct usb_header_descriptor*)desc)->bLength;
    }
    *address = (void*)desc;
    *length = len;
    return usbd_ack;
}

static usbd_respond device_control(usbd_device *dev, usbd_ctlreq *req, usbd_rqc_callback *callback) {
#if (ENABLE_CDC_COMM == 1)
    if (((USB_REQ_RECIPIENT | USB_REQ_TYPE) & req->bmRequestType) == (USB_REQ_INTERFACE | USB_REQ_CLASS)
        && req->wIndex == 0 ) {
#elif (ENABLE_HID_WEBUSB_COMM == 1)
    if (((USB_REQ_RECIPIENT | USB_REQ_TYPE) & req->bmRequestType) == (USB_REQ_INTERFACE | USB_REQ_CLASS)
        && req->wIndex == 1 ) {
#else
    #error "No USB protocol defined select ENABLE_CDC_COMM or ENABLE_HID_WEBUSB_COMM"
#endif
        switch (req->bRequest) {
        case USB_CDC_SET_CONTROL_LINE_STATE:
            return usbd_ack;
        case USB_CDC_SET_LINE_CODING:
            memcpy(&cdc_line, req->data, sizeof(cdc_line));
            return usbd_ack;
        case USB_CDC_GET_LINE_CODING:
            dev->status.data_ptr = &cdc_line;
            dev->status.data_count = sizeof(cdc_line);
            return usbd_ack;
        default:
            return usbd_fail;
        }
    }
#if (ENABLE_HID_WEBUSB_COMM == 1)
    if (((USB_REQ_RECIPIENT | USB_REQ_TYPE) & req->bmRequestType) == (USB_REQ_INTERFACE | USB_REQ_CLASS)
        && req->wIndex == 0 ) {
        switch (req->bRequest) {
        case USB_HID_SETIDLE:
            return usbd_ack;
        case USB_HID_GETREPORT:
            dev->status.data_ptr = (uint8_t*)&hid_report_desc;
            dev->status.data_count = sizeof(hid_report_desc);
            return usbd_ack;
        default:
            return usbd_fail;
        }
    }
    if (((USB_REQ_RECIPIENT | USB_REQ_TYPE) & req->bmRequestType) == (USB_REQ_INTERFACE | USB_REQ_STANDARD)
        && req->wIndex == 0
        && req->bRequest == USB_STD_GET_DESCRIPTOR) {
        switch (req->wValue >> 8) {
        case USB_DTYPE_HID:
            dev->status.data_ptr = (uint8_t*)&(config_desc.hid_desc);
            dev->status.data_count = sizeof(config_desc.hid_desc);
            return usbd_ack;
        case USB_DTYPE_HID_REPORT:
            dev->status.data_ptr = (uint8_t*)hid_report_desc;
            dev->status.data_count = sizeof(hid_report_desc);
            return usbd_ack;
        default:
            return usbd_fail;
        }
    }
    if (((USB_REQ_RECIPIENT | USB_REQ_TYPE) & req->bmRequestType) == (USB_REQ_STANDARD | USB_REQ_DEVICE)
        && req->wIndex == 0
        && req->bRequest == USB_STD_GET_DESCRIPTOR) {
        switch (req->wValue >> 8) {
        case USB_DTYPE_BOS:
            dev->status.data_ptr = (uint8_t*)&bos_desc;
            dev->status.data_count = sizeof(bos_desc);
            return usbd_ack;
        default:
            return usbd_fail;
        }
    }
    if (((USB_REQ_RECIPIENT | USB_REQ_TYPE) & req->bmRequestType) == (USB_REQ_STANDARD | USB_REQ_VENDOR)
        && req->wIndex == 7
        && req->bRequest == 0x02) {
        switch (req->wValue >> 8) {
        case 0x00:
            dev->status.data_ptr = (uint8_t*)&msos2_desc_set;
            dev->status.data_count = sizeof(msos2_desc_set);
            return usbd_ack;
        default:
            return usbd_fail;
        }
    }
#endif
    return usbd_fail;
}

/**
 * @brief Function to read data from usb RX endpoints and write to respective buffers
 * 
 * @param dev pointer to USB device
 * @param ep active endpoint number
 */
static void comm_rxonly(usbd_device *dev, uint8_t ep){
    rxtx_buffer *buff;
    comm_libusb__interface_e interface;

    // Set receive buffer based on endpoint
#if (ENABLE_HID_WEBUSB_COMM == 1)
    if(ep == DEVICE_DATA_RX_EP){
        buff = &data_ep_buffer;
        interface = COMM_LIBUSB__HID;
    }else if(ep == WEBUSB_RXD_EP){
        buff = &webusb_buffer;
        interface = COMM_LIBUSB__WEBUSB;
    }else{
        return;
    }
#elif (ENABLE_CDC_COMM == 1)
    buff = &data_ep_buffer;
    interface = COMM_LIBUSB__CDC;
#else
    #error "No USB protocol defined select ENABLE_CDC_COMM or ENABLE_HID_WEBUSB_COMM"
#endif

    uint32_t dataSize = usbd_ep_read(dev, ep, buff->rx_buffer, LU_RX_BUF_SIZE);

    // Call application parser
    if(parseFun != NULL)
        parseFun(buff->rx_buffer, dataSize, interface);
}

/**
 * @brief Function to write data to usb TX endpoints from respective buffers
 * 
 * @param dev pointer to USB device
 * @param ep active endpoint number
 */
static void comm_txonly(usbd_device *dev, uint8_t ep){
    rxtx_buffer *buff = NULL;
    uint32_t  tx_len = 0, fifo_move_len = 0;

    // Set buffer based on endpoint from which data will be sent
#if (ENABLE_HID_WEBUSB_COMM == 1)
    if(ep == DEVICE_DATA_TX_EP){
        buff = &data_ep_buffer;
    }else if(ep == WEBUSB_TXD_EP){
        buff = &webusb_buffer;
    }else{
        return;
    }
#elif (ENABLE_CDC_COMM == 1)
    buff = &data_ep_buffer;
#else
    #error "No USB protocol defined select ENABLE_CDC_COMM or ENABLE_HID_WEBUSB_COMM"
#endif

    /**
     * Set data stream length to pass to TX endpoint. In some cases the length of data
     * stream can be greater than the data present in the fifo buffer. This needs to be
     * defined as we need to move the fifo buffer so the data already transmitted is not
     * repeated. So lenght of data stream to move from fifo is also set based on ep and 
     * present data size.
     */
    if(buff->fifo_pos >= DEVICE_DATA_EP_SIZE){
        fifo_move_len = tx_len = DEVICE_DATA_EP_SIZE;
    } else {
        fifo_move_len = tx_len = buff->fifo_pos;
#if (ENABLE_HID_WEBUSB_COMM == 1)
        /**
         * For HID endpoint, data size is fixed to 64 or 0, so all data streams with less than
         * DEVICE_DATA_EP_SIZE bytes needs to be padded.
         */
        if(ep == DEVICE_DATA_TX_EP){
            tx_len = (buff->fifo_pos == 0)? 0 : DEVICE_DATA_EP_SIZE;
        }
#endif
    }

    usbd_ep_write(dev, ep, &buff->tx_fifo[0], tx_len);

    // Clear and Move the tx fifo buffer by number of bytes transmitted
    memset(&buff->tx_fifo[0], 0, fifo_move_len);
    memmove(&buff->tx_fifo[0], &buff->tx_fifo[fifo_move_len], buff->fifo_pos - fifo_move_len);
    buff->fifo_pos -= fifo_move_len;
}

/**
 * @brief Callback function for RX and TX endpoints.
 * 
 * @param dev pointer to USB device
 * @param event @ref USB_EVENTS "USB event"
 * @param ep active endpoint number
 */
static void comm_rxtx(usbd_device *dev, uint8_t event, uint8_t ep) {
    if (event == usbd_evt_eptx) {
        comm_txonly(dev, ep);
    } else if (event == usbd_evt_eprx){
        comm_rxonly(dev, ep);
    }
}

static usbd_respond device_setconf (usbd_device *dev, uint8_t cfg) {
    switch (cfg) {
    case 0:
        /* deconfiguring device */
        usbd_ep_deconfig(dev, DEVICE_DATA_RX_EP);
        usbd_ep_deconfig(dev, DEVICE_DATA_TX_EP);
        usbd_reg_endpoint(dev, DEVICE_DATA_RX_EP, 0);
        usbd_reg_endpoint(dev, DEVICE_DATA_TX_EP, 0);
#if (ENABLE_HID_WEBUSB_COMM == 1)
        usbd_ep_deconfig(dev, WEBUSB_RXD_EP);
        usbd_ep_deconfig(dev, WEBUSB_TXD_EP);
        usbd_reg_endpoint(dev, WEBUSB_RXD_EP, 0);
        usbd_reg_endpoint(dev, WEBUSB_TXD_EP, 0);
#endif
        return usbd_ack;
    case 1:
        /* configuring device */
        usbd_ep_config(dev, DEVICE_DATA_RX_EP, DEVICE_DATA_EP_TYPE /*| USB_EPTYPE_DBLBUF*/, DEVICE_DATA_EP_SIZE);
        usbd_ep_config(dev, DEVICE_DATA_TX_EP, DEVICE_DATA_EP_TYPE /*| USB_EPTYPE_DBLBUF*/, DEVICE_DATA_EP_SIZE);
        usbd_reg_endpoint(dev, DEVICE_DATA_RX_EP, comm_rxtx);
        usbd_reg_endpoint(dev, DEVICE_DATA_TX_EP, comm_rxtx);
        usbd_ep_write(dev, DEVICE_DATA_TX_EP, 0, 0);
#if (ENABLE_HID_WEBUSB_COMM == 1)
        usbd_ep_config(dev, WEBUSB_RXD_EP, USB_EPTYPE_INTERRUPT /*| USB_EPTYPE_DBLBUF*/, DEVICE_DATA_EP_SIZE);
        usbd_ep_config(dev, WEBUSB_TXD_EP, USB_EPTYPE_INTERRUPT /*| USB_EPTYPE_DBLBUF*/, DEVICE_DATA_EP_SIZE);
        usbd_reg_endpoint(dev, WEBUSB_RXD_EP, comm_rxtx);
        usbd_reg_endpoint(dev, WEBUSB_TXD_EP, comm_rxtx);
        usbd_ep_write(dev, WEBUSB_TXD_EP, 0, 0);
#endif
        return usbd_ack;
    default:
        return usbd_fail;
    }
}

void usb_device_config_init(void) {
    usbd_init(&libusb_udev, &usbd_hw, USB_EP0_SIZE, ubuf, sizeof(ubuf));
    usbd_reg_config(&libusb_udev, device_setconf);
    usbd_reg_control(&libusb_udev, device_control);
    usbd_reg_descr(&libusb_udev, device_getdesc);
}

#if defined(CDC_USE_IRQ)
#if defined(STM32L052xx) || defined(STM32F070xB) || \
	defined(STM32F042x6)
#define USB_HANDLER     USB_IRQHandler
    #define USB_NVIC_IRQ    USB_IRQn
#elif defined(STM32L100xC) || defined(STM32G4)
    #define USB_HANDLER     USB_LP_IRQHandler
    #define USB_NVIC_IRQ    USB_LP_IRQn
#elif defined(USBD_PRIMARY_OTGHS) && \
    (defined(STM32F446xx) || defined(STM32F429xx))
    #define USB_HANDLER     OTG_HS_IRQHandler
    #define USB_NVIC_IRQ    OTG_HS_IRQn
    /* WA. With __WFI/__WFE interrupt will not be fired
     * faced with F4 series and OTGHS only
     */
    #undef  __WFI
    #define __WFI __NOP
#elif defined(STM32L476xx) || defined(STM32F429xx) || \
      defined(STM32F105xC) || defined(STM32F107xC) || \
      defined(STM32F446xx) || defined(STM32F411xE) || \
      defined(STM32L486xx)
    #define USB_HANDLER     OTG_FS_IRQHandler
    #define USB_NVIC_IRQ    OTG_FS_IRQn
#elif defined(STM32F103x6)
    #define USB_HANDLER     USB_LP_CAN1_RX0_IRQHandler
    #define USB_NVIC_IRQ    USB_LP_CAN1_RX0_IRQn
#elif defined(STM32F103xE)
    #define USB_HANDLER     USB_LP_CAN1_RX0_IRQHandler
    #define USB_NVIC_IRQ    USB_LP_CAN1_RX0_IRQn
#else
    #error Not supported
#endif

void USB_HANDLER(void) {
    usbd_poll(&libusb_udev);
}

void libusb_init(void) {
    usb_device_config_init();

    NVIC_EnableIRQ(USB_NVIC_IRQ);
    usbd_enable(&libusb_udev, true);
    usbd_connect(&libusb_udev, true);
}
#else
void libusb_init(void) {
    usb_device_config_init();
    usbd_enable(&libusb_udev, true);
    usbd_connect(&libusb_udev, true);
    // Need to add a call to usbd_poll() in the event loop
}
#endif

void lusb_write(const uint8_t *data, const uint16_t size, comm_libusb__interface_e interface) {
    rxtx_buffer *buff = NULL;

    // Set tx buffer based on interface paramter
#if (ENABLE_CDC_COMM == 1)
    if(interface == COMM_LIBUSB__CDC){
        buff = &data_ep_buffer;
    }
#elif (ENABLE_HID_WEBUSB_COMM == 1)
    if(interface == COMM_LIBUSB__HID){
        buff = &data_ep_buffer;
    }
    else if(interface == COMM_LIBUSB__WEBUSB){
        buff = &webusb_buffer;
    }
#else
    #error "No USB protocol defined select ENABLE_CDC_COMM or ENABLE_HID_WEBUSB_COMM"
#endif
    else{
        return;
    }

    memcpy(buff->tx_fifo, data, size);
    buff->fifo_pos = size;
}

void lusb_register_parserFunction(libusb_parser_fptr_t func) {
	parseFun = func;
}
