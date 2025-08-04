#if !defined(__USBCDC_H__)
#define __USBCDC_H__

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/stm32/f4/nvic.h>

#define CDC_DATA_PACKET_SIZE    (32U)
#define CDC_COMM_PACKET_SIZE    (16U)
#define USB_IRQ                 NVIC_OTG_FS_IRQ
#define USB_ISR                 otg_fs_isr
#define IRQ_PRI_USB             (255U)

extern usbd_device *usb_device;
extern char input_buff[CDC_DATA_PACKET_SIZE];
extern uint8_t input_buff_len;

void usbd_cdcacm_init(void);
uint8_t usbd_get_host_connected_flag(void);

#endif /* __USBCDC_H__ */
