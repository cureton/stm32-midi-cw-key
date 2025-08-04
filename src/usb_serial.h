#if !defined(__USB_SERIAL_H__)
#define __USB_SERIAL_H__

#include "usbcdc.h"
#include "delay.h"

#define FLOAT_PRECISION (6U)

/* 1 = resistor can be dis-/connected via a transistor and GPIOB9 signal; 0 = resistor is permanently connected: */
#define USB_DP_1K5_PULLUP_SWITCHABLE_VIA_GPIOB9 (0U)

void USB_Serial_init(void);
void USB_Serial_write(char *buff);
void USB_Serial_write_u32(uint32_t buff);
void USB_Serial_write_i32(int32_t buff);
void USB_Serial_write_float(float buff);
char *USB_Serial_read(void);
uint32_t USB_Serial_read_u32(void);
int32_t USB_Serial_read_i32(void);
float USB_Serial_read_float(void);

#endif /* __USB_SERIAL_H__ */
