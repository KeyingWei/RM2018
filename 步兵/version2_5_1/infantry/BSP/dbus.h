#ifndef __DBUS_H__
#define __DBUS_H__

void DbusConfig(void);
void DMA2_Stream5_IRQHandler(void);

extern volatile unsigned char sbus_rx_buffer[25];

#endif

