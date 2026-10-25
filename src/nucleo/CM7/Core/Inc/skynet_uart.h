#ifndef SKYNET_UART_H
#define SKYNET_UART_H

/** @file skynet_uart.h
 *  @brief USART3 (ST-LINK virtual COM port) at ::SKYNET_LINK_BAUD into skynet_link's byte ring. */

/** @brief Clock, pins, 8N1 receive with FIFO, interrupt on. Call once after the HAL clock setup. */
void skynet_uart_init(void);
#endif
