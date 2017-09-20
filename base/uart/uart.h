/*
 * uart.h
 *
 *  Created on: 2017Äê9ÔÂ13ÈÕ
 *      Author: saber
 */

#ifndef BASE_UART_UART_H_
#define BASE_UART_UART_H_

#include "comm.h"

typedef struct uart_config_t{
    uint8_t gpio_group;
    uint16_t gpio_pins;
    uint32_t port;
    eUSCI_UART_Config* pconf;
}uart_config;

extern void uart_init(uart_config *uart_conf);
extern void uart_close(uart_config *uart_conf);
#endif /* BASE_UART_UART_H_ */
