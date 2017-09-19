/*
 * uart.c
 *
 *  Created on: 2017Äê9ÔÂ13ÈÕ
 *      Author: saber
 */

#include "uart.h"

void uart_init(uart_config *uart_conf){
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(uart_conf->gpio_group,
                     uart_conf->gpio_pins, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_UART_initModule(uart_conf->port, uart_conf->pconf);
    MAP_UART_enableModule(uart_conf->port);
}
