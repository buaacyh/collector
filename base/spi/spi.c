/*
 * spi.c
 *
 *  Created on: 2017Äê9ÔÂ12ÈÕ
 *      Author: saber
 */

#include "spi.h"

void spi_init(spi_config* spi_config){
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(spi_config->gpio_group,
                                                   spi_config->gpio_pins, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_SPI_initMaster(spi_config->port, spi_config->pconf);
    MAP_SPI_enableModule(spi_config->port);
}
