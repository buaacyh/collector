/*
 * spi.h
 *
 *  Created on: 2017Äê9ÔÂ12ÈÕ
 *      Author: saber
 */

#ifndef SPI_SPI_H_
#define SPI_SPI_H_
#include "comm.h"

typedef struct spi_config_t{
    uint8_t gpio_group;
    uint16_t gpio_pins;
    uint32_t port;
    eUSCI_SPI_MasterConfig* pconf;
}spi_config;

extern void spi_init(spi_config *spi_config);

#endif /* SPI_SPI_H_ */
