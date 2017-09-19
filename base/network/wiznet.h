

#ifndef WIZNET_CONFIG
#define WIZNET_CONFIG

#include "comm.h"

#define wiznet_disable()          SPI_selectFourPinFunctionality(EUSCI_A1_BASE,EUSCI_SPI_ENABLE_SIGNAL_FOR_4WIRE_SLAVE )
#define wiznet_enable()         SPI_selectFourPinFunctionality(EUSCI_A1_BASE,EUSCI_SPI_PREVENT_CONFLICTS_WITH_OTHER_MASTERS)
#define WIZNET_BYTE              0x00
#define WIZNET_ERROR             0
//ifdefine with debug printf
#define _WIZNET_DEBUG_


extern void wizchip_spi_configuration(void);
extern void wiznet_init(void);
extern void    wizchip_cs_sel();
extern void    wizchip_cs_desel();
extern uint8_t wizchip_spi_readbyte(void);
extern void    wizchip_spi_writebyte(uint8_t wb);
extern void    wizchip_spi_readburst(uint8_t* pBuf, uint16_t len);
extern void    wizchip_spi_writeburst(uint8_t* pBuf, uint16_t len);

#endif
