/*
 * wiznet_config.c
 *
 *  Created on: 2017年7月27日
 *      Author: saber
 */

#include "network/wiznet.h"
#include "network/Ethernet/wizchip_conf.h"
#include "network/Ethernet/socket.h"
#include "spi/spi.h"




const eUSCI_SPI_MasterConfig wiznet_spi_config =
{
        EUSCI_A_SPI_CLOCKSOURCE_SMCLK,             // SMCLK Clock Source
        24000000,                                   // SMCLK = DCO = 24MHZ
        100000,                                    // SPICLK = 100KHZ
        EUSCI_A_SPI_MSB_FIRST,                     // MSB First
        EUSCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT,    // 时钟相位0
        EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_HIGH, // 时钟极性1
        EUSCI_SPI_4PIN_UCxSTE_ACTIVE_LOW
};

//SPI port B2--P3.4,P3.5,P3.6,P3.7
void wizchip_spi_configuration(){
    //initialize spi port
    spi_config wiznet_spi_conf = {.port = EUSCI_B2_BASE,
                                      .gpio_group = GPIO_PORT_P3,
                                      .gpio_pins = GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7,
                                      .pconf = &wiznet_spi_config};

    spi_init(&wiznet_spi_conf);

    //initialize wiznet
    reg_wizchip_cs_cbfunc(wizchip_cs_sel, wizchip_cs_desel);
    reg_wizchip_spi_cbfunc(wizchip_spi_readbyte, wizchip_spi_writebyte);
    //reg_wizchip_spiburst_cbfunc(wizchip_spi_readburst, wizchip_spi_writeburst);

}

void wiznet_init(){
    ctlwizchip(CW_RESET_WIZCHIP,NULL);
    //set the timeout parameters
    wiz_NetTimeout mWIZNETTIMEOUT = {.retry_cnt = 5, .time_100us = 1000};
    wizchip_settimeout(&mWIZNETTIMEOUT);
    wizchip_gettimeout(&mWIZNETTIMEOUT);
#ifdef _WIZNET_DEBUG_
    printf("retry_cnt = %d, time 100us = %d\n", mWIZNETTIMEOUT.retry_cnt, mWIZNETTIMEOUT.time_100us);
#endif
    //set the ip wiznet w5500 network parameters as ip, mask, gateway etc.
    wiz_NetInfo mWIZNETINFO= {.mac = {0x00, 0x08, 0xDC, 0x44, 0xef, 0x62},
            .ip = {192, 168, 1, 105},
            .sn = {255, 255, 255, 0},
            .gw = {192, 168, 1, 1},
            .dns = {192,168,1,1},
            .dhcp = NETINFO_STATIC };
    ctlnetwork(CN_SET_NETINFO, (void *)&mWIZNETINFO);
#ifdef _WIZNET_DEBUG_
    ctlnetwork(CN_GET_NETINFO, (void *)&mWIZNETINFO);
    printf("mac: 0x%02X-0x%02X-0x%02X-0x%02X-0x%02X-0x%02X\n",
           mWIZNETINFO.mac[0], mWIZNETINFO.mac[1], mWIZNETINFO.mac[2], mWIZNETINFO.mac[3], mWIZNETINFO.mac[4], mWIZNETINFO.mac[5]);
    printf("ip: %d.%d.%d.%d\n", mWIZNETINFO.ip[0],mWIZNETINFO.ip[1],mWIZNETINFO.ip[2],mWIZNETINFO.ip[3]);
    printf("sn: %d.%d.%d.%d\n", mWIZNETINFO.sn[0],mWIZNETINFO.sn[1],mWIZNETINFO.sn[2],mWIZNETINFO.sn[3]);
    printf("gw: %d.%d.%d.%d\n", mWIZNETINFO.gw[0],mWIZNETINFO.gw[1],mWIZNETINFO.gw[2],mWIZNETINFO.gw[3]);
#endif
    return;
}

int wizchip_spi_read_write(uint8_t data)
{
    //< Loop while DR register in not emplty  判断SPI总线是否空闲
    //while (SPI_I2S_GetFlagStatus(SPIX, SPI_I2S_FLAG_TXE) == RESET);
    while (!(SPI_getInterruptStatus(EUSCI_B2_BASE,EUSCI_B_SPI_TRANSMIT_INTERRUPT)));

    //< Send byte through the SPI1 peripheral
    //SPI_I2S_SendData(SPIX, data);
    SPI_transmitData(EUSCI_A1_BASE,data);
    //< Wait to receive a byte
    //while (SPI_I2S_GetFlagStatus(SPIX, SPI_I2S_FLAG_RXNE) == RESET);
    while (!(SPI_getInterruptStatus(EUSCI_B2_BASE,EUSCI_B_SPI_RECEIVE_INTERRUPT)));

    // < Return the byte read from the SPI bus
    return SPI_receiveData(EUSCI_B2_BASE);
}
void wizchip_cs_sel(){
    wiznet_enable();
}

void wizchip_cs_desel(){
    wiznet_disable();
}
uint8_t wizchip_spi_readbyte(void){
    return wizchip_spi_read_write(WIZNET_BYTE);
}

void wizchip_spi_writebyte(uint8_t wb){
    wizchip_spi_read_write(wb);
}

void wizchip_spi_readburst(uint8_t* pBuf, uint16_t len){
    register uint16_t retry;
    for(retry=0; retry<len; retry++)
    {
      *(pBuf+retry) = wizchip_spi_read_write(WIZNET_BYTE);
    }
}
void wizchip_spi_writeburst(uint8_t* pBuf, uint16_t len){
    register uint16_t retry;
    for(retry=0; retry<len; retry++)
    {
        wizchip_spi_read_write(*pBuf++);
    }
}

