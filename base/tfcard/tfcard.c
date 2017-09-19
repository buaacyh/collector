/*
 * tfcard.c
 *
 *  Created on: 2017年9月12日
 *      Author: saber
 */

#include "tfcard/tfcard.h"
#include "spi/spi.h"
#include "fatfs/diskio.h"

MSD_CARDINFO tfcard_info;


const eUSCI_SPI_MasterConfig tf_spi_config =
{
        EUSCI_B_SPI_CLOCKSOURCE_SMCLK,             // SMCLK Clock Source
        24000000,                                   // SMCLK = DCO = 24MHZ
        12000000,                                    // SPICLK = 12MHZ
        EUSCI_B_SPI_MSB_FIRST,                     // MSB First
        EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT,    // 时钟相位0
        EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW, // 时钟极性0
        EUSCI_SPI_4PIN_UCxSTE_ACTIVE_LOW
};


/*******************************************************************************
* Function Name  : tfcard_driver_config
* Description    : SD Card SPI Configuration
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
//tfcard SPI portd B1 P6.2,P6.3,P6.4,P6.5
void tfcard_driver_config()
{
    spi_config tfcard_spi_conf = {.port = EUSCI_B1_BASE,
                                  .gpio_group = GPIO_PORT_P6,
                                  .gpio_pins = GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5,
                                  .pconf = &tf_spi_config};



    spi_init(&tfcard_spi_conf);


    MAP_GPIO_setAsInputPin(GPIO_PORT_P6, GPIO_PIN6 );       //TF_CD
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P10, GPIO_PIN4 );       //TF_POW

    //关闭片选
    tfcard_disable();

}

/*******************************************************************************
* Function Name  : tfcard_spi_read_write
* Description    : None
* Input          : - data:
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
int tfcard_spi_read_write(uint8_t data)
{
    //< Loop while DR register in not emplty  判断SPI总线是否空闲
    //while (SPI_I2S_GetFlagStatus(SPIX, SPI_I2S_FLAG_TXE) == RESET);
    while (!(SPI_getInterruptStatus(EUSCI_B1_BASE,EUSCI_B_SPI_TRANSMIT_INTERRUPT)));

    //< Send byte through the SPI1 peripheral
    //SPI_I2S_SendData(SPIX, data);
    SPI_transmitData(EUSCI_B1_BASE,data);
    //< Wait to receive a byte
    //while (SPI_I2S_GetFlagStatus(SPIX, SPI_I2S_FLAG_RXNE) == RESET);
    while (!(SPI_getInterruptStatus(EUSCI_B1_BASE,EUSCI_B_SPI_RECEIVE_INTERRUPT)));

    // < Return the byte read from the SPI bus
    return SPI_receiveData(EUSCI_B1_BASE);
}


/*******************************************************************************
* Function Name  : tfcard_init
* Description    : SD Card initializtion
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
int tfcard_init(void)
{
    uint8_t r1,r2;
    uint8_t buff[6] = {0};
    uint16_t retry;
    DSTATUS Stat;

    //
    tfcard_driver_config();
    tfcard_spi_read_write(DUMMY_BYTE);
    //tfcard_enable();

    /* 上电延时，最开始最少发送74个时钟周期*/
    for(retry=0; retry<100; retry++)
    {
        tfcard_spi_read_write(DUMMY_BYTE);
    }

    /* 复位卡，发送CMD0直到收到回复0x01 */
    retry=20;
    do
    {
        r1=tfcard_send_command(CMD0,0,0x95);//进入IDLE状态
    }while((r1!=0x01) && retry--);

    /* 超时处理 */
    if(retry == 0)
    {
        #ifdef PRINT_INFO
        printf("Reset card into IDLE state failed!\r\n");
        #endif
    }

    /* 获取SD卡片版本 */
    retry=20;
    do
    {
        r2=tfcard_send_command_hold(CMD8, 0x000001AA,0x87);//进入IDLE状态0x87
    }while((r2!=0x01) && retry--);
    /* r1=0x01 -> V2.x, read OCR register */
    if(r2 == 0x01)
    {
        /* 4Bytes returned after CMD8 sent  */
        buff[0] = tfcard_spi_read_write(DUMMY_BYTE);              /* should be 0x00 */
        buff[1] = tfcard_spi_read_write(DUMMY_BYTE);              /* should be 0x00 */
        buff[2] = tfcard_spi_read_write(DUMMY_BYTE);              /* should be 0x01 */
        buff[3] = tfcard_spi_read_write(DUMMY_BYTE);              /* should be 0xAA */

        /* End of CMD8, chip disable and dummy byte */
        tfcard_disable();
        tfcard_spi_read_write(DUMMY_BYTE);

        /* Check voltage range be 2.7-3.6V  */
        if(buff[2]==0x01 && buff[3]==0xAA)
        {
            retry=0xFF;
            do
            {
                tfcard_send_command(CMD55,0,0x01);               /* 指令意思为下一条指令为ACMD指令0x01 should be return 0x01 */
                r1=tfcard_send_command(ACMD41,0x40000000,0x01);   /* 01表示还没有初始化完成，00表示初始化完成0x01 should be return 0x00 */
            }while(r1&&retry--);

            /* Read OCR by CMD58 */
            retry=20;
            do
            {
                r1 = tfcard_send_command_hold(CMD58, 0, 0);//0x01
            }while((r1!=0x00) && retry--);

            if(r1!=0x00)
            {
                #ifdef PRINT_INFO
                printf("Send CMD58 should return 0x00, response=0x%02x\r\n", r1);
                #endif
             }

            buff[0] = tfcard_spi_read_write(DUMMY_BYTE);
            buff[1] = tfcard_spi_read_write(DUMMY_BYTE);
            buff[2] = tfcard_spi_read_write(DUMMY_BYTE);
            buff[3] = tfcard_spi_read_write(DUMMY_BYTE);

            /* End of CMD58, chip disable and dummy byte */
            tfcard_disable();
            tfcard_spi_read_write(DUMMY_BYTE);

            /* OCR -> CCS(bit30)  1: SDV2HC  0: SDV2 */
            char OCR_CCS=(buff[0] & 0x40);
            if(OCR_CCS)
            {
                tfcard_info.CardType = CARDTYPE_SDV2HC;
                #ifdef PRINT_INFO
                printf("Card Type                     : SD V2HC\r\n");
                #endif
            }
            else
            {
                tfcard_info.CardType = CARDTYPE_SDV2;
                #ifdef PRINT_INFO
                printf("Card Type                     : SD V2\r\n");
                #endif
            }
        }



    }
    /* r2=0x05 -> V1.0如果回复0x05则是1.0版本*/
    else if(r2 == 0x05)
    {
        tfcard_info.CardType = CARDTYPE_SDV1;

        /* End of CMD8, chip disable and dummy byte */
        tfcard_disable();
        tfcard_spi_read_write(DUMMY_BYTE);

        /* 获取卡片类型，是MMC卡还是SD1.0卡，然后进行初始化 */
        /* SD1.0/MMC start initialize */
        /* Send CMD55+ACMD41, No-response is a MMC card, otherwise is a SD1.0 card */
        for(retry=0; retry<0xFFF; retry++)
        {
            r1 = tfcard_send_command(CMD55, 0, 0);            /* should be return 0x01 */
            if(r1 != 0x01)
            {
                #ifdef PRINT_INFO
                printf("Send CMD55 should return 0x01, response=0x%02x\r\n", r1);
                #endif
                return r1;
            }
            r1 = tfcard_send_command(ACMD41, 0, 0);           /* should be return 0x00 */
            if(r1 == 0x00)
            {
                retry = 0;
                break;
            }
        }

        /* 卡片类型是MMC，对MMC进行初始化 */
        if(retry == 0xFFF)
        {
            for(retry=0; retry<0xFFF; retry++)
            {
                r1 = tfcard_send_command(CMD1, 0, 0);     /* 发送CMD1，判断是不是MMC卡，should be return 0x00 */
                if(r1 == 0x00)
                {
                    retry = 0;
                    break;
                }
            }

            /* Timeout return */
            if(retry == 0xFFF)
            {
                #ifdef PRINT_INFO
                printf("Send CMD1 should return 0x00, response=0x%02x\r\n", r1);
                #endif
                return 2;
            }

            tfcard_info.CardType = CARDTYPE_MMC;
            #ifdef PRINT_INFO
            printf("Card Type                     : MMC\r\n");
            #endif
        }
        /* SD1.0 card detected, print information */
        else
        #ifdef PRINT_INFO
        {
            printf("Card Type                     : SD V1\r\n");
        }
        #endif

        /* Set spi speed high */
        //tfcard_SPIHighSpeed(1);

        /* CRC disable */
        r1 = tfcard_send_command(CMD59, 0, 0x01);
        if(r1 != 0x00)
        {
            #ifdef PRINT_INFO
            printf("Send CMD59 should return 0x00, response=0x%02x\r\n", r1);
            #endif
            return r1;      /* response error, return r1 */
        }

        /* 设置读写块数据长度 Set the block size */
        r1 = tfcard_send_command(CMD16, MSD_BLOCKSIZE, 0xFF);
        if(r1 != 0x00)
        {
            #ifdef PRINT_INFO
            printf("Send CMD16 should return 0x00, response=0x%02x\r\n", r1);
            #endif
            return r1;      /* response error, return r1 */
        }
   }
    else
    #ifdef PRINT_INFO
    printf("Send CMD8 should return 0x01, response=0x%02x\r\n", r2);
    #endif

    //取消片选，设为高速
    //tfcard_disable();
    Stat &= ~STA_NOINIT;
    return Stat;
}

/*******************************************************************************
* Function Name  : tfcard_get_card_info
* Description    : Get SD Card Information
* Input          : None
* Output         : None
* Return         : 0：NO_ERR; TRUE: Error
* Attention      : None
*******************************************************************************/
int tfcard_get_card_info(PMSD_CARDINFO tfcard_info)
{
    uint8_t r1;
    uint8_t CSD_Tab[16];
    uint8_t CID_Tab[16];

    /* Send CMD9, Read CSD */
    r1 = tfcard_send_command(CMD9, 0, 0xFF);
    if(r1 != 0x00)
    {
        return r1;
    }

    if(tfcard_read_buffer(CSD_Tab, 16, RELEASE))
    {
        return 1;
    }

    /* Send CMD10, Read CID */
    r1 = tfcard_send_command(CMD10, 0, 0xFF);
    if(r1 != 0x00)
    {
        return r1;
    }

    if(tfcard_read_buffer(CID_Tab, 16, RELEASE))
    {
        return 2;
    }

    /* Byte 0 */
    tfcard_info->CSD.CSDStruct = (CSD_Tab[0] & 0xC0) >> 6;
    tfcard_info->CSD.SysSpecVersion = (CSD_Tab[0] & 0x3C) >> 2;
    tfcard_info->CSD.Reserved1 = CSD_Tab[0] & 0x03;
    /* Byte 1 */
    tfcard_info->CSD.TAAC = CSD_Tab[1] ;
    /* Byte 2 */
    tfcard_info->CSD.NSAC = CSD_Tab[2];
    /* Byte 3 */
    tfcard_info->CSD.MaxBusClkFrec = CSD_Tab[3];
    /* Byte 4 */
    tfcard_info->CSD.CardComdClasses = CSD_Tab[4] << 4;
    /* Byte 5 */
    tfcard_info->CSD.CardComdClasses |= (CSD_Tab[5] & 0xF0) >> 4;
    tfcard_info->CSD.RdBlockLen = CSD_Tab[5] & 0x0F;
    /* Byte 6 */
    tfcard_info->CSD.PartBlockRead = (CSD_Tab[6] & 0x80) >> 7;
    tfcard_info->CSD.WrBlockMisalign = (CSD_Tab[6] & 0x40) >> 6;
    tfcard_info->CSD.RdBlockMisalign = (CSD_Tab[6] & 0x20) >> 5;
    tfcard_info->CSD.DSRImpl = (CSD_Tab[6] & 0x10) >> 4;
    tfcard_info->CSD.Reserved2 = 0; /* Reserved */
    tfcard_info->CSD.DeviceSize = (CSD_Tab[6] & 0x03) << 10;
    /* Byte 7 */
    tfcard_info->CSD.DeviceSize |= (CSD_Tab[7]) << 2;
    /* Byte 8 */
    tfcard_info->CSD.DeviceSize |= (CSD_Tab[8] & 0xC0) >> 6;
    tfcard_info->CSD.MaxRdCurrentVDDMin = (CSD_Tab[8] & 0x38) >> 3;
    tfcard_info->CSD.MaxRdCurrentVDDMax = (CSD_Tab[8] & 0x07);
    /* Byte 9 */
    tfcard_info->CSD.MaxWrCurrentVDDMin = (CSD_Tab[9] & 0xE0) >> 5;
    tfcard_info->CSD.MaxWrCurrentVDDMax = (CSD_Tab[9] & 0x1C) >> 2;
    tfcard_info->CSD.DeviceSizeMul = (CSD_Tab[9] & 0x03) << 1;
    /* Byte 10 */
    tfcard_info->CSD.DeviceSizeMul |= (CSD_Tab[10] & 0x80) >> 7;
    tfcard_info->CSD.EraseGrSize = (CSD_Tab[10] & 0x7C) >> 2;
    tfcard_info->CSD.EraseGrMul = (CSD_Tab[10] & 0x03) << 3;
    /* Byte 11 */
    tfcard_info->CSD.EraseGrMul |= (CSD_Tab[11] & 0xE0) >> 5;
    tfcard_info->CSD.WrProtectGrSize = (CSD_Tab[11] & 0x1F);
    /* Byte 12 */
    tfcard_info->CSD.WrProtectGrEnable = (CSD_Tab[12] & 0x80) >> 7;
    tfcard_info->CSD.ManDeflECC = (CSD_Tab[12] & 0x60) >> 5;
    tfcard_info->CSD.WrSpeedFact = (CSD_Tab[12] & 0x1C) >> 2;
    tfcard_info->CSD.MaxWrBlockLen = (CSD_Tab[12] & 0x03) << 2;
    /* Byte 13 */
    tfcard_info->CSD.MaxWrBlockLen |= (CSD_Tab[13] & 0xc0) >> 6;
    tfcard_info->CSD.WriteBlockPaPartial = (CSD_Tab[13] & 0x20) >> 5;
    tfcard_info->CSD.Reserved3 = 0;
    tfcard_info->CSD.ContentProtectAppli = (CSD_Tab[13] & 0x01);
    /* Byte 14 */
    tfcard_info->CSD.FileFormatGrouop = (CSD_Tab[14] & 0x80) >> 7;
    tfcard_info->CSD.CopyFlag = (CSD_Tab[14] & 0x40) >> 6;
    tfcard_info->CSD.PermWrProtect = (CSD_Tab[14] & 0x20) >> 5;
    tfcard_info->CSD.TempWrProtect = (CSD_Tab[14] & 0x10) >> 4;
    tfcard_info->CSD.FileFormat = (CSD_Tab[14] & 0x0C) >> 2;
    tfcard_info->CSD.ECC = (CSD_Tab[14] & 0x03);
    /* Byte 15 */
    tfcard_info->CSD.CSD_CRC = (CSD_Tab[15] & 0xFE) >> 1;
    tfcard_info->CSD.Reserved4 = 1;

    if(tfcard_info->CardType == CARDTYPE_SDV2HC)
    {
        /* Byte 7 */
        tfcard_info->CSD.DeviceSize = (uint16_t)(CSD_Tab[8]) *256;
        /* Byte 8 */
        tfcard_info->CSD.DeviceSize += CSD_Tab[9] ;
    }

    tfcard_info->Capacity = tfcard_info->CSD.DeviceSize * MSD_BLOCKSIZE * 1024;
    tfcard_info->BlockSize = MSD_BLOCKSIZE;

    /* Byte 0 */
    tfcard_info->CID.ManufacturerID = CID_Tab[0];
    /* Byte 1 */
    tfcard_info->CID.OEM_AppliID = CID_Tab[1] << 8;
    /* Byte 2 */
    tfcard_info->CID.OEM_AppliID |= CID_Tab[2];
    /* Byte 3 */
    tfcard_info->CID.ProdName1 = CID_Tab[3] << 24;
    /* Byte 4 */
    tfcard_info->CID.ProdName1 |= CID_Tab[4] << 16;
    /* Byte 5 */
    tfcard_info->CID.ProdName1 |= CID_Tab[5] << 8;
    /* Byte 6 */
    tfcard_info->CID.ProdName1 |= CID_Tab[6];
    /* Byte 7 */
    tfcard_info->CID.ProdName2 = CID_Tab[7];
    /* Byte 8 */
    tfcard_info->CID.ProdRev = CID_Tab[8];
    /* Byte 9 */
    tfcard_info->CID.ProdSN = CID_Tab[9] << 24;
    /* Byte 10 */
    tfcard_info->CID.ProdSN |= CID_Tab[10] << 16;
    /* Byte 11 */
    tfcard_info->CID.ProdSN |= CID_Tab[11] << 8;
    /* Byte 12 */
    tfcard_info->CID.ProdSN |= CID_Tab[12];
    /* Byte 13 */
    tfcard_info->CID.Reserved1 |= (CID_Tab[13] & 0xF0) >> 4;
    /* Byte 14 */
    tfcard_info->CID.ManufactDate = (CID_Tab[13] & 0x0F) << 8;
    /* Byte 15 */
    tfcard_info->CID.ManufactDate |= CID_Tab[14];
    /* Byte 16 */
    tfcard_info->CID.CID_CRC = (CID_Tab[15] & 0xFE) >> 1;
    tfcard_info->CID.Reserved2 = 1;

    return 0;
}

/*******************************************************************************
* Function Name  : tfcard_read_buffer
* Description    : None
* Input          : - *buff:
*                  - len:
*                  - release:
* Output         : None
* Return         : 0：NO_ERR; TRUE: Error
* Attention      : None
*******************************************************************************/
int tfcard_read_buffer(uint8_t *buff, uint16_t len, uint8_t release)
{
    uint8_t r1;
    register uint16_t retry;

    /* Card enable, Prepare to read */
    tfcard_enable();

    //等待SD卡发回数据起始令牌0xFE
    retry = 0;
    do
    {
        r1 = tfcard_spi_read_write(DUMMY_BYTE);
        retry++;
        if(retry>2000)  //2000次等待后没有应答，退出报错
        {
            tfcard_disable();
            return 1;
        }
    }while(r1 != 0xFE);

    /* Start reading */
    for(retry=0; retry<len; retry++)
    {
        *(buff+retry) = tfcard_spi_read_write(DUMMY_BYTE);
    }

    /* 2bytes dummy CRC */
    tfcard_spi_read_write(DUMMY_BYTE);
    tfcard_spi_read_write(DUMMY_BYTE);

    /* chip disable and dummy byte */
    if(release)
    {
        tfcard_disable();
        tfcard_spi_read_write(DUMMY_BYTE);
    }

    return 0;
}

/*******************************************************************************
* Function Name  : tfcard_ReadSingleBlock
* Description    : None
* Input          : - sector:
*                  - buffer:
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
int tfcard_ReadSingleBlock(uint32_t sector, uint8_t *buffer)
{
    uint8_t r1;
    //tfcard_SPIHighSpeed(1);
    /* if ver ！= SD2.0 HC, sector need <<9 */
    if(tfcard_info.CardType != CARDTYPE_SDV2HC)
    {
        sector = sector<<9;
    }

    /* Send CMD17 : Read single block command */
    r1 = tfcard_send_command(CMD17, sector, 0x01);

    if(r1 != 0x00)
    {
        return 1;
    }

    /* Start read and return the result */

    r1 = tfcard_read_buffer(buffer, MSD_BLOCKSIZE, RELEASE);

    return r1;
}

/*******************************************************************************
* Function Name  : tfcard_ReadMultiBlock
* Description    : None
* Input          : - sector:
*                  - buffer:
*                  - NbrOfSector:
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
int tfcard_ReadMultiBlock(uint32_t sector, uint8_t *buffer, uint32_t NbrOfSector)
{
    uint8_t r1;
    register uint32_t i;

    /* if ver = SD2.0 HC, sector need <<9 */
    if(tfcard_info.CardType != CARDTYPE_SDV2HC)
    {
        sector = sector<<9;
    }

    /* Send CMD18 : Read multi block command */
    r1 = tfcard_send_command(CMD18, sector, 0);
    if(r1 != 0x00)
    {
        return 1;
    }

    /* Start read   */
    for(i=0; i<NbrOfSector; i++)
    {
        if(tfcard_read_buffer(buffer+i*MSD_BLOCKSIZE, MSD_BLOCKSIZE, HOLD))
        {
            /* Send stop data transmit command - CMD12  */
            tfcard_send_command(CMD12, 0, 0);
            /* chip disable and dummy byte */
            tfcard_disable();
            return 2;
        }
    }

    /* Send stop data transmit command - CMD12 */
    tfcard_send_command(CMD12, 0, 0);

    /* chip disable and dummy byte */
    tfcard_disable();
    tfcard_spi_read_write(DUMMY_BYTE);

    return 0;
}

/*******************************************************************************
* Function Name  : tfcard_WriteSingleBlock
* Description    : None
* Input          : - sector:
*                  - buffer:
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
int tfcard_WriteSingleBlock(uint32_t sector, uint8_t *buffer)
{
    uint8_t r1;
    register uint16_t i;
    uint32_t retry;

    /* if ver = SD2.0 HC, sector need <<9 */
    if(tfcard_info.CardType != CARDTYPE_SDV2HC)
    {
        sector = sector<<9;
    }

    /* Send CMD24 : Write single block command */
    r1 = tfcard_send_command(CMD24, sector, 0);

    if(r1 != 0x00)
    {
        return 1;
    }

    /* Card enable, Prepare to write */
    tfcard_enable();
    tfcard_spi_read_write(DUMMY_BYTE);
    tfcard_spi_read_write(DUMMY_BYTE);
    tfcard_spi_read_write(DUMMY_BYTE);
    /* Start data write token: 0xFE */
    tfcard_spi_read_write(0xFE);

    /* Start single block write the data buffer */
    for(i=0; i<MSD_BLOCKSIZE; i++)
    {
        tfcard_spi_read_write(*buffer++);
    }

    /* 2Bytes dummy CRC */
    tfcard_spi_read_write(DUMMY_BYTE);
    tfcard_spi_read_write(DUMMY_BYTE);

    /* MSD card accept the data */
    r1 = tfcard_spi_read_write(DUMMY_BYTE);
    if((r1&0x1F) != 0x05)
    {
        tfcard_disable();
        return 2;
    }

    /* Wait all the data programm finished */
    retry = 0;
    while(tfcard_spi_read_write(DUMMY_BYTE) == 0x00)
    {
        /* Timeout return */
        if(retry++ == 0x40000)
        {
            //tfcard_disable();
            return 3;
        }
    }

    /* chip disable and dummy byte */
    tfcard_disable();
    tfcard_spi_read_write(DUMMY_BYTE);

    return 0;
}

/*******************************************************************************
* Function Name  : tfcard_WriteMultiBlock
* Description    : None
* Input          : - sector:
*                  - buffer:
*                  - NbrOfSector:
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
int tfcard_WriteMultiBlock(uint32_t sector, uint8_t *buffer, uint32_t NbrOfSector)
{
    uint8_t r1;
    register uint16_t i;
    register uint32_t n;
    uint32_t retry;

    /* if ver = SD2.0 HC, sector need <<9 */
    if(tfcard_info.CardType != CARDTYPE_SDV2HC)
    {
        sector = sector<<9;
    }

    /* Send command ACMD23 berfore multi write if is not a MMC card */
    if(tfcard_info.CardType != CARDTYPE_MMC)
    {
        tfcard_send_command(ACMD23, NbrOfSector, 0x00);
    }

    /* Send CMD25 : Write nulti block command   */
    r1 = tfcard_send_command(CMD25, sector, 0);

    if(r1 != 0x00)
    {
        return 1;
    }

    /* Card enable, Prepare to write */
    tfcard_enable();
    tfcard_spi_read_write(DUMMY_BYTE);
    tfcard_spi_read_write(DUMMY_BYTE);
    tfcard_spi_read_write(DUMMY_BYTE);

    for(n=0; n<NbrOfSector; n++)
    {
        /* Start multi block write token: 0xFC */
        tfcard_spi_read_write(0xFC);

        for(i=0; i<MSD_BLOCKSIZE; i++)
        {
            tfcard_spi_read_write(*buffer++);
        }

        /* 2Bytes dummy CRC */
        tfcard_spi_read_write(DUMMY_BYTE);
        tfcard_spi_read_write(DUMMY_BYTE);

        /* MSD card accept the data */
        r1 = tfcard_spi_read_write(DUMMY_BYTE);
        if((r1&0x1F) != 0x05)
        {
            tfcard_disable();
            return 2;
        }

        /* Wait all the data programm finished  */
        retry = 0;
        while(tfcard_spi_read_write(DUMMY_BYTE) != 0xFF)
        {
            /* Timeout return */
            if(retry++ == 0x40000)
            {
                tfcard_disable();
                return 3;
            }
        }
    }

    /* Send end of transmit token: 0xFD */
    r1 = tfcard_spi_read_write(0xFD);
    if(r1 == 0x00)
    {
        return 4;
    }

    /* Wait all the data programm finished */
    retry = 0;
    while(tfcard_spi_read_write(DUMMY_BYTE) != 0xFF)
    {
        /* Timeout return */
        if(retry++ == 0x40000)
        {
            tfcard_disable();
            return 5;
        }
    }

    /* chip disable and dummy byte */
    tfcard_disable();
    tfcard_spi_read_write(DUMMY_BYTE);

    return 0;
}


/*******************************************************************************
* Function Name  : tfcard_send_command
* Description    : None
* Input          : - cmd:
*                  - arg:
*                  - crc:
* Output         : None
* Return         : R1 value, response from card
* Attention      : None
*******************************************************************************/
int tfcard_send_command(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    uint8_t r1;
    uint8_t retry;

    /* Dummy byte and chip enable */
    tfcard_spi_read_write(DUMMY_BYTE);
    tfcard_enable();

    /* Command, argument and crc */
    tfcard_spi_read_write(cmd | 0x40);
    tfcard_spi_read_write(arg >> 24);
    tfcard_spi_read_write(arg >> 16);
    tfcard_spi_read_write(arg >> 8);
    tfcard_spi_read_write(arg);
    tfcard_spi_read_write(crc);

    /* Wait response, quit till timeout */
    for(retry=0; retry<200; retry++)
    {
        r1 = tfcard_spi_read_write(DUMMY_BYTE);
        if(r1 != 0xFF)
        {
            break;
        }
    }

    /* Chip disable and dummy byte */
    tfcard_disable();

    return r1;
}

/*******************************************************************************
* Function Name  : tfcard_send_command_hold
* Description    : None
* Input          : - cmd:
*                  - arg:
*                  - crc:
* Output         : None
* Return         : R1 value, response from card
* Attention      : None
*******************************************************************************/
int tfcard_send_command_hold(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    uint8_t r1;
    uint8_t retry;

    /* Dummy byte and chip enable */
    tfcard_enable();
    tfcard_spi_read_write(DUMMY_BYTE);
    tfcard_spi_read_write(DUMMY_BYTE);
    tfcard_spi_read_write(DUMMY_BYTE);

    /* Command, argument and crc */
    tfcard_spi_read_write(cmd | 0x40);
    tfcard_spi_read_write(arg >> 24);
    tfcard_spi_read_write(arg >> 16);
    tfcard_spi_read_write(arg >> 8);
    tfcard_spi_read_write(arg);
    tfcard_spi_read_write(crc);

    /* Wait response, quit till timeout */
    for(retry=0; retry<200; retry++)
    {
        r1 = tfcard_spi_read_write(DUMMY_BYTE);
        if(r1 != 0xFF)
        {
            break;
        }
    }

    return r1;
}
