/*
 * tfcard.h
 *
 *  Created on: 2017年9月12日
 *      Author: saber
 */

#ifndef TFCARD_TFCARD_H_
#define TFCARD_TFCARD_H_

/* Private define ------------------------------------------------------------*/
#define CARDTYPE_MMC             0x00
#define CARDTYPE_SDV1            0x01
#define CARDTYPE_SDV2            0x02
#define CARDTYPE_SDV2HC          0x04

#define DUMMY_BYTE               0xFF
#define MSD_BLOCKSIZE            512

#include "comm.h"


/* SD/MMC command list - SPI mode */
//鍗＄殑璇嗗埆銆佸垵濮嬪寲绛夊熀鏈懡浠ら泦
#define CMD0                     0       /* Reset澶嶄綅SD鍗� */
#define CMD1                     1       /* Send Operator Condition 璇籓CR瀵勫瓨鍣� */
#define CMD2                     2       /*杩斿洖CID淇℃伅*/
#define CMD3                     3
#define CMD6                     6
#define CMD7                     7
#define CMD8                     8       /* Send Interface Condition - SEND_IF_COND */
#define CMD9                     9       /* Read CSD 璇籆SD瀵勫瓨鍣�*/
#define CMD10                    10      /* Read CID 璇籆ID瀵勫瓨鍣�*/
#define CMD12                    12      /* Stop data transmit 鍋滄璇诲鍧楁椂鐨勬暟鎹紶杈�*/

//璇诲崱鍛戒护
#define CMD16                    16      /* Set block size, should return 0x00 璁剧疆鍧楃殑闀垮害*/
#define CMD17                    17      /* Read single block 璇诲崟鍧�*/
#define CMD18                    18      /* Read multi block 璇诲鍧�,鐩磋嚦涓绘満鍙戦�丆MD12涓烘*/

#define ACMD23                   23      /* Prepare erase N-blokcs before multi block write */

#define CMD24                    24      /* Write single block 鍐欏崟鍧�*/
#define CMD25                    25      /* Write multi block 鍐欏鍧�*/

#define ACMD41                   41      /* should return 0x00 */
#define CMD55                    55      /* should return 0x01 */
#define CMD58                    58      /* Read OCR */
#define CMD59                    59      /* CRC disable/enbale, should return 0x00 */

#define tfcard_enable()              SPI_selectFourPinFunctionality(EUSCI_B1_BASE,EUSCI_SPI_ENABLE_SIGNAL_FOR_4WIRE_SLAVE )
#define tfcard_disable()             SPI_selectFourPinFunctionality(EUSCI_B1_BASE,EUSCI_SPI_PREVENT_CONFLICTS_WITH_OTHER_MASTERS)
#define tfcard_power_on()            ;
#define tfcard_insert()              0

//#define open_tfcard()            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P10,GPIO_PIN4)
//#define close_tfcard()           MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P10,GPIO_PIN4)

/* Private typedef -----------------------------------------------------------*/
enum _CD_HOLD
{
    HOLD = 0,
    RELEASE = 1,
};



typedef struct               /* Card Specific Data */
{
    uint8_t  CSDStruct;            /* CSD structure */
    uint8_t  SysSpecVersion;       /* System specification version */
    uint8_t  Reserved1;            /* Reserved */
    uint8_t  TAAC;                 /* Data read access-time 1 */
    uint8_t  NSAC;                 /* Data read access-time 2 in CLK cycles */
    uint8_t  MaxBusClkFrec;        /* Max. bus clock frequency */
    uint16_t CardComdClasses;      /* Card command classes */
    uint8_t  RdBlockLen;           /* Max. read data block length */
    uint8_t  PartBlockRead;        /* Partial blocks for read allowed */
    uint8_t  WrBlockMisalign;      /* Write block misalignment */
    uint8_t  RdBlockMisalign;      /* Read block misalignment */
    uint8_t  DSRImpl;              /* DSR implemented */
    uint8_t  Reserved2;            /* Reserved */
    uint32_t DeviceSize;           /* Device Size */
    uint8_t  MaxRdCurrentVDDMin;   /* Max. read current @ VDD min */
    uint8_t  MaxRdCurrentVDDMax;   /* Max. read current @ VDD max */
    uint8_t  MaxWrCurrentVDDMin;   /* Max. write current @ VDD min */
    uint8_t  MaxWrCurrentVDDMax;   /* Max. write current @ VDD max */
    uint8_t  DeviceSizeMul;        /* Device size multiplier */
    uint8_t  EraseGrSize;          /* Erase group size */
    uint8_t  EraseGrMul;           /* Erase group size multiplier */
    uint8_t  WrProtectGrSize;      /* Write protect group size */
    uint8_t  WrProtectGrEnable;    /* Write protect group enable */
    uint8_t  ManDeflECC;           /* Manufacturer default ECC */
    uint8_t  WrSpeedFact;          /* Write speed factor */
    uint8_t  MaxWrBlockLen;        /* Max. write data block length */
    uint8_t  WriteBlockPaPartial;  /* Partial blocks for write allowed */
    uint8_t  Reserved3;            /* Reserded */
    uint8_t  ContentProtectAppli;  /* Content protection application */
    uint8_t  FileFormatGrouop;     /* File format group */
    uint8_t  CopyFlag;             /* Copy flag (OTP) */
    uint8_t  PermWrProtect;        /* Permanent write protection */
    uint8_t  TempWrProtect;        /* Temporary write protection */
    uint8_t  FileFormat;           /* File Format */
    uint8_t  ECC;                  /* ECC code */
    uint8_t  CSD_CRC;              /* CSD CRC */
    uint8_t  Reserved4;            /* always 1*/
}
MSD_CSD;

typedef struct               /*Card Identification Data*/
{
    uint8_t  ManufacturerID;       /* ManufacturerID */
    uint16_t OEM_AppliID;          /* OEM/Application ID */
    uint32_t ProdName1;            /* Product Name part1 */
    uint8_t  ProdName2;            /* Product Name part2*/
    uint8_t  ProdRev;              /* Product Revision */
    uint32_t ProdSN;               /* Product Serial Number */
    uint8_t  Reserved1;            /* Reserved1 */
    uint16_t ManufactDate;         /* Manufacturing Date */
    uint8_t  CID_CRC;              /* CID CRC */
    uint8_t  Reserved2;            /* always 1 */
}
MSD_CID;

typedef struct
{
    MSD_CSD CSD;
    MSD_CID CID;
    uint32_t Capacity;              /* Card Capacity */
    uint32_t BlockSize;             /* Card Block Size */
    uint16_t RCA;
    uint8_t CardType;
    uint32_t SpaceTotal;            /* Total space size in file system */
    uint32_t SpaceFree;              /* Free space size in file system */
}
MSD_CARDINFO, *PMSD_CARDINFO;

extern MSD_CARDINFO tfcard_info;

extern int tfcard_init();
extern int tfcard_get_card_info(PMSD_CARDINFO cardinfo);
extern int tfcard_ReadSingleBlock(uint32_t sector, uint8_t *buffer);
extern int tfcard_ReadMultiBlock(uint32_t sector, uint8_t *buffer, uint32_t NbrOfSector);
extern int tfcard_WriteSingleBlock(uint32_t sector, uint8_t *buffer);
extern int tfcard_WriteMultiBlock(uint32_t sector, uint8_t *buffer, uint32_t NbrOfSector);

extern void tfcard_driver_config(void);

extern int tfcard_spi_read_write(uint8_t data);
extern int tfcard_send_command(uint8_t cmd, uint32_t arg, uint8_t crc);
extern int tfcard_send_command_hold(uint8_t cmd, uint32_t arg, uint8_t crc);
extern int tfcard_read_buffer(uint8_t *buff, uint16_t len, uint8_t release);

#endif /* TFCARD_TFCARD_H_ */
