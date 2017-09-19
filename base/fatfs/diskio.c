/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2016        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "fatfs/diskio.h"		/* FatFs lower layer API */

/* Definitions of physical drive number for each drive */
#define DEV_RAM		0	/* Example: Map Ramdisk to physical drive 0 */
#define DEV_MMC		1	/* Example: Map MMC/SD card to physical drive 1 */
#define DEV_USB		2	/* Example: Map USB MSD to physical drive 2 */


/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/


DSTATUS Stat=STA_NOINIT;

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
    if (pdrv) return STA_NOINIT;     /* Supports only drive 0 */

    return Stat;    /* Return disk status */
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
	{
		int result;
	
		switch (pdrv) 
			{
				case DEV_RAM :
						result = tfcard_init();
						if(result==0) 
							{
								return RES_OK; 
							}
						else
							{ 
								return STA_NOINIT; 
							}
	
				case DEV_MMC :
			
						return STA_NOINIT;
	
				case DEV_USB :
			
						return STA_NOINIT;
	
			}
		return STA_NOINIT;
	}


/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
    tfcard_enable();
	int result;

	if( !count )
	{    
    	return RES_PARERR;  /* count不能等于0，否则返回参数错误 */
	}

	switch (pdrv) 
		{
			case DEV_RAM :
					if(count==1)			/* 1个sector的读操作 */ 	 
					{	
						result =  tfcard_ReadSingleBlock( sector ,buff );
						if(result == 0) 
							{ return RES_OK; }
						else
							{ return RES_ERROR; }	 
					}												 
					else					/* 多个sector的读操作 */	 
					{  
						result = tfcard_ReadMultiBlock( sector , buff ,count);
						if(result == 0)
							{ return RES_OK; }
						else
							{ return RES_ERROR; } 
					}	

			case DEV_MMC :
					if(count==1)			/* 1个sector的读操作 */ 	 
					{ return RES_OK; }												 
					else					/* 多个sector的读操作 */	 
					{ return RES_OK; } 

			case DEV_USB :
					if(count==1)			/* 1个sector的读操作 */ 	 
					{ return RES_OK; }												 
					else					/* 多个sector的读操作 */	 
					{ return RES_OK; } 
	}

	return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
	//DRESULT res;
	int result;

	if( !count )
	{    
            return RES_PARERR;  /* count不能等于0，否则返回参数错误 */
	}

	switch (pdrv) {
	case DEV_RAM :
        	if(count==1)            /* 1个sector的写操作 */      
			{   
            	result = tfcard_WriteSingleBlock( sector , (BYTE *)(&buff[0]) );
                if(result == 0)
					{ return RES_OK; }
                else
					{ return RES_ERROR; } 
			}                                                
			else                    /* 多个sector的写操作 */    
			{  
                result = tfcard_WriteMultiBlock( sector , (BYTE *)(&buff[0]) , count );
                if(result == 0)
					{ return RES_OK; }
                else
					{ return RES_ERROR; }   
			}   

	case DEV_MMC :
		if(count==1)			/* 1个sector的写操作 */ 	 
		{ return RES_OK; }												 
		else					/* 多个sector的写操作 */	 
		{ return RES_OK; } 


	case DEV_USB :
		if(count==1)			/* 1个sector的写操作 */ 	 
		{ return RES_OK; }												 
		else					/* 多个sector的写操作 */	 
		{ return RES_OK; } 
	}

	return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{

	switch (pdrv) 
		{
			case DEV_RAM :
	
			tfcard_get_card_info(&tfcard_info);
			switch (cmd) 
			{
				case CTRL_SYNC : 
					return RES_OK;
				case GET_SECTOR_COUNT : 
					*(DWORD*)buff = tfcard_info.Capacity/tfcard_info.BlockSize;
					return RES_OK;
				case GET_BLOCK_SIZE :
					*(WORD*)buff = tfcard_info.BlockSize;
					return RES_OK;	
				case CTRL_POWER :
					break;
				case CTRL_LOCK :
					break;
				case CTRL_EJECT :
					break;
				case MMC_GET_TYPE :
					break;
				case MMC_GET_CSD :
					break;
				case MMC_GET_CID :
					break;
				case MMC_GET_OCR :
					break;
				case MMC_GET_SDSTAT :
					break;	
			}


			case DEV_MMC :

				return RES_OK;

			case DEV_USB :

				return RES_OK;
		}

	return RES_PARERR;
}



