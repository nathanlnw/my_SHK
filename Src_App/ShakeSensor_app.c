#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "stm32l0xx_hal.h"
#include <stdio.h>
#include <string.h>
#include "stdlib.h"
#include "ShakeSensor_app.h"
#include "adxl375.h"
#include "FM25V10.h"
#include "Hmc5883l.h"
#include "rtc8564.h"
#include "Main.h"


RTCTIME rtc_current;
 
ADXL_STATUS interryptflag = ADXL_INTERRUPT_IDLE;
KEY_ST  K1_Value;
u32   Shake_times=0;   // creat for  debugging   


u8	Ack_buf[300];
u8	Ack_len=0;	

uint8_t  L3GD_reg[8];
uint8_t  ADX_reg[8];
u8       Reg_common[8];
u8  rtc_tmp[10];
u8   eep_reg[30];
u8   eep_read[30];
u8   Adx375_trigger_flag=0;
uint8_t byData[13]={0};


extern u16  Protocol7E_Encode(u8 *Dest, u8 *Src, u16 srclen);

unsigned bcd2bin(unsigned char val)
{
	return (val & 0x0f) + (val >> 4) * 10;
}

unsigned char bin2bcd(unsigned val)
{
	return ((val / 10) << 4) + val % 10;
}


void writei2c_byte(I2C_HandleTypeDef *hi2c,uint8_t addr,uint8_t reg_address,uint8_t reg_data)
{
	uint8_t wr_data[2] = {reg_address,reg_data};
    while(HAL_I2C_Master_Transmit(hi2c,addr,wr_data,2,40) != HAL_OK)
    {
        if(HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF)
        {
        	printf("WRITE_I2C ERROR!!!!\n");
			break;
        }
    }
}

uint8_t readi2c_nbyte(I2C_HandleTypeDef *hi2c,uint8_t addr,uint8_t reg_address,uint8_t *data,uint8_t size)
{
    while(HAL_I2C_Master_Transmit(hi2c,addr,&reg_address,1,40) != HAL_OK)
    {
        if(HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF)
        {
        	;//printf("read_WRITE_I2C ERROR!!!!\n");
        }
    }
    
    if(HAL_I2C_Master_Receive(hi2c,addr+1,data,size,40) != HAL_OK)
    {
        if(HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF)
        {
        	;//printf("READ_I2C ERROR!!!!\n");
        }
    }
    return 0;
}


//  part 1  ============>   HMC58831     I2C1

/***********************************HMC5883l**************************/
void  HMC5883L_Init(void)   //  3 -Aixs    Digital  Compass     µØ´Å´«¸ÐÆ÷  1¡ã µ½ 2 ¡ã ¾«¶È
{
    u8  Hmc5993l_ID[3];
    printf("\r\n  HMC5883L init ");
	writei2c_byte(&hi2c1,HMC5883L_WR,HMC5883L_REGA,0x11);   //75Hz
	writei2c_byte(&hi2c1,HMC5883L_WR,HMC5883L_REGB,0x00);   //75Hz
	writei2c_byte(&hi2c1,HMC5883L_WR,HMC5883L_MODE,0);   //Á¬Ðø²âÁ¿Ä£Ê½
    readi2c_nbyte(&hi2c1,HMC5883L_WR,HMC5883L_IRA,Hmc5993l_ID,3);
	printf("\r\n  HMC5883L init 1- ID=%02X%02X%02X \r\n",Hmc5993l_ID[0],Hmc5993l_ID[1],Hmc5993l_ID[2]);
	readi2c_nbyte(&hi2c1,HMC5883L_WR,HMC5883L_REGA,Hmc5993l_ID,3);
	printf("\r\n  HMC5883L init 2- ID=%02X%02X%02X \r\n",Hmc5993l_ID[0],Hmc5993l_ID[1],Hmc5993l_ID[2]); 
}

void HMC5883L_Get(void)
{
   
   uint8_t	HMMC_reg[8];
   //u8 i=0;
   
	if(HAL_GPIO_ReadPin(SHK_DRDY_GPIO_Port,SHK_DRDY_Pin) == 1)   // ÅÐ¶ÏÐ¾Æ¬µÄRDY  ¹Ü½Å
	{
		memset(HMMC_reg,0x88,8);
		 if(HAL_OK == readi2c_nbyte(&hi2c1,HMC5883L_WR,HMC5883L_HX_H,HMMC_reg,6))   //¶à¶Á¶Á³ö´«¸ÐÆ÷Êý¾Ý
	     {  
	        //  LSB  mode
	        Project_SHK.COMPASS_HMC_6BYTES[0]=HMMC_reg[1];  
			Project_SHK.COMPASS_HMC_6BYTES[1]=HMMC_reg[0];
			Project_SHK.COMPASS_HMC_6BYTES[2]=HMMC_reg[3];
			Project_SHK.COMPASS_HMC_6BYTES[3]=HMMC_reg[2];
			Project_SHK.COMPASS_HMC_6BYTES[4]=HMMC_reg[5];
			Project_SHK.COMPASS_HMC_6BYTES[5]=HMMC_reg[4];
			
       #ifdef  DEBUG_SHK
		    Project_SHK.COMPASS_X_vaule=(u16)(HMMC_reg[0]<<8)+(u16)HMMC_reg[1];
		    Project_SHK.COMPASS_Y_vaule=(u16)(HMMC_reg[2]<<8)+(u16)HMMC_reg[3];
		    Project_SHK.COMPASS_Z_vaule=(u16)(HMMC_reg[4]<<8)+(u16)HMMC_reg[5];

	      /*    
	       printf("\r\n 6 byte :");
			for(i=0;i<6;i++)
				{
		            printf("%02X ",HMMC_reg[i]);
				}
			printf("\r\n Stuff byte :");
			for(i=0;i<6;i++)
				{
		            printf("%02X ",Project_SHK.COMPASS_HMC_6BYTES[i]);
				}
			
		    printf("\r\n HMC5883L  X: 0x%0004X	Y: 0x%0004X  Z: 0x%0004X\r\n",Project_SHK.COMPASS_X_vaule,Project_SHK.COMPASS_Y_vaule,Project_SHK.COMPASS_Z_vaule);
		    */
         #endif
		 } 
	 }	 


} 










// part 2   ======>     Out RTC chip   I2C2

void RTC8564_Init(void)
{
	HAL_Delay(1000);
	writei2c_byte(&hi2c2,RTC_WR,reg_ctr1,0x00);
	//½ûÖ¹ÖÐ¶Ï
	writei2c_byte(&hi2c2,RTC_WR,reg_ctr2,0x00);	
	//ÉèÖÃÊ±¼ä2016/2/26 10:50:30 5
	rtc_current.year = 0x10;
	rtc_current.month = 0x02;
	rtc_current.day = 0x26;
	rtc_current.hour = 0x10;
	rtc_current.min = 0x11;
	rtc_current.sec=0x12;
	
	//RTC8564_Set(rtc_current);
	//½ûÖ¹ËùÓÐµÄÄÖÖÓ
	writei2c_byte(&hi2c2,RTC_WR,reg_alarm_min,0x80);	
	writei2c_byte(&hi2c2,RTC_WR,reg_alarm_hour,0x80);	
	writei2c_byte(&hi2c2,RTC_WR,reg_alarm_day,0x80);	
	writei2c_byte(&hi2c2,RTC_WR,reg_alarm_week,0x80);	
	//½ûÖ¹Ê±ÖÓÒý½ÅÊä³ö
	writei2c_byte(&hi2c2,RTC_WR,reg_clk,0x7F);
	//½ûÖ¹¶¨Ê±ÖÐ¶Ï
	writei2c_byte(&hi2c2,RTC_WR,reg_timer_ctr,0x7F);

	writei2c_byte(&hi2c2,RTC_WR,reg_timer,0x01); 

	rtc_current.year = 0x56;
	rtc_current.month = 0x05;
	rtc_current.day = 0x16;
	rtc_current.hour = 0x08;
	rtc_current.min = 0x13;
	rtc_current.sec=0x20;
}


HAL_StatusTypeDef RTC8564_Set(RTCTIME time)
{
	uint8_t getdata=0;
	readi2c_nbyte(&hi2c2,RTC_WR,reg_sec,&getdata,1);
	if((getdata&0x80)==0x80)
	{
		;//SEGGER_RTT_printf(0,"getdata&0x80 = %d\n",(getdata&0x80));
		
	}
	readi2c_nbyte(&hi2c2,RTC_WR,reg_ctr1,&getdata,1);
	getdata = getdata | 0x20;//0xDF;
	writei2c_byte(&hi2c2,RTC_WR,reg_ctr1,getdata);
	//²âÊÔ
	readi2c_nbyte(&hi2c2,RTC_WR,reg_ctr1,&getdata,1);
	printf("\r\n   Set  STOP BIT    MIN=%02X  ",getdata);
	
	//¿ªÊ¼Ö´ÐÐÊ±¼äÅäÖÃ
	if(time.sec<=0x59)
	{
		writei2c_byte(&hi2c2,RTC_WR,reg_sec,time.sec);	
		//²âÊÔ
	readi2c_nbyte(&hi2c2,RTC_WR,reg_ctr1,&getdata,1);
		printf("\r\n Sec=%02X  ",getdata);
	}
	else
	{
		goto set_paramer_error;
	}
	if(time.min<=0x59)
	{
		writei2c_byte(&hi2c2,RTC_WR,reg_min,time.min);	
		//²âÊÔ
	readi2c_nbyte(&hi2c2,RTC_WR,reg_min,&getdata,1);
		printf(" MIN=%02X  ",getdata);
	}
	else
	{
		goto set_paramer_error;
	}
	if(time.hour<=0x23)
	{
		writei2c_byte(&hi2c2,RTC_WR,reg_hour,time.hour);	
		//²âÊÔ
	readi2c_nbyte(&hi2c2,RTC_WR,reg_hour,&getdata,1);
		printf(" Hour=%02X  ",getdata);
	}
	else
	{
		goto set_paramer_error;
	}
	if(time.year<=0x99)
	{
		writei2c_byte(&hi2c2,RTC_WR,reg_year,time.year);
		//²âÊÔ
	readi2c_nbyte(&hi2c2,RTC_WR,reg_year,&getdata,1);
		printf(" Year=%02X  ",getdata);
	}
	else
	{
		goto set_paramer_error;
	}
	if(time.month<=12)
	{
		writei2c_byte(&hi2c2,RTC_WR,reg_month,time.month);
		//²âÊÔ
	readi2c_nbyte(&hi2c2,RTC_WR,reg_month,&getdata,1);
		printf(" MONTH=%02X  ",getdata);
	}
	else
	{
		goto set_paramer_error;
	}
	if(time.day<=0x31)
	{
		writei2c_byte(&hi2c2,RTC_WR,reg_day,time.day);	
		//²âÊÔ
	readi2c_nbyte(&hi2c2,RTC_WR,reg_day,&getdata,1);
		printf(" Day=%02X  \r\n",getdata);
	}
	else
	{
		goto set_paramer_error;
	}
	//SEGGER_RTT_printf(0,"RTC SET\n");
	readi2c_nbyte(&hi2c2,RTC_WR,reg_ctr1,&getdata,1);
	printf("\r\n Read  old  REG_CTRL1=%02X  ",getdata);
	getdata = 0x00;
	writei2c_byte(&hi2c2,RTC_WR,reg_ctr1,getdata);
	//²âÊÔ
	readi2c_nbyte(&hi2c2,RTC_WR,reg_ctr1,&getdata,1); 
	printf("\r\n Read  NEW  REG_CTRL1=%02X  ",getdata);
	return HAL_OK;
set_paramer_error:
		//SEGGER_RTT_printf(0,"RTC_SET_ERROR\n");	
		return HAL_ERROR;
	
}

void RTC_Demo_init(void)
{
             rtc_current.year=8;
			rtc_current.month=4;
			rtc_current.day =11;
			rtc_current.hour =11;
			rtc_current.min =25;
			rtc_current.sec =31;
			RTC8564_Set(rtc_current);


}

void RTC8564_Get(void)
{

	 memset(rtc_tmp,0xEE,8);  
	 readi2c_nbyte(&hi2c2,RTC_WR,reg_year,&rtc_tmp[0],1);
	 readi2c_nbyte(&hi2c2,RTC_WR,reg_month,&rtc_tmp[1],1);
	 readi2c_nbyte(&hi2c2,RTC_WR,reg_day,&rtc_tmp[2],1);
	 readi2c_nbyte(&hi2c2,RTC_WR,reg_hour,&rtc_tmp[3],1);
	 readi2c_nbyte(&hi2c2,RTC_WR,reg_min,&rtc_tmp[4],1);
	 readi2c_nbyte(&hi2c2,RTC_WR,reg_sec,&rtc_tmp[5],1);
	 readi2c_nbyte(&hi2c2,RTC_WR,reg_week,&rtc_tmp[6],1); 
	
	 if(rtc_tmp[5]&0x80)
		;// printf("\r\n	RTC µçÑ¹µÍ \r\n");
	 else
	 {
	  Reg_common[0]=rtc_tmp[0];  //  get useful bits  year
	  Reg_common[1]=rtc_tmp[1]&0x1F;  // month	 
	  Reg_common[2]=rtc_tmp[2]&0x3F; // day
	  Reg_common[3]=rtc_tmp[3]&0x3F;  //  get useful bits	 hour
	  Reg_common[4]=rtc_tmp[4]&0x7F; // min 
	  Reg_common[5]=rtc_tmp[5]&0x7F; // sec
	
	  //---sec check ---
	  if((bcd2bin(Reg_common[5])>59)||(bcd2bin(Reg_common[4])>59)||(bcd2bin(Reg_common[3])>23)|| \
		 (bcd2bin(Reg_common[2])>31)||(bcd2bin(Reg_common[1])>12)||(bcd2bin(Reg_common[0])>99))
	  {    
		//  printf("\r\n »ñÈ¡µ½µÄRTC ²»ºÏ·¨ ÐèÒªÖØÐÂÉèÖÃ %02X-%02X-%02X %02X:%02X:%02X\r\n",Reg_common[0],Reg_common[1],Reg_common[2],Reg_common[3],Reg_common[4],Reg_common[5]);
		  MX_I2C2_Init(); 
		  RTC8564_Init();
	  }
	  else	  
	  { 
		memset(rtc_current.BCD_6_Bytes,0,6);
		memcpy(rtc_current.BCD_6_Bytes,Reg_common,6);
		
		//printf("\r\n RTC One Bye one   %02X-%02X-%02X %02X:%02X:%02X\r\n",rtc_current.BCD_6_Bytes[0],rtc_current.BCD_6_Bytes[1],rtc_current.BCD_6_Bytes[2],rtc_current.BCD_6_Bytes[3],rtc_current.BCD_6_Bytes[4],rtc_current.BCD_6_Bytes[5]);
	  }  
	 }

}


// part 3  ===========> Out  EEPROM    2  chips     SPI1

static void spi1_error(void)
{
	HAL_SPI_MspDeInit(&hspi1);
	MX_SPI1_Init();
}


static void spi1_write(uint8_t value)
{
  HAL_StatusTypeDef status = HAL_OK;

 status = HAL_SPI_Transmit(&hspi1, (uint8_t*) &value,1, SPIXTIMEOUT);
    
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    spi1_error();
  }
}
void eeprom_wren(uint8_t chip)
{

	if(chip ==1)
	{
		eeprom_cs1_low();
		spi1_write(WREN);
		eeprom_cs1_high();
	}
	if(chip ==2)
	{
		eeprom_cs2_low();
		spi1_write(WREN);
		eeprom_cs2_high();
	}
}
void eeprom_wrdisable(uint8_t chip)
{
	if(chip ==1)
	{
		eeprom_cs1_low();
	}
	else
	{
		eeprom_cs2_low();
	}
	spi1_write(WRDI);

		if(chip ==1)
	{
		eeprom_cs1_high();
	}
	else
	{
		eeprom_cs2_high();
	}
}

void eeprom_writemultipledata(uint32_t addr ,uint8_t *pdata, uint32_t size)
{
   uint32_t operate_addr=0;
   static uint8_t operate_addr_REG[3];
   uint8_t chip=1;

   // calirify
    if(addr>=0x40000)
    	{
           operate_addr=addr-0x40000; 
		   chip=2;
    	}
    else
		 operate_addr=addr;

    
    operate_addr_REG[0]=(u8)(operate_addr>>16);	// MSB	 
    operate_addr_REG[1]=(u8)(operate_addr>>8);
	operate_addr_REG[2]=(u8)(operate_addr);
   // excute below
	
	eeprom_waitforwriteend(chip);
	eeprom_wren(chip);
	if(chip ==1)
	{
		eeprom_cs1_low();		
	}
	else
	{
		eeprom_cs2_low();		
	}
	spi1_write(WRITE);
	HAL_SPI_Transmit_IT(&hspi1, (uint8_t*)operate_addr_REG,3);//, SPIXTIMEOUT); 
	HAL_SPI_Transmit_IT(&hspi1, pdata,size);//,SPIXTIMEOUT);

	/*
	while(size--)
	{
		spi1_write(*pdata);
		pdata++;
	}
	*/
	
	if(chip ==1)
	{
		eeprom_cs1_high();
	}
	else
	{
		eeprom_cs2_high();
	}

	
	//eeprom_wrdisable(chip); // add later
}

uint8_t eeprom_read_reg(uint8_t chip)
{
	
	uint8_t txtemp = RDSR;
	uint8_t rxtemp = 0;
	if(chip ==1)
	{
		eeprom_cs1_low();
	}
	else
	{
		eeprom_cs2_low();
	}
	//spi1_write(txtemp);
	//temp = spi1_receive_byte(0xff);
	HAL_SPI_Transmit(&hspi1,&txtemp,1,SPIXTIMEOUT);
	HAL_SPI_Receive(&hspi1,&rxtemp,1,SPIXTIMEOUT);
	if(chip ==1)
	{
		eeprom_cs1_high();
	}
	else
	{
		eeprom_cs2_high();
	}
	return rxtemp;
}

uint8_t eeprom_RegisterByte_Read(uint8_t chip,uint8_t reg_Name)
{
	
	uint8_t txtemp = reg_Name;
	uint8_t rxtemp = 0;
	if(chip ==1)
	{
		eeprom_cs1_low();
	}
	else
	{
		eeprom_cs2_low();
	}
	//spi1_write(txtemp);
	//temp = spi1_receive_byte(0xff);
	HAL_SPI_Transmit(&hspi1,&txtemp,1,SPIXTIMEOUT);
	HAL_SPI_Receive(&hspi1,&rxtemp,1,SPIXTIMEOUT);
	if(chip ==1)
	{
		eeprom_cs1_high();
	}
	else
	{
		eeprom_cs2_high();
	}
	return rxtemp;
}


void eeprom_RegisterByte_WR(uint8_t chip,uint8_t reg_Name,uint8_t reg_value)
{
	
	uint8_t txtemp = reg_Name;
	uint8_t rxtemp = reg_value;
	if(chip ==1)
	{
		eeprom_cs1_low();
	}
	else
	{
		eeprom_cs2_low();
	}
	//spi1_write(txtemp);
	//temp = spi1_receive_byte(0xff);
	HAL_SPI_Transmit(&hspi1,&txtemp,1,SPIXTIMEOUT);
	HAL_SPI_Transmit(&hspi1,&rxtemp,1,SPIXTIMEOUT);  
	if(chip ==1)
	{
		eeprom_cs1_high();
	}
	else
	{
		eeprom_cs2_high();
	}
}


void eeprom_multipleread(uint32_t addr,uint8_t* pbuffer, uint32_t num)
{
   uint32_t  operate_addr=0;
   static uint8_t operate_addr_REG[3];
   uint8_t chip=1;

   // calirify
    if(addr>=0x40000)
    	{
           operate_addr=addr-0x40000; 
		   chip=2;		   
    	}
    else
		 operate_addr=addr;

    operate_addr_REG[0]=(u8)(operate_addr>>16);  // MSB   
	operate_addr_REG[1]=(u8)(operate_addr>>8);
	operate_addr_REG[2]=(u8)(operate_addr);

   
    eeprom_wrdisable(chip); 

   // excute below
	eeprom_waitforwriteend(chip);
  	if(chip ==1)
	{
		eeprom_cs1_low();
	}
	else
	{
		eeprom_cs2_low();
	}	
    spi1_write(READ);
	HAL_SPI_Transmit_IT(&hspi1, (uint8_t*)operate_addr_REG,3);//,SPIXTIMEOUT);  
	HAL_SPI_Receive_IT(&hspi1, pbuffer,num);//,SPIXTIMEOUT);
    if(chip ==1)
	{
		eeprom_cs1_high();
	}
	else
	{
		eeprom_cs2_high();
	}
}

HAL_StatusTypeDef eeprom_waitforwriteend(uint8_t chip)
{
	uint8_t wipflag=0xff;
  do
  {
  	wipflag=eeprom_read_reg(chip);
  } while((wipflag & WIP_BIT) == SET);
  return HAL_OK;
}
HAL_StatusTypeDef eeprom_ease_all(void)
{
#if 0
	uint8_t tmp[256] ={0};
	uint32_t i =0;
	memset(tmp,0xff,256);
	delay_t = 1000;
	/*
	for(i=0 ; i<1024 ;i++)
	{
		eeprom_writemultipledata(i*256 ,tmp, 256);
	}
	*/
	write_sensor_data(tmp,256);
	eeprom_multipleread(0,tmp,256);
	for(i=0;i<256;i++)
	{
		//SEGGER_RTT_printf(0,"%x ",tmp[i]);
	}
	/*
	for(i=0 ; i<1024 ;i++)
	{
		eeprom_writemultipledata(i*256 ,tmp, 256);
	}
	*/
#endif
 	return HAL_OK;
}











//part 4:    ADX375 accelerate   main       L3D20H  compass   auxiliary    SPI2

  // Common SPI2 Driver





static void spi2_write(uint8_t value)
{
  HAL_StatusTypeDef status = HAL_OK;

 status = HAL_SPI_Transmit(&hspi2, (uint8_t*) &value,1, SPIXTIMEOUT);
    
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
     printf("\r\n spi2_error");// spi2_error();
  }
}

  void ADXL375_SPI2_WriteNbytes(uint8_t Register_CMD,uint8_t* InStr,uint16_t  wr_len)
  	{
        // chip  	 CS low
		ADXL375_CS_LOW();
	    spi2_write(Register_CMD|0x40);
	    HAL_SPI_Transmit(&hspi2, InStr,wr_len, SPIXTIMEOUT);

   	   // chip  CS_HIGH
		ADXL375_CS_HIGH();
  	}
  
    void ADXL375_Wr_OneByte(uint8_t Register_CMD,uint8_t value)
  	{
        // chip  	 CS low
		ADXL375_CS_LOW();
	    spi2_write(Register_CMD);
	    HAL_SPI_Transmit(&hspi2, (uint8_t*) &value,1,SPIXTIMEOUT);
   	   // chip  CS_HIGH
		ADXL375_CS_HIGH();
  	}
    
	 uint8_t ADXL375_SPI2_ReadNbytes(uint8_t Register_CMD,uint8_t* OutStr,uint16_t  Rd_len)
  	{
        // chip  	 CS low
		ADXL375_CS_LOW();
	    spi2_write(Register_CMD|0xC0);//|L3G_ADDR_RW|L3G_ADDR_MS 
	    HAL_SPI_Receive(&hspi2, OutStr,Rd_len, SPIXTIMEOUT); 
	    // chip  CS_HIGH
		ADXL375_CS_HIGH(); 
        return HAL_OK;

  	}

	uint8_t ADXL375_SPI2_Rd_Onebytes(uint8_t Register_CMD,uint8_t *Data)
  	{
        // chip  	 CS low
		ADXL375_CS_LOW();
	    spi2_write(Register_CMD|0x80); 
	    HAL_SPI_Receive(&hspi2, Data,1, SPIXTIMEOUT);   // default 2
	    // chip  CS_HIGH
		ADXL375_CS_HIGH();
        return HAL_OK;

  	}

	u8  ADXL375_WhoAmI_check(void)
	{
	   uint8_t  Reg_ID=0;
	   
        ADXL375_SPI2_Rd_Onebytes(XL375_DEVID,(uint8_t *)&Reg_ID);
		if(Reg_ID==0xE5)
			return 1;
		else 
			return 0;

	}

	u8   ADXL375_INT1_Judge(void)
	{   
	    u8  value_rtn=0;     //   0: default    1:  shock     2 :  Act    3:  inact   
	    u8   Reg_30=0;   //   INT  source     Note:   state change
		                 //      B7                   B6                            B5                 B4       B3          B2       B1               B0             
		                 //    Data_ready      single shocke     doubule shock       ACT    INACT    NULL   WaterMark   OverRun


	    u8   Reg_2B=0;   //   Shock Status    Note : state  keep
						 //      B7                   B6                            B5                B4               B3                 B2                B1               B0       
                         //     0                   ACT_X                   ACT_Y           ACT_Z         asleep          SHOCK_X     SHOCK_Y      SHOCK_Z
       MX_SPI2_Init_ADXL375();  
	   ADXL375_SPI2_ReadNbytes(0x2B,(uint8_t *)&Reg_2B,1); 
	   //  printf("     Reg 0x%02x==>0x%02X\r\n",0x2B,Reg_2B); 	 
       ADXL375_SPI2_ReadNbytes(0x30,(uint8_t *)&Reg_30,1); 	 
	   //  printf("\r\nOutput   ADX375  Reg 0x%02x==>0x%02X  ",0x30,Reg_30);  

        if(0x60&Reg_30)   //  single &  double shocke
        {
		        /*
		           if(Reg_2B&0x04)
				   	    printf("\r\n  X axis shock"); 
				   if(Reg_2B&0x02)
				   	    printf("\r\n  Y axis shock");
				   if(Reg_2B&0x01)
				   	    printf("\r\n  Z axis shock"); 
		   	    */
		   return 1 ;    // shock return
        } 
		if(0x10&Reg_30)   // ACT
        {
	           /*
	           if(Reg_2B&0x40)
			   	     printf("\r\n  X axis ACT");
			   if(Reg_2B&0x20)
			   	     printf("\r\n  Y axis ACT");
			   if(Reg_2B&0x10)
			   	     printf("\r\n  Z axis ACT");   
		   	     */
		   return 2 ;    // shock return
        } 
        if(0x08&Reg_30)   //  INACT
        {
		   return 3 ;    // shock return
        }    
	     

	}
	


	void  ADXL375_INT1_IRQHandler(void)
	{
      if(1==ADXL375_INT1_Judge())
       {

			   if(SHK_WRK_STATE.ENABLE_Triger==0)  // Ö®Ç°Ã»ÓÐ¹ý´¥·¢
			   {
			      LED2_ON;
			      SHK_SampleData_TrigEnable();  
			   }
			   else
			   if( SHK_WRK_STATE.ENABLE_Triger==1)
				    SHK_WRK_STATE.Enable_Again=1;    //   ¼ÇÂ¼Ò»ÏÂ ´¥·¢ÔÙ´Î·¢Éú¹ý
	   }	
	}

  //   ADX375    SPI2 

	void ADXL375_init(void)    //¼ÓËÙ¶È´«¸ÐÆ÷
	{
		uint8_t  Reg_ID=0;
		u8 i=0;
  
	
    MX_SPI2_Init_ADXL375();  //It's very immportant

    
	if(ADXL375_WhoAmI_check())
	   printf("\r\n    ADXL375	first  Who am  I  ok");	
	
	   //¶ÁÈ¡Ó²¼þID		
	   ADXL375_SPI2_Rd_Onebytes(XL375_DEVID,(uint8_t *)&Reg_ID);
	   printf("\r\nOutput	ADX375 ID 0x%0004X\r\n",Reg_ID); 
  
	   ADXL375_Wr_OneByte(0x31,0X2B); //4ÏßÖÆSPI½Ó¿Ú,µÍµçÆ½ÖÐ¶ÏÊä³ö,LSB  mode  

	   ADXL375_SPI2_Rd_Onebytes(0x31,(uint8_t *)&Reg_ID);
	   printf("\r\nOutput	ADX375	Reg 0x%02x==>0x%02X\r\n",0x31,Reg_ID);  

#if 1
	 //ÅäÖÃÇÃ»÷Á¦¶È Threshod		 ADXL375	  780mg/LSB <=>  200/256  
	 ADXL375_Wr_OneByte(0x1D,ADXL375_INIT_THRESHOD);   //	´Ë´¦ÐÞ¸ÄÕð¶¯

	 
	 ADXL375_Wr_OneByte(0x1E,ADXL375_INIT_X_OFFSET); //X Æ«ÒÆÁ¿ ¸ù¾Ý²âÊÔ´«¸ÐÆ÷µÄ×´Ì¬Ð´Èëpdf29Ò³ 0X00 (196mg/LSB)
	 ADXL375_Wr_OneByte(0x1F,ADXL375_INIT_Y_OFFSET); //Y Æ«ÒÆÁ¿ ¸ù¾Ý²âÊÔ´«¸ÐÆ÷µÄ×´Ì¬Ð´Èëpdf29Ò³ 0X00 (196mg/LSB)
	 ADXL375_Wr_OneByte(0x20,ADXL375_INIT_Z_OFFSET); //Z Æ«ÒÆÁ¿ ¸ù¾Ý²âÊÔ´«¸ÐÆ÷µÄ×´Ì¬Ð´Èëpdf29Ò³ 0X00 (196mg/LSB)
#if 1
	 ADXL375_Wr_OneByte(0x21,ADXL375_INIT_DUR);  //ÇÃ»÷ÑÓÊ±0:½ûÓÃ; (625us/LSB)
	 ADXL375_Wr_OneByte(0x22,0x00);  //¼ì²âµÚÒ»´ÎÇÃ»÷ºóµÄÑÓÊ±0:½ûÓÃ; (1.25ms/LSB)  ½ûÖ¹2´Î³å»÷
	 ADXL375_Wr_OneByte(0x23,0x00);  //ÇÃ»÷´°¿Ú0:½ûÓÃ; (1.25ms/LSB)  ½ûÖ¹2 ´Î³å»÷
	
	 ADXL375_Wr_OneByte(0x24,THRESH_ACT_Defaut);  //±£´æ¼ì²â»î¶¯·§Öµ; (780mg/LSB)	 01 
	 ADXL375_Wr_OneByte(0x25,THRESH_INACT_Defaut);  //±£´æ¼ì²â¾²Ö¹·§Öµ; (780mg/LSB)
	 ADXL375_Wr_OneByte(0x26,TIME_INACT_Defaut);  //¼ì²â»î¶¯Ê±¼ä·§Öµ; (1s/LSB) 16s
	 ADXL375_Wr_OneByte(0x27,0x77);  //  Ê¹ÄÜ  ACT  INACT   DC  all enable     ¾²Ö¹Ê±¼ä  // Disable  ACT and INACT  Leave only  shock INT
                                     // 0x77  Enable  ACT  and INACT 
 
	 
	 ADXL375_Wr_OneByte(0x2A,0x07);  // x y  z axes Enalbe  07
	 //ADXL375_read_byte(0x2B);    //Ö»¶Á¼Ä´æÆ÷,×´Ì¬¶ÁÈ¡       ¼ì²âÐÂÊý¾ÝµÄ×´Ì¬  ÐÂÊý¾Ý¿ÉÓÃÊ±1  µ«»á±»¸²¸Ç
	 ADXL375_SPI2_ReadNbytes(0x2B,(uint8_t *)&Reg_ID,1); 	 
	 printf("\r\nOutput   ADX375  Reg 0x%02x==>0x%02X\r\n",0x2B,Reg_ID);  
#endif	 
	 ADXL375_Wr_OneByte(0x2C,ADXL375_INIT_FREQ); // 	Õý³£Ä£Ê½ ²ÉÑùÆµÂÊ 1600HZ   E
	 ADXL375_Wr_OneByte(0x2D,0x08); // 28 Ñ¡ÔñµçÔ´Ä£Ê½¹Ø±Õ×Ô¶¯ÐÝÃß,ÐÝÃß,»½ÐÑ¹¦ÄÜ²Î¿¼pdf24Ò³
	 ADXL375_Wr_OneByte(0x2E,0x60);  //Ê¹ÄÜ DATA_READY ÖÐ¶Ï  single shock  double shock 
	 ADXL375_Wr_OneByte(0x2F,0x00);   //  all	0	 Ê¹ÄÜ ÖÐ¶Ïµ½INT1 Òý½Å 
	 //ADXL375_read_byte(0x30);    0x30,×´Ì¬¶ÁÈ¡
	 ADXL375_SPI2_ReadNbytes(0x30,(uint8_t *)&Reg_ID,1); 
	 printf("\r\nOutput   ADX375  Reg 0x%02x==>0x%02X\r\n",0x30,Reg_ID);  
	 
   //  ADXL375_Wr_OneByte(0x31,0X2B); //4ÏßÖÆSPI½Ó¿Ú,µÍµçÆ½ÖÐ¶ÏÊä³ö,LSB  mode
	 ADXL375_Wr_OneByte(0x38,0x9f);  //FIFOÄ£Ê½Éè¶¨,StreamÄ£Ê½£¬´¥·¢Á¬½ÓINT1,31¼¶Ñù±¾»º³å
	//ADXL375_read_byte(0x39);	  //Ö»¶Á¼Ä´æÆ÷,×´Ì¬¶ÁÈ¡ 

	ADXL375_SPI2_ReadNbytes(0x39,(uint8_t *)&Reg_ID,1); 	
	printf("\r\nOutput	 ADX375  Reg 0x%02x==>0x%02X\r\n",0x39,Reg_ID); // D7  ±íÊ¾ÊÇ·ñ·¢Éú¹ýÖÐ¶Ï
	                                                                 //  D5-D0  ±íÊ¾·¢Éú¹ýµÄÊýÄ¿
	
	HAL_Delay(3);
	ADXL375_SPI2_ReadNbytes(XL375_DEVID,(uint8_t *)&Reg_ID,1);	
	printf("\r\nOutput ADX375 ID 0x%02X\r\n",Reg_ID);   


	
	ADXL375_SPI2_ReadNbytes(0x1D,(uint8_t *)&Reg_ID,1); 	 
	printf("\r\nOutput	Threshod  ADX375  Reg 0x%02x==>0x%02X\r\n",0x1D,Reg_ID); // D7  ±íÊ¾ÊÇ·ñ·¢Éú¹ýÖÐ¶Ï
	                                                                 //  D5-D0  ±íÊ¾·¢Éú¹ýµÄÊýÄ¿
	

#endif
	}
  
  void calc_init(int32_t *pstr,int8_t *pout)
  {
	  //ÒòÎªÈ«·Ö±æÂÊÒ²Ö»ÓÐ13Î»ËùÒÔÊý¾ÝÖ»ÓÐ13Î»ÓÐÐ§
	  if((((*pstr)&0x1000)>>12)==1)
	  {
		  *pout = -((~(*pstr & 0xFFF))+1);
	  }
	  else
	  {
		  *pout = *pstr & 0x1FFF;
	  }
  }

  u16   Compansate_change(u16 invalue)
  {   
  	 u16  value=0;
	 if(invalue&0x8000)  
	 {
         value=0xFFFF-invalue+1; 
		 // printf("-");
	 }
	 else
	 {
	   value=invalue;	   
	  // printf("+");
	 }  
	 return value;
  }

  void ADXL375_Get(void) 
  	{
  	  u8 i=0,j=0;;
	  u8 reg_get[6];
	  u8  Reg_ID=0;
	  u16  Aux_Reg[6];   
	  
	  MX_SPI2_Init_ADXL375();  //It's very immportant
       /*     LSB =0   ÎªÓÒ¶ÔÆë£¬ÎÒÃÇµ±Ç°²ÉÓÃµÄ¾ÍÊÇÓÒ¶ÔÆë£¬ºÍ¸ßÆµÂÊ²ÉÑùÊÇÒ»ÑùµÄ
              Ê¹ÓÃ3200 Hz»ò1600 HzÊä³öÊý¾ÝËÙÂÊÊ±£¬Êä³öÊý¾Ý×ÖµÄ
		LSBÊ¼ÖÕÎª0¡£Êý¾ÝÓÒ¶ÔÆëÊ±£¬ LSB¶ÔÓ¦ÓÚDATAx0¼Ä´æÆ÷
		µÄÎ»D0£»Êý¾Ý×ó¶ÔÆëÊ±£¬ LSB¶ÔÓ¦DATAx0¼Ä´æÆ÷µÄ
		Î»D3
	    */
	//  if(ADXL375_WhoAmI_check())
	//        printf("\r\n    ADXL375	 Who am  I  Check Pass");	 
	   ADXL375_SPI2_ReadNbytes(0x30,(uint8_t *)&Reg_ID,1); 	 
	//  printf("\r\nOutput   ADX375  Reg 0x%02x==>0x%02X  ",0x30,Reg_ID);  
	   ADXL375_SPI2_ReadNbytes(0x2B,(uint8_t *)&Reg_ID,1); 	 
	//  printf("     Reg 0x%02x==>0x%02X\r\n",0x2B,Reg_ID); 	  
	  ADXL375_SPI2_ReadNbytes(0x32,reg_get,6);	

#if 0	
       // keep  for   debug
      j=0;
      for(i=0;i<6;i++)
      	{
           if(reg_get[i]==0x00)
           	{
               j++;
           	}
      	}

  if(j!=6)
  { 
      printf("\r\n  main_counter=%d ADXL375  RAW:",main_counter);	  
	  for(i=0;i<6;i++)
	  {
        printf(" %02X",  reg_get[i]);
	  }  
	  memcpy(Project_SHK.ACCEL_ADX375_ACCEL_6BYTES,reg_get,6);
	  

	 printf ("          X=0x%02X%02X  Y=0x%02X%02X    Z=0x%02X%02X ",Project_SHK.ACCEL_ADX375_ACCEL_6BYTES[1],\
	 Project_SHK.ACCEL_ADX375_ACCEL_6BYTES[0],Project_SHK.ACCEL_ADX375_ACCEL_6BYTES[3],Project_SHK.ACCEL_ADX375_ACCEL_6BYTES[2],\
	 Project_SHK.ACCEL_ADX375_ACCEL_6BYTES[5],Project_SHK.ACCEL_ADX375_ACCEL_6BYTES[4]);

	 printf ("     Final      X=0x%0004X  Y=0x%0004X    Z=0x%0004X   shaketimes=%d\r\n",Compansate_change((Project_SHK.ACCEL_ADX375_ACCEL_6BYTES[1]<<8)+Project_SHK.ACCEL_ADX375_ACCEL_6BYTES[0]),\
	 Compansate_change((Project_SHK.ACCEL_ADX375_ACCEL_6BYTES[3]<<8)+Project_SHK.ACCEL_ADX375_ACCEL_6BYTES[2]),\
	 Compansate_change((Project_SHK.ACCEL_ADX375_ACCEL_6BYTES[5]<<8)+Project_SHK.ACCEL_ADX375_ACCEL_6BYTES[4]),Shake_times);

  }
   #endif
 }


  
  
  void ADXL375_multRead(ADXL375_TYPE * ptResult)
  {
	  uint8_t tmp[6];
	  //Ê®ÈýÎ»È«ÓÐÐ§Î» Ôò»¹Ó¦¸Ã&ÉÏ0x1fff;
	  if(HAL_OK==  ADXL375_SPI2_ReadNbytes(XL375_DATAX0,tmp,6))
	  { 
  
		  ptResult->ax = ((tmp[1]<<8)+tmp[0]);
		  ptResult->ay = ((tmp[3]<<8)+tmp[2]);
		  ptResult->az = ((tmp[5]<<8)+tmp[4]);

		  Output_Data(tmp,6);
    	#if 0
		  ptResult->ax		= *( (int16_t *)(&tmp[0]) );   
		  ptResult->ay		= *( (int16_t *)(&tmp[2]) );
		  ptResult->az		= *( (int16_t *)(&tmp[4]) );
		#endif
	  }
  }
  
  
  //  L3D20H    SPI2  
  /*
          Note :  ÓÉÓÚPCB   Æ÷¼þ·½ÏòÃ»Í³Ò» £¬    L3GD20H µÄÊý¾ÝÐèÒª×ª»»×ª»»¹æÔòÈçÏÂå£º

          X=-X
          Y=-Y
          Z=Z 
  */
  
  
  void L3GD20H_SPI2_WriteNbytes(uint8_t Register_CMD,uint8_t* InStr,uint16_t  wr_len)
  {
		// chip    CS low
	  L3GD20H_CS_LOW();
		 
	  spi2_write(Register_CMD|L3G_ADDR_MS);
	  HAL_SPI_Transmit(&hspi2, InStr,wr_len, SPIXTIMEOUT);
  
	 // chip  CS_HIGH
	  L3GD20H_CS_HIGH();
  
  
  }
  void L3GD20H_SPI2_Wr_1_bytes(uint8_t Register_CMD,uint8_t value)
	{
		// chip 	 CS low
		L3GD20H_CS_LOW();		   
		spi2_write(Register_CMD);
		HAL_SPI_Transmit(&hspi2, (uint8_t*) &value,1, SPIXTIMEOUT);  
  
	   // chip	CS_HIGH
		L3GD20H_CS_HIGH();
	}  
  
  
  void L3GD20H_SPI2_ReadNbytes(uint8_t Register_CMD,uint8_t* OutStr,uint16_t  Rd_len)
  {
		// chip    CS low
		L3GD20H_CS_LOW();
		
		spi2_write(Register_CMD|L3G_ADDR_RW|L3G_ADDR_MS);
		HAL_SPI_Receive(&hspi2, OutStr,Rd_len, SPIXTIMEOUT*3);
  
		// chip  CS_HIGH
		L3GD20H_CS_HIGH(); 
  
  }
  
	 void L3GD20H_SPI2_OneNbytes(uint8_t Register_CMD,uint8_t Data)
  {
		// chip    CS low
		L3GD20H_CS_LOW();
		
		spi2_write(Register_CMD|L3G_ADDR_RW);
		HAL_SPI_Receive(&hspi2, &Data,1, SPIXTIMEOUT*3); 
  
		// chip  CS_HIGH
		L3GD20H_CS_HIGH();   
  }
  
u8  L3GD20H_WhoAmI_check(void)
{
	   uint8_t  Reg_ID=0;
	   
       L3GD20H_SPI2_ReadNbytes(WHO_AM_I,&Reg_ID,1); 
	   printf("\r\n WHOAMI=%02X    \r\n",Reg_ID);
	   if(Reg_ID==0xD7)
			return 1;
	   else 
			return 0;
}
  
  
void Init_L3GD20H(void)     //  motion sensor   :    Î»ÒÆ´«¸ÐÆ÷
{  // Create  by Nathan

   uint8_t  Reg_who_am_I=0;
   u8  i=0;

    MX_SPI2_Init_L3GD20H();  //  It's  Very important 
    
   // L3GD20H_CS_LOW();  // Set High
   // L3GD20H_CS_HIGH();  // Set High
	//delay_ms_1(1);

   if(L3GD20H_WhoAmI_check())
        printf("\r\n    L3GD20H  first  Who am  I  ok");


     
    //Who am I 
    L3GD20H_SPI2_ReadNbytes(WHO_AM_I,&Reg_who_am_I,1); 
	
	printf("\r\nL3GD20H  WHO_AM_I: 0x%02X\r\n",Reg_who_am_I);

	

   #if 0
  	for(i=0;i<200;i++)
	{
	   HAL_Delay(50);
       L3GD20H_SPI2_ReadNbytes(WHO_AM_I,&Reg_who_am_I,1); 
	
	   printf("\r\nL3GD20H  WHO_AM_I counter=%d: 0x%02X\r\n",i,Reg_who_am_I);

	}
	#endif

    // Init  Register  one by one
     L3GD20H_SPI2_Wr_1_bytes(0x39,0x10);// SPI only   Normal  mode   
    L3GD20H_SPI2_Wr_1_bytes(CTRL_REG1,0x5F);  //   200 HZ   normal mode   xyz axis all enable  ,details see table 21 cutoff  -

	L3GD20H_SPI2_ReadNbytes(CTRL_REG1,&Reg_who_am_I,1); 	
	printf("\r\nL3GD20H  CTRL_REG1 SET= 5F: 0x%02X\r\n",Reg_who_am_I); 
	
	L3GD20H_SPI2_Wr_1_bytes(CTRL_REG2,0x20); // Normal mode     E0 (enable interrupt)
	
	L3GD20H_SPI2_Wr_1_bytes(CTRL_REG3,0x80); //  Interrupt enable on INT1
	L3GD20H_SPI2_Wr_1_bytes(CTRL_REG4,0x20); //   continuse update; LSB  ;2000dps  ; level sensitive disabled;4 wires-SPI
	L3GD20H_SPI2_Wr_1_bytes(CTRL_REG5,0x03); // Normal mode ; INT1 out interrupt generator
	
    L3GD20H_SPI2_Wr_1_bytes(FIFO_CTRL_REG,0x00);  // bypass  mode  
	L3GD20H_SPI2_Wr_1_bytes(0x30,0x3F);  // Z Y X  interrupt enable 

	

    //  0x27 ,

    L3GD20H_SPI2_ReadNbytes(FIFO_CTRL_REG,&Reg_who_am_I,1); 	
	printf("\r\nL3GD20H  FIFO_CTRL_REG= 0x%02X\r\n",Reg_who_am_I);  

	L3GD20H_SPI2_ReadNbytes(0x26,&Reg_who_am_I,1); 	
	printf("\r\nL3GD20H  OUTPUT_TEMP= 0x%02X\r\n",Reg_who_am_I); 

	L3GD20H_SPI2_ReadNbytes(0x31,&Reg_who_am_I,1); 	
	printf("\r\nL3GD20H  IG_SRC= 0x%02X\r\n",Reg_who_am_I); 

	
    	L3GD20H_SPI2_ReadNbytes(CTRL_REG2,&Reg_who_am_I,1); 	
	printf("\r\nL3GD20H  CTRL_REG1 SET= 20: 0x%02X\r\n",Reg_who_am_I); 

	 L3GD20H_SPI2_ReadNbytes(WHO_AM_I,&Reg_who_am_I,1); 
	
	printf("\r\nL3GD20H  WHO_AM_I: 0x%02X\r\n",Reg_who_am_I);

   // L3GD20H_SPI2_ReadNbytes(0x00,&Reg_who_am_I,1); 
	
	//printf("\r\nL3GD20H  reserved: 0x%02X\r\n",Reg_who_am_I); 
}
 

void L3GD20H_GET(void)
{
    //uint8_t  reg[8];
   // u8 i=0; 

	MX_SPI2_Init_L3GD20H();  //  It's  Very important 
	
    memset(L3GD_reg,0xF1,8);
	L3GD20H_SPI2_ReadNbytes(STATUS_REG,L3GD_reg,7);
	#if 0
	printf("\r\n 7 byte :");
	for(i=0;i<7;i++)
		{
            printf("%02X ",L3GD_reg[i]);
		}
	#endif
	 if((L3GD_reg[0]&0x0F)==0x0F)	 //  Judge  STATUS REG
	 {
	    memcpy(Project_SHK.GYRO_L3G_6BYTES,L3GD_reg+1,6); 
	  #if 0	
	    Project_SHK.GYRO_X_vaule=(u16)(L3GD_reg[2]<<8)+(u16)L3GD_reg[1]; 
		Project_SHK.GYRO_Y_vaule=(u16)(L3GD_reg[4]<<8)+(u16)L3GD_reg[3];
		Project_SHK.GYRO_Z_vaule=(u16)(L3GD_reg[6]<<8)+(u16)L3GD_reg[5];  
	    printf("\r\nL3GD20H  X: 0x%0004X  Y: 0x%0004X  Z: 0x%0004X\r\n",Project_SHK.GYRO_X_vaule,Project_SHK.GYRO_Y_vaule,Project_SHK.GYRO_Z_vaule);
      #endif
	 }
}

void  USART1_RX_enable(void)
{
   if(HAL_UART_Receive_IT(&hlpuart1,aRxBuffer,1)!=HAL_OK)
  	 printf("USART1_Rx_enable Fail! :( \r\n");

}

void get_chipid(void)
{
   u8 i=0,j=0;   
   u32	 Stm32_DeviceID[3];
   
    Stm32_DeviceID[0] =  *(__IO uint32_t *)(0X1FF80050);
	Stm32_DeviceID[1] =  *(__IO uint32_t *)(0X1FF80054);
	Stm32_DeviceID[2] =  *(__IO uint32_t *)(0X1FF80064);

	for(i=0;i<3;i++)
		{
          sys_config.Device_ID[j++]=(u8)(Stm32_DeviceID[i]>>24);
          sys_config.Device_ID[j++]=(u8)(Stm32_DeviceID[i]>>16);
		  sys_config.Device_ID[j++]=(u8)(Stm32_DeviceID[i]>>8);
		  sys_config.Device_ID[j++]=(u8)(Stm32_DeviceID[i]);
		}
	printf("\r\n  STM32_DEVICEID=%00000008X  %00000008X   %00000008X \r\n",Stm32_DeviceID[0],Stm32_DeviceID[1],Stm32_DeviceID[2]);
    
	printf("\r\n             STM32_DEVICEID=");
	for(i=0;i<12;i++)
		printf("%02X",sys_config.Device_ID[i]);
	printf("\r\n"); 
}


void Key_Check(void)
{
   // K1  check     
   if(HAL_GPIO_ReadPin(SHK_K1_GPIO_Port,SHK_K1_Pin) == 0)	// ÅÐ¶ÏÐ¾Æ¬µÄRDY  ¹Ü½Å
   	{
         K1_Value.keep_counter++;
		 if(K1_Value.keep_counter>2)
		 {  
		     if(K1_Value.work_state==0)
		      {
		         K1_Value.work_state=1;
				 LED2_ON;

				 SHK_SampleData_TrigEnable();
				 
		      }                    
		 }
   	}
   else
   	{
   	     K1_Value.keep_counter=0;
		 if(K1_Value.work_state!=1)
		    K1_Value.work_state=0;
		 //LED2_OFF;
   	}
}



void  SHK_SampleData_TrigEnable(void)
{  
    u8  reg[5];
     SHK_WRK_STATE.ENABLE_Triger=1;
      //  -----
	 SHK_WRK_STATE.Page_in_offset=0;  // clear
	 SHK_WRK_STATE.current_packet=0;
	 SHK_WRK_STATE.total_packet=800;  //800 ¸öµã  
}

void SHK_Event_SaveEnd(void)
{
 	  sys_config.SaveWr++; //    ´æ´¢Ê±¼äÀÛ¼Ó
 	  
	  if(sys_config.SaveWr>=SHK_EVENT_MAXNUM)
	  {
	    sys_config.SaveWr=0;
        sys_config.SaveFull_state=1;
	  }
      //Write  µ±Ç°´æ´¢ÌõÊý 3 ·Ý   ÊÇ·ñ´æ´¢Âú2 ·Ý
	  eep_reg[0]=sys_config.SaveWr;
	  eep_reg[1]=sys_config.SaveWr;
	  eep_reg[2]=sys_config.SaveWr;
	  eep_reg[3]=sys_config.SaveFull_state; 
	  eep_reg[4]=sys_config.SaveFull_state;
	  //delay_us_1(300);
      eeprom_writemultipledata( SHK_ADDR_SAVE_STATUS,eep_reg,5);  
	  SHK_WRK_STATE.Page_in_offset=0; 
	  SHK_WRK_STATE.ENABLE_Triger=0;  // clear
	   printf("\r\n    Event Num=%d     Full state=%d     save  over  !\r\n",sys_config.SaveWr,sys_config.SaveFull_state); 
	   K1_Value.work_state=0;
	   LED2_OFF;
}

void SHK_Check_EventHppen_Again(void)
{  //  ¼ì²âÅÐ¶ÏÔÚ²É¼¯µÄ¹ý³ÌÖÐÊÇ·ñÓÐ¹ý´¥·¢·¢Éú
     if(SHK_WRK_STATE.Enable_Again==1)
	  {   //   Ê±¼ä·¢Éú¹ý£¬ ¼´¿ÌÔÙ²É¼¯Ò»´ÎÊý¾Ý
	      printf("\r\n   ²É¼¯¹ý³ÌÖÐÓÖ·¢Éú¹ý ÊÂ¼þ´¥·¢! \r\n");
	      SHK_SampleData_TrigEnable(); 
          SHK_WRK_STATE.Enable_Again=0;
		  LED2_ON; 
	  }
}

void  SHK_SaveData_Checking(void)
{
   u8 eep_reg[6];
   u8 i=0,record_num=0;
   u32 Read_ADDR=0;

   eeprom_multipleread(SHK_ADDR_SAVE_STATUS,eep_reg, 5);   
   sys_config.SaveWr=eep_reg[0];
   sys_config.SaveFull_state=eep_reg[3];

  if(sys_config.SaveFull_state==0)
  {
     if(sys_config.SaveWr)
     {
	   for(i=0;i<sys_config.SaveWr;i++)
	   	{
           record_num++;
           Read_ADDR=SHK_ADDR_DATA_START+((u32)(sys_config.SaveWr-i-1)<<14); 
		   eeprom_multipleread(Read_ADDR,eep_reg, 6);  
		   printf("\r\n     Num=%d   ADDR=0x%0004X   Start time %02X-%02X-%02X %02X:%02X:%02X \r\n",record_num,Read_ADDR,eep_reg[0],eep_reg[1],eep_reg[2],eep_reg[3],eep_reg[4],eep_reg[5]);
	   	}
     }
  }
  else
  if(sys_config.SaveFull_state==1)   // ´æ´¢Âú¹ý
  {
    if(sys_config.SaveWr)
     { 
	    for(i=0;i< sys_config.SaveWr;i++)
	   	{
           record_num++;
		   Read_ADDR=SHK_ADDR_DATA_START+((u32)(sys_config.SaveWr-i-1)<<14); 
		   eeprom_multipleread(Read_ADDR,eep_reg, 6);  
		   printf("\r\n  Full-1-1   Num=%d   ADDR=0x%0004X    Start time %02X-%02X-%02X %02X:%02X:%02X \r\n",record_num,Read_ADDR,eep_reg[0],eep_reg[1],eep_reg[2],eep_reg[3],eep_reg[4],eep_reg[5]);

	   	}

	    for(i=0;i<(SHK_EVENT_MAXNUM-sys_config.SaveWr);i++)
	   	{
          record_num++;
		   Read_ADDR=SHK_ADDR_DATA_START+((u32)(SHK_EVENT_MAXNUM-1-i)<<14);
		   eeprom_multipleread(Read_ADDR,eep_reg, 6);  
		   printf("\r\n   Full-1-2   Num=%d   ADDR=0x%0004X    Start time %02X-%02X-%02X %02X:%02X:%02X \r\n",record_num,Read_ADDR,eep_reg[0],eep_reg[1],eep_reg[2],eep_reg[3],eep_reg[4],eep_reg[5]);

	   	}
    }
	else
	{
    
	    for(i=sys_config.SaveWr;i<SHK_EVENT_MAXNUM;i++)
	   	{
          record_num++;
		   Read_ADDR=SHK_ADDR_DATA_START+((u32)(SHK_EVENT_MAXNUM-i-1)<<14);
		   eeprom_multipleread(Read_ADDR,eep_reg, 6);  
		   printf("\r\n   Full-2-2   Num=%d   ADDR=0x%0004X    Start time %02X-%02X-%02X %02X:%02X:%02X \r\n",record_num,Read_ADDR,eep_reg[0],eep_reg[1],eep_reg[2],eep_reg[3],eep_reg[4],eep_reg[5]);

	   	}
		 
	}
	   
  }	

}

void  SHK_SaveData_Output(u8  Rec_Num)
{
   
   u16 i=0,j=0;
   u32 Read_ADDR=0;
   u32 Counter_addr=0;

  
  Read_ADDR=SHK_ADDR_DATA_START+((u32)Rec_Num<<14);
  eeprom_multipleread(Read_ADDR,eep_reg, 6);  
  printf("\r\n	  ADDR=0x%0004X	  Start time %02X-%02X-%02X %02X:%02X:%02X \r\n",Read_ADDR,eep_reg[0],eep_reg[1],eep_reg[2],eep_reg[3],eep_reg[4],eep_reg[5]);

   for(i=0;i<32;i++)
   	{
   	   eeprom_multipleread(Read_ADDR+Counter_addr,SHK_SaveData_Buff,512);  
	   Counter_addr+=512;
	   if(Counter_addr>0x3846)    //0x3846=18*800+6
	   	{
             for(j=0;j<0x46;j++)
	       	{
	            printf(" %02X",SHK_SaveData_Buff[j]);
	       	}

	   	}
	   else
	   	{	   
	       for(j=0;j<512;j++)
	       	{
	            printf(" %02X",SHK_SaveData_Buff[j]); 
	       	}
	   	}  		   

   	}
  

}


void  SHK_OutPut_OneEvent_Protocol(u8  Rec_Num)
{
   
   u16 i=0,j=0;
   u32 Read_ADDR=0;
   u32 Counter_addr=0;
   u16  Read_len=0;

  
  Read_ADDR=SHK_ADDR_DATA_START+((u32)Rec_Num<<14);
 // eeprom_multipleread(Read_ADDR,eep_reg, 6);  
 // printf("\r\n	  ADDR=0x%0004X	  Start time %02X-%02X-%02X %02X:%02X:%02X \r\n",Read_ADDR,eep_reg[0],eep_reg[1],eep_reg[2],eep_reg[3],eep_reg[4],eep_reg[5]);

   for(i=0;i<29;i++)
   	{
   	     //Light
   	     if(i==0)
	   	     LED4_ON;
	 
            SHK_SaveData_Buff_wr=0;
            SHK_SaveData_Buff[SHK_SaveData_Buff_wr++]='s';
			SHK_SaveData_Buff[SHK_SaveData_Buff_wr++]=0x01; //  ·Ö°ü±êÖ¾
			SHK_SaveData_Buff[SHK_SaveData_Buff_wr++]=i+1; // LSB	
			SHK_SaveData_Buff[SHK_SaveData_Buff_wr++]=0x00; //   MSB
			SHK_SaveData_Buff[SHK_SaveData_Buff_wr++]=29;//  ×Ü°üÊý  LSB
			SHK_SaveData_Buff[SHK_SaveData_Buff_wr++]=0x00;// MSB
		   if(i==0)	
		   	{
				SHK_SaveData_Buff[SHK_SaveData_Buff_wr++]=0x06 ;   //  ³¤¶ÈLSB     504+14=518
				SHK_SaveData_Buff[SHK_SaveData_Buff_wr++]= 0x02;
				Read_len=0x0206-8;
		   	}
		   else
		   	if(i==28)
			{
				SHK_SaveData_Buff[SHK_SaveData_Buff_wr++]=0x28 ;	 //  ³¤¶ÈLSB	 288+8=296
				SHK_SaveData_Buff[SHK_SaveData_Buff_wr++]= 0x01; 
				Read_len=0x0128-8;
			}
		   else
		   	{
                SHK_SaveData_Buff[SHK_SaveData_Buff_wr++]=0x00 ;   //  ³¤¶ÈLSB     504+8=512
				SHK_SaveData_Buff[SHK_SaveData_Buff_wr++]= 0x02; 
				Read_len=0x0200-8;
		   	}

        //----------------------------------
		 eeprom_multipleread(Read_ADDR+Counter_addr,SHK_SaveData_Buff+SHK_SaveData_Buff_wr,Read_len);  
		 Counter_addr+=Read_len;

		 SHK_SaveData_Buff_wr+=Read_len;  	    
			 
        //   ------------------     
	             U1_TxBuf[0]=0x7E;
	             U1_TxBuf_wr=Protocol7E_Encode(U1_TxBuf+1,SHK_SaveData_Buff,SHK_SaveData_Buff_wr);
				 U1_TxBuf_wr++;
				 U1_TxBuf[U1_TxBuf_wr++]=0x7E; 
				 for(j=0;j<U1_TxBuf_wr;j++)
				    U1_Tx_OneByte( U1_TxBuf[j]);
		//------------------------------------------------		
	    // Light
	    if(i==28)
			    LED4_OFF;
		 

  }
 }

void  SHK_OutPut_AllEvent_Protocol(void)
{
	 u8 eep_reg[6];
	 u8 i=0;
	
	 eeprom_multipleread(SHK_ADDR_SAVE_STATUS,eep_reg, 5);	 
	 sys_config.SaveWr=eep_reg[0];
	 sys_config.SaveFull_state=eep_reg[3];
	
 
  if(sys_config.SaveFull_state==0)
  {
     if(sys_config.SaveWr)
     {
	   for(i=0;i<sys_config.SaveWr;i++)
	   	{
		   SHK_OutPut_OneEvent_Protocol((sys_config.SaveWr-i-1));
	   	}
     }
  }
  else
  if(sys_config.SaveFull_state==1)   // ´æ´¢Âú¹ý
  {
    if(sys_config.SaveWr)
     { 
	    for(i=0;i< sys_config.SaveWr;i++)
	   	{
			SHK_OutPut_OneEvent_Protocol((sys_config.SaveWr-i-1));

	   	}

	    for(i=0;i<(SHK_EVENT_MAXNUM-sys_config.SaveWr);i++)
	   	{
		   SHK_OutPut_OneEvent_Protocol((SHK_EVENT_MAXNUM-1-i));

	   	}
    }
	else
	{
    
	    for(i=sys_config.SaveWr;i<SHK_EVENT_MAXNUM;i++)
	   	{
		  SHK_OutPut_OneEvent_Protocol(SHK_EVENT_MAXNUM-i-1);
	   	}
		 
	}
	   
  }	



}


void Sample_Get(void)
{
   u16   i=0;
   u8    compare[600];
   u8    reg[5];
   	
	 if(SHK_WRK_STATE.ENABLE_Triger==1)    //  K1 °´¼ü°´ÏÂ
	 {
		 //    Part 1  :  Trans 
		 if((SHK_WRK_STATE.current_packet==0)&&(SHK_WRK_STATE.Page_in_offset==0))
		 {	//	Ö»ÓÐµÚÒ»°ü ¼ÓÊ±¼ä´Á
		    //--------------------------------------------------------------------
			 // read eeprom
			 eeprom_multipleread(SHK_ADDR_SAVE_STATUS,reg, 5);	 
			 sys_config.SaveWr=reg[0];
			 sys_config.SaveFull_state=reg[3];
			  printf("\r\n  Trigger  Enable sys_config.SaveWr=%d	sys_config.SaveFull_state=%d  \r\n",sys_config.SaveWr,sys_config.SaveFull_state);
                

		    //-------------------------------------------------------------------
		        SHK_SampleData_Buff_wr=0;				
				SHK_WRK_STATE.Event_WriteADDR=SHK_ADDR_DATA_START+((u32)sys_config.SaveWr<<14); // ¼ÇÂ¼ÎÄ¼þµÄÆðÊ¼µØÖ·
				memcpy(SHK_SampleData_Buff+SHK_SampleData_Buff_wr,rtc_current.BCD_6_Bytes,6); 
				//printf("\r\n Start ADDR=0x%0004X  Save start time %02X-%02X-%02X %02X:%02X:%02X \r\n",SHK_WRK_STATE.Event_WriteADDR,SHK_SampleData_Buff[0],SHK_SampleData_Buff[1],SHK_SampleData_Buff[2],SHK_SampleData_Buff[3],SHK_SampleData_Buff[4],SHK_SampleData_Buff[5]);
				SHK_SampleData_Buff_wr+=6;	
         }	   
		  // Below	   run	every time 
		 SHK_Sensor_data_Get();
		 memcpy(SHK_SampleData_Buff+SHK_SampleData_Buff_wr,Project_SHK.ACCEL_ADX375_ACCEL_6BYTES,6);
		 SHK_SampleData_Buff_wr+=6;
		 memcpy(SHK_SampleData_Buff+SHK_SampleData_Buff_wr,Project_SHK.GYRO_L3G_6BYTES,6);
		 SHK_SampleData_Buff_wr+=6;
		 memcpy(SHK_SampleData_Buff+SHK_SampleData_Buff_wr,Project_SHK.COMPASS_HMC_6BYTES,6);
		 SHK_SampleData_Buff_wr+=6;
		 SHK_WRK_STATE.current_packet++;
			 
		 //   ------------------
         if(SHK_SampleData_Buff_wr>=SHK_PAGE_SIZE)
         {
                
				// 1<<14  ±íÊ¾x16K
				eeprom_writemultipledata(SHK_WRK_STATE.Event_WriteADDR,SHK_SampleData_Buff,SHK_PAGE_SIZE);
                SHK_WRK_STATE.Event_WriteADDR+=SHK_PAGE_SIZE;

               #if 0
				 for(i=0;i<SHK_PAGE_SIZE;i++)
						printf(" %02X",SHK_SampleData_Buff[i]);
               #endif 
			   
                SHK_WRK_STATE.Page_in_offset=SHK_SampleData_Buff_wr-SHK_PAGE_SIZE;

				// copy left to head    ,update  write  addr
				memcpy(SHK_SampleData_Buff,SHK_SampleData_Buff+PAGESIZE,SHK_WRK_STATE.Page_in_offset);
			    SHK_SampleData_Buff_wr=SHK_WRK_STATE.Page_in_offset;			   
				  
				 
			  

            

         }

		 if(SHK_WRK_STATE.current_packet>=SHK_WRK_STATE.total_packet)
		 {

                 
				 eeprom_writemultipledata(SHK_WRK_STATE.Event_WriteADDR,SHK_SampleData_Buff,SHK_SampleData_Buff_wr);
                
				SHK_WRK_STATE.Event_WriteADDR+=SHK_SampleData_Buff_wr;
			//	printf("\r\n Packet:%d  ADDR=0x%0004X    Subnum: %d:\r\n",SHK_WRK_STATE.current_packet,SHK_WRK_STATE.Event_WriteADDR,SHK_WRK_STATE.Page_in_offset);
				      #if 0
				 for(i=0;i<SHK_SampleData_Buff_wr;i++)
						printf(" %02X",SHK_SampleData_Buff[i]);
               #endif 
       
                //  Save end  Process
				 SHK_Event_SaveEnd();
				 //  Checke  wether event  happen again 
				 SHK_Check_EventHppen_Again();		

				 SHK_SaveData_Buff_wr=0;
		 }

	}



}


void  EEPROM_of_SYS_Init(void)
{    //  ºÍ´æ´¢Ïà¹ØµÄEEPROM  ²Ù×÷
   u16  u16_reg[3];
    u8  eep_read[50];
	u8  i=0,state=1,couter=0;
/*
	u8  chip=1;
    eep_read[0]=eeprom_RegisterByte_Read(1,RDSR);
	printf("\r\n chip %d RDSR=0x%02X ",chip, eep_read[0]);
     eep_read[0]=eeprom_RegisterByte_Read(1,WRSR);
	printf("\r\n chip %d  WRSR=0x%02X ",chip, eep_read[0]);
	 eep_read[0]=eeprom_RegisterByte_Read(1,LIDS);
	printf("\r\n chip %d  LIDS=0x%02X ",chip, eep_read[0]);
	 eep_read[0]=eeprom_RegisterByte_Read(1,WREN);
	printf("\r\n chip %d  WREN=0x%02X ",chip, eep_read[0]);
	 eep_read[0]=eeprom_RegisterByte_Read(1,RDLS);
	printf("\r\n chip %d  RDLS=0x%02X ",chip, eep_read[0]);

   chip=2;
    eep_read[0]=eeprom_RegisterByte_Read(chip,RDSR);
	printf("\r\n chip %d RDSR=0x%02X ",chip, eep_read[0]);
     eep_read[0]=eeprom_RegisterByte_Read(chip,WRSR);
	printf("\r\n chip %d  WRSR=0x%02X ",chip, eep_read[0]);
	 eep_read[0]=eeprom_RegisterByte_Read(chip,LIDS);
	printf("\r\n chip %d  LIDS=0x%02X ",chip, eep_read[0]);
	 eep_read[0]=eeprom_RegisterByte_Read(chip,WREN);
	printf("\r\n chip %d  WREN=0x%02X ",chip, eep_read[0]);
	 eep_read[0]=eeprom_RegisterByte_Read(chip,RDLS);
	printf("\r\n chip %d  RDLS=0x%02X ",chip, eep_read[0]);

    eeprom_RegisterByte_WR(1,RDSR,0x0f);
	delay_ms_1(200);
	eep_read[0]=eeprom_RegisterByte_Read(1,RDSR);
	printf("\r\n  wr read chip %d RDSR=0x%02X ",chip, eep_read[0]);
*/
   // Read and  Check  the  EEprom
	eeprom_multipleread(SHK_ADDR_INIT,eep_read, 6);  
    u16_reg[0]=((u16)eep_read[0]<<8)+(u16)eep_read[1];  //  ´æ´¢3 ¸öÇøÓò
	u16_reg[1]=((u16)eep_read[2]<<8)+(u16)eep_read[3];
	u16_reg[2]=((u16)eep_read[4]<<8)+(u16)eep_read[5]; 
	printf("\r\n  Read  SYS_ID  part    reg1=0x%0004X reg2=0x%0004X   reg3=0x%0004X",u16_reg[0],u16_reg[1],u16_reg[2]);

    if(((SYS_ID==u16_reg[0])&&(SYS_ID==u16_reg[1]))||((SYS_ID==u16_reg[1])&&(SYS_ID==u16_reg[2]))||((SYS_ID==u16_reg[0])&&(SYS_ID==u16_reg[2])))	
    {  //  ÓÐ2 ¸ö Ò»Ñù¾ÍÐÐ
        sys_config.SYStem_ID=SYS_ID;
		sys_config.Version_Info=VERSION_INFO;
        
		eeprom_multipleread( SHK_ADDR_SYSCONFI,eep_read,15); 
        memcpy(sys_config.StartWork_RTC,eep_read,6);
	    memcpy(sys_config.EndWork_RTC,eep_read+6,6);
		memcpy(sys_config.Threshod_ADXL375,eep_read+12,6);
        
		eeprom_multipleread( SHK_ADDR_SAVE_STATUS,eep_read,5);	
		if(eep_read[0]>SHK_EVENT_MAXNUM)
			sys_config.SaveWr=eep_read[1];  // È¡ÏÂÒ»¸ö
		else
			sys_config.SaveWr=eep_read[0]; 

		if((eep_read[3]!=0)&&(eep_read[3]!=1))
			  sys_config.SaveFull_state=eep_read[4];  // È¥ÏÂÒ»¸ö
		else
			  sys_config.SaveFull_state=eep_read[3];

        eeprom_multipleread( SHK_ADDR_SAVE_USRINFO,sys_config.User_info,256);  


		printf("\r\n   Normal  Read    work StartTime    %02X-%02X-%02X %02X:%02X:%02X",sys_config.StartWork_RTC[0],\
			 sys_config.StartWork_RTC[1],sys_config.StartWork_RTC[2],sys_config.StartWork_RTC[3],sys_config.StartWork_RTC[4],\
			 sys_config.StartWork_RTC[5]);
		printf("\r\n                        EndTime      %02X-%02X-%02X %02X:%02X:%02X",sys_config.EndWork_RTC[0],\
			 sys_config.EndWork_RTC[1],sys_config.EndWork_RTC[2],sys_config.EndWork_RTC[3],sys_config.EndWork_RTC[4],\
			 sys_config.EndWork_RTC[5]);
		printf("\r\n   Wr_record=%d    saveFull_state=%d \r\n",sys_config.SaveWr,sys_config.SaveFull_state);
        printf("\r\n    TRESH_ACT=0x%02X    TRESH_INACT=0x%02X    TIME_INACT=0x%02X \r\n",sys_config.Threshod_ADXL375[0],sys_config.Threshod_ADXL375[1],sys_config.Threshod_ADXL375[2]);
        printf("\r\n    ÓÃ»§×Ô¶¨ÒåÐÅÏ¢: %s\r\n",sys_config.User_info);

	}
	else
	{   // write        
	   //  Write   SYS_ID
		eep_reg[0]=(u8)(SYS_ID>>8);
		eep_reg[1]=(u8)(SYS_ID);
		eep_reg[2]=(u8)(SYS_ID>>8);
		eep_reg[3]=(u8)(SYS_ID);
		eep_reg[4]=(u8)(SYS_ID>>8);
		eep_reg[5]=(u8)(SYS_ID);		
		eeprom_writemultipledata( SHK_ADDR_INIT, eep_reg,6);
	  // Write  default   START  TIME  ºÍ shake  window 	
	   memcpy(eep_reg,sys_config.StartWork_RTC,6);
	   memcpy(eep_reg+6,sys_config.EndWork_RTC,6);
	   memcpy(eep_reg+12,sys_config.Threshod_ADXL375,3);
       eeprom_writemultipledata( SHK_ADDR_SYSCONFI,eep_reg,15); 

	  //Write  µ±Ç°´æ´¢ÌõÊý 3 ·Ý   ÊÇ·ñ´æ´¢Âú2 ·Ý
	  eep_reg[0]=sys_config.SaveWr;
	  eep_reg[1]=sys_config.SaveWr;
	  eep_reg[2]=sys_config.SaveWr;
	  eep_reg[3]=sys_config.SaveFull_state;
	  eep_reg[4]=sys_config.SaveFull_state;
      eeprom_writemultipledata( SHK_ADDR_SAVE_STATUS,eep_reg,5);  


	  eeprom_writemultipledata( SHK_ADDR_SAVE_USRINFO,sys_config.User_info,256);  

      //    Ê×´ÎÊ¹ÓÃÉèÖÃ³õÊ¼»¯RTC
	rtc_current.year = 0x10;
	rtc_current.month = 0x02;
	rtc_current.day = 0x26;
	rtc_current.hour = 0x10;
	rtc_current.min = 0x11;
	rtc_current.sec=0x12;
	
      RTC8564_Set(rtc_current);


	  
	  printf("\r\n   Ê×´Î Ê¹ÓÃ ³õÊ¼»¯´æ´¢Ïà¹ØÐÅÏ¢ ! ");
	  
	}
    // check again    À´ÅÐ¶ÏEEPROM ÊÇ·ñËð»µ »ò¹¤×÷Òì³£    
	eeprom_multipleread(SHK_ADDR_INIT,eep_read, 10) ;
	couter=0;
    for(i=0;i<10;i++)
    {
       if((eep_read[i]!=0xFF)&&(eep_read[i]!=0x00))
       {
          state=0;
          break;
       }	 
	   else
	   	  couter++;
    }

    if((state==1)&&(couter==10))
		 printf("\r\n         EEPROM  Ð¾Æ¬ÓÐ¹ÊÕÏ !!!!");

}



//   Total  Use
void Devices_Init_Total(void)
{
  u16 i=0, left=0;
  u8  compare[600];
  
  HMC5883L_Init(); 
  RTC8564_Init();    
  Init_L3GD20H(); 
  //------ eeprom ---------
  eeprom_POWER_ON();   
  eeprom_wrdisable(1);
  eeprom_wrdisable(2);
  eeprom_cs1_high();
  eeprom_cs2_high();  
  eeprom_cs1_wpHIGH();
  eeprom_cs2_wpHIGH();
  
  System_config_init();
  EEPROM_of_SYS_Init();
  //-----------------------
  printf("\r\n --------------------------------------   ");
   ADXL375_init();  
  printf("\r\n --------------------------------------   ");   
 
  get_chipid();
  USART1_RX_enable();

#if 0
sys_config.SaveWr=8;
SHK_WRK_STATE.Event_WriteADDR=SHK_ADDR_DATA_START+((u32)sys_config.SaveWr<<14); // ¼ÇÂ¼ÎÄ¼þµÄÆðÊ¼µØÖ·
SHK_SaveData_Buff_wr=266;
for(i=0;i<SHK_SaveData_Buff_wr;i++)
  {
     if(i)
	 	 SHK_SaveData_Buff[i]=i+7*i^SHK_SaveData_Buff[i-1]+(left*i)%9;
     else
	   SHK_SaveData_Buff[i]=i+5*i+left*3;
     left+=i;
  }
		   eeprom_multipleread(SHK_WRK_STATE.Event_WriteADDR+256,compare,30);
			
	   printf("\r\n read  0:\r\n"); 	
		  for(i=0;i<30;i++)
		  {
					   printf(" %02X",compare[i]); 
		  }

		 eeprom_multipleread(SHK_WRK_STATE.Event_WriteADDR,compare,30);
			
	   printf("\r\n read  1:\r\n"); 	
		  for(i=0;i<30;i++)
		  {
					   printf(" %02X",compare[i]); 
		  }

    //   SHK_WRK_STATE.Event_WriteADDR+=6;
	   eeprom_writemultipledata(SHK_WRK_STATE.Event_WriteADDR,SHK_SaveData_Buff,SHK_SaveData_Buff_wr);
			   printf("\r\n Packet:%d Subnum: %d ADDR=0x%0004X :\r\n",SHK_WRK_STATE.current_packet,SHK_WRK_STATE.Page_in_offset,SHK_WRK_STATE.Event_WriteADDR);
			   for(i=0;i<SHK_SaveData_Buff_wr;i++)
				  printf(" %02X",SHK_SaveData_Buff[i]); 

   // SHK_WRK_STATE.Event_WriteADDR-=6;
	eeprom_multipleread(SHK_WRK_STATE.Event_WriteADDR,compare,SHK_SaveData_Buff_wr);
	 
printf("\r\n read  2:\r\n");	 
   for(i=0;i<SHK_SaveData_Buff_wr;i++)
   {
				printf(" %02X",compare[i]); 
   }	

	eeprom_multipleread(SHK_WRK_STATE.Event_WriteADDR+256,compare,30);
	 
printf("\r\n read  3:\r\n");	 
   for(i=0;i<30;i++)
   {
				printf(" %02X",compare[i]); 
   }

   
#endif
#if 0



sys_config.SaveWr=10;
SHK_WRK_STATE.Event_WriteADDR=SHK_ADDR_DATA_START+((u32)sys_config.SaveWr<<14); // ¼ÇÂ¼ÎÄ¼þµÄÆðÊ¼µØÖ·
SHK_SaveData_Buff_wr=213;
for(i=0;i<SHK_SaveData_Buff_wr;i++)
  {
	 SHK_SaveData_Buff[i]=i+5;

  }
	   

	   eeprom_writemultipledata(SHK_WRK_STATE.Event_WriteADDR,SHK_SaveData_Buff,SHK_SaveData_Buff_wr);
			   printf("\r\n Packet:%d Subnum: %d ADDR=0x%0004X :\r\n",SHK_WRK_STATE.current_packet,SHK_WRK_STATE.Page_in_offset,SHK_WRK_STATE.Event_WriteADDR);
			   for(i=0;i<SHK_SaveData_Buff_wr;i++)
				  printf(" %02X",SHK_SaveData_Buff[i]); 
//---------------------------
	sys_config.SaveWr=6;
	SHK_WRK_STATE.Event_WriteADDR=SHK_ADDR_DATA_START+((u32)sys_config.SaveWr<<14); // ¼ÇÂ¼ÎÄ¼þµÄÆðÊ¼µØÖ·
	SHK_SaveData_Buff_wr=222;

	eeprom_multipleread(SHK_WRK_STATE.Event_WriteADDR,compare,SHK_SaveData_Buff_wr);
	 
printf("\r\n read  1:");	 
   for(i=0;i<SHK_SaveData_Buff_wr;i++)
   {
				printf(" %02X",compare[i]); 
   }
//---------------------------
sys_config.SaveWr=10;
SHK_WRK_STATE.Event_WriteADDR=SHK_ADDR_DATA_START+((u32)sys_config.SaveWr<<14); // ¼ÇÂ¼ÎÄ¼þµÄÆðÊ¼µØÖ·
SHK_SaveData_Buff_wr=213;			   
			   eeprom_multipleread(SHK_WRK_STATE.Event_WriteADDR,compare,SHK_SaveData_Buff_wr);
		   printf("\r\n read 2 :");	
			  for(i=0;i<SHK_SaveData_Buff_wr;i++)
			  {
				   printf(" %02X",compare[i]); 
			  }



//--------------------------
sys_config.SaveWr=3;
SHK_WRK_STATE.Event_WriteADDR=SHK_ADDR_DATA_START+((u32)sys_config.SaveWr<<14); // ¼ÇÂ¼ÎÄ¼þµÄÆðÊ¼µØÖ·
SHK_SaveData_Buff_wr=202;

for(i=0;i<SHK_SaveData_Buff_wr;i++)
  {
	 SHK_SaveData_Buff[i]=i+2;

  }
	   /*

	   eeprom_writemultipledata(SHK_WRK_STATE.Event_WriteADDR,SHK_SaveData_Buff,SHK_SaveData_Buff_wr);
			   delay_ms_1(300);
			   printf("\r\n Packet:%d Subnum: %d ADDR=0x%0004X :\r\n",SHK_WRK_STATE.current_packet,SHK_WRK_STATE.Page_in_offset,SHK_WRK_STATE.Event_WriteADDR);
			   for(i=0;i<SHK_SaveData_Buff_wr;i++)
				  printf(" %02X",SHK_SaveData_Buff[i]); 
*/			   
			   eeprom_multipleread(SHK_WRK_STATE.Event_WriteADDR,compare,SHK_SaveData_Buff_wr);
				delay_ms_1(300);
		   printf("\r\n read :");	
			  for(i=0;i<SHK_SaveData_Buff_wr;i++)
			  {
				   if(compare[i]!=SHK_SaveData_Buff[i])
				   {
					  printf(" \r\n error happened at  %d  wr: %02X  rd:%02X ",i,SHK_SaveData_Buff[i],compare[i]); 
				   }
				   else
					   printf(" %02X",compare[i]); 
			  }


#endif


#if 0
  sys_config.SaveWr=6;
  SHK_WRK_STATE.Event_WriteADDR=SHK_ADDR_DATA_START+((u32)sys_config.SaveWr<<14)+15; // ¼ÇÂ¼ÎÄ¼þµÄÆðÊ¼µØÖ·
  SHK_SaveData_Buff_wr=372;
  for(i=0;i<SHK_SaveData_Buff_wr;i++)
  	{
       SHK_SaveData_Buff[i]=i+2;

  	}
  
  		 eeprom_writemultipledata(SHK_WRK_STATE.Event_WriteADDR,SHK_SaveData_Buff,SHK_SaveData_Buff_wr);
				 delay_ms_1(300);
				 printf("\r\n Packet:%d Subnum: %d ADDR=0x%0004X :\r\n",SHK_WRK_STATE.current_packet,SHK_WRK_STATE.Page_in_offset,SHK_WRK_STATE.Event_WriteADDR);
				 for(i=0;i<SHK_SaveData_Buff_wr;i++)
				 	printf(" %02X",SHK_SaveData_Buff[i]); 
				 
				 eeprom_multipleread(SHK_WRK_STATE.Event_WriteADDR,compare,SHK_SaveData_Buff_wr);
				  delay_ms_1(300);
			 printf("\r\n read :");	  
             if(SHK_SaveData_Buff_wr<=250)			 	
             {
                for(i=0;i<SHK_SaveData_Buff_wr;i++)
				{
					 if(compare[i]!=SHK_SaveData_Buff[i])
					 {
					    printf(" \r\n error happened at  %d  wr: %02X  rd:%02X ",i,SHK_SaveData_Buff[i],compare[i]); 
					 }
					 else
					 	 printf(" %02X",compare[i]); 
                }
             }	
			 else
			 	{
			 	       for(i=0;i<250;i++)
						{
							 if(compare[i]!=SHK_SaveData_Buff[i])
							 {
							    printf(" \r\n Large-1 error happened at  %d  wr: %02X  rd:%02X ",i,SHK_SaveData_Buff[i],compare[i]); 
							 }
							 else
							 	printf(" %02X",compare[i]);
		                } 
					   
					  //  eeprom_writemultipledata(0x40100,compare,SHK_SaveData_Buff_wr);
					    delay_ms_1(300); 
                        //SHK_WRK_STATE.Event_WriteADDR=SHK_WRK_STATE.Event_WriteADDR+250;
						left=SHK_SaveData_Buff_wr-250;
                        eeprom_multipleread(SHK_WRK_STATE.Event_WriteADDR,compare,left);
                        for(i=0;i<(SHK_SaveData_Buff_wr-250);i++)
						{
							 if(compare[i]!=SHK_SaveData_Buff[i+250])
							 {
							    printf(" \r\n Large-2 error happened at  %d  wr: %02X  rd:%02X ",i,SHK_SaveData_Buff[i+250],compare[i]); 
							 }
							 else
							 	printf(" %02X",compare[i]); 
		                } 

			 	}
  #endif			 

}


u16  Protocol7E_Encode(u8 *Dest, u8 *Src, u16 srclen)
{
    u16  lencnt = 0, destcnt = 0;

    for(lencnt = 0; lencnt < srclen; lencnt++)
    {
        if(Src[lencnt] == 0x7e) // 7e ×ªÒå
        {
            Dest[destcnt++] = 0x7d;
            Dest[destcnt++] = 0x02;
        }
        else if(Src[lencnt] == 0x7d) //  7d  ×ªÒå
        {
            Dest[destcnt++] = 0x7d;
            Dest[destcnt++] = 0x01;
        }
        else
            Dest[destcnt++] = Src[lencnt]; // Ô­Ê¼ÐÅÏ¢
    }

    return destcnt; //·µ»Ø×ªÒåºóµÄ³¤¶È

}
//-------------------------------------------------------------------------------
u16 Protocol7E_Decode(u8 *Dest, u8 *Src, u16 srclen) // ½âÎöÖ¸¶¨buffer :  UDP_HEX_Rx
{
    //-----------------------------------
    u16 i = 0,UDP_DecodeHex_Len=0;
  
    // 1.  clear  write_counter
   // UDP_DecodeHex_Len = 0; //clear DecodeLen

    // 2   decode process
    for(i = 0; i < srclen; i++)
    {
        if((Dest[i] == 0x7d) && (Src[i + 1] == 0x02))
        {
            Dest[UDP_DecodeHex_Len] = 0x7e;
            i++;
        }
        else if((Dest[i] == 0x7d) && (Src[i + 1] == 0x01))
        {
            Dest[UDP_DecodeHex_Len] = 0x7d;
            i++;
        }
        else
        {
            Dest[UDP_DecodeHex_Len] = Src[i];
        }
        UDP_DecodeHex_Len++;
    }
    //  3.  The  End
     return  UDP_DecodeHex_Len;
}

void  SHK_Sensor_data_Get(void)
{
	 HMC5883L_Get();
     L3GD20H_GET();     
     ADXL375_Get();
}

void Voltage_Get(void)
{  /*
             ²Î¿¼µçÑ¹ÊÇ3.1V                 
             ³äÂú  4.2 V                  
                                         stage 4 :  4.1  ~ 4.2    100%
                                         stage 3:  4.0~4.1        75%
                                         stage 2:  4.0~3.8      50%
                                         stage 1:  3.8~3.7      25%
                                         stage 0    <3.7          no power
             À¡µç  3.7 V  
     */
			 
    Project_SHK.Voltage_ADC_Value=HAL_ADC_GetValue(&hadc);
	Project_SHK.Voltage_V=(u16)(Project_SHK.Voltage_ADC_Value*0.2256);  //    3.08  3.0*3     (SHK_ADC_VALUE*900/4095);  900/4095=0.22      930/4095=0.227
    if(Project_SHK.Voltage_V>=410)
            Project_SHK.Power_Stage=4;
	else
	if((Project_SHK.Voltage_V>=395)&&(Project_SHK.Voltage_V<410))
            Project_SHK.Power_Stage=3;
	else
	if((Project_SHK.Voltage_V>=380)&&(Project_SHK.Voltage_V<395)) 
            Project_SHK.Power_Stage=2;
	else
	if((Project_SHK.Voltage_V>=370)&&(Project_SHK.Voltage_V<380))
            Project_SHK.Power_Stage=1;
	else
	        Project_SHK.Power_Stage=0;

	
	// printf("\r\n ADC value=0x%0004X	  =>  %d  Voltage=%d.%02d V  Stage=%d ",Project_SHK.Voltage_ADC_Value,Project_SHK.Voltage_ADC_Value,Project_SHK.Voltage_V/100,Project_SHK.Voltage_V%100,Project_SHK.Power_Stage );  
}

void Rx_CMD_process(void)
{
  u8  Decode_buf[100];
  u16 decode_len=0;
  
  memset(Decode_buf,0,sizeof(Decode_buf));
  if(U1_rx_sub1_wr>2)
     decode_len=Protocol7E_Decode(Decode_buf,U1_rx_sub1+1,U1_rx_sub1_wr-2);  // ½âÎöÄÚÈÝÈ¥µôÍ·Î² 7E  ³¤¶È¼õÈ¥2
  else
  	 return;
  if(Uart1Ready_R)
  {
     
     switch(Decode_buf[0])
     	{
           case 'T':   if((decode_len==1)||(decode_len==7) )    //  ¶ÁÈ¡µ±Ç°Ê±¼ä
           	           {
                          
						  if(decode_len==7)	//	ÉèÖÃµ±Ç°Ê±¼ä
						 {
						    rtc_current.year = Decode_buf[1];
							rtc_current.month =Decode_buf[2];
							rtc_current.day = Decode_buf[3];
							rtc_current.hour = Decode_buf[4];
							rtc_current.min = Decode_buf[5];
							rtc_current.sec=Decode_buf[6];							
							RTC8564_Set(rtc_current);
						  
						 }
					   
         				   // ACK 
							memset(Ack_buf,0,sizeof(Ack_buf));
							Ack_buf[0]=0x7E;
							Ack_buf[1]='t';
							memcpy(Ack_buf+2,Decode_buf,6);
							Ack_buf[2]=	rtc_current.year;
							Ack_buf[3]=	rtc_current.month;
							Ack_buf[4]=	rtc_current.day ;
							Ack_buf[5]=	rtc_current.hour;
							Ack_buf[6]=	rtc_current.min ;
							Ack_buf[7]=	rtc_current.sec;	
							Ack_buf[8]=0x7E;
							Output_Data(Ack_buf,9); 
                           
           	           }
		               else
					   if(decode_len==13)    // ÉèÖÃRTC  ºÍÆðÊ¼Ê±¼äÊ±¼ä
		               {
		                    rtc_current.year = Decode_buf[1];
							rtc_current.month =Decode_buf[2];
							rtc_current.day = Decode_buf[3];
							rtc_current.hour = Decode_buf[4];
							rtc_current.min = Decode_buf[5];
							rtc_current.sec=Decode_buf[6];							
							RTC8564_Set(rtc_current);
							memcpy(sys_config.StartWork_RTC,Decode_buf+7,6);  // The same with  current
                            eeprom_writemultipledata( SHK_ADDR_SYSCONFI,sys_config.StartWork_RTC,6); 
							// ACK 
							memset(Ack_buf,0,sizeof(Ack_buf));
							Ack_buf[0]=0x7E;
							Ack_buf[1]='t';
							memcpy(Ack_buf+2,Decode_buf,12);
							Ack_buf[14]=0x7E;
							Output_Data(Ack_buf,15); 

		               }		   	           
		   	           break;
          
		   case 'D':
		   	          if((decode_len==1)||(decode_len==7) )    //  ¶ÁÈ¡ ÉèÖÃ
           	           {
                          
						  if(decode_len==7)	//	ÉèÖÃ ½áÊøÊ±¼ä
						 {
                            memcpy(sys_config.EndWork_RTC,Decode_buf+1,6);  // The same with  current
                            eeprom_writemultipledata( SHK_ADDR_SYSCONFI,sys_config.EndWork_RTC,6); 
						 }

						  //  ACK
						  memset(Ack_buf,0,sizeof(Ack_buf));
							Ack_buf[0]=0x7E;
							Ack_buf[1]='d';
							memcpy(Ack_buf+2,sys_config.Device_ID,12);
							memcpy(Ack_buf+14,sys_config.EndWork_RTC,6);
							Ack_buf[20]=Project_SHK.Power_Stage;  // µçÔ´×´Ì¬
						    Ack_buf[21]=0x7E;
							Output_Data(Ack_buf,22); 
						  
		   	           }  
					   break;
					   
		   case 'P':
		   	         if((decode_len==1)||(decode_len==4) )    //  ¶ÁÈ¡ ÉèÖÃ
           	           {
                          
						  if(decode_len==4)	//	ÉèÖÃ ½áÊøÊ±¼ä
						 {
                            memcpy(sys_config.Threshod_ADXL375,Decode_buf+1,3);  // The same with  current
                            eeprom_writemultipledata( SHK_ADDR_SYSCONFI+12,sys_config.Threshod_ADXL375,3); 
							// ÉèÖÃ ¼ÓËÙ¶È´«¸ÐÆ÷
							ADXL375_Wr_OneByte(0x24,sys_config.Threshod_ADXL375[0]);  //±£´æ¼ì²â»î¶¯·§Öµ; (780mg/LSB)	01 
							ADXL375_Wr_OneByte(0x25,sys_config.Threshod_ADXL375[1]);	//±£´æ¼ì²â¾²Ö¹·§Öµ; (780mg/LSB)
							ADXL375_Wr_OneByte(0x26,sys_config.Threshod_ADXL375[2]);  //¼ì²â»î¶¯Ê±¼ä·§Öµ; (1s/LSB)
						 }

						  //  ACK
						    memset(Ack_buf,0,sizeof(Ack_buf));
							Ack_buf[0]=0x7E;
							Ack_buf[1]='p';
							memcpy(Ack_buf+2,sys_config.Threshod_ADXL375,3);
							Ack_buf[5]=0x7E;
							Output_Data(Ack_buf,6); 
						  
		   	           } 
					   break;
		   case 'Q':   // Õð¶¯ÐÅÏ¢
                        if((decode_len==1)||(decode_len==4) )    //  ¶ÁÈ¡ ÉèÖÃ
           	           {
                          
						  if(decode_len==4)	//	ÉèÖÃ ½áÊøÊ±¼ä
						 {
                            memcpy(sys_config.Threshod_ADXL375+3,Decode_buf+1,3);  // The same with  current
                            eeprom_writemultipledata( SHK_ADDR_SYSCONFI+12+3,sys_config.Threshod_ADXL375+3,3); 
							// ÉèÖÃ ¼ÓËÙ¶È´«¸ÐÆ÷
							ADXL375_Wr_OneByte(0x1D,sys_config.Threshod_ADXL375[3]);  //±£´æ¼ì²â»î¶¯·§Öµ; (780mg/LSB)	01 
							ADXL375_Wr_OneByte(0x2C,sys_config.Threshod_ADXL375[4]);	//±£´æ¼ì²â¾²Ö¹·§Öµ; (780mg/LSB)
							//  Ô¤Áô ADXL375_Wr_OneByte(0x1F,sys_config.Threshod_ADXL375[2]);  //¼ì²â»î¶¯Ê±¼ä·§Öµ; (1s/LSB)
						 }

						  //  ACK
						    memset(Ack_buf,0,sizeof(Ack_buf));
							Ack_buf[0]=0x7E;
							Ack_buf[1]='q';
							memcpy(Ack_buf+2,sys_config.Threshod_ADXL375+3,3); 
							Ack_buf[5]=0x7E;
							Output_Data(Ack_buf,6); 
						  
		   	           } 


		   
					   break;
		   case 'R':   // ÓÃ»§ÐÅÏ¢
                       if(decode_len )    //  ¶ÁÈ¡ ÉèÖÃ
           	           {
                          
						  if(decode_len>1)	//	ÉèÖÃ ½áÊøÊ±¼ä
						 {
						    memset(sys_config.User_info,0,256);
                            memcpy(sys_config.User_info,Decode_buf+1,decode_len-1);  // The same with  current
                            eeprom_writemultipledata( SHK_ADDR_SAVE_USRINFO,sys_config.User_info,decode_len-1); 
						 }

						  //  ACK
						    memset(Ack_buf,0,sizeof(Ack_buf));
							Ack_buf[0]=0x7E;
							Ack_buf[1]='r';
							memcpy(Ack_buf+2,sys_config.User_info,256);
							Ack_buf[257]=0x7E;
							Output_Data(Ack_buf,258); 		 				  
		   	           } 
		               
					   break;			   
					   
		   case 'L':    // add  for debugging                       
						
		               SHK_SaveData_Checking(); 

					     //  ACK
						    memset(Ack_buf,0,sizeof(Ack_buf));
							Ack_buf[0]=0x7E;
							Ack_buf[1]='l';
							Ack_buf[2]=sys_config.SaveWr;
							Ack_buf[3]=0x7E;
							Output_Data(Ack_buf,4); 		
		   	           break;
		   case 'M':
		   	          if(Decode_buf[1]<=SHK_EVENT_MAXNUM)
                         SHK_SaveData_Output(Decode_buf[1]);
					  else
					  	 printf("\r\n M Illegal  Record get\r\n");
			           break;
		   case 'N'	:  
		   	          if(decode_len==2 )
		   	          {
						   if(Decode_buf[1]<=SHK_EVENT_MAXNUM)
	                         SHK_OutPut_OneEvent_Protocol(Decode_buf[1]);
						  else
						  	 printf("\r\n N  Illegal  Record get\r\n");	   
		   	          }
			           break;
		   case  'O':
		   case  'S':	
                       SHK_OutPut_AllEvent_Protocol(); 
			           break;
					   
           default: 
		   	           
					   Output_Data(U1_rx_sub1,U1_rx_sub1_wr); 
		   	           break;



     	}
  
      Uart1Ready_R=0; 
  }



}

	
void  Function_test(void)
{
   uint32_t ADDR_VALUE=0x0000F0;
  // u8   Ready_u8=0;



   //  0 .  AX375      ¼ÓËÙ¶È´«¸ÐÆ÷    LSB  
#if 0    
    memset(Sensor_6_bytes,0xAA,8);
    ADXL375_SPI2_ReadNbytes(XL375_DATAX0,Sensor_6_bytes,6);    
	//-------------------------------------------------------------
    memcpy(Project_SHK.ACCEL_ADX375_ACCEL_6BYTES,Sensor_6_bytes,6); 
    Project_SHK.ACCEL_X_vaule=(u16)(Sensor_6_bytes[1]<<8)+(u16)Sensor_6_bytes[0];
	Project_SHK.ACCEL_Y_vaule=(u16)(Sensor_6_bytes[3]<<8)+(u16)Sensor_6_bytes[2];
	Project_SHK.ACCEL_Z_vaule=(u16)(Sensor_6_bytes[5]<<8)+(u16)Sensor_6_bytes[4];

    
	ADXL375_SPI2_ReadNbytes(0x1D,Sensor_6_bytes+6,1); 
	//-------------------------------------------------------------
    Output_Data(Sensor_6_bytes,8);    
#endif	
	HAL_Delay(300);  

#if  1	
    //  3.  RTC  
      RTC8564_Get();
     
    //   4.  eeprom
 #if 0
    memset(eep_reg,0xBB,sizeof(eep_reg));
    memcpy(eep_reg,rtc_tmp,8);
    eeprom_writemultipledata( ADDR_VALUE, eep_reg,8);

	//Output_Data(eep_reg,8);
	HAL_Delay(300);
#endif	   
	
//	memset(eep_read,0xCC,sizeof(eep_read));
//	eeprom_multipleread(ADDR_VALUE,eep_read, 10);   
//	Output_Data(eep_read,12); 
//   printf("\r\n  eeprom  read         %02X-%02X-%02X %02X:%02X:%02X",eep_read[0],eep_read[1],eep_read[2],eep_read[3],eep_read[4],eep_read[5]);

   // ADXL375_Get();

    Key_Check();
  
//--------------------------------------------------------------
 #if 0
    if(Adx375_trigger_flag==1)
    	{
           Adx375_trigger_flag=0;
		   printf("Íâ²¿ÖÐ¶Ï´¥·¢! Count:%d\r\n",Shake_times++);    


    	}
 #endif	
#endif
  //  ADC  get      
  Voltage_Get();

    
   
}
  
