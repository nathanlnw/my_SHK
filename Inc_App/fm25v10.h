#ifndef __FM25V10_H
#define __FM25V10_H
#include <stdio.h>
#include <string.h>
#include "stdlib.h"
#include "stm32l0xx_hal.h"



//  EEPROM   存储存储区域使用规划
#define     SHK_ADDR_INIT            0x00000000  // 256 Bytes
#define     SHK_ADDR_SYSCONFI        SHK_ADDR_INIT+0x100     // 256Bytes    开始时间(6Bytes)结束时间(6Bytes)震动阈值(3bytes)  THRES_ACT THRESH_INACT TIME_INACT
#define     SHK_ADDR_SAVE_STATUS      SHK_ADDR_SYSCONFI+0x100 // 256 Bytes    当前存储的条数3(bytes)   是否完全存储满澹2 bytes
#define     SHK_ADDR_SAVE_USRINFO     SHK_ADDR_SAVE_STATUS+0x100   //256   字节用户自定义信息  
// 单片 eeprom 存储空间为256K bytes      第一片预留出32 K 作为其他预留应用，
// 32K 以后存储相关数据， 每个event    6 个BCD 数据开头，后边是18bytes 一组的数据，
// 每组数据的采样间隔是400HZ ,即2.5ms 采样一次
//暂定每个时间的 存储区域大小暂定为   16K  bytes     ( 16*1024-6)/18/400 = 2.27 秒的数据

#define    SHK_ADDR_DATA_START          0x8000               //  32K Bytes  的起始地址
#define    SHK_EVENT_FILE_SIZE          0x4000               //    16 K Bytes

// Note :    第一片中只能存储14 个Event     第二片能存储16个Event      
#define     SHK_EVENT_MAXNUM         30      //    14+16  
#define     SHK_PAGE_SIZE                256

//-------------------------------------------------------------------------------------------------

#define SPIx                             SPI1
#define SPIx_CLK_ENABLE()                __HAL_RCC_SPI1_CLK_ENABLE()
#define SPIx_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPIx_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPIx_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define SPIx_FORCE_RESET()               __HAL_RCC_SPI1_FORCE_RESET()
#define SPIx_RELEASE_RESET()             __HAL_RCC_SPI1_RELEASE_RESET()

/* Definition for SPIx Pins */


#define WIP_BIT 0x01

#define WREN  0x06
#define WRDI  0x04
#define RDSR  0x05
#define WRSR  0x01
#define READ  0x03
#define WRITE 0x02
#define WRID  0x82
#define RDID  0x83
#define RDLS  0x83
#define LIDS  0x82

#define eeprom_POWER_ON()   HAL_GPIO_WritePin(SHK_PB4_EEPowr_GPIO_Port,SHK_PB4_EEPowr_Pin,GPIO_PIN_SET)
#define eeprom_POWER_OFF()  HAL_GPIO_WritePin(SHK_PB4_EEPowr_GPIO_Port,SHK_PB4_EEPowr_Pin,GPIO_PIN_RESET)


#define eeprom_cs1_low()  HAL_GPIO_WritePin(SHK_CS1_GPIO_Port,SHK_CS1_Pin,GPIO_PIN_RESET)
#define eeprom_cs1_high() HAL_GPIO_WritePin(SHK_CS1_GPIO_Port,SHK_CS1_Pin,GPIO_PIN_SET)
#define eeprom_cs1_wpHIGH() HAL_GPIO_WritePin(SHK_WP1_GPIO_Port,SHK_WP1_Pin,GPIO_PIN_SET)
#define eeprom_cs1_wpLOW() HAL_GPIO_WritePin(SHK_WP1_GPIO_Port,SHK_WP1_Pin,GPIO_PIN_RESET)


#define eeprom_cs2_low()  HAL_GPIO_WritePin(SHK_CS2_GPIO_Port,SHK_CS2_Pin,GPIO_PIN_RESET)
#define eeprom_cs2_high() HAL_GPIO_WritePin(SHK_CS2_GPIO_Port,SHK_CS2_Pin,GPIO_PIN_SET)
#define eeprom_cs2_wpHIGH() HAL_GPIO_WritePin(SHK_WP2_GPIO_Port,SHK_WP2_Pin,GPIO_PIN_SET)
#define eeprom_cs2_wpLOW() HAL_GPIO_WritePin(SHK_WP2_GPIO_Port,SHK_WP2_Pin,GPIO_PIN_RESET)




#define SPIXTIMEOUT		1000
#define EMPTYDATA		0x00
#define ONE_EXTRENAL_EEPROM	0x3FFFF
#define TWO_EXTRENAL_EEPROM 0x7FFFF
#define FIX_DATA_LENGTH		32
#define ONLYPAGE_WRITEREAD	256

 

void eeprom_wren(uint8_t chip);
void eeprom_writemultipledata(uint32_t addr ,uint8_t *pdata, uint32_t size);
uint8_t eeprom_read_byte(uint32_t addr,uint8_t chip);
uint8_t eeprom_read_reg(uint8_t chip);
uint8_t eeprom_RegisterByte_Read(uint8_t chip,uint8_t reg_Name);
void  eeprom_RegisterByte_WR(uint8_t chip,uint8_t reg_Name,uint8_t reg_value);



void eeprom_multipleread(uint32_t addr,uint8_t* pbuffer, uint32_t num); 

void eeprom_wrdisable(uint8_t chip);
void eeprom_write_byte(uint32_t addr,uint8_t *data,uint8_t chip);
HAL_StatusTypeDef eeprom_waitforwriteend(uint8_t chip);





#endif 

