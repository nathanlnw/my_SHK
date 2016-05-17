#ifndef __RTC8564_H
#define __RTC8564_H
#include <stdio.h>
#include <string.h>
#include "stdlib.h"
#include "stm32l0xx_hal.h"

#define RTC_WR			0xA2
#define RTC_RE			0xA3


#define	reg_ctr1			0x00
#define	reg_ctr2			0x01
#define reg_sec				0x02
#define reg_min				0x03
#define	reg_hour			0x04
#define	reg_day				0x05
#define reg_week			0x06
#define reg_month			0x07
#define	reg_year			0x08
#define	reg_alarm_min		0x09
#define reg_alarm_hour		0x0A
#define reg_alarm_day		0x0B
#define reg_alarm_week		0x0C

#define reg_clk				0x0D
#define reg_timer_ctr		0x0E
#define reg_timer			0x0F

typedef struct RTC_TIME_
{
    uint8_t     BCD_6_Bytes[6]; 
	uint8_t		year;
	uint8_t		month;
	uint8_t		day;
	uint8_t		hour;
	uint8_t     min;
	uint8_t		sec;
	uint8_t		week;
}RTCTIME;
extern RTCTIME rtc_current;
unsigned char bin2bcd(unsigned val);
unsigned bcd2bin(unsigned char val);
void RTC8564_Get(void);
HAL_StatusTypeDef RTC8564_Set(RTCTIME time);
void RTC8564_Init(void);

#endif




