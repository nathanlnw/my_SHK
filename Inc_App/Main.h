#ifndef _MAIN
#define _MAIN

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "stm32l0xx_hal.h"
#include <stdio.h>
#include <string.h>
#include "stdlib.h"
#include "mxconstants.h"
#include "ShakeSensor_app.h"
#include "adxl375.h"
#include "FM25V10.h"
#include "Hmc5883l.h"
#include "rtc8564.h"
#include "L3GD20H.h"
#include "Application.h"

#define   DEBUG_SHK

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;


#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


#define     SYS_ID            0x0758 
#define     VERSION_INFO      0x0101      // MSB  : HW   LSB: SW




#define  ADXL375_INIT_THRESHOD     0x80      // 2X780mg
#define  ADXL375_INIT_X_OFFSET     0x00      //  196mg/LSB    1  TEST
#define  ADXL375_INIT_Y_OFFSET     0x00     //  196mg/LSB
#define  ADXL375_INIT_Z_OFFSET     0x00     //  196mg/LSB
#define  ADXL375_INIT_DUR          0x01     //  625us/LSB  

#define  ADXL375_INIT_FREQ    0x0E   // 0x0E   1600HZ



#define  THRESH_ACT_Defaut       0x14    //保存检测活动阀值; (780mg/LSB)   15.6g   14 
#define  THRESH_INACT_Defaut     0x06    //保存检测静止阀值; (780mg/LSB)   7.8g   A
#define  TIME_INACT_Defaut       0x10    //检测活动时间阀值; (1s/LSB)       16s     10       




//LED  Related

//  Blue
#define  LED1_ON     HAL_GPIO_WritePin(SHK_LED1_GPIO_Port, SHK_LED1_Pin, GPIO_PIN_RESET)
#define  LED1_OFF    HAL_GPIO_WritePin(SHK_LED1_GPIO_Port, SHK_LED1_Pin, GPIO_PIN_SET)

// Red
#define  LED2_OFF     HAL_GPIO_WritePin(SHK_LED2_GPIO_Port, SHK_LED2_Pin, GPIO_PIN_RESET)
#define  LED2_ON    HAL_GPIO_WritePin(SHK_LED2_GPIO_Port, SHK_LED2_Pin, GPIO_PIN_SET)

// Green
#define  LED3_ON     HAL_GPIO_WritePin(SHK_LED3_GPIO_Port, SHK_LED3_Pin, GPIO_PIN_RESET)
#define  LED3_OFF    HAL_GPIO_WritePin(SHK_LED3_GPIO_Port, SHK_LED3_Pin, GPIO_PIN_SET)

// Yellow
#define  LED4_ON     HAL_GPIO_WritePin(SHK_LED4_GPIO_Port, SHK_LED4_Pin, GPIO_PIN_RESET)   
#define  LED4_OFF    HAL_GPIO_WritePin(SHK_LED4_GPIO_Port, SHK_LED4_Pin, GPIO_PIN_SET)





typedef struct  _SYS_CONFIG
{
   u8   ADXL375_ShakeThreshod;    //   震动传感器的  阈值
   u8   StartWork_RTC[6];   //  开始工作时间      
   u8   EndWork_RTC[6];   //  失效工作时间      
   u16  SYStem_ID;    // 版本信息      Default:  0x0712
   u16  Version_Info; //  MSB : hw version  LSB: software version    Default: 0101
   u8   Device_ID[12]; //    BCD    stm32  ID  24 个数字 12 个字节
   u8   SaveWr;   // 当前存储记录
   u8   SaveFull_state;// 是否存储满过
   u8   Threshod_ADXL375[6];//  THRESHOD ACT , THRESH_INACT , TIME_INACT
   u8   User_info[256]; //  用户自定义信息
 }SYS_CONFIG;

typedef struct _PRO_STRUCT
{
   u8   ACCEL_ADX375_ACCEL_6BYTES[6];//  LSB   XL  XH YL YH ZL ZH   
   u8   COMPASS_HMC_6BYTES[6];//  LSB   XL  XH YL YH ZL ZH   
   u8   GYRO_L3G_6BYTES[6];//  LSB   XL  XH YL YH ZL ZH
   u16  ACCEL_X_vaule;// ACCEL
   u16  ACCEL_Y_vaule;
   u16  ACCEL_Z_vaule;
   u16  COMPASS_X_vaule;   //COMPASS
   u16  COMPASS_Y_vaule;
   u16  COMPASS_Z_vaule;
   u16  GYRO_X_vaule; //GYRO
   u16  GYRO_Y_vaule;
   u16  GYRO_Z_vaule;
   u16  Voltage_ADC_Value; // 电压监测的ADC 数值
   u16  Voltage_V;   //   电压数值   单位0.01 V 
   u8   Power_Stage;  //  电源状态    
                          /*
						             参考电压是3.1V                 
						             充满  4.2 V                  
			                                         stage 4 :  4.1  ~ 4.2    100%
			                                         stage 3:  4.0~4.1        75%
			                                         stage 2:  4.0~3.8      50%
			                                         stage 1:  3.8~3.7      25%
			                                         stage 0    <3.7          no power
						             馈电  3.7 V  
						     */

}PRO_STRUCT;

typedef struct  _WRK_STATE
{
   u32  timer_coutner; // 采集计数器
   u32  Page_in_offset;//  发送一次512 bytes  填充的包数 18bytes
   u16  current_packet;
   u16  total_packet;    
   u16  info_len;
   u8   ENABLE_Triger;  //  表示中断触发开始   default:0     enable:  1
   u8   Enable_Again;   //  在采集的过程中，是否又发生过新的触发条件   0: 没有发生过  1 :  发生过了
   u32  Event_WriteADDR;    //  Event  写入地址

}WRK_STATE;

//-------------------------------------------
extern u8     Uart1_RxBuf[100];
extern u16    Uart1_RxBuf_wr;
extern u8     U1_rx_sub1[100];
extern u8     U1_rx_sub1_wr;

extern u8     U1_TxBuf[600];
extern u16    U1_TxBuf_wr;
extern u8     SHK_SampleData_Buff[600];
extern u16    SHK_SampleData_Buff_wr; 

extern 	u8	   SHK_SaveData_Buff[520];	 // 存储缓存
extern 	u16    SHK_SaveData_Buff_wr;   


extern PRO_STRUCT   Project_SHK;
extern SYS_CONFIG   sys_config;
extern WRK_STATE    SHK_WRK_STATE;
extern uint8_t aRxBuffer[1];
extern u8   Uart1Ready_R;
extern u8	Seven_one_rx; 


//-----------------------------------------
extern ADC_HandleTypeDef hadc;
extern DMA_HandleTypeDef hdma_adc;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

extern UART_HandleTypeDef hlpuart1;
extern DMA_HandleTypeDef hdma_lpuart1_rx;

extern RTC_HandleTypeDef hrtc;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;


extern WWDG_HandleTypeDef hwwdg;
extern u32    main_counter;


extern void MX_GPIO_Init(void);
extern void MX_DMA_Init(void);
extern void MX_ADC_Init(void);
extern void MX_I2C1_Init(void);
extern void MX_I2C2_Init(void);
extern void MX_SPI1_Init(void);
extern void MX_TIM2_Init(void);
extern void MX_WWDG_Init(void);
extern void MX_LPUART1_UART_Init(void);
extern void MX_SPI2_Init(void);
extern void MX_SPI2_Init_ADXL375(void);
extern void MX_SPI2_Init_L3GD20H(void);

extern void MX_RTC_Init(void);
extern void System_config_init(void);


extern void  Output_string(u8 * instr);
extern void  Output_Data(u8 * instr,u16 len); 
extern void delay_us_1(u16 j);
extern void delay_ms_1(u16 j);
extern void U1_Tx_OneByte(uint8_t ch);
extern void TIM2_Service(void);

#endif
