#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal.h"
#include "stdlib.h"
#include "application.h"
#include "adxl375.h"
#include "FM25V10.h"
#include "stm32l0xx_hal_spi.h"
#include "rtc8564.h"
#include "ShakeSensor_app.h"
#include "Main.h"


#define FULLEASE	0x800
#define NO_BLOCK_T	0
#define BLOCK_T		10

static FLASH_EraseInitTypeDef EraseInitStruct;
ADC_HandleTypeDef             AdcHandle;
ADC_ChannelConfTypeDef        sconfig;

UART_HandleTypeDef UartHandle;
I2C_HandleTypeDef I2CxHandle;
I2C_HandleTypeDef I2C2Handle;

SPI_HandleTypeDef SpiHandle;


uint8_t rxbyte=0;
HMC5883L_TYPE hmcdata;
uint8_t g_wakup =0;
static uint32_t deviceid =0;

struct tm sys_time;
S_DEVICE sdevice = DEVICE_POWER_OFF;
//KEY_STATUS keystatus =KEY_STATUS_IDLE;
KEY_STATUS powerkeystatus=KEY_STATUS_IDLE;
KEY_STATUS funkeystatus=KEY_STATUS_IDLE;



static uint32_t delay_t =BLOCK_T;


/*****************************************************************************
 Prototype    : key_PB12_init
 Description  : 初始化中断唤醒的引脚目前PB12唤醒睡眠模式
 Input        : void  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2016/2/20
    Author       : wxg
    Modification : Created function

*****************************************************************************/
void key_PB12_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE(); 
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void interrupt_pb_init(GPIO_TypeDef  *GPIOx,uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE(); 
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/*****************************************************************************
 Prototype    : key_PB3_init
 Description  : 按键K1中断配置
 Input        : void  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2016/2/22
    Author       : wxg
    Modification : Created function

*****************************************************************************/
void key_PB3_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE(); 
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_NVIC_SetPriority(EXTI2_3_IRQn , 3, 0);
	HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
}

/*****************************************************************************
 Prototype    : LED_LIGHT
 Description  : 中断回调函数中调用，点亮led
 Input        : void  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2016/2/20
    Author       : wxg
    Modification : Created function

*****************************************************************************/
void LED_LIGHT(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);
	HAL_GPIO_TogglePin(GPIOx,GPIO_Pin);
	
}

/*****************************************************************************
 Prototype    : uart_configure
 Description  : 配置串口的参数
 Input        : void  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2016/2/21
    Author       : wxg
    Modification : Created function

*****************************************************************************/
void uart_configure(void)
{
	//UART_HandleTypeDef UartHandle;
	UartHandle.Instance        = LPUART1;
	UartHandle.Init.BaudRate   = 115200;
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits   = UART_STOPBITS_1;
	UartHandle.Init.Parity     = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode       = UART_MODE_TX_RX;
	if(HAL_UART_Init(&UartHandle) != HAL_OK)
	{
		;//SEGGER_RTT_printf(0,"ERROR_LPUART!\n");
	}
	if(HAL_UART_Receive_IT(&UartHandle, &rxbyte, 1)!=HAL_OK)
	{
		;//SEGGER_RTT_printf(0,"ERROR_LPUART_RX_IT!\n");
	}
}
/*****************************************************************************
 Prototype    : uart_sendbyte
 Description  : 串口中断发送数据
 Input        : uint8_t *txbuffer  
                uint16_t txsize    
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2016/2/21
    Author       : wxg
    Modification : Created function

*****************************************************************************/
void uart_senddata(uint8_t *txbuffer,uint16_t txsize)
{
	/*
	if(HAL_UART_Transmit_IT(&UartHandle, txbuffer, txsize)!= HAL_OK)
	{
		;//SEGGER_RTT_printf(0,"ERROR_LPUART_TX!\n");
	}
	*/
	if(HAL_UART_Transmit(&UartHandle, txbuffer, txsize, 50)!=HAL_OK)
	{
		;//SEGGER_RTT_printf(0,"ERROR_LPUART_TX!\n");
	}
}

/*****************************************************************************
 Prototype    : uart_rcvdata
 Description  : 串口中断接受数据
 Input        : uint8_t *rxbuffer  
                uint16_t rxsize    
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2016/2/21
    Author       : wxg
    Modification : Created function

*****************************************************************************/
void uart_rcvdata(uint8_t *rxbuffer,uint16_t rxsize)
{
	if(HAL_UART_Receive_IT(&UartHandle, rxbuffer, rxsize) != HAL_OK)
	{
		;//SEGGER_RTT_printf(0,"ERROR_LPUART_RX!\n");
		
	}
	
}

/*****************************************************************************
 Prototype    : power_key_scan
 Description  : power_key按键扫描判断一个按键是长按还是短按
 Input        : GPIO_TypeDef* GPIOx     
                uint16_t GPIO_Pin       
                GPIO_PinState PinState  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2016/3/8
    Author       : wxg
    Modification : Created function

*****************************************************************************/
KEY_STATUS power_key_scan(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
	static uint8_t keycount =0;
	static uint8_t key_press =0;
	static uint32_t keydownstart_time =0;
	if(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin)==PinState)
	{
		keycount++;
		////SEGGER_RTT_printf(0,"keycount = %d\n",keycount);
		if(keycount>=40)
		{
			if(key_press==0)
			{
				////SEGGER_RTT_printf(0,"keycount = %d\n",keycount);
				keycount =0;
				key_press = KEY_DOWN_FIRST;
			}
			else
			{
				keycount =0;
			}
		}
		if(key_press == KEY_DOWN_FIRST)
		{
			//LED_LIGHT(GPIOC,GPIO_PIN_12);
			
			key_press =KEY_DOWN_SHORT;
			//SEGGER_RTT_printf(0,"g_wakup=%d\n",g_wakup);
		}
		if(key_press ==KEY_DOWN_SHORT)
		{
			keydownstart_time++;
			
			if(keydownstart_time > MAX_DOWN_TIME)
			{
				key_press =KEY_DOWN_LONG;
			}
			////SEGGER_RTT_printf(0,"keydownstart_time=%d,,%d\n",keydownstart_time,MAX_DOWN_TIME);
		}
	}
	else
	{
		////SEGGER_RTT_printf(0,"key_lift \n");
		
		if(key_press == KEY_DOWN_LONG)
		{
			//power off
			//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);
			g_wakup =0;
			powerkeystatus = KEY_FUNCTION_DOWN_LONG;
			//SEGGER_RTT_printf(0,"press_long\n");
			
		}
		else if(key_press == KEY_DOWN_SHORT)
		{
			g_wakup =0;
			powerkeystatus = KEY_FUNCTION_DOWN_SHORT;
			//SEGGER_RTT_printf(0,"press_short\n");
		}
		/*
		else
		{
			powerkeystatus =  KEY_STATUS_IDLE;
		}*/
		keycount =0;
		key_press =0;
		keydownstart_time=0;
		return powerkeystatus;
	}
	return powerkeystatus;
}
/*****************************************************************************
 Prototype    : fun_key_scan
 Description  : 功能键判断
 Input        : GPIO_TypeDef* GPIOx     
                uint16_t GPIO_Pin       
                GPIO_PinState PinState  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2016/3/8
    Author       : wxg
    Modification : Created function

*****************************************************************************/
KEY_STATUS fun_key_scan(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
	static uint8_t keycount =0;
	static uint8_t key_press =0;
	static uint32_t keydownstart_time =0;
	if(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin)==PinState)
	{
		keycount++;
		////SEGGER_RTT_printf(0,"keycount = %d\n",keycount);
		if(keycount>=40)
		{
			if(key_press==0)
			{
				////SEGGER_RTT_printf(0,"keycount = %d\n",keycount);
				keycount =0;
				key_press = KEY_DOWN_FIRST;
			}
			else
			{
				keycount =0;
			}
		}
		if(key_press == KEY_DOWN_FIRST)
		{
			//LED_LIGHT(GPIOC,GPIO_PIN_12);
			
			key_press =KEY_DOWN_SHORT;
			//SEGGER_RTT_printf(0,"g_wakup=%d\n",g_wakup);
		}
		if(key_press ==KEY_DOWN_SHORT)
		{
			keydownstart_time++;
			
			if(keydownstart_time > MAX_DOWN_TIME)
			{
				key_press =KEY_DOWN_LONG;
			}
			//SEGGER_RTT_printf(0,"keydownstart_time=%d,,%d\n",keydownstart_time,MAX_DOWN_TIME);
		}
	}
	else
	{
		//SEGGER_RTT_printf(0,"key_lift \n");
		
		if(key_press == KEY_DOWN_LONG)
		{
			g_wakup =0;
			//power off
			//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);
			funkeystatus = KEY_FUNCTION_DOWN_LONG;
			//SEGGER_RTT_printf(0,"press_long\n");
			
		}
		else if(key_press == KEY_DOWN_SHORT)
		{
			g_wakup =0;
			funkeystatus = KEY_FUNCTION_DOWN_SHORT;
			//SEGGER_RTT_printf(0,"press_short\n");
		}
		/*
		else
		{
			keystatus =  KEY_STATUS_IDLE;
		}
		*/
		keycount =0;
		key_press =0;
		keydownstart_time=0;
		////SEGGER_RTT_printf(0,"funkeystatus3=%d\n",funkeystatus);
		return funkeystatus;
	}
	return funkeystatus;
}

unsigned bcd2bin(unsigned char val)
{
	return (val & 0x0f) + (val >> 4) * 10;
}

unsigned char bin2bcd(unsigned val)
{
	return ((val / 10) << 4) + val % 10;
}

HAL_StatusTypeDef detect_it(uint32_t value)
{
	static uint32_t starttick =0;
	if(g_wakup==0)
	{
		starttick = HAL_GetTick();
		g_wakup =1;
	}
	if((g_wakup==1)&&(starttick==0))
	{
		return HAL_BUSY;
	}
	else
	{
		if(((HAL_GetTick()-starttick)%1000)>500)
		{
			uart_senddata("123",3);
		}
		if((HAL_GetTick()-starttick) > value)
		{
			starttick =0;
			return HAL_OK;
		}
		else
		{
			return HAL_BUSY;
		}
	}
}
/****************************************内部eeprom的操作***************************/
/*****************************************************************************
 Prototype    : internal_eeprom_ease
 Description  : 内部EEPROM的擦出，直接时整片的擦出
 Input        : void  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2016/3/2
    Author       : wxg
    Modification : Created function

*****************************************************************************/
HAL_StatusTypeDef internal_eeprom_ease(void)
{
	uint32_t nbofpage =0;
	uint32_t pageerror =0;
	nbofpage =  FULLEASE >> 7;

  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = EEPROM_USER_START_ADDR;
  EraseInitStruct.NbPages = nbofpage;
  
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &pageerror) != HAL_OK)
  { 
    return HAL_ERROR;
  }
	return HAL_OK;
}

/*****************************************************************************
 Prototype    : internal_eeprom_write
 Description  : 写内部的eeprom，库函数默认一次写四个字节
 Input        : uint32_t addr  
                uint8_t *data  
                uint32_t len   
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2016/3/2
    Author       : wxg
    Modification : Created function

*****************************************************************************/
HAL_StatusTypeDef internal_eeprom_write(uint32_t addr,uint8_t *data,uint32_t len)
{
	uint32_t wr_data = 0;
	uint32_t once_wr_end_addr =0;
	uint8_t shift =0;
	uint8_t *psr =NULL;
	uint32_t original_len =0;
	if((addr+len) > FULLEASE)
	{
		return HAL_ERROR;
	}
	original_len = len;
	//取大于4的最小整数倍
	if(( len % 4 ) != 0)
	{
		len =((len/4)+1)*4;
	}
	once_wr_end_addr = addr+len+EEPROM_USER_START_ADDR;
	psr=malloc(len);
	if(psr==NULL)
	{
		return HAL_ERROR;
	}
	memset(psr,0xFF,len);
	memcpy(psr,data,original_len);
	HAL_FLASH_Unlock();
	while(addr+EEPROM_USER_START_ADDR < once_wr_end_addr)
	{
		////SEGGER_RTT_printf(0,"%x,%x,%x,%x\n",psr[shift*4],psr[shift*4+1],psr[shift*4+2],psr[shift*4+3] );
		wr_data = psr[shift*4]<<24 | psr[shift*4+1]<<16 |psr[shift*4+2]<<8 |psr[shift*4+3];
		////SEGGER_RTT_printf(0,"%x,%x,%x,%x\n",psr[shift*4]<<24,psr[shift*4+1]<<16,psr[shift*4+2]<<8,psr[shift*4+3] );
		//SEGGER_RTT_printf(0,"storagedata = %x\n",wr_data );
	    if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr+EEPROM_USER_START_ADDR,wr_data) == HAL_OK)
	    {
	      addr = addr + 4;
		  shift = shift+1;
	    }
	    else
	    { 
	      free(psr);
	      return HAL_ERROR;
	    }
	}
	HAL_FLASH_Lock();
	free(psr);
	addr =once_wr_end_addr;
	return HAL_OK;
}

/*****************************************************************************
 Prototype    : internal_eeprom_read
 Description  : 读内部EEPROM的数据
 Input        : uint32_t addr  
                uint8_t *data  
                uint32_t len   
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2016/3/2
    Author       : wxg
    Modification : Created function

*****************************************************************************/
HAL_StatusTypeDef internal_eeprom_read(uint32_t addr,uint8_t *data,uint32_t len)
{

	uint32_t re_addr =0;
	uint32_t data32 =0;
	uint16_t shift =0;
	re_addr = addr+len+EEPROM_USER_START_ADDR;
	if((addr+len) > FULLEASE)
	{
		return HAL_ERROR;
	}
	while (addr+EEPROM_USER_START_ADDR < re_addr)
	{
		data32 = *(__IO uint32_t*)(addr+EEPROM_USER_START_ADDR);
		////SEGGER_RTT_printf(0,"data32 = %x\n",data32 );
		addr = addr + 4;
		data[shift*4] = (data32>>24)&0xFF;
		data[shift*4+1] = (data32>>16)&0xFF;
		data[shift*4+2] = (data32>>8)&0xFF;
		data[shift*4+3] = data32&0xFF;
		shift =shift+1;
	} 
	addr =re_addr;
	return HAL_OK;
}
/***************************ADC检测电池电量********************************/

/*****************************************************************************
 Prototype    : adc_configure
 Description  : ADC的初始化配置
 Input        : void  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2016/3/2
    Author       : wxg
    Modification : Created function

*****************************************************************************/
void adc_configure(void)
{
	AdcHandle.Instance = ADC1;

	AdcHandle.Init.OversamplingMode      = DISABLE;

	AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV1;
	AdcHandle.Init.LowPowerAutoPowerOff  = DISABLE;
	AdcHandle.Init.LowPowerFrequencyMode = ENABLE;
	AdcHandle.Init.LowPowerAutoWait      = DISABLE;

	AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;
	AdcHandle.Init.SamplingTime          = ADC_SAMPLETIME_7CYCLES_5;
	AdcHandle.Init.ScanConvMode          = ADC_SCAN_DIRECTION_FORWARD;
	AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
	AdcHandle.Init.ContinuousConvMode    = ENABLE;
	AdcHandle.Init.DiscontinuousConvMode = DISABLE;
	AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
	AdcHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
	AdcHandle.Init.DMAContinuousRequests = DISABLE;
	if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
	{
		;//SEGGER_RTT_printf(0,"ADC_INIT_ERROR\n");
	}

	if (HAL_ADCEx_Calibration_Start(&AdcHandle, ADC_SINGLE_ENDED) != HAL_OK)
	{
		;//SEGGER_RTT_printf(0,"ADC_Calibration_ERROR\n");
	}
	sconfig.Channel = ADC_CHANNEL_2;    
	if (HAL_ADC_ConfigChannel(&AdcHandle, &sconfig) != HAL_OK)
	{
		;//SEGGER_RTT_printf(0,"ADC_CHANNEL_ERROR\n");
	}
	if(HAL_ADC_Start(&AdcHandle) != HAL_OK)
	{
		;//SEGGER_RTT_printf(0,"ADC_START_ERROR\n");
	}
}

/*****************************************************************************
 Prototype    : adcvalue_read
 Description  : ADC值的读取，返回ADC的值
 Input        : void  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2016/3/2
    Author       : wxg
    Modification : Created function

*****************************************************************************/
uint32_t adcvalue_read(void)
{
	//uint32_t dec_value=0;
	//float realvalue =0;
	HAL_ADC_PollForConversion(&AdcHandle, 10);
    if ((HAL_ADC_GetState(&AdcHandle) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC)
    {
    	//dec_value = HAL_ADC_GetValue(&AdcHandle);  //  ADC 数值读取
		//realvalue = (dec_value)*(2.3) /4096;
      return HAL_ADC_GetValue(&AdcHandle);
    }
		return 1;
}

/*****************************************************************************
 Prototype    : get_chipid
 Description  : 获取芯片的唯一的ID
 Input        : void  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2016/3/3
    Author       : wxg
    Modification : Created function

*****************************************************************************/
uint32_t get_chipid(void)
{
	uint32_t mcu_id =0;
	mcu_id =  *(__IO uint32_t *)(0X1FF80064);
	deviceid = mcu_id;
	return mcu_id;
}
/*****************************************************************************
 Prototype    : data_decode
 Description  : 数据解码：接受PC机数据并转义
 Input        : uint8_t *str  
                uint16_t len  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2016/3/3
    Author       : wxg
    Modification : Created function

*****************************************************************************/
uint8_t data_decode(uint8_t *str,uint16_t len)
{
	
	uint16_t i=0;
	uint16_t senddata_decode_len = 0;
	for(i=0;i<len;i++)
	{
		if((str[i]==0x7d)&&(str[i+1]==0x02))
		{
			str[senddata_decode_len]=0x7e;
			i++;
		}
		else
		if((str[i]==0x7d)&&(str[i+1]==0x01))
		{
			str[senddata_decode_len]=0x7d;
			i++;
		}
		else  
		{
			str[senddata_decode_len]=str[i];  
		}
		senddata_decode_len++;
	}	
	return senddata_decode_len;
}
/*****************************************************************************
 Prototype    : data_code
 Description  : 将原始数据中存在7E的数据进行转义
 Input        : u8 *dest    
                u8 *src     
                u16 srclen  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2016/3/3
    Author       : wxg
    Modification : Created function

*****************************************************************************/
uint16_t data_code(uint8_t *dest,uint8_t *src, uint8_t srclen)
{
	uint16_t lencnt=0,destcnt=0;

	for(lencnt=0;lencnt<srclen;lencnt++)
	{
		if(src[lencnt]==0x7e)  
		{
			dest[destcnt++]=0x7d;
			dest[destcnt++]=0x02;
		}
		else
		{
			if(src[lencnt]==0x7d)    	
			{
				dest[destcnt++]=0x7d;
				dest[destcnt++]=0x01; 
			}
			else
			{
				dest[destcnt++]=src[lencnt];
			}
		}
	}

	return destcnt;

}
/*****************************************************************************
 Prototype    : data_package_send
 Description  : 将数据按照打包好的格式发送出去
 Input        : PARAMCMD cmd  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2016/3/4
    Author       : wxg
    Modification : Created function

*****************************************************************************/
HAL_StatusTypeDef data_package_send(PARAMCMD cmd,uint8_t* pdata,uint8_t dlen)
{
	uint8_t dest[100]={0};
	uint8_t src[100] ={0};
	uint8_t devicedata[32]={0};
	uint8_t len =0;
	dest[0]=0x7E;
	
	switch(cmd)
	{
		case PARAM_TIME:
			dest[1] ='t';
			
			internal_eeprom_read(PARAM_TIME_ADDR,src,PARAM_FIX_LENGTH);
			if(src[0] >=32)
			{
				goto dataerror;
			}
			break;
		case PARAM_DEVICE:
			internal_eeprom_read(PARAM_DEVICE_INVALID_ADDR,devicedata,PARAM_FIX_LENGTH);
			dest[1] ='d';
			memcpy(src+1,&deviceid,4);
			memcpy(src+5,devicedata+1,devicedata[0]);
			src[0] = devicedata[0]+4;
			if(src[0] >=32)
			{
				goto dataerror;
			}
			break;
		case PARAM_ADXL:
			internal_eeprom_read(PARAM_ADXL_ADDR,src,PARAM_FIX_LENGTH);
			dest[1] ='p';
			if(src[0] >=32)
			{
				goto dataerror;
			}
			break;
		case PARAM_POWER:
			src[0] =1;
			dest[1]='r';
			src[1] = adcvalue_read();
			break;	
		case PARAM_DATA:
			//从eeprom读处数据添加代码
			dest[1] ='s'; 
			memcpy(dest+2,&deviceid,4);
			len = data_code(dest+6,pdata+1,dlen);
			dest[len+6] =0x7E;
			uart_senddata(dest,len+6+1);
			return HAL_OK;
			
	}
	len = data_code(dest+2,src+1,src[0]);
	dest[len+2] =0x7E;
	uart_senddata(dest,len+3);
	return HAL_OK;
dataerror:
	return HAL_ERROR;
	
}

/*****************************************************************************
 Prototype    : pc_cmd_parsing
 Description  : 上位机命令分析并执行相应的操作
 Input        : uint8_t *psr  
                uint16_t len  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2016/3/4
    Author       : wxg
    Modification : Created function
tmp存储的结构是第一个是长度，后面是数据
*****************************************************************************/
HAL_StatusTypeDef pc_cmd_parsing(uint8_t *psr,uint16_t len)
{
	uint8_t str[64] ={0};
	uint8_t i=0;
	uint8_t tmp[50] ={0xFF};
	memcpy(str,psr,len);
	if(psr==NULL)
	{
		return HAL_ERROR;
	}
	//改变str结构数据转义
	tmp[0] = data_decode(str+2,len-3);
	//tmp[0]=len-2;
	memcpy(tmp+1,str+2,tmp[0]);
	//打印包括长度在内的接受命令
	for(i=0 ;i< tmp[0]+1;i++)
	{
		;//SEGGER_RTT_printf(0,"test=%x\n",tmp[i]);
	}
	if(psr[1]=='T')
	{
		if(tmp[0] > 3)
		{
			rtc_current.year=tmp[1];
			rtc_current.month=tmp[2];
			rtc_current.day =tmp[3];
			rtc_current.hour =tmp[4];
			rtc_current.min =tmp[5];
			rtc_current.sec =tmp[6];
			RTC8564_Set(rtc_current);
			internal_eeprom_write(PARAM_TIME_ADDR,tmp,tmp[0]+1);
		}
		else
			data_package_send(PARAM_TIME,NULL,0);
	}
	else if(psr[1]=='D')
	{
		if(tmp[0] > 3)
		{
			
			internal_eeprom_write(PARAM_DEVICE_INVALID_ADDR,tmp,tmp[0]+1);
		}
		else
		{
			data_package_send(PARAM_DEVICE,NULL,0);
		}
	}
	else if(psr[1]=='S')
	{
		//置上标志位开始读取外部eeprom shuju 直到读完eeprom区域
		sensor_data_report();
	}
	else if(psr[1]=='P')
	{
		if(tmp[0] > 1)
			internal_eeprom_write(PARAM_ADXL_ADDR,tmp,tmp[0]+1);
		else
			data_package_send(PARAM_ADXL,NULL,0);
	}
	else if(psr[1]=='R')
	{
		data_package_send(PARAM_POWER,NULL,0);
	}
	return HAL_OK;
}
/*****************************************************************************
 Prototype    : pc_data_parsing
 Description  : 上位机数据分析，看中断接收到的数据是否符合协议
 Input        : uint8_t*str   
                uint16_t len  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2016/3/4
    Author       : wxg
    Modification : Created function

*****************************************************************************/
void pc_data_parsing(uint8_t*str,uint16_t len)
{
	uint16_t i=0,seven_e=0,j=0;
	uint16_t seven_eplace[50]={0};
	uint16_t repeat_counter =0;
	uint8_t tmpbuf[50]={0};
	uint8_t reallen =0;
	for(i=0;i<len;i++)
	{
		if(str[i]==0x7E)
		{
			seven_e++; 
			seven_eplace[j++]=i;//存储当前7E的位置
		}
	}
	repeat_counter=(seven_e/2);
	if((repeat_counter)&&(repeat_counter<=25))  // 大于 0  且 小于 10
	{  
		for(i=0;i<repeat_counter;i++)
		{ 
			//一次拷贝一包7e的内容 连包分多次拷贝
			reallen = seven_eplace[(i*2)+1]-seven_eplace[i*2]+1;
			memcpy(tmpbuf,str+seven_eplace[i*2],reallen);
			
			pc_cmd_parsing(tmpbuf,reallen);   
		}
	}
}

/*******************************Data storage application layer program*********************/
/*****************************************************************************
 Prototype    : get_eeprom_write_addr
 Description  : 上电首先遍历得出上一次写的地址从上一次写的地址里面等到最大时
                间就是要写的地址
 Input        : void  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2016/3/7
    Author       : wxg
    Modification : Created function

*****************************************************************************/
HAL_StatusTypeDef get_eeprom_write_addr(void)
{
	storagedata.rdaddr =0;
	storagedata.wraddr =0;
	uint32_t w_real_addr =0;
	uint32_t r_real_addr =0;
	uint8_t tmp[40]= {0};
	uint32_t utc_time =0,i = 0;
	uint32_t cmp_time =0;
	uint8_t s_chip =1;
	while(storagedata.rdaddr < TWO_EXTRENAL_EEPROM)
	{
		if(storagedata.rdaddr > ONE_EXTRENAL_EEPROM)
		{
			r_real_addr =storagedata.rdaddr - 0x40000;
		}
		memset(tmp,0,40);
		eeprom_multipleread(r_real_addr,tmp,FIX_DATA_LENGTH,s_chip);
		if(tmp[0]!=0x0ff)
			;//SEGGER_RTT_printf(0,"storagedata.rdaddr =%x,%x\n",storagedata.rdaddr,tmp[0]);
		if(tmp[0]==0xFF)
		{
			goto next_analy;
		}
		for(i=1;i<4;i++)
		{
			tmp[i] = bcd2bin(tmp[i]);
			
		}
		sys_time.tm_year = 2000+tmp[i]-1900;
		sys_time.tm_mon  = tmp[2] -1;
		sys_time.tm_mday = tmp[3];
		sys_time.tm_hour = tmp[4];
		sys_time.tm_min  = tmp[5];
		sys_time.tm_sec  = tmp[6];
		utc_time = mktime(&sys_time);
		if(utc_time > cmp_time)
		{
			cmp_time = utc_time;
			storagedata.wraddr = storagedata.rdaddr;
		}
	next_analy:
		storagedata.rdaddr = storagedata.rdaddr+FIX_DATA_LENGTH;
		if((storagedata.rdaddr >=0x3FFFF)&&(s_chip == 1))
		{
			s_chip =2;
			storagedata.rdaddr =0x40000;
		}
		if((storagedata.rdaddr >0x7FFFF)&&(s_chip == 2))
		{
			break;
		}
			
		
	}
	//SEGGER_RTT_printf(0,"storagedata.wraddr =%d,%x,%x\n",storagedata.wraddr,tmp[0],w_real_addr);
	if(storagedata.wraddr >0x3FFFF)
	{
		w_real_addr = storagedata.wraddr -0x40000;
		s_chip =2;
	}
	else
	{
		s_chip =1;
	}
	eeprom_multipleread(w_real_addr,tmp,FIX_DATA_LENGTH,s_chip);
	{
		if(tmp[0]!=0xFF)
		{
			storagedata.wraddr +=32; 
		}
	}
	//SEGGER_RTT_printf(0,"storagedata.wraddr =%d,%x\n",storagedata.wraddr,tmp[0]);
	storagedata.rdaddr = 0;
	return HAL_OK;
}
#if 0
/*****************************************************************************
 Prototype    : write_sensor_data
 Description  : 从缓存里直接向eeprom写数据，len的长度不能大于256
 Input        : uint8_t *data  
                uint16_t len   
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2016/3/7
    Author       : wxg
    Modification : Created function

*****************************************************************************/
HAL_StatusTypeDef write_sensor_data(uint8_t *data,uint16_t len)
{
	static uint8_t cs_chip =1;
	uint16_t maybelen =0;
	uint32_t realaddr =0;
	/*	
	if((len /FIX_DATA_LENGTH)!=0)
	{
		return HAL_ERROR;
	}
	*/
	//memset(easedata,0xff,256);
	if(storagedata.wraddr >= ONE_EXTRENAL_EEPROM)
	{
		cs_chip = 2;
	}
	if((storagedata.wraddr+len) > 0x7FFFF)
	{
			maybelen = 0x7FFFF - storagedata.wraddr+32;
			realaddr = storagedata.wraddr - 0x40000;
			//先把区域写成0xFF
			//eeprom_writemultipledata(realaddr,easedata,maybelen,cs_chip);
			//写真实的数值
			//SEGGER_RTT_printf(0,"storagedata.wraddr =%x,%d\n",realaddr,cs_chip);
			eeprom_writemultipledata(realaddr,data,maybelen,cs_chip);
			storagedata.wraddr =0;
			cs_chip = 1;
			//先把区域写成0xFF
			//eeprom_writemultipledata(storagedata.wraddr,easedata,len-maybelen,cs_chip);
			//写真实的数值
			eeprom_writemultipledata(storagedata.wraddr,data+maybelen,len-maybelen,cs_chip);
			storagedata.wraddr = storagedata.wraddr + len-maybelen;
	}
	else if(storagedata.wraddr > 0x3FFFF)
	{
		realaddr = storagedata.wraddr - 0x40000;
		//SEGGER_RTT_printf(0,"storagedata.wraddr =%x,%d\n",realaddr,cs_chip);
		//先把区域写成0xFF
		//eeprom_writemultipledata(realaddr,easedata,len,cs_chip);
		//写真实的数值
		eeprom_writemultipledata(realaddr,data,len,cs_chip);
		storagedata.wraddr = storagedata.wraddr+len;
	}
	else
	{
		//先把区域写成0xFF
		//eeprom_writemultipledata(storagedata.wraddr,easedata,len,cs_chip);
		//写真实的数值
		//SEGGER_RTT_printf(0,"storagedata.wraddr =%x,%d\n",storagedata.wraddr,cs_chip);
		eeprom_writemultipledata(storagedata.wraddr,data,len,cs_chip);
		storagedata.wraddr = storagedata.wraddr+len;
	}
	
	return HAL_OK;
}
#endif
HAL_StatusTypeDef write_sensor_data(uint8_t *data,uint16_t len)
{
	static uint8_t cs_chip =1;
	uint32_t realaddr =0;
	if(len!=256)
	{
		//SEGGER_RTT_printf(0,"长度错误!\n");
		return HAL_ERROR;
	}
	if(storagedata.wraddr >= ONE_EXTRENAL_EEPROM)
	{
		cs_chip = 2;
	}
	if((storagedata.wraddr+FIX_DATA_LENGTH) > 0x7FFFF)
	{
		storagedata.wraddr =0;
		cs_chip = 1;
		eeprom_writemultipledata(storagedata.wraddr,data,len,cs_chip);
		storagedata.wraddr = storagedata.wraddr + len;
	}
	else if(storagedata.wraddr > 0x3FFFF)
	{
		realaddr = storagedata.wraddr - 0x40000;
		//SEGGER_RTT_printf(0,"storagedata.wraddr =%x,%d\n",realaddr,cs_chip);
		eeprom_writemultipledata(realaddr,data,len,cs_chip);
		storagedata.wraddr = storagedata.wraddr+len;
	}
	else
	{
		//SEGGER_RTT_printf(0,"storagedata.wraddr =%x,%d\n",storagedata.wraddr,cs_chip);
		eeprom_writemultipledata(storagedata.wraddr,data,len,cs_chip);
		storagedata.wraddr = storagedata.wraddr+len;
	}
	return HAL_OK;
}
/*****************************************************************************
 Prototype    : sensor_data_report
 Description  : 存储数据上报，现在是出现上报错误怎么？和上位机心跳几次才能确
                定数据异常
 Input        : None
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2016/3/7
    Author       : wxg
    Modification : Created function

*****************************************************************************/
void sensor_data_report(void)
{
	storagedata.rdaddr =0;
	uint8_t tmp[256] ={0};
	uint8_t s_chip =1;
	uint32_t realaddr =0;
	while(storagedata.rdaddr < 0x7FFFF)
	{
		if(storagedata.rdaddr >= 0x3FFFF)
		{
			s_chip = 2;
			if(storagedata.rdaddr == 0x3FFFF)
			{
				storagedata.rdaddr = 0x40000;
				continue;
			}
			else
			{
				realaddr = storagedata.rdaddr - 0x40000;
			}
		}
		else
		{
			realaddr = storagedata.rdaddr;
		}
		realaddr =realaddr&0x1FF;
		eeprom_multipleread(realaddr,tmp,ONLYPAGE_WRITEREAD,s_chip);
		uart_senddata(tmp,256);
		storagedata.rdaddr +=ONLYPAGE_WRITEREAD;
		HAL_Delay(10);
	}
		
	
}
/*****************************************************************************
 Prototype    : led_on
 Description  : 点亮LED1,2,3,4
 Input        : LED_SHOW led  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2016/3/7
    Author       : wxg
    Modification : Created function

*****************************************************************************/
void led_on(LED_SHOW led)
{
	switch(led)
	{
		case LED1:
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_RESET);	
		break;
		case LED2:
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
		break;
		case LED3:
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
		break;
		case LED4:
			//等待国庆最后制版
		break;
	}
}
/*****************************************************************************
 Prototype    : led_off
 Description  : 关闭LED1,2,3,4
 Input        : LED_SHOW led  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2016/3/7
    Author       : wxg
    Modification : Created function

*****************************************************************************/
void led_off(LED_SHOW led)
{
	switch(led)
	{
		case LED1:
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_SET);	
		break;
		case LED2:
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
		break;
		case LED3:
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);
		break;
		case LED4:
			//等待国庆最后制版
		break;
	}
}

void led_status_battery_power(void)
{
	//uint32_t res=0;
	//res = adcvalue_read();
	//将value的值分4个阶段，4灯亮3灯2灯1灯五秒中后全部熄灭
	//SEGGER_RTT_printf(0,"CHECK_BATTERY\n");
	funkeystatus = KEY_STATUS_IDLE;
	powerkeystatus = KEY_STATUS_IDLE;
}
void led_status_power_run(void)
{
	static uint8_t runflag =0;
	static uint32_t startflag =0;
	uint32_t value =0;
	if(runflag==0)
	{
		startflag = HAL_GetTick();
		runflag =1;
	}
	if(runflag==1)
	{
		delay_t = NO_BLOCK_T;
		value = HAL_GetTick() -startflag;
		////SEGGER_RTT_printf(0,"value=%d\n",value);
		if((value>800)&&(value< 1000))
		{
			led_on(LED1);
			//SEGGER_RTT_printf(0,"led_on(LED1)=%d\n",value);
		}
		if((value>1800)&&(value<2000))
		{
			led_on(LED2);
			//SEGGER_RTT_printf(0,"led_on(LED2=%d\n",value);
		}
		if((value>2800)&&(value<3000))
		{
			led_on(LED3);
			//SEGGER_RTT_printf(0,"led_on(LED3)=%d\n",value);
		}
		if((value>3800)&&(value<4000))
		{
			led_on(LED4);
			//SEGGER_RTT_printf(0,"led_on(LED4)=%d\n",value);
		}
		if(value >=5000)
		{
			led_off(LED1);
			led_off(LED2);
			led_off(LED3);
			led_off(LED4);
			runflag =0;
			startflag =0;
			funkeystatus = KEY_STATUS_IDLE;
			powerkeystatus = KEY_STATUS_IDLE;
			delay_t = BLOCK_T;
			//SEGGER_RTT_printf(0,"led_off\n");
		}
		
	}
}

/*****************************************************************************
 Prototype    : check_status
 Description  : 功能键短按查询当前设备状态的时候的led的状态表达
 Input        : void  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2016/3/7
    Author       : wxg
    Modification : Created function

*****************************************************************************/
void check_status(void)
{
	/*
	if(sdevice == DEVICE_POWER_ON)
	{
		if(non_blocking_delay(500)==HAL_OK)
		{
			LED_LIGHT(GPIOC, GPIO_PIN_12);
			//SEGGER_RTT_printf(0,"FUN_KEY_SHORT_ON\n");
		}
	}
	*/
	if(sdevice == DEVICE_POWER_RUN)
	{
		led_status_power_run();
		//SEGGER_RTT_printf(0,"FUN_KEY\n");
		
	}
	else
	{
		
	}

//如是未运行太，则一灯直闪烁
}
HAL_StatusTypeDef non_blocking_delay(uint32_t value)
{
	static uint32_t blocktick =0;
	static uint8_t flag =0;
	if(flag==0)
	{
		blocktick = HAL_GetTick();
		flag =1;
	}
	if((flag==1)&&(blocktick==0))
	{
		return HAL_BUSY;
	}
	else
	{
		if((HAL_GetTick()-blocktick) > value)
		{
			blocktick =0;
			flag =0;
			return HAL_OK;
		}
		else
		{
			return HAL_BUSY;
		}
	}
}
void led_show_to_user(S_DEVICE cmd)
{
	switch(cmd)
	{
		case DEVICE_POWER_ON:
			////SEGGER_RTT_printf(0,"short= %d\n",funkeystatus);
			if(funkeystatus == KEY_FUNCTION_DOWN_SHORT)
			{
				funkeystatus = KEY_STATUS_IDLE;
				powerkeystatus = KEY_STATUS_IDLE;
				////SEGGER_RTT_printf(0,"short_on %d\n",funkeystatus);
			}
			if(funkeystatus == KEY_FUNCTION_DOWN_LONG)
			{
				//led_status_power_run();
				//SEGGER_RTT_printf(0,"devicerun\n");
				sdevice = DEVICE_POWER_RUN;
			}
			if(powerkeystatus ==KEY_FUNCTION_DOWN_SHORT )
			{
				led_status_battery_power();
			}
			if(powerkeystatus == KEY_FUNCTION_DOWN_LONG)
			{
				sdevice = DEVICE_POWER_OFF;
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);
			}
		break;
		case DEVICE_POWER_RUN:
			if(funkeystatus == KEY_FUNCTION_DOWN_SHORT)
			{
				funkeystatus = KEY_STATUS_IDLE;
				powerkeystatus = KEY_STATUS_IDLE;
				//SEGGER_RTT_printf(0,"short_run %d\n",funkeystatus);
			}	
			if(funkeystatus == KEY_FUNCTION_DOWN_LONG)
			{
				////SEGGER_RTT_printf(0,"leddevicerun\n");
				led_status_power_run();
			}
			if(powerkeystatus ==KEY_FUNCTION_DOWN_SHORT )
			{
				led_status_battery_power();
			}
			if(powerkeystatus == KEY_FUNCTION_DOWN_LONG)
			{
				sdevice = DEVICE_POWER_OFF;
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);
			}
		break;
		case DEVICE_INVALID:
			if(funkeystatus == KEY_FUNCTION_DOWN_SHORT)
			{
				funkeystatus = KEY_STATUS_IDLE;
				powerkeystatus = KEY_STATUS_IDLE;
				//SEGGER_RTT_printf(0,"short_run %d\n",funkeystatus);
			}	
			if(funkeystatus == KEY_FUNCTION_DOWN_LONG)
			{
				funkeystatus = KEY_STATUS_IDLE;
				powerkeystatus = KEY_STATUS_IDLE;
				//SEGGER_RTT_printf(0,"short_run %d\n",funkeystatus);
			}
			if(powerkeystatus ==KEY_FUNCTION_DOWN_SHORT )
			{
				led_status_battery_power();
			}
			if(powerkeystatus == KEY_FUNCTION_DOWN_LONG)
			{
				sdevice = DEVICE_POWER_OFF;
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);
			}
		break;	
	}
	
}

/*****************************************************************************
 Prototype    : device_init
 Description  : 设备初始化
 Input        : void  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2016/3/9
    Author       : wxg
    Modification : Created function

*****************************************************************************/
void device_init(void)
{
	
	//MX_GPIO_Init();
	init_fifo();
	key_PB12_init();
	key_PB3_init();
	interrupt_pb_init(GPIOB,GPIO_PIN_10);
	//interrupt_pb_init(GPIOB,GPIO_PIN_11);
	uart_configure();
	RTC8564_Init();
	adc_configure();
 	get_chipid();
	g_wakup =0;
	
}

/*****************************************************************************
 Prototype    : start_adxl375
 Description  : 加速度传感器的初始化(休眠醒来不需要再初始化了)
 Input        : void  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2016/3/25
    Author       : wxg
    Modification : Created function

*****************************************************************************/

uint8_t data_process(uint8_t *data )
{
	uint8_t getres =0,i=0;
	if(sdevice==DEVICE_INVALID)
	{
		return 0;
	}
	getres = get_data(data,datanum);
	if(getres!=0)
	{
		for(i=0 ;i< getres;i++)
		{
			;//SEGGER_RTT_printf(0,"%x\n",data[i]);
		}
		if(getres == datanum)
		{
			datanum =0;
		}
		else
		{
			datanum = datanum -getres;
		}
		if(getres == 0)
		{
			return 0;
		}
		pc_data_parsing(data,getres);
	}
	return 0;
}
/*****************************************************************************
 Prototype    : no_press_run_status
 Description  : 没有按运行键程序会一直无阻赛闪烁
 Input        : void  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2016/3/9
    Author       : wxg
    Modification : Created function

*****************************************************************************/
void no_press_run_status(void)
{
	if((sdevice==DEVICE_POWER_ON)&&(funkeystatus ==KEY_STATUS_IDLE)&&(powerkeystatus==KEY_STATUS_IDLE))
	{
		if((non_blocking_delay(500)==HAL_OK))
		{
			LED_LIGHT(GPIOC,GPIO_PIN_12);
			////SEGGER_RTT_printf(0,"no_press_run_status\n");
		}
	}
	if(sdevice==DEVICE_INVALID)
	{
		if((non_blocking_delay(500)==HAL_OK))
		{
			LED_LIGHT(GPIOC,GPIO_PIN_12);
			LED_LIGHT(GPIOB,GPIO_PIN_4);
			////SEGGER_RTT_printf(0,"no_press_run_status\n");
		}
	}
}


HAL_StatusTypeDef equipment_failure(void)
{
	uint8_t tmpdata[32] ={0};
	uint32_t utc_invalid =0;
	uint32_t utc_now =0;
	internal_eeprom_read(PARAM_DEVICE_INVALID_ADDR,tmpdata,FIX_DATA_LENGTH);
	if(tmpdata[0]!=0xFF)
	{
		if(tmpdata[0] <= 6)
		{
			return HAL_ERROR;
				
		}
		else
		{
			tmpdata[1] = bcd2bin(tmpdata[1]);
			tmpdata[2] = bcd2bin(tmpdata[2]);
			tmpdata[3] = bcd2bin(tmpdata[3]);
			tmpdata[4] = bcd2bin(tmpdata[4]);
			tmpdata[5] = bcd2bin(tmpdata[5]);
			tmpdata[6] = bcd2bin(tmpdata[6]);
			sys_time.tm_year = 2000+tmpdata[1]-1900;
			sys_time.tm_mon  = tmpdata[2] -1;
			sys_time.tm_mday = tmpdata[3];
			sys_time.tm_hour = tmpdata[4];
			sys_time.tm_min  = tmpdata[5];
			sys_time.tm_sec  = tmpdata[6];
			utc_invalid = mktime(&sys_time);
			RTC8564_Read(rtc_current);
			tmpdata[1] = rtc_current.year;
			tmpdata[2] = rtc_current.month& 0x1F;
			tmpdata[3] = rtc_current.day & 0x3F;
			tmpdata[4] = rtc_current.hour & 0x3F;
			tmpdata[5] = rtc_current.min & 0x7F;
			tmpdata[6] = rtc_current.sec & 0x7F;
			
			tmpdata[1] = bcd2bin(tmpdata[1]);
			tmpdata[2] = bcd2bin(tmpdata[2]);
			tmpdata[3] = bcd2bin(tmpdata[3]);
			tmpdata[4] = bcd2bin(tmpdata[4]);
			tmpdata[5] = bcd2bin(tmpdata[5]);
			tmpdata[6] = bcd2bin(tmpdata[6]);
			sys_time.tm_year = 2000+tmpdata[1]-1900;
			sys_time.tm_mon  = tmpdata[2] -1;
			sys_time.tm_mday = tmpdata[3];
			sys_time.tm_hour = tmpdata[4];
			sys_time.tm_min  = tmpdata[5];
			sys_time.tm_sec  = tmpdata[6];
			utc_now = mktime(&sys_time);
			if(utc_now > utc_invalid)
			{
				sdevice = DEVICE_INVALID;
				//SEGGER_RTT_printf(0,"device_is_invalid\n");
				return HAL_OK;
			}
			else
			{
				return HAL_ERROR;
			}
			
		}
	}
	else
	{
		return HAL_ERROR;
	}
}

void block_delay(void)
{
	HAL_Delay(500);
}

