/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include <stdio.h>
#include <string.h>
#include "stdlib.h"
#include "Main.h"
#include "ShakeSensor_app.h"



/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef hlpuart1;
DMA_HandleTypeDef hdma_lpuart1_tx;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

WWDG_HandleTypeDef hwwdg;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
SYS_CONFIG   sys_config;
PRO_STRUCT   Project_SHK;
WRK_STATE    SHK_WRK_STATE;


u32    main_counter=0;
u8     Uart1_RxBuf[100];
u16    Uart1_RxBuf_wr=0;
u8     U1_rx_sub1[100];
u8     U1_rx_sub1_wr=0;
uint8_t aRxBuffer[1];
u8     Uart1Ready_R=0;
u8     Seven_one_rx=0;  // 接收到7E 的个数



u8     U1_TxBuf[600];//   串口发送寄存器
u16    U1_TxBuf_wr;
u8     SHK_SampleData_Buff[600];  //  采集缓存
u16    SHK_SampleData_Buff_wr; 

u8     SHK_SaveData_Buff[520];   // 存储缓存
u16    SHK_SaveData_Buff_wr; 





/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_ADC_Init(void);
void MX_I2C1_Init(void);
void MX_I2C2_Init(void);
void MX_SPI1_Init(void);
void MX_WWDG_Init(void);
void MX_LPUART1_UART_Init(void);
void MX_SPI2_Init_ADXL375(void);
void MX_SPI2_Init_L3GD20H(void);

void MX_RTC_Init(void);
void MX_TIM2_Init(void);
void MX_NVIC_Init(void);






PUTCHAR_PROTOTYPE
{
  /* e.g. write a character to the USART */
    hlpuart1.Instance->TDR = (uint8_t) ch;
 
  /* Loop until the end of transmission */
    while(__HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_TC) == RESET){}
 
  return ch;

}

void U1_Tx_OneByte(uint8_t ch)
{
    hlpuart1.Instance->TDR = (uint8_t) ch;
 
  /* Loop until the end of transmission */
    while(__HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_TC) == RESET){}

}

void  Output_string(u8 * instr)
{
   u16  len=strlen((char*)instr);
   HAL_UART_Transmit_IT(&hlpuart1,instr,len);   
    
}

void  Output_Data(u8 * instr,u16 len)
{
     //HAL_UART_Transmit_DMA(&hlpuart1,instr,len); 
	 HAL_UART_Transmit_IT(&hlpuart1,instr,len); 
}
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

void delay_us_1(u16 j)
{
    u8 i;
    while(j--)
    {
        i = 3;
        while(i--);
    }
}

void delay_ms_1(u16 j )
{
    while(j--)
    {
        delay_us_1(2000); // 1000
    }
}

#if  0
void Sample_test(void)
{
     if(SHK_WRK_STATE.ENABLE_Triger==1)    //  K1 按键按下
     {
       //  Part 1  :  Trans 
         if(SHK_WRK_STATE.Page_in_offset==0)
         {
            SHK_SampleData_Buff_wr=0;
            SHK_SampleData_Buff[SHK_SampleData_Buff_wr++]='s';
			SHK_SampleData_Buff[SHK_SampleData_Buff_wr++]=0x01; //  分包标志
			SHK_SampleData_Buff[SHK_SampleData_Buff_wr++]=SHK_WRK_STATE.current_packet+1; // LSB	
			SHK_SampleData_Buff[SHK_SampleData_Buff_wr++]=0x00; //   MSB
			SHK_SampleData_Buff[SHK_SampleData_Buff_wr++]=29;//  总包数  LSB
			SHK_SampleData_Buff[SHK_SampleData_Buff_wr++]=0x00;// MSB
		   if( SHK_WRK_STATE.current_packet==0)	
		   	{
				SHK_SampleData_Buff[SHK_SampleData_Buff_wr++]=0x06 ;   //  长度LSB     504+14=518
				SHK_SampleData_Buff[SHK_SampleData_Buff_wr++]= 0x02;
				memcpy(SHK_SampleData_Buff+SHK_SampleData_Buff_wr,rtc_current.BCD_6_Bytes,6); 
				SHK_SampleData_Buff_wr+=6;
		   	}
		   else
		   	if(SHK_WRK_STATE.current_packet==28)
			{
							SHK_SampleData_Buff[SHK_SampleData_Buff_wr++]=0x28 ;	 //  长度LSB	 288+8=512
							SHK_SampleData_Buff[SHK_SampleData_Buff_wr++]= 0x01; 
			}
		   else
		   	{
                SHK_SampleData_Buff[SHK_SampleData_Buff_wr++]=0x00 ;   //  长度LSB     504+8=512
				SHK_SampleData_Buff[SHK_SampleData_Buff_wr++]= 0x02; 
		   	}
         }	   
		  //   run  every time 
		 SHK_Sensor_data_Get();
		 memcpy(SHK_SampleData_Buff+SHK_SampleData_Buff_wr,Project_SHK.ACCEL_ADX375_ACCEL_6BYTES,6);
		 SHK_SampleData_Buff_wr+=6;
		 memcpy(SHK_SampleData_Buff+SHK_SampleData_Buff_wr,Project_SHK.GYRO_L3G_6BYTES,6);
		 SHK_SampleData_Buff_wr+=6;
		 memcpy(SHK_SampleData_Buff+SHK_SampleData_Buff_wr,Project_SHK.COMPASS_HMC_6BYTES,6);
		 SHK_SampleData_Buff_wr+=6;
		 SHK_WRK_STATE.Page_in_offset++;
			 
         //   ------------------
        if((SHK_WRK_STATE.current_packet>=28)&&(SHK_WRK_STATE.Page_in_offset>=16))
		 	{
			 	 //------
	             U1_TxBuf[0]=0x7E;
	             U1_TxBuf_wr=Protocol7E_Encode(U1_TxBuf+1,SHK_SampleData_Buff,SHK_SampleData_Buff_wr);
				 U1_TxBuf_wr++;
				 U1_TxBuf[U1_TxBuf_wr++]=0x7E;
				 Output_Data(U1_TxBuf,U1_TxBuf_wr);
				 SHK_SampleData_Buff_wr=0;
			    //---------------
			    SHK_WRK_STATE.Page_in_offset=0; 
                K1_Value.work_state=0; 
			    LED2_OFF;
		 	} 
		else
		//-------------------------------	 
         if((SHK_WRK_STATE.Page_in_offset>=28)&&(SHK_WRK_STATE.current_packet<28))   
         { 
            //----------            
            SHK_WRK_STATE.current_packet++;
			#if 1
			//------
             U1_TxBuf[0]=0x7E;
             U1_TxBuf_wr=Protocol7E_Encode(U1_TxBuf+1,SHK_SampleData_Buff,SHK_SampleData_Buff_wr);
			 U1_TxBuf_wr++;
			 U1_TxBuf[U1_TxBuf_wr++]=0x7E;
			 Output_Data(U1_TxBuf,U1_TxBuf_wr);
			 #endif
			 SHK_SampleData_Buff_wr=0;
		     //---------------
             SHK_WRK_STATE.Page_in_offset=0;
          }
		 
		 //  ------------------------
    }



}
#endif

 void TIM2_Service(void)
 {   //  2.5  ms   in  once
     Sample_Get();
	 
	 // light 
	 main_counter++;
	 if(main_counter%20==0)
	  {
		LED1_ON;
		//LED2_ON;
		// LED3_ON;
		//LED4_ON;
		
	  } 
	 else
	  {
		LED1_OFF;	
		// LED2_OFF; 
		//LED3_OFF; 
		//LED4_OFF;   
	  }
	  //--------


 }


int main(void)
{

  /* USER CODE BEGIN 1 */
  

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  //HAL_Delay(30);
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_I2C1_Init();//HMC5883L
  MX_I2C2_Init();// out RTC  8564 chip   
  eeprom_POWER_ON(); 
  HAL_Delay(10);
  MX_SPI1_Init();// EEPROM
  MX_WWDG_Init();
  MX_LPUART1_UART_Init();
  //-------------------------------------
  L3GD20H_CS_HIGH();  // Set High
  ADXL375_CS_HIGH();
  L3GD20H_DEN_HIGH(); // Disable   
 // MX_SPI2_Init();//L3D20H aux      ADXL375 main     two chip init operations are different
  //------------------------------------
  MX_RTC_Init();
  MX_TIM2_Init();  
  MX_NVIC_Init();
  HAL_ADC_Start(&hadc);
  //   Modules  Status  init
  printf("\r\n  Starting ........\r\n");  
  Devices_Init_Total(); 
  
  HAL_TIM_Base_Start_IT(&htim2); //  启动定时器中断 
   printf("\r\n my name  is  Nathan \r\n");
  while (1)
  {
	  main_counter++;
      
    #if 0 
	 if(main_counter%2)
	  {
	    LED1_ON;
		LED2_ON;
		LED3_ON;
		LED4_ON;
		
	  }	
	 else
	  {
	    LED1_OFF;   
		LED2_OFF; 
		LED3_OFF; 
		LED4_OFF; 
	  }
    #endif
    //  function  test
     Function_test();  
	
    //   Rx  process
    Rx_CMD_process();
  }


}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_RTC;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** NVIC Configuration
*/
void MX_NVIC_Init(void)
{
  /* WWDG_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(WWDG_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(WWDG_IRQn);
  /* EXTI4_15_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* SPI1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SPI1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(SPI1_IRQn);
  /* SPI2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SPI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(SPI2_IRQn);
  /* AES_RNG_LPUART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(AES_RNG_LPUART1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(AES_RNG_LPUART1_IRQn);
  /* DMA1_Channel4_5_6_7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);
  /* ADC1_COMP_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC1_COMP_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC1_COMP_IRQn);
}

/* ADC init function */
void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  HAL_ADC_Init(&hadc);

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);

}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);

    /**Configure Analogue filter 
    */
  HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);

}

/* I2C2 init function */
void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x0000020B;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c2);

    /**Configure Analogue filter 
    */
  HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE);

}

/* LPUART1 init function */
void MX_LPUART1_UART_Init(void)
{

  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate =256000 ;//115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  hlpuart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  hlpuart1.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  HAL_UART_Init(&hlpuart1); 

}

/* RTC init function */
void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

    /**Initialize RTC and set the Time and Date 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  HAL_RTC_Init(&hrtc);

  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD);

    /**Enable the WakeUp 
    */
  HAL_RTCEx_SetWakeUpTimer(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16);

    /**Enable Calibrartion 
    */
  HAL_RTCEx_SetCalibrationOutPut(&hrtc, RTC_CALIBOUTPUT_1HZ);

}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;//SPI_POLARITY_LOW 
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4; // 4  - 4MHZ      
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  HAL_SPI_Init(&hspi1);

}

/* SPI2 init function */
#if 0
void MX_SPI2_Init(void)
{
       // base on ADXL375
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;   
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity =SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;  
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB; 
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;  
  hspi2.Init.CRCPolynomial = 7;
  HAL_SPI_Init(&hspi2);

}
#endif

void MX_SPI2_Init_ADXL375(void)
{
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;   
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity =SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;  
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; 
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB; 
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;  
  hspi2.Init.CRCPolynomial = 7;
  HAL_SPI_Init(&hspi2);   
}

void MX_SPI2_Init_L3GD20H(void)
{
  hspi2.Instance = SPI2; 
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;   
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity =SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;  
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; 
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB; 
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;  
  hspi2.Init.CRCPolynomial = 7;
  HAL_SPI_Init(&hspi2);
   
}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 160-1;  // 当前是16 M 的主频
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 250;//2.5ms
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

}



/* WWDG init function */
void MX_WWDG_Init(void)
{

  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_1;
  hwwdg.Init.Window = 64;
  hwwdg.Init.Counter = 64;
  HAL_WWDG_Init(&hwwdg);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_5_6_7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SHK_WP2_Pin|SHK_WP1_Pin|SHK_CS1_Pin|SHK_375_CS_Pin 
                          |SHK_L3G_CS_Pin|SHK_LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SHK_HOLD2_GPIO_Port, SHK_HOLD2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SHK_CS2_Pin|SHK_HOLD2A1_Pin|SHK_DEN_Pin|SHK_LED4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SHK_PWR_OnOff_GPIO_Port, SHK_PWR_OnOff_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SHK_LED2_Pin|SHK_LED1_Pin|SHK_PB4_EEPowr_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SHK_WP2_Pin SHK_WP1_Pin SHK_HOLD2_Pin SHK_CS1_Pin 
                           SHK_375_CS_Pin SHK_L3G_CS_Pin SHK_LED3_Pin */
  GPIO_InitStruct.Pin = SHK_WP2_Pin|SHK_WP1_Pin|SHK_HOLD2_Pin|SHK_CS1_Pin 
                          |SHK_375_CS_Pin|SHK_L3G_CS_Pin|SHK_LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;//default LOW
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SHK_CS2_Pin SHK_HOLD2A1_Pin SHK_DEN_Pin SHK_DRDY_Pin 
                           SHK_LED4_Pin */
  GPIO_InitStruct.Pin = SHK_CS2_Pin|SHK_HOLD2A1_Pin|SHK_DEN_Pin|SHK_LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SHK_PWR_OnOff_Pin SHK_LED2_Pin SHK_LED1_Pin SHK_PB4_EEPowr_Pin */
  GPIO_InitStruct.Pin = SHK_PWR_OnOff_Pin|SHK_LED2_Pin|SHK_LED1_Pin|SHK_PB4_EEPowr_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SHK_PWR_TEST_Pin SHK_INT_RTC_Pin SHK_CHARGE_Pin */
  GPIO_InitStruct.Pin = SHK_PWR_TEST_Pin|SHK_INT_RTC_Pin|SHK_CHARGE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SHK_375_INT1_Pin */
  GPIO_InitStruct.Pin = SHK_375_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SHK_375_INT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SHK_L3G_INT2_Pin SHK_L3G_INT1_Pin SHK_DRDY_Pin SHK_K1_Pin */
  GPIO_InitStruct.Pin = SHK_L3G_INT2_Pin|SHK_L3G_INT1_Pin|SHK_DRDY_Pin|SHK_K1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
void System_config_init(void)
{
 
     sys_config.ADXL375_ShakeThreshod=ADXL375_INIT_THRESHOD;   //  
     sys_config.StartWork_RTC[0]=0x15;   // 2015-5-5     10:12:25
     sys_config.StartWork_RTC[1]=0x05;  
	 sys_config.StartWork_RTC[2]=0x05;  
	 sys_config.StartWork_RTC[3]=0x10;  
	 sys_config.StartWork_RTC[4]=0x12;  
	 sys_config.StartWork_RTC[5]=0x25;  

     
	 sys_config.EndWork_RTC[0]=0x50;   // 2050-6-6    11:25:22       失效时间
     sys_config.EndWork_RTC[1]=0x06;    
	 sys_config.EndWork_RTC[2]=0x06;  
	 sys_config.EndWork_RTC[3]=0x11;  
	 sys_config.EndWork_RTC[4]=0x25;  
	 sys_config.EndWork_RTC[5]=0x22;  	

     sys_config.Threshod_ADXL375[0]=THRESH_ACT_Defaut;
	 sys_config.Threshod_ADXL375[1]=THRESH_INACT_Defaut;
	 sys_config.Threshod_ADXL375[2]=TIME_INACT_Defaut;

	 sys_config.Threshod_ADXL375[3]=ADXL375_INIT_THRESHOD;
	 sys_config.Threshod_ADXL375[4]=ADXL375_INIT_FREQ;
	 sys_config.Threshod_ADXL375[5]=0x00;

	 memset(sys_config.User_info,0,256);
	 memcpy(sys_config.User_info,"TCB 震动传感器",14);
	 
	 // sys ID 
	 sys_config.SYStem_ID=SYS_ID;

	 // Version
	 sys_config.Version_Info=VERSION_INFO;

     sys_config.SaveWr=0;
	 sys_config.SaveFull_state=0;
}
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
