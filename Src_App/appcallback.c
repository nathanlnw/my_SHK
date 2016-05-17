#include <stdio.h>
#include <string.h>

#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal.h"
#include "stdlib.h"
#include "application.h"
#include "main.h"


/*****************************************************************************
 Prototype    : HAL_GPIO_EXTI_Callback
 Description  : �ⲿ�жϻص�����
 Input        : uint16_t GPIO_Pin  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2016/2/20
    Author       : wxg
    Modification : Created function

*****************************************************************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_10)
	{
		printf("interrupt_GPIO_PIN_10\n",22);
		
	}
 
}

void HAL_SYSTICK_Callback(void)
{
  HAL_IncTick();
 
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: trasfer complete*/
}




/*****************************************************************************
 Prototype    : HAL_UART_RxCpltCallback
 Description  : �жϽ������ݵĻص�����
 Input        : UART_HandleTypeDef *UartHandle  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2016/2/22
    Author       : wxg
    Modification : Created function

*****************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	
	/* Set transmission flag: trasfer complete*/
	
	//HAL_UART_Receive_IT(UartHandle,&rxbyte, 1); 	
#if 0
	   HAL_UART_Receive_IT(UartHandle,Uart1_RxBuf+Uart1_RxBuf_wr, 1); 	
	   Uart1_RxBuf_wr++;
	   if(Uart1_RxBuf_wr>=512)
	   	   Uart1_RxBuf_wr=0;
	   #endif
	//   else
	      // hlpuart1.Instance->TDR = Uart1_RxBuf[Uart1_RxBuf_wr-1];//���ͽ��յ�����

	   /*
	     if(__HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_RXNE) == SET)//�н��ܵ��ַ���
        {
            Uart1_RxBuf[Uart1_RxBuf++] = (uint8_t)(hlpuart1.Instance->DR & (uint8_t)0x00FF);//����
            hlpuart1.Instance->TDR = Uart1_RxBuf[Uart1_RxBuf-1];//���ͽ��յ�����
        }
        */
        //------------------------------------
       uint8_t i=0;
	//if(UartHandle->Instance==UART5)
	{Uart1_RxBuf[Uart1_RxBuf_wr]=aRxBuffer[0];
		if((Uart1_RxBuf[Uart1_RxBuf_wr-1]==0x0D)&&(Uart1_RxBuf[Uart1_RxBuf_wr]==0x0A))
			{
			  Uart1Ready_R=SET;
			  Uart1_RxBuf_wr++; 
		      //Rx_Num_UART5=++Rx_count_UART5; 
			  //Rx_count_UART5=0;
			 }
		else
			Uart1_RxBuf_wr++;
		
		HAL_UART_Receive_IT(&hlpuart1,aRxBuffer,1);//������һ�ν����ж�
	} 

	   //-----------------------------------
}
/*****************************************************************************
 Prototype    : HAL_UART_ErrorCallback
 Description  : �ÿ⺯�����жϳ��ִ���ʱ��Ĵ�����
 Input        : UART_HandleTypeDef *huart  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2016/2/22
    Author       : wxg
    Modification : Created function
	note:Ŀǰ�ҵĴ������Ƚϴֱ����Ǽ򵥵Ĺر��ж���ʹ���ж�
*****************************************************************************/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	NVIC_DisableIRQ(USART1_IRQn); /* Disable USART1_IRQn */
	NVIC_EnableIRQ(USART1_IRQn);
	//SEGGER_RTT_printf(0,"errorrxIRQ!!\n");
}

