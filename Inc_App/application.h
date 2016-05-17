#ifndef __APPLICATION_H_
#define __APPLICATION_H_

#include <stdio.h>
#include <string.h>
#include "stdlib.h"
#include "adxl375.h"
#include "hmc5883l.h"
#include "fm25v10.h"
#include "rtc8564.h"
#include "kfifo.h"

//#define TXBUFFERSIZE                      (COUNTOF(TxBuffer) - 1)
/* Size of Reception buffer */
//#define RXBUFFERSIZE                      TXBUFFERSIZE




  
/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))


#define I2Cx_SDA_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2Cx_SCL_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2Cx_SCL_PIN                    GPIO_PIN_8
#define I2Cx_SCL_GPIO_PORT              GPIOB
#define I2Cx_SCL_AF                     GPIO_AF4_I2C1
#define I2Cx_SDA_PIN                    GPIO_PIN_9
#define I2Cx_SDA_GPIO_PORT              GPIOB
#define I2Cx_SDA_AF                     GPIO_AF4_I2C1

#define I2Cx_FORCE_RESET()               __HAL_RCC_I2C1_FORCE_RESET()
#define I2Cx_RELEASE_RESET()             __HAL_RCC_I2C1_RELEASE_RESET()

#define I2Cx                              I2C1
#define I2Cx_CLK_ENABLE()                __HAL_RCC_I2C1_CLK_ENABLE()
#define I2C_TIMING_400KHZ       0x00B1112E /* Analog Filter ON, Rise Time 250ns, Fall Time 100ns */ 

#define ADXL375_WR      0xA6
#define ADXL375_RE      0xA7


#define I2C2_SCL_PIN                    GPIO_PIN_13
#define I2C2_SCL_GPIO_PORT              GPIOB
#define I2C2_SCL_AF                     GPIO_AF5_I2C2
#define I2C2_SDA_PIN                    GPIO_PIN_14
#define I2C2_SDA_GPIO_PORT              GPIOB
#define I2C2_SDA_AF                     GPIO_AF5_I2C2

#define I2C2_FORCE_RESET()               __HAL_RCC_I2C2_FORCE_RESET()
#define I2C2_RELEASE_RESET()             __HAL_RCC_I2C2_RELEASE_RESET()

#define I2C2_CLK_ENABLE()                __HAL_RCC_I2C2_CLK_ENABLE()
#define I2C_TIMING_400KHZ       0x00B1112E /* Analog Filter ON, Rise Time 250ns, Fall Time 100ns */ 

/**********************内部EEPROM作为参数的地址划分***/
#define	PARAM_FIX_LENGTH			32
#define EEPROM_USER_START_ADDR		0x08080000
#define PARAM_TIME_ADDR				0
#define PARAM_DEVICE_INVALID_ADDR	32
#define PARAM_SOFT_VERSION_ADDR		64
#define PARAM_ADXL_ADDR				96
#define MAX_POINT_X					128
#define MAX_POINT_Y					160
#define MAX_POINT_Z					192


#define KEY_DOWN_FIRST				0x08
#define KEY_DOWN_SHORT				0x01
#define KEY_DOWN_LONG				0x02
#define MAX_DOWN_TIME				4000



extern uint8_t g_wakup;


typedef enum 
{
  PARAM_TIME      = 0x00,
  PARAM_DEVICE   = 0x01,
  PARAM_ADXL     = 0x02,
  PARAM_DATA     = 0x03,
  PARAM_POWER	= 0x04
} PARAMCMD;

typedef enum
{
	LED1 =0,
	LED2,
	LED3,
	LED4	
}LED_SHOW;

typedef enum
{
	DEVICE_POWER_OFF=0,
	DEVICE_POWER_ON,
	DEVICE_POWER_RUN,
	DEVICE_INVALID
}S_DEVICE;


typedef enum
{
	ADXL_INTERRUPT_IDLE=0,
	ADXL_INTERRUPT_ACT,
	ADXL_INTERRUPT_INACT
}ADXL_STATUS;

typedef enum
{
	KEY_INTERRUPT_IDLE=0,
	KEY_INTERRUPT_ACT,
	KEY_INTERRUPT_INACT
}KEY_STATUS;


extern KEY_STATUS powerkeystatus;
extern KEY_STATUS funkeystatus;

void SystemClock_Config(void);
//void MX_GPIO_Init(void);
void key_PB12_init(void);
void SystemPower_Config(void);
void uart_configure(void);
void uart_senddata(uint8_t *txbuffer,uint16_t txsize);
void uart_rcvdata(uint8_t *rxbuffer,uint16_t rxsize);
void key_PB3_init(void);
void LED_LIGHT(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
KEY_STATUS power_key_scan(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
KEY_STATUS fun_key_scan(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
uint8_t key_debounce(void);
void calc_init(int32_t *pstr,int8_t *pout);
void writei2c_byte(I2C_HandleTypeDef *hi2c,uint8_t addr,uint8_t reg_address,uint8_t reg_data);
uint8_t readi2c_nbyte(I2C_HandleTypeDef *hi2c,uint8_t addr,uint8_t reg_address,uint8_t *data,uint8_t size);
void calc_init(int32_t *pstr,int8_t *pout);
void SystemClockConfig_STOP(void);
HAL_StatusTypeDef detect_it(uint32_t value);
HAL_StatusTypeDef internal_eeprom_ease(void);
HAL_StatusTypeDef internal_eeprom_write(uint32_t addr,uint8_t *data,uint32_t len);
HAL_StatusTypeDef internal_eeprom_read(uint32_t addr,uint8_t *data,uint32_t len);
void adc_configure(void);
uint32_t adcvalue_read(void);


void led_show_to_user(S_DEVICE cmd);
HAL_StatusTypeDef non_blocking_delay(uint32_t value);
void check_status(void);
void led_status_power_run(void);
void led_status_battery_power(void);
void led_off(LED_SHOW led);
void led_on(LED_SHOW led);
void sensor_data_report(void);
HAL_StatusTypeDef write_sensor_data(uint8_t *data,uint16_t len);
HAL_StatusTypeDef get_eeprom_write_addr(void);
HAL_StatusTypeDef pc_cmd_parsing(uint8_t *psr,uint16_t len);
HAL_StatusTypeDef data_package_send(PARAMCMD cmd,uint8_t* pdata,uint8_t dlen);
uint16_t data_code(uint8_t *dest,uint8_t *src, uint8_t srclen);
uint8_t data_decode(uint8_t *str,uint16_t len);
void device_init(void);
uint8_t data_process(uint8_t *data );
void no_press_run_status(void);
HAL_StatusTypeDef equipment_failure(void);
HAL_StatusTypeDef eeprom_ease_all(void);
HAL_StatusTypeDef adxl_interrupt_source(void);
HAL_StatusTypeDef adxl_fifo_data(uint8_t *data);
void block_delay(void);

void interrupt_pb_init(GPIO_TypeDef  *GPIOx,uint16_t GPIO_Pin);







#endif



