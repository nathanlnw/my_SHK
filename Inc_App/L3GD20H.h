#ifndef _L3GD20H
#define  _L3GD20H

#include "main.h"
/* l3gd20 gyroscope registers */
#define WHO_AM_I              0x0F

#define CTRL_REG1             0x20    /* CTRL REG1 */
#define CTRL_REG2             0x21    /* CTRL REG2 */
#define CTRL_REG3             0x22    /* CTRL_REG3 */
#define CTRL_REG4             0x23    /* CTRL_REG4 */
#define CTRL_REG5             0x24    /* CTRL_REG5 */
#define	REFERENCE             0x25    /* REFERENCE REG */
#define STATUS_REG                0x27    /* XYZ  */
#define	FIFO_CTRL_REG         0x2E    /* FIFO CONTROL REGISTER */
#define FIFO_SRC_REG          0x2F    /* FIFO SOURCE REGISTER */
#define	OUT_X_L               0x28    /* 1st AXIS OUT REG of 6 */
#define	OUT_Y_L               0x2A    /* 1st AXIS OUT REG of 6 */
#define	OUT_Z_L               0x2C    /* 1st AXIS OUT REG of 6 */ 




#define AXISDATA_REG	      OUT_X_L

/* CTRL_REG1 */
#define ALL_ZEROES            0x00
#define PM_OFF                0x00
#define PM_NORMAL             0x08
#define ENABLE_ALL_AXES       0x07
#define ENABLE_NO_AXES        0x00
#define BW00                  0x00
#define BW01                  0x10
#define BW10                  0x20
#define BW11                  0x30
#define ODR095                0x00  /* ODR =  95Hz */
#define ODR190                0x40  /* ODR = 190Hz */
#define ODR380                0x80  /* ODR = 380Hz */
#define ODR760                0xC0  /* ODR = 760Hz */

/* CTRL_REG3 bits */
#define	I2_DRDY               0x08
#define	I2_WTM                0x04
#define	I2_OVRUN              0x02
#define	I2_EMPTY              0x01
#define	I2_NONE               0x00
#define	I2_MASK               0x0F

/* CTRL_REG4 bits */
#define	FS_MASK               0x30
#define	BDU_ENABLE            0x80

/* CTRL_REG5 bits */
#define	FIFO_ENABLE           0x40
#define HPF_ENALBE            0x11

/* FIFO_CTRL_REG bits */
#define	FIFO_MODE_MASK        0xE0
#define	FIFO_MODE_BYPASS      0x00
#define	FIFO_MODE_FIFO        0x20
#define	FIFO_MODE_STREAM      0x40
#define	FIFO_MODE_STR2FIFO    0x60
#define	FIFO_MODE_BYPASS2STR  0x80
#define	FIFO_WATERMARK_MASK   0x1F


// SPI  ADDRESS  RW   MS
#define  L3G_ADDR_RW    0x80      //  bit value   0:  write     1:  read
#define  L3G_ADDR_MS    0x40      // bit  value  0: don't move     1:  address auto incrementd


//------ SPI2  

#define  L3GD20H_CS_LOW()     HAL_GPIO_WritePin(SHK_L3G_CS_GPIO_Port,SHK_L3G_CS_Pin,GPIO_PIN_RESET)
#define  L3GD20H_CS_HIGH()    HAL_GPIO_WritePin(SHK_L3G_CS_GPIO_Port,SHK_L3G_CS_Pin,GPIO_PIN_SET)
#define  L3GD20H_DEN_LOW()    HAL_GPIO_WritePin(SHK_DEN_GPIO_Port,SHK_DEN_Pin,GPIO_PIN_SET)
#define  L3GD20H_DEN_HIGH()   HAL_GPIO_WritePin(SHK_DEN_GPIO_Port,SHK_DEN_Pin,GPIO_PIN_RESET)



extern void L3GD20H_GET(void);





#endif
