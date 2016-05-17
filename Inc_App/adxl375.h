#ifndef __ADXL375_H__
#define  __ADXL375_H__

#include "stdint.h"
#include "main.h"






#define ADXL375_ADDR    0x53
/* Bit values in TAP_AXES                                               */
#define XL375_TAP_Z_ENABLE    0x01
#define XL375_TAP_Z_DISABLE   0x00
#define XL375_TAP_Y_ENABLE    0x02
#define XL375_TAP_Y_DISABLE   0x00
#define XL375_TAP_X_ENABLE    0x04
#define XL375_TAP_X_DISABLE   0x00
#define XL375_TAP_SUPPRESS    0x08
//FIFO
#define XL375_TRIGGER_INT1         0x00
#define XL375_TRIGGER_INT2         0x20
#define XL375_FIFO_MODE_BYPASS     0x00
#define XL375_FIFO_RESET           0x00
#define XL375_FIFO_MODE_FIFO       0x40
#define XL375_FIFO_MODE_STREAM     0x80
#define XL375_FIFO_MODE_TRIGGER    0xc0
//INT
#define XL375_OVERRUN              0x01
#define XL375_WATERMARK            0x02
#define XL375_FREEFALL             0x04
#define XL375_INACTIVITY           0x08
#define XL375_ACTIVITY             0x10
#define XL375_DOUBLETAP            0x20
#define XL375_SINGLETAP            0x40
#define XL375_DATAREADY            0x80




/* ------- Register names ------- */
#define  XL375_DEVID           0x00
#define XL375_RESERVED1       0x01
#define XL375_THRESH_TAP      0x1d
#define XL375_OFSX            0x1e
#define XL375_OFSY            0x1f
#define XL375_OFSZ            0x20
#define XL375_DUR             0x21
#define XL375_LATENT          0x22
#define XL375_WINDOW          0x23
#define XL375_THRESH_ACT      0x24
#define XL375_THRESH_INACT    0x25
#define XL375_TIME_INACT      0x26
#define XL375_ACT_INACT_CTL   0x27
#define XL375_THRESH_FF       0x28
#define XL375_TIME_FF         0x29
#define XL375_TAP_AXES        0x2a
#define XL375_ACT_TAP_STATUS  0x2b
#define XL375_BW_RATE         0x2c
#define XL375_POWER_CTL       0x2d
#define XL375_INT_ENABLE      0x2e
#define XL375_INT_MAP         0x2f
#define XL375_INT_SOURCE      0x30
#define XL375_DATA_FORMAT     0x31
#define XL375_DATAX0          0x32
#define XL375_DATAX1          0x33
#define XL375_DATAY0          0x34
#define XL375_DATAY1          0x35
#define XL375_DATAZ0          0x36
#define XL375_DATAZ1          0x37
#define XL375_FIFO_CTL        0x38
#define XL375_FIFO_STATUS     0x39

typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;   
}ADXL375_TYPE;


//  SPI2   PIN
#define  ADXL375_CS_LOW()     HAL_GPIO_WritePin(SHK_375_CS_GPIO_Port,SHK_375_CS_Pin,GPIO_PIN_RESET)
#define  ADXL375_CS_HIGH()    HAL_GPIO_WritePin(SHK_375_CS_GPIO_Port,SHK_375_CS_Pin,GPIO_PIN_SET)




 extern   void ADXL375_Get(void);
#endif



