#ifndef __HMC55831_H__
#define __HMC55831_H__

#define HMC5883L_WR       0x3C
#define HMC5883L_RE       0x3D




/*---------------------* 
*  HMC5883L内部寄存器  * 
*----------------------*/
#define HMC5883L_REGA   0x00
#define HMC5883L_REGB   0x01
#define HMC5883L_MODE   0x02
#define HMC5883L_HX_H   0x03
#define HMC5883L_HX_L   0x04 
#define HMC5883L_HZ_H   0x05
#define HMC5883L_HZ_L   0x06
#define HMC5883L_HY_H   0x07
#define HMC5883L_HY_L   0x08
#define HMC5883L_STATE  0x09
#define HMC5883L_IRA    0x0a    //读序列号使用的寄存器
#define HMC5883L_IRB    0x0b
#define HMC5883L_IRC    0x0c 

#define HMC5883L_OFFSET_X   (9)
#define HMC5883L_OFFSET_Y   (149)
//比例因子
#define HMC5883L_GAIN_X     1f
#define HMC5883L_GAIN_Y     10403     //实际1.04037582,这里便于整除 


typedef struct
{
    int16_t  hx;
    int16_t  hy;
    int16_t  hz;
    uint16_t ha;   
}HMC5883L_TYPE;
extern HMC5883L_TYPE hmcdata;

void  HMC5883L_Init(void);


#endif

