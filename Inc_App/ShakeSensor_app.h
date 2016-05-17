#ifndef _SHAKESENSOR
#define _SHAKESENSOR

#include "Main.h"
#include "adxl375.h"
#include "hmc5883l.h"
#include "fm25v10.h"
#include "rtc8564.h"
#include "kfifo.h"



typedef struct KEY_ST
{
    uint8_t  work_state;  //   0 : idle  1: triggered   other:process over
    uint16_t  keep_counter; //  dur    3



}KEY_ST;

extern KEY_ST  K1_Value;


//  part 1     HMC58831     I2C1





// part 2     Out RTC chip   I2C2





// part 3   Out  EEPROM    2  chips     SPI1



//part 4:    ADX375 accelerate   main       L3D20H  compass   auxiliary    SPI2
#define  SPI2_RUN_ADX375      1
#define  SPI2_RUN_L3D20H      2

extern uint8_t   Adx375_trigger_flag;
extern uint32_t Shake_times;





extern void Devices_Init_Total(void);
extern void ADXL375_multRead(ADXL375_TYPE * ptResult);
extern void start_adxl375(void);
extern void   Function_test(void);
extern 	void RTC_Demo_init(void);
extern void eeprom_wrdisable(uint8_t chip);
extern void Rx_CMD_process(void);
extern void  SHK_Sensor_data_Get(void);
extern void  Voltage_Get(void);
extern void  ADXL375_Wr_OneByte(uint8_t Register_CMD,uint8_t value);
extern 	uint8_t    ADXL375_INT1_Judge(void);  
extern  void  ADXL375_INT1_IRQHandler(void);


extern void  SHK_SampleData_TrigEnable(void);
extern void  SHK_Event_SaveEnd(void);
extern void  SHK_Check_EventHppen_Again(void);
extern void  SHK_SaveData_Checking(void);
extern void  SHK_SaveData_Output(uint8_t  Rec_Num); 
extern void  Sample_Get(void);

#endif
