#ifndef __KFIFO_H_
#define __KFIFO_H_
#include <stdio.h>
#include <string.h>
#include "stdlib.h"
#include <stdint.h>

extern uint16_t datanum;

int init_fifo(void);
unsigned int put_data(uint8_t *str,uint16_t len);
unsigned int get_data(uint8_t *str,uint16_t len);





#endif

