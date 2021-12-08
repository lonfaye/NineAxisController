#ifndef __DELAY_H
#define __DELAY_H 			   
//#include "sys.h"
#include <stdint.h>
//#include "stm32f1xx_hal.h"



#define u32 uint32_t
#define u16 uint16_t
#define u8 uint8_t

void delay_init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);

//void RCCdelay_us(uint32_t udelay);
//void delay_us(uint32_t t);
//void delay_ms(uint32_t t);
#endif





























