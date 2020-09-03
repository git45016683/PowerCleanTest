#ifndef __SHT30_H
#define	__SHT30_H


//#include "stm32f10x.h"
#include "stm32f1xx_hal.h"
#include "main.h"
#include "stdint.h"

void Sht30_Init(void);
void Sht30_Init(void);
void Sht30_WriteCmd(uint8_t msb,uint8_t lsb);
void Sht30_ReadData(void);
void Convert_sht30(void);

#endif
