/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
//#include "stm32f4xx_dma.h"

#include <stdio.h>

u16 readADC1(u8 channel);
void ADC_Configuration(void);
