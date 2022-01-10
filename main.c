/**
 ******************************************************************************
 * @file    LED Cube
 * @author  Steff
 * @version V0.1
 * @date    9.12.12
 * @brief   Main program body
 ******************************************************************************
 * @attention

 */

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx.h"
#include "3d.h"
#include "draw_3d.h"
#include "cube.h"
#include "effect.h"
#include "launch_effect.h"
#include "draw.h"
#include <stdlib.h>
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_flash.h"
#include "eeprom.h"
#include "misc.h"
#include "keys.h"
//#include "adc.h"

#define CUBE_SIZE 8
#define CUBE_BYTES 64

#define ADC1_DR_ADDRESS    ((uint32_t)0x4001204C)
#define FILTER_SHIFT 4    // k für Tp Filter
volatile unsigned char cube[CUBE_SIZE][CUBE_SIZE];

// Framebuffer
// Animations that take a lot of time to compute are temporarily
// stored to this array, then loaded into cube[8][8] when the image
// is ready to be displayed
volatile unsigned char fb[CUBE_SIZE][CUBE_SIZE];
// Define USART stuff

//#include "cube.h"
//#include "stm324xg_eval.h"

/* Private typedef -----------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
RCC_ClocksTypeDef RCC_Clocks;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern volatile uint16_t delay_ticks;
extern volatile uint8_t tick;
volatile uint16_t ADC1ConvertedValue = 0;
uint16_t max_sound = 0;
/* Private function prototypes -----------------------------------------------*/
void GPIO_Conf(void);
void TIM2_Config(void);
void TIM3_Config(void);
void PWM_Config(int period);
void PWM_SetDC(uint16_t channel, uint16_t dutycycle);
void ADC3_CH7_DMA_Config(void);
void ADCConvert_Potentiometer(void);
int16_t TpFilter(int16_t filter_input);
//void delay_ms(uint16_t ms);
volatile float f1 = 1.48, f2;

/* Virtual address defined by the user: 0xFFFF value is prohibited */
uint16_t VirtAddVarTab[NB_OF_VAR] = { 0x5555, 0x6666, 0x7777 };
uint16_t pwmDS = 0;

int main(void) {

	int i = 0, j;
	uint16_t max_count = 0;
	volatile uint16_t value;
	//FLASH_OB_BORConfig(OB_BOR_LEVEL2);
	SystemInit();
	SystemCoreClockUpdate();
	RCC_GetClocksFreq(&RCC_Clocks);
	SysTick_Config(SystemCoreClock/666);
	//SysTick_Config(RCC_Clocks.HCLK_Frequency / 18600); //1ms //2100 25000

	TIM2_Config();
	TIM3_Config();
	PWM_Config(100);
	GPIO_Conf();
	ADC3_CH7_DMA_Config();
	ADC_SoftwareStartConv(ADC1);
	FLASH_Unlock();
	EE_Init();

	EE_ReadVariable(VirtAddVarTab[0], &pwmDS);
	//pwmDS = 100;
	PWM_SetDC(3, pwmDS);
	srand(ADC1ConvertedValue);
//	fill(0xff);
	delay_ms(1000);
//	fill(0x00);
	if (get_key_press(1 << TASTER)) // Taste lang gedrückt
			{
		fill(0xff);
		while (!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12)) {
			(pwmDS > 100) ? (pwmDS = 2) : (pwmDS += 2);
			PWM_SetDC(3, pwmDS);
			delay_ms(200);
		}
		delay_ms(500);
		GPIOD->BSRRL = GPIO_Pin_3;
		EE_WriteVariable(VirtAddVarTab[0], pwmDS);
		delay_ms(500);
		GPIOD->BSRRH = GPIO_Pin_3;
	}
	fill(0x00);
	while (1) {
		for(i = 24;i < EFFECTS_TOTAL; i++)
		{
//			delay_ms(10);
//			GPIOD->BSRRL = GPIO_Pin_3;
//			delay_ms(10);
//			GPIOD->BSRRH = GPIO_Pin_3;
//			tube_cube(500,1000);
//			random_pixels(500,1000);
			//launch_effect(i);
			launch_effect(rand()%EFFECTS_TOTAL);
		}
//		if(tick == 4)
//		{
//			max_count ++;
//			value = TpFilter(ADC1ConvertedValue);
//			tick = 0;
//			if ( value > max_sound)
//			{
//				max_sound = value;
//			}
//			if( max_count == 2000)
//			{
//				max_count = 0;
//				max_sound = value;
//			}
//		}
//
//		 if(value > max_sound *0.95)
//		 {
//			 GPIOD->BSRRL = GPIO_Pin_3;
//		 }
//		 else GPIOD->BSRRH = GPIO_Pin_3;
//		 GPIOD->ODR ^= GPIO_Pin_4;
////
//
//// 			GPIO_SetBits(GPIOA, GPIO_Pin_11); // Set OE high, disabling all outputs on latch array
//// 		delay_ms(100);
//// 		GPIO_ResetBits(GPIOA, GPIO_Pin_11);
//// 		delay_ms(100);
//// 				for (i=0; i<EFFECTS_TOTAL; i++)
//// 			launch_effect(i);
////
////     /* Set PG6 and PG8 */
////      GPIOD->BSRRL = GPIO_Pin_3 | GPIO_Pin_4;// | GPIO_Pin_4;
//// 				delay_ms(100);
////     /* Reset PG6 and PG8 */
////     GPIOD->BSRRH = GPIO_Pin_3 | GPIO_Pin_4;
//// 				delay_ms(200);

		
	}
}
	void GPIO_Conf(void) {
		/* GPIOG Periph clock enable */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

		/* PA0-7 Daten Out */
		GPIO_InitStructure.GPIO_Pin = 0xFF;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		/* PC0-7 Ebenen Out */
		GPIO_InitStructure.GPIO_Pin = 0xFF;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
		/* Latch Adressen und Output Enable */
		GPIO_InitStructure.GPIO_Pin = 0x0f00;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		/* LED */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOD, &GPIO_InitStructure);

		/* Taster */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
//	/* MIC in */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
//	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
	}
	/*
	 * Name         : TIM2_Config
	 * Synopsis     : void TIM2_Config(void)
	 * Description  :
	 *
	 */
	void TIM2_Config(void) {
		NVIC_InitTypeDef NVIC_InitStructure;
		/* Enable the TIM2 gloabal Interrupt */
		NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		/* TIM2 clock enable */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		/* Time base configuration */
		TIM_TimeBaseStructure.TIM_Period = 1000 - 1; // 1 MHz down to 1 KHz (1 ms)
		TIM_TimeBaseStructure.TIM_Prescaler = 21 - 1; // 24 MHz Clock down to 1 MHz (adjust per your clock) //240Hz
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
		/* TIM IT enable */
		TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
		/* TIM2 enable counter */
		TIM_Cmd(TIM2, ENABLE);
	}
	void TIM3_Config(void) {

		/* TIM3 clock enable */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		/* GPIOC and GPIOB clock enable */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

		/* GPIOC Configuration: TIM3 CH3 (PC8) */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		/* Connect TIM3 pins to AF2 */
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
	}
	void PWM_Config(int period) {
		uint16_t PrescalerValue = 0;
		/* Compute the prescaler value */
		PrescalerValue = (uint16_t)((SystemCoreClock / 2) / 1000000) - 1;
		/* Time base configuration */
		TIM_TimeBaseStructure.TIM_Period = period;
		TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
		/* PWM1 Mode configuration: Channel1 */
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = 0;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OC1Init(TIM3, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
		/* PWM1 Mode configuration: Channel3 */
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = 0;
		TIM_OC3Init(TIM3, &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
		TIM_ARRPreloadConfig(TIM3, ENABLE);
		/* TIM3 enable counter */
		TIM_Cmd(TIM3, ENABLE);
	}
	void PWM_SetDC(uint16_t channel, uint16_t dutycycle) {
		if (channel == 1) {
			TIM3->CCR1 = dutycycle;
		} else if (channel == 2) {
			TIM3->CCR2 = dutycycle;
		} else if (channel == 3) {
			TIM3->CCR3 = dutycycle;
		} else {
			TIM3->CCR4 = dutycycle;
		}
	}

	void ADC3_CH7_DMA_Config(void) {
		ADC_InitTypeDef ADC_InitStructure;
		ADC_CommonInitTypeDef ADC_CommonInitStructure;
		DMA_InitTypeDef DMA_InitStructure;
		GPIO_InitTypeDef GPIO_InitStructure;

		/* Enable ADC3, DMA2 and GPIO clocks ****************************************/
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOB,
				ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

		/* DMA2 Stream0 channel0 configuration **************************************/
		DMA_InitStructure.DMA_Channel = DMA_Channel_0;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) ADC1_DR_ADDRESS;
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) &ADC1ConvertedValue;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
		DMA_InitStructure.DMA_BufferSize = 1;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
		DMA_InitStructure.DMA_PeripheralDataSize =
				DMA_PeripheralDataSize_HalfWord;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_Init(DMA2_Stream0, &DMA_InitStructure);
		DMA_Cmd(DMA2_Stream0, ENABLE);

		/* Configure ADC3 Channel7 pin as analog input ******************************/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		/* ADC Common Init **********************************************************/
		ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
		ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;
		ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
		ADC_CommonInitStructure.ADC_TwoSamplingDelay =
				ADC_TwoSamplingDelay_5Cycles;
		ADC_CommonInit(&ADC_CommonInitStructure);

		/* ADC3 Init ****************************************************************/
		ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;
		ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
		ADC_InitStructure.ADC_ExternalTrigConvEdge =
				ADC_ExternalTrigConvEdge_None;
		ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
		ADC_InitStructure.ADC_NbrOfConversion = 1;
		ADC_Init(ADC1, &ADC_InitStructure);

		/* ADC3 regular channel7 configuration *************************************/
		ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1,
				ADC_SampleTime_3Cycles);

		/* Enable DMA request after last transfer (Single-ADC mode) */
		ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

		/* Enable ADC3 DMA */
		ADC_DMACmd(ADC1, ENABLE);

		/* Enable ADC3 */
		ADC_Cmd(ADC1, ENABLE);
	}

	int16_t TpFilter(int16_t filter_input) {
		// Parameter K
		// Specify 32-bit integer
		// Specify 16-bit integer

		// Delay element – 32 bits
		// Filter input – 16 bits
		// Filter output – 16 bits
		static int32_t filter_reg = 0;
		// Update filter with current sample.
		filter_reg = filter_reg - (filter_reg >> FILTER_SHIFT) + filter_input;

		// Scale output for unity gain.
		return (filter_reg >> FILTER_SHIFT);

	}

