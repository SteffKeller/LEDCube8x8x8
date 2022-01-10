/**
  ******************************************************************************
  * @file    keys.c
  * @author  chstke1
  * @version V1.0
  * @date    10.12.12
  * @brief  Routinen für die Entprellung und universelle 
			Tastenabfrage mit bis zu 16 Tasten an einem Port
  ******************************************************************************/

#include "keys.h"

volatile uint16_t key_state;				// debounced and inverted key state
volatile uint16_t key_press;				// key press detect
volatile uint16_t key_rpt;				// key uint16_t press and repeat


void process_keys(void)		// every 10ms
{
  static uint16_t ct0, ct1, rpt;
  uint16_t i;

  i = key_state ^ ~KEY_INPUT;		// key changed ?
  ct0 = ~( ct0 & i );                             // reset or count ct0
  ct1 = ct0 ^ (ct1 & i);                          // reset or count ct1
  i &= ct0 & ct1;                                 // count until roll over ?
  key_state ^= i;                                 // then toggle debounced state
  key_press |= key_state & i;                     // 0->1: key press detect

  if( (key_state & REPEAT_MASK) == 0 )	// check repeat function
     rpt = REPEAT_START;		// start delay
  if( --rpt == 0 ){
    rpt = REPEAT_NEXT;			// repeat delay
    key_rpt |= key_state & REPEAT_MASK;
  }
}


uint16_t get_key_press( uint16_t key_mask )
{

	__disable_irq();
	key_mask &= key_press;                        // read key(s)
	key_press ^= key_mask;                        // clear key(s)
	__enable_irq();
	return key_mask;
}


uint16_t get_key_rpt( uint16_t key_mask )
{

	__disable_irq();
	key_mask &= key_rpt;                        	// read key(s)
	key_rpt ^= key_mask;                        	// clear key(s)
	__enable_irq();
	return key_mask;
}


uint16_t get_key_short( uint16_t key_mask )
{
	uint16_t x;

	__disable_irq();
  x = get_key_press( ~key_state & key_mask );
  __enable_irq();
  return x;
}


uint16_t get_key_long( uint16_t key_mask )
{
  return get_key_press( get_key_rpt( key_mask ));
}

/* Beispiel:
Timer2 Initialisieren: 

/*
 * Name         : TIM2_Init
 * Synopsis     : void TIM2_Init(void)
 * Description  : Timer 2 für LED Dimmung initialisieren-> LEDPWM Timer 2 Channel 3
 * 
 *
void TIM2_Init(void)
{
// Clocks einschalten
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

// Timer Zeiten konfigurieren
	TIM_TimeBase_InitStructure.TIM_ClockDivision             = TIM_CKD_DIV1;
	TIM_TimeBase_InitStructure.TIM_CounterMode               = TIM_CounterMode_Up;
	TIM_TimeBase_InitStructure.TIM_Period                    = 65535; // 16Bit PWM Abstufung
	TIM_TimeBase_InitStructure.TIM_Prescaler                 = 7; // +1 Vorteiler
	TIM_TimeBaseInit(TIM2, &TIM_TimeBase_InitStructure);
// Timer Mode konfigurieren
	TIM_OC_InitStructure.TIM_OCMode                          = TIM_OCMode_PWM1;
	TIM_OC_InitStructure.TIM_OCIdleState                     = TIM_OCIdleState_Reset;
	TIM_OC_InitStructure.TIM_OCNIdleState                    = TIM_OCNIdleState_Set;
	TIM_OC_InitStructure.TIM_OCPolarity                      = TIM_OCPolarity_High;
	TIM_OC_InitStructure.TIM_OCNPolarity                     = TIM_OCNPolarity_High;
	TIM_OC_InitStructure.TIM_OutputState                     = TIM_OutputState_Enable;
	TIM_OC_InitStructure.TIM_OutputNState                    = TIM_OutputNState_Disable;
	TIM_OC_InitStructure.TIM_Pulse                           = 0;
	TIM_OC3Init(TIM2, &TIM_OC_InitStructure); 
	TIM_Cmd(TIM2, ENABLE); // Timer enable

}

Interrupt: 
/**
* @brief  This function handles SysTick Handler.
* @param  None
* @retval None
*
void SysTick_Handler(void)
{
	static uint8_t ms10 = 0;
	ms10++;
	if(delay_ticks) delay_ticks--;
	if(ms10 == 10)
	{
		ms10 =0;
		process_keys();
	}
}


Tastenabfrage im Main Programm:

		if( get_key_press (1<<TASTER2 )|| get_key_rpt (1<<TASTER2 ))
		{
			GPIOC->ODR ^= GPIO_Pin_12;
		}
		if( get_key_long (1<<TASTER1 ))
		{
			GPIOC->ODR ^= GPIO_Pin_12;
		}
*/
