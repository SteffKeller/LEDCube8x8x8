/**
  ******************************************************************************
  * @file    GPIO/IOToggle/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    13-April-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "cube.h"

/** @addtogroup STM32F4xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup IOToggle
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint16_t delay_ticks;
volatile uint8_t tick;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  *
void HardFault_Handler(void)
{
   Go to infinite loop when Hard Fault exception occurs
  while (1)
  {
  }
}
*/
/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	static uint8_t ms10 = 0;
	//GPIOD->ODR ^= GPIO_Pin_3;
	ms10++;
	tick ++;
	if(delay_ticks) delay_ticks--;

	if(ms10 == 60) // 10ms vorbei
		{
			ms10 =0;
			process_keys(); // Tasten entprellen
		}
}

void TIM2_IRQHandler(void)
{
	static uint8_t current_layer = 8;
	int i;
	uint16_t temp_port;

  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
   TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
   current_layer--;
		
	GPIO_Write(GPIOC, 0x00); // Turn all cathode layers off. (all transistors off)
	GPIO_SetBits(GPIOA, GPIO_Pin_11); // Set OE high, disabling all outputs on latch array

	// Loop through all 8 bytes of data in the current layer
	// and latch it onto the cube.
	for (i = 0; i < 8; i++)
	{
		// Increment the latch address chip, 74HC138, to create
		// a rising edge (LOW to HIGH) on the current latch.
		temp_port = GPIO_ReadOutputData(GPIOA) & 0xf8ff;
		GPIO_Write(GPIOA, (i<<8) | temp_port);

		temp_port = GPIO_ReadOutputData(GPIOA) & 0xFF00;
		// Set the data on the data-bus of the latch array.
		GPIO_Write(GPIOA, cube[current_layer][i] | temp_port);


	}

	GPIO_ResetBits(GPIOA, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10); // Set OE high, disabling all outputs on latch array
	GPIO_ResetBits(GPIOA, GPIO_Pin_11); // Set OE low, enabling outputs on the latch array
	GPIO_Write(GPIOC, 0x01 << current_layer); // Transistor ON for current layer
	// Increment the curren_layer counter so that the next layer is
	// drawn the next time this function runs.

	// We want to count from 0-7, so set it to 0 when it reaches 8.
	if (current_layer == 0) current_layer = 8;
  }
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
