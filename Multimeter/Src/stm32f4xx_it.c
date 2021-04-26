/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
#include "UI_Functions.c"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

void deactivateLEDs(void) {
	//Just turns all leds off before new one is turned on by master function 
	GPIOD -> ODR &= ~GPIO_PIN_8;
	GPIOD -> ODR &= ~GPIO_PIN_9;
	GPIOD -> ODR &= ~GPIO_PIN_10;
	GPIOD -> ODR &= ~GPIO_PIN_11;
	GPIOD -> ODR &= ~GPIO_PIN_12;
	GPIOD -> ODR &= ~GPIO_PIN_13;
	GPIOD -> ODR &= ~GPIO_PIN_14;
	GPIOD -> ODR &= ~GPIO_PIN_15;
}

double getADC() {
  HAL_ADC_Start(&hadc1);
 
  HAL_ADC_PollForConversion(&hadc1, 100);
 
  adcResult = HAL_ADC_GetValue(&hadc1);
 
  HAL_ADC_Stop(&hadc1);

  return adcResult; 
}

double valueToDisplay = 0;

/**
  * @brief This function handles EXTI line[9:5] interrupts.  //Interrupt handler of sw2 and 3
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  deactivateLEDs();
  dispClear();

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  //Detect which switch has been pressed 
	//SW2 - Vdc
		if (GPIOE -> IDR & GPIO_PIN_8) {
			
			//Activate LED
			GPIOD -> ODR |= GPIO_PIN_8;
			
			//Binary output decimal 1 - for Vdc measurment - Voltage
			GPIOB -> ODR |= GPIO_PIN_7;//1
			GPIOB -> ODR &= ~GPIO_PIN_5;//0
			GPIOB -> ODR &= ~GPIO_PIN_4;//0
			
			while (1) {
					//Measure
          valueToDisplay = scaleUpVdc(adcResult());

					//Display
          dispVotlage(valueToDisplay);

		}
		
		/*Vdc eq (x-1.5)/0.15*/
		
		/*Idc eq ((x-1.5)/0.15)/100 CURRENT*/
		
		//SW3 - Idc - Current
		if (GPIOE -> IDR & GPIO_PIN_9) {

      //Activate LED
			GPIOD -> ODR |= GPIO_PIN_9;

			//Binary output decimal 2 - for Current DC
			GPIOB -> ODR &= ~GPIO_PIN_7;//0
			GPIOB -> ODR |= GPIO_PIN_5;//1
			GPIOB -> ODR &= ~GPIO_PIN_4;//0
			
			while (1) {
					//Measure
          valueToDisplay = scaleUpIdc(adcResult());

					//Display
          dispCurrent(valueToDisplay);
		}

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts. //Handler for the rest of the sw's (2+)
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  deactivateLEDs();
  dispClear();

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  //SW4 - Rdc - Resistance 
  if (GPIOE -> IDR & GPIO_PIN_10) {
    GPIOD -> ODR |= GPIO_PIN_10;

    //Activate LED
    GPIOD -> ODR |= GPIO_PIN_10;

    //Binary output decimal 3
    GPIOB -> ODR |= GPIO_PIN_7;//1
    GPIOB -> ODR |= GPIO_PIN_5;//1
    GPIOB -> ODR &= ~GPIO_PIN_4;//0
    
    while (1) {
        //Measure
        valueToDisplay = scaleUpRdc(adcResult());

        //Display
        dispResistance(valueToDisplay);
		}
  }
  
  //SW5
  if (GPIOE -> IDR & GPIO_PIN_11) {
    GPIOD -> ODR |= GPIO_PIN_11;
  }
  
  //SW6
  if (GPIOE -> IDR & GPIO_PIN_12) {
    GPIOD -> ODR |= GPIO_PIN_12;
  }
  
  //SW7
  if (GPIOE -> IDR & GPIO_PIN_13) {
    GPIOD -> ODR |= GPIO_PIN_13;
  }
  
  //SW8
  if (GPIOE -> IDR & GPIO_PIN_14) {
    GPIOD -> ODR |= GPIO_PIN_14;
  }
  
  //SW9
  if (GPIOE -> IDR & GPIO_PIN_15) {
    GPIOD -> ODR |= GPIO_PIN_15;
  }

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
