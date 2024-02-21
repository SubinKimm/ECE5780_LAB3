/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

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
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	
  /* USER CODE BEGIN 1 */
HAL_Init();
	
SystemClock_Config(); 


  /* USER CODE END 1 */

__HAL_RCC_GPIOC_CLK_ENABLE(); // Enable the GPIOC clock in the RCC
	
GPIO_InitTypeDef initStr1 = { GPIO_PIN_8 | GPIO_PIN_9,
GPIO_MODE_OUTPUT_PP,
GPIO_SPEED_FREQ_LOW,
GPIO_NOPULL};
HAL_GPIO_Init(GPIOC, &initStr1); // Initialize pins PC6, PC7, PC8, and  PC9
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); // Start PC8 high


GPIO_InitTypeDef initStr2 = { GPIO_PIN_6 | GPIO_PIN_7,
GPIO_MODE_AF_PP,
GPIO_SPEED_FREQ_LOW,
GPIO_NOPULL};
HAL_GPIO_Init(GPIOC, &initStr2); // Initialize pins PC6, PC7 and configure alternate function mode
//GPIO_PIN_6 | GPIO_PIN_7 |
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  
RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // GPIOC Enable
RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	
TIM2->PSC = 7999; // 7999 ; 1kHz

TIM2->ARR = 250; // 250 ; 4Hz
	
TIM2->DIER |= (1 << 0); // Enable the update interrupt

TIM2->CR1 |= (1 << 0); // Before the timer can operate, we must enable this bit.

	

NVIC_EnableIRQ(15); // TIM2_IRQn

// 

RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

TIM3->PSC = 7; // 100kHz

TIM3->ARR = 1250; // 800Hz

TIM3->CCMR1 &= ~(1 << 0)|(1 << 1); // CC1S[0-1] CC1 channel is configured as output.
TIM3->CCMR1 &= ~(1 << 8)|(1 << 9); // CC2S(8-9) CC2 channel is configured as output.

TIM3->CCMR1 |= (1 << 4)|(1 << 5)|(1 << 6); // OC1M(4-6) Set output channel 1 to PWM mode2.

TIM3->CCMR1 &= ~(1 << 12); // OC2M(12-14) Set channel 2 to PWM mode1.
TIM3->CCMR1 |= (1 << 13);
TIM3->CCMR1 |= (1 << 14);


TIM3->CCMR1 |= (1 << 11); // OC2PE(11) Output compare 2 preload enable.
TIM3->CCMR1 |= (1 << 3); // OC1PE(3) Output compare 1 preload enable.

TIM3->CCER |= (1 << 0); // CC1E(0) Capture/Compare 1 output enable.
TIM3->CCER |= (1 << 4); // CC2E(4) Capture/Compare 2 output enable.

TIM3->CCR1 = 1000; // 250 1200
TIM3->CCR2 = 1000; // 250 20

GPIOC->AFR[0] &= ~GPIO_AFRL_AFSEL6; // (0 << 24)|(0 << 25)|(0 << 26)|(0 << 27);
GPIOC->AFR[0] &= ~GPIO_AFRL_AFSEL7;

TIM3->CR1 |= TIM_CR1_CEN; // Enable TIM3
//NVIC_EnableIRQ(16); // TIM3_IRQn


  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

//volatile int j;
void TIM2_IRQHandler(void) {

	GPIOC->ODR ^= (GPIO_ODR_8)|(GPIO_ODR_9);
	
	//for(j = 0; j <1500000; j++) {}

	TIM2->SR &= ~(1 << 0);
	
	
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
