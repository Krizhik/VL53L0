/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
I2C_HandleTypeDef hi2c1;

USART_HandleTypeDef husart1;
GPIO_InitTypeDef a5,a6;
/* USER CODE BEGIN PV */

/* USER CODE END PV */
uint8_t mes[4];
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_Init(void);
void init_both_vl53(void);
void dist_to_mes(void);
/* USER CODE BEGIN PFP */
void marker(uint8_t mes)
{	HAL_USART_Transmit(&husart1,&mes,1,0xFFFF);//маркер
	HAL_Delay(10);}
/* USER CODE END PFP */
uint8_t adr1=0x30,adr2=0x31;
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
	
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
	
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
	
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_Init();
	
	//этот пин объявляем для того что бы в момент\
	инициализации мы могли один выключить, тот на\
	котором осается адрес 0x29
	a5.Pin=GPIO_PIN_5;
	a5.Mode=GPIO_MODE_OUTPUT_PP;
	a5.Speed=GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init (GPIOA,&a5);
	HAL_GPIO_WritePin(GPIOA,a5.Pin,GPIO_PIN_RESET);
	a6.Pin=GPIO_PIN_6;
	HAL_GPIO_Init (GPIOA,&a6);
	HAL_GPIO_WritePin(GPIOA,a6.Pin,GPIO_PIN_RESET);
	
	
	
	/*
	//Включаем первый, определяем его адрес
	HAL_GPIO_WritePin(GPIOA,a5.Pin,1);
	HAL_Delay(10);
	*/
		for(int i=1; i<128; i++)
    {
        uint8_t ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 3, 5);
				if(ret==HAL_OK) {VL53L0X.address=i;break;}
		}
	//если его адрес не 0х35 то делаем его 0х35
	//if(VL53L0X.address!=adr1)
		
	/*
	//HAL_GPIO_WritePin(GPIOA,a5.Pin,0);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOA,a6.Pin,1);
	HAL_Delay(10);
	for(int i=1; i<128; i++)
    {
        uint8_t ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 3, 5);
				if((i!=adr1)&&(ret==HAL_OK)) {VL53L0X.address=i;break;}
		}
	//если его адрес не 0х35 то делаем его 0х35
	if(VL53L0X.address!=adr2)VL53L0X_setAddress(adr2);
	*/
	HAL_GPIO_WritePin(GPIOA,a5.Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,a6.Pin,GPIO_PIN_SET);
	HAL_Delay(10);
	
	init_both_vl53();
  while (1)
  {
		dist_to_mes();
		HAL_Delay(300);
  }
}

void dist_to_mes()
{
	
	VL53L0X.address=adr1;
	uint16_t mes1=VL53L0X_readRangeSingleMillimeters();
	mes[0]=mes1>>2;
	mes[1]=0x20;
	VL53L0X.address=adr2;
	mes1=VL53L0X_readRangeSingleMillimeters();
	mes[2]=mes1>>2;
	mes[3]=0x0a;
	HAL_USART_Transmit(&husart1,mes,4,0xFFFF);
}

//инициализация обоих датчиков
void init_both_vl53()
{
	//ресетим оба датчика
	HAL_GPIO_WritePin(GPIOA,a6.Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,a5.Pin,GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOA,a6.Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,a5.Pin,GPIO_PIN_SET);
	HAL_Delay(10);
	//активируем первый, ресетим второй
	HAL_GPIO_WritePin(GPIOA,a6.Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,a5.Pin,GPIO_PIN_RESET);
	HAL_Delay(10);
	VL53L0X.address=0x29;
	VL53L0X_setAddress(adr1);
	init_VL53L0X();
	HAL_Delay(10);
	//включаем второго мсье
	HAL_GPIO_WritePin(GPIOA,a5.Pin,GPIO_PIN_SET);
	HAL_Delay(10);
	VL53L0X.address=0x29;
	VL53L0X_setAddress(adr2);
	init_VL53L0X();
	HAL_Delay(10);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  husart1.Instance = USART1;
  husart1.Init.BaudRate = 115200;
  husart1.Init.WordLength = USART_WORDLENGTH_8B;
  husart1.Init.StopBits = USART_STOPBITS_1;
  husart1.Init.Parity = USART_PARITY_NONE;
  husart1.Init.Mode = USART_MODE_TX_RX;
  husart1.Init.CLKPolarity = USART_POLARITY_LOW;
  husart1.Init.CLKPhase = USART_PHASE_1EDGE;
  husart1.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  if (HAL_USART_Init(&husart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
