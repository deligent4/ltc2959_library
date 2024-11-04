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
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../ltc2959/ltc2959.h"
#include "../../ssd1306_oled_lib/inc/ssd1306.h"
#include "../../ssd1306_oled_lib/inc/ssd1306_tests.h"
#include <stdbool.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define _DEBUG
//#define TEST_ACC
#define TEST_TIME_MS		5000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//LTC2959_Config_t *ltc2959_config;
LTC2959_Config_t ltc2959 = {
    .ADC_mode		= 	CTRL_ADC_MODE_CONT_ALT_V_I,
    .GPIO_config 	= 	CTRL_GPIO_CONFIG_ANALOG_INPUT_1560mV,
    .voltage_input 	= 	CTRL_CONFIG_VOLTAGE_INPUT_SENSEN,
    .CC_deadband 	= 	CC_CONFIG_DEADBAND_20,
};

uint32_t tick, prev_tick = 0, i2c_timeout = 10;
uint32_t prev_print_delay = 0, print_delay = 1000;
uint16_t blink_delay = 100;
uint32_t charge = 0;
int32_t current = 0, voltage = 0;
uint16_t temp = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void LTC2959_Device_Config(void);

void myOLED_char(uint16_t cursorX, uint16_t cursorY, char* data);
void myOLED_float(uint16_t cursorX, uint16_t cursorY, float data);
void myOLED_int(uint16_t cursorX, uint16_t cursorY, uint16_t data);
void myOLED_int8(uint16_t cursorX, uint16_t cursorY, uint8_t data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 1000);
  return ch;
}

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
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("LTC2959 Begin\n\r");
  while(HAL_I2C_IsDeviceReady(&LTC2959_I2C_PORT, LTC2959_I2C_ADDR, 100, 1000) != HAL_OK);	// wait for it to come alive
  LTC2959_Init(&ltc2959);
  HAL_Delay(10);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  tick = HAL_GetTick();

	  if(tick - prev_tick >= blink_delay){
		  prev_tick = tick;
		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  }
	  if(tick - prev_print_delay >= print_delay){
		  voltage = LTC2959_Get_Voltage();
		  printf("LTC2959_Voltage = %ld\n\r", voltage);
		  current = LTC2959_Get_Current();
		  printf("LTC2959_current = %ld\n\r", current);
		  charge = LTC2959_Get_Acc_Charge();
		  printf("LTC2959_charge = %lu \n\r\v", charge);

		  prev_print_delay = tick;
	  }
#ifdef TEST_ACC
	  if(tick >= TEST_TIME_MS){
//		  printf("tick %ld\n\r", tick);
		  LTC2959_Set_Do_Not_Count(false); 	// Stop the counting after
		  HAL_Delay(1000);
	  }
#endif

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
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

/* USER CODE BEGIN 4 */


void myOLED_char(uint16_t cursorX, uint16_t cursorY, char* data){

	ssd1306_SetCursor(cursorX, cursorY);
	ssd1306_WriteString(data, Font_7x10, White);
}

void myOLED_float(uint16_t cursorX, uint16_t cursorY, float data){
	char str_data[10];

	sprintf(str_data, "%.3f", data);
	ssd1306_SetCursor(cursorX, cursorY);
	ssd1306_WriteString(str_data, Font_7x10, White);
}

void myOLED_int(uint16_t cursorX, uint16_t cursorY, uint16_t data){
	char str_data[10];

	sprintf(str_data, "%u", data);
	ssd1306_SetCursor(cursorX, cursorY);
	ssd1306_WriteString(str_data, Font_7x10, White);
}

void myOLED_int8(uint16_t cursorX, uint16_t cursorY, uint8_t data){
	char str_data[10];

	sprintf(str_data, "%d", data);
	ssd1306_SetCursor(cursorX, cursorY);
	ssd1306_WriteString(str_data, Font_7x10, White);
}


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
