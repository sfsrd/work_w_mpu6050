/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "stm32f1xx_hal.h"
#include "ssd1306_tests.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "stm32f1xx_hal_uart.h"
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
MPU6050_t MPU6050;
SSD1306_t SSD1306;
HAL_StatusTypeDef status = 1;
uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];
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
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  while (MPU6050_Init(&hi2c1) == 1);
  ssd1306_Reset();
  HAL_Delay(200);

  ssd1306_WriteCommand(0xAE); //display off
  ssd1306_WriteCommand(0x20); //Set Memory Addressing Mode
  ssd1306_WriteCommand(0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
  ssd1306_WriteCommand(0xB0); //Set Page Start Address for Page Addressing Mode,0-7
  ssd1306_WriteCommand(0xC8); //Set COM Output Scan Direction
  ssd1306_WriteCommand(0x00); //---set low column address
  ssd1306_WriteCommand(0x10); //---set high column address
  ssd1306_WriteCommand(0x40); //--set start line address
  ssd1306_WriteCommand(0x81); //--set contrast control register
  ssd1306_WriteCommand(0xFF);
  ssd1306_WriteCommand(0xA1); //--set segment re-map 0 to 127
  ssd1306_WriteCommand(0xA6); //--set normal display
  ssd1306_WriteCommand(0xA8); //--set multiplex ratio(1 to 64)
  ssd1306_WriteCommand(0x3F); //
  ssd1306_WriteCommand(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
  ssd1306_WriteCommand(0xD3); //-set display offset
  ssd1306_WriteCommand(0x00); //-not offset
  ssd1306_WriteCommand(0xD5); //--set display clock divide ratio/oscillator frequency
  ssd1306_WriteCommand(0xF0); //--set divide ratio
  ssd1306_WriteCommand(0xD9); //--set pre-charge period
  ssd1306_WriteCommand(0x22); //
  ssd1306_WriteCommand(0xDA); //--set com pins hardware configuration
  ssd1306_WriteCommand(0x12);
  ssd1306_WriteCommand(0xDB); //--set vcomh
  ssd1306_WriteCommand(0x20); //0x20,0.77xVcc
  ssd1306_WriteCommand(0x8D); //--set DC-DC enable
  ssd1306_WriteCommand(0x14); //
  ssd1306_WriteCommand(0xAF); //--turn on SSD1306 panel


  ssd1306_Fill(Black);

      // Flush buffer to screen
  ssd1306_UpdateScreen();

      // Set default values for screen object
  SSD1306.CurrentX = 0;
  SSD1306.CurrentY = 0;
  SSD1306.Initialized = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	// ssd1306_Fill(Black);
	 SSD1306.CurrentX = 0;
	 SSD1306.CurrentY = 0;
	 MPU6050_Read_All(&hi2c1, &MPU6050);

	 //ssd1306_Fill(White);
	ssd1306_WriteString("Temperature", Font_7x10,White);
	ssd1306_UpdateScreen();

	 HAL_Delay (100);
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
void ssd1306_Reset(void) {
	// CS = High (not selected)
	HAL_GPIO_WritePin(SSD1306_CS_Port, SSD1306_CS_Pin, GPIO_PIN_SET);

	// Reset the OLED
	HAL_GPIO_WritePin(SSD1306_Reset_Port, SSD1306_Reset_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(SSD1306_Reset_Port, SSD1306_Reset_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
}

void ssd1306_WriteCommand(uint8_t byte) {
    HAL_GPIO_WritePin(SSD1306_CS_Port, SSD1306_CS_Pin, GPIO_PIN_RESET); // select OLED
    HAL_Delay(3);
    HAL_GPIO_WritePin(SSD1306_DC_Port, SSD1306_DC_Pin, GPIO_PIN_RESET); // command
    status = HAL_SPI_Transmit(&SSD1306_SPI_PORT, (uint8_t *) &byte, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SSD1306_CS_Port, SSD1306_CS_Pin, GPIO_PIN_SET); // un-select OLED
    HAL_Delay(3);
}

void ssd1306_WriteData(uint8_t* buffer, size_t buff_size) {
    HAL_GPIO_WritePin(SSD1306_CS_Port, SSD1306_CS_Pin, GPIO_PIN_RESET); // select OLED
    HAL_Delay(3);
    HAL_GPIO_WritePin(SSD1306_DC_Port, SSD1306_DC_Pin, GPIO_PIN_SET); // data
    HAL_SPI_Transmit(&SSD1306_SPI_PORT, buffer, buff_size, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SSD1306_CS_Port, SSD1306_CS_Pin, GPIO_PIN_SET); // un-select OLED
    HAL_Delay(3);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
