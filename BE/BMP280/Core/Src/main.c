/* USER CODE BEGIN Header */
/**
  ****************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ****************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ****************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "lib_lcd.h"
#include <stdio.h>
#include "bmp280.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
BMP280_HandleTypedef bmp280;
static rgb_lcd lcdData;

float pressure, temperature, humidity;
int i=1;

uint16_t size;
uint8_t Data[4];
uint8_t Data1[4];

uint8_t Recep[4];

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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{ if(i==1)  {
			lcd_position(&hi2c1,11,0);
		    lcd_print(&hi2c1,(char*)Recep);
		    lcd_position(&hi2c1,15,0);
		    lcd_print(&hi2c1,"%");
		i=2;
			}
	else if(i==2) {

					lcd_position(&hi2c1,11,1);
					lcd_print(&hi2c1,(char*)Recep);

					lcd_position(&hi2c1,15,1);
					lcd_print(&hi2c1,"C");
					i=1;
			}

	}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{ 		//  HAL_UART_Transmit_DMA(&huart1, Data1, 4);

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();

  lcd_init(&hi2c1, &lcdData); // initialise le lcd
  lcd_position(&hi2c1,0,0);
   lcd_print(&hi2c1,"Humidite ");;
   lcd_position(&hi2c1,0,1);
   lcd_print(&hi2c1,"Temperature ");

   HAL_UART_Receive_DMA (&huart1, Recep, 4);
  /* USER CODE END 2 */
  /* USER CODE BEGIN 2 */
  bmp280_init_default_params(&bmp280.params);
  bmp280.addr = BMP280_I2C_ADDRESS_1;
  bmp280.i2c = &hi2c1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  bmp280_init(&bmp280, &bmp280.params);
  while (1)
  {
	  /* USER CODE END WHILE */
	  	  while (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) {
	  	  			size = sprintf((char *)Data,"Temperature/pressure reading failed\n\r");
	  	  			HAL_UART_Transmit_DMA(&huart1, Data, size);
	  	  			HAL_Delay(2000);
	  	  		}
	  	  pressure=pressure/100.;
	  	 // size = sprintf((char *)Tx,"htgmnj 🙂");
	  	  	//  HAL_UART_Transmit(&huart1, Tx, size, 1000);

	  	  size = sprintf((char *)Data,"%.1f", temperature);
	  	  HAL_UART_Transmit(&huart1, Data, 4,1000);
	  	 HAL_Delay(100);

	  	  size = sprintf((char *)Data,"%.f", pressure);
	  	  if (size==3) {
	  		HAL_UART_Transmit(&huart1,"0",1,1000);
	  		HAL_UART_Transmit(&huart1, Data, 3,1000);
	  	  }
	  	  else {
		  	  HAL_UART_Transmit(&huart1, Data, 4,1000);

	  	  }
	  	/*  lcd_position(&hi2c1,0,1);
	  	  lcd_print(&hi2c1,Data);*/

	      /* USER CODE BEGIN 3 */
	  	 HAL_Delay(1000);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
