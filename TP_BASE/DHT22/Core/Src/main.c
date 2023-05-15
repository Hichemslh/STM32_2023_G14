/* USER CODE BEGIN Header */
/*
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lib_lcd.h"
#include <stdio.h>
 //pour creer des microsecondes
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static rgb_lcd lcdData;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

float Temperature , Humidite ;
uint16_t size , RH = 0, TEMP = 0;
uint8_t dataH1;
uint8_t dataH2;
uint8_t dataT1;
uint8_t dataT2;
uint8_t SUM;
uint8_t check;
char bufRH[20]; //pour stocker une valeur et l'afficher sur le LCD
char bufT[20]; //pour stocker une valeur et l'afficher sur le LCD
uint8_t Data[256];
uint8_t Data36[256];
uint16_t size1;
uint16_t size2;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint32_t DWT_Delay_Init(void)
{
    /* Disable TRC */
    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
    /* Enable TRC */
    CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;

    /* Disable clock cycle counter */
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
    /* Enable  clock cycle counter */
    DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk; //0x00000001;

    /* Reset the clock cycle counter value */
    DWT->CYCCNT = 0;

    /* 3 NO OPERATION instructions */
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");

    /* Check if clock cycle counter has started */
    if(DWT->CYCCNT)
    {
       return 0; /*clock cycle counter started*/
    }
    else
    {
      return 1; /*clock cycle counter not started*/
    }
}

// This Function Provides Delay In Microseconds Using DWT

__STATIC_INLINE void DWT_Delay_us(volatile uint32_t au32_microseconds)
{
  uint32_t au32_initial_ticks = DWT->CYCCNT;
  uint32_t au32_ticks = (HAL_RCC_GetHCLKFreq() / 1000000);
  au32_microseconds *= au32_ticks;
  while ((DWT->CYCCNT - au32_initial_ticks) < au32_microseconds-au32_ticks);
}

// This Function Provides Delay In Milliseconds Using DWT

__STATIC_INLINE void DWT_Delay_ms(volatile uint32_t au32_milliseconds)
{
  uint32_t au32_initial_ticks = DWT->CYCCNT;
  uint32_t au32_ticks = (HAL_RCC_GetHCLKFreq() / 1000);
  au32_milliseconds *= au32_ticks;
  while ((DWT->CYCCNT - au32_initial_ticks) < au32_milliseconds);
}


#define DHT22_PORT GPIOB
#define DHT22_PIN GPIO_PIN_4

/*uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t SUM, RH, TEMP;*/

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Read_data (uint8_t *data)
  {
  	int i, k;
  	for (i=0;i<8;i++)
  	{
  		if (HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET)
  		{
  			(*data)&= ~(1<<(7-i)); //data bit is 0
  			while(!(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)));
  			DWT_Delay_us(40);
  		}
  		else                       //data bit is 1
  		{
  			(*data)|= (1<<(7-i));
  			for (k=0;k<1000;k++)
  			{
  				if (HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET)
  				  {
  				  	break;
  				  }
  			}
  			while(!(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)));
  			DWT_Delay_us(40);
  		}
  	 }
  }



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

  int k = 0; //variable pour les boucles while

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
  MX_USART2_UART_Init();
  //MX_USART1_UART_Init();

  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  DWT_Delay_Init();
  lcd_init(&hi2c1, &lcdData); // initialise le lcd
 // lcd_position(&hi2c1,0,0);
  //lcd_print(&hi2c1,"Bonjour!!");
//  HAL_Delay(3000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* USER CODE END WHILE */

	      /* USER CODE BEGIN 3 */

	  	  /*commence la communication avec le capteur*/

 	  HAL_Delay(1000);// Effectuer des mesures chaque seconde
	  	  Set_Pin_Output(GPIOB, GPIO_PIN_4); //Configuration du pin en sortie
	  	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); //Mise à 0 du pin
	  	  DWT_Delay_us(1000); //Envoi de la premiere partie du signal de commande
	  	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); // Mise a 1 du pin
	  	  DWT_Delay_us(30); //Envoi de la deuxieme partie du signal de commande
	  	  Set_Pin_Input(GPIOB, GPIO_PIN_4); //Configuration du pin en entrée

	  	  // Lecture des données

	  	  while(!(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)));

	  	  for (k=0;k<1000;k++)
	  	  {
	  		  if (HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET)
	  		  {
	  	  	  	break;
	  	  	  }
	  	  }

	  	  while(!(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)));
	  	  DWT_Delay_us(40);

	  	  Read_data(&dataH1); //Lecture du premier octet d'humidité
	  	  Read_data(&dataH2); //Lecture du deuxieme octet d'humidité
	  	  Read_data(&dataT1); //Lecture du premier octet de temperature
	  	  Read_data(&dataT2); //Lecture du deuxieme octet de temperature
	  	  Read_data(&SUM);  //Lecture d'octet de sécurité

	  	  check = dataH1 + dataH2 + dataT1 + dataT2;


	  	  RH = (dataH1<<8) | dataH2;  //Concatenation des octets d'humidité
	  	  TEMP = (dataT1<<8) | dataT2; //Concatenation des octets de temperature

	  	  Humidite = (int) RH / 10.0; //Les valeurs recues sont egales à fois les valeurs reelles
	  	  Temperature = (int) TEMP / 10.0;

	  	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); //Preparation pour lecture suivante

	  	 //Affichage sur LCD

	  	  size=sprintf((char *)Data,"HUMIDITE : %.1f", Humidite);

	  	  lcd_position(&hi2c1,0,0);
	  	  lcd_print(&hi2c1,(char*)Data);
	  	  lcd_print(&hi2c1,"%");
	  	  size=  sprintf((char *)Data, "TEMP :    %.1f C", Temperature);

	  	  lcd_position(&hi2c1,0,1);
	  	  lcd_print(&hi2c1,(char*)Data);
	    }
	    /* USER CODE END 3 */
} //

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
