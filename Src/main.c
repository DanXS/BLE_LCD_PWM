/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "lcd.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

// Used to process commands from BLE Commands via UART

#define RX_CMD 1
#define RX_CMD_LENGTH 2
#define RX_CMD_DATA 3
#define RX_CMD_READY 4
#define RX_CMD_BUFFER_SIZE 255
#define RX_BUFFER_SIZE 1024
#define LCD_LINE_LENGTH 16

int rx_state = RX_CMD;
int rx_write_index = 0;
int rx_read_index = 0;
int rx_cmd_index = 0;
int rx_length = 0;
uint8_t rx_byte;
uint8_t rx_cmd;
uint8_t rx_cmd_buffer[RX_CMD_BUFFER_SIZE];
uint8_t rx_buffer[RX_BUFFER_SIZE];

char lcd_line1[LCD_LINE_LENGTH];
char lcd_line2[LCD_LINE_LENGTH];

// Pulse Width for PWM
#define TRUE 1
#define FALSE 0
uint16_t pulsewidth[4] = {0, 0, 0, 0};
int update_motors = TRUE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_NVIC_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

int parseCmd(void);
void processCmd(void);
void addToCircularRxBuffer(uint8_t);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	lcd_line1[0] = '\0';
	lcd_line2[0] = '\0';
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();

  /* USER CODE BEGIN 2 */

  // LCD
  initLCD(&hi2c2);
  //char* msg = "LCD Active";
  //lcd_display_string(&hi2c2, msg, 1);
  lcd_clear(&hi2c2);
  // UART
  while(huart1.gState == HAL_UART_STATE_BUSY_RX || huart1.gState == HAL_UART_STATE_BUSY_TX)
	  ;
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

  // PWM Timer
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  while(!parseCmd())
		  ;
	  processCmd();
	  if (rx_cmd == 2) {
		  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, (uint16_t)(1000+pulsewidth[0]));
		  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, (uint16_t)(1000+pulsewidth[1]));
		  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, (uint16_t)(1000+pulsewidth[2]));
		  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, (uint16_t)(1000+pulsewidth[3]));
	  }
	  else if (rx_cmd == 3) {
		  lcd_display_string(&hi2c2, &lcd_line1[0], 1);
		  lcd_display_string(&hi2c2, &lcd_line2[0], 2);
	  }
	  else if (rx_cmd == 4) {
		  lcd_line1[0] = '\0';
		  lcd_line2[0] = '\0';
		  lcd_clear(&hi2c2);
	  }

	  /*
	for (uint32_t i = 0; i < 1000; i++) {
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 1000+i);
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 2000-(2*i % 1000));
		HAL_Delay(10);
		//char countStr[10];
		//sprintf(countStr, "%d", pulsewidth);
		//lcd_display_string(&hi2c2, countStr, 1);
	}
	*/

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** NVIC Configuration
*/
static void MX_NVIC_Init(void)
{
  /* I2C2_EV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C2_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
  /* I2C2_ER_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C2_ER_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Pinout Configuration
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

void unknownCmd() {
	char* error = "unknown cmd";
	lcd_clear(&hi2c2);
	lcd_display_string(&hi2c2, error, 1);
}

void processCmd() {
	if (rx_cmd == 2) {
		uint8_t servo = rx_cmd_buffer[0];
		uint8_t valueHigh = rx_cmd_buffer[1];
		uint8_t valueLow = rx_cmd_buffer[2];
		uint16_t value = ((((uint16_t)valueHigh) << 8) & 0xFF00) | (((uint16_t)valueLow) & 0xFF);
		pulsewidth[servo] = value;
	}
	else if (rx_cmd == 3) {
		uint8_t line = (rx_cmd_buffer[0]+1);
		char* str = (char*)(&rx_cmd_buffer[1]);
		char* dest = &lcd_line1[0];
		if (line == 2) {
			dest = &lcd_line2[0];
		}
		while(*str != '\0')
			*dest++ = *str++;
		*dest++ = '\0';
	}
	if (rx_cmd < 1 || rx_cmd > 4) {
		unknownCmd();
	}
}

int parseCmd() {
	if (rx_read_index == rx_write_index) {
		// underflow - (no data to read)
		return FALSE;
	}
	uint8_t byte = rx_buffer[rx_read_index++];
	if (rx_read_index == RX_BUFFER_SIZE) {
		rx_read_index = 0;
	}
	switch(rx_state) {
	case RX_CMD:
		rx_cmd = byte;
		rx_state++;
		break;
	case RX_CMD_LENGTH:
		rx_length = byte;
		rx_state++;
		if (rx_length == 0) {
			rx_state = RX_CMD;
			return TRUE;
		}
		break;
	case RX_CMD_DATA:
		rx_length--;
		if (rx_length == 0) {
			rx_length = 0;
			rx_cmd_index = 0;
			rx_state = RX_CMD;
			return TRUE;
		}
		else {
			rx_cmd_buffer[rx_cmd_index++] = byte;
			// Place a zero after in case its a string that needs null terminating
			rx_cmd_buffer[rx_cmd_index] = 0x00;
		}
		break;
	default:
		break;
	}
	return FALSE;
}

void rxBufferOverflow(uint8_t rx_byte) {
	HAL_NVIC_DisableIRQ(USART1_IRQn);
	while(rx_write_index == rx_read_index)
		HAL_Delay(1);
	addToCircularRxBuffer(rx_byte);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void addToCircularRxBuffer(uint8_t rx_byte) {
	rx_buffer[rx_write_index++] = rx_byte;
	if (rx_write_index == RX_BUFFER_SIZE) {
		rx_write_index = 0;
	}
	if (rx_write_index == rx_read_index) {
		rxBufferOverflow(rx_byte);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		//HAL_UART_Transmit(huart, &rx_byte, 1, 0xFFF);
		addToCircularRxBuffer(rx_byte);
		HAL_UART_Receive_IT(huart, &rx_byte, 1);
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
