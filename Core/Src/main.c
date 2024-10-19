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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ssd1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ESP32_AT_UART_HANDLE &huart1
#define ESP32_AT_UART_INSTANCE USART1
#define RX_BUFF_SIZE 128
#define OLED_LINE_LENGTH 21
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t echo_off[] = "ATE0\r\n";
uint8_t init_ble[] = "AT+BLEINIT=2\r\n";
uint8_t ble_addr[] = "AT+BLEADDR?\r\n";
uint8_t ble_adv_param[] = "AT+BLEADVPARAM=50,50,0,0,7,0,,\r\n";
uint8_t ble_adv_data[] =
		"AT+BLEADVDATA=\"0201060A09457370726573736966030302A0\"\r\n";
uint8_t ble_adv[] = "AT+BLEADVSTART\r\n";

uint8_t rx_terminal_message[RX_BUFF_SIZE];
uint8_t rx_terminal_message_copy[RX_BUFF_SIZE];
volatile uint16_t rx_length;
char lcd_line[OLED_LINE_LENGTH + 1];
volatile uint8_t esp32c6at_data_received_flag = 0;
char *text_message_ptr;

volatile uint8_t green_button_flag = 0;
volatile uint8_t yellow_button_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
char* ParseAtResponse(char *at_string);
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
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	ssd1306_Init();
	ssd1306_Fill(Black);
	ssd1306_SetCursor(22, 2);
	ssd1306_WriteString("ufnalski.edu.pl", Font_6x8, White);
	ssd1306_SetCursor(10, 17);
	ssd1306_WriteString("ESP32C6-AT + STM32", Font_6x8, White);
	ssd1306_SetCursor(5, 27);
	ssd1306_WriteString("----  BLE demo  ----", Font_6x8, White);
	ssd1306_SetCursor(2, 42);
	ssd1306_WriteString("RX from XCover 5:", Font_6x8, White);
	ssd1306_UpdateScreen();

	HAL_UART_Transmit(ESP32_AT_UART_HANDLE, echo_off, sizeof(echo_off) - 1,
			100);
	HAL_Delay(100);
	HAL_UART_Transmit(ESP32_AT_UART_HANDLE, init_ble, sizeof(init_ble) - 1,
			100);
	HAL_Delay(100);
	HAL_UART_Transmit(ESP32_AT_UART_HANDLE, ble_addr, sizeof(ble_addr) - 1,
			100);
	HAL_Delay(100);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		if (green_button_flag == 1)
		{
			green_button_flag = 0;
			HAL_UART_Transmit(ESP32_AT_UART_HANDLE, ble_adv_param,
					sizeof(ble_adv_param) - 1, 100);
			HAL_Delay(200);
			HAL_UART_Transmit(ESP32_AT_UART_HANDLE, ble_adv_data,
					sizeof(ble_adv_data) - 1, 100);
			HAL_Delay(200);
			HAL_UART_Transmit(ESP32_AT_UART_HANDLE, ble_adv,
					sizeof(ble_adv) - 1, 100);
			HAL_Delay(200);
		}

		if (yellow_button_flag == 1)
		{
			yellow_button_flag = 0;
			memset(rx_terminal_message, 0x00, RX_BUFF_SIZE);
			HAL_UARTEx_ReceiveToIdle_IT(ESP32_AT_UART_HANDLE,
					rx_terminal_message, RX_BUFF_SIZE);
		}

		if (esp32c6at_data_received_flag == 1)
		{
			esp32c6at_data_received_flag = 0;

			memcpy((char*) rx_terminal_message_copy,
					(char*) rx_terminal_message, RX_BUFF_SIZE);
			rx_terminal_message_copy[0] = ' ';
			text_message_ptr = ParseAtResponse(
					(char*) rx_terminal_message_copy);

			if (text_message_ptr != NULL)
			{
				memcpy(lcd_line,
						(char*) rx_terminal_message
								+ (text_message_ptr
										- (char*) rx_terminal_message_copy),
						OLED_LINE_LENGTH);
				lcd_line[OLED_LINE_LENGTH] = '\0';
				ssd1306_SetCursor(2, 53);
				ssd1306_WriteString("                     ", Font_6x8, White);
				ssd1306_UpdateScreen();
				ssd1306_SetCursor(2, 53);
				ssd1306_WriteString(lcd_line, Font_6x8, White);
				ssd1306_UpdateScreen();
			}
			else
			{
				ssd1306_SetCursor(2, 53);
				ssd1306_WriteString("ERROR!               ", Font_6x8, White);
				ssd1306_UpdateScreen();
			}

			memset(rx_terminal_message, 0x00, RX_BUFF_SIZE);
			HAL_UARTEx_ReceiveToIdle_IT(ESP32_AT_UART_HANDLE,
					rx_terminal_message, RX_BUFF_SIZE);
		}

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
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
	RCC_OscInitStruct.PLL.PLLN = 85;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GREEN_BUTTON_Pin)
	{
		green_button_flag = 1;
	}

	if (GPIO_Pin == YELLOW_BUTTON_Pin)
	{
		yellow_button_flag = 1;
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == ESP32_AT_UART_INSTANCE)
	{
		esp32c6at_data_received_flag = 1;
		rx_length = Size;
	}
}

char* ParseAtResponse(char *at_string)
{
	char *ptr = NULL;
	uint8_t idx = 0;
	ptr = strtok(at_string, ",");
	while (ptr != NULL)
	{
		idx++;
		if (idx == 5)
		{
			return ptr;
		}
		ptr = strtok(NULL, ",");
	}
	return NULL;
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
