/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "E220.h"
#include "ssd1306.h"
#include "fonts.h"
#include "foundPoint.h"
#include "RadioCommunication.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define SSD1306_DISPLAY true

#define E220_START_BYTE	0xAA
#define E220_STOP_BYTE	0xBB

_Bool flag_start_recv=false;
uint16_t counterBuffer=0;
uint8_t rx_byte;
uint8_t recvBuffer[100];




uint32_t count;
extern uint32_t count_sys;
_Bool AUX_Flag = false;

char buf[30];
uint8_t recv[50] = {0x00, };
uint8_t recvBuffer[100];



uint8_t comma[5];

uint32_t UART_ERROR = 0;
float d;
float d0;
uint8_t RSSI_d0;
float n;

uint8_t RSSI_recv[3];
uint8_t NumPacket_recv[3];

int i = 0;
int numPackNow = 0;
int numPackNowSend[3];

uint16_t all_cnt = 1;

uint16_t loseCntPer_1 = 0;
uint16_t loseCntPer_2 = 0;

uint16_t count_1 = 0;
uint16_t count_2 = 0;
uint16_t count_3 = 0;

uint16_t lose_count_1 = 0;
uint16_t lose_count_2 = 0;
uint16_t lose_count_3 = 0;

//ДАННЫЕ НА ОТПРАВКУ
uint8_t data_RSSI_0[3] = {0xA0, 0xFF, 0x00};
uint8_t data_RSSI_1[3] = {0xA1, 0xFF, 0x00};
uint8_t data_RSSI_2[3] = {0xA2, 0xFF, 0x00};


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

HAL_UART_StateTypeDef uart_state;
AGRO_HandleTypeDef AGRO_Device;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
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
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  LED_OFF;
  E220_SetMode(DeepSleep); // Режим сна
  E220_WaitReady(); // Ожидание включения
  LED_ON;
  HAL_Delay(500);
  LED_OFF;
  /*Установка настроек*/
  /*Выбор устройства установка адресов и каналов всех используемых устройств*/

  AGRO_Init(AGRO_Device, Device_0);

  // Передатчик 1
  AGRO_Device.AddrDevice_1 = 0x0001;
  AGRO_Device.ChDevice_1 = 0x02;

  // Приемник 1
  AGRO_Device.AddrDevice_0 = 0x0003;
  AGRO_Device.ChDevice_0 = 0x04;

  E220_SetDefaultSettings(AGRO_Device.AddrDevice_0, AGRO_Device.ChDevice_0, DISABLE);

  HAL_Delay(500);
  LED_ON;
  HAL_Delay(500);
  LED_OFF;


  E220_SetMode(NORMAL);




//  HAL_UART_Receive_IT(&huart1, (uint8_t*)&recv, 5);
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);



#if SSD1306_DISPLAY

  SSD1306_Init();
  SSD1306_GotoXY(0, 0);
  sprintf(buf, "AGRO ROBOT");
  SSD1306_Puts(buf, &Font_7x10, SSD1306_COLOR_WHITE);
  SSD1306_UpdateScreen();
  HAL_Delay(1000);
//  for(int i = 0; i <= 2; i++)
//  {
//	  HAL_Delay(200);
//	  SSD1306_GotoXY(28, 0);
//	  sprintf(buf, ".  ");
//	  SSD1306_Puts(buf, &Font_7x10, SSD1306_COLOR_WHITE);
//	  SSD1306_UpdateScreen();
//	  HAL_Delay(200);
//	  SSD1306_GotoXY(28, 0);
//	  sprintf(buf, "..");
//	  SSD1306_Puts(buf, &Font_7x10, SSD1306_COLOR_WHITE);
//	  SSD1306_UpdateScreen();
//	  HAL_Delay(200);
//	  SSD1306_GotoXY(28, 0);
//	  sprintf(buf, "...");
//	  SSD1306_Puts(buf, &Font_7x10, SSD1306_COLOR_WHITE);
//	  SSD1306_UpdateScreen();
//	  HAL_Delay(200);
//  }
  SSD1306_Fill(SSD1306_COLOR_BLACK);
  SSD1306_UpdateScreen();
#endif

  uint16_t numPack = 100;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET)
//	  {
//		  //***********************//
//		  //***************//
//		  count_1 = 0;
//		  lose_count_1 = 0;
//		  //***************//
//		  count_2 = 0;
//		  lose_count_2 = 0;
//		  //***************//
//		  all_cnt = 1;
//		  i = 0;
//		  //***********************//
//		  for(i = 1 ; i <= numPack; i++)
//		  {
//			  data_RSSI_0[2] = numPackNowSend[0];
//			  data_RSSI_1[2] = numPackNowSend[1];
//			  SendOnChannel(AGRO_Device.AddrDevice_1, AGRO_Device.ChDevice_1, data_RSSI_0, 3);
//			  HAL_Delay(600);
//			  SendOnChannel(AGRO_Device.AddrDevice_2, AGRO_Device.ChDevice_2, data_RSSI_1, 3);
//			  HAL_Delay(600);
//
//			  lose_count_1 = i - count_1;
//			  lose_count_2 = i - count_2;
//			  all_cnt++;
//			  numPackNowSend[0]++;
//			  numPackNowSend[1]++;
//		  }
//		  loseCntPer_1 = lose_count_1;
//		  while(((numPack - count_1) > 0) || ((numPack - count_2) > 0))
//		  {
//			  if((numPack - count_1) > 0)
//			  {
//				  data_RSSI_0[2] = numPackNowSend[0];
//				  SendOnChannel(AGRO_Device.AddrDevice_1, AGRO_Device.ChDevice_1, data_RSSI_0, 3);
//				  HAL_Delay(600);
//				  lose_count_1 = i - count_1;
//				  numPackNowSend[0]++;
//			  }
//			  else HAL_Delay(600);
//			  if((numPack - count_2) > 0)
//			  {
//				  data_RSSI_1[2] = numPackNowSend[1];
//				  SendOnChannel(AGRO_Device.AddrDevice_2, AGRO_Device.ChDevice_2, data_RSSI_1, 3);
//				  HAL_Delay(600);
//				  lose_count_2 = i - count_2;
//				  numPackNowSend[1]++;
//			  }
//			  else HAL_Delay(600);
//			  all_cnt++;
//			  i++;
//			  sprintf(buf, "%d\t%d\t%d\t%d\t%d\r\n", i, RSSI_recv[0], NumPacket_recv[0], RSSI_recv[1], NumPacket_recv[1]);
//			  HAL_UART_Transmit_IT(&huart3, (uint8_t*)buf, strlen(buf));
//			  RSSI_recv[0] = 0;
//			  RSSI_recv[1] = 0;
//			  RSSI_recv[2] = 0;
//		  }
//		  all_cnt = all_cnt - 1;
//		  HAL_Delay(600);
//		  sprintf(buf, "\n%d,%d,%d,%d  ", all_cnt, count_1, loseCntPer_1, lose_count_1);
//		  HAL_UART_Transmit_IT(&huart3, (uint8_t*)buf, strlen(buf));
//		  HAL_Delay(600);
//		  sprintf(buf, "\n%d,%d,%d,%d  ", all_cnt, count_2, loseCntPer_2, lose_count_2);
//		  HAL_UART_Transmit_IT(&huart3, (uint8_t*)buf, strlen(buf));
//
//		  HAL_UART_Transmit_IT(&huart3, (uint8_t*)"_____", strlen("_____"));
//	  }




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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

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
  hi2c1.Init.ClockSpeed = 100000;
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
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|E220_M1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|E220_M0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : E220_M0_Pin */
  GPIO_InitStruct.Pin = E220_M0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(E220_M0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : E220_M1_Pin */
  GPIO_InitStruct.Pin = E220_M1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(E220_M1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1) // Radio kanalidan ma'lumot kelsa
	{
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        // Start baytini tekshirish
        if (rx_byte == E220_START_BYTE)
        {
            flag_start_recv = true; // Qabul qilishni boshlash uchun flagni yoqamiz
            counterBuffer = 0; // Bufer hisoblagichini 0 ga sozlaymiz
        }
        else if (flag_start_recv) // Agar qabul qilish boshlangan bo'lsa
        {
            // Stop baytini tekshirish
            if (rx_byte == E220_STOP_BYTE)
            {
                flag_start_recv = false; // Qabul qilishni to'xtatish uchun flagni o'chiramiz
                LED_OFF;

                // Ekranni tozalash
                SSD1306_Fill(SSD1306_COLOR_BLACK);

                // Buferni to'xtatish baytidan oldingi holatga keltirish
                recvBuffer[counterBuffer] = '\0';

                // Ekranga matnni chiqarish
				SSD1306_GotoXY(0, 0);
				SSD1306_Puts("Received:", &Font_7x10, SSD1306_COLOR_WHITE);
				SSD1306_GotoXY(0, 15);
				SSD1306_Puts((char*)recvBuffer, &Font_7x10, SSD1306_COLOR_WHITE);
				SSD1306_UpdateScreen();

				// Qayta ishlashdan so'ng buferni tozalash
				memset(recvBuffer, 0, counterBuffer);
				counterBuffer = 0; // Hisoblagichni nolga qaytarish
                // Misol uchun, uni UART3 orqali kompyuterga yuborish
                HAL_UART_Transmit(&huart3, recvBuffer, counterBuffer, 100);

                // Qayta ishlashdan so'ng buferni tozalash
                memset(recvBuffer, 0, counterBuffer);
            }
            else // Agar start va stop bayti bo'lmasa, ma'lumotni buferga yozish
            {
            	if (counterBuffer < sizeof(recvBuffer) - 1)
				{
					recvBuffer[counterBuffer++] = rx_byte;
				}
            }
        }

	}
	else if(huart == &huart3) // Приняли с порта (bu qism o'zgarishsiz qolishi mumkin)
	{
        // Bu yerda o'zingizning funksionalligingiz bo'lishi mumkin
	}

    // Keyingi baytni qabul qilishni kutish
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	UART_ERROR = HAL_UART_GetError(&huart1);
	if(UART_ERROR == HAL_UART_ERROR_ORE)	// Ошибка переполнения
	{
		HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_1)
	{

	}
	else if(GPIO_Pin == GPIO_PIN_9)
	{
		AUX_Flag = true;
	}
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
