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
#include "E220.h"
#include "ssd1306.h"
#include "fonts.h"
#include "RadioCommunication.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define SSD1306_DISPLAY true
#define START_BYTE	0xAA
#define STOP_BYTE	0xBB
#define MAX_PACKET_SIZE 256

// UART packet reception state variables
_Bool receiving = false;
uint16_t rxIndex = 0;
uint8_t packetLength = 0;
uint8_t rx_byte;
uint8_t recvBuffer[MAX_PACKET_SIZE];

// Global device and state variables
_Bool AUX_Flag = false;
AGRO_HandleTypeDef AGRO_Device;
char buf[30];
uint8_t checkStatus = 0;

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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);

// New function prototypes for refactored code
static void Hardware_Init(void);
static void Device_Init(void);
static void Display_Init(void);
static void ProcessReceivedPacket(void);
static void UpdateOledDisplay(uint8_t *buffer, uint8_t payload_start);

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
  HAL_Init();
  SystemClock_Config();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Initialize all configured peripherals */
  Hardware_Init();
  Device_Init();
  Display_Init();

  /* USER CODE BEGIN 2 */
  // Start the first UART receive interrupt
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Main application logic goes here
      // For example, handle external events or send data periodically
      if(AUX_Flag) {
          // Logic for AUX pin interrupt
          AUX_Flag = false;
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
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
  * @brief All hardware peripherals initialization
  * @retval None
  */
static void Hardware_Init(void) {
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_USART3_UART_Init();
    MX_I2C1_Init();
}

/**
  * @brief E220 and AGRO device initialization and configuration
  * @retval None
  */
static void Device_Init(void) {
    LED_OFF;
    E220_SetMode(DeepSleep);
    E220_WaitReady();
    LED_ON;
    HAL_Delay(500);
    LED_OFF;

    // The AGRO_Init function expects the structure by value, not a pointer
    AGRO_Init(AGRO_Device, Device_0);
    AGRO_Device.AddrDevice_1 = 0x0001;
    AGRO_Device.ChDevice_1 = 0x02;
    AGRO_Device.AddrDevice_0 = 0x0003;
    AGRO_Device.ChDevice_0 = 0x04;
    E220_SetDefaultSettings(AGRO_Device.AddrDevice_0, AGRO_Device.ChDevice_0, DISABLE);

    HAL_Delay(500);
    LED_ON;
    HAL_Delay(500);
    LED_OFF;

    E220_SetMode(NORMAL);
}

/**
  * @brief Initializes and displays a welcome message on the SSD1306
  * @retval None
  */
static void Display_Init(void) {
#if SSD1306_DISPLAY
    SSD1306_Init();
    SSD1306_GotoXY(0, 0);
    sprintf(buf, "AGRO ROBOT");
    SSD1306_Puts(buf, &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_UpdateScreen();
    HAL_Delay(1000);
    SSD1306_Fill(SSD1306_COLOR_BLACK);
    SSD1306_UpdateScreen();
#endif
}

/**
  * @brief Updates the OLED display with received packet data.
  * @param buffer: Pointer to the received buffer.
  * @param payload_start: Start index of the payload in the buffer.
  * @retval None
  */
static void UpdateOledDisplay(uint8_t *buffer, uint8_t payload_start) {
    uint8_t address = buffer[payload_start];
    uint8_t deviceType = buffer[payload_start + 1];
    uint8_t mode = buffer[payload_start + 2];
    uint16_t speed = (buffer[payload_start + 3] << 8) | buffer[payload_start + 4];

    checkStatus = 0x00;

    // Check address
	if (address != 0x01 && address != 0x02) {
		checkStatus = 0x01;
	}
	// Check mode (only if address is valid)
	if (checkStatus == 0x00 && mode != 0x03 && mode != 0x04) {
		checkStatus = 0x02;
	}

#if SSD1306_DISPLAY
    char disp_buf[20];
    SSD1306_Fill(SSD1306_COLOR_BLACK);

    if (checkStatus == 0x00) {
        // No error - display all information
        SSD1306_GotoXY(0,0);
        sprintf(disp_buf, "Adr: %02X", address);
        SSD1306_Puts(disp_buf, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0,15);
        sprintf(disp_buf, "Dev: %02X", deviceType);
        SSD1306_Puts(disp_buf, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0,30);
        sprintf(disp_buf, "Mode: %02X", mode);
        SSD1306_Puts(disp_buf, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0,45);
        sprintf(disp_buf, "Speed: %u", speed);
        SSD1306_Puts(disp_buf, &Font_7x10, SSD1306_COLOR_WHITE);

        // Display status (no error)
        SSD1306_GotoXY(0,55);
        sprintf(disp_buf, "Status: OK");
        SSD1306_Puts(disp_buf, &Font_7x10, SSD1306_COLOR_WHITE);
    } else {
        // Error - display only status message
        SSD1306_GotoXY(0,0);

        if (checkStatus == 0x01) {
            sprintf(disp_buf, "Address Error");
        } else if (checkStatus == 0x02) {
            sprintf(disp_buf, "Mode Error");
        }

        SSD1306_Puts(disp_buf, &Font_7x10, SSD1306_COLOR_WHITE);

        // Display error code
        SSD1306_GotoXY(0,15);
        sprintf(disp_buf, "Code: %02X", checkStatus);
        SSD1306_Puts(disp_buf, &Font_7x10, SSD1306_COLOR_WHITE);
    }

    SSD1306_UpdateScreen();
#endif
}

/**
  * @brief Processes a fully received packet based on its length.
  * @retval None
  */
static void ProcessReceivedPacket(void) {
    uint8_t payload_start = 3; // start bytes (2) + length byte (1)

    // Check for valid stop bytes
    if (recvBuffer[rxIndex-2] != STOP_BYTE || recvBuffer[rxIndex-1] != STOP_BYTE) {
        // Invalid packet, return without processing
        return;
    }

    // Process the packet based on its length
    switch(packetLength) {
        case 0x05: {
            UpdateOledDisplay(recvBuffer, payload_start);
        } break;
        case 0x06: {
            UpdateOledDisplay(recvBuffer, payload_start);
        } break;
        default: {
            // Unknown packet length, handle as an error or ignore
            // For now, we will just return
        }
    }
}

/**
  * @brief  UART Rx Transfer complete callback.
  * @param  huart: Pointer to a UART_HandleTypeDef structure.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        if (!receiving) {
            // Check for the start sequence (0xAA 0xAA)
            if (rxIndex < 2 && rx_byte == START_BYTE) {
                recvBuffer[rxIndex++] = rx_byte;
                if (rxIndex == 2) {
                    receiving = true;
                }
            } else {
                rxIndex = 0;
            }
        } else {
            // Receive packet payload and stop bytes
            recvBuffer[rxIndex++] = rx_byte;
            if (rxIndex == 3) {
                packetLength = recvBuffer[2];
                // Validate packet length
                if (packetLength > (MAX_PACKET_SIZE - 6)) {
                    rxIndex = 0;
                    receiving = false;
                }
            } else if (packetLength > 0 && rxIndex >= (3 + packetLength + 2)) {
                // A full packet has been received
                ProcessReceivedPacket();
                rxIndex = 0;
                receiving = false;
                packetLength = 0;
            } else if (rxIndex >= MAX_PACKET_SIZE) {
                // Buffer overflow, reset
                rxIndex = 0;
                receiving = false;
                packetLength = 0;
            }
        }
        // Start receiving the next byte
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

/**
  * @brief  UART error callback.
  * @param  huart: Pointer to a UART_HandleTypeDef structure.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        uint32_t UART_ERROR = HAL_UART_GetError(&huart1);
        if (UART_ERROR == HAL_UART_ERROR_ORE) // Overrun error
        {
            HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
        }
    }
}

/**
  * @brief  GPIO EXTI callback.
  * @param  GPIO_Pin: Specifies the pin where the interrupt occurred.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_9)
    {
        AUX_Flag = true;
    }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
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
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
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
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{
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
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

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
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
