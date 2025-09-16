/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (variant 2 - full code)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <string.h>
#include <stdio.h>
#include "E220.h"
#include "ssd1306.h"
#include "fonts.h"
#include "RadioCommunication.h"

/* Private defines -----------------------------------------------------------*/
#define SSD1306_DISPLAY true
#define START_BYTE      0x01
#define STOP_BYTE       0xBB
#define MAX_PACKET_SIZE 256

/* Packet reception state machine states */
typedef enum {
    PACKET_STATE_IDLE,
    PACKET_STATE_WAIT_START2,
    PACKET_STATE_WAIT_LENGTH,
    PACKET_STATE_WAIT_PAYLOAD,
    PACKET_STATE_WAIT_STOP1,
    PACKET_STATE_WAIT_STOP2
} PacketState_t;

/* Private variables ---------------------------------------------------------*/
PacketState_t packetState = PACKET_STATE_IDLE;
uint16_t     rxIndex     = 0;
uint8_t      packetLength= 0;
uint8_t      rx_byte;
uint8_t      recvBuffer[MAX_PACKET_SIZE];
_Bool        PacketReceived_Flag = false;
_Bool        AUX_Flag            = false;
AGRO_HandleTypeDef AGRO_Device;
char         buf[30];

/* Peripheral handles */
I2C_HandleTypeDef  hi2c1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void Hardware_Init(void);
static void Device_Init(void);
static void Display_Init(void);
static void UpdateOledDisplay(char *title, uint8_t data, uint8_t x, uint8_t y, uint8_t status);
static void SendResponse(uint8_t status_code, uint8_t received_address);

/**
  * @brief  Main entry point
  */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    Hardware_Init();
    Device_Init();
    Display_Init();

    /* Start UART1 receive interrupt */
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

    while (1)
    {
        if (PacketReceived_Flag)
        {
            uint8_t payload_start = 3;
            uint8_t status_code   = 0x00;
            uint8_t received_addr = 0xFF;

            if (packetLength >= 3)
            {
                received_addr = recvBuffer[payload_start];
                if (packetLength == 0x05)
                {
                    uint8_t deviceType = recvBuffer[payload_start+1];
                    uint8_t mode       = recvBuffer[payload_start+2];
                    if (received_addr != 0x01 && received_addr != 0x02)
                        status_code = 0x01;  // Address error
                    else if (mode != 0x03 && mode != 0x04)
                        status_code = 0x02;  // Mode error
                }
                else if (packetLength != 0x04 && packetLength != 0x06)
                {
                    status_code = 0x03;     // Length error
                }
            }
            else
            {
                status_code = 0x04;         // Too short
            }

            UpdateOledDisplay("Addr:", received_addr, 0, 0, status_code);
            SendResponse(status_code, received_addr);
            PacketReceived_Flag = false;
        }

        if (AUX_Flag)
        {
            AUX_Flag = false;
            /* Handle AUX pin event if needed */
        }
    }
}

/**
  * @brief  System Clock Configuration
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState            = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue      = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType           = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                           |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource        = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider       = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider      = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider      = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

/**
  * @brief  Initialize GPIO, UART, I2C
  */
static void Hardware_Init(void)
{
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_USART3_UART_Init();
    MX_I2C1_Init();
}

/**
  * @brief  Device-specific init (E220, AGRO)
  */
static void Device_Init(void)
{
    LED_OFF;
    E220_SetMode(DeepSleep);
    E220_WaitReady();
    LED_ON; HAL_Delay(500); LED_OFF;

    AGRO_Init(AGRO_Device, Device_0);
    AGRO_Device.AddrDevice_1 = 0x0001;
    AGRO_Device.ChDevice_1   = 0x02;
    AGRO_Device.AddrDevice_0 = 0x0003;
    AGRO_Device.ChDevice_0   = 0x04;
    E220_SetDefaultSettings(AGRO_Device.AddrDevice_0, AGRO_Device.ChDevice_0, DISABLE);

    HAL_Delay(500); LED_ON; HAL_Delay(500); LED_OFF;
    E220_SetMode(NORMAL);
}

/**
  * @brief  Initialize SSD1306 OLED
  */
static void Display_Init(void)
{
#if SSD1306_DISPLAY
    SSD1306_Init();
    SSD1306_GotoXY(0,0);
    sprintf(buf, "AGRO ROBOT");
    SSD1306_Puts(buf, &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_UpdateScreen();
    HAL_Delay(1000);
    SSD1306_Fill(SSD1306_COLOR_BLACK);
    SSD1306_UpdateScreen();
#endif
}

/**
  * @brief  Update OLED with title and data or error status
  */
static void UpdateOledDisplay(char *title, uint8_t data, uint8_t x, uint8_t y, uint8_t status)
{
#if SSD1306_DISPLAY
    char disp[30];
    SSD1306_Fill(SSD1306_COLOR_BLACK);

    if (status == 0x00)
    {
        SSD1306_GotoXY(x,y);
        snprintf(disp, sizeof(disp), "%s %d", title, data);
        SSD1306_Puts(disp, &Font_7x10, SSD1306_COLOR_WHITE);
    }
    else
    {
        SSD1306_GotoXY(0,0);
        if (status==0x01) snprintf(disp, sizeof(disp),"Addr Error");
        else if(status==0x02) snprintf(disp,sizeof(disp),"Mode Error");
        else if(status==0x03) snprintf(disp,sizeof(disp),"Len Error");
        else snprintf(disp,sizeof(disp),"Unknown Err");
        SSD1306_Puts(disp,&Font_7x10,SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0,15);
        snprintf(disp,sizeof(disp),"Code:%02X",status);
        SSD1306_Puts(disp,&Font_7x10,SSD1306_COLOR_WHITE);
    }
    SSD1306_UpdateScreen();
#endif
}

/**
  * @brief  Send response packet over E220
  */
static void SendResponse(uint8_t status_code, uint8_t received_address)
{
    uint8_t packet[8];
    packet[0]=START_BYTE; packet[1]=START_BYTE;
    packet[2]=0x03;     // payload length
    packet[3]=0x01;     // response type
    packet[4]=status_code;
    packet[5]=received_address;
    packet[6]=STOP_BYTE; packet[7]=STOP_BYTE;

    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
    HAL_Delay(200);
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);

    SendOnChannel(AGRO_Device.AddrDevice_1, AGRO_Device.ChDevice_1, packet, sizeof(packet));
}

/**
  * @brief  UART Rx Complete callback
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart==&huart1)
    {
        switch(packetState)
        {
        case PACKET_STATE_IDLE:
            if(rx_byte==START_BYTE)
            {
                rxIndex=0; recvBuffer[rxIndex++]=rx_byte;
                packetState=PACKET_STATE_WAIT_START2;
            }
            break;

        case PACKET_STATE_WAIT_START2:
            if(rx_byte==START_BYTE)
            {
                recvBuffer[rxIndex++]=rx_byte;
                packetState=PACKET_STATE_WAIT_LENGTH;
            }
            else packetState=PACKET_STATE_IDLE;
            break;

        case PACKET_STATE_WAIT_LENGTH:
            packetLength=rx_byte;
            if(packetLength>(MAX_PACKET_SIZE-5)) packetState=PACKET_STATE_IDLE;
            else { recvBuffer[rxIndex++]=rx_byte; packetState=PACKET_STATE_WAIT_PAYLOAD; }
            break;

        case PACKET_STATE_WAIT_PAYLOAD:
            recvBuffer[rxIndex++]=rx_byte;
            if(rxIndex-3==packetLength) packetState=PACKET_STATE_WAIT_STOP1;
            break;

        case PACKET_STATE_WAIT_STOP1:
            if(rx_byte==STOP_BYTE)
            {
                recvBuffer[rxIndex++]=rx_byte;
                packetState=PACKET_STATE_WAIT_STOP2;
            }
            else packetState=PACKET_STATE_IDLE;
            break;

        case PACKET_STATE_WAIT_STOP2:
            if(rx_byte==STOP_BYTE)
            {
                recvBuffer[rxIndex++]=rx_byte;
                PacketReceived_Flag=true;
            }
            packetState=PACKET_STATE_IDLE;
            break;

        default:
            packetState=PACKET_STATE_IDLE;
            break;
        }

        HAL_UART_Receive_IT(&huart1,&rx_byte,1);
    }
}

/**
  * @brief  UART Error callback
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart==&huart1)
    {
        uint32_t err=HAL_UART_GetError(&huart1);
        if(err==HAL_UART_ERROR_ORE)
            HAL_UART_Receive_IT(&huart1,&rx_byte,1);
    }
}

/**
  * @brief  GPIO EXTI callback
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin==GPIO_PIN_9)
        AUX_Flag=true;
}

/* Initialization functions generated by CubeMX ----------------------------*/

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct={0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* LCD LED pin */
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);

    /* E220 M0/M1 control pins */
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4|E220_M1_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0|E220_M0_Pin,GPIO_PIN_RESET);

    /* PC13 (LED) */
    GPIO_InitStruct.Pin   = GPIO_PIN_13;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC,&GPIO_InitStruct);

    /* PA4 (M1) */
    GPIO_InitStruct.Pin   = GPIO_PIN_4;
    HAL_GPIO_Init(GPIOA,&GPIO_InitStruct);

    /* PB0 (M0) */
    GPIO_InitStruct.Pin   = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOB,&GPIO_InitStruct);

    /* PB1 input */
    GPIO_InitStruct.Pin   = GPIO_PIN_1;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB,&GPIO_InitStruct);

    /* E220_M0 & M1 pins */
    GPIO_InitStruct.Pin   = E220_M0_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
    HAL_GPIO_Init(E220_M0_GPIO_Port,&GPIO_InitStruct);

    GPIO_InitStruct.Pin   = E220_M1_Pin;
    HAL_GPIO_Init(E220_M1_GPIO_Port,&GPIO_InitStruct);

    /* PB9 EXTI */
    GPIO_InitStruct.Pin   = GPIO_PIN_9;
    GPIO_InitStruct.Mode  = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB,&GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn,0,0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

static void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 9600;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if(HAL_UART_Init(&huart1)!=HAL_OK) Error_Handler();
}

static void MX_USART3_UART_Init(void)
{
    huart3.Instance          = USART3;
    huart3.Init.BaudRate     = 9600;
    huart3.Init.WordLength   = UART_WORDLENGTH_8B;
    huart3.Init.StopBits     = UART_STOPBITS_1;
    huart3.Init.Parity       = UART_PARITY_NONE;
    huart3.Init.Mode         = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if(HAL_UART_Init(&huart3)!=HAL_OK) Error_Handler();
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 100000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if(HAL_I2C_Init(&hi2c1)!=HAL_OK) Error_Handler();
}

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
    __disable_irq();
    while(1) {}
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
