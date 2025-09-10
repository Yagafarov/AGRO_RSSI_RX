/*
 * E220.h
 *
 *  Created on: Sep 28, 2023
 *      Author: Wyder
 */

#ifndef INC_E220_H_
#define INC_E220_H_


#include "stm32f1xx_hal.h"
#include <stdbool.h>

/*------------------------------------------*/
#define E220_AUX_PORT	GPIOB
#define E220_AUX_PIN	GPIO_PIN_9

#define E220_M0_PORT	GPIOB
#define E220_M0_PIN		GPIO_PIN_15

#define E220_M1_PORT	GPIOA
#define E220_M1_PIN		GPIO_PIN_8

#define STM_LED_PORT	GPIOC
#define STM_LED_PIN		GPIO_PIN_13
/*------------------------------------------*/
#define E220_MO_SET		HAL_GPIO_WritePin(E220_M0_PORT, E220_M0_PIN, GPIO_PIN_SET)
#define E220_MO_RESET	HAL_GPIO_WritePin(E220_M0_PORT, E220_M0_PIN, GPIO_PIN_RESET)

#define E220_M1_SET		HAL_GPIO_WritePin(E220_M1_PORT, E220_M1_PIN, GPIO_PIN_SET)
#define E220_M1_RESET	HAL_GPIO_WritePin(E220_M1_PORT, E220_M1_PIN, GPIO_PIN_RESET)

#define LED_ON			HAL_GPIO_WritePin(STM_LED_PORT, STM_LED_PIN, GPIO_PIN_RESET)
#define LED_OFF			HAL_GPIO_WritePin(STM_LED_PORT, STM_LED_PIN, GPIO_PIN_SET)
/*------------------------------------------*/
#define TIMEOUT			1000
/*------------------------------------------*/
/*	 REGISTERS	 */
#define	E220_ADDH		0x00
#define	E220_ADDL		0x01
#define	E220_REG0		0x02
#define	E220_REG1		0x03
#define E220_REG2		0x04
#define E220_REG3		0x05
#define E220_CRYPT_H	0x06
#define E220_CRYPT_L	0x07
/*------------------------------------------*/
#define	E220_CMD0		0xC0
#define	E220_CMD1		0xC1
#define	E220_CMD2		0xC2
#define E220_CMD3		0xC3
/*------------------------------------------*/
#define ENABLE			true
#define DISABLE			false
/*------------------------------------------*/
typedef enum OperatingMode
{
	NORMAL = 0,										/*Нормальная работа (обычная для Serial port)			*/
	WOR_Transmission,										/*Режим ТОЛЬКО передачи									*/
	WOR_Receiving,											/*Режим ТОЛЬКО приема									*/
	DeepSleep,												/*Примем и передача не осуществляются (сон)				*/
} OperatingMode;
/*------------------------------------------*/
typedef enum SerialPortRate
{
	Rate_1200 = 0,
	Rate_2400,
	Rate_4800,
	Rate_9600,
	Rate_19200,
	Rate_38400,
	Rate_57600,
	Rate_115200,
} SerialPortRate;
/*------------------------------------------*/
typedef enum SerialPorityBit
{
	_8N1,
	_8O1,
	_8E1,
	_8N1_2,
} SerialPorityBit;
/*------------------------------------------*/
typedef enum AirDataRate
{
	_2_4k = 0,
	_2_4k_2,
	_2_4k_3,
	_4_8k,
	_9_6k,
	_19_2k,
	_38_4k,
	_62_5k,
} AirDataRate;
/*------------------------------------------*/
typedef enum SubPacketSetting
{
	_200bytes = 0,
	_128bytes,
	_64bytes,
	_32bytes,
} SubPacketSetting;
/*------------------------------------------*/
typedef enum TransmittingPower
{
	_22dBm = 0,
	_17dBm,
	_13dBm,
	_10dBm,
} TransmittingPower;
/*------------------------------------------*/
typedef enum TransmissionMethod
{
	TransparentTransmissionMode = 0,
	FixedTransmissionMode,
} TransmissionMethod;
/*------------------------------------------*/
typedef enum WORCycle
{
	_500ms = 0,
	_1000ms,
	_1500ms,
	_2000ms,
	_2500ms,
	_3000ms,
	_3500ms,
	_4000ms,
} WORCycle;
/*------------------------------------------*/
typedef struct E220_HandleTypeDef
{
	uint16_t Address;
	SerialPortRate Rate;
	SerialPorityBit Pority;
	AirDataRate AirDataRate;
	SubPacketSetting SubPacket;
	_Bool RSSIAmbientNoiseEnable;
	TransmittingPower TransPower;
	uint8_t ChannelControl;
	_Bool EnableRSSIByte;
	TransmissionMethod TransMethod;
	_Bool LBT;
	WORCycle WORCycle;
	uint16_t Key;
} E220_HandleTypeDef;

typedef struct E220_GlobalRegister
{
	uint16_t addr;
	uint8_t reg0;
	uint8_t reg1;
	uint8_t reg2;
	uint8_t reg3;
	uint16_t Key;
} E220_GlobalRegister;
/*------------------------------------------*/

/*------------------------------------------*/
/*ДЛЯ ОТЛАДКИ*/
uint16_t E220_ReadAddress_REG(void);
uint8_t E220_ReadREG0_REG(void);
uint8_t E220_ReadREG1_REG(void);
uint8_t E220_ReadREG2_REG(void);
uint8_t E220_ReadREG3_REG(void);
/*------------------------------------------*/
GPIO_PinState E220_WaitReady(void); 						/*Функция ожидания установки флага рабочего состояния	*/
GPIO_PinState E220_CheckSentReadPacket(void); 				/*Функция ожидания флага конца отправки пакета			*/
GPIO_PinState E220_SendPacket(uint8_t data, uint16_t size);	/*Функция отправки пакета								*/
GPIO_PinState E220_ReadPacket(uint8_t *received_data);		/*Функция чтения пакета									*/
OperatingMode E220_SetMode(OperatingMode mode); 			/*Функция установки режима работы  						*/
/*------------------------------------------*/
/*Функции настройки*/
void E220_Set_AddressModule(uint16_t addr);
void E220_Set_Uart_Rate_Parity_Data(SerialPortRate rate, SerialPorityBit pority, AirDataRate air_data);
void E220_Set_SubPacket_RSSI_TransPower(SubPacketSetting sub, _Bool RSSIAmbientNoiseEnable, TransmittingPower tr_pow);
void E220_Set_Channel(uint8_t channel);
void E220_Set_RSSI_TransmissionMethod_LBT_WORCycle(_Bool RSSIByte, TransmissionMethod method, _Bool LBT, WORCycle wor);
void E220_Set_Key(uint16_t key);
/*------------------------------------------*/
void E220_Set_Settings(E220_HandleTypeDef str);
void E220_SetDefaultSettings(uint16_t u1, uint8_t u2, _Bool u3);		/*Установка базовых настроек кроме канала и адреса и RSSI_byte*/
/*------------------------------------------*/
/*Функции чтения установленных настроек*/
_Bool E220_Read_Address(void);
_Bool E220_Read_SerialPortRate(void);
_Bool E220_Read_SerialParityBit(void);
_Bool E220_Read_AirDataRate(void);
_Bool E220_Read_SubPacketSetting(void);
_Bool E220_Read_RSSIAmbientNoiseEnable(void);
_Bool E220_Read_TransmitiingPower(void);
_Bool E220_Read_ChannelControl(void);
_Bool E220_Read_EnableRSSIByte(void);
_Bool E220_Read_TransmissionMethod(void);
_Bool E220_Read_LBTEnable(void);
_Bool E220_Read_WORCycle(void);
//uint16_t E220_Read_NOW_RSSI(_Bool noise_rssi, _Bool rssi_last); Don`t worked!
/*------------------------------------------*/
void SendOnChannel(uint16_t addr, uint8_t channel, uint8_t *data, uint8_t bytes);





#endif /* INC_E220_H_ */

