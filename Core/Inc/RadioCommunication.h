/*
 * RadioCommunication.h
 *
 *  Created on: 29 нояб. 2023 г.
 *      Author: Wyder
 */

#ifndef INC_RADIOCOMMUNICATION_H_
#define INC_RADIOCOMMUNICATION_H_


#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "main.h"

#include "E220.h"
#include "foundPoint.h"
//-------------------------------------------------------------------//
/*Команды радиоканала*/
#define	AGRO_COM_STATUS		0x00			// Статус
#define	AGRO_COM_ID			0x01			// Идентификатор
#define	AGRO_COM_RSSI		0x02			// Мощность сигнала
#define	AGRO_COM_TEST		0x03			// Тест
//#define	AGRO_COM_STATUS		0x00
//#define	AGRO_COM_STATUS		0x00
//#define	AGRO_COM_STATUS		0x00
//#define	AGRO_COM_STATUS		0x00
//#define	AGRO_COM_STATUS		0x00
//#define	AGRO_COM_STATUS		0x00
//#define	AGRO_COM_STATUS		0x00
//#define	AGRO_COM_STATUS		0x00
//#define	AGRO_COM_STATUS		0x00
//#define	AGRO_COM_STATUS		0x00
//#define	AGRO_COM_STATUS		0x00
//#define	AGRO_COM_STATUS		0x00
//#define	AGRO_COM_STATUS		0x00
//#define	AGRO_COM_STATUS		0x00
//#define	AGRO_COM_STATUS		0x00
//-------------------------------------------------------------------//


/*Установка адресов и каналов используемых устройств*/

typedef enum NumberDevice
{
	Device_0 = 0,
	Device_1,
	Device_2,
	Device_3,
} NumberDevice;

typedef struct AGRO_HandleTypeDef
{
	uint16_t AddrDevice_0;
	uint16_t AddrDevice_1;
	uint16_t AddrDevice_2;
	uint16_t AddrDevice_3;

	uint8_t ChDevice_0;
	uint8_t ChDevice_1;
	uint8_t ChDevice_2;
	uint8_t ChDevice_3;

} AGRO_HandleTypeDef;


//-------------------------------------------------------------------//
/*Служебные функции (внутренние)*/
uint8_t DeviceHEX(NumberDevice dev);												// Получение HEX-адрес данного устройства
NumberDevice DeviceNUM(uint8_t dev);												// Получение NUM-адрес данного устройства (номер)
_Bool AGRO_SendPacket(NumberDevice device, uint8_t *load_data, uint8_t bytes);		// Отправка пакета по радио-каналу
//-------------------------------------------------------------------//
NumberDevice AGRO_Init(AGRO_HandleTypeDef Device, NumberDevice divice);				// Инициализация
/*Команды*/
_Bool AGRO_Get_Status(NumberDevice device);											// Получения статуса работы маяка
_Bool AGRO_Get_ID(NumberDevice device);												// Получения идентификатора устройства с адресом xx
_Bool AGRO_Get_RSSI(NumberDevice device);											// Получения мощности сигнала с маяка
_Bool AGRO_Run_Test(NumberDevice device);											// Запуск тестирования связи с маяком
//-------------------------------------------------------------------//


_Bool AGRO_Perform_Command(uint8_t *recv_byte);										// Выполнить команду (касается радиосвязи)
//-------------------------------------------------------------------//



//-------------------------------------------------------------------//
//-------------------------------------------------------------------//
//-------------------------------------------------------------------//


//-------------------------------------------------------------------//
//-------------------------------------------------------------------//


//-------------------------------------------------------------------//
















#endif /* INC_RADIOCOMMUNICATION_H_ */

