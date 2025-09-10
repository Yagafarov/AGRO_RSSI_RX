/*
 * RadioCommunication.c
 *
 *  Created on: 29 нояб. 2023 г.
 *      Author: Wyder
 */

#include "RadioCommunication.h"

AGRO_HandleTypeDef SettingsAllDevice;
NumberDevice MyDevice = 0;
//-------------------------------------------------------------------//
uint8_t ID_device[4] = {0xA0, 0xA1, 0xA2, 0xA3};
//-------------------------------------------------------------------//
uint8_t DeviceHEX(NumberDevice dev)
{
	uint8_t res = 0xA0;
	switch(dev)
	{
	case Device_0:
		res = 0xA0;
		break;
	case Device_1:
		res = 0xA1;
		break;
	case Device_2:
		res = 0xA2;
		break;
	case Device_3:
		res = 0xA3;
		break;
	}

	return res;
}
//-------------------------------------------------------------------//
NumberDevice DeviceNUM(uint8_t dev)
{
	uint8_t res = 0xA0;
	switch(dev)
	{
	case Device_0:
		res = 0xA0;
		break;
	case Device_1:
		res = 0xA1;
		break;
	case Device_2:
		res = 0xA2;
		break;
	case Device_3:
		res = 0xA3;
		break;
	}

	return res;
}
//-------------------------------------------------------------------//
NumberDevice AGRO_Init(AGRO_HandleTypeDef Device, NumberDevice divice)
{
	SettingsAllDevice = Device;
	MyDevice = divice;

	return MyDevice;
}
//-------------------------------------------------------------------//
_Bool AGRO_SendPacket(NumberDevice device, uint8_t *load_data, uint8_t bytes)
{
	if(device == MyDevice) return false;
	else
	{
		if(device == Device_0)
		{
			SendOnChannel(SettingsAllDevice.AddrDevice_0, SettingsAllDevice.ChDevice_0, load_data, bytes);

			return true;
		}
		else if(device == Device_1)
		{
			SendOnChannel(SettingsAllDevice.AddrDevice_1, SettingsAllDevice.ChDevice_1, load_data, bytes);

			return true;
		}
		else if(device == Device_2)
		{
			SendOnChannel(SettingsAllDevice.AddrDevice_2, SettingsAllDevice.ChDevice_2, load_data, bytes);

			return true;
		}
		else if(device == Device_3)
		{
			SendOnChannel(SettingsAllDevice.AddrDevice_3, SettingsAllDevice.ChDevice_3, load_data, bytes);

			return true;
		}
	}
}
//-------------------------------------------------------------------//
_Bool AGRO_Get_RSSI(NumberDevice device)
{
	uint8_t data[2];

	if(device == MyDevice) return false;
	else
	{
		data[0] = DeviceHEX(MyDevice);
		data[1] = AGRO_COM_RSSI;
		AGRO_SendPacket(device, data, 2);

		return true;
	}
}
//-------------------------------------------------------------------//
_Bool AGRO_Get_ID(NumberDevice device)
{
	uint8_t data[2];

	if(device == MyDevice) return false;
	else
	{
		data[0] = DeviceHEX(MyDevice);
		data[1] = AGRO_COM_ID;
		AGRO_SendPacket(device, data, 2);

		return true;
	}
}
//-------------------------------------------------------------------//
_Bool AGRO_Get_Status(NumberDevice device)
{
	uint8_t data[2];

	if(device == MyDevice) return false;
	else
	{
		data[0] = DeviceHEX(MyDevice);
		data[1] = AGRO_COM_STATUS;
		AGRO_SendPacket(device, data, 2);

		return true;
	}
}
//-------------------------------------------------------------------//
_Bool AGRO_Run_Test(NumberDevice device)
{
	uint8_t data[2];

	if(device == MyDevice) return false;
	else
	{
		data[0] = DeviceHEX(MyDevice);
		data[1] = AGRO_COM_TEST;
		AGRO_SendPacket(device, data, 2);

		return true;
	}
}
_Bool AGRO_Perform_Command(uint8_t *recv_byte)
{
	uint8_t devFrom = 0x00;
	uint8_t com = 0x00;
	uint16_t addr_send = 0x00;
	uint8_t ch_send = 0x00;
	uint8_t data[2] = {0x00, };

	devFrom = recv_byte[0];
	com = recv_byte[1];
	data[0] = DeviceHEX(MyDevice);

	switch(devFrom)
	{
	case 0xA0:
		addr_send = SettingsAllDevice.AddrDevice_0;
		ch_send = SettingsAllDevice.ChDevice_0;
		break;
	case 0xA1:
		addr_send = SettingsAllDevice.AddrDevice_1;
		ch_send = SettingsAllDevice.ChDevice_1;
		break;
	case 0xA2:
		addr_send = SettingsAllDevice.AddrDevice_2;
		ch_send = SettingsAllDevice.ChDevice_2;
		break;
	case 0xA3:
		addr_send = SettingsAllDevice.AddrDevice_3;
		ch_send = SettingsAllDevice.ChDevice_3;
		break;
	}

	if(com == AGRO_COM_STATUS)
	{
		data[1] = 0x00;
		SendOnChannel(addr_send, ch_send, data, 2);
	}
	else if(com == AGRO_COM_ID)
	{
		data[1] = 0x01;
		SendOnChannel(addr_send, ch_send, data, 2);
	}
	else if(com == AGRO_COM_RSSI)
	{
		data[1] = 0x02;
		SendOnChannel(addr_send, ch_send, data, 2);
	}
	else if(com == AGRO_COM_TEST)
	{
		data[1] = 0x03;
		SendOnChannel(addr_send, ch_send, data, 2);
	}
	return true;
}

//-------------------------------------------------------------------//
//_Bool AGRO_Perform_Settings()
//-------------------------------------------------------------------//

//-------------------------------------------------------------------//



//-------------------------------------------------------------------//
//-------------------------------------------------------------------//
//-------------------------------------------------------------------//
//-------------------------------------------------------------------//
//-------------------------------------------------------------------//
//-------------------------------------------------------------------//
//-------------------------------------------------------------------//


//-------------------------------------------------------------------//
/*
//-----//
ID устройств
0xA0 - device 0
0xA1 - device 1
...
//-----//
При передачи пакета для RSSI
0xFF - Затычка, пакет, указывает на то что данный пакет нужен лишь для получения RSSI


//-----//

//-----//


//-----//


//-----//


*/
//-------------------------------------------------------------------//














