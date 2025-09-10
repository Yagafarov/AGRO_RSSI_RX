/*
 * E220.c
 *
 *  Created on: Sep 28, 2023
 *      Author: Wyder
 */


#include "E220.h"
#include <string.h>
#include <stdio.h>
/*------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

E220_GlobalRegister E220_GlobReg;
extern _Bool AUX_Flag;

uint8_t data_send[200];

/*------------------------------------------*/
extern uint8_t recv[50];
/*------------------------------------------*/
GPIO_PinState E220_WaitReady(void)
{
	while(!(AUX_Flag == true)) {};
	//while(!(HAL_GPIO_ReadPin(E220_AUX_PORT, E220_AUX_PIN))) {};
	AUX_Flag = false;
	return GPIO_PIN_SET;
}
/*------------------------------------------*/
GPIO_PinState E220_CheckSentReadPacket(void)
{
	while(!(AUX_Flag == true)) {};
	//while(!(HAL_GPIO_ReadPin(E220_AUX_PORT, E220_AUX_PIN))) {};
	AUX_Flag = false;
	return GPIO_PIN_SET;
}
/*------------------------------------------*/
GPIO_PinState E220_SendPacket(uint8_t data, uint16_t size)
{
	HAL_UART_Transmit(&huart1, &data, size, TIMEOUT);
	E220_CheckSentReadPacket();
	return GPIO_PIN_SET;
}
/*------------------------------------------*/
OperatingMode E220_SetMode(OperatingMode mode)
{
	if(mode == NORMAL)
	{
		E220_MO_RESET;
		E220_M1_RESET;
	}
	else if(mode == WOR_Transmission)
	{
		E220_MO_SET;
		E220_M1_RESET;
	}
	else if(mode == WOR_Receiving)
	{
		E220_MO_RESET;
		E220_M1_SET;
	}
	else if(mode == DeepSleep)
	{
		E220_MO_SET;
		E220_M1_SET;
	}
	return mode;
	HAL_Delay(50);
	E220_CheckSentReadPacket();
}
/*------------------------------------------*/
void E220_Set_AddressModule(uint16_t addr)
{
	uint8_t addr_h = 0x00;
	uint8_t addr_l = 0x00;
	uint8_t send_data[5] = {E220_CMD0, E220_ADDH, 2, addr_h, addr_l};
	uint16_t mask = 0xFFFF;

	addr_h = (addr >> 8) & mask;
	addr_l = (addr << 0) & mask;

	send_data[3] = addr_h;
	send_data[4] = addr_l;

	HAL_UART_Transmit(&huart1, send_data, 5, TIMEOUT);
	E220_CheckSentReadPacket();
	E220_GlobReg.addr = addr;
}
/*------------------------------------------*/
void E220_Set_Uart_Rate_Parity_Data(SerialPortRate rate, SerialPorityBit pority, AirDataRate air_data)
{
	uint8_t all_data = 0x00;
	uint8_t send_data[4] = {E220_CMD0, E220_REG0, 1, all_data};
	/*UART Serial Port Rate*/
	if(rate == Rate_1200) all_data |= 0x00;
	else if(rate == Rate_2400) all_data |= 0x20;
	else if(rate == Rate_4800) all_data |= 0x40;
	else if(rate == Rate_9600) all_data |= 0x60;
	else if(rate == Rate_19200) all_data |= 0x80;
	else if(rate == Rate_38400) all_data |= 0xA0;
	else if(rate == Rate_57600) all_data |= 0xC0;
	else if(rate == Rate_115200) all_data |= 0xE0;
	/*UART Serial Port Rate*/
	if(pority == _8N1) all_data |= 0x00;
	else if(pority == _8O1) all_data |= 0x80;
	else if(pority == _8E1) all_data |= 0x10;
	else if(pority == _8N1_2) all_data |= 0x18;
	/*UART Serial Port Rate*/
	if(air_data == _2_4k) all_data |= 0x00;
	else if(air_data == _2_4k_2) all_data |= 0x01;
	else if(air_data == _2_4k_3) all_data |= 0x02;
	else if(air_data == _4_8k) all_data |= 0x03;
	else if(air_data == _9_6k) all_data |= 0x04;
	else if(air_data == _19_2k) all_data |= 0x05;
	else if(air_data == _38_4k) all_data |= 0x06;
	else if(air_data == _62_5k) all_data |= 0x07;

	send_data[3] = all_data;

	HAL_UART_Transmit(&huart1, send_data, 4, TIMEOUT);
	E220_CheckSentReadPacket();

	E220_GlobReg.reg0 = all_data;
}
/*------------------------------------------*/
void E220_Set_SubPacket_RSSI_TransPower(SubPacketSetting sub, _Bool RSSIAmbientNoiseEnable, TransmittingPower tr_pow)
{
	uint8_t all_data = 0x00;
	uint8_t send_data[4] = {E220_CMD0, E220_REG1, 1, all_data};
	/*Sub-Packet Setting*/
	if(sub == _200bytes) all_data |= 0x00;
	else if(sub == _128bytes) all_data |= 0x40;
	else if(sub == _64bytes) all_data |= 0x80;
	else if(sub == _32bytes) all_data |= 0xC0;
	/*RSSI Ambient noise enable*/
	if(RSSIAmbientNoiseEnable == true) all_data |= 0x20;
	else all_data |= 0x00;
	/*Transimitting Power*/
	if(tr_pow == _22dBm) all_data |= 0x00;
	else if(tr_pow == _17dBm) all_data |= 0x01;
	else if(tr_pow == _13dBm) all_data |= 0x02;
	else if(tr_pow == _10dBm) all_data |= 0x03;

	send_data[3] = all_data;

	HAL_UART_Transmit(&huart1, send_data, 4, TIMEOUT);
	E220_CheckSentReadPacket();
	E220_GlobReg.reg1 = all_data;
}
/*------------------------------------------*/
void E220_Set_Channel(uint8_t channel)
{
	uint8_t all_data = 0x00;
	uint8_t send_data[4] = {E220_CMD0, E220_REG2, 1, all_data};
	all_data |= channel;

	send_data[3] = all_data;
	HAL_UART_Transmit(&huart1, send_data, 4, TIMEOUT);
	E220_CheckSentReadPacket();
	E220_GlobReg.reg2 = all_data;
}
/*------------------------------------------*/
void E220_Set_RSSI_TransmissionMethod_LBT_WORCycle(_Bool RSSIByte, TransmissionMethod method, _Bool LBT, WORCycle wor)
{
	uint8_t all_data = 0x00;
	uint8_t send_data[4] = {E220_CMD0, E220_REG3, 1, all_data};
	/*Enable RSSI Byte*/
	if(RSSIByte == DISABLE) all_data |= 0x00;
	else all_data |= 0x80;
	/*Transmission Method*/
	if(method == DISABLE) all_data |= 0x00;
	else all_data |= 0x40;
	/*LBT Enable*/
	if(LBT == DISABLE) all_data |= 0x00;
	else all_data |= 0x10;
	/*WOR Cycle*/
	if(wor == _500ms) all_data |= 0x00;
	else if(wor == _1000ms) all_data |= 0x01;
	else if(wor == _1500ms) all_data |= 0x02;
	else if(wor == _2000ms) all_data |= 0x03;
	else if(wor == _2500ms) all_data |= 0x04;
	else if(wor == _3000ms) all_data |= 0x05;
	else if(wor == _3500ms) all_data |= 0x06;
	else if(wor == _4000ms) all_data |= 0x07;

	send_data[3] = all_data;
	HAL_UART_Transmit(&huart1, send_data, 4, TIMEOUT);
	E220_CheckSentReadPacket();
	E220_GlobReg.reg3 = all_data;
}
void E220_Set_Key(uint16_t key)
{
	uint8_t key_h = 0x00;
	uint8_t key_l = 0x00;
	uint8_t send_data[5] = {E220_CMD0, E220_CRYPT_H, 2, key_h, key_l};
	uint16_t mask = 0xFFFF;

	key_h = (key >> 8) & mask;
	key_l = (key << 0) & mask;

	send_data[3] = key_h;
	send_data[4] = key_l;

	HAL_UART_Transmit(&huart1, send_data, 5, TIMEOUT);
	E220_CheckSentReadPacket();
	E220_GlobReg.Key = key;
}
/*------------------------------------------*/
void E220_Set_Settings(E220_HandleTypeDef str)
{
	E220_Set_AddressModule(str.Address);
	HAL_Delay(50);
	E220_Set_Uart_Rate_Parity_Data(str.Rate, str.Pority, str.AirDataRate);
	HAL_Delay(50);
	E220_Set_SubPacket_RSSI_TransPower(str.SubPacket, str.RSSIAmbientNoiseEnable, str.TransPower);
	HAL_Delay(50);
	E220_Set_Channel(str.ChannelControl);
	HAL_Delay(50);
	E220_Set_RSSI_TransmissionMethod_LBT_WORCycle(str.EnableRSSIByte, str.TransMethod, str.LBT, str.WORCycle);
	HAL_Delay(50);
	E220_Set_Key(str.Key);
	HAL_Delay(50);
}
/*------------------------------------------*/
void E220_SetDefaultSettings(uint16_t u1, uint8_t u2, _Bool u3)
{
	E220_HandleTypeDef E220_Settings = {0, };
	E220_Settings.Address = u1;
	E220_Settings.Rate = Rate_9600;
	E220_Settings.Pority = _8N1;
	E220_Settings.AirDataRate = _2_4k;
	E220_Settings.SubPacket = _200bytes;
	E220_Settings.RSSIAmbientNoiseEnable = DISABLE;
	E220_Settings.TransPower = _22dBm;
	E220_Settings.ChannelControl = u2;
	E220_Settings.EnableRSSIByte = u3;
	E220_Settings.TransMethod = FixedTransmissionMode;
	E220_Settings.LBT = DISABLE;
	E220_Settings.WORCycle = _500ms;
	E220_Settings.Key = 0x0000;

	E220_Set_AddressModule(E220_Settings.Address);
	HAL_Delay(30);
	E220_Set_Uart_Rate_Parity_Data(E220_Settings.Rate, E220_Settings.Pority, E220_Settings.AirDataRate);
	HAL_Delay(30);
	E220_Set_SubPacket_RSSI_TransPower(E220_Settings.SubPacket, E220_Settings.RSSIAmbientNoiseEnable, E220_Settings.TransPower);
	HAL_Delay(30);
	E220_Set_Channel(E220_Settings.ChannelControl);
	HAL_Delay(30);
	E220_Set_RSSI_TransmissionMethod_LBT_WORCycle(E220_Settings.EnableRSSIByte, E220_Settings.TransMethod, E220_Settings.LBT, E220_Settings.WORCycle);
	HAL_Delay(30);
	E220_Set_Key(E220_Settings.Key);
	HAL_Delay(30);
}
/*------------------------------------------*/
/*--------------------*/
/*------------------------------------------*/
uint16_t E220_ReadAddress_REG(void)
{
	uint8_t send_data[3] = {E220_CMD1, E220_ADDH, 2};
	uint8_t recv_data[6] = {0, };
	uint16_t addr = 0x0000;
	HAL_UART_Transmit(&huart1, send_data, 3, TIMEOUT);
	HAL_UART_Receive(&huart1, recv_data, 6, TIMEOUT);
	addr |= (recv_data[4] << 8) | (recv_data[5] << 0);
	return addr;
}
/*------------------------------------------*/
uint8_t E220_ReadREG0_REG(void)
{
	uint8_t send_data[3] = {E220_CMD1, E220_REG0, 1};
	uint8_t recv_data[4] = {0, };

	HAL_UART_Transmit(&huart1, send_data, 3, TIMEOUT);
	HAL_UART_Receive(&huart1, recv_data, 4, TIMEOUT);
	return recv_data[3];
}
/*------------------------------------------*/
uint8_t E220_ReadREG1_REG(void)
{
	uint8_t send_data[4] = {E220_CMD1, E220_REG1, 1};
	uint8_t recv_data[4] = {0, };

	HAL_UART_Transmit(&huart1, send_data, 3, TIMEOUT);
	HAL_UART_Receive(&huart1, recv_data, 4, TIMEOUT);
	return recv_data[3];
}
/*------------------------------------------*/
uint8_t E220_ReadREG2_REG(void)
{
	uint8_t send_data[4] = {E220_CMD1, E220_REG2, 1};
	uint8_t recv_data[5] = {0, };

	HAL_UART_Transmit(&huart1, send_data, 3, TIMEOUT);
	HAL_UART_Receive(&huart1, recv_data, 4, TIMEOUT);
	return recv_data[3];
}
/*------------------------------------------*/
uint8_t E220_ReadREG3_REG(void)
{
	uint8_t send_data[4] = {E220_CMD1, E220_REG3, 1};
	uint8_t recv_data[4] = {0, };

	HAL_UART_Transmit(&huart1, send_data, 3, TIMEOUT);
	HAL_UART_Receive(&huart1, recv_data, 4, TIMEOUT);
	return recv_data[3];
}
/*------------------------------------------*/
/*--------------------*/
/*------------------------------------------*/
_Bool E220_Read_Address(void)
{
	uint8_t addr_h = 0x00;
	uint8_t addr_l = 0x00;
	uint16_t addr = 0x0000;
	addr = E220_ReadAddress_REG();
	uint16_t mask = 0xFFFF;
	addr_h = (addr >> 8) & mask;
	addr_l = (addr << 0) & mask;
	char str[4] = {0, };



	HAL_UART_Transmit(&huart3, (uint8_t*)"Addr:", strlen("Addr:"), TIMEOUT);
	sprintf(str,"0x%02X", addr_h);
	HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), TIMEOUT);
	sprintf(str,"%02X", addr_l);
	HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), TIMEOUT);
	HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", strlen("\r\n"), TIMEOUT);

	if(E220_GlobReg.addr & addr)
	{
		return true;
	}
	else return false;

}
/*------------------------------------------*/
_Bool E220_Read_SerialPortRate(void)
{
	uint8_t reg0 = 0x00;
	uint8_t uart_rate = 0x00;
	reg0 = E220_ReadREG0_REG();
	uart_rate |= (reg0 & 0xE0);

	HAL_UART_Transmit(&huart3, (uint8_t*)"Uart rate:", strlen("Uart rate:"), TIMEOUT);
	if(uart_rate == 0x00) HAL_UART_Transmit(&huart3, (uint8_t*)"1200bps", strlen("1200bps"), TIMEOUT);
	else if(uart_rate == 0x20) HAL_UART_Transmit(&huart3, (uint8_t*)"2400bps", strlen("2400bps"), TIMEOUT);
	else if(uart_rate == 0x40) HAL_UART_Transmit(&huart3, (uint8_t*)"4800bps", strlen("4800bps"), TIMEOUT);
	else if(uart_rate == 0x60) HAL_UART_Transmit(&huart3, (uint8_t*)"9600bps", strlen("9600bps"), TIMEOUT);
	else if(uart_rate == 0x80) HAL_UART_Transmit(&huart3, (uint8_t*)"19200bps", strlen("19200bps"), TIMEOUT);
	else if(uart_rate == 0xA0) HAL_UART_Transmit(&huart3, (uint8_t*)"38400bps", strlen("38400bps"), TIMEOUT);
	else if(uart_rate == 0xC0) HAL_UART_Transmit(&huart3, (uint8_t*)"57600bps", strlen("57600bps"), TIMEOUT);
	else if(uart_rate == 0xE0) HAL_UART_Transmit(&huart3, (uint8_t*)"115200bps", strlen("115200bps"), TIMEOUT);
	HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", strlen("\r\n"), TIMEOUT);

	if(E220_GlobReg.reg0 & uart_rate)
	{
		return true;
	}
	else return false;
}
/*------------------------------------------*/
_Bool E220_Read_SerialParityBit(void)
{
	uint8_t reg0 = 0x00;
	uint8_t parity = 0x00;
	reg0 = E220_ReadREG0_REG();
	parity |= (reg0 & 0x18);

	HAL_UART_Transmit(&huart3, (uint8_t*)"Parity:", strlen("Parity:"), TIMEOUT);
	if(parity == 0x00) HAL_UART_Transmit(&huart3, (uint8_t*)"8N1", strlen("8N1"), TIMEOUT);
	else if(parity == 0x08) HAL_UART_Transmit(&huart3, (uint8_t*)"8O1", strlen("8O1"), TIMEOUT);
	else if(parity == 0x10) HAL_UART_Transmit(&huart3, (uint8_t*)"8E1", strlen("8E1"), TIMEOUT);
	else if(parity == 0x18) HAL_UART_Transmit(&huart3, (uint8_t*)"8N1", strlen("8N1"), TIMEOUT);
	HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", strlen("\r\n"), TIMEOUT);

	if((E220_GlobReg.reg0 & parity) == parity)
	{
		return true;
	}
	else return false;
}
/*------------------------------------------*/
_Bool E220_Read_AirDataRate(void)
{
	uint8_t reg0 = 0x00;
	uint8_t air_data_rate = 0x00;
	reg0 = E220_ReadREG0_REG();
	air_data_rate |= (reg0 & 0x07);

	HAL_UART_Transmit(&huart3, (uint8_t*)"Air data rate:", strlen("Air data rate:"), TIMEOUT);
	if(air_data_rate == 0x00) HAL_UART_Transmit(&huart3, (uint8_t*)"2.4k", strlen("2.4k"), TIMEOUT);
	else if(air_data_rate == 0x01) HAL_UART_Transmit(&huart3, (uint8_t*)"2.4k", strlen("2.4k"), TIMEOUT);
	else if(air_data_rate == 0x02) HAL_UART_Transmit(&huart3, (uint8_t*)"2.4k", strlen("2.4k"), TIMEOUT);
	else if(air_data_rate == 0x03) HAL_UART_Transmit(&huart3, (uint8_t*)"4.8k", strlen("4.8k"), TIMEOUT);
	else if(air_data_rate == 0x04) HAL_UART_Transmit(&huart3, (uint8_t*)"9.6k", strlen("9.6k"), TIMEOUT);
	else if(air_data_rate == 0x05) HAL_UART_Transmit(&huart3, (uint8_t*)"19.2k", strlen("19.2k"), TIMEOUT);
	else if(air_data_rate == 0x05) HAL_UART_Transmit(&huart3, (uint8_t*)"38.4k", strlen("38.4k"), TIMEOUT);
	else if(air_data_rate == 0x07) HAL_UART_Transmit(&huart3, (uint8_t*)"62.5k", strlen("62.5k"), TIMEOUT);
	HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", strlen("\r\n"), TIMEOUT);

	if((E220_GlobReg.reg0 & air_data_rate) == air_data_rate)
	{
		return true;
	}
	else return false;
}
/*------------------------------------------*/
_Bool E220_Read_SubPacketSetting(void)
{
	uint8_t reg1 = 0x00;
	uint8_t sub_packet_set = 0x00;
	reg1 = E220_ReadREG1_REG();
	sub_packet_set |= (reg1 & 0xC0);

	HAL_UART_Transmit(&huart3, (uint8_t*)"Sub packet setting:", strlen("Sub packet setting:"), TIMEOUT);
	if(sub_packet_set == 0x00) HAL_UART_Transmit(&huart3, (uint8_t*)"200bytes", strlen("200bytes"), TIMEOUT);
	else if(sub_packet_set == 0x40) HAL_UART_Transmit(&huart3, (uint8_t*)"128bytes", strlen("128bytes"), TIMEOUT);
	else if(sub_packet_set == 0x80) HAL_UART_Transmit(&huart3, (uint8_t*)"64bytes", strlen("64bytes"), TIMEOUT);
	else if(sub_packet_set == 0xC0) HAL_UART_Transmit(&huart3, (uint8_t*)"32bytes", strlen("32bytes"), TIMEOUT);
	HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", strlen("\r\n"), TIMEOUT);

	if((E220_GlobReg.reg1 & sub_packet_set) == sub_packet_set)
	{
		return true;
	}
	else return false;
}
/*------------------------------------------*/
_Bool E220_Read_RSSIAmbientNoiseEnable(void)
{
	uint8_t reg1 = 0x00;
	uint8_t RSSI_ANE = 0x00;
	reg1 = E220_ReadREG1_REG();
	RSSI_ANE |= (reg1 & 0x20);

	HAL_UART_Transmit(&huart3, (uint8_t*)"RSSI ambient noise enable:", strlen("RSSI ambient noise enable:"), TIMEOUT);
	if(RSSI_ANE == 0x00) HAL_UART_Transmit(&huart3, (uint8_t*)"DISABLE", strlen("DISABLE"), TIMEOUT);
	else if(RSSI_ANE == 0x20) HAL_UART_Transmit(&huart3, (uint8_t*)"ENABLE", strlen("ENABLE"), TIMEOUT);
	HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", strlen("\r\n"), TIMEOUT);

	if((E220_GlobReg.reg1 & RSSI_ANE) == RSSI_ANE)
	{
		return true;
	}
	else return false;
}
/*------------------------------------------*/
_Bool E220_Read_TransmitiingPower(void)
{
	uint8_t reg1 = 0x00;
	uint8_t TransPow = 0x00;
	reg1 = E220_ReadREG1_REG();
	TransPow |= (reg1 & 0x03);

	HAL_UART_Transmit(&huart3, (uint8_t*)"Transmitting power:", strlen("Transmitting power:"), TIMEOUT);
	if(TransPow == 0x00) HAL_UART_Transmit(&huart3, (uint8_t*)"22dBm", strlen("22dBm"), TIMEOUT);
	else if(TransPow == 0x01) HAL_UART_Transmit(&huart3, (uint8_t*)"17dBm", strlen("17dBm"), TIMEOUT);
	else if(TransPow == 0x02) HAL_UART_Transmit(&huart3, (uint8_t*)"13dBm", strlen("13dBm"), TIMEOUT);
	else if(TransPow == 0x03) HAL_UART_Transmit(&huart3, (uint8_t*)"10dBm", strlen("10dBm"), TIMEOUT);
	HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", strlen("\r\n"), TIMEOUT);

	if((E220_GlobReg.reg1 & TransPow) == TransPow)
	{
		return true;
	}
	else return false;
}
/*------------------------------------------*/
_Bool E220_Read_ChannelControl(void)
{
	uint8_t reg2 = 0x00;
	uint8_t channel_control = 0x00;
	reg2 = E220_ReadREG2_REG();
	channel_control |= (reg2 & 0xFF);
	char str[4] = {0, };

	HAL_UART_Transmit(&huart3, (uint8_t*)"Channel control:", strlen("Channel control:"), TIMEOUT);
	sprintf(str,"0x%02X", channel_control);
	HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), TIMEOUT);
	HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", strlen("\r\n"), TIMEOUT);

	if((E220_GlobReg.reg2 & channel_control) == channel_control)
	{
		return true;
	}
	else return false;
}
/*------------------------------------------*/
_Bool E220_Read_EnableRSSIByte(void)
{
	uint8_t reg3 = 0x00;
	uint8_t enable_RSSI = 0x00;
	reg3 = E220_ReadREG3_REG();
	enable_RSSI |= (reg3 & 0x80);

	HAL_UART_Transmit(&huart3, (uint8_t*)"RSSI enable bit:", strlen("RSSI enable bit:"), TIMEOUT);
	if(enable_RSSI == 0x00) HAL_UART_Transmit(&huart3, (uint8_t*)"DISABLE", strlen("DISABLE"), TIMEOUT);
	else if(enable_RSSI == 0x80) HAL_UART_Transmit(&huart3, (uint8_t*)"ENABLE", strlen("ENABLE"), TIMEOUT);
	HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", strlen("\r\n"), TIMEOUT);

	if((E220_GlobReg.reg3 & enable_RSSI) == enable_RSSI)
	{
		return true;
	}
	else return false;
}
/*------------------------------------------*/
_Bool E220_Read_TransmissionMethod(void)
{
	uint8_t reg3 = 0x00;
	uint8_t transmit_method = 0x00;
	reg3 = E220_ReadREG3_REG();
	transmit_method |= (reg3 & 0x40);

	HAL_UART_Transmit(&huart3, (uint8_t*)"Transmission method:", strlen("Transmission method:"), TIMEOUT);
	if(transmit_method == 0x00) HAL_UART_Transmit(&huart3, (uint8_t*)"TRANSPARENT", strlen("TRANSPARENT"), TIMEOUT);
	else if(transmit_method == 0x40) HAL_UART_Transmit(&huart3, (uint8_t*)"FIXED", strlen("FIXED"), TIMEOUT);
	HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", strlen("\r\n"), TIMEOUT);

	if((E220_GlobReg.reg3 & transmit_method) == transmit_method)
	{
		return true;
	}
	else return false;
}
/*------------------------------------------*/
_Bool E220_Read_LBTEnable(void)
{
	uint8_t reg3 = 0x00;
	uint8_t LBT = 0x00;
	reg3 = E220_ReadREG3_REG();
	LBT |= (reg3 & 0x10);

	HAL_UART_Transmit(&huart3, (uint8_t*)"LBT bit:", strlen("LBT bit:"), TIMEOUT);
	if(LBT == 0x00) HAL_UART_Transmit(&huart3, (uint8_t*)"DISABLE", strlen("DISABLE"), TIMEOUT);
	else if(LBT == 0x10) HAL_UART_Transmit(&huart3, (uint8_t*)"ENABLE", strlen("ENABLE"), TIMEOUT);
	HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", strlen("\r\n"), TIMEOUT);

	if((E220_GlobReg.reg3 & LBT) == LBT)
	{
		return true;
	}
	else return false;
}
/*------------------------------------------*/
_Bool E220_Read_WORCycle(void)
{
	uint8_t reg3 = 0x00;
	uint8_t WOR = 0x00;
	reg3 = E220_ReadREG3_REG();
	WOR |= (reg3 & 0x07);

	HAL_UART_Transmit(&huart3, (uint8_t*)"WOR cycle:", strlen("WOR cycle:"), TIMEOUT);
	if(WOR == 0x00) HAL_UART_Transmit(&huart3, (uint8_t*)"500ms", strlen("500ms"), TIMEOUT);
	else if(WOR == 0x01) HAL_UART_Transmit(&huart3, (uint8_t*)"1000ms", strlen("1000ms"), TIMEOUT);
	else if(WOR == 0x02) HAL_UART_Transmit(&huart3, (uint8_t*)"1500ms", strlen("1500ms"), TIMEOUT);
	else if(WOR == 0x03) HAL_UART_Transmit(&huart3, (uint8_t*)"2000ms", strlen("2000ms"), TIMEOUT);
	else if(WOR == 0x04) HAL_UART_Transmit(&huart3, (uint8_t*)"2500ms", strlen("2500ms"), TIMEOUT);
	else if(WOR == 0x05) HAL_UART_Transmit(&huart3, (uint8_t*)"3000ms", strlen("3000ms"), TIMEOUT);
	else if(WOR == 0x06) HAL_UART_Transmit(&huart3, (uint8_t*)"3500ms", strlen("3500ms"), TIMEOUT);
	else if(WOR == 0x07) HAL_UART_Transmit(&huart3, (uint8_t*)"4000ms", strlen("4000ms"), TIMEOUT);
	HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", strlen("\r\n"), TIMEOUT);

	if((E220_GlobReg.reg3 & WOR) == WOR)
	{
		return true;
	}
	else return false;
}

/*	??????????????????????????????????????? Don`t worked!
uint16_t E220_Read_NOW_RSSI(_Bool noise_rssi, _Bool rssi_last)
{
	uint8_t length = 0x00;
	uint8_t back_count = 0;
	uint8_t data_com[8] = {0xC0, 0xC1, 0xC2, 0xC3, 0x00, length};
	uint8_t back_data[5] = {0, };
	if((noise_rssi == true) && (rssi_last == true))
	{
		length = 0x02;
		back_count = 5;
	}
	else if((noise_rssi == true) && (rssi_last == false))
	{
		length = 0x01;
		back_count = 4;
	}
	data_com[5] = length;
	HAL_UART_Transmit(&huart1, data_com, 6, TIMEOUT);
	HAL_UART_Receive(&huart1, back_data, back_count, TIMEOUT);
	uint16_t RSSI = (back_data[3] << 8) | (back_data[4] << 0);
	return  RSSI;
}
*/

/*------------------------------------------*/
void SendOnChannel(uint16_t addr, uint8_t channel, uint8_t *data, uint8_t bytes)
{
	uint8_t count_bytes = 0;
	uint8_t addr_h = 0x00;
	uint8_t addr_l = 0x00;
	uint16_t mask = 0xFFFF;
	addr_h = (addr >> 8) & mask;
	addr_l = (addr << 0) & mask;

	data_send[0] = addr_h;
	data_send[1] = addr_l;
	data_send[2] = channel;

	for(int i = 0; i <= bytes + 2; i++)
	{
		if(i > 2)
		{
			data_send[i] = data[i - 3];
		}
		count_bytes++;
	}

	HAL_UART_Transmit(&huart1, (uint8_t*)&data_send, count_bytes, TIMEOUT);

	//E220_CheckSentReadPacket(); // данная функция не работает из-за проблем с кнопкой
}

/*------------------------------------------*/
/*--------------------*/
/*------------------------------------------*/














