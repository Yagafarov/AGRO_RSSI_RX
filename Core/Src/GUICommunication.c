/*
 * GUICommunication.c
 *
 *  Created on: 12 окт. 2023 г.
 *      Author: Wyder
 */

#include "E220.h"
#include <string.h>
#include <stdio.h>
#include "GUICommunication.h"
/*------------------------------------------------------*/
/* Global variables */

uint8_t len_buf = 0;
/*------------------------------------------------------*/

WYD_GUI_Com Com;
/*------------------------------------------------------*/
void ReadCommand(uint8_t* buf)
{
	/* Длина пакета */
	len_buf = sizeof(buf);
	/* Разбиение пакета */
	Com.command = buf[0];
	Com.section = buf[1];
	Com.subsection = buf[2];
	Com.value = buf[3];
//	 = buf[4];


	SelectCommand(Com.command);





	//uint16_t len = 0;
	//CDC_Transmit_FS(, );


}
/*------------------------------------------------------*/
Command SelectCommand(uint8_t com)
{
	if(com == 0xA0) return Com_0;
	else if(com == 0xA1) return Com_1;
	else if(com == 0xA2) return Com_2;
	else if(com == 0xA3) return Com_3;
	else if(com == 0xA4) return Com_4;
	else if(com == 0xA5) return Com_5;
	else if(com == 0xA6) return Com_6;
	else if(com == 0xA7) return Com_7;
}
/*------------------------------------------------------*/

/*------------------------------------------------------*/
/*------------------------------------------------------*/
/*------------------------------------------------------*/
/*------------------------------------------------------*/


/*------------------------------------------------------*/
/*------------------------------------------------------*/


/*------------------------------------------------------*/




































