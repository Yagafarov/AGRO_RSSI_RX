/*
 * GUICommunication.h
 *
 *  Created on: 12 окт. 2023 г.
 *      Author: Wyder
 */

#ifndef INC_GUICOMMUNICATION_H_
#define INC_GUICOMMUNICATION_H_

#include "stm32f1xx_hal.h"
#include <stdbool.h>


typedef enum Command
{
	Com_0 = 0,
	Com_1,
	Com_2,
	Com_3,
	Com_4,
	Com_5,
	Com_6,
	Com_7,
	/*...*/
} Command;

typedef struct WYD_GUI_Com
{
	uint8_t command;
	uint8_t section;
	uint8_t subsection;
	uint8_t value;
} WYD_GUI_Com;



/*------------------------------------------------------*/
void ReadCommand(uint8_t* buf);			// Чтение полученной команды
/*------------------------------------------------------*/
Command SelectCommand(uint8_t com);

Command SelectCommand(uint8_t com);
Command SelectCommand(uint8_t com);
Command SelectCommand(uint8_t com);
Command SelectCommand(uint8_t com);
Command SelectCommand(uint8_t com);
Command SelectCommand(uint8_t com);

/*------------------------------------------------------*/
/*------------------------------------------------------*/



/*------------------------------------------------------*/



/*------------------------------------------------------*/
/*------------------------------------------------------*/
/*------------------------------------------------------*/






























#endif /* INC_GUICOMMUNICATION_H_ */
