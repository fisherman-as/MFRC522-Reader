/*
 * as_functions.h
 *
 *  Created on: Oct 14, 2024
 *      Author: user
 */

#ifndef INC_AS_FUNCTIONS_H_
#define INC_AS_FUNCTIONS_H_

#include "main.h"


typedef enum {OFF, ON}STATE;

void EnableChip(STATE state);
void spi_read_as(uint32_t ReadAdress, uint8_t* pvReceiveArray, uint16_t Size);



#endif /* INC_AS_FUNCTIONS_H_ */
