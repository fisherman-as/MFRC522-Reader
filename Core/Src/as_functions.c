/*
 * as_functions.c
 *
 *  Created on: Oct 14, 2024
 *      Author: user
 */

#include "as_functions.h"


extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;

void EnableChip(STATE state)
{
  if (state==ON)
	  {HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_RESET);}
  else
	  {HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_SET);}
}


void spi_read_as(uint32_t ReadAdress, uint8_t* pvReceiveArray, uint16_t Size)
{

	uint8_t address[2] = {0};
	address[0] = (ReadAdress<<1)|0x80; //address = 1xxxxxx0, xxxxxx = ReadAdress
	EnableChip(ON);
	HAL_SPI_TransmitReceive(&hspi1, address, pvReceiveArray, Size+1, 100);
	EnableChip(OFF);
}















