/*
 * i2c_master.c
 *
 *  Created on: Sep 30, 2025
 *      Author: Kelvin
 */

#include "main.h"
#include "i2c_master.h"

#include <string.h>

extern I2C_HandleTypeDef hi2c1;

#define SLAVE_ADDRESS 8
#define SIZE 2

uint8_t txDataMaster[SIZE];
uint8_t rxDataMaster[SIZE];

uint16_t devAddress = SLAVE_ADDRESS << 1;

void WriteData(uint8_t *pdata, uint8_t len)
{
	memcpy(&txDataMaster[0], pdata, len);

	HAL_I2C_Master_Transmit(&hi2c1, devAddress, txDataMaster, len, 100);
}

void ReadData(uint8_t *pdata, uint8_t len)
{
	HAL_I2C_Master_Receive(&hi2c1, devAddress, pdata, len, 100);

//	memcpy(&rxDataMaster[0], pdata, len);
}
