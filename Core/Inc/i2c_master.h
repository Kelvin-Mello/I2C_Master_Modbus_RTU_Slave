/*
 * i2c_master.h
 *
 *  Created on: Sep 30, 2025
 *      Author: Kelvin
 */

#ifndef INC_I2C_MASTER_H_
#define INC_I2C_MASTER_H_

	#include <stdint.h>       // Adicionado para reconhecer uint8_t

	void WriteData(uint8_t *pdata, uint8_t len);
	void ReadData(uint8_t *pdata, uint8_t len);

#endif /* INC_I2C_MASTER_H_ */
