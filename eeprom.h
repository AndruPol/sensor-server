/*
 * eeprom.h
 *
 *  Created on: 11 авг. 2016 г.
 *      Author: andru
 */

#ifndef EEPROM_H_
#define EEPROM_H_

#include "main.h"

#define EEPROM_AREA_START	0
#define EEPROM_AREA_SIZE	((CFGLEN / 32 + 1) * 32)

typedef enum {
	EEPROM_OK,
	EEPROM_ERR_SEEK,
	EEPROM_ERR_READ,
	EEPROM_ERR_WRITE,
} eeprom_error_t;

void eepromInit(void);
void eepromStop(void);
eeprom_error_t eepromRead(uint8_t *buf, uint16_t len);
eeprom_error_t eepromWrite(uint8_t *buf, uint16_t len);

#endif /* EEPROM_H_ */
