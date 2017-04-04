/*
 * eeprom.c
 *
 *  Created on: 11 авг. 2016 г.
 *      Author: andru
 */

#include "ch.h"
#include "hal.h"

#include "eeprom.h"
#include "hal_eeprom.h"
#include "eeprom_conf.h"

#define EEPROM_ADDR   	0b1010000
#define EEPROM_DRIVER	I2CD2
#define EEPROM_PORT		GPIOB
#define EEPROM_SCL		GPIOB_PIN10
#define EEPROM_SDA		GPIOB_PIN11

/* I2C2 */
static const I2CConfig EEPROMConfig = {
    OPMODE_I2C,
    100000,
	STD_DUTY_CYCLE,
};

/* buffer for I2C read/write data */
static uint8_t eeprom_writebuf[EEPROM_PAGE_SIZE+2];
static I2CEepromFileStream efile;

static I2CEepromFileConfig efilecfg = {
  0,
  0,
  EEPROM_SIZE,
  EEPROM_PAGE_SIZE,
  MS2ST(EEPROM_WRITE_TIME_MS),
  &EEPROM_DRIVER,
  EEPROM_ADDR,
  eeprom_writebuf,
};

// init i2c driver for eeprom
void eepromInit(void) {
	if (EEPROM_DRIVER.state < I2C_READY) {
		i2cStart(&EEPROM_DRIVER, &EEPROMConfig);

		/* tune pins for I2C2*/
		palSetPadMode(EEPROM_PORT, EEPROM_SCL, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);
		palSetPadMode(EEPROM_PORT, EEPROM_SDA, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);
	}
}

void eepromStop(void) {
	i2cStop(&EEPROM_DRIVER);
}

eeprom_error_t eepromRead(uint8_t *buf, uint16_t len) {
	EepromFileStream *iefs;

	efilecfg.barrier_low  = EEPROM_AREA_START;
	efilecfg.barrier_hi   = EEPROM_AREA_SIZE;
	iefs = I2CEepromFileOpen(&efile, &efilecfg, EepromFindDevice(EEPROM_DEVICE));
	fileStreamSeek(iefs, 0);
	if (0 != fileStreamGetPosition(iefs))
	    return EEPROM_ERR_SEEK;

	uint8_t status = fileStreamRead(iefs, buf, len);
	if (status != len)
	   return EEPROM_ERR_READ;

	fileStreamClose(iefs);

	return EEPROM_OK;
}

eeprom_error_t eepromWrite(uint8_t *buf, uint16_t len) {
	EepromFileStream *oefs;

	efilecfg.barrier_low  = EEPROM_AREA_START;
	efilecfg.barrier_hi   = EEPROM_AREA_SIZE;
	oefs = I2CEepromFileOpen(&efile, &efilecfg, EepromFindDevice(EEPROM_DEVICE));
	fileStreamSeek(oefs, 0);
	if (0 != fileStreamGetPosition(oefs))
	   return EEPROM_ERR_SEEK;

	/* write */
	uint8_t status = fileStreamWrite(oefs, buf, len);
	if (status != len)
	   return EEPROM_ERR_WRITE;

	/* check */
	uint8_t pos = fileStreamGetPosition(oefs);
	if (pos != len)
	   return EEPROM_ERR_WRITE;

	fileStreamClose(oefs);

	return EEPROM_OK;
}
