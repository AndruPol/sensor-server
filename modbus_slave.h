/*
 * modbus_slave.h
 *
 *  Created on: 29 июня 2015 г.
 *      Author: andru
 */

#ifndef MODBUS_SLAVE_H_
#define MODBUS_SLAVE_H_

#include "port.h"

/* -----------------------Slave Defines -------------------------------------*/
#define MB_PORT		1				// not used now
#define MB_MODE		MB_RTU			// rtu mode only
#define MB_ADDR		1				// slave address
#define MB_BITRATE	MB_BR_19200		//
#define MB_PARITY	MB_PARITY_NONE	//

typedef enum {
	MB_BR_2400,
	MB_BR_9600,
	MB_BR_19200,
	MB_BR_38400,
	MB_BR_57600,
	MB_BR_115200,
} mb_bitrate_t;

typedef enum {
    MB_PARITY_NONE, /*!< No parity. */
    MB_PARITY_ODD,  /*!< Odd parity. */
    MB_PARITY_EVEN  /*!< Even parity. */
} mb_parity_t;

typedef enum {
	DEV_INT,		// внутреннее устройство
	DEV_EXT,		// удаленное устройство
} devtype_t;

typedef enum {
	CMD_NONE,		// command none
	CMD_RUN,		// command send immediately
	CMD_DELAY,		// command with wait message from device
} devcmd_t;

typedef struct mb_devmap mb_devmap_t;
struct mb_devmap {
	uint8_t id;			// идентификатор устройства
	devtype_t type;		// тип внутреннее/удаленное
	devcmd_t cmd;		// устройство поддерживает команды шлюза
	uint8_t addr;		// количество датчиков устройства
};

extern USHORT usSNRegs;
extern USHORT usSCoilLen;
extern UCHAR *	pucSChgCoilBuf;

// create & start modbus poll process
void createModbusThd(void);
void setDiscreteBit(uint16_t regAddr, uint8_t ucValue);
uint8_t getCoilBit(uint16_t regAddr);
void setCoilBit(uint16_t regAddr, uint8_t ucValue);
void writeInputRound(uint16_t regAddr, float regValue);
void writeInput(uint16_t regAddr, int16_t val);
USHORT readHolding(uint16_t regAddr);
void writeHoldingRound(uint16_t regAddr, float regValue);
void writeHolding(uint16_t regAddr, int16_t val);

uint16_t mbmap_nregcount(void);
int16_t mbmap_nreg(uint8_t device, uint8_t addr, devtype_t devtype);
uint8_t mbmap_search(uint16_t reg, mb_devmap_t *mb_device);

#endif /* MODBUS_SLAVE_H_ */
