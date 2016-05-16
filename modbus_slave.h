/*
 * modbus_slave.h
 *
 *  Created on: 29 июня 2015 г.
 *      Author: andru
 */

#ifndef MODBUS_SLAVE_H_
#define MODBUS_SLAVE_H_

#include "port.h"

/* -----------------------описание регистров -------------------------------------*/
/*
#define S_DISCRETE						(20)	- флаги ошибок данных с беспроводных датчиков
#define S_DISCRETE_INPUT_START      0	(+2)	- ошибки встроенного датчика BMP085
										(+2)	- не используются
										(+4)	- sensorID == 0 датчик на балконе
										(+4)	- sensorID == 1 датчик в спальне
										(+4)	- sensorID == 2 датчик на кухне
										(+4)	- sensorID == 3 датчик в зале
#define S_COIL               			(20)	- признаки получения данных от датчиков
#define S_COIL_START		        0	(+2)	- обновлены данные встроенного датчика BMP085
										(+2)	- не используются
										(+4)	- sensorID == 0 датчик на балконе
										(+4)	- sensorID == 1 датчик в спальне
										(+4)	- sensorID == 2 датчик на кухне
										(+4)	- sensorID == 3 датчик в зале
#define S_REG_INPUT						(20)	- данные датчика * 10 в формате integer
#define S_REG_INPUT_START	        0	(+2)	- данные встроенного датчика BMP085
										(+2)	- не используются
										(+4)	- sensorID == 0 датчик на балконе
										(+4)	- sensorID == 1 датчик в спальне
										(+4)	- sensorID == 2 датчик на кухне
										(+4)	- sensorID == 3 датчик в зале
*/
/* -----------------------Slave Defines -------------------------------------*/
#define S_DISCRETE_INPUT_START        0
#define S_DISCRETE_INPUT_NDISCRETES   20
#define S_COIL_START                  0
#define S_COIL_NCOILS                 20
#define S_REG_INPUT_START             0
#define S_REG_INPUT_NREGS             20
#define S_REG_HOLDING_START           0
#define S_REG_HOLDING_NREGS           0
/* salve mode: holding register's all address */
#define S_HD_RESERVE                  0
#define S_HD_CPU_USAGE_MAJOR          1
#define S_HD_CPU_USAGE_MINOR          2
/* salve mode: input register's all address */
#define S_IN_RESERVE                  0
/* salve mode: coil's all address */
#define S_CO_RESERVE                  0
/* salve mode: discrete's all address */
#define S_DI_RESERVE                  0

//Slave mode:DiscreteInputs variables
extern USHORT   usSDiscInStart;
extern UCHAR    ucSDiscInBuf[];
//Slave mode:Coils variables
extern USHORT   usSCoilStart;
extern UCHAR    ucSCoilBuf[];
//Slave mode:InputRegister variables
extern USHORT   usSRegInStart;
extern SHORT   usSRegInBuf[];
//Slave mode:HoldingRegister variables
extern USHORT   usSRegHoldStart;
extern SHORT   usSRegHoldBuf[];

// create & start modbus poll process
void createModbusThd(void);
void setReceivedBit(uint16_t regAddr, uint8_t ucValue);
void setErrorBit(uint16_t regAddr, uint8_t ucValue);
void writeInputReg(uint16_t regAddr, float regValue);

#endif /* MODBUS_SLAVE_H_ */
