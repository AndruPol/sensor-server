/*
 * main.h
 *
 *  Created on: 24.10.2013
 *      Author: pae
 */

#ifndef MAIN_H_
#define MAIN_H_
#ifdef __cplusplus
extern "C" {
#endif

#define FIRMWARE			"0.0.2"	// версия прошивки
#define BMP085_DELAY_MS		1500		// интервал таймера задержки перед опросом датчика, мсек

// message format
typedef struct MESSAGE MESSAGE_T;
struct MESSAGE{
	uint8_t msgType;	// message type: 0 - info, 1 - sensor value, 2 - sensor error
	uint8_t errorCode;	// error code for msgType == 0: 0 - normal level, 1 - critical battery level
	uint8_t sensorType;	// sensor type: 0 - 1-wire, 1 - BH1750, 2 - DHT сенсор, 3 - BMP085
	uint8_t valueType;	// value type: 0 - temperature, 1 - humidity, 2 - pressure
	uint8_t owkey[8];	// sensor id for 1-wire, sensor number for DHT in owkey[0]
	union	{			// sensor value depend of sensor type
		float	fValue;
		int32_t	iValue;
		uint8_t ch[4];
	} data;
};

// convert nrf24l01 5 bytes address to char[11]
void addr_hexstr(uint8_t *addr, uint8_t *str);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H_ */
