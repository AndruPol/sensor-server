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

#define FIRMWARE			"0.3"	// версия прошивки
#define BMP085_DELAY_MS		900000	// интервал таймера задержки перед опросом датчика BMP085, мсек

#define MAXSENSORS			16		// max sensors for trace
#define MSGSIZE				16		// message data size
#define MSGDUPTIME			30		// duplicate message timeout, s

typedef enum {
	SENSOR_INFO = 0,	// not implemented
	SENSOR_DATA,
	SENSOR_ERROR
} msgtype_t;

typedef enum {
	DS1820 = 0,	// not implemented
	BH1750,
	DHT,
	BMP085,
	ADC
} sensortype_t;

typedef enum {
	TEMPERATURE = 0,	// not implemented
	HUMIDITY,
	PRESSURE,
	LIGHT,
	VOLTAGE
} valuetype_t;

// message format
typedef struct MESSAGE MESSAGE_T;
struct MESSAGE{
	msgtype_t msgType;			// message type: 0 - info, 1 - sensor value, 2 - sensor error
	uint8_t sensorID;			// remote sensor ID
	sensortype_t sensorType;	// sensor type: 0 - 1-wire, 1 - BH1750, 2 - DHT сенсор, 3 - BMP085
	valuetype_t valueType;		// value type: 0 - temperature, 1 - humidity, 2 - pressure
	uint8_t owkey[8];			// sensor id for 1-wire, sensor number for DHT in owkey[0]
	union	{					// sensor value depend of sensor type
		float	fValue;
		int32_t	iValue;
		uint8_t cValue[4];
	} data;
};

typedef struct MESSAGE_TIME MESSAGE_TIME_T;
struct MESSAGE_TIME{
	uint8_t sensorID;	// remote sensor ID
	uint8_t sensorType;	// sensor type: 0 - 1-wire, 1 - BH1750, 2 - DHT сенсор, 3 - BMP085, 4 - ADC
	uint8_t valueType;	// value type: 0 - temperature, 1 - humidity, 2 - pressure, 3 - light, 4 - voltage
	uint8_t owkey0;		// value for owkey[0] for DHT sensors
	systime_t sysTime;	// system time at message received
};

typedef struct MESSAGE_TRACE MESSAGE_TRACE_T;
struct MESSAGE_TRACE{
	uint8_t count;						// message time counter
	MESSAGE_TIME_T	msgtm[MAXSENSORS];	// message time
};

// convert nrf24l01 5 bytes address to char[11]
void addr_hexstr(uint8_t *addr, uint8_t *str);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H_ */
