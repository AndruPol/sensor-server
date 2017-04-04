/*
 * main.h
 *
 *  Created on: 24.10.2013
 *      Author: pae
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "ch.h"

#include "modbus_slave.h"
#include "nrf24l01.h"

#define MAXDEV		12		// max devices in gateway

typedef enum {
	EVT_MSGIN 	= (1<<0),
	EVT_MBCOIL	= (1<<1),
} EVT_MASK_t;

typedef enum {
	GATE_STATUS,
	GATE_CONFIG,
	BMP085_TEMPERATURE,
	BMP085_PRESSURE
} INTERNAL_MAP;

// типы передаваемых сообщений
typedef enum {
	SENSOR_INFO = 0,
	SENSOR_DATA,
	SENSOR_ERROR,
	SENSOR_CMD,
	CMD_ERROR,
} msgtype_t;

// типы датчиков
typedef enum {
	DS1820 = 0,
	BH1750,
	DHT,
	BMP085,
	ADC,
	HCSR04,
} sensortype_t;

// типы получаемых значений
typedef enum {
	TEMPERATURE = 0,
	HUMIDITY,
	PRESSURE,
	LIGHT,
	VOLTAGE,
	DISTANCE,
} valuetype_t;

typedef enum {
    BH1750_LIGHT = 0,
	DS18B20_TEMPERATURE,
    DHT_TEMPERATURE,
	DHT_HUMIDITY,
} NRF_SENSOR_MAP;

typedef enum {
	CMD_CFGREAD = 1,	// read configuration value
	CMD_CFGWRITE,		// write configuration value
	CMD_RESET,			// reset device
	CMD_SENSREAD = 10,	// read sensor value
	CMD_ON = 20,		// ON
	CMD_OFF,			// OFF
	CMD_ONTM,			// ON timeout (S) message.data.iValue
	CMD_OFFTM,			// OFF timeout (S) message.data.iValue
	CMD_GETREG,			// GET register value
	CMD_SETREG,			// SET register value
} command_t;

// message length
#define MSGLEN		sizeof(MESSAGE_T)

// NRF receive/send message format
typedef struct MESSAGE MESSAGE_T;
struct MESSAGE {
	msgtype_t msgType;			// message type
	uint8_t deviceID;			// remote device ID
	sensortype_t sensorType;	// sensor type: 0 - 1-wire, 1 - BH1750, 2 - DHT сенсор, 3 - BMP085
	valuetype_t valueType;		// value type: 0 - temperature, 1 - humidity, 2 - pressure
	uint8_t address;			// internal sensor address
	command_t command;			// command
	uint8_t error;				//
	uint8_t notused[5];			//
	union {						// sensor value depend of sensor type
		float	fValue;
		int32_t	iValue;
		uint8_t cValue[4];
	} data;
};

// command length
#define CMDLEN		sizeof(CMD_T)

// remote sensor command
typedef struct CMD CMD_T;
struct CMD {
	uint8_t deviceID;			// remote device ID
	uint8_t address;			// internal address in remote device
	command_t command;			// command
	uint32_t param;				// command parameter depend of command
};

#define NRF_CMD_BUFFERS		4	// command queue length for each device

// remote sensor command queue
typedef struct DEVCMD DEVCMD_T;
struct DEVCMD {
	volatile uint8_t deviceID;			// remote device ID
	volatile uint8_t cmdcnt;			// command count in maibox
	volatile int16_t delay;				// delay before send command to device
	CMD_T nrf_cmd_buf[NRF_CMD_BUFFERS];
	CMD_T *cmd_free[NRF_CMD_BUFFERS];
	CMD_T *cmd_fill[NRF_CMD_BUFFERS];
	mailbox_t mb_cmd_free;
	mailbox_t mb_cmd_fill;
};

// eeprom configuration format
typedef struct CONFIG CONFIG_T;
struct CONFIG {
	uint16_t magic;							// magic data
	uint16_t bmp085_poll;					// BMP085 polling interval, s
	mb_devmap_t	devmap[MAXDEV];				// modbus devices mapping array
	uint8_t nrfaddr_rcv[NRF_ADDR_LEN];		// nRF24 receive address
	uint8_t nrfadrr_snd[NRF_ADDR_LEN];		// nRF24 send address (first 4 bytes)
	uint8_t mb_addr;						// modbus address
	mb_bitrate_t mb_bitrate;				// modbus bitrate enum
	mb_parity_t mb_parity;					// modbus parity enum
};

// eeprom config length
#define CFGLEN		sizeof(CONFIG_T)

// internal register map
typedef enum {
	GATE_REG_CMD = 1,
	GATE_MB_ADDR,			// address registers: lo - status, hi - uint8_t mb_addr;
	GATE_MB_BITRATE,
	GATE_MB_PARITY,			// address registers: lo - mb_bitrate_t mb_bitrate;, hi - mb_parity_t mb_parity;
	GATE_BMP085_POLL,		// address register: uint16_t bmp085_poll
	GATE_NRF_RCVADDR0,		// address registers: lo - nrfaddr_rcv[0], lo - nrfaddr_rcv[1]
	GATE_NRF_RCVADDR1,		// address registers: lo - nrfaddr_rcv[2], lo - nrfaddr_rcv[3]
	GATE_NRF_RCVADDR2,		// address registers: lo - nrfaddr_rcv[4]
	GATE_NRF_RCVADDR3,		// address registers: lo - nrfaddr_rcv[4]
	GATE_NRF_RCVADDR4,		// address registers: lo - nrfaddr_rcv[4]
	GATE_NRF_SNDADDR0,		// address registers: lo - nrfadrr_snd[0], lo - nrfadrr_snd[1]
	GATE_NRF_SNDADDR1,		// address registers: lo - nrfadrr_snd[2], lo - nrfadrr_snd[3]
	GATE_NRF_SNDADDR2,		// address registers: lo - nrfadrr_snd[4]
	GATE_NRF_SNDADDR3,		// address registers: lo - nrfadrr_snd[4]
	GATE_NRF_SNDADDR4,		// address registers: lo - nrfadrr_snd[4]
	GATE_DEV_MAP,
} gate_regmap_t;

// internal registers map legth
#define REGLEN		(15 + (MAXDEV * sizeof(mb_devmap_t)))

typedef enum {
	GATE_NO_ERROR,		// no error
	GATE_CMD_ERROR,		// command error
	GATE_CMD_PARAM,		// command parameter error
	GATE_CFG_WRITE,		// config write error
	GATE_READ_OVRF,		// read & parse queue overflow
	GATE_SEND_OVRF,		// send queue overflow
	GATE_CMD_OVRF,		// command queue overflow
	GATE_SEND_TIMEOUT,	// nrf send timeout
} gate_error_t;

extern event_source_t event_src;
extern CONFIG_T config;

void port_halt(void);

#endif /* MAIN_H_ */
