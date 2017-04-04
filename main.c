/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#include "main.h"
#include "nrf24l01.h"
#include "bmp085.h"
#include "eeprom.h"
#include "util/printfs.h"
#include "util/crc8.h"
#include "iwdg_driver.h"

#include <string.h>

#define FIRMWARE			201		// версия прошивки
#define MAGIC				0xAE68	// eeprom magic data
#define BMP085_POLL_S		90		// интервал таймера опроса датчика BMP085, сек
#define NRFSEND_MAX			5

//#define OLDDEVMAX			0		// old devices id max
#define CMDTIMERINT			10		// command delay timer interval, mS
#define CMDDELAY			5		// delay before send command after receive message = CMDDELAY * CMDTIMERINT (mS)

/* ----------------------- Modbus includes ----------------------------------*/
#include "modbus_slave.h"

#define LED_GPIO			GPIOB
#define LED_PIN				2

#define USE_AES				1		// AES encryption flag
#define USE_BMP085			1		// BMP085 polling
#define DEBUG				0

#if USE_AES
#include "aes.h"
#include "aes_secret.h"
#endif

CONFIG_T config;
MESSAGE_T rcvmsg;
MESSAGE_T *prcvMsg = &rcvmsg;

static virtual_timer_t bmp085Timer, cmdTimer;
static binary_semaphore_t bmp085sem;
event_source_t event_src, cmd_src;

#define NRF_READ_BUFFERS	6
static MESSAGE_T nrf_read_buf[NRF_READ_BUFFERS];
static MESSAGE_T *read_free[NRF_READ_BUFFERS];
static MESSAGE_T *read_fill[NRF_READ_BUFFERS];
static mailbox_t mb_read_free, mb_read_fill;

#define NRF_SEND_BUFFERS	6
static MESSAGE_T nrf_send_buf[NRF_SEND_BUFFERS];
static MESSAGE_T *send_free[NRF_SEND_BUFFERS];
static MESSAGE_T *send_fill[NRF_SEND_BUFFERS];
static mailbox_t mb_send_free, mb_send_fill;

static DEVCMD_T * pdevcmd;
static volatile uint8_t devcmdcnt;

static void executeCmd4Gate(uint16_t command, uint16_t param);

#define GATE_CMD_NONE		0
#define GATE_CMD_RESTART	1
#define GATE_CMD_CFGWRITE	2
static volatile uint8_t cmd_flag;

/*===========================================================================*/
/* IWDG related.
         */
/*===========================================================================*/
static const IWDGConfig iwdgcfg = {
        2400,                 // 2400 * 1.6mS = ~3.84S IWDG reset time
        IWDG_DIV_64           // 64 / ~40000 = 1.6mS
};

static uint8_t devcmd_search(uint8_t deviceID, uint8_t *devidx) {
	for (uint8_t i = 0; i < devcmdcnt; i++) {
	  if (pdevcmd[i].deviceID == deviceID) {
		  *devidx = i;
		  return 1;
	  }
	}
	return 0;
}

// set modbus error status for gateway
static void gate_error(gate_error_t error, uint8_t param) {
	int16_t nreg = mbmap_nreg(0, GATE_STATUS, DEV_INT);
	if (nreg >= 0) {
		writeInput(nreg, error);
		writeHolding(nreg, param);
		setCoilBit(nreg, 1);
	}
}

static void cmdTimer_handler(void *arg) {
	(void) arg;

	chSysLockFromISR();
	for (uint8_t i = 0; i < devcmdcnt; i++) {
	  if (pdevcmd[i].cmdcnt == 0) continue;
	  if (pdevcmd[i].delay == 0)	{
		  chEvtBroadcastI(&cmd_src);
	  } else if (pdevcmd[i].delay > 0) {
		  pdevcmd[i].delay--;
	  }
	}
	chVTSetI(&cmdTimer, MS2ST(CMDTIMERINT), cmdTimer_handler, 0);
	chSysUnlockFromISR();
}

/*
 * Triggered when the NRF24L01 triggers an interrupt
 */
static void extcbnrf(EXTDriver *extp, expchannel_t channel) {
	(void)extp;
	(void)channel;

  	chSysLockFromISR();
	NRFReportIRQ();
	chSysUnlockFromISR();
}

static const EXTConfig extcfg = {
  {
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_FALLING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOB, extcbnrf},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
  }
};


/*===========================================================================*/
/* Main and generic code.                                                    */
/*===========================================================================*/

// BMP085 polling timer callback
static void bmp085Timer_handler(void *arg){
	(void)arg;
	chSysLockFromISR();
	chBSemSignalI(&bmp085sem);
	chVTSetI(&bmp085Timer, S2ST(BMP085_POLL_S), bmp085Timer_handler, 0);
	chSysUnlockFromISR();
}

/*
 * BMP085 polling thread
 */
static THD_WORKING_AREA(waBMPPollThread, 256);
static THD_FUNCTION(BMPPollThread, arg) {
	(void)arg;
	chRegSetThreadName("BMPPollThd");

	while (TRUE) {
		chBSemWait(&bmp085sem);

		int16_t temperature;
		uint32_t pressure;
		int16_t nreg;
		bmp085_error_t ret = bmp085_read(&temperature, &pressure);
		if (ret == BMP085_OK) {
			nreg = mbmap_nreg(0, BMP085_TEMPERATURE, DEV_INT);
			if (nreg >= 0)
			  writeHoldingRound(nreg, (float)temperature);
			nreg = mbmap_nreg(0, BMP085_PRESSURE, DEV_INT);
			if (nreg >= 0)
			  writeHoldingRound(nreg, (float)pressure*75/1000);
		}
		else {
			nreg = mbmap_nreg(0, BMP085_TEMPERATURE, DEV_INT);
			if (nreg >= 0) {
				setDiscreteBit(nreg, 1);
				writeInput(nreg, ret);
				setCoilBit(nreg, 1);
			}
			nreg = mbmap_nreg(0, BMP085_PRESSURE, DEV_INT);
			if (nreg >= 0) {
				setDiscreteBit(nreg, 1);
				writeInput(nreg, ret);
				setCoilBit(nreg, 1);
			}
		}
	} //while
}

static THD_WORKING_AREA(waEventThread, 384);
static THD_FUNCTION(eventThread, arg) {
	(void)arg;
	event_listener_t event_el;

	chRegSetThreadName("evtThd");

	chEvtObjectInit(&event_src);
	chEvtRegister(&event_src, &event_el, 0);

	for (uint8_t i = 0; i < devcmdcnt; i++) {
		chMBObjectInit(&pdevcmd[i].mb_cmd_free, (msg_t *) pdevcmd[i].cmd_free, NRF_CMD_BUFFERS);
		chMBObjectInit(&pdevcmd[i].mb_cmd_fill, (msg_t *) pdevcmd[i].cmd_fill, NRF_CMD_BUFFERS);

		// fill cmd buffers mailbox
		for (uint8_t j=0; j < NRF_CMD_BUFFERS; j++)
			chMBPost(&pdevcmd[i].mb_cmd_free, (msg_t) &pdevcmd[i].nrf_cmd_buf[j], TIME_IMMEDIATE);
	}

	while (TRUE) {
		chEvtWaitAny(EVENT_MASK(0));

		chSysLock();
	    eventflags_t flags = chEvtGetAndClearFlagsI(&event_el);
	    chSysUnlock();

	    // NRF incoming message event
	    if (flags & EVT_MSGIN) {
			for (uint8_t i = 0; i < devcmdcnt; i++) {
			  if (pdevcmd[i].deviceID == prcvMsg->deviceID && pdevcmd[i].cmdcnt > 0) {
				  pdevcmd[i].delay = CMDDELAY;
				  break;
			  }
			}
	    }

	    // Modbus coil write event
	    if (flags & EVT_MBCOIL) {
	    	mb_devmap_t mb_device;

	    	uint16_t regIndex = 0;
            for (uint16_t i = 0; i < usSCoilLen; i++) {
            	if (pucSChgCoilBuf[i] > 0) {
            		uint16_t word = ((uint16_t) pucSChgCoilBuf[i]) << 8;
            		for (uint8_t j = 1; j <= 8; j++) {
            			if (regIndex >= usSNRegs) break;
            			if ((word >> j & 0x0080) && regIndex % 2 && getCoilBit(regIndex)) {
            				if (mbmap_search(regIndex, &mb_device) && readHolding(regIndex) > 0) {
            					if (mb_device.type == DEV_INT && mb_device.addr == GATE_CONFIG) {
            					  // command to gate
            					  executeCmd4Gate(readHolding(regIndex), readHolding(regIndex-1));
								  setCoilBit(regIndex, 0);
            					} else if (mb_device.type == DEV_EXT && mb_device.cmd == CMD_DELAY) {
								  // post command to mailbox & wait device message before send
								  void *pbuf;
								  CMD_T cmd = {
									deviceID : mb_device.id,
									address : mb_device.addr,
									command : readHolding(regIndex),
									param : readHolding(regIndex-1),
								  };
								  uint8_t devidx;
								  if (devcmd_search(mb_device.id, &devidx) &&
									  chMBFetch(&pdevcmd[devidx].mb_cmd_free, (msg_t *) &pbuf, TIME_IMMEDIATE) == MSG_OK) {
									  memcpy(pbuf, &cmd, CMDLEN);
									  if (chMBPost(&pdevcmd[devidx].mb_cmd_fill, (msg_t) pbuf, TIME_IMMEDIATE) == MSG_OK) {
										  setCoilBit(regIndex, 0);
										  pdevcmd[devidx].cmdcnt++;
										  pdevcmd[devidx].delay = -1;
										  writeInput(regIndex, pdevcmd[devidx].cmdcnt);
									  }
								  } else {
									  gate_error(GATE_CMD_OVRF, 0);
								  }

            					} else if (mb_device.type == DEV_EXT && mb_device.cmd == CMD_RUN) {
  								  // send command to device immediately
            					  MESSAGE_T sndmsg;
           						  memset(&sndmsg, 0, MSGLEN);
           						  sndmsg.msgType = SENSOR_CMD;
           						  sndmsg.deviceID = mb_device.id;
           						  sndmsg.address = mb_device.addr;
           						  sndmsg.command = readHolding(regIndex);
           						  sndmsg.data.iValue = readHolding(regIndex-1);

           						  void *pbuf;
           						  if (chMBFetch(&mb_send_free, (msg_t *) &pbuf, TIME_IMMEDIATE) == MSG_OK) {
           							  memcpy(pbuf, &sndmsg, MSGLEN);
           							  if (chMBPost(&mb_send_fill, (msg_t) pbuf, TIME_IMMEDIATE) == MSG_OK)
           								  setCoilBit(regIndex, 0);
           						  } else {
           							  gate_error(GATE_SEND_OVRF, mb_device.id);
           						  }
            					}
            				} // mbmap_search()
            			}
            			regIndex++;
            		}
            	} else {
            		regIndex += 8;
            	}
            }

	    }

	} //while
}

static THD_WORKING_AREA(waNRFSendThread, 384);
static THD_FUNCTION(nrfSendThread, arg) {
	(void)arg;
	chRegSetThreadName("nrfSend");

	chMBObjectInit(&mb_send_free, (msg_t*) send_free, NRF_SEND_BUFFERS);
	chMBObjectInit(&mb_send_fill, (msg_t*) send_fill, NRF_SEND_BUFFERS);

	// fill free buffers mailbox
	for (uint8_t i=0; i < NRF_SEND_BUFFERS; i++)
		chMBPost(&mb_send_free, (msg_t) &nrf_send_buf[i], TIME_IMMEDIATE);

	while (TRUE) {
		MESSAGE_T msg;
		MESSAGE_T *pbuf;
		uint8_t device_id;
		if (chMBFetch(&mb_send_fill, (msg_t *) &pbuf, TIME_INFINITE) == MSG_OK) {
			memcpy(&msg, pbuf, MSGLEN);
			chMBPost(&mb_send_free, (msg_t) pbuf, TIME_IMMEDIATE);
		} else {
			continue;
		}

		device_id = msg.deviceID;
		if (nrfSndAddr[NRF_ADDR_LEN-1] != device_id) {
			nrfSndAddr[NRF_ADDR_LEN-1] = device_id;
			NRFChangeSendAddr(nrfSndAddr, NRF_ADDR_LEN);
		}

#if USE_AES
		MESSAGE_T aesmsg;
		AES128_ECB_encrypt((uint8_t*) &msg, aes_key, (uint8_t*) &aesmsg);
		memcpy(&msg, &aesmsg, MSGLEN);
#endif
		uint8_t sendcnt = NRFSEND_MAX;
		while (!NRFSendData((uint8_t *) &msg) && --sendcnt);

		if (sendcnt == 0) {
			// set nRF24L01 send error
			gate_error(GATE_SEND_TIMEOUT, device_id);
		}
	}
}

static THD_WORKING_AREA(waCmdThread, 256);
static THD_FUNCTION(cmdThread, arg) {
	(void)arg;
	event_listener_t cmd_el;

	chRegSetThreadName("cmdThd");

	chEvtObjectInit(&cmd_src);
	chEvtRegister(&cmd_src, &cmd_el, 0);

	while (TRUE) {
		chEvtWaitAny(EVENT_MASK(0));

	    // send command to device flag
		for (uint8_t i = 0; i < devcmdcnt; i++) {
		  if (pdevcmd[i].cmdcnt > 0 && pdevcmd[i].delay == 0)	{
			  CMD_T *pcmd;
			  if (chMBFetch(&pdevcmd[i].mb_cmd_fill, (msg_t *) &pcmd, TIME_INFINITE) == MSG_OK) {
				  chMBPost(&pdevcmd[i].mb_cmd_free, (msg_t) pcmd, TIME_IMMEDIATE);
				  pdevcmd[i].cmdcnt--;
				  pdevcmd[i].delay = -1;

				  if (pdevcmd[i].cmdcnt > 0) {
					  pdevcmd[i].delay = CMDDELAY;
				  }

				  int16_t nreg = mbmap_nreg(pcmd->deviceID, pcmd->address, DEV_EXT);
				  if (nreg >= 0)
					  writeInput(nreg+1, pdevcmd[i].cmdcnt);

				  MESSAGE_T sndmsg;
				  memset(&sndmsg, 0, MSGLEN);
				  sndmsg.msgType = SENSOR_CMD;
				  sndmsg.deviceID = pcmd->deviceID;
				  sndmsg.address = pcmd->address;
				  sndmsg.command = pcmd->command;
				  sndmsg.data.iValue = pcmd->param;

				  void *pbuf;
				  if (chMBFetch(&mb_send_free, (msg_t *) &pbuf, TIME_IMMEDIATE) == MSG_OK) {
					  memcpy(pbuf, &sndmsg, MSGLEN);
					  chMBPost(&mb_send_fill, (msg_t) pbuf, TIME_IMMEDIATE);
				  } else {
					  gate_error(GATE_SEND_OVRF, 0);
				  }
			  }
		  }
		}
	} //while
}

static void parseSensorInfo(MESSAGE_T *msg) {
	int16_t nreg;

	if (msg->msgType != SENSOR_INFO)
		return;

	nreg = mbmap_nreg(msg->deviceID, msg->address, DEV_EXT);
	if (nreg >= 0) {
		writeHolding(nreg, (int16_t) msg->data.iValue);
		setDiscreteBit(nreg, 0);
		setCoilBit(nreg, 1);
	}
}

static void parseSensorData(MESSAGE_T *msg) {
	int16_t nreg;

	if (msg->msgType != SENSOR_DATA)
		return;

	switch (msg->sensorType) {
	case DS1820:
	  nreg = mbmap_nreg(msg->deviceID, msg->address, DEV_EXT);
	  if (nreg >= 0) {
		   writeHoldingRound(nreg, (float)msg->data.iValue);
	  }
	  break;
	case BH1750:
	  nreg = mbmap_nreg(msg->deviceID, msg->address, DEV_EXT);
	  if (nreg >= 0)
		  writeHoldingRound(nreg, (float)msg->data.iValue * 10);
	  break;
	case DHT:
	  if (msg->valueType == TEMPERATURE) {
		  nreg = mbmap_nreg(msg->deviceID, msg->address, DEV_EXT);
		  if (nreg >= 0)
			  writeHoldingRound(nreg, (int16_t)msg->data.iValue);
	  } else if (msg->valueType == HUMIDITY) {
		  nreg = mbmap_nreg(msg->deviceID, msg->address, DEV_EXT);
		  if (nreg >= 0)
			  writeHoldingRound(nreg, (int16_t)msg->data.iValue);
	  }
	  break;
	default:	//UNKNOWN SENSOR
	  nreg = mbmap_nreg(msg->deviceID, msg->address, DEV_EXT);
	  if (nreg >= 0) {
		  writeHolding(nreg, (int16_t) msg->data.iValue);
		  setDiscreteBit(nreg, 0);
		  setCoilBit(nreg, 1);
	  }
	  break;
	}

}

static void parseSensorError(MESSAGE_T *msg) {
	int16_t nreg;

	if (msg->msgType != SENSOR_ERROR)
		return;

	switch (msg->sensorType) {
	case DS1820:
	  nreg = mbmap_nreg(msg->deviceID, msg->address, DEV_EXT);
	  if (nreg >= 0) {
		  setDiscreteBit(nreg, 1);
		  writeInput(nreg, (uint8_t) msg->error);
		  setCoilBit(nreg, 1);
	  }
	  break;
	case BH1750:
	  nreg = mbmap_nreg(msg->deviceID, msg->address, DEV_EXT);
	  if (nreg >= 0) {
		  setDiscreteBit(nreg, 1);
		  writeInput(nreg, (uint8_t) msg->error);
		  setCoilBit(nreg, 1);
	  }
	  break;
	case DHT:
	  nreg = mbmap_nreg(msg->deviceID, msg->address, DEV_EXT);
	  if (nreg >= 0) {
		  setDiscreteBit(nreg, 1);
		  writeInput(nreg, (uint8_t) msg->error);
		  setCoilBit(nreg, 1);
	  }
	  nreg = mbmap_nreg(msg->deviceID, msg->address, DEV_EXT);
	  if (nreg >= 0) {
		  setDiscreteBit(nreg, 1);
		  writeInput(nreg, (uint8_t) msg->error);
		  setCoilBit(nreg, 1);
	  }
	  break;
	case ADC:
	  nreg = mbmap_nreg(msg->deviceID, msg->address, DEV_EXT);
	  if (nreg >= 0) {
		  setDiscreteBit(nreg, 1);
		  writeInput(nreg, (uint8_t) msg->error);
		  setCoilBit(nreg, 1);
	  }
	  break;
	default:	// UNKNOWN ERROR
	  nreg = mbmap_nreg(msg->deviceID, msg->address, DEV_EXT);
	  if (nreg >= 0) {
		  setDiscreteBit(nreg, 1);
		  writeInput(nreg, (uint8_t) msg->error);
		  setCoilBit(nreg, 1);
	  }
	  break;
	}

}

static THD_WORKING_AREA(waNRFReceiveThread, 256);
static THD_FUNCTION(nrfReceiveThread, arg) {
  (void)arg;
  chRegSetThreadName("nrfReceive");

  chMBObjectInit(&mb_read_free, (msg_t*) read_free, NRF_READ_BUFFERS);
  chMBObjectInit(&mb_read_fill, (msg_t*) read_fill, NRF_READ_BUFFERS);

  // fill free buffers mailbox
  for (uint8_t i=0; i < NRF_READ_BUFFERS; i++)
	  chMBPost(&mb_read_free, (msg_t) &nrf_read_buf[i], TIME_IMMEDIATE);

  while (TRUE) {
	  uint8_t pipeNr;
	  uint8_t inbuf[MSGLEN+1];

	  chBSemWait(&nrf.NRFSemRX);
	  memset(&inbuf, 0, MSGLEN+1);
	  NRFReceiveData(&pipeNr, inbuf);

	  if (pipeNr != 1) continue;

	  void *pbuf;
	  if (chMBFetch(&mb_read_free, (msg_t *) &pbuf, TIME_IMMEDIATE) == MSG_OK) {
		  memcpy(pbuf, inbuf, MSGLEN);
		  chMBPost(&mb_read_fill, (msg_t) pbuf, TIME_IMMEDIATE);
	  } else {
		  gate_error(GATE_READ_OVRF, 0);
	  }

  }
}

static THD_WORKING_AREA(waNRFParseThread, 256);
static THD_FUNCTION(nrfParseThread, arg) {
  (void)arg;
  chRegSetThreadName("nrfParse");

#if DEBUG
  char line[60], stype[7];
  char owkey[17];
#endif

  while (TRUE) {
	  void *pbuf;

	  if (chMBFetch(&mb_read_fill, (msg_t *) &pbuf, TIME_INFINITE) == MSG_OK) {
		  memcpy(&rcvmsg, pbuf, MSGLEN);
		  chMBPost(&mb_read_free, (msg_t) pbuf, TIME_IMMEDIATE);
	  } else {
		  continue;
	  }


#if USE_AES
	  AES128_ECB_decrypt(pbuf, aes_key, (uint8_t*) &rcvmsg);
#else
	  memcpy(&rcvmsg, pbuf, MSGLEN);
#endif

	  chEvtBroadcastFlags(&event_src, EVT_MSGIN);

	  switch (prcvMsg->msgType) {
	  case SENSOR_INFO:
		  parseSensorInfo(prcvMsg);
		  break;
	  case SENSOR_DATA:
		  parseSensorData(prcvMsg);
		  break;
	  case SENSOR_ERROR:
		  parseSensorError(prcvMsg);
		  break;
	  default:
		  break;
	  }
  } // while
}

static void default_config(void) {
	  config.magic = MAGIC;
	  config.mb_addr = MB_ADDR;
	  config.mb_bitrate = MB_BITRATE;
	  config.mb_parity = MB_PARITY_NONE;
	  memcpy(config.nrfaddr_rcv, nrfRcvAddr, NRF_ADDR_LEN);
	  memcpy(config.nrfadrr_snd, nrfSndAddr, NRF_ADDR_LEN);
	  config.bmp085_poll = BMP085_POLL_S;
	  memset(&config.devmap, 0, sizeof(mb_devmap_t) * MAXDEV);
	  mb_devmap_t* dev;
	  dev = &config.devmap[0];
	  dev->id = 0;
	  dev->type = DEV_INT;
	  dev->cmd = CMD_RUN;
	  dev->addr = 4;
	  dev = &config.devmap[1];
	  dev->id = 0;
	  dev->type = DEV_EXT;
	  dev->cmd = CMD_DELAY;
	  dev->addr = 8;
	  dev = &config.devmap[2];
	  dev->id = 1;
	  dev->type = DEV_EXT;
	  dev->cmd = CMD_DELAY;
	  dev->addr = 8;
	  dev = &config.devmap[3];
	  dev->id = 2;
	  dev->type = DEV_EXT;
	  dev->cmd = CMD_DELAY;
	  dev->addr = 8;
	  dev = &config.devmap[4];
	  dev->id = 3;
	  dev->type = DEV_EXT;
	  dev->cmd = CMD_DELAY;
	  dev->addr = 8;
	  dev = &config.devmap[5];
	  dev->id = 10;
	  dev->type = DEV_EXT;
	  dev->cmd = CMD_DELAY;
	  dev->addr = 11;
}

static uint8_t write_config(void) {
	uint8_t cfgbuf[CFGLEN+1];

	memcpy(&cfgbuf, &config, CFGLEN);
	cfgbuf[CFGLEN] = CRC8((uint8_t*) &config, CFGLEN);

	uint8_t ret = eepromWrite((uint8_t*) &cfgbuf, CFGLEN+1);
	if (ret != EEPROM_OK) {
		// eeprom configuration write fail
		return false;
	}
	return true;
}

// execute command to gateway
static void executeCmd4Gate(uint16_t command, uint16_t param) {
	uint8_t cmd, addr;

	cmd = command & 0x00FF;
	addr = command >> 8;

	int16_t nreg = mbmap_nreg(0, GATE_CONFIG, DEV_INT);
	if (nreg < 0) return;

	switch (cmd) {
	  case CMD_RESET:
		// board reset
		if (addr == GATE_REG_CMD && param == MAGIC) {
	    	cmd_flag = GATE_CMD_RESTART;
		} else {
			gate_error(GATE_CMD_ERROR, 0);
		}
		break;
	  case CMD_CFGWRITE:
		// write configuration to eeprom
		if (addr == GATE_REG_CMD && param == MAGIC) {
	    	cmd_flag = GATE_CMD_CFGWRITE;
		} else {
			gate_error(GATE_CMD_ERROR, 0);
		}
		break;
	  case CMD_GETREG:
		// read config register value
		if (addr > REGLEN) {
			gate_error(GATE_CMD_PARAM, 0);
			break;
		}
		switch (addr) {
			case GATE_MB_ADDR:
				writeInput(nreg, config.mb_addr);
				setCoilBit(nreg, 1);
				break;
			case GATE_MB_BITRATE:
				writeInput(nreg, config.mb_bitrate);
				setCoilBit(nreg, 1);
				break;
			case GATE_MB_PARITY:
				writeInput(nreg, config.mb_parity);
				setCoilBit(nreg, 1);
				break;
			case GATE_BMP085_POLL:
				writeInput(nreg, config.bmp085_poll);
				setCoilBit(nreg, 1);
				break;
			case GATE_NRF_RCVADDR0:
				writeInput(nreg, config.nrfaddr_rcv[0]);
				setCoilBit(nreg, 1);
				break;
			case GATE_NRF_RCVADDR1:
				writeInput(nreg, config.nrfaddr_rcv[1]);
				setCoilBit(nreg, 1);
				break;
			case GATE_NRF_RCVADDR2:
				writeInput(nreg, config.nrfaddr_rcv[2]);
				setCoilBit(nreg, 1);
				break;
			case GATE_NRF_RCVADDR3:
				writeInput(nreg, config.nrfaddr_rcv[3]);
				setCoilBit(nreg, 1);
				break;
			case GATE_NRF_RCVADDR4:
				writeInput(nreg, config.nrfaddr_rcv[4]);
				setCoilBit(nreg, 1);
				break;
			case GATE_NRF_SNDADDR0:
				writeInput(nreg, config.nrfadrr_snd[0]);
				setCoilBit(nreg, 1);
				break;
			case GATE_NRF_SNDADDR1:
				writeInput(nreg, config.nrfadrr_snd[1]);
				setCoilBit(nreg, 1);
				break;
			case GATE_NRF_SNDADDR2:
				writeInput(nreg, config.nrfadrr_snd[2]);
				setCoilBit(nreg, 1);
				break;
			case GATE_NRF_SNDADDR3:
				writeInput(nreg, config.nrfadrr_snd[3]);
				setCoilBit(nreg, 1);
				break;
			case GATE_NRF_SNDADDR4:
				writeInput(nreg, config.nrfadrr_snd[4]);
				setCoilBit(nreg, 1);
				break;
			default:
				if (addr >= GATE_DEV_MAP) {
					uint8_t* pdevmap = (uint8_t*) config.devmap;
					writeInput(nreg, pdevmap[addr - GATE_DEV_MAP]);
					setCoilBit(nreg, 1);
				}
				break;
		}
		break;
	  case CMD_SETREG:
		// set config register value
		if (addr > REGLEN) {
			gate_error(GATE_CMD_PARAM, 0);
			break;
		}
		switch (addr) {
			case GATE_MB_ADDR:
				if (param > 0 && param < 254) {
					config.mb_addr = param;
					writeInput(nreg, config.mb_addr);
					setCoilBit(nreg, 1);
				} else {
					gate_error(GATE_CMD_PARAM, 0);
				}
				break;
			case GATE_MB_BITRATE:
				if (param <= MB_BR_115200) {
					config.mb_bitrate = param;
					writeInput(nreg, config.mb_bitrate);
					setCoilBit(nreg, 1);
				} else {
					gate_error(GATE_CMD_PARAM, 0);
				}
				break;
			case GATE_MB_PARITY:
				if (param <= MB_PARITY_EVEN) {
					config.mb_parity = param;
					writeInput(nreg, config.mb_parity);
					setCoilBit(nreg, 1);
				} else {
					gate_error(GATE_CMD_PARAM, 0);
				}
				break;
			case GATE_BMP085_POLL:
				config.bmp085_poll = param;
				writeInput(nreg, config.bmp085_poll);
				setCoilBit(nreg, 1);
				break;
			case GATE_NRF_RCVADDR0:
				config.nrfaddr_rcv[0] = param & 0x00FF;
				writeInput(nreg, config.nrfaddr_rcv[0]);
				setCoilBit(nreg, 1);
				break;
			case GATE_NRF_RCVADDR1:
				config.nrfaddr_rcv[1] = param & 0x00FF;
				writeInput(nreg, config.nrfaddr_rcv[1]);
				setCoilBit(nreg, 1);
				break;
			case GATE_NRF_RCVADDR2:
				config.nrfaddr_rcv[2] = param & 0x00FF;
				writeInput(nreg, config.nrfaddr_rcv[2]);
				setCoilBit(nreg, 1);
				break;
			case GATE_NRF_RCVADDR3:
				config.nrfaddr_rcv[3] = param & 0x00FF;
				writeInput(nreg, config.nrfaddr_rcv[3]);
				setCoilBit(nreg, 1);
				break;
			case GATE_NRF_RCVADDR4:
				config.nrfaddr_rcv[4] = param & 0x00FF;
				writeInput(nreg, config.nrfaddr_rcv[4]);
				setCoilBit(nreg, 1);
				break;
			case GATE_NRF_SNDADDR0:
				config.nrfadrr_snd[0] = param & 0x00FF;
				writeInput(nreg, config.nrfadrr_snd[0]);
				setCoilBit(nreg, 1);
				break;
			case GATE_NRF_SNDADDR1:
				config.nrfadrr_snd[1] = param & 0x00FF;
				writeInput(nreg, config.nrfadrr_snd[1]);
				setCoilBit(nreg, 1);
				break;
			case GATE_NRF_SNDADDR2:
				config.nrfadrr_snd[2] = param & 0x00FF;
				writeInput(nreg, config.nrfadrr_snd[2]);
				setCoilBit(nreg, 1);
				break;
			case GATE_NRF_SNDADDR3:
				config.nrfadrr_snd[3] = param & 0x00FF;
				writeInput(nreg, config.nrfadrr_snd[3]);
				setCoilBit(nreg, 1);
				break;
			case GATE_NRF_SNDADDR4:
				config.nrfadrr_snd[4] = param & 0x00FF;
				writeInput(nreg, config.nrfadrr_snd[4]);
				setCoilBit(nreg, 1);
				break;
			default:
				if (addr >= GATE_DEV_MAP) {
					uint8_t* pdevmap = (uint8_t*) config.devmap;
					pdevmap[addr - GATE_DEV_MAP] = param & 0x00FF;
					writeInput(nreg, pdevmap[addr - GATE_DEV_MAP]);
					setCoilBit(nreg, 1);
				}
				break;
		}
		break;
	  default:
		gate_error(GATE_CMD_ERROR, 0);
		break;
	}
}

// called on kernel panic
void port_halt(void){
	port_disable();
	palClearPad(GPIOA, GPIOA_PIN8); 	// release RS485 TX_EN pin
	palSetPad(LED_GPIO, LED_PIN); 		// LED turn on error
	while(TRUE)
	{
	}
}

/*
 * Application entry point.
 */
int main(void) {
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  // LED pin PAL mode
  palSetPadMode(LED_GPIO, LED_PIN, PAL_MODE_OUTPUT_PUSHPULL);

#if DEBUG
  // serial port for slave MCU read/write
  sdStart(&SD2, NULL);
  palSetPadMode(GPIOA, GPIOA_PIN2, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
  palSetPadMode(GPIOA, GPIOA_PIN3, PAL_MODE_INPUT);
  chBSemInit(&mcusem, FALSE);

  char line[60];
  sprintf(line, "\r\nnRF24 modbus gateway, F/W: %d", FIRMWARE);
  pcPutLine(line);
#endif

  /*
   * Configuration related
   */
#if 1
  uint8_t cfgbuf[CFGLEN+1];
  eepromInit();
  if (eepromRead(cfgbuf, CFGLEN+1) != EEPROM_OK) {
	  // eeprom configuration read fail
	  port_halt();
  }
  memcpy(&config, &cfgbuf, CFGLEN);

  if (config.magic != MAGIC || CRC8((uint8_t*) &config, CFGLEN) != cfgbuf[CFGLEN]) {
	  // eeprom configuration wrong
	  default_config();
	  if (! write_config()) {
		  port_halt();
	  }
  }
#else
  default_config();
#endif

  devcmdcnt = 0;
  for (uint8_t i = 0; i < MAXDEV; i++) {
	  if (config.devmap[i].cmd == CMD_DELAY)
		  devcmdcnt++;
  }
  if (devcmdcnt > 0) {
	  // memory allocate
	  pdevcmd = (DEVCMD_T*) chHeapAlloc(NULL, sizeof(DEVCMD_T) * devcmdcnt);
	  if (pdevcmd == NULL) port_halt();
	  memset(pdevcmd, 0, sizeof(DEVCMD_T) * devcmdcnt);
	  // fill device command array
	  uint8_t cnt = 0;
	  for (uint8_t i = 0; i < MAXDEV; i++) {
		  if (config.devmap[i].cmd == CMD_DELAY) {
			  pdevcmd[cnt].deviceID = config.devmap[i].id;
			  pdevcmd[cnt++].delay = CMDDELAY;
		  }
	  }
	  // start command delay timer
	  chVTSet(&cmdTimer, MS2ST(CMDTIMERINT), cmdTimer_handler, 0);
  }

  // event thread
  chThdCreateStatic(waEventThread, sizeof(waEventThread), NORMALPRIO+2, eventThread, NULL);

  // Setup NRF24L01 IRQ pad.
  palSetPadMode(NRF_PORT_CE_IRQ, NRF_PORT_IRQ, PAL_MODE_INPUT);
  /*
   * Activates the EXT driver 1.
   */
  extStart(&EXTD1, &extcfg);
  /*
   * Enable NRF24L01 interrupts.
   */
  extChannelEnable(&EXTD1, NRF_PORT_IRQ);

  /*
   * NRF24L01+ device initialization
   */
  NRFInit();

#if DEBUG
  uint8_t txaddr[5], rxaddr[5], rxaddrstr[11] = {'\0'}, txaddrstr[11] = {'\0'};
  NRFGetAddrs(txaddr, rxaddr);
  addr_hexstr(rxaddr, rxaddrstr);
  addr_hexstr(txaddr, txaddrstr);
  sprintf(line,"CHANNEL: %d, TX ADDR:%s, RX ADDR:%s", CHANNEL, txaddrstr, rxaddrstr);
  pcPutLine(line);
#endif

  // message receive thread from NRF24L01+
  chThdCreateStatic(waNRFReceiveThread, sizeof(waNRFReceiveThread), NORMALPRIO+1, nrfReceiveThread, NULL);

  // received message parse thread
  chThdCreateStatic(waNRFParseThread, sizeof(waNRFParseThread), NORMALPRIO, nrfParseThread, NULL);

  // message send thread over NRF24L01+
  chThdCreateStatic(waNRFSendThread, sizeof(waNRFSendThread), NORMALPRIO+1, nrfSendThread, NULL);

  // command queue parse thread
  chThdCreateStatic(waCmdThread, sizeof(waCmdThread), NORMALPRIO, cmdThread, NULL);

#if USE_BMP085
  // BMP085 driver init
  bmp085_init();

  // BMP085 polling semaphore
  chBSemObjectInit(&bmp085sem, FALSE);

  // start BMP085 polling timer
  chVTSet(&bmp085Timer, S2ST(BMP085_POLL_S), bmp085Timer_handler, 0);

  chThdCreateStatic(waBMPPollThread, sizeof(waBMPPollThread), NORMALPRIO, BMPPollThread, NULL);
#endif

  /*
   * Creates the MODBUS thread.
   */
  createModbusThd();

  /*
    * IWDG start
  */
  iwdgInit();
  iwdgStart(&IWDGD, &iwdgcfg);

  int16_t nreg = mbmap_nreg(0, GATE_STATUS, DEV_INT);
  if (nreg >= 0) {
	  writeHolding(nreg+1, FIRMWARE);
	  setCoilBit(nreg+1, 1);
  }

  uint16_t dummy=0;
  cmd_flag = GATE_CMD_NONE;

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and listen for events.
   */
  while (TRUE) {
    int16_t nreg = mbmap_nreg(0, GATE_STATUS, DEV_INT);
    if (nreg >= 0) {
		writeInput(nreg+1, dummy++);
    }
    palTogglePad(LED_GPIO, LED_PIN);

    // reset watchdog
    iwdgReset(&IWDGD);

    switch (cmd_flag) {
    case GATE_CMD_RESTART:
    	palClearPad(GPIOA, GPIOA_PIN8); 	// release RS485 TX_EN pin
    	NVIC_SystemReset();
    	break;
    case GATE_CMD_CFGWRITE:
		if (! write_config()) {
			gate_error(GATE_CFG_WRITE, 0);
		}
    	break;
    default:
    	break;
    }
    cmd_flag = GATE_CMD_NONE;

   	chThdSleepMilliseconds(1000);
  }
}
