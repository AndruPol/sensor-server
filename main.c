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
#include "util/printfs.h"

#include <string.h>

#define FIRMWARE			101		// версия прошивки
#define BMP085_POLL_S		900		// интервал таймера опроса датчика BMP085, сек
#define BMP085_TRYCNT		3		// max попыток чтения датчика BMP085

/* ----------------------- Modbus includes ----------------------------------*/
#include "modbus_slave.h"

#define LED_GPIO			GPIOC
#define LED_PIN				13

#define USE_SHELL			0		// ChibiOS shell use
#if USE_SHELL
#include "evtimer.h"
#include "chrtclib.h"
#include "shell.h"
#endif

#define AES					1		// decrypt received data
#define DEBUG				0

#if AES
#include "aes/inc/aes.h"
#include "aes/inc/aes_user_options.h"
#include "aes_secret.h"
static aes_data_t aes_data;
#endif

static VirtualTimer bmp085Timer;
static BinarySemaphore bmp085sem;

static BinarySemaphore mcusem;
static MESSAGE_TRACE_T msgtrace = {0};

#define NRF_READ_BUFFERS	6
static MESSAGE_T nrf_read_buf[NRF_READ_BUFFERS];
static MESSAGE_T *read_free[NRF_READ_BUFFERS];
static MESSAGE_T *read_fill[NRF_READ_BUFFERS];
static MAILBOX_DECL(mb_read_free, (msg_t*) read_free, NRF_READ_BUFFERS);
static MAILBOX_DECL(mb_read_fill, (msg_t*) read_fill, NRF_READ_BUFFERS);

// convert nrf24l01 5 bytes address to char[11]
static void addr_hexstr(uint8_t *addr, uint8_t *str){
    uint8_t *pin = addr;
    const char *hex = "0123456789ABCDEF";
    uint8_t *pout = str;
    for(uint8_t i=0; i < 5; i++){
        *pout++ = hex[(*pin>>4)&0xF];
        *pout++ = hex[(*pin++)&0xF];
    }
    *pout = 0;
}

void pcPutLine(const char *line) {
	chBSemWait(&mcusem);
	chprintf((BaseSequentialStream *)&SD2,"%s\r\n", line);
	chBSemSignal(&mcusem);
}

// check received message for duplicate
static bool_t msgReceived(MESSAGE_T *msg) {
	uint8_t idx = MAXSENSORS;
	// search received message in msgtrace
	for (uint8_t i=0; i < msgtrace.count; i++) {
		if (msgtrace.msgtm[i].sensorID == msg->sensorID && msgtrace.msgtm[i].sensorType == msg->sensorType &&
				msgtrace.msgtm[i].valueType == msg->valueType && msgtrace.msgtm[i].owkey0 == msg->owkey[0]) {
			idx = i;
			break;
		}
	}
	if (idx == MAXSENSORS) { // add message time to msgtrace
		if (msgtrace.count < MAXSENSORS) {
			idx = msgtrace.count;
			msgtrace.msgtm[idx].sensorID = msg->sensorID;
			msgtrace.msgtm[idx].sensorType = msg->sensorType;
			msgtrace.msgtm[idx].valueType = msg->valueType;
			msgtrace.msgtm[idx].owkey0 = msg->owkey[0];
			msgtrace.msgtm[idx].sysTime = chTimeNow();
			msgtrace.count = idx+1;
		}
		return FALSE;
	}
	if ((chTimeNow() - msgtrace.msgtm[idx].sysTime) < S2ST(MSGDUPTIME)) {
		msgtrace.msgtm[idx].sysTime = chTimeNow();
		return TRUE;
	}
	msgtrace.msgtm[idx].sysTime = chTimeNow();
	return FALSE;
}

/*
 * Triggered when the NRF24L01 triggers an interrupt
 */
static void extcbnrf(EXTDriver *extp, expchannel_t channel) {
  (void)extp;
  (void)channel;

	/*
	 * Call interrupt handler
	 */
  	chSysLockFromIsr();
	NRFReportIRQ();
	chSysUnlockFromIsr();
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

void system_reset(void) {
  chThdSleepMilliseconds(100);
  NVIC_SystemReset();
}

#if USE_SHELL
/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/
static Thread *shelltp = NULL;
#define SHELL_WA_SIZE   THD_WA_SIZE(2048)

static void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]) {
  size_t n, size;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: mem\r\n");
    return;
  }
  n = chHeapStatus(NULL, &size);
  chprintf(chp, "core free memory : %u bytes\r\n", chCoreStatus());
  chprintf(chp, "heap fragments   : %u\r\n", n);
  chprintf(chp, "heap free total  : %u bytes\r\n", size);
}

static void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[]) {
  static const char *states[] = {THD_STATE_NAMES};
  Thread *tp;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: threads\r\n");
    return;
  }
  chprintf(chp, "    addr    stack prio refs     state\r\n");
  tp = chRegFirstThread();
  do {
    chprintf(chp, "%.8lx %.8lx %4lu %4lu %9s %lu\r\n",
            (uint32_t)tp, (uint32_t)tp->p_ctx.r13,
            (uint32_t)tp->p_prio, (uint32_t)(tp->p_refs - 1),
            states[tp->p_state]);
//                   , (uint32_t)tp->p_time);
    tp = chRegNextThread(tp);
  } while (tp != NULL);
}

static void cmd_reboot(BaseSequentialStream *chp, int argc, char *argv[]){
  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: reboot\r\n");
    return;
  }
  chprintf(chp, "rebooting...\r\n");
  chThdSleepMilliseconds(100);
  NVIC_SystemReset();
}

static const ShellCommand commands[] = {
  {"mem", cmd_mem},
  {"threads", cmd_threads},
  {"reboot", cmd_reboot},
  {NULL, NULL}
};

static const ShellConfig shell_cfg = {
  (BaseSequentialStream *)&SD2,
  commands
};
#endif	//USE_SHELL

/*===========================================================================*/
/* Main and generic code.                                                    */
/*===========================================================================*/

// BMP085 polling timer callback
static void bmp085Timer_handler(void *arg){
	(void)arg;
	chSysLockFromIsr();
	chBSemSignalI(&bmp085sem);
	chVTSetI(&bmp085Timer, S2ST(BMP085_POLL_S), bmp085Timer_handler, 0);
	chSysUnlockFromIsr();
}

/*
 * BMP085 polling thread
 */
static WORKING_AREA(waBMP085Thread, 256);
__attribute__((noreturn))
static msg_t BMP085Thread(void *arg) {
  (void)arg;
  chRegSetThreadName("BMP085Thd");
  while (TRUE) {
    chBSemWait(&bmp085sem);

    int16_t temperature;
    uint32_t pressure;
    uint8_t readcnt = BMP085_TRYCNT;
    while (readcnt-- > 0) {
    	if (bmp085_read(&temperature, &pressure) == BMP085_OK) {
#if DEBUG
    		sprintf(line,"SENSOR:0:BMP085:1:TEMPERATURE:%.2f", (float)temperature/10);
    		pcPutLine(line);
#endif
    		writeInputReg(BMP085_TEMPERATURE, (float)temperature);
#if DEBUG
    		sprintf(line,"SENSOR:0:BMP085:1:PRESSURE:%.2f", (float)pressure*75/10000);
    		pcPutLine(line);
#endif
    		writeInputReg(BMP085_PRESSURE, (float)pressure*75/1000);
        	break;
    }
    else {
#if DEBUG
    		sprintf(line,"ERROR:0:BMP085");
    		pcPutLine(line);
#endif
    		setReceivedBit(BMP085_TEMPERATURE, 1);
    		setErrorBit(BMP085_TEMPERATURE, 1);
    		setReceivedBit(BMP085_PRESSURE, 1);
    		setErrorBit(BMP085_PRESSURE, 1);
    	}
    }
  }
}

#if 0
#define SEND_TIMEOUT		2000
static BinarySemaphore nrfsem;

static WORKING_AREA(waNRFSendThread,1024);
static Thread *nrfSendThread_p;
__attribute__((noreturn))
static msg_t nrfSendThread(void *arg) {

  (void)arg;
  chRegSetThreadName("nrfSend");

  while (TRUE) {
	MESSAGE_T *req;
	Thread *tp;

	/* wait for send request */
	tp = chMsgWait();
	req = (MESSAGE_T *) chMsgGet(tp);
	chMsgRelease(tp, (msg_t) req);

	chBSemWait(&nrfsem);
	NRFSendData((uint8_t *)&req);
	chBSemSignal(&nrfsem);
  }
}
#endif

static WORKING_AREA(waNRFReceiveThread, 256);
__attribute__((noreturn))
static msg_t nrfReceiveThread(void *arg) {
  (void)arg;
  chRegSetThreadName("nrfReceive");

  uint8_t pipeNr=0;

  for (uint8_t i=0; i < NRF_READ_BUFFERS; i++)
	  chMBPost(&mb_read_free, (msg_t) &nrf_read_buf[i], TIME_IMMEDIATE);

  while (TRUE) {
	  chBSemWait(&nrf.NRFSemRX);
	  uint8_t inbuf[sizeof(MESSAGE_T)+1] = {0};
	  NRFReceiveData(&pipeNr, inbuf);

#if 0	// TODO: send answer to client
	  chMsgSend(nrfSendThread_p, (msg_t) msg);

	  /* wait for reply */
	  if(chBSemWaitTimeout(&nrfsem, MS2ST(SEND_TIMEOUT)) == RDY_TIMEOUT) {
		  chBSemReset(&nrfsem, FALSE);
	  }
#endif

	  void *pbuf;
	  if (chMBFetch(&mb_read_free, (msg_t *) &pbuf, TIME_IMMEDIATE) == RDY_OK) {
		  memcpy(pbuf, inbuf, sizeof(MESSAGE_T));
		  chMBPost(&mb_read_fill, (msg_t) pbuf, TIME_IMMEDIATE);
	  }

  }
}

static WORKING_AREA(waNRFParseThread, 256);
__attribute__((noreturn))
static msg_t nrfParseThread(void *arg) {
  (void)arg;
  chRegSetThreadName("nrfParse");

#if AES
  uint8_t mbuf[16];
#endif

#if DEBUG
  char line[60], stype[7];
  char owkey[17];
#endif

  while (TRUE) {
	  void *pbuf;
	  if (chMBFetch(&mb_read_fill, (msg_t *) &pbuf, TIME_INFINITE) == RDY_OK) {
	     chMBPost(&mb_read_free, (msg_t) pbuf, TIME_IMMEDIATE);
	  } else {
		  continue;
	  }

	  MESSAGE_T *msg = (MESSAGE_T *)pbuf;

#if AES
	  aes_decrypt_ecb(&aes_data, pbuf, mbuf);
	  msg = (MESSAGE_T *)mbuf;
#endif

#if 0
	  if (msgReceived(msg)) continue;
#endif

	  switch (msg->msgType) {
	  case SENSOR_DATA:	//SENSOR
		  switch (msg->sensorType) {
		  case DS1820:
#if DEBUG
			  owkey_hexstr(msg->owkey, owkey);
			  sprintf(line,"SENSOR:%d:DS1820:%s:TEMPERATURE:%.2f", msg->sensorID, owkey, (float)msg->data.fValue);
			  pcPutLine(line);
#endif
			  writeInputReg((1 + msg->sensorID) * NRF_SENSOR_REGS + DS18B20_TEMPERATURE, (float)msg->data.fValue * 10);
			  break;
		  case BH1750:
#if DEBUG
			  sprintf(line,"SENSOR:%d:BH1750:1:LIGHT:%d", msg->sensorID, (int)msg->data.iValue);
			  pcPutLine(line);
#endif
			  writeInputReg((1 + msg->sensorID) * NRF_SENSOR_REGS + BH1750_LIGHT, (float)msg->data.iValue * 10);
			  break;
		  case DHT:
			  if (msg->valueType == TEMPERATURE){
#if DEBUG
				  sprintf(line,"SENSOR:%d:DHT:%d:TEMPERATURE:%.2f", msg->sensorID, msg->owkey[0], (float)msg->data.iValue/10);
				  pcPutLine(line);
#endif
				  writeInputReg((1 + msg->sensorID) * NRF_SENSOR_REGS + DHT_TEMPERATURE, (float)msg->data.iValue);
			  } else if (msg->valueType == HUMIDITY){
#if DEBUG
				  sprintf(line,"SENSOR:%d:DHT:%d:HUMIDITY:%.2f", msg->sensorID, msg->owkey[0], (float)msg->data.iValue/10);
				  pcPutLine(line);
#endif
				  writeInputReg((1 + msg->sensorID) * NRF_SENSOR_REGS + DHT_HUMIDITY, (float)msg->data.iValue);
			  }
			  break;
		  case BMP085:
			  if (msg->valueType == TEMPERATURE){
#if DEBUG
				  sprintf(line,"SENSOR:%d:BMP085:1:TEMPERATURE:%.2f", msg->sensorID, (float)msg->data.fValue/10);
				  pcPutLine(line);
#endif
			  } else if (msg->valueType == PRESSURE){
#if DEBUG
				  sprintf(line,"SENSOR:%d:BMP085:1:PRESSURE:%.2f", msg->sensorID, (float)msg->data.fValue*75/10000);
				  pcPutLine(line);
#endif
			  }
			  break;
		  case ADC:
			  if (msg->valueType == LIGHT){
#if DEBUG
				  sprintf(line,"SENSOR:%d:ADC:1:LIGHT:%d", msg->sensorID, (int)msg->data.iValue);
				  pcPutLine(line);
#endif
				  writeInputReg((1 + msg->sensorID) * NRF_SENSOR_REGS + BH1750_LIGHT, (float)msg->data.iValue * 10);
			  } else if (msg->valueType == VOLTAGE){
#if DEBUG
				  sprintf(line,"SENSOR:%d:ADC:1:VOLTAGE:%d", msg->sensorID, (int)msg->data.iValue);
				  pcPutLine(line);
#endif
			  }
			  break;
		  default:	//UNKNOWN
#if DEBUG
			  sprintf(line,"SENSOR:%d:ERROR:unknown sensor type", msg->sensorID);
			  pcPutLine(line);
#endif
			  break;
		  }
		  break;
	  case SENSOR_ERROR:
		  switch (msg->sensorType) {
		  case DS1820:
			  setReceivedBit((1 + msg->sensorID) * NRF_SENSOR_REGS + DS18B20_TEMPERATURE, 1);
			  setErrorBit((1 + msg->sensorID) * NRF_SENSOR_REGS + DS18B20_TEMPERATURE, 1);
			  break;
		  case BH1750:
			  setReceivedBit((1 + msg->sensorID) * NRF_SENSOR_REGS + BH1750_LIGHT, 1);
			  setErrorBit((1 + msg->sensorID) * NRF_SENSOR_REGS + BH1750_LIGHT, 1);
			  break;
		  case DHT:
			  setReceivedBit((1 + msg->sensorID) * NRF_SENSOR_REGS + DHT_TEMPERATURE, 1);
			  setErrorBit((1 + msg->sensorID) * NRF_SENSOR_REGS + DHT_TEMPERATURE, 1);
			  setReceivedBit((1 + msg->sensorID) * NRF_SENSOR_REGS + DHT_HUMIDITY, 1);
			  setErrorBit((1 + msg->sensorID) * NRF_SENSOR_REGS + DHT_HUMIDITY, 1);
			  break;
		  case ADC:
			  setReceivedBit((1 + msg->sensorID) * NRF_SENSOR_REGS + BH1750_LIGHT, 1);
			  setErrorBit((1 + msg->sensorID) * NRF_SENSOR_REGS + BH1750_LIGHT, 1);
			  break;
		  default:
			  break;
		  }
#if DEBUG
		  memset(&stype,0,7);
		  if (get_sensor_type(msg->sensorType, stype)) {
			  sprintf(line,"ERROR:%d:%s:%d:%d", msg->sensorID, stype, msg->owkey[0], msg->data.cValue[0]);
			  pcPutLine(line);
		  }
#endif
		  break;
	  default:
#if DEBUG
		  sprintf(line,"ERROR:%d:unknown message type", msg->sensorID);
		  pcPutLine(line);
#endif
		  break;
	  }
  } // while
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

#if DEBUG
  // serial port for slave MCU read/write
  sdStart(&SD2, NULL);
  palSetPadMode(GPIOA, GPIOA_PIN2, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
  palSetPadMode(GPIOA, GPIOA_PIN3, PAL_MODE_INPUT);
  chBSemInit(&mcusem, FALSE);

  char line[60];
  sprintf(line, "\r\nnRF24 wireless gateway, F/W: %d", FIRMWARE);
  pcPutLine(line);
#endif

#if USE_SHELL
  /*
   * Shell manager initialization.
   */
  shellInit();
#endif

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

  // send thread over NRF24L01+
  //nrfSendThread_p = chThdCreateStatic(waNRFSendThread, sizeof(waNRFSendThread), NORMALPRIO, nrfSendThread, NULL);

  chMBInit(&mb_read_free, (msg_t*) read_free, NRF_READ_BUFFERS);
  chMBInit(&mb_read_fill, (msg_t*) read_fill, NRF_READ_BUFFERS);

  // message receive thread from NRF24L01+
  chThdCreateStatic(waNRFReceiveThread, sizeof(waNRFReceiveThread), NORMALPRIO+1, nrfReceiveThread, NULL);

  // message parse thread from NRF24L01+
  chThdCreateStatic(waNRFParseThread, sizeof(waNRFParseThread), NORMALPRIO, nrfParseThread, NULL);

#if AES
  aes_initialize(&aes_data, AES_KEY_LENGTH_128_BITS, aes_key, NULL);
#endif

  // BMP085 driver init
  bmp085_init();

  // BMP085 polling semaphore
  chBSemInit(&bmp085sem, FALSE);

  // start BMP085 polling timer
  chVTSet(&bmp085Timer, S2ST(BMP085_POLL_S), bmp085Timer_handler, 0);

  chThdCreateStatic(waBMP085Thread, sizeof(waBMP085Thread), NORMALPRIO, BMP085Thread, NULL);

  /*
   * Creates the MODBUS thread.
   */
  createModbusThd();

#if DEBUG
  uint8_t txaddr[5], rxaddr[5], rxaddrstr[11] = {'\0'}, txaddrstr[11] = {'\0'};
  NRFGetAddrs(txaddr, rxaddr);
  addr_hexstr(rxaddr, rxaddrstr);
  addr_hexstr(txaddr, txaddrstr);
  sprintf(line,"CHANNEL: %d, TX ADDR:%s, RX ADDR:%s", CHANNEL, txaddrstr, rxaddrstr);
  pcPutLine(line);
#endif

  // LED pin PAL mode
  palSetPadMode(LED_GPIO, LED_PIN, PAL_MODE_OUTPUT_PUSHPULL);

  writeInputReg(FIRMWARE_VERSION, FIRMWARE);
  uint16_t dummy=0;

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and listen for events.
   */
  while (TRUE) {
#if USE_SHELL
    if (!shelltp)
    	shelltp = shellCreate(&shell_cfg, SHELL_WA_SIZE, NORMALPRIO);
    else {
    	if (chThdTerminated(shelltp)) {
    		chThdRelease(shelltp);    /* Recovers memory of the previous shell.   */
    		shelltp = NULL;           /* Triggers spawning of a new shell.        */
    	}
    }
#endif

    writeInputReg(DUMMY_COUNTER, dummy++);
    palTogglePad(LED_GPIO, LED_PIN);

   	chThdSleepMilliseconds(1000);
  }
}
