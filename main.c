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
#include "evtimer.h"
#include "chrtclib.h"

#include "main.h"
#include "usb_serial.h"
#include "nrf24l01.h"
#include "bmp085.h"
#include "util/printfs.h"
#include "shell.h"

#define AES				1	// decrypt received data

#if AES
#include "aes/inc/aes.h"
#include "aes/inc/aes_user_options.h"
#include "aes_secret.h"
static aes_data_t aes_data;
#endif

#define ARRAY_LEN(a) (sizeof(a)/sizeof(a[0]))
#define USE_SHELL		0		// ChibiOS shell use

static VirtualTimer delayTimer;
static BinarySemaphore mainsem, putsem;
static MESSAGE_TRACE_T msgtrace = {0};

// convert nrf24l01 5 bytes address to char[11]
void addr_hexstr(uint8_t *addr, uint8_t *str){
    uint8_t *pin = addr;
    const char *hex = "0123456789ABCDEF";
    uint8_t *pout = str;
    for(uint8_t i=0; i < 5; i++){
        *pout++ = hex[(*pin>>4)&0xF];
        *pout++ = hex[(*pin++)&0xF];
    }
    *pout = 0;
}

// convert 1-wire 8 bytes address to char[17]
void owkey_hexstr(uint8_t *addr, uint8_t *str){
    uint8_t *pin = addr;
    const char *hex = "0123456789ABCDEF";
    uint8_t *pout = str;
    for(uint8_t i=0; i < 8; i++){
        *pout++ = hex[(*pin>>4)&0xF];
        *pout++ = hex[(*pin++)&0xF];
    }
    *pout = 0;
}

// put line over serial port
void serialPutLine(const char *line) {
	if (SDU1.config->usbp->state == USB_ACTIVE) {
		chBSemWait(&putsem);
		chprintf((BaseSequentialStream *)&SDU1,"%s\r\n", line);
		chBSemSignal(&putsem);
	}
}

// check received message for duplicate
bool_t msgReceived(MESSAGE_T *msg) {
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

// delayTimer callback
void delayTimer_handler(void *arg){
	(void)arg;
	chSysLockFromIsr();
	chBSemSignalI(&mainsem);
	chVTSetI(&delayTimer, MS2ST(BMP085_DELAY_MS), delayTimer_handler, 0);
	chSysUnlockFromIsr();
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
  (BaseSequentialStream *)&SDU1,
  commands
};
#endif	//USE_SHELL

/*===========================================================================*/
/* Main and generic code.                                                    */
/*===========================================================================*/

/*
 * Red LEDs blinker thread, times are in milliseconds.
 */
static WORKING_AREA(waBlinkerThread, 128);
__attribute__((noreturn))
static msg_t blinkerThread(void *arg) {

  (void)arg;
  chRegSetThreadName("blinkerThd");
  while (TRUE) {
    palTogglePad(GPIOC, GPIOC_LED);
    chThdSleepMilliseconds(500);
  }
}

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

bool_t get_sensor_type(const sensortype_t type, char *line) {
	  switch (type) {
	  case DS1820:
		  memcpy(line,"DS1820",6);
		  return TRUE;
	  case BH1750:
		  memcpy(line,"BH1750",6);
		  return TRUE;
	  case DHT:
		  memcpy(line,"DHT",3);
		  return TRUE;
	  case BMP085:
		  memcpy(line,"BMP085",6);
		  return TRUE;
	  case ADC:
		  memcpy(line,"ADC",3);
		  return TRUE;
	  }
	  return FALSE;
}

static WORKING_AREA(waNRFReceiveThread,1024);
__attribute__((noreturn))
static msg_t nrfReceiveThread(void *arg) {

  (void)arg;
  chRegSetThreadName("nrfReceive");

  static uint8_t pipeNr=0;
  static uint8_t inBuf[17]={0};
#if AES
  static uint8_t mBuf[16];
#endif
  static uint8_t owkey[17];
  static char line[60], stype[7];


  while (TRUE) {
	  chBSemWait(&nrfsem);
	  NRFReceiveData(&pipeNr, inBuf);
	  chBSemSignal(&nrfsem);
#if AES
	  aes_decrypt_ecb(&aes_data, inBuf, mBuf);
	  MESSAGE_T *msg = (MESSAGE_T *)mBuf;
#else
	  MESSAGE_T *msg = (MESSAGE_T *)inBuf;
#endif

#if 0	// TODO: send answer to client
	  chMsgSend(nrfSendThread_p, (msg_t) msg);

	  /* wait for reply */
	  if(chBSemWaitTimeout(&nrfsem, MS2ST(SEND_TIMEOUT)) == RDY_TIMEOUT) {
		  chBSemReset(&nrfsem, FALSE);
	  }
#endif
	  if (msgReceived(msg)) continue;
	  switch (msg->msgType) {
	  case SENSOR_DATA:	//SENSOR
		  switch (msg->sensorType) {
		  case DS1820:
			  owkey_hexstr(msg->owkey, owkey);
			  sprintf(line,"SENSOR:%d:DS1820:%s:TEMPERATURE:%.2f", msg->sensorID, owkey, (float)msg->data.fValue);
			  serialPutLine(line);
			  break;
		  case BH1750:
			  sprintf(line,"SENSOR:%d:BH1750:1:LIGHT:%d", msg->sensorID, (int)msg->data.iValue);
			  serialPutLine(line);
			  break;
		  case DHT:
			  if (msg->valueType == TEMPERATURE){
				  sprintf(line,"SENSOR:%d:DHT:%d:TEMPERATURE:%.2f", msg->sensorID, msg->owkey[0], (float)msg->data.iValue/10);
				  serialPutLine(line);
			  }
			  if (msg->valueType == HUMIDITY){
				  sprintf(line,"SENSOR:%d:DHT:%d:HUMIDITY:%.2f", msg->sensorID, msg->owkey[0], (float)msg->data.iValue/10);
				  serialPutLine(line);
			  }
			  break;
		  case BMP085:
			  if (msg->valueType == TEMPERATURE){
				  sprintf(line,"SENSOR:%d:BMP085:1:TEMPERATURE:%.2f", msg->sensorID, (float)msg->data.fValue/10);
				  serialPutLine(line);
			  }
			  if (msg->valueType == PRESSURE){
				  sprintf(line,"SENSOR:%d:BMP085:1:PRESSURE:%.2f", msg->sensorID, (float)msg->data.fValue*75/10000);
				  serialPutLine(line);
			  }
			  break;
		  case ADC:
			  if (msg->valueType == LIGHT){
				  sprintf(line,"SENSOR:%d:ADC:1:LIGHT:%d", msg->sensorID, (int)msg->data.iValue);
				  serialPutLine(line);
			  }
			  if (msg->valueType == VOLTAGE){
				  sprintf(line,"SENSOR:%d:ADC:1:VOLTAGE:%d", msg->sensorID, (int)msg->data.iValue);
				  serialPutLine(line);
			  }
			  break;
		  default:	//UNKNOWN
			  sprintf(line,"SENSOR:%d:ERROR:unknown sensor type", msg->sensorID);
			  serialPutLine(line);
			  break;
		  }
		  break;
	  case SENSOR_ERROR:
		  memset(&stype,0,7);
		  if (get_sensor_type(msg->sensorType, stype)) {
			  sprintf(line,"ERROR:%d:%s:%d:%d", msg->sensorID, stype, msg->owkey[0], msg->data.cValue[0]);
			  serialPutLine(line);
		  }
		  break;
	  default:
		  sprintf(line,"ERROR:%d:unknown message type", msg->sensorID);
		  serialPutLine(line);
	  }
  } // while
}

// called on kernel panic
void port_halt(void){
	port_disable();
	palSetPad(GPIOC, GPIOC_LED); // turn on error
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

  /*
   * Initializes a serial-over-USB CDC driver.
   */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1500);
  usbConnectBus(serusbcfg.usbp);
  usbStart(serusbcfg.usbp, &usbcfg);

  chBSemInit(&putsem, FALSE);
  static char line[60];
  sprintf(line, "\r\nMaster sensor module, F/W:%s", FIRMWARE);
  serialPutLine(line);

#if USE_SHELL
  /*
   * Shell manager initialization.
   */
  shellInit();
#endif

  /*
   * Setup NRF24L01 IRQ pad.
   */
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

  static uint8_t txaddr[5], rxaddr[5], rxaddrstr[11] = {'\0'}, txaddrstr[11] = {'\0'};
  NRFGetAddrs(txaddr, rxaddr);
  addr_hexstr(rxaddr, rxaddrstr);
  addr_hexstr(txaddr, txaddrstr);
  sprintf(line,"CHANNEL: %d, TX ADDR:%s, RX ADDR:%s", CHANNEL, txaddrstr, rxaddrstr);
  serialPutLine(line);

#if AES
  aes_initialize(&aes_data, AES_KEY_LENGTH_128_BITS, aes_key, NULL);
#endif

  chBSemInit(&nrfsem, FALSE);

  // send thread over NRF24L01+
  nrfSendThread_p = chThdCreateStatic(waNRFSendThread, sizeof(waNRFSendThread), NORMALPRIO, nrfSendThread, NULL);
  // receive thread from NRF24L01+
  chThdCreateStatic(waNRFReceiveThread, sizeof(waNRFReceiveThread), NORMALPRIO, nrfReceiveThread, NULL);

  // LED pin PAL mode
  palSetPadMode(GPIOC, GPIOC_LED, PAL_MODE_OUTPUT_PUSHPULL);

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waBlinkerThread, sizeof(waBlinkerThread), NORMALPRIO, blinkerThread, NULL);

  // BMP085 driver init
  bmp085_init();

  // start BMP085 delay timer
  chVTSet(&delayTimer, MS2ST(BMP085_DELAY_MS/15), delayTimer_handler, 0);

  // BMP085 mode switch
  chBSemInit(&mainsem, TRUE);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and listen for events.
   */
  while (TRUE) {
#if USE_SHELL
    if (!shelltp && (SDU1.config->usbp->state == USB_ACTIVE))
    	shelltp = shellCreate(&shell_cfg, SHELL_WA_SIZE, NORMALPRIO);
    else {
    	if (chThdTerminated(shelltp)) {
    		chThdRelease(shelltp);    /* Recovers memory of the previous shell.   */
    		shelltp = NULL;           /* Triggers spawning of a new shell.        */
    	}
    }
#endif
    chBSemWait(&mainsem);

    // BMP085
    int16_t temperature;
    uint32_t pressure;
    static uint8_t readcnt = 3;
    static bool_t read_ok = FALSE;
    while (readcnt-- > 0){
    	read_ok = bmp085_read(&temperature, &pressure) == BMP085_NO_ERROR;
    	if (read_ok) break;
    }
    if (read_ok) {
    	sprintf(line,"SENSOR:0:BMP085:1:TEMPERATURE:%.2f", (float)temperature/10);
    	serialPutLine(line);
    	sprintf(line,"SENSOR:0:BMP085:1:PRESSURE:%.2f", (float)pressure*75/10000);
    	serialPutLine(line);
    }
    else {
    	sprintf(line,"ERROR:0:BMP085");
    	serialPutLine(line);
    }
  }
}
