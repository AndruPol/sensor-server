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

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "evtimer.h"
#include "chrtclib.h"

#include "main.h"
#include "usb_serial.h"
#include "nrf24l01.h"
#include "bmp085.h"

#include "shell.h"

#define ARRAY_LEN(a) (sizeof(a)/sizeof(a[0]))
//#define USE_SHELL		// ChibiOS shell use

static VirtualTimer delayTimer;
static EVENTSOURCE_DECL(delay_evsrc);
static EVENTSOURCE_DECL(main_evsrc);
static BinarySemaphore mainsem;

// convert nrf24l01 5 bytes address to char[11]
void addr_hexstr(uint8_t *addr, uint8_t *str){
    uint8_t *pin = addr;
    static const uint8_t *hex = "0123456789ABCDEF";
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
    static const uint8_t *hex = "0123456789ABCDEF";
    uint8_t *pout = str;
    for(uint8_t i=0; i < 8; i++){
        *pout++ = hex[(*pin>>4)&0xF];
        *pout++ = hex[(*pin++)&0xF];
    }
    *pout = 0;
}

// delayTimer callback
void delayTimer_handler(void *arg){
	(void)arg;
	chSysLockFromIsr();
	chBSemSignalI(&mainsem);
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

#ifdef USE_SHELL
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
//  {"sleep", cmd_sleep},
//  {"date", cmd_date},
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

static BinarySemaphore nrfsem;
static WORKING_AREA(waNRFReceiveThread, 1024);
__attribute__((noreturn))
static msg_t nrfReceiveThread(void *arg) {

  (void)arg;
  chRegSetThreadName("nrfReceive");

  chBSemInit(&nrfsem, FALSE);
  static uint8_t pipeNr=0;
  static uint8_t inBuf[17]={0};
  static uint8_t owkey[17];

  while (TRUE) {
	  chBSemWait(&nrfsem);
	  NRFReceiveData(&pipeNr, inBuf);
	  MESSAGE_T *msg = inBuf;
	  switch (msg->msgType) {
	  case 0:
		  switch (msg->errorCode) {
		  case 0:
			  chprintf((BaseSequentialStream *)&SDU1,"INFO:%d:VALUE:%d\r\n",msg->errorCode,msg->data.iValue);
			  break;
		  case 1:
			  chprintf((BaseSequentialStream *)&SDU1,"INFO:%d:VALUE:%d\r\n",msg->errorCode,msg->data.iValue);
			  break;
	  	  default:
			  chprintf((BaseSequentialStream *)&SDU1,"INFO:%d:unknown errorCode\r\n");
	  		  break;
	  	  }
		  break;
	  case 1:
		  switch (msg->sensorType) {
		  case 0:
			  owkey_hexstr(msg->owkey, owkey);
			  chprintf((BaseSequentialStream *)&SDU1,"SENSOR:DS1820:%s:TEMPERATURE:%.2f\r\n", owkey, msg->data.fValue);
			  break;
		  case 1:
			  chprintf((BaseSequentialStream *)&SDU1,"SENSOR:BH1750:1:LIGHT:%d\r\n", msg->data.iValue);
			  break;
		  case 2:
			  if (msg->valueType == 0){
				  chprintf((BaseSequentialStream *)&SDU1,"SENSOR:DHT:%d:TEMPERATURE:%.2f\r\n", msg->owkey[0], (float) msg->data.iValue/10);
			  }
			  if (msg->valueType == 1){
				  chprintf((BaseSequentialStream *)&SDU1,"SENSOR:DHT:%d:HUMIDITY:%.2f\r\n", msg->owkey[0], (float) msg->data.iValue/10);
			  }
			  break;
		  default:
			  chprintf((BaseSequentialStream *)&SDU1,"SENSOR:ERROR:unknown sensor type\r\n");
			  break;
		  }
		  break;
	  case 2:
		  switch (msg->sensorType) {
		  case 0:
			  chprintf((BaseSequentialStream *)&SDU1,"ERROR:DS1820\r\n");
			  break;
		  case 1:
			  chprintf((BaseSequentialStream *)&SDU1,"ERROR:BH1750\r\n");
			  break;
		  case 2:
			  chprintf((BaseSequentialStream *)&SDU1,"ERROR:DHT:%d\r\n",msg->owkey[0]);
			  break;
		  case 3:
			  chprintf((BaseSequentialStream *)&SDU1,"ERROR:BMP085\r\n");
			  break;
		  }
		  break;
	  default:
		  chprintf((BaseSequentialStream *)&SDU1,"ERROR:unknown message type\r\n");
	  }
	  chVTReset(&delayTimer);
	  chVTSet(&delayTimer, MS2ST(BMP085_DELAY_MS), delayTimer_handler, 0);
	  chBSemSignal(&nrfsem);
  }
}

static BinarySemaphore evtsem;

/*
 * event manager thread
*/
WORKING_AREA(waEventThread, 1024);
__attribute__((noreturn))
static msg_t EventThread(void *arg) {
	(void)arg;
	enum{
	  	DELAY_ESID		// id delay_evsrc
  	  	};
	static struct EventListener delay_el;
	eventmask_t active;

	chRegSetThreadName("eventThd");

	chEvtInit(&delay_evsrc);
	chEvtRegister(&delay_evsrc, &delay_el, DELAY_ESID);

	while (TRUE) {
	    chBSemWait(&evtsem);
		active = chEvtWaitAny(
								EVENT_MASK(DELAY_ESID)
								);
        // delay event listener
        if ((active & EVENT_MASK(DELAY_ESID)) != 0){
   			chBSemSignal(&mainsem);
        }

        chSysLock();
    	chEvtGetAndClearFlagsI(&delay_el);
    	chSysUnlock();

        chBSemSignal(&evtsem);
	} //while
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

  chprintf((BaseSequentialStream *)&SDU1,"Master sensor module, F/W:%s\r\n", FIRMWARE);

#ifdef USE_SHELL
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
  chprintf((BaseSequentialStream *)&SDU1,"CHANNEL: %d, TX ADDR:%s, RX ADDR:%s\r\n", CHANNEL, txaddrstr, rxaddrstr);

  // receiving data from NRF24L01+
  chThdCreateStatic(waNRFReceiveThread, sizeof(waNRFReceiveThread), NORMALPRIO, nrfReceiveThread, NULL);

  // LED pin PAL mode
  palSetPadMode(GPIOC, GPIOC_LED, PAL_MODE_OUTPUT_PUSHPULL);
  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waBlinkerThread, sizeof(waBlinkerThread), NORMALPRIO, blinkerThread, NULL);

  // event manager thread
  chBSemInit(&evtsem, TRUE);

  // Creates event manager thread.
  chThdCreateStatic(waEventThread, sizeof(waEventThread), NORMALPRIO+1, EventThread, NULL);

  // BMP085 driver init
  bmp085_init();

  // start event manager thread
  chBSemSignal(&evtsem);

  // start 1s delay timer
//  chVTSet(&delayTimer, MS2ST(DELAYPERIOD), delayTimer_handler, 0);

  // sleep mode switch
  chBSemInit(&mainsem, TRUE);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and listen for events.
   */
  while (TRUE) {
#ifdef USE_SHELL
    if (!shelltp && (SDU1.config->usbp->state == USB_ACTIVE))
      shelltp = shellCreate(&shell_cfg, SHELL_WA_SIZE, NORMALPRIO);
    else if (chThdTerminated(shelltp)) {
      chThdRelease(shelltp);    /* Recovers memory of the previous shell.   */
      shelltp = NULL;           /* Triggers spawning of a new shell.        */
    }
#endif
    chBSemWait(&mainsem);

    // BMP085
    int16_t temperature;
    uint32_t pressure;
    bmp085_error_t	bmp085_error;
    static uint8_t readcnt = 5;
    static bool_t read_ok = FALSE;
    while (readcnt-- > 0){
    	read_ok = bmp085_read(&temperature, &pressure, &bmp085_error) == 0;
    	if (read_ok) break;
    }
	chprintf((BaseSequentialStream *)&SDU1,"SENSOR:BMP085:1:TEMPERATURE:%.2f\r\n", (float)temperature/10);
	chprintf((BaseSequentialStream *)&SDU1,"SENSOR:BMP085:1:PRESSURE:%.2f\r\n", (float)pressure*75/10000);
  }
}
