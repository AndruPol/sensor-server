/*
 * bmp085.c
 */

#include "ch.h"
#include "hal.h"

#include "bmp085.h"
#include "bmp085_table.h"

#define BARO_PRIO			(NORMALPRIO + 1)
#define I2CD_BMP085			I2CD2

#define BMP085_ADDR 		0x77 // device address
#define BMP085_CTL			0xF4 // control register address
#define BMP085_TEMP			0x2E // read temperature command
#define BMP085_PRES  		0x34 // read pressure command
#define BMP085_ADC_MSB		0xF6 // MSB
#define BMP085_ADC_LSB		0xF7 // LSB
#define BMP085_ADC_XLSB		0xF8 // XLSB
#define BMP085_OSS 			3 	 // sensor precision (see datasheet), 3 -- max
#define BMP085_TEMP_DELAY	5	 // wait temperature results (datasheet says 4.5 ms)
#define BMP085_PRES_DELAY   26   // wait pressure results (OSS=3 datasheet says 25.5 ms)
#define BMP085_TIMEOUT_MS	10	 // i2c transmit timeout

typedef struct bmp085_t {
	struct bmp085_cc_t // calibration coefficients
	{
		int16_t ac1;
		int16_t ac2;
		int16_t ac3;
		uint16_t ac4;
		uint16_t ac5;
		uint16_t ac6;
		int16_t b1;
		int16_t b2;
		int16_t mb;
		int16_t mc;
		int16_t md;
	} cc;
	// temperature
	int16_t ut;
	// pressure
	int32_t up;
} BMP085_T;

static BMP085_T bmp085;
static binary_semaphore_t bmp085sem;
static bool bmp085_calibrated = FALSE;
static bmp085_error_t bmp085_error;

// compensated temperature and pressure values
//uint32_t pval = 0;  // temp in 0.1C
//int32_t  tval = 0;  // press in Pa
int32_t baro_altitude = 0;

#define N_AWG      		32 			// Averaging values count
#define FIX_FORMAT 		5  			// 32 == 2^5 to replace division by shift
static uint32_t pres_awg = 10UL << FIX_FORMAT; // aweraged value

static const I2CConfig i2cfg = { OPMODE_I2C, 100000, STD_DUTY_CYCLE, };

/* calculation height from pressure using precalculated table and linear interpolation */
static int16_t pres_to_height(uint32_t pres) {
	uint16_t i = 0;
	int32_t dp = 0;

	if (pres > BMP085_MAX_PRES)
		return ptable[0];
	if (pres < (BMP085_MIN_PRES - BMP085_STEP))
		return ptable[(sizeof(ptable) / sizeof(int16_t)) - 1];

	i = (BMP085_MAX_PRES - pres) / 256;
	dp = (BMP085_MAX_PRES - pres) % 256;

	return (ptable[i] + ((ptable[i + 1] - ptable[i]) * dp) / 256);
}

/* calculation compensated pressure value */
static void bmp085_calculate(int16_t *tval, uint32_t *pval) {
	int32_t x1, x2, x3, b3, b5, b6, p;
	uint32_t b4, b7;

	x1 = (bmp085.ut - bmp085.cc.ac6) * bmp085.cc.ac5 >> 15;
	x2 = ((int32_t) bmp085.cc.mc << 11) / (x1 + bmp085.cc.md);
	b5 = x1 + x2;
	*tval = (b5 + 8) >> 4;

	b6 = b5 - 4000;
	x1 = (bmp085.cc.b2 * (b6 * b6 >> 12)) >> 11;
	x2 = bmp085.cc.ac2 * b6 >> 11;
	x3 = x1 + x2;
	b3 = ((((int32_t) bmp085.cc.ac1 * 4 + x3) << BMP085_OSS) + 2) >> 2;

	x1 = bmp085.cc.ac3 * b6 >> 13;
	x2 = (bmp085.cc.b1 * (b6 * b6 >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (bmp085.cc.ac4 * (uint32_t)(x3 + 32768)) >> 15;
	b7 = ((uint32_t) bmp085.up - b3) * (50000 >> BMP085_OSS);
	if (b7 < 0x80000000)
		p = (b7 * 2) / b4;
	else
		p = (b7 / b4) * 2;

	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	*pval = p + ((x1 + x2 + 3791) >> 4);
	// end of black magic

	// refresh aweraged pressure value
	pres_awg = pres_awg - (pres_awg >> FIX_FORMAT) + *pval;
	// calculate height
	baro_altitude = pres_to_height(pres_awg >> FIX_FORMAT);
}

// read calibrate data
static int16_t read_calibrate(uint8_t addr, bmp085_error_t *error) {
	uint8_t txbuf[2] = { 0 }, rxbuf[2] = { 0 };
	*error = BMP085_OK;
	txbuf[0] = addr;

#if I2C_USE_MUTUAL_EXCLUSION
	i2cAcquireBus(&I2CD_BMP085);
#endif

	if (i2cMasterTransmitTimeout(&I2CD_BMP085, BMP085_ADDR, txbuf, 1, rxbuf, 2,
			MS2ST(BMP085_TIMEOUT_MS)) == MSG_TIMEOUT) {
		*error = BMP085_TIMEOUT;
	}

#if I2C_USE_MUTUAL_EXCLUSION
	i2cReleaseBus(&I2CD_BMP085);
#endif

	return (rxbuf[0] << 8) + rxbuf[1];
}

/* Polling thread */
thread_t *baroThread_p;
static THD_WORKING_AREA(waBaroThread, 256);
static THD_FUNCTION( BaroThread, arg) {
	chRegSetThreadName("baroThd");
	(void) arg;

	chBSemObjectInit(&bmp085sem, FALSE);
	while (TRUE) {
		uint8_t txbuf[2], rxbuf[2];
		msg_t ret;

		/* wait for read request */
		bmp085_read_t *req;
		thread_t *tp;
		tp = chMsgWait();
		req = (bmp085_read_t *) chMsgGet(tp);
		chMsgRelease(tp, (msg_t) req);

		bmp085_error = BMP085_OK;
		if (!bmp085_calibrated) {
			bmp085_error = BMP085_CALIBRATE;
			goto error;
		}

		i2cStart(&I2CD_BMP085, &i2cfg);

		txbuf[0] = BMP085_CTL;
		txbuf[1] = BMP085_TEMP;

#if I2C_USE_MUTUAL_EXCLUSION
		i2cAcquireBus(&I2CD_BMP085);
#endif

		ret = i2cMasterTransmitTimeout(&I2CD_BMP085, BMP085_ADDR, txbuf, 2,
				NULL, 0, MS2ST(BMP085_TIMEOUT_MS));

#if I2C_USE_MUTUAL_EXCLUSION
		i2cReleaseBus(&I2CD_BMP085);
#endif

		if (ret == MSG_TIMEOUT) {
			bmp085_error = BMP085_TIMEOUT;
			goto error;
		}

		/* wait temperature results (datasheet says 4.5 ms) */
		chThdSleepMilliseconds(BMP085_TEMP_DELAY);

		/* read measured value */
		txbuf[0] = BMP085_ADC_MSB;

#if I2C_USE_MUTUAL_EXCLUSION
		i2cAcquireBus(&I2CD_BMP085);
#endif

		ret = i2cMasterTransmitTimeout(&I2CD_BMP085, BMP085_ADDR, txbuf, 1,
				rxbuf, 2, MS2ST(BMP085_TIMEOUT_MS));

#if I2C_USE_MUTUAL_EXCLUSION
		i2cReleaseBus(&I2CD_BMP085);
#endif

		if (ret == MSG_TIMEOUT) {
			bmp085_error = BMP085_TIMEOUT;
			goto error;
		}

		bmp085.ut = (int16_t)(rxbuf[0] << 8) + rxbuf[1];

		// command to measure pressure
		txbuf[0] = BMP085_CTL;
		txbuf[1] = (BMP085_PRES + (BMP085_OSS << 6));

#if I2C_USE_MUTUAL_EXCLUSION
		i2cAcquireBus(&I2CD_BMP085);
#endif

		ret = i2cMasterTransmitTimeout(&I2CD_BMP085, BMP085_ADDR, txbuf, 2,
				NULL, 0, MS2ST(BMP085_TIMEOUT_MS));

#if I2C_USE_MUTUAL_EXCLUSION
		i2cReleaseBus(&I2CD_BMP085);
#endif

		if (ret == MSG_TIMEOUT) {
			bmp085_error = BMP085_TIMEOUT;
			goto error;
		}

		// wait pressure results (datasheet says 25.5 ms)
		chThdSleepMilliseconds(BMP085_PRES_DELAY);

		/* unfortunately sensor can not transmit all three values at one transaction */
		// read first byte
		txbuf[0] = BMP085_ADC_XLSB;

#if I2C_USE_MUTUAL_EXCLUSION
		i2cAcquireBus(&I2CD_BMP085);
#endif

		ret = i2cMasterTransmitTimeout(&I2CD_BMP085, BMP085_ADDR, txbuf, 1,
				rxbuf, 2, MS2ST(BMP085_TIMEOUT_MS));

#if I2C_USE_MUTUAL_EXCLUSION
		i2cReleaseBus(&I2CD_BMP085);
#endif

		if (ret == MSG_TIMEOUT) {
			bmp085_error = BMP085_TIMEOUT;
			goto error;
		}
		/* "shift" first byte to the end of buffer */
		bmp085.up = (int32_t) rxbuf[0];

		// read last 2 bytes
		txbuf[0] = BMP085_ADC_MSB;

#if I2C_USE_MUTUAL_EXCLUSION
		i2cAcquireBus(&I2CD_BMP085);
#endif

		ret = i2cMasterTransmitTimeout(&I2CD_BMP085, BMP085_ADDR, txbuf, 1,
				rxbuf, 2, MS2ST(BMP085_TIMEOUT_MS));

#if I2C_USE_MUTUAL_EXCLUSION
		i2cReleaseBus(&I2CD_BMP085);
#endif

		if (ret == MSG_TIMEOUT) {
			bmp085_error = BMP085_TIMEOUT;
			goto error;
		}
		/* now we have all 3 bytes. Calculate pressure using black magic from datasheet */
		bmp085.up |= (int32_t)((rxbuf[0] << 16) + (rxbuf[1] << 8));
		bmp085.up >>= (8 - BMP085_OSS);

		error: chBSemSignal(&bmp085sem);
	}
}

void bmp085_init(void) {
	/* wait until sensor inits */
	chThdSleepMilliseconds(10); //(datasheet 10 ms)

	// Start the i2c driver
	if (I2CD_BMP085.state < I2C_READY) {
		i2cStart(&I2CD_BMP085, &i2cfg);
		palSetPadMode(GPIOB, GPIOB_PIN10, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);
		palSetPadMode(GPIOB, GPIOB_PIN11, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);
	}

	bmp085_error = BMP085_OK;

	bmp085.cc.ac1 = read_calibrate(0xAA, &bmp085_error);
	if (bmp085_error == BMP085_OK)
		bmp085.cc.ac2 = read_calibrate(0xAC, &bmp085_error);
	if (bmp085_error == BMP085_OK)
		bmp085.cc.ac3 = read_calibrate(0xAE, &bmp085_error);
	if (bmp085_error == BMP085_OK)
		bmp085.cc.ac4 = read_calibrate(0xB0, &bmp085_error);
	if (bmp085_error == BMP085_OK)
		bmp085.cc.ac5 = read_calibrate(0xB2, &bmp085_error);
	if (bmp085_error == BMP085_OK)
		bmp085.cc.ac6 = read_calibrate(0xB4, &bmp085_error);
	if (bmp085_error == BMP085_OK)
		bmp085.cc.b1 = read_calibrate(0xB6, &bmp085_error);
	if (bmp085_error == BMP085_OK)
		bmp085.cc.b2 = read_calibrate(0xB8, &bmp085_error);
	if (bmp085_error == BMP085_OK)
		bmp085.cc.mb = read_calibrate(0xBA, &bmp085_error);
	if (bmp085_error == BMP085_OK)
		bmp085.cc.mc = read_calibrate(0xBC, &bmp085_error);
	if (bmp085_error == BMP085_OK)
		bmp085.cc.md = read_calibrate(0xBE, &bmp085_error);

	bmp085_calibrated = (bmp085_error == BMP085_OK);
	baroThread_p = chThdCreateStatic(waBaroThread, sizeof(waBaroThread),
			BARO_PRIO, BaroThread, NULL);
}

#define BMP085_WAIT_MS		100
bmp085_error_t bmp085_read(int16_t *temperature, uint32_t *pressure) {
	bmp085_read_t rd;
	bmp085_read_t *rd_p = &rd;

	chBSemWait(&bmp085sem); /* to be sure */

	chMsgSend(baroThread_p, (msg_t) rd_p);

	/* wait for reply */
	if (chBSemWaitTimeout(&bmp085sem, MS2ST(BMP085_WAIT_MS)) == MSG_TIMEOUT) {
		return BMP085_TIMEOUT;
	}
	chBSemReset(&bmp085sem, FALSE);

	if (bmp085_error != BMP085_OK) {
		return bmp085_error;
	}

	// calculate temp & press
	bmp085_calculate(temperature, pressure);

	return BMP085_OK;
}
