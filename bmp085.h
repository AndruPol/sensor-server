/*
 * bmp085.h
 */

#ifndef BMP085_H_
#define BMP085_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "ch.h"
#include "hal.h"

#define BARO_PRIO	(NORMALPRIO + 1)
typedef enum {
	BMP085_NO_ERROR,
	BMP085_ERROR,
	BMP085_TIMEOUT,
} bmp085_error_t;

typedef struct _bmp085_read_t bmp085_read_t;
struct _bmp085_read_t {
	bmp085_error_t error;	/* out */
	int16_t temperature; 	/* out */
	uint32_t pressure;	 	/* out */
};

// bmp085 init
void bmp085_init(void);
// read compensated temperature and pressure values
//int16_t  temperature in 0.1C
//uint32_t pressure in Pa, pressure*75/10000 in mmHg
bmp085_error_t bmp085_read(int16_t *temperature, uint32_t *pressure);

#ifdef __cplusplus
}
#endif
#endif /* BMP085_H_ */
