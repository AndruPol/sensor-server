/*
 * bmp085.h
 */

#ifndef BMP085_H_

#define BMP085_H_

typedef enum {
	BMP085_OK,
	BMP085_CALIBRATE,
	BMP085_TIMEOUT,
} bmp085_error_t;

typedef struct _bmp085_read_t bmp085_read_t;
struct _bmp085_read_t {
	int16_t temperature; 	/* out */
	uint32_t pressure;	 	/* out */
};

#ifdef __cplusplus
extern "C" {
#endif

// bmp085 init
void bmp085_init(void);
// read compensated temperature and pressure values
// temperature in 0.1C, pressure in Pa, pressure*75/10000 in mmHg
bmp085_error_t bmp085_read(int16_t *temperature, uint32_t *pressure);

#ifdef __cplusplus
}
#endif

#endif /* BMP085_H_ */
