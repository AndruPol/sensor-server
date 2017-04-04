#ifndef __EEPROM_CONFIG_H__
#define __EEPROM_CONFIG_H__

/* Configuration for AT24C32. */

#define EEPROM_PAGE_SIZE        32          /* page size in bytes. Consult datasheet. */
#define EEPROM_SIZE             4096        /* total amount of memory in bytes */
#define EEPROM_WRITE_TIME_MS    10          /* time to write one page in ms. Consult datasheet! */
#define EEPROM_DEVICE			24

#endif /* __EEPROM_CONFIG_H__ */
