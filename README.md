
# Шлюз приема/передачи данных с удаленных модулей nRF24LXX (nRF24LE1-sensor, др.) на базе ChibiOS 3.x.x or later

- прием с удаленных датчиков nRF24LXX

- опрос внутреннего датчика BMP085

- передача показаний на ПК по протоколу Modbus RTU через RS485

- I2C EEPROM для хранения настроек шлюза

- IWDG строжевой таймер

- AES128 ECB шифрование данных


## Custom gateway board on MCU STM32F103C8Tx


PERIPHERALS	MODES	FUNCTIONS	PINS

подключение EEPROM, BMP085 

- I2C2	I2C	I2C2_SCL	PB10
- I2C2	I2C	I2C2_SDA	PB11

связь с удаленными модулями nRF24LXX

- SPI1	Full-Duplex Master	SPI1_MISO	PA6
- SPI1	Full-Duplex Master	SPI1_MOSI	PA7
- SPI1	Full-Duplex Master	SPI1_SCK	PA5
- SPI1	Hardware	SPI1_NSS	PA4

SWD

- SYS	Trace-Asynchronous_SW	SYS_JTMS-SWDIO	PA13
- SYS	Trace-Asynchronous_SW	SYS_JTCK-SWCLK	PA14
- SYS	Trace-Asynchronous_SW	SYS_JTDO-TRACESWO	PB3

Modbus через RS485

- TIM4	Internal Clock	TIM4_VS_ClockSourceINT	VP_TIM4_VS_ClockSourceINT
- USART2	Asynchronous	USART2_RX	PA3
- USART2	Asynchronous	USART2_TX	PA2

Pin Nb	PINs	FUNCTIONs	LABELs

cветодиод работы

- 2	PC13-TAMPER-RTC	GPIO_Output	SYSLED

связь с удаленными датчиками nRF24L01

- 14	PA4	SPI1_NSS	NRF24_NSS
- 15	PA5	SPI1_SCK	NRF24_SCK
- 16	PA6	SPI1_MISO	NRF24_MISO
- 17	PA7	SPI1_MOSI	NRF24_MOSI
- 45	PB8	GPIO_Output	NRF24_CE
- 46	PB9	GPIO_Input	NRF24_IRQ

Modbus через RS485

- 29	PA8	GPIO_Output	MODBAS_TX_EN
- 12	PA2	USART2_TX	MODBAS_TX
- 13	PA3	USART2_RX	MODBAS_RX

SWD 

- 34	PA13	SYS_JTMS-SWDIO	
- 37	PA14	SYS_JTCK-SWCLK	
- 39	PB3	SYS_JTDO-TRACESWO	

подключение BMP085, EEPROM AT24C32

- 21	PB10	I2C1_SCL	SCL
- 22	PB11	I2C1_SDA	SDA
