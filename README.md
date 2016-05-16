
# Шлюз приема данных с удаленных датчиков на nRF24LE1 (nRF24LE1-sensor) на базе ChibiOS 2.6.8 or later

- опрос удаленных датчиков nRF24LE1

- опрос датчика BMP085

- передача показаний датчиков на ПК по протоколу Modbus RTU через RS485


## Custom gateway board on MCU STM32F103C8Tx


PERIPHERALS	MODES	FUNCTIONS	PINS

подключение BMP085 

- I2C1	I2C	I2C1_SCL	PB6
- I2C1	I2C	I2C1_SDA	PB7

связь с удаленными датчиками nRF24L01

- SPI1	Full-Duplex Master	SPI1_MISO	PA6
- SPI1	Full-Duplex Master	SPI1_MOSI	PA7
- SPI1	Full-Duplex Master	SPI1_SCK	PA5
- SPI1	Hardware	SPI1_NSS	PA4

SWD

- SYS	Trace-Asynchronous_SW	SYS_JTMS-SWDIO	PA13
- SYS	Trace-Asynchronous_SW	SYS_JTCK-SWCLK	PA14
- SYS	Trace-Asynchronous_SW	SYS_JTDO-TRACESWO	PB3

Modbus через RS485

- TIM2	Internal Clock	TIM2_VS_ClockSourceINT	VP_TIM2_VS_ClockSourceINT
- USART1	Asynchronous	USART1_RX	PA10
- USART1	Asynchronous	USART1_TX	PA9

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
- 30	PA9	USART1_TX	MODBAS_TX
- 31	PA10	USART1_RX	MODBAS_RX

SWD 

- 34	PA13	SYS_JTMS-SWDIO	
- 37	PA14	SYS_JTCK-SWCLK	
- 39	PB3	SYS_JTDO-TRACESWO	

подключение BMP085

- 42	PB6	I2C1_SCL	BMP085_SCL
- 43	PB7	I2C1_SDA	BMP085_SDA
