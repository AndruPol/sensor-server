#Описание протокола обмена шлюза с беспроводным(и) устройствами

*Формат сообщения (длина сообщения = 16 байт):*

    // структура сообщения
    typedef struct MESSAGE MESSAGE_T;  
    struct MESSAGE {  
    	msgtype_t msgType;		// тип сообщения  
    	uint8_t deviceID;		// уникальный идентификатор устройства  
    	sensortype_t sensorType;	// тип датчика  
    	valuetype_t valueType;	// тип показания датчика  
    	uint8_t address;		// адрес датчика внутри устройства  
    	command_t command;		// команда
    	uint8_t error;		// код ошибки датчика/команды  
    	uint8_t notused[5];		//
    	union {			// показание датчика в различных форматах  
    		float	fValue;  
    		int32_t	iValue;  
    		uint8_t cValue[4];  
    	} data;  

    // типы передаваемых сообщений  
    typedef enum {  
    	SENSOR_INFO = 0,	// значение настройки/состояние исполнительного устройства  
    	SENSOR_DATA,	// показание удаленного физического датчика  
    	SENSOR_ERROR,	// ошибка получения показания датчика/выполнения команды  
    	SENSOR_CMD,		// команда удаленному датчику  
    } msgtype_t;  

    // типы датчиков
    typedef enum {
    	DS1820 = 0,
    	BH1750,
    	DHT,
    	BMP085,
    	ADC,
    	HCSR04,
    } sensortype_t;

    // типы значений получаемых от датчиков
    typedef enum {
    	TEMPERATURE = 0,
    	HUMIDITY,
    	PRESSURE,
    	LIGHT,
    	VOLTAGE,
    	DISTANCE,
    } valuetype_t;

    // команды отправляемые устройству (может зависеть от устройства)
	typedef enum {
		CMD_CFGREAD = 1,	// read configuration value
		CMD_CFGWRITE,	// write configuration value
		CMD_RESET,		// reset device
		CMD_SENSREAD = 10,	// read sensor value
		CMD_ON = 20,	// ON
		CMD_OFF,		// OFF
		CMD_ONTM,		// ON timeout (S) message.data.iValue
		CMD_OFFTM,		// OFF timeout (S) message.data.iValue
	} command_t;


*Запись сообщений в регистры Modbus, отправка команд из регистров Modbus*  
 	* - обязательные к заполнению поля
 
	1. Прием показаний датчика:    

    	deviceID* = уникальный код устройства
    	msgType* = SENSOR_DATA
    	sensorType = тип датчика  
    	valueType = тип показания
    	address* = адрес датчика
    	data.iValue или data.fValue = значение датчика

	в случае ошибки чтения датчика устройство отправляет:
    
    	deviceID* = уникальный код устройства
    	msgType* = SENSOR_ERROR
    	sensorType = тип датчика  
    	valueType = тип показания
    	address* = адрес датчика
    	error = код ошибки
    
	2. Отправка команды чтения датчика:

    	deviceID* = уникальный код устройства
    	msgType* = SENSOR_CMD
    	address* = адрес датчика
		command* = CMD_SENSREAD    

    Регистр|Discrete| Coil	| Input	|Holding  
    ----------------------------------------     
    	x  | err	| rd	| erCode |value  
    	x+1| -	| wr	| cnt	 |cmd  
     
     err = 1 - признак ошибки датчика
     erCode - код ошибки (зависит от типа датчика)
     rd = 1 - получено значение/ошибка датчика
     value 	- значение датчика
     wr = 1 - отправить команду датчику
     cmd 	- код команды 
     cnt	- количество команд
    
    3. Чтение параметра конфигурации устройства
  
    	deviceID* = уникальный код устройства
    	msgType* = SENSOR_INFO
    	address* = адрес параметра
    	data.iValue* = значение параметра

    4. Отправка команды чтения параметра конфигурации:

    	deviceID* = уникальный код устройства
    	msgType* = SENSOR_CMD
    	address* = адрес параметра
		command* = CMD_CFGREAD    

    устройство отправляет сообщение:  

    	deviceID* = уникальный код устройства
    	msgType* = SENSOR_INFO
    	address* = адрес параметра
    	data.iValue* = значение параметра

    5. Отправка команды записи параметра конфигурации:

    	deviceID* = уникальный код устройства
    	msgType* = SENSOR_CMD
    	address* = адрес параметра
		command* = CMD_CFGWRITE    
    	data.iValue* = значение параметра

	в случае ошибки записи параметра устройство отправляет:
    
    	deviceID* = уникальный код устройства
    	msgType* = SENSOR_ERROR
    	address* = адрес параметра
    	error* = код ошибки
    
    Регистр|Discrete| Coil	| Input	|Holding  
    ----------------------------------------     
    	x  | err	| rd	| erCode | value  
    	x+1| -	| wr	| cnt	 | cmd  
     
     err = 1 - признак ошибки записи параметра
     erCode - код ошибки
     rd = 1 - получено значение параметра/ошибки
     value 	- значение параметра
     wr = 1 - отправить команду датчику
     cnt	- количество команд

    6. Чтение состояния исполнительного устройства
  
    Команда чтения состояния:  
  		
    	deviceID* = уникальный код устройства
    	msgType* = SENSOR_CMD
    	address* = адрес исполнительного устройства
		command* = CMD_SENSREAD     

    устройство отправляет сообщение:  

    	deviceID* = уникальный код устройства
    	msgType* = SENSOR_INFO
    	address* = адрес исполнительного устройства
    	data.iValue* = значение состояния

    7. Отправка команды исполнительному устройству:

    	deviceID* = уникальный код устройства
    	msgType* = SENSOR_CMD
    	address* = адрес исполнительного устройства
		command* = CMD_ON | CMD_OFF | CMD_ONTM | CMD_OFFTM     

	в случае ошибки выполнения команды устройство отправляет:
    
    	deviceID* = уникальный код устройства
    	msgType* = SENSOR_ERROR
    	address* = адрес исполнительного устройства
    	error* = код ошибки (зависит от исполнительного устройства)
    	data.iValue* = значение состояния
    
    Регистр|Discrete| Coil	| Input	|Holding  
    ----------------------------------------     
    	x  | err	| rd	| erCode | <value,>param  
    	x+1| -	| wr	| cnt	 | cmd  
     
     err = 1 - признак ошибки выполнения команды
     erCode - код ошибки (зависит от исполнительного устройства)
     rd = 1 - получено значение состояния/ошибки
     value 	- значение состояния
     wr = 1 - отправить команду датчику
     cmd 	- команда исполнительному устройству
     param - параметр команды (таймаут)
     cnt	- количество команд
    
    