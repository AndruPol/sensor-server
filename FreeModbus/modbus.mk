MODBUS = FreeModbus/modbus
MODBUSPORT = FreeModbus/port

MODBUSPORTSRC = ${MODBUSPORT}/portother.c \
		${MODBUSPORT}/porttimer.c \
		${MODBUSPORT}/portserial.c \
		${MODBUSPORT}/portevent.c

#		${MODBUSPORT}/porttcp.c

MODBUSSRC =	${MODBUS}/mb.c \
		${MODBUS}/functions/mbfuncinput.c \
		${MODBUS}/functions/mbfuncdisc.c \
		${MODBUS}/functions/mbfuncholding.c \
		${MODBUS}/functions/mbutils.c \
		${MODBUS}/functions/mbfunccoils.c \
		${MODBUS}/functions/mbfuncdiag.c \
		${MODBUS}/functions/mbfuncother.c \
		${MODBUS}/rtu/mbrtu.c \
		${MODBUS}/rtu/mbcrc.c \
		${MODBUS}/ascii/mbascii.c \
		${MODBUS}/tcp/mbtcp.c \
		${MODBUSPORTSRC}

MODBUSPORTINC = ${MODBUSPORT}

MODBUSINC =	${MODBUS}/include \
		${MODBUSPORTINC}