/*
 * modbas_slave.c
 *
 *  Created on: 29 июня 2015 г.
 *      Author: andru
 */

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "mbutils.h"
#include "modbus_slave.h"

#define MB_ADDR		1			// slave address
#define MB_PORT		1			// not used now
#define MB_MODE		MB_RTU		// rtu mode only
#define MB_BITRATE	115200		//
#define MB_PARITY	MB_PAR_NONE	//

#define MB_PRIO		(NORMALPRIO+1)	// Modbus process priority
#define MB_WA_SIZE	256

static const uint8_t *UniqProcessorId = (uint8_t *) 0x1FFFF7E8;
static const uint8_t UniqProcessorIdLen = 12;

/*------------------------Slave mode use these variables----------------------*/
//Slave mode:DiscreteInputs variables
USHORT   usSDiscInStart                               = S_DISCRETE_INPUT_START;
#if S_DISCRETE_INPUT_NDISCRETES%8
UCHAR    ucSDiscInBuf[S_DISCRETE_INPUT_NDISCRETES/8+1];
#else
UCHAR    ucSDiscInBuf[S_DISCRETE_INPUT_NDISCRETES/8]  ;
#endif
//Slave mode:Coils variables
USHORT   usSCoilStart                                 = S_COIL_START;
#if S_COIL_NCOILS%8
UCHAR    ucSCoilBuf[S_COIL_NCOILS/8+1]                ;
#else
UCHAR    ucSCoilBuf[S_COIL_NCOILS/8]                  ;
#endif
//Slave mode:InputRegister variables
USHORT   usSRegInStart                                = S_REG_INPUT_START;
SHORT   usSRegInBuf[S_REG_INPUT_NREGS]                ;
//Slave mode:HoldingRegister variables
USHORT   usSRegHoldStart                              = S_REG_HOLDING_START;
SHORT   usSRegHoldBuf[S_REG_HOLDING_NREGS]            ;

/**
 * Modbus slave input register callback function.
 *
 * @param pucRegBuffer input register buffer
 * @param usAddress input register address
 * @param usNRegs input register number
 *
 * @return result
 */
eMBErrorCode eMBRegInputCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    USHORT          iRegIndex;
    USHORT *        pusRegInputBuf;
    USHORT          REG_INPUT_START;
    USHORT          REG_INPUT_NREGS;
    USHORT          usRegInStart;

    pusRegInputBuf = usSRegInBuf;
    REG_INPUT_START = S_REG_INPUT_START;
    REG_INPUT_NREGS = S_REG_INPUT_NREGS;
    usRegInStart = usSRegInStart;

    /* it already plus one in modbus function method. */
    usAddress--;

    if ((usAddress >= REG_INPUT_START)
            && (usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS))
    {
        iRegIndex = usAddress - usRegInStart;
        while (usNRegs > 0)
        {
            *pucRegBuffer++ = (UCHAR) (pusRegInputBuf[iRegIndex] >> 8);
            *pucRegBuffer++ = (UCHAR) (pusRegInputBuf[iRegIndex] & 0xFF);
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

/**
 * Modbus slave holding register callback function.
 *
 * @param pucRegBuffer holding register buffer
 * @param usAddress holding register address
 * @param usNRegs holding register number
 * @param eMode read or write
 *
 * @return result
 */
eMBErrorCode eMBRegHoldingCB(UCHAR * pucRegBuffer, USHORT usAddress,
        USHORT usNRegs, eMBRegisterMode eMode)
{
    eMBErrorCode    eStatus = MB_ENOERR;
    USHORT          iRegIndex;
    USHORT *        pusRegHoldingBuf;
    USHORT          REG_HOLDING_START;
    USHORT          REG_HOLDING_NREGS;
    USHORT          usRegHoldStart;

    pusRegHoldingBuf = usSRegHoldBuf;
    REG_HOLDING_START = S_REG_HOLDING_START;
    REG_HOLDING_NREGS = S_REG_HOLDING_NREGS;
    usRegHoldStart = usSRegHoldStart;

    /* it already plus one in modbus function method. */
    usAddress--;

    if ((usAddress >= REG_HOLDING_START)
            && (usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS))
    {
        iRegIndex = usAddress - usRegHoldStart;
        switch (eMode)
        {
        /* read current register values from the protocol stack. */
        case MB_REG_READ:
            while (usNRegs > 0)
            {
                *pucRegBuffer++ = (UCHAR) (pusRegHoldingBuf[iRegIndex] >> 8);
                *pucRegBuffer++ = (UCHAR) (pusRegHoldingBuf[iRegIndex] & 0xFF);
                iRegIndex++;
                usNRegs--;
            }
            break;

        /* write current register values with new values from the protocol stack. */
        case MB_REG_WRITE:
            while (usNRegs > 0)
            {
                pusRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                pusRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;
            }
            break;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

/**
 * Modbus slave coils callback function.
 *
 * @param pucRegBuffer coils buffer
 * @param usAddress coils address
 * @param usNCoils coils number
 * @param eMode read or write
 *
 * @return result
 */
eMBErrorCode eMBRegCoilsCB(UCHAR * pucRegBuffer, USHORT usAddress,
        USHORT usNCoils, eMBRegisterMode eMode)
{
    eMBErrorCode    eStatus = MB_ENOERR;
    USHORT          iRegIndex , iRegBitIndex , iNReg;
    UCHAR *         pucCoilBuf;
    USHORT          COIL_START;
    USHORT          COIL_NCOILS;
    USHORT          usCoilStart;
    iNReg =  usNCoils / 8 + 1;

    pucCoilBuf = ucSCoilBuf;
    COIL_START = S_COIL_START;
    COIL_NCOILS = S_COIL_NCOILS;
    usCoilStart = usSCoilStart;

    /* it already plus one in modbus function method. */
    usAddress--;

    if( ( usAddress >= COIL_START ) &&
        ( usAddress + usNCoils <= COIL_START + COIL_NCOILS ) )
    {
        iRegIndex = (USHORT) (usAddress - usCoilStart) / 8;
        iRegBitIndex = (USHORT) (usAddress - usCoilStart) % 8;
        switch ( eMode )
        {
        /* read current coil values from the protocol stack. */
        case MB_REG_READ:
            while (iNReg > 0)
            {
                *pucRegBuffer++ = xMBUtilGetBits(&pucCoilBuf[iRegIndex++],
                        iRegBitIndex, 8);
                iNReg--;
            }
            pucRegBuffer--;
            /* last coils */
            usNCoils = usNCoils % 8;
            /* filling zero to high bit */
            *pucRegBuffer = *pucRegBuffer << (8 - usNCoils);
            *pucRegBuffer = *pucRegBuffer >> (8 - usNCoils);
            break;

            /* write current coil values with new values from the protocol stack. */
        case MB_REG_WRITE:
            while (iNReg > 1)
            {
                xMBUtilSetBits(&pucCoilBuf[iRegIndex++], iRegBitIndex, 8,
                        *pucRegBuffer++);
                iNReg--;
            }
            /* last coils */
            usNCoils = usNCoils % 8;
            /* xMBUtilSetBits has bug when ucNBits is zero */
            if (usNCoils != 0)
            {
                xMBUtilSetBits(&pucCoilBuf[iRegIndex++], iRegBitIndex, usNCoils,
                        *pucRegBuffer++);
            }
            break;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

/**
 * Modbus slave discrete callback function.
 *
 * @param pucRegBuffer discrete buffer
 * @param usAddress discrete address
 * @param usNDiscrete discrete number
 *
 * @return result
 */
eMBErrorCode eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    USHORT          iRegIndex , iRegBitIndex , iNReg;
    UCHAR *         pucDiscreteInputBuf;
    USHORT          DISCRETE_INPUT_START;
    USHORT          DISCRETE_INPUT_NDISCRETES;
    USHORT          usDiscreteInputStart;
    iNReg =  usNDiscrete / 8 + 1;

    pucDiscreteInputBuf = ucSDiscInBuf;
    DISCRETE_INPUT_START = S_DISCRETE_INPUT_START;
    DISCRETE_INPUT_NDISCRETES = S_DISCRETE_INPUT_NDISCRETES;
    usDiscreteInputStart = usSDiscInStart;

    /* it already plus one in modbus function method. */
    usAddress--;

    if ((usAddress >= DISCRETE_INPUT_START)
            && (usAddress + usNDiscrete    <= DISCRETE_INPUT_START + DISCRETE_INPUT_NDISCRETES))
    {
        iRegIndex = (USHORT) (usAddress - usDiscreteInputStart) / 8;
        iRegBitIndex = (USHORT) (usAddress - usDiscreteInputStart) % 8;

        while (iNReg > 0)
        {
            *pucRegBuffer++ = xMBUtilGetBits(&pucDiscreteInputBuf[iRegIndex++],
                    iRegBitIndex, 8);
            iNReg--;
        }
        pucRegBuffer--;
        /* last discrete */
        usNDiscrete = usNDiscrete % 8;
        /* filling zero to high bit */
        *pucRegBuffer = *pucRegBuffer << (8 - usNDiscrete);
        *pucRegBuffer = *pucRegBuffer >> (8 - usNDiscrete);
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

bool_t initModbus (void)
{
  eMBErrorCode eStatus;

  eStatus = eMBInit(MB_MODE, MB_ADDR, MB_PORT, MB_BITRATE, MB_PARITY);
  if (eStatus != MB_ENOERR) {
    return FALSE;
  }

  eStatus = eMBSetSlaveID(MB_ADDR, TRUE, UniqProcessorId, UniqProcessorIdLen);
  if (eStatus != MB_ENOERR) {
    return FALSE;
  }

  eStatus = eMBEnable();
  if (eStatus != MB_ENOERR) {
    return FALSE;
  }

  pxMBPortCBTimerExpired();

  return TRUE;
}

/*
 * The MODBUS main thread
 */

static WORKING_AREA(waThdModbus, MB_WA_SIZE);
static msg_t thdModbus(void *arg)
{
  (void)arg;

  chRegSetThreadName("MODBUS");

  while (initModbus() != TRUE) {
    chThdSleepMilliseconds(1000);

    if (chThdShouldTerminate())
      goto cleanAndExit;
  }

  chThdSleepMilliseconds(10);

  do {
    eMBPoll();

  } while (!chThdShouldTerminate());

cleanAndExit:
  eMBDisable();
  eMBClose();

  return 0;
}

void createModbusThd(void) {
  chThdCreateStatic(waThdModbus, sizeof(waThdModbus), MB_PRIO, thdModbus, NULL);
}

void setReceivedBit(uint16_t regAddr, uint8_t ucValue) {
    USHORT          iRegIndex , iRegBitIndex;
    UCHAR *         pucCoilBuf;
    USHORT          usCoilStart;

    if (regAddr < S_REG_INPUT_NREGS) {
    	pucCoilBuf = ucSCoilBuf;
    	usCoilStart = usSCoilStart;

	    if (regAddr + 1    <= S_COIL_START + S_COIL_NCOILS)
	    {
	        iRegIndex = (USHORT) (regAddr - usCoilStart) / 8;
	        iRegBitIndex = (USHORT) (regAddr - usCoilStart) % 8;
            xMBUtilSetBits(&pucCoilBuf[iRegIndex], iRegBitIndex, 1, ucValue);
	    }
  }
}

void setErrorBit(uint16_t regAddr, uint8_t ucValue) {
    USHORT          iRegIndex , iRegBitIndex;
    UCHAR *         pucDiscreteInputBuf;
    USHORT          usDiscreteInputStart;

  if (regAddr < S_REG_INPUT_NREGS) {
	    pucDiscreteInputBuf = ucSDiscInBuf;
	    usDiscreteInputStart = usSDiscInStart;

	    if (regAddr + 1    <= S_DISCRETE_INPUT_START + S_DISCRETE_INPUT_NDISCRETES)
	    {
	        iRegIndex = (USHORT) (regAddr - usDiscreteInputStart) / 8;
	        iRegBitIndex = (USHORT) (regAddr - usDiscreteInputStart) % 8;
	        xMBUtilSetBits(&pucDiscreteInputBuf[iRegIndex], iRegBitIndex, 1, ucValue);
	    }
  }
}

void writeInputReg(uint16_t regAddr, float regValue) {
  int16_t val;
  if (regAddr < S_REG_INPUT_NREGS) {
	  val = (int)  (regValue > 0) ? (regValue + 0.5) : (regValue - 0.5);
	  usSRegInBuf[regAddr] = val;
	  setReceivedBit(regAddr, 1);
	  setErrorBit(regAddr, 0);
  }
}
