/*
 * modbas_slave.c
 *
 *  Created on: 29 июня 2015 г.
 *      Author: andru
 */

#include <string.h>

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "mbutils.h"
#include "modbus_slave.h"

#include "main.h"

#define MB_PRIO		(NORMALPRIO+1)	// Modbus process priority
#define MB_WA_SIZE	384

static const uint8_t *UniqProcessorId = (uint8_t *) 0x1FFFF7E8;
static const uint8_t UniqProcessorIdLen = 12;

static const uint32_t mb_bitrates[] = {2400, 9600, 19200, 38400, 57600, 115200};

/*------------------------Slave mode use these variables----------------------*/
USHORT   usSNRegs;		// Number registers for modbus variables
USHORT   usSCoilLen;	// Discrete & Coil variables array length

//Slave mode:DiscreteInputs variables
USHORT   usSDiscInStart                               = 0;
UCHAR *  pucSDiscInBuf  							  ;

//Slave mode:Coils variables
USHORT   usSCoilStart                                 = 0;
UCHAR *  pucSCoilBuf                 				  ;

// изменнные Coil
UCHAR *  pucSChgCoilBuf                 			  ;

//Slave mode:InputRegister variables
USHORT   usSRegInStart                                = 0;
USHORT * psSRegInBuf					              ;

//Slave mode:HoldingRegister variables
USHORT   usSRegHoldStart                              = 0;
USHORT * psSRegHoldBuf						          ;

uint16_t mbmap_nregcount(void) {
	uint16_t nreg = 0;
	for (uint8_t i=0; i < MAXDEV; i++) {
		nreg += config.devmap[i].addr * 2;
	}
	return nreg;
}

int16_t mbmap_nreg(uint8_t device, uint8_t addr, devtype_t devtype) {
	uint16_t nreg = 0;
	for (uint8_t i=0; i < MAXDEV; i++) {
		if (device == config.devmap[i].id && devtype == config.devmap[i].type) {
			if (addr < config.devmap[i].addr) {
				return nreg +  2 * addr;
			}
			return -1;
		} else {
			nreg += config.devmap[i].addr * 2;
		}
	}
	return -1;
}

uint8_t mbmap_search(uint16_t reg, mb_devmap_t *mb_device) {
	uint16_t nreg = 0;
	for (uint8_t i=0; i < MAXDEV; i++) {
		if (reg >= nreg && reg < (nreg + config.devmap[i].addr * 2) && config.devmap[i].cmd > CMD_NONE) {
			mb_device->id = config.devmap[i].id;
			mb_device->type = config.devmap[i].type;
			mb_device->cmd = config.devmap[i].cmd;
			mb_device->addr = (reg - nreg) / 2;
			return 1;
		} else {
			nreg += config.devmap[i].addr * 2;
		}
	}
	return 0;
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

    pucDiscreteInputBuf = pucSDiscInBuf;
    DISCRETE_INPUT_START = 0;
    DISCRETE_INPUT_NDISCRETES = usSNRegs;
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

    USHORT sz = 0;
    if (usSNRegs % 8) {
    	sz = usSNRegs / 8 + 1;
    } else {
    	sz = usSNRegs / 8;
    }
    memset(pucSChgCoilBuf, 0, sz);

    iNReg =  usNCoils / 8 + 1;

    pucCoilBuf = pucSCoilBuf;
    COIL_START = 0;
    COIL_NCOILS = usSNRegs;
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
            	// check register change
            	uint8_t prev = xMBUtilGetBits(&pucSCoilBuf[iRegIndex], iRegBitIndex, 8);
               	xMBUtilSetBits(&pucSChgCoilBuf[iRegIndex], iRegBitIndex, 8,
                                *pucRegBuffer ^ prev);

            	xMBUtilSetBits(&pucCoilBuf[iRegIndex++], iRegBitIndex, 8,
                        *pucRegBuffer++);
                iNReg--;
            }
            /* last coils */
            usNCoils = usNCoils % 8;
            /* xMBUtilSetBits has bug when ucNBits is zero */
            if (usNCoils != 0)
            {
            	// check register change
            	uint8_t prev = xMBUtilGetBits(&pucCoilBuf[iRegIndex], iRegBitIndex, usNCoils);
                xMBUtilSetBits(&pucSChgCoilBuf[iRegIndex], iRegBitIndex, usNCoils,
                        *pucRegBuffer ^ prev);

                xMBUtilSetBits(&pucCoilBuf[iRegIndex++], iRegBitIndex, usNCoils,
                        *pucRegBuffer++);
            }

            for (uint8_t i = 0; i < usSCoilLen; i++) {
            	if (pucSChgCoilBuf[i] > 0) {
                	chEvtBroadcastFlags(&event_src, EVT_MBCOIL);
                	break;
            	}
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
    USHORT *        psRegInputBuf;
    USHORT          REG_INPUT_START;
    USHORT          REG_INPUT_NREGS;
    USHORT          usRegInStart;

    psRegInputBuf = psSRegInBuf;
    REG_INPUT_START = 0;
    REG_INPUT_NREGS = usSNRegs;
    usRegInStart = usSRegInStart;

    /* it already plus one in modbus function method. */
    usAddress--;

    if ((usAddress >= REG_INPUT_START)
            && (usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS))
    {
        iRegIndex = usAddress - usRegInStart;
        while (usNRegs > 0)
        {
            *pucRegBuffer++ = (UCHAR) (psRegInputBuf[iRegIndex] >> 8);
            *pucRegBuffer++ = (UCHAR) (psRegInputBuf[iRegIndex] & 0xFF);
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
    USHORT *        psRegHoldingBuf;
    USHORT          REG_HOLDING_START;
    USHORT          REG_HOLDING_NREGS;
    USHORT          usRegHoldStart;

    psRegHoldingBuf = psSRegHoldBuf;
    REG_HOLDING_START = 0;
    REG_HOLDING_NREGS = usSNRegs;
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
                *pucRegBuffer++ = (UCHAR) (psRegHoldingBuf[iRegIndex] >> 8);
                *pucRegBuffer++ = (UCHAR) (psRegHoldingBuf[iRegIndex] & 0xFF);
                iRegIndex++;
                usNRegs--;
            }
            break;

        /* write current register values with new values from the protocol stack. */
        case MB_REG_WRITE:
            while (usNRegs > 0)
            {
                psRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                psRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
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


bool initModbus (void)
{
  eMBErrorCode eStatus;

  uint16_t addrs = 0;
  for (uint8_t i = 0; i < MAXDEV; i++) {
	  addrs += config.devmap[i].addr;
  }
  // memory allocation for modbus variables
  if (addrs > 0) {
	  usSNRegs = addrs * 2;
	  psSRegInBuf = (USHORT*) chHeapAlloc(NULL, usSNRegs * 2);
	  assert_param(psSRegInBuf != NULL);
//	  memset(psSRegInBuf, 0, usSNRegs * 2);
	  psSRegHoldBuf = (USHORT*) chHeapAlloc(NULL, usSNRegs * 2);
	  assert_param(psSRegHoldBuf != NULL);
//	  memset(psSRegHoldBuf, 0, usSNRegs * 2);
	  if (usSNRegs % 8) {
		  usSCoilLen = usSNRegs / 8 + 1;
	  } else {
		  usSCoilLen = usSNRegs / 8;
	  }
	  pucSDiscInBuf = (UCHAR*) chHeapAlloc(NULL, usSCoilLen);
	  assert_param(pucSDiscInBuf != NULL);
	  memset(pucSDiscInBuf, 0, usSCoilLen);
	  pucSCoilBuf = (UCHAR*) chHeapAlloc(NULL, usSCoilLen);
	  assert_param(pucSCoilBuf != NULL);
	  memset(pucSCoilBuf, 0, usSCoilLen);
	  pucSChgCoilBuf = (UCHAR*) chHeapAlloc(NULL, usSCoilLen);
	  assert_param(pucSChgCoilBuf != NULL);
	  memset(pucSChgCoilBuf, 0, usSCoilLen);
  }

  eStatus = eMBInit(MB_MODE, config.mb_addr, MB_PORT, mb_bitrates[config.mb_bitrate], config.mb_parity);
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

static THD_WORKING_AREA(waThdModbus, MB_WA_SIZE);
static THD_FUNCTION(thdModbus, arg)
{
  (void)arg;

  chRegSetThreadName("MODBUS");

  while (initModbus() != TRUE) {
    chThdSleepMilliseconds(1000);

    if (chThdShouldTerminateX())
      goto cleanAndExit;
  }

  chThdSleepMilliseconds(10);

  do {
    eMBPoll();

  } while (!chThdShouldTerminateX());

cleanAndExit:
  eMBDisable();
  eMBClose();
}

void createModbusThd(void) {
  chThdCreateStatic(waThdModbus, sizeof(waThdModbus), MB_PRIO, thdModbus, NULL);
}

void setDiscreteBit(uint16_t regAddr, uint8_t ucValue) {
    USHORT          iRegIndex , iRegBitIndex;
    UCHAR *         pucDiscreteInputBuf;
    USHORT          usDiscreteInputStart;

  if (regAddr < usSNRegs) {
	    pucDiscreteInputBuf = pucSDiscInBuf;
	    usDiscreteInputStart = usSDiscInStart;

	    if (regAddr + 1    <= 0 + usSNRegs)
	    {
	        iRegIndex = (USHORT) (regAddr - usDiscreteInputStart) / 8;
	        iRegBitIndex = (USHORT) (regAddr - usDiscreteInputStart) % 8;
	        xMBUtilSetBits(&pucDiscreteInputBuf[iRegIndex], iRegBitIndex, 1, ucValue);
	    }
  }
}

uint8_t getCoilBit(uint16_t regAddr) {
    USHORT          iRegIndex , iRegBitIndex;
    UCHAR *         pucCoilBuf;
    USHORT          usCoilStart;

    if (regAddr < usSNRegs) {
    	pucCoilBuf = pucSCoilBuf;
    	usCoilStart = usSCoilStart;

	    if (regAddr + 1    <= 0 + usSNRegs)
	    {
	        iRegIndex = (USHORT) (regAddr - usCoilStart) / 8;
	        iRegBitIndex = (USHORT) (regAddr - usCoilStart) % 8;
            return xMBUtilGetBits(&pucCoilBuf[iRegIndex], iRegBitIndex, 1);
	    }
    }
    return 0xFF;
}

void setCoilBit(uint16_t regAddr, uint8_t ucValue) {
    USHORT          iRegIndex , iRegBitIndex;
    UCHAR *         pucCoilBuf;
    USHORT          usCoilStart;

    if (regAddr < usSNRegs) {
    	pucCoilBuf = pucSCoilBuf;
    	usCoilStart = usSCoilStart;

	    if (regAddr + 1    <= 0 + usSNRegs)
	    {
	        iRegIndex = (USHORT) (regAddr - usCoilStart) / 8;
	        iRegBitIndex = (USHORT) (regAddr - usCoilStart) % 8;
            xMBUtilSetBits(&pucCoilBuf[iRegIndex], iRegBitIndex, 1, ucValue);
	    }
  }
}

void writeInputRound(uint16_t regAddr, float regValue) {
  int16_t val;
  if (regAddr < usSNRegs) {
	  val = (int)  (regValue > 0) ? (regValue + 0.5) : (regValue - 0.5);
	  psSRegInBuf[regAddr] = val;
	  setCoilBit(regAddr, 1);
	  setDiscreteBit(regAddr, 0);
  }
}

void writeInput(uint16_t regAddr, int16_t val) {
  if (regAddr < usSNRegs) {
	  psSRegInBuf[regAddr] = val;
  }
}

USHORT readHolding(uint16_t regAddr) {
  if (regAddr < usSNRegs) {
	  return psSRegHoldBuf[regAddr];
  }
  return 0;
}

void writeHoldingRound(uint16_t regAddr, float regValue) {
  int16_t val;
  if (regAddr < usSNRegs) {
	  val = (int)  (regValue > 0) ? (regValue + 0.5) : (regValue - 0.5);
	  psSRegHoldBuf[regAddr] = val;
	  setCoilBit(regAddr, 1);
	  setDiscreteBit(regAddr, 0);
  }
}

void writeHolding(uint16_t regAddr, int16_t val) {
  if (regAddr < usSNRegs) {
	  psSRegHoldBuf[regAddr] = val;
  }
}
