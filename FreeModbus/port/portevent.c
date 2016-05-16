/*
 * FreeModbus Libary: Atmel AT91SAM3S Demo Application
 * Copyright (C) 2010 Christian Walter <cwalter@embedded-solutions.at>
 *
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * IF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: portevent.c,v 1.1 2010/06/06 13:07:20 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include <stdlib.h>
#include <ch.h>


/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mb_m.h"
#include "mbport.h"
#include "port.h"

/* ----------------------- Variables ----------------------------------------*/
static msg_t bufferQueue;
static Mailbox xQueueHdl;
//static MAILBOX_DECL(xQueueHdl, bufferQueue, 1);


/* ----------------------- Start implementation -----------------------------*/
#if MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0

static BinarySemaphore xMasterRunRes;

BOOL
xMBMasterPortEventInit( void )
{
  chMBInit(&xQueueHdl, &bufferQueue, 1);

  return TRUE;
}

BOOL
xMBMasterPortEventPost( eMBMasterEventType eEvent )
{
  BOOL            bStatus = TRUE;

  if (bMBPortIsWithinException () == TRUE) {

    if (chMBPostI (&xQueueHdl, (msg_t) eEvent) != RDY_OK)
      bStatus = FALSE;

  } else {

    if (chMBPost (&xQueueHdl, (msg_t) eEvent, TIME_INFINITE) != RDY_OK)
      bStatus = FALSE;
  }

  return bStatus;
}

BOOL
xMBMasterPortEventGet( eMBMasterEventType * peEvent )
{
  if (chMBFetch (&xQueueHdl, (msg_t *) peEvent, TIME_INFINITE) != RDY_OK)
	return FALSE;

  return TRUE;
}

/**
 * This function is initialize the OS resource for modbus master.
 * Note:The resource is define by OS.If you not use OS this function can be empty.
 *
 */
void vMBMasterOsResInit( void )
{
    chBSemInit(&xMasterRunRes, FALSE);
}

/**
 * This function is take Modbus Master running resource.
 * Note:The resource is define by Operating System.If you not use OS this function can be just return TRUE.
 *
 * @param lTimeOut the waiting time.
 *
 * @return resource taked result
 */
BOOL xMBMasterRunResTake( LONG lTimeOut )
{
    /*If waiting time is -1 .It will wait forever */
   	if (lTimeOut == -1)
   		return chBSemWaitTimeout(&xMasterRunRes, TIME_INFINITE) == RDY_TIMEOUT ? FALSE : TRUE ;
   	else
   		return chBSemWaitTimeout(&xMasterRunRes, lTimeOut) == RDY_TIMEOUT ? FALSE : TRUE ;
}

/**
 * This function is release Modbus Master running resource.
 * Note:The resource is define by Operating System.If you not use OS this function can be empty.
 *
 */
void vMBMasterRunResRelease( void )
{
    /* release resource */
    if (bMBPortIsWithinException () == TRUE) {
    	chBSemSignalI(&xMasterRunRes);
    } else {
    	chBSemSignal(&xMasterRunRes);
    }
}

/**
 * This is modbus master respond timeout error process callback function.
 * @note There functions will block modbus master poll while execute OS waiting.
 * So,for real-time of system.Do not execute too much waiting process.
 *
 * @param ucDestAddress destination salve address
 * @param pucPDUData PDU buffer data
 * @param ucPDULength PDU buffer length
 *
 */
void vMBMasterErrorCBRespondTimeout(UCHAR ucDestAddress, const UCHAR* pucPDUData,
        USHORT ucPDULength) {
    (void)ucDestAddress;
    (void)pucPDUData;
    (void)ucPDULength;
	/**
     * @note This code is use OS's event mechanism for modbus master protocol stack.
     * If you don't use OS, you can change it.
     */
    if (bMBPortIsWithinException () == TRUE) {
    	chMBPostI (&xQueueHdl, EV_MASTER_ERROR_RESPOND_TIMEOUT);
    } else {
    	chMBPost (&xQueueHdl, EV_MASTER_ERROR_RESPOND_TIMEOUT, TIME_IMMEDIATE);
    }

    /* You can add your code under here. */

}

/**
 * This is modbus master receive data error process callback function.
 * @note There functions will block modbus master poll while execute OS waiting.
 * So,for real-time of system.Do not execute too much waiting process.
 *
 * @param ucDestAddress destination salve address
 * @param pucPDUData PDU buffer data
 * @param ucPDULength PDU buffer length
 *
 */
void vMBMasterErrorCBReceiveData(UCHAR ucDestAddress, const UCHAR* pucPDUData,
        USHORT ucPDULength) {
    (void)ucDestAddress;
    (void)pucPDUData;
    (void)ucPDULength;
    /**
     * @note This code is use OS's event mechanism for modbus master protocol stack.
     * If you don't use OS, you can change it.
     */
    if (bMBPortIsWithinException () == TRUE) {
    	chMBPostI (&xQueueHdl, EV_MASTER_ERROR_RECEIVE_DATA);
    } else {
    	chMBPost (&xQueueHdl, EV_MASTER_ERROR_RECEIVE_DATA, TIME_IMMEDIATE);
    }

    /* You can add your code under here. */

}

/**
 * This is modbus master execute function error process callback function.
 * @note There functions will block modbus master poll while execute OS waiting.
 * So,for real-time of system.Do not execute too much waiting process.
 *
 * @param ucDestAddress destination salve address
 * @param pucPDUData PDU buffer data
 * @param ucPDULength PDU buffer length
 *
 */
void vMBMasterErrorCBExecuteFunction(UCHAR ucDestAddress, const UCHAR* pucPDUData,
        USHORT ucPDULength) {
    (void)ucDestAddress;
    (void)pucPDUData;
    (void)ucPDULength;
    /**
     * @note This code is use OS's event mechanism for modbus master protocol stack.
     * If you don't use OS, you can change it.
     */
    if (bMBPortIsWithinException () == TRUE) {
    	chMBPostI (&xQueueHdl, EV_MASTER_ERROR_EXECUTE_FUNCTION);
    } else {
    	chMBPost (&xQueueHdl, EV_MASTER_ERROR_EXECUTE_FUNCTION, TIME_IMMEDIATE);
    }

    /* You can add your code under here. */

}

/**
 * This is modbus master request process success callback function.
 * @note There functions will block modbus master poll while execute OS waiting.
 * So,for real-time of system.Do not execute too much waiting process.
 *
 */
void vMBMasterCBRequestSuccess( void ) {
    /**
     * @note This code is use OS's event mechanism for modbus master protocol stack.
     * If you don't use OS, you can change it.
     */
    if (bMBPortIsWithinException () == TRUE) {
    	chMBPostI (&xQueueHdl, EV_MASTER_PROCESS_SUCCESS);
    } else {
    	chMBPost (&xQueueHdl, EV_MASTER_PROCESS_SUCCESS, TIME_IMMEDIATE);
    }

    /* You can add your code under here. */

}

/**
 * This function is wait for modbus master request finish and return result.
 * Waiting result include request process success, request respond timeout,
 * receive data error and execute function error.You can use the above callback function.
 * @note If you are use OS, you can use OS's event mechanism. Otherwise you have to run
 * much user custom delay for waiting.
 *
 * @return request error code
 */
eMBMasterReqErrCode eMBMasterWaitRequestFinish( void ) {
    eMBMasterReqErrCode    eErrStatus = MB_MRE_NO_ERR;
    msg_t recvedEvent;
    /* waiting for OS event */
#if 0
    do {
    	chMBFetch (&xQueueHdl, &recvedEvent, TIME_INFINITE);
    	if (recvedEvent < EV_MASTER_ERROR_PROCESS)
    		chMBPost(&xQueueHdl, recvedEvent, TIME_IMMEDIATE);
    	else
    		break;
    } while ( 1 );
#endif

    switch (recvedEvent)
    {
    case EV_MASTER_ERROR_PROCESS:
        eErrStatus = MB_MRE_TIMEDOUT;
        break;
    case EV_MASTER_PROCESS_SUCCESS:
        break;
    case EV_MASTER_ERROR_RESPOND_TIMEOUT:
    {
        eErrStatus = MB_MRE_TIMEDOUT;
        break;
    }
    case EV_MASTER_ERROR_RECEIVE_DATA:
    {
        eErrStatus = MB_MRE_REV_DATA;
        break;
    }
    case EV_MASTER_ERROR_EXECUTE_FUNCTION:
    {
        eErrStatus = MB_MRE_EXE_FUN;
        break;
    }
    }
    vMBMasterRunResRelease ();

    return eErrStatus;
}

#elif MB_SLAVE_RTU_ENABLED > 0 || MB_SLAVE_ASCII_ENABLED > 0

BOOL
xMBPortEventInit( void )
{
  chMBInit(&xQueueHdl, &bufferQueue, 1);

  return TRUE;
}

BOOL
xMBPortEventPost( eMBEventType eEvent )
{
  BOOL            bStatus = TRUE;

  if (bMBPortIsWithinException () == TRUE) {
    
    if (chMBPostI (&xQueueHdl, (msg_t) eEvent) != RDY_OK)
      bStatus = FALSE;
    
  } else {
    
    if (chMBPost (&xQueueHdl, (msg_t) eEvent, TIME_INFINITE) != RDY_OK)
      bStatus = FALSE;
  }

  return bStatus;
}

BOOL
xMBPortEventGet( eMBEventType * peEvent )
{
    BOOL            xEventHappened = FALSE;

    if(chMBFetch (&xQueueHdl, (msg_t *) peEvent, MS2ST (50)) == RDY_OK)
      xEventHappened = TRUE;

    return xEventHappened;
}

#endif

void
vMBPortEventClose( void )
{
  chMBReset (&xQueueHdl);
}
