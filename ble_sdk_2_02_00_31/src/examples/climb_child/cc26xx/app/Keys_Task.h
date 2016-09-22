/*******************************************************************************
  Filename:       SensorTag_Tmp.h
  Revised:        $Date: 2013-09-26 16:06:29 +0200 (to, 26 sep 2013) $
  Revision:       $Revision: 35457 $

  Description:    This file contains the Sensor Tag sample application,
                  IR Temperature part, for use with the TI Bluetooth Low 
                  Energy Protocol Stack.

  Copyright 2015  Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
*******************************************************************************/

#ifndef SENSORTAGTMP_H
#define SENSORTAGTMP_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "climb_child_app.h"
   
/*********************************************************************
 * TYPEDEFS
 */
/**
 * GAP Peripheral Role States.
 */
//! [GAP Peripheral Role State]
typedef enum
{
  RIGHT_SHORT = 0,
  LEFT_SHORT,
  RIGHT_LONG,
  LEFT_LONG,
  BOTH,
  REED_SWITCH_LONG
} keys_Notifications_t;

/**
 * Callback when the device has been started.  Callback event to
 * the Notify of a state change.
 */
typedef void (*keysNotify_t)(keys_Notifications_t notificationType);

typedef struct
{
  keysNotify_t   	 pfnKeysNotification;  //!< Whenever the device changes state
} keysEventCBs_t;

/*********************************************************************
 * MACROS
 */


/*********************************************************************
 * CALLBACKS
 */



/*********************************************************************
 * FUNCTIONS
 */
/*
 * Create the IR temperature sensor task
 */
extern void Keys_createTask(void);
//extern void keysTaskFxn(uint32* a0, uint32* a1);

void Keys_Init(keysEventCBs_t *pAppCallbacks);
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SENSORTAGTMP_H */
