/*******************************************************************************
  Filename:       SensorTag_Tmp.c
  Revised:        $Date: 2013-11-06 17:27:44 +0100 (on, 06 nov 2013) $
  Revision:       $Revision: 35922 $

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

/*********************************************************************
 * INCLUDES
 */
#include "gatt.h"
#include "gattservapp_giova.h"

#include "Keys_Task.h"
#include <ti/mw/sensors/SensorUtil.h>
//#include "sensor_tmp007.h"
//#include "sensor.h"
#include "Board.h"

#include "string.h"
#include "util.h"
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

#include "hci_tl.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Task configuration
#define KEYS_TASK_PRIORITY    2
#define KEYS_TASK_STACK_SIZE  512


#define KEY_RIGHT_EVT		      0x0001
#define KEY_LEFT_EVT		      0x0002
#define KEY_RELAY_EVT		      0x0004
#define LONG_PRESSURE_TIMEOUT_EVT 0x0008

#define KEYS_LONG_PRESSURE_TIMEOUT 2000
#define ANTIBOUNCHE_TIMEOUT_MS     100
/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct {
  appEvtHdr_t hdr;  // event header.
  uint8_t *pData;  // event data
} keysTaskEvt_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
// Entity ID used to check for source and/or destination of messages
static ICall_EntityID keysSelfEntity;

// Semaphore used to post events to the application thread
static ICall_Semaphore keysSem;

// Task setup
static Task_Struct keysTask;
static Char keysTaskStack[KEYS_TASK_STACK_SIZE];

// Pins that are actively used by the application
static PIN_Config KeysPinTable[] =
{
    Board_KEY_LEFT   | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,        /* Button is active low          */
    Board_KEY_RIGHT  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,        /* Button is active low          */
#ifdef CC2650STK
    Board_RELAY      | PIN_INPUT_EN | PIN_PULLDOWN | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,      /* Relay is active high          */
#endif
    PIN_TERMINATE
};

// Global pin resources
static PIN_State pinGpioStateKeys;
static PIN_Handle hGpioPinKeys;

// Application callbacks
static keysEventCBs_t *Keys_AppCGs = NULL;

// Task pending events
static uint16_t events = 0;

static uint8_t longPressedButtonCheck = 0;

static Clock_Struct longPressCheckClock;

static uint8 longPressNotiSent = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void keysTaskFxn(UArg a0, UArg a1);
static void Key_callback(PIN_Handle handle, PIN_Id pinId);
static void Keys_clockHandler(UArg arg);
static void keysTask_setEvent(uint32_t event);
static void leftKeyEvent_Handler(void);
static void rightKeyEvent_Handler(void);
static void relayEvent_Handler(void);
static void longPress_Handler(void);
#ifdef STACK_METRICS
static void checkStackMetrics(void);
#endif
/*********************************************************************
 * PROFILE CALLBACKS
 */



/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SensorTagTmp_createTask
 *
 * @brief   Task creation function for the SensorTag
 *
 * @param   none
 *
 * @return  none
 */
void Keys_createTask(void)
{
  Task_Params taskParames;

  // Create the task for the state machine
  Task_Params_init(&taskParames);
  taskParames.stack = keysTaskStack;
  taskParames.stackSize = KEYS_TASK_STACK_SIZE;
  taskParames.priority = KEYS_TASK_PRIORITY;

  Task_construct(&keysTask, keysTaskFxn, &taskParames, NULL);
}

void Keys_Init(keysEventCBs_t *pAppCallbacks){

    if (pAppCallbacks)
    {
      Keys_AppCGs = pAppCallbacks;
    }

}

/*********************************************************************
* Private functions
*/

/*********************************************************************
 * @fn      keysTaskInit
 *
 * @brief   Initialization function for the SensorTag IR temperature sensor
 *
 */
PIN_Status status;
static void keysTaskInit(void)
{

	Util_constructClock(&longPressCheckClock, Keys_clockHandler, KEYS_LONG_PRESSURE_TIMEOUT,
			0, false, LONG_PRESSURE_TIMEOUT_EVT);

	hGpioPinKeys = PIN_open(&pinGpioStateKeys, KeysPinTable);
	status = PIN_registerIntCb(hGpioPinKeys, Key_callback);

	// Register task with BLE stack
	ICall_registerApp(&keysSelfEntity, &keysSem);
}

/*********************************************************************
 * @fn      keysTaskFxn
 *
 * @brief   The task loop of the temperature readout task
 *
 * @return  none
 */
static void keysTaskFxn(UArg a0, UArg a1)
{

	// Initialize the task
  keysTaskInit();

  // Profile main loop
    for (;;)
    {
      // Waits for a signal to the semaphore associated with the calling thread.
      // Note that the semaphore associated with a thread is signaled when a
      // message is queued to the message receive queue of the thread or when
      // ICall_signal() function is called onto the semaphore.
      ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);

      if (events & KEY_LEFT_EVT){
        events &= ~KEY_LEFT_EVT;

        //delay_ms(ANTIBOUNCHE_TIMEOUT_MS);

        leftKeyEvent_Handler();

      }
      if (events & KEY_RIGHT_EVT){
        events &= ~KEY_RIGHT_EVT;

        //delay_ms(ANTIBOUNCHE_TIMEOUT_MS);

        rightKeyEvent_Handler();

      }
      if (events & KEY_RELAY_EVT){
         events &= ~KEY_RELAY_EVT;

         relayEvent_Handler();
      }
      if (events & LONG_PRESSURE_TIMEOUT_EVT){
         events &= ~LONG_PRESSURE_TIMEOUT_EVT;

         longPress_Handler();
      }

#ifdef STACK_METRICS
      checkStackMetrics();
#endif
    } // for
}


static void leftKeyEvent_Handler(void) {
	if (PIN_getInputValue(Board_KEY_LEFT) == 0) {
		longPressedButtonCheck |= KEY_LEFT_EVT;
		longPressNotiSent = 0;

		if(Util_isActive(&longPressCheckClock)){
			Util_startClock(&longPressCheckClock);
		}else{
			Util_startClock(&longPressCheckClock);
		}
	} else {
		longPressedButtonCheck &= ~KEY_LEFT_EVT;
		if(longPressedButtonCheck == 0){
			Util_stopClock(&longPressCheckClock);
		}

		if (Keys_AppCGs && longPressNotiSent == 0) {
			Keys_AppCGs->pfnKeysNotification(LEFT_SHORT);
		}
	}
}

static void rightKeyEvent_Handler(void) {

	if (PIN_getInputValue(Board_KEY_RIGHT) == 0) {
		longPressedButtonCheck |= KEY_RIGHT_EVT;
		longPressNotiSent = 0;

		if(Util_isActive(&longPressCheckClock)){
			Util_startClock(&longPressCheckClock);
		}else{
			Util_startClock(&longPressCheckClock);
		}
	} else {
		longPressedButtonCheck &= ~KEY_RIGHT_EVT;
		if(longPressedButtonCheck == 0){
			Util_stopClock(&longPressCheckClock);
		}

		if (Keys_AppCGs && longPressNotiSent == 0) {
			Keys_AppCGs->pfnKeysNotification(RIGHT_SHORT);
		}
	}



}

static void relayEvent_Handler(void){

}

static void longPress_Handler(void){

	if (longPressedButtonCheck == KEY_LEFT_EVT) { //pressione lunga solo del tasto sinistro
		longPressedButtonCheck = 0;
		if (PIN_getInputValue(Board_KEY_LEFT) == 0) {
			Keys_AppCGs->pfnKeysNotification(LEFT_LONG);
		}
	}

	if (longPressedButtonCheck == KEY_RIGHT_EVT) { //pressione lunga solo del tasto destra
		longPressedButtonCheck = 0;
		if (PIN_getInputValue(Board_KEY_RIGHT) == 0) {
			Keys_AppCGs->pfnKeysNotification(RIGHT_LONG);
		}
	}

	if (longPressedButtonCheck == KEY_RIGHT_EVT | KEY_LEFT_EVT) { //pressione lunga di entrambi
		longPressedButtonCheck = 0;
		if (PIN_getInputValue(Board_KEY_RIGHT) == 0 && PIN_getInputValue(Board_KEY_LEFT) == 0) {
			while((PIN_getInputValue(Board_KEY_RIGHT) == 0 || PIN_getInputValue(Board_KEY_LEFT) == 0)){
				//delay_ms(100);
				DELAY_MS(100);
			}
			Keys_AppCGs->pfnKeysNotification(BOTH);
			//HCI_EXT_ResetSystemCmd(HCI_EXT_RESET_SYSTEM_HARD);
		}

	}

	longPressNotiSent = 1;


}
/*!*****************************************************************************
 *  @fn         Key_callback
 *
 *  Interrupt service routine for buttons, relay and MPU
 *
 *  @param      handle PIN_Handle connected to the callback
 *
 *  @param      pinId  PIN_Id of the DIO triggering the callback
 *
 *  @return     none
 ******************************************************************************/
static void Key_callback(PIN_Handle handle, PIN_Id pinId)
{
	if(pinId == Board_KEY_LEFT){
		keysTask_setEvent(KEY_LEFT_EVT);
	}

	if(pinId == Board_KEY_RIGHT){
		keysTask_setEvent(KEY_RIGHT_EVT);
	}
#ifdef CC2650STK
	if(pinId == Board_RELAY){
		keysTask_setEvent(KEY_RELAY_EVT);
	}
#endif
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void Keys_clockHandler(UArg arg) {
	// Store the event.
	events |= arg;

	// Wake up the application.
	Semaphore_post(keysSem);
}

/*********************************************************************
 * @fn      keysTask_setEvent
 *
 * @brief   Set an event
 *
 * @param   event - event to be set
 *
 * @return  none
 */
static void keysTask_setEvent(uint32_t event)
{
  events |= event;

  // Wake up the application thread when it waits for clock event
  Semaphore_post(keysSem);
}


#ifdef STACK_METRICS
static void checkStackMetrics() {
	static uint16 maxStackUse = 0;

	Task_Stat statbuf; /* declare buffer */
	Task_stat(Task_self(), &statbuf); /* call func to get status */
//	System_printf("\nSTACK METRICS\n");

	if(statbuf.used > maxStackUse){
		maxStackUse = statbuf.used;
	}
//#ifdef PRINTF_ENABLED
//	System_printf("stack size: %d, stack pointer: 0x%x, stack base: 0x%x, used: %d, absolute max used: %d\n", statbuf.stackSize ,statbuf.sp, statbuf.stack , statbuf.used, maxStackUse);
//	if (statbuf.used > (statbuf.stackSize * 9 / 10)) {
//		System_printf("Over 90% of task's stack is in use.\n");
//	}
//#endif

	//RESET UNUSED STACK
	uint32 i;

	for(i = 0; i < (statbuf.sp - statbuf.stack)-1 ; i++){ //-1 for safety, don't care warning
		keysTaskStack[i] = 0xBE;
	}


}
#endif
/*********************************************************************
*********************************************************************/

