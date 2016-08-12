/******************************************************************************
 * @file  multirole.c
 *
 * @description Multirole GAPRole abstraction layer for GAP
 *
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright   
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include <xdc/std.h>

#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include <driverlib/ioc.h>

#include "gap.h"
#include "gatt.h"
#include "hci_tl.h"
#include "linkdb.h"
#include "util.h"

#include "gattservapp.h"
#include "multi_giova.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
// Profile Events
#define START_ADVERTISING_EVT         0x0001  // Start Advertising
#define START_CONN_UPDATE_EVT         0x0002  // Start Connection Update Procedure
#define CONN_PARAM_TIMEOUT_EVT        0x0004  // Connection Parameters Update Timeout

#define DEFAULT_ADVERT_OFF_TIME       30000   // 30 seconds

#define DEFAULT_MIN_CONN_INTERVAL     0x0006  // 100 milliseconds
#define DEFAULT_MAX_CONN_INTERVAL     0x0C80  // 4 seconds

#define MIN_CONN_INTERVAL             0x0006
#define MAX_CONN_INTERVAL             0x0C80

#define DEFAULT_TIMEOUT_MULTIPLIER    1000

#define CONN_INTERVAL_MULTIPLIER      6

#define MIN_SLAVE_LATENCY             0
#define MAX_SLAVE_LATENCY             500

#define MIN_TIMEOUT_MULTIPLIER        0x000a
#define MAX_TIMEOUT_MULTIPLIER        0x0c80

#define MAX_TIMEOUT_VALUE             0xFFFF

// Task configuration
#define GAPROLE_TASK_PRIORITY         3

#ifndef GAPROLE_TASK_STACK_SIZE
#define GAPROLE_TASK_STACK_SIZE       600
#endif

#ifndef MAX_SCAN_RESPONSES
#define MAX_SCAN_RESPONSES 3
#endif

/*********************************************************************
 * TYPEDEFS
 */

// App event structure
typedef struct {
	uint8_t event;  // event type
	uint8_t status; // event status
} gapRoleEvt_t;

// set/get param event structure
typedef struct {
	uint16_t connHandle;
	uint16_t value;
} gapRoleInfoParam_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// structure to store link attributes 
gapRoleInfo_t multiConnInfo[MAX_NUM_BLE_CONNS];

// Link DB maximum number of connections
uint8 linkDBNumConns;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Queue object used for app messages
static Queue_Struct appMsg;

// Clock object used to signal timeout
static Clock_Struct startAdvClock;
static Clock_Struct startUpdateClock;
static Clock_Struct updateTimeoutClock;

// Task pending events
static uint16_t events = 0;

// Task setup
Task_Struct gapRoleTask;
Char gapRoleTaskStack[GAPROLE_TASK_STACK_SIZE];

/*********************************************************************
 * Profile Parameters - reference GAPROLE_PROFILE_PARAMETERS for
 * descriptions
 */
static uint8_t gapRole_profileRole;
static uint8_t gapRole_IRK[KEYLEN];
static uint8_t gapRole_SRK[KEYLEN];
static uint32_t gapRole_signCounter;
static uint8_t gapRole_bdAddr[B_ADDR_LEN];
static uint8_t gapRole_AdvEnabled = TRUE;
static uint8_t gapRole_AdvNonConnEnabled = FALSE;
static uint16_t gapRole_AdvertOffTime = DEFAULT_ADVERT_OFF_TIME;
static uint8_t gapRole_AdvertDataLen = 3;

static uint8_t gapRole_AdvertData[B_MAX_ADV_LEN] = { 0x02,   // length of this data
		GAP_ADTYPE_FLAGS,   // AD Type = Flags
		// Limited Discoverable & BR/EDR not supported
		(GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

static uint8_t gapRole_ScanRspDataLen = 0;
static uint8_t gapRole_ScanRspData[B_MAX_ADV_LEN] = { 0 };
static uint8_t gapRole_AdvEventType;
static uint8_t gapRole_AdvDirectType;
static uint8_t gapRole_AdvDirectAddr[B_ADDR_LEN] = { 0 };
static uint8_t gapRole_AdvChanMap;
static uint8_t gapRole_AdvFilterPolicy;

static uint8_t paramUpdateNoSuccessOption = GAPROLE_NO_ACTION;

//these are the same for each connection
static uint16_t gapRole_MinConnInterval = DEFAULT_MIN_CONN_INTERVAL;
static uint16_t gapRole_MaxConnInterval = DEFAULT_MAX_CONN_INTERVAL;
static uint16_t gapRole_SlaveLatency = MIN_SLAVE_LATENCY;
static uint16_t gapRole_TimeoutMultiplier = DEFAULT_TIMEOUT_MULTIPLIER;

static uint8_t gapRole_ParamUpdateEnable = FALSE;

static uint8_t gapRoleMaxScanRes = 0;

// Application callbacks
static gapRolesCBs_t *pGapRoles_AppCGs = NULL;

static Peripheral_States_t gapRole_peripheralState;
/*********************************************************************
 * Profile Attributes - variables
 */

/*********************************************************************
 * Profile Attributes - Table
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void gapRole_init(void);
static void gapRole_taskFxn(UArg a0, UArg a1);

static uint8 gapRole_processStackMsg(ICall_Hdr *pMsg);
static uint8_t gapRole_processGAPMsg(gapEventHdr_t *pMsg);
static void gapRole_SetupGAP(void);
static void gapRole_HandleParamUpdateNoSuccess(void);
static bStatus_t gapRole_startConnUpdate(uint8_t handleFailure, uint8 connHandle);
static void gapRole_setEvent(uint32_t event);

uint8_t gapRoleInfo_Find(uint16 connectionHandle);
uint8 gapRoleInfo_Add(gapEstLinkReqEvent_t* linkInfo);

static void gapRole_peripheralStateChangeHandler(Peripheral_States_t newState);

// for debugging
static void gapRole_abort(void);

/*********************************************************************
 * CALLBACKS
 */
void gapRole_clockHandler(UArg a0);

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @brief   Set a GAP Role parameter.
 *
 * Public function defined in peripheral.h.
 */
bStatus_t GAPRole_SetParameter(uint16_t param, uint8_t len, void *pValue, uint8 connHandle) {
	bStatus_t ret = SUCCESS;
	switch (param) {
	case GAPROLE_IRK:
		if (len == KEYLEN) {
			VOID memcpy(gapRole_IRK, pValue, KEYLEN);
		} else {
			ret = bleInvalidRange;
		}
		break;

	case GAPROLE_SRK:
		if (len == KEYLEN) {
			VOID memcpy(gapRole_SRK, pValue, KEYLEN);
		} else {
			ret = bleInvalidRange;
		}
		break;

	case GAPROLE_SIGNCOUNTER:
		if (len == sizeof(uint32_t)) {
			gapRole_signCounter = *((uint32_t*) pValue);
		} else {
			ret = bleInvalidRange;
		}
		break;

	case GAPROLE_ADVERT_ENABLED:  //connectable advertising
		if (len == sizeof(uint8_t)) {
			// Non-connectable advertising must be disabled.
			if (gapRole_AdvNonConnEnabled != TRUE) {
				uint8_t oldAdvEnabled = gapRole_AdvEnabled;
				gapRole_AdvEnabled = *((uint8_t*) pValue);

				if ((oldAdvEnabled) && (gapRole_AdvEnabled == FALSE)) {
					// Turn off advertising.
					//if ((gapRole_peripheralState == GAPROLE_ADVERTISING) || (gapRole_peripheralState == GAPROLE_WAITING_AFTER_TIMEOUT)) {
						VOID GAP_EndDiscoverable(selfEntity);
					//}
				} else if ((oldAdvEnabled == FALSE) && (gapRole_AdvEnabled)) {
					// Turn on advertising.
					if (gapRoleNumLinks(GAPROLE_AVAILABLE_LINKS)) //don't do conn adv if we don't have any avilable links
							{
						// Turn on advertising.
						//if ((gapRole_peripheralState == GAPROLE_STARTED) || (gapRole_peripheralState == GAPROLE_WAITING) || (gapRole_peripheralState == GAPROLE_WAITING_AFTER_TIMEOUT)) {
							gapRole_setEvent(START_ADVERTISING_EVT);
						//}
						//gapRole_setEvent(START_ADVERTISING_EVT);
					} else {
						return bleNoResources; // no more available links
					}
				}
			} else {
				ret = bleIncorrectMode;
			}
		} else {
			ret = bleInvalidRange;
		}
		break;

	case GAPROLE_ADV_NONCONN_ENABLED:
		if (len == sizeof(uint8_t)) {
			// Connectable advertising must be disabled.
			if (gapRole_AdvEnabled != TRUE) {
				uint8_t oldAdvEnabled = gapRole_AdvNonConnEnabled;
				gapRole_AdvNonConnEnabled = *((uint8_t*) pValue);

				if ((oldAdvEnabled) && (gapRole_AdvNonConnEnabled == FALSE)) {
					//if ((gapRole_peripheralState == GAPROLE_ADVERTISING_NONCONN) || (gapRole_peripheralState == GAPROLE_CONNECTED_ADV) || (gapRole_peripheralState == GAPROLE_WAITING_AFTER_TIMEOUT)) {
						VOID GAP_EndDiscoverable(selfEntity);
					//}
				} else if ((oldAdvEnabled == FALSE) && (gapRole_AdvNonConnEnabled)) {
					// Turn on advertising.
					//if ((gapRole_peripheralState == GAPROLE_STARTED) || (gapRole_peripheralState == GAPROLE_WAITING) || (gapRole_peripheralState == GAPROLE_CONNECTED)
					//		|| (gapRole_peripheralState == GAPROLE_WAITING_AFTER_TIMEOUT)) {
						gapRole_setEvent(START_ADVERTISING_EVT);
					//}
				}
			} else {
				ret = bleIncorrectMode;
			}
		} else {
			ret = bleInvalidRange;
		}
		break;

	case GAPROLE_ADVERT_OFF_TIME:
		if (len == sizeof(uint16_t)) {
			gapRole_AdvertOffTime = *((uint16_t*) pValue);
		} else {
			ret = bleInvalidRange;
		}
		break;

	case GAPROLE_ADVERT_DATA:
		if (len <= B_MAX_ADV_LEN) {
			VOID memset(gapRole_AdvertData, 0, B_MAX_ADV_LEN);
			VOID memcpy(gapRole_AdvertData, pValue, len);
			gapRole_AdvertDataLen = len;

			// Update the advertising data
			ret = GAP_UpdateAdvertisingData(selfEntity,
			TRUE, gapRole_AdvertDataLen, gapRole_AdvertData);
		} else {
			ret = bleInvalidRange;
		}
		break;

	case GAPROLE_SCAN_RSP_DATA:
		if (len <= B_MAX_ADV_LEN) {
			VOID memset(gapRole_ScanRspData, 0, B_MAX_ADV_LEN);
			VOID memcpy(gapRole_ScanRspData, pValue, len);
			gapRole_ScanRspDataLen = len;

			// Update the Response Data
			ret = GAP_UpdateAdvertisingData(selfEntity,
			FALSE, gapRole_ScanRspDataLen, gapRole_ScanRspData);
		} else {
			ret = bleInvalidRange;
		}
		break;

	case GAPROLE_ADV_EVENT_TYPE:
		if ((len == sizeof(uint8_t)) && (*((uint8_t*) pValue) <= GAP_ADTYPE_ADV_LDC_DIRECT_IND)) {
			gapRole_AdvEventType = *((uint8_t*) pValue);
		} else {
			ret = bleInvalidRange;
		}
		break;

	case GAPROLE_ADV_DIRECT_TYPE:

		if ((len == sizeof(uint8_t)) && (*((uint8_t*) pValue) <= ADDRMODE_PRIVATE_RESOLVE)) {
			gapRole_AdvDirectType = *((uint8_t*) pValue);
		} else {
			ret = bleInvalidRange;
		}
		break;

	case GAPROLE_ADV_DIRECT_ADDR:
		if (len == B_ADDR_LEN) {
			VOID memcpy(gapRole_AdvDirectAddr, pValue, B_ADDR_LEN);
		} else {
			ret = bleInvalidRange;
		}
		break;

	case GAPROLE_ADV_CHANNEL_MAP:
		if ((len == sizeof(uint8_t)) && (*((uint8_t*) pValue) <= 0x07)) {
			gapRole_AdvChanMap = *((uint8_t*) pValue);
		} else {
			ret = bleInvalidRange;
		}
		break;

	case GAPROLE_ADV_FILTER_POLICY:
		if ((len == sizeof(uint8_t)) && (*((uint8_t*) pValue) <= GAP_FILTER_POLICY_WHITE)) {
			gapRole_AdvFilterPolicy = *((uint8_t*) pValue);
		} else {
			ret = bleInvalidRange;
		}
		break;

	case GAPROLE_PARAM_UPDATE_ENABLE:
		if ((len == sizeof(uint8_t)) && (*((uint8_t*) pValue) <= TRUE)) {
			gapRole_ParamUpdateEnable = *((uint8_t*) pValue);
		} else {
			ret = bleInvalidRange;
		}
		break;

	case GAPROLE_MIN_CONN_INTERVAL: {
		uint16_t newInterval = *((uint16_t*) pValue);
		if (len == sizeof(uint16_t) && (newInterval >= MIN_CONN_INTERVAL) && (newInterval <= MAX_CONN_INTERVAL)) {
			gapRole_MinConnInterval = newInterval;
		} else {
			ret = bleInvalidRange;
		}
	}
		break;

	case GAPROLE_MAX_CONN_INTERVAL: {
		uint16_t newInterval = *((uint16_t*) pValue);
		if (len == sizeof(uint16_t) && (newInterval >= MIN_CONN_INTERVAL) && (newInterval <= MAX_CONN_INTERVAL)) {
			gapRole_MaxConnInterval = newInterval;
		} else {
			ret = bleInvalidRange;
		}
	}
		break;

	case GAPROLE_SLAVE_LATENCY: {
		uint16_t latency = *((uint16_t*) pValue);
		if (len == sizeof(uint16_t) && (latency < MAX_SLAVE_LATENCY)) {
			gapRole_SlaveLatency = latency;
		} else {
			ret = bleInvalidRange;
		}
	}
		break;

	case GAPROLE_TIMEOUT_MULTIPLIER: {
		uint16_t newTimeout = *((uint16_t*) pValue);
		if (len == sizeof(uint16_t) && (newTimeout >= MIN_TIMEOUT_MULTIPLIER) && (newTimeout <= MAX_TIMEOUT_MULTIPLIER)) {
			gapRole_TimeoutMultiplier = newTimeout;
		} else {
			ret = bleInvalidRange;
		}
	}
		break;

	case GAPROLE_PARAM_UPDATE_REQ: {
		uint8_t req = *((uint8_t*) pValue);
		if (len == sizeof(uint8_t) && (req == TRUE)) {
			// Make sure we don't send an L2CAP Connection Parameter Update Request
			// command within TGAP(conn_param_timeout) of an L2CAP Connection Parameter
			// Update Response being received.
			if (Util_isActive(&updateTimeoutClock) == FALSE) {
				// Start connection update procedure
				uint8 index = 0;

				while (index < MAX_NUM_BLE_CONNS && multiConnInfo[index].gapRole_ConnRole != GAP_PROFILE_PERIPHERAL
						&& multiConnInfo[index].gapRole_ConnectionHandle != GAPROLE_CONN_JUST_TERMINATED) {
					index++;
				}
				if (index >= MAX_NUM_BLE_CONNS) { //no connection as peripheral found
					ret = INVALIDPARAMETER;
					//Util_stopClock(&startUpdateClock);
				} else {
					ret = gapRole_startConnUpdate(GAPROLE_NO_ACTION, multiConnInfo[index].gapRole_ConnectionHandle);
				}
				if (ret == SUCCESS) {
					// Connection update requested by app, cancel such pending procedure (if active)
					Util_stopClock(&startUpdateClock);
				}
			} else {
				ret = blePending;
			}
		} else {
			ret = bleInvalidRange;
		}
	}
		break;

	case GAPROLE_MAX_SCAN_RES:
		if (len == sizeof(uint8_t)) {
			gapRoleMaxScanRes = *((uint8_t*) pValue);
		} else {
			ret = bleInvalidRange;
		}
		break;

	default:
		// The param value isn't part of this profile, try the GAP.
		if ((param < TGAP_PARAMID_MAX) && (len == sizeof(uint16_t))) {
			ret = GAP_SetParamValue(param, *((uint16_t*) pValue));
		} else {
			ret = INVALIDPARAMETER;
		}
		break;
	}

	return (ret);
}

/*********************************************************************
 * @brief   Get a GAP Role parameter.
 *
 * Public function defined in peripheral.h.
 */
bStatus_t GAPRole_GetParameter(uint16_t param, void *pValue, uint8_t connHandle) {
	bStatus_t ret = SUCCESS;
	uint8_t index;

	switch (param) {
	case GAPROLE_PROFILEROLE:
		*((uint8_t*) pValue) = gapRole_profileRole;
		break;

	case GAPROLE_IRK:
		VOID memcpy(pValue, gapRole_IRK, KEYLEN);
		break;

	case GAPROLE_SRK:
		VOID memcpy(pValue, gapRole_SRK, KEYLEN);
		break;

	case GAPROLE_SIGNCOUNTER:
		*((uint32_t*) pValue) = gapRole_signCounter;
		break;

	case GAPROLE_BD_ADDR:
		VOID memcpy(pValue, gapRole_bdAddr, B_ADDR_LEN);
		break;

	case GAPROLE_ADVERT_ENABLED:
		*((uint8_t*) pValue) = gapRole_AdvEnabled;
		break;

	case GAPROLE_ADV_NONCONN_ENABLED:
		*((uint8_t*) pValue) = gapRole_AdvNonConnEnabled;
		break;

	case GAPROLE_ADVERT_OFF_TIME:
		*((uint16_t*) pValue) = gapRole_AdvertOffTime;
		break;

	case GAPROLE_ADVERT_DATA:
		VOID memcpy(pValue, gapRole_AdvertData, gapRole_AdvertDataLen);
		break;

	case GAPROLE_SCAN_RSP_DATA:
		VOID memcpy(pValue, gapRole_ScanRspData, gapRole_ScanRspDataLen);
		break;

	case GAPROLE_ADV_EVENT_TYPE:
		*((uint8_t*) pValue) = gapRole_AdvEventType;
		break;

	case GAPROLE_ADV_DIRECT_TYPE:
		*((uint8_t*) pValue) = gapRole_AdvDirectType;
		break;

	case GAPROLE_ADV_DIRECT_ADDR:
		VOID memcpy(pValue, gapRole_AdvDirectAddr, B_ADDR_LEN);
		break;

	case GAPROLE_ADV_CHANNEL_MAP:
		*((uint8_t*) pValue) = gapRole_AdvChanMap;
		break;

	case GAPROLE_ADV_FILTER_POLICY:
		*((uint8_t*) pValue) = gapRole_AdvFilterPolicy;
		break;

	case GAPROLE_PARAM_UPDATE_ENABLE:
		*((uint16_t*) pValue) = gapRole_ParamUpdateEnable;
		break;

	case GAPROLE_MIN_CONN_INTERVAL:
		*((uint16_t*) pValue) = gapRole_MinConnInterval;
		break;

	case GAPROLE_MAX_CONN_INTERVAL:
		*((uint16_t*) pValue) = gapRole_MaxConnInterval;
		break;

	case GAPROLE_SLAVE_LATENCY:
		*((uint16_t*) pValue) = gapRole_SlaveLatency;
		break;

	case GAPROLE_TIMEOUT_MULTIPLIER:
		*((uint16_t*) pValue) = gapRole_TimeoutMultiplier;
		break;

	case GAPROLE_CONN_BD_ADDR:
		index = gapRoleInfo_Find(connHandle);
		if (index != 0xFF) {
			VOID memcpy(pValue, multiConnInfo[index].gapRole_devAddr,
			B_ADDR_LEN);
		} else {
			return bleNotConnected;
		}
		break;

	case GAPROLE_CONN_INTERVAL:
		index = gapRoleInfo_Find(connHandle);
		if (index != 0xFF) {
			*((uint16_t*) pValue) = multiConnInfo[index].gapRole_ConnInterval;
		} else {
			return bleNotConnected;
		}
		break;

	case GAPROLE_CONN_LATENCY:
		index = gapRoleInfo_Find(connHandle);
		if (index != 0xFF) {
			*((uint16_t*) pValue) = multiConnInfo[index].gapRole_ConnSlaveLatency;
		} else {
			return bleNotConnected;
		}
		break;

	case GAPROLE_CONN_TIMEOUT:
		index = gapRoleInfo_Find(connHandle);
		if (index != 0xFF) {
			*((uint16_t*) pValue) = multiConnInfo[index].gapRole_ConnTimeout;
		} else {
			return bleNotConnected;
		}
		break;

	case GAPROLE_MAX_SCAN_RES:
		*((uint8_t*) pValue) = gapRoleMaxScanRes;
		break;

	default:
		// The param value isn't part of this profile, try the GAP.
		if (param < TGAP_PARAMID_MAX) {
			*((uint16_t*) pValue) = GAP_GetParamValue(param);
		} else {
			ret = INVALIDPARAMETER;
		}
		break;
	}

	return (ret);
}

/*********************************************************************
 * @brief   Does the device initialization.
 *
 * Public function defined in peripheral.h.
 */
bStatus_t GAPRole_StartDevice(gapRolesCBs_t *pAppCallbacks) {
	// Clear all of the Application callbacks
	if (gapRole_peripheralState == GAPROLE_INIT) {
		if (pAppCallbacks) {
			pGapRoles_AppCGs = pAppCallbacks;
		}

		// Start the GAP
		gapRole_SetupGAP();

		return (SUCCESS);
	} else {
		return (bleAlreadyInRequestedMode);
	}
}

/*********************************************************************
 * @brief   Terminates the existing connection.
 *
 * Public function defined in peripheral.h.
 */
bStatus_t GAPRole_TerminateConnection(uint16_t connHandle) {

	uint8 connHandleIndex = gapRoleInfo_Find(connHandle);

	if (multiConnInfo[connHandleIndex].gapRole_ConnRole == GAP_PROFILE_PERIPHERAL)

		if ((gapRole_peripheralState == GAPROLE_CONNECTED) || (gapRole_peripheralState == GAPROLE_CONNECTED_ADV)) {
			return (GAP_TerminateLinkReq(selfEntity, connHandle,
			HCI_DISCONNECT_REMOTE_USER_TERM));
		} else {
			return (bleIncorrectMode);
		}

	else {
		return (GAP_TerminateLinkReq(selfEntity, connHandle,
		HCI_DISCONNECT_REMOTE_USER_TERM));
	}
}

/*********************************************************************
 * @fn      GAPRole_createTask
 *
 * @brief   Task creation function for the GAP Peripheral Role.
 *
 * @param   none
 *
 * @return  none
 */
void GAPRole_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = gapRoleTaskStack;
  taskParams.stackSize = GAPROLE_TASK_STACK_SIZE;
  taskParams.priority = GAPROLE_TASK_PRIORITY;

  Task_construct(&gapRoleTask, gapRole_taskFxn, &taskParams, NULL);
}
/*********************************************************************
 * LOCAL FUNCTION PROTOTYPES
 */

/*********************************************************************
 * @fn      gapRole_init
 *
 * @brief   Initialization function for the GAP Role Task.
 *
 * @param   none
 *
 * @return  none
 */
static void gapRole_init(void) {
	uint8_t i;

	// Register the current thread as an ICall dispatcher application
	// so that the application can send and receive messages.
	ICall_registerApp(&selfEntity, &sem);

	gapRole_peripheralState = GAPROLE_INIT;
	gapRole_peripheralStateChangeHandler(gapRole_peripheralState);

	for (i = 0; i < MAX_NUM_BLE_CONNS; i++) {
		multiConnInfo[i].gapRole_ConnectionHandle = INVALID_CONNHANDLE;
	}

	// Get link DB maximum number of connections
	linkDBNumConns = linkDB_NumConns();

	// Setup timers as one-shot timers
	Util_constructClock(&startAdvClock, gapRole_clockHandler, 0, 0, false,
	START_ADVERTISING_EVT);
	Util_constructClock(&startUpdateClock, gapRole_clockHandler, 0, 0, false,
	START_CONN_UPDATE_EVT);
	Util_constructClock(&updateTimeoutClock, gapRole_clockHandler, 0, 0, false,
	CONN_PARAM_TIMEOUT_EVT);

	// Initialize the Profile Advertising and Connection Parameters
	gapRole_profileRole = GAP_PROFILE_PERIPHERAL | GAP_PROFILE_OBSERVER;
	VOID memset(gapRole_IRK, 0, KEYLEN);
	VOID memset(gapRole_SRK, 0, KEYLEN);
	gapRole_signCounter = 0;
	gapRole_AdvEventType = GAP_ADTYPE_ADV_IND;
	gapRole_AdvDirectType = ADDRTYPE_PUBLIC;
	gapRole_AdvChanMap = GAP_ADVCHAN_ALL;
	gapRole_AdvFilterPolicy = GAP_FILTER_POLICY_ALL;

	// Restore Items from NV
	VOID osal_snv_read(BLE_NVID_IRK, KEYLEN, gapRole_IRK);
	VOID osal_snv_read(BLE_NVID_CSRK, KEYLEN, gapRole_SRK);
	VOID osal_snv_read(BLE_NVID_SIGNCOUNTER, sizeof(uint32_t), &gapRole_signCounter);
}

/**
 * @brief   Terminate a link.
 *
 * Public function defined in central.h.
 */
bStatus_t GAPRole_TerminateLink(uint16_t connHandle) {
	return GAP_TerminateLinkReq(selfEntity, connHandle,
	HCI_DISCONNECT_REMOTE_USER_TERM);
}

/**
 * @brief   Establish a link to a peer device.
 *
 * Public function defined in central.h.
 */
bStatus_t GAPRole_EstablishLink(uint8_t highDutyCycle, uint8_t whiteList, uint8_t addrTypePeer, uint8_t *peerAddr) {
	gapEstLinkReq_t params;

	params.taskID = ICall_getLocalMsgEntityId(ICALL_SERVICE_CLASS_BLE_MSG, selfEntity);
	params.highDutyCycle = highDutyCycle;
	params.whiteList = whiteList;
	params.addrTypePeer = addrTypePeer;
	VOID memcpy(params.peerAddr, peerAddr, B_ADDR_LEN);

	return GAP_EstablishLinkReq(&params);
}

/**
 * @brief   Start a device discovery scan.
 *
 * Public function defined in central.h.
 */
bStatus_t GAPRole_StartDiscovery(uint8_t mode, uint8_t activeScan, uint8_t whiteList) {
	gapDevDiscReq_t params;

	params.taskID = ICall_getLocalMsgEntityId(ICALL_SERVICE_CLASS_BLE_MSG, selfEntity);
	params.mode = mode;
	params.activeScan = activeScan;
	params.whiteList = whiteList;

	return GAP_DeviceDiscoveryRequest(&params);
}

/**
 * @brief   Cancel a device discovery scan.
 *
 * Public function defined in central.h.
 */
bStatus_t GAPRole_CancelDiscovery(void) {
	return GAP_DeviceDiscoveryCancel(selfEntity);
}

/*********************************************************************
 * @fn      gapRole_taskFxn
 *
 * @brief   Task entry point for the GAP Peripheral Role.
 *
 * @param   a0 - first argument
 * @param   a1 - second argument
 *
 * @return  none
 */
static void gapRole_taskFxn(UArg a0, UArg a1) {
	// Initialize profile
	gapRole_init();

	// Profile main loop
	for (;;) {
		// Waits for a signal to the semaphore associated with the calling thread.
		// Note that the semaphore associated with a thread is signaled when a
		// message is queued to the message receive queue of the thread or when
		// ICall_signal() function is called onto the semaphore.
		ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);

		if (errno == ICALL_ERRNO_SUCCESS) {
			ICall_EntityID dest;
			ICall_ServiceEnum src;
			ICall_HciExtEvt *pMsg = NULL;

			if (ICall_fetchServiceMsg(&src, &dest, (void **) &pMsg) == ICALL_ERRNO_SUCCESS) {
				uint8_t safeToDealloc = TRUE;

				if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity)) {
					ICall_Stack_Event *pEvt = (ICall_Stack_Event *) pMsg;

					// Check for BLE stack events first
					if (pEvt->signature == 0xffff) {
						if (pEvt->event_flag & GAP_EVENT_SIGN_COUNTER_CHANGED) {
							// Sign counter changed, save it to NV
							VOID osal_snv_write(BLE_NVID_SIGNCOUNTER, sizeof(uint32_t), &gapRole_signCounter);
						}
					} else {
						// Process inter-task message
						safeToDealloc = gapRole_processStackMsg((ICall_Hdr *) pMsg);
					}
				}

				if (pMsg && safeToDealloc) {
					ICall_freeMsg(pMsg);
				}
			}
		}

		if (events & START_ADVERTISING_EVT) {
			events &= ~START_ADVERTISING_EVT;

			if (gapRole_AdvEnabled || gapRole_AdvNonConnEnabled) {
				gapAdvertisingParams_t params;

				// Setup advertisement parameters
				if (gapRole_AdvNonConnEnabled) {
					// Only advertise non-connectable undirected.
					params.eventType = GAP_ADTYPE_ADV_NONCONN_IND;
				} else {
					params.eventType = gapRole_AdvEventType;
					params.initiatorAddrType = gapRole_AdvDirectType;
					VOID memcpy(params.initiatorAddr, gapRole_AdvDirectAddr,
					B_ADDR_LEN);
				}

				params.channelMap = gapRole_AdvChanMap;
				params.filterPolicy = gapRole_AdvFilterPolicy;

				if (GAP_MakeDiscoverable(selfEntity, &params) != SUCCESS) {
					gapRole_peripheralState = GAPROLE_ERROR;
					gapRole_peripheralStateChangeHandler(gapRole_peripheralState);
					//gapRole_abort();
				}
			}
		}
		if (events & START_CONN_UPDATE_EVT) {
			events &= ~START_CONN_UPDATE_EVT;

			// Start connection update procedure
			uint8 index = 0;

			while (index < MAX_NUM_BLE_CONNS && multiConnInfo[index].gapRole_ConnRole != GAP_PROFILE_PERIPHERAL
					&& multiConnInfo[index].gapRole_ConnectionHandle != GAPROLE_CONN_JUST_TERMINATED) {
				index++;
			}

			if (index >= MAX_NUM_BLE_CONNS) { //no connection as peripheral found
				//INVALIDPARAMETER;
				//Util_stopClock(&startUpdateClock);
			} else {
				gapRole_startConnUpdate(GAPROLE_NO_ACTION, multiConnInfo[index].gapRole_ConnectionHandle);
			}
		}

		if (events & CONN_PARAM_TIMEOUT_EVT) {
			events &= ~CONN_PARAM_TIMEOUT_EVT;

			// Unsuccessful in updating connection parameters
			gapRole_HandleParamUpdateNoSuccess();
		}
	} // for
}

/*********************************************************************
 * @fn      gapRole_processStackMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static uint8_t gapRole_processStackMsg(ICall_Hdr *pMsg) {
	uint8_t safeToDealloc = TRUE;

	switch (pMsg->event) {
	case GAP_MSG_EVENT:
		safeToDealloc = gapRole_processGAPMsg((gapEventHdr_t *) pMsg);
		break;

	case L2CAP_SIGNAL_EVENT: {
		l2capSignalEvent_t *pPkt = (l2capSignalEvent_t *) pMsg;

		// Process the Parameter Update Response
		if (pPkt->opcode == L2CAP_PARAM_UPDATE_RSP) {
			l2capParamUpdateRsp_t *pRsp = (l2capParamUpdateRsp_t *) &(pPkt->cmd.updateRsp);

			if ((pRsp->result == L2CAP_CONN_PARAMS_REJECTED) && (paramUpdateNoSuccessOption == GAPROLE_TERMINATE_LINK)) {
				// Cancel connection param update timeout timer
				Util_stopClock(&updateTimeoutClock);

				//TODO:CHECK, this should not be commented
				// Terminate connection immediately
				//GAPRole_TerminateConnection();
			} else {
				uint16_t timeout = GAP_GetParamValue(TGAP_CONN_PARAM_TIMEOUT);

				// Let's wait for Controller to update connection parameters if they're
				// accepted. Otherwise, decide what to do based on no success option.
				Util_restartClock(&updateTimeoutClock, timeout);
			}
		}
	}
		break;
	default:
		break;
	}

	return (safeToDealloc);
}

/*********************************************************************
 * @fn      gapRole_processGAPMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static uint8_t gapRole_processGAPMsg(gapEventHdr_t *pMsg) {
	uint8_t notify = FALSE;   // State changed notify the app? (default no)

	switch (pMsg->opcode) {
	case GAP_DEVICE_INIT_DONE_EVENT: {
		gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t *) pMsg;
		bStatus_t stat = pPkt->hdr.status;

		if (stat == SUCCESS) {
			// Save off the generated keys
			VOID osal_snv_write(BLE_NVID_IRK, KEYLEN, gapRole_IRK);
			VOID osal_snv_write(BLE_NVID_CSRK, KEYLEN, gapRole_SRK);

			// Save off the information
			VOID memcpy(gapRole_bdAddr, pPkt->devAddr, B_ADDR_LEN);

			gapRole_peripheralState = GAPROLE_STARTED;
			gapRole_peripheralStateChangeHandler(gapRole_peripheralState);

			// Update the advertising data
			stat = GAP_UpdateAdvertisingData(selfEntity,
			TRUE, gapRole_AdvertDataLen, gapRole_AdvertData);
		}

		if (stat != SUCCESS) {
			gapRole_peripheralState = GAPROLE_ERROR;
			gapRole_peripheralStateChangeHandler(gapRole_peripheralState);
			//gapRole_abort();
		}

		notify = TRUE;
	}
		break;

	case GAP_ADV_DATA_UPDATE_DONE_EVENT: {
		gapAdvDataUpdateEvent_t *pPkt = (gapAdvDataUpdateEvent_t *) pMsg;

        if (pPkt->hdr.status == SUCCESS)
        {
          if (pPkt->adType)
          {
            // Setup the Response Data
            pPkt->hdr.status = GAP_UpdateAdvertisingData(selfEntity,
                              FALSE, gapRole_ScanRspDataLen, gapRole_ScanRspData);
          }
          else if ((gapRole_peripheralState != GAPROLE_ADVERTISING)   &&
                   (gapRole_peripheralState != GAPROLE_CONNECTED_ADV) &&
                   (gapRole_peripheralState != GAPROLE_CONNECTED ||
                    gapRole_AdvNonConnEnabled == TRUE)      &&
                   (Util_isActive(&startAdvClock) == FALSE))
          {
            // Start advertising
            gapRole_setEvent(START_ADVERTISING_EVT);
          }
          notify = FALSE;
        }else {
			// Set into Error state
			gapRole_peripheralState = GAPROLE_ERROR;
			gapRole_peripheralStateChangeHandler(gapRole_peripheralState);
			//gapRole_abort();
			notify = TRUE;
		}
	}
		break;

	case GAP_MAKE_DISCOVERABLE_DONE_EVENT:
	case GAP_END_DISCOVERABLE_DONE_EVENT: {
		gapMakeDiscoverableRspEvent_t *pPkt = (gapMakeDiscoverableRspEvent_t *) pMsg;

		if (pPkt->hdr.status == SUCCESS) {
			if (pMsg->opcode == GAP_MAKE_DISCOVERABLE_DONE_EVENT) {
				if (gapRole_peripheralState == GAPROLE_CONNECTED) {
					gapRole_peripheralState = GAPROLE_CONNECTED_ADV;
					gapRole_peripheralStateChangeHandler(gapRole_peripheralState);
				} else if (gapRole_AdvEnabled) {
					gapRole_peripheralState = GAPROLE_ADVERTISING;
					gapRole_peripheralStateChangeHandler(gapRole_peripheralState);
				} else {
					gapRole_peripheralState = GAPROLE_ADVERTISING_NONCONN;
					gapRole_peripheralStateChangeHandler(gapRole_peripheralState);
				}
			} else // GAP_END_DISCOVERABLE_DONE_EVENT
			{
				if (gapRole_AdvertOffTime != 0) //restart advertising if param is set
						{
					if ((gapRole_AdvEnabled) || (gapRole_AdvNonConnEnabled)) {
						Util_restartClock(&startAdvClock, gapRole_AdvertOffTime);
					}
				} else {
					// Since gapRole_AdvertOffTime is set to 0, the device should not
					// automatically become discoverable again after a period of time.
					// Set enabler to FALSE; device will become discoverable again when
					// this value gets set to TRUE
					if (gapRole_AdvEnabled == TRUE) {
						gapRole_AdvEnabled = FALSE;
					} else {
						gapRole_AdvNonConnEnabled = FALSE;
					}
				}

				// Update state.
				if (gapRole_peripheralState == GAPROLE_CONNECTED_ADV) {
					// In the Advertising Off period
					gapRole_peripheralState = GAPROLE_CONNECTED;
					gapRole_peripheralStateChangeHandler(gapRole_peripheralState);
				} else {
					// In the Advertising Off period
					gapRole_peripheralState = GAPROLE_WAITING;
					gapRole_peripheralStateChangeHandler(gapRole_peripheralState);
				}
			}
			notify = TRUE;
		} else if (pPkt->hdr.status == LL_STATUS_ERROR_COMMAND_DISALLOWED) //we're already advertising
		{
			notify = FALSE;
		} else {
			gapRole_peripheralState = GAPROLE_ERROR;
			gapRole_peripheralStateChangeHandler(gapRole_peripheralState);
			//gapRole_abort();
		}
	}
		break;

	case GAP_LINK_ESTABLISHED_EVENT: {
		gapEstLinkReqEvent_t *pPkt = (gapEstLinkReqEvent_t *) pMsg;

		if (pPkt->hdr.status == SUCCESS) {
			//add to database
			gapRoleInfo_Add(pPkt);

			// advertising will stop when a connection forms in the peripheral role
			if (pPkt->connRole == GAP_PROFILE_PERIPHERAL) {

				gapRole_peripheralState = GAPROLE_CONNECTED;
				gapRole_peripheralStateChangeHandler(gapRole_peripheralState);

				// Check whether update parameter request is enabled
				if (gapRole_ParamUpdateEnable == TRUE) {
						// Get the minimum time upon connection establishment before the
						// peripheral can start a connection update procedure.
						uint16_t timeout = GAP_GetParamValue(
						TGAP_CONN_PAUSE_PERIPHERAL);

						Util_restartClock(&startUpdateClock, timeout * 1000);

				}
			}
			// Notify the Bond Manager to the connection
			VOID GAPBondMgr_LinkEst(pPkt->devAddrType, pPkt->devAddr, pPkt->connectionHandle, pPkt->connRole);
		} else if (pPkt->hdr.status == bleGAPConnNotAcceptable) {
			// Set enabler to FALSE; device will become discoverable again when
			// this value gets set to TRUE
			gapRole_AdvEnabled = FALSE;

	        // Go to WAITING state, and then start advertising
			if (pPkt->connRole == GAP_PROFILE_PERIPHERAL) {
				gapRole_peripheralState = GAPROLE_WAITING;
				gapRole_peripheralStateChangeHandler(gapRole_peripheralState);
			}
		} else {
			if (pPkt->connRole == GAP_PROFILE_PERIPHERAL) {
				gapRole_peripheralState = GAPROLE_ERROR;
				gapRole_peripheralStateChangeHandler(gapRole_peripheralState);
			} else {
				gapRole_abort();
			}
		}
		notify = TRUE;
	}
		break;

	case GAP_LINK_TERMINATED_EVENT: {
		gapTerminateLinkEvent_t *pPkt = (gapTerminateLinkEvent_t *) pMsg;

		// notify bond manager
		GAPBondMgr_LinkTerm(pPkt->connectionHandle);

		// Erase connection information (maybe make this a function)
		uint8 connHandleIndex = gapRoleInfo_Find(pPkt->connectionHandle);
		multiConnInfo[connHandleIndex].gapRole_ConnectionHandle = INVALID_CONNHANDLE;
		multiConnInfo[connHandleIndex].gapRole_ConnInterval = 0;
		multiConnInfo[connHandleIndex].gapRole_ConnSlaveLatency = 0;
		multiConnInfo[connHandleIndex].gapRole_ConnTimeout = 0;

		// Cancel all connection parameter update timers (if any active)
		Util_stopClock(&startUpdateClock);
		Util_stopClock(&updateTimeoutClock);

		notify = TRUE;

		if (multiConnInfo[connHandleIndex].gapRole_ConnRole == GAP_PROFILE_PERIPHERAL) {
			// If device was advertising when connection dropped
			if (gapRole_AdvNonConnEnabled) {
				// Continue advertising.
				gapRole_peripheralState = GAPROLE_ADVERTISING_NONCONN;
				gapRole_peripheralStateChangeHandler(gapRole_peripheralState);
			}
			// Else go to WAITING state.
			else {
				if (pPkt->reason == LL_SUPERVISION_TIMEOUT_TERM) {
					gapRole_peripheralState = GAPROLE_WAITING_AFTER_TIMEOUT;
					gapRole_peripheralStateChangeHandler(gapRole_peripheralState);
				} else {
					gapRole_peripheralState = GAPROLE_WAITING;
					gapRole_peripheralStateChangeHandler(gapRole_peripheralState);
				}

				// Start advertising, if enabled.
				gapRole_setEvent(START_ADVERTISING_EVT);
			}
		}
	}
		break;

	case GAP_LINK_PARAM_UPDATE_EVENT: {
		gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *) pMsg;

		// Cancel connection param update timeout timer (if active)
		Util_stopClock(&updateTimeoutClock);

		if (pPkt->hdr.status == SUCCESS) {
			// Store new connection parameters
//              gapRole_ConnInterval = pPkt->connInterval;
//              gapRole_ConnSlaveLatency = pPkt->connLatency;
//              gapRole_ConnTimeout = pPkt->connTimeout;

			// Make sure there's no pending connection update procedure
			if (Util_isActive(&startUpdateClock) == FALSE) {
//                // Notify the application with the new connection parameters
//                if (pGapRoles_ParamUpdateCB != NULL)
//                {
//                  (*pGapRoles_ParamUpdateCB)(gapRole_ConnInterval,
//                                             gapRole_ConnSlaveLatency,
//                                             gapRole_ConnTimeout);
//                }
			}
		}
		notify = TRUE;
	}
		break;

	case GAP_PAIRING_REQ_EVENT: {
		gapPairingReqEvent_t *pPkt = (gapPairingReqEvent_t *) pMsg;

		// Send Pairing Failed Response
		VOID GAP_TerminateAuth(pPkt->connectionHandle,
		SMP_PAIRING_FAILED_NOT_SUPPORTED);
	}
		break;

	case GAP_SLAVE_REQUESTED_SECURITY_EVENT: {
        uint16_t connHandle = ((gapSlaveSecurityReqEvent_t *)pMsg)->connectionHandle;
        uint8_t authReq = ((gapSlaveSecurityReqEvent_t *)pMsg)->authReq;

        GAPBondMgr_SlaveReqSecurity(connHandle, authReq);
	}
		break;

	case GAP_DEVICE_INFO_EVENT:
	case GAP_DEVICE_DISCOVERY_EVENT:
		notify = TRUE;
		break;

	default:
		notify = FALSE;
		break;
	}

	if (notify == TRUE) //app needs to take further action
	{
		if (pGapRoles_AppCGs && pGapRoles_AppCGs->pfnPassThrough) {
			return (pGapRoles_AppCGs->pfnPassThrough((gapMultiRoleEvent_t *) pMsg));
		}
	}

	return TRUE;

}

/*********************************************************************
 * @fn      gapRole_SetupGAP
 *
 * @brief   Call the GAP Device Initialization function using the
 *          Profile Parameters.
 *
 * @param   none
 *
 * @return  none
 */
static void gapRole_SetupGAP(void) {
	VOID GAP_DeviceInit(selfEntity, gapRole_profileRole, gapRoleMaxScanRes, gapRole_IRK, gapRole_SRK, (uint32*) &gapRole_signCounter);
}

/*********************************************************************
 * @fn      gapRole_HandleParamUpdateNoSuccess
 *
 * @brief   Handle unsuccessful connection parameters update.
 *
 * @param   none
 *
 * @return  none
 */
static void gapRole_HandleParamUpdateNoSuccess(void) {
	// See which option was chosen for unsuccessful updates
	switch (paramUpdateNoSuccessOption) {
	//TODO:CHECK, this should not be commented
//    case GAPROLE_RESEND_PARAM_UPDATE:
//      GAPRole_SendUpdateParam(gapRole_MinConnInterval, gapRole_MaxConnInterval,
//                              gapRole_SlaveLatency, gapRole_TimeoutMultiplier,
//                              GAPROLE_RESEND_PARAM_UPDATE);
//      break;
//
//    case GAPROLE_TERMINATE_LINK:
//      GAPRole_TerminateConnection();
//      break;

	case GAPROLE_NO_ACTION:
		// fall through
	default:
		//do nothing
		break;
	}
}

/********************************************************************
 * @fn          gapRole_startConnUpdate
 *
 * @brief       Start the connection update procedure
 *
 * @param       handleFailure - what to do if the update does not occur.
 *              Method may choose to terminate connection, try again,
 *              or take no action
 *
 * @return      none
 */
static bStatus_t gapRole_startConnUpdate(uint8_t handleFailure, uint8 connHandle) {
	bStatus_t status;
	uint8 connHandleIndex = gapRoleInfo_Find(connHandle);

	//update only connection to master, not those to slave
	if (multiConnInfo[connHandleIndex].gapRole_ConnRole != GAP_PROFILE_PERIPHERAL) {
		return (bleNotConnected);
	}
	// If there is no existing connection no update need be sent
	if (multiConnInfo[connHandleIndex].gapRole_ConnectionHandle == GAPROLE_CONN_JUST_TERMINATED || multiConnInfo[connHandleIndex].gapRole_ConnectionHandle == INVALID_CONNHANDLE) {
		return (bleNotConnected);
	}

	// First check the current connection parameters versus the configured parameters
//	if ((multiConnInfo[connHandleIndex].gapRole_ConnInterval < gapRole_MinConnInterval) || (multiConnInfo[connHandleIndex].gapRole_ConnInterval > gapRole_MaxConnInterval)
//			|| (multiConnInfo[connHandleIndex].gapRole_ConnSlaveLatency != gapRole_SlaveLatency)
//			|| (multiConnInfo[connHandleIndex].gapRole_ConnTimeout != gapRole_TimeoutMultiplier)) {
	if (gapRole_MinConnInterval > 6 && gapRole_MinConnInterval < 3200){

		//TODO: do some test to check if parameters are within specification of ble

		uint16_t timeout = GAP_GetParamValue(TGAP_CONN_PARAM_TIMEOUT);
#if defined(L2CAP_CONN_UPDATE)
		l2capParamUpdateReq_t updateReq;

		updateReq.intervalMin = gapRole_MinConnInterval;
		updateReq.intervalMax = gapRole_MaxConnInterval;
		updateReq.slaveLatency = gapRole_SlaveLatency;
		updateReq.timeoutMultiplier = gapRole_TimeoutMultiplier;

		status = L2CAP_ConnParamUpdateReq(connHandle, &updateReq, selfEntity);
#else
		gapUpdateLinkParamReq_t linkParams;

		linkParams.connectionHandle = connHandle;
		linkParams.intervalMin = gapRole_MinConnInterval;
		linkParams.intervalMax = gapRole_MaxConnInterval;
		linkParams.connLatency = gapRole_SlaveLatency;
		linkParams.connTimeout = gapRole_TimeoutMultiplier;

		status = GAP_UpdateLinkParamReq(&linkParams);
#endif // L2CAP_CONN_UPDATE

		if (status == SUCCESS) {
			paramUpdateNoSuccessOption = handleFailure;
			// Let's wait either for L2CAP Connection Parameters Update Response or
			// for Controller to update connection parameters
			Util_restartClock(&updateTimeoutClock, timeout);
		}
	} else {
		status = bleInvalidRange;
	}

	return status;
}

/********************************************************************
 * @fn          GAPRole_SendUpdateParam
 *
 * @brief       Update the parameters of an existing connection
 *
 * @param       minConnInterval - the new min connection interval
 * @param       maxConnInterval - the new max connection interval
 * @param       latency - the new slave latency
 * @param       connTimeout - the new timeout value
 * @param       handleFailure - what to do if the update does not occur.
 *              Method may choose to terminate connection, try again,
 *              or take no action
 *
 * @return      SUCCESS, bleNotConnected, or bleInvalidRange
 */
bStatus_t GAPRole_SendUpdateParam(uint16_t minConnInterval, uint16_t maxConnInterval, uint16_t latency, uint16_t connTimeout, uint8_t handleFailure, uint16_t connHandle) {

	uint8 connHandleIndex = gapRoleInfo_Find(connHandle);

	// If there is no existing connection no update need be sent
	if (multiConnInfo[connHandleIndex].gapRole_ConnectionHandle == GAPROLE_CONN_JUST_TERMINATED || multiConnInfo[connHandleIndex].gapRole_ConnectionHandle == INVALID_CONNHANDLE) {
		return (bleNotConnected);
	}

	if (multiConnInfo[connHandleIndex].gapRole_ConnRole != GAP_PROFILE_PERIPHERAL) {
		return (bleNotConnected);
	}

	  // If there is no existing connection no update need be sent
	if (gapRole_peripheralState != GAPROLE_CONNECTED) {
		return (bleNotConnected);
	}

	// Check that all parameters are in range before sending request
	if ((minConnInterval >= DEFAULT_MIN_CONN_INTERVAL) && (minConnInterval < DEFAULT_MAX_CONN_INTERVAL) && (maxConnInterval >= DEFAULT_MIN_CONN_INTERVAL)
			&& (maxConnInterval < DEFAULT_MAX_CONN_INTERVAL) && (latency < MAX_SLAVE_LATENCY) && (connTimeout >= MIN_TIMEOUT_MULTIPLIER)
			&& (connTimeout < MAX_TIMEOUT_MULTIPLIER)) {
		gapRole_MinConnInterval = minConnInterval;
		gapRole_MaxConnInterval = maxConnInterval;
		gapRole_SlaveLatency = latency;
		gapRole_TimeoutMultiplier = connTimeout;

		// Start connection update procedure
		VOID gapRole_startConnUpdate(handleFailure, multiConnInfo[connHandleIndex].gapRole_ConnectionHandle);

		// Connection update requested by app, cancel such pending procedure (if active)
		Util_stopClock(&startUpdateClock);

		return (SUCCESS);
	}

	return (bleInvalidRange);
}

/*********************************************************************
 * @fn      gapRole_setEvent
 *
 * @brief   Set an event
 *
 * @param   event - event to be set
 *
 * @return  none
 */
static void gapRole_setEvent(uint32_t event) {
	events |= event;

	// Wake up the application thread when it waits for clock event
	Semaphore_post(sem);
}

/*********************************************************************
 * @fn      gapRole_clockHandler
 *
 * @brief   Clock handler function
 *
 * @param   a0 - event
 *
 * @return  none
 */
void gapRole_clockHandler(UArg a0) {
	gapRole_setEvent(a0);
}

/*********************************************************************
 * @fn          gapRoleInfo_Find
 *
 * @brief       Find the connection handle's index.  Uses the connection handle to search
 *              the gapRoleInfo database for the indx.
 *
 * @param       connectionHandle - controller link connection handle.
 *
 * @return      index of the found item, 0xFF if not found
 */
uint8_t gapRoleInfo_Find(uint16 connectionHandle) {
	uint8_t x;
	// Find link record
	for (x = 0; x < MAX_NUM_BLE_CONNS; x++) {
		if (multiConnInfo[x].gapRole_ConnectionHandle == connectionHandle) {
			// Found
			return (x);
		}
	}

	// Not Found!!
	return (0xFF);
}

/*********************************************************************
 *********************************************************************/
/*********************************************************************
 * @fn          gapRoleInfo_Add
 *
 * @brief       Adds a record to the link database.
 *
 * @param       taskID - Application task ID
 * @param       connectionHandle - new record connection handle
 * @param       newState - starting connection state
 * @param       addrType - new address type
 * @param       pAddr - new address
 * @param       connRole - connection formed as Master or Slave
 * @param       connInterval - connection's communications interval (n * 1.23 ms)
 * @param       MTU - connection's MTU size
 *
 * @return      SUCCESS if successful
 *              bleIncorrectMode - hasn't been initialized.
 *              bleNoResources - table full
 *              bleAlreadyInRequestedMode - already exist connectionHandle
 *
 */
uint8 gapRoleInfo_Add(gapEstLinkReqEvent_t* linkInfo) {
	// Don't need to check to ensure link doesn't exist. This is handled by the stack
	uint8_t openIndex = gapRoleInfo_Find( INVALID_CONNHANDLE); //try to find open link index (identified by INVALID_CONNHANDLE)
	if (openIndex < MAX_NUM_BLE_CONNS) {
		// Copy link info (that isn't stored in linkdb)
		multiConnInfo[openIndex].gapRole_ConnectionHandle = linkInfo->connectionHandle;
		multiConnInfo[openIndex].gapRole_ConnSlaveLatency = linkInfo->connLatency;
		multiConnInfo[openIndex].gapRole_ConnTimeout = linkInfo->connTimeout;
		multiConnInfo[openIndex].gapRole_ConnInterval = linkInfo->connInterval;
		multiConnInfo[openIndex].gapRole_ConnRole = linkInfo->connRole;
		VOID memcpy(multiConnInfo[openIndex].gapRole_devAddr, linkInfo->devAddr,
		B_ADDR_LEN);
		return ( SUCCESS);
	} else {
		// Table is full
		return ( bleNoResources);
	}
}

/*********************************************************************
 * @fn          gapRoleInfo_Find
 *
 * @brief       Find the connection handle's index.  Uses the connection handle to search
 *              the gapRoleInfo database for the indx.
 *
 * @param       connectionHandle - controller link connection handle.
 *
 * @return      index of the found item, 0xFF if not found
 */
uint8_t gapRoleNumLinks(uint8_t linkType) {
	uint8_t activeLinks = 0;
	// Find link record
	uint8_t x;
	for (x = 0; x < MAX_NUM_BLE_CONNS; x++) {
		if (multiConnInfo[x].gapRole_ConnectionHandle != INVALID_CONNHANDLE) {
			// Found
			activeLinks++;
		}
	}

	if (linkType == GAPROLE_ACTIVE_LINKS) {
		return (activeLinks);
	} else if (linkType == GAPROLE_AVAILABLE_LINKS) {
		return (MAX_NUM_BLE_CONNS - activeLinks);
	} else {
		return bleInvalidRange;
	}
}

static void gapRole_peripheralStateChangeHandler(Peripheral_States_t newState) {
//	uint8_t advertEnabled;
//	uint8_t old_advertEnabled;
	switch (newState) {
	case GAPROLE_INIT: //!< Waiting to be started

		break;
	case GAPROLE_STARTED: //!< Started but not advertising

		break;
	case GAPROLE_ADVERTISING:  //!< Currently Advertising

		break;
	case GAPROLE_ADVERTISING_NONCONN: //!< Currently using non-connectable Advertising
		// Disable connectable advertising.
//		GAPRole_GetParameter(GAPROLE_ADV_NONCONN_ENABLED, &old_advertEnabled, NULL);
//		advertEnabled = FALSE;
//		GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t), &advertEnabled, NULL);
//
//		// Enable connectable advertising.
//		GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &old_advertEnabled, NULL);

		break;
	case GAPROLE_WAITING: //!< Device is started but not advertising, is in waiting period before advertising again

		break;
	case GAPROLE_WAITING_AFTER_TIMEOUT: //!< Device just timed out from a connection but is not yet advertising, is in waiting period before advertising again

		break;
	case GAPROLE_CONNECTED:  //!< In a connection
		// Disable connectable advertising.
//		GAPRole_GetParameter(GAPROLE_ADVERT_ENABLED, &old_advertEnabled, NULL);
//		advertEnabled = FALSE;
//		GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advertEnabled, NULL);
//
//		// Enable non-connectable advertising.
//		GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t), &old_advertEnabled, NULL);

		break;
	case GAPROLE_CONNECTED_ADV: //!< In a connection + advertising

		break;
	case GAPROLE_ERROR:
		gapRole_abort();
		break;
	default:
		break;
	}

}

static void gapRole_abort(void) {
#ifdef GAP_ROLE_ABORT  
	while(1);
#endif  
}
