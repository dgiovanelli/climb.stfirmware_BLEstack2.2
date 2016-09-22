/******************************************************************************
 * @file  simple_ble_topology.c
 *
 * @description Application task for the Simple Topology example
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

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "hci_tl.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp_giova.h"
#include "devinfoservice.h"
#include "ClimbProfile.h"

#include "multi_giova.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
//#include "ICallBleAPIMSG.h"
#include "icall_apimsg.h"

#include "util.h"
//#include "board_lcd.h"
//#include "board_key.h"
#include "Board.h"

#include "linkdb.h"
#include "climb_child_app.h"
#include "Keys_Task.h"

#ifdef INCLUDE_BATTERY_SERVICE
#include "battservice.h"
#endif
#if defined(FEATURE_OAD) || defined(IMAGE_INVALIDATE)
#include "oad_target.h"
#include "oad.h"
#endif //FEATURE_OAD || IMAGE_INVALIDATE

#include <xdc/runtime/Types.h>
#include <ti/sysbios/BIOS.h>

#include "hal_mcu.h"
#include <ti/mw/display/Display.h>
#include <ti/mw/lcd/LCDDogm1286.h>
#include <ti/mw/sensors/SensorI2C.h>
#include <ti/mw/sensors/SensorTmp007.h>
#include <ti/mw/sensors/SensorHdc1000.h>
#include <ti/mw/sensors/SensorBmp280.h>
#include <ti/mw/sensors/SensorOpt3001.h>
#include <ti/mw/sensors/SensorMpu9250.h>

#include <driverlib/aon_batmon.h>

#ifdef PRINTF_ENABLED
#include <xdc/runtime/System.h>
#endif
/*********************************************************************
 * CONSTANTS
 */
// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#ifdef HIGH_PERFORMANCE
#define DEFAULT_CONNECTABLE_ADVERTISING_INTERVAL          240
#define DEFAULT_NON_CONNECTABLE_ADVERTISING_INTERVAL          240
#else
#define DEFAULT_CONNECTABLE_ADVERTISING_INTERVAL          1600
#define DEFAULT_NON_CONNECTABLE_ADVERTISING_INTERVAL          DEFAULT_CONNECTABLE_ADVERTISING_INTERVAL
#endif

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL   	 6

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     12

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter
// update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          210

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         10

#ifdef HIGH_PERFORMANCE
// Scan interval value in 0.625ms ticks
#define SCAN_INTERVAL 						  80
// scan window value in 0.625ms ticks
#define SCAN_WINDOW							  80
#else
// Scan interval value in 0.625ms ticks
#define SCAN_INTERVAL 						  80
// scan window value in 0.625ms ticks
#define SCAN_WINDOW							  80
#endif

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 1250//10000 //Tempo di durata di una scansione,

#define PRE_ADV_TIMEOUT				 		  DEFAULT_NON_CONNECTABLE_ADVERTISING_INTERVAL*0.625-10 //DEFAULT_ADVERTISING_INTERVAL*0.625-5

// Whether to report all contacts or only the first for each device
#define FILTER_ADV_REPORTS					  FALSE

// Discovery mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL //non è ancora chiaro cosa cambi, con le altre due opzioni non vede

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         FALSE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// How often to perform periodic event (in msec)
#define PERIODIC_EVT_PERIOD_MSEC               2000

#define NODE_TIMEOUT_OS_TICKS				500000

#define LED_TIMEOUT_MSEC						  	  10

#warning CHECK TIMEOUTS!!!
#define CONNECTABLE_TIMEOUT_MSEC				1000*60
#define WAKEUP_DEFAULT_TIMEOUT_SEC				60*60*24
#define GOTOSLEEP_DEFAULT_TIMEOUT_SEC			2*60*60
#define GOING_TO_SLEEP_DELAY_AFTER_REQUEST_MSEC 1000*60

#define NODE_ID								  0x00

#define CHILD_NODE_ID_LENGTH				  1
#define MASTER_NODE_ID_LENGTH				  6

#define MAX_SUPPORTED_CHILD_NODES			  10
#define MAX_SUPPORTED_MASTER_NODES			  10
#if MAX_SUPPORTED_CHILD_NODES+MAX_SUPPORTED_MASTER_NODES < 80
#warning MAX_SUPPORTED_CHILD_NODES is low because of debugging pourposes, it can be set to 90
#endif

// Maximum number of scan responses to be reported to application
#define DEFAULT_MAX_SCAN_RES                  MAX_SUPPORTED_CHILD_NODES+MAX_SUPPORTED_MASTER_NODES

#define ADV_PKT_ID_OFFSET					  12
#define ADV_PKT_STATE_OFFSET				  ADV_PKT_ID_OFFSET + CHILD_NODE_ID_LENGTH

#define BROADCAST_MSG_TYPE_STATE_UPDATE_CMD	  0x01
#define BROADCAST_MSG_LENGTH_STATE_UPDATE_CMD 0x02

#define BROADCAST_MSG_TYPE_WU_SCHEDULE_CMD	  0x02
#define BROADCAST_MSG_LENGTH_WU_SCHEDULE_CMD  0x04

#define BROADCAST_MSG_TYPE_MODE_CHANGE_CMD	  0x03
#define BROADCAST_MSG_LENGTH_MODE_CHANGE_CMD  0x02

#define GATT_PKT_SET_ID_CMD					  0x01
#define GATT_PKT_SET_BEACON_MODE_CMD		  BROADCAST_MSG_TYPE_MODE_CHANGE_CMD

#define	DEFAULT_MTU_LENGTH						23

#define SNV_BASE_ID							  BLE_NVID_CUST_START
#define CONFIGURATION_SNV_OFFSET_ID			  0x00
#define TIMERS_SNV_OFFSET_ID			      0x01

#define STORED_CONFIGURATION_NODE_ID_OFFSET	  0x00
#define STORED_CONFIGURATION_MODE_OFFSET	  STORED_CONFIGURATION_NODE_ID_OFFSET + CHILD_NODE_ID_LENGTH
#define STORED_CONFIGURATION_MODE_LENGTH	  0x01

#define STORED_CONFIGURATION_TOTAL_LENGTH     CHILD_NODE_ID_LENGTH + STORED_CONFIGURATION_MODE_LENGTH

#define MAX_ALLOWED_TIMER_DURATION_SEC	      42000 //actual max timer duration 42949.67sec

#define DEFAULT_BEACON_MODE					  BEACON_ONLY
// Task configuration
#define SBT_TASK_PRIORITY                     1

#ifndef SBT_TASK_STACK_SIZE
#define SBT_TASK_STACK_SIZE                   664
#endif
#ifdef FEATURE_OAD
// The size of an OAD packet.
#define OAD_PACKET_SIZE                       ((OAD_BLOCK_SIZE) + 2)
#endif // FEATURE_OAD

#define SBT_STATE_CHANGE_EVT                  0x0001
#define SBT_CHAR_CHANGE_EVT                   0x0002
#define PERIODIC_EVT                          0x0004
#define CONN_EVT_END_EVT                      0x0008
#define ADVERTISE_EVT					      0x0010
#define KEY_CHANGE_EVT					  	  0x0020
#define LED_TIMEOUT_EVT						  0x0040
#define RESET_BROADCAST_CMD_EVT				  0x0080
#define EPOCH_EVT							  0x0100
#define PRE_ADV_EVT							  0x0200
#define WATCHDOG_EVT						  0x0400
#define WAKEUP_TIMEOUT_EVT					  0x0800
#define GOTOSLEEP_TIMEOUT_EVT				  0x1000
#define PRE_CE_EVT							  0x2000
#define MR_PAIRING_STATE_EVT                  0x4000
#define CONNECTABLE_TIMEOUT_EVT				  0x8000
/*********************************************************************
 * TYPEDEFS
 */

typedef enum ChildClimbNodeStateType_t {
	BY_MYSELF = 0,
	CHECKING = 1,
	ON_BOARD = 2,
	ALERT = 3,
	GOING_TO_SLEEP = 4,
	BEACON = 5,
	INVALID_STATE = 0xFE,
	ERROR = 0xFF
} ChildClimbNodeStateType_t;

typedef enum MasterClimbNodeStateType_t {
	NOT_CONNECTED,
	CONNECTED,
	INVALID_MASTER_STATE
} MasterClimbNodeStateType_t;

typedef enum ClimbNodeType_t {
	CLIMB_CHILD_NODE,
	CLIMB_MASTER_NODE,
	NOT_CLIMB_NODE,
	NAME_NOT_PRESENT,
	WRONG_PARKET_TYPE
} ClimbNodeType_t;

// App event passed from profiles.
//typedef struct {
//  appEvtHdr_t hdr;  // event header.
//  uint8_t *pData;  // event data
//} sbpEvt_t;

typedef struct {
	//gapDevRec_t devRec;
	//uint8 addr[B_ADDR_LEN]; //!< Device's Address
	//uint8 id[NODE_ID_LENGTH];
	uint8 id[6];
	ChildClimbNodeStateType_t state;
	//uint8 advDataLen;
	//uint8 advData[31];
	//uint8 scnDataLen;
	//uint8 scnData[31];
	uint32 lastContactTicks;
	uint8 rssi;
	//ChildClimbNodeStateType_t stateToImpose;
} myGapDevRec_t;

typedef struct listNode{
    myGapDevRec_t device;
    struct listNode *next;
}listNode_t;

typedef struct {
	Clock_Struct wakeUpClock_Struct;
	Clock_Struct goToSleepClock_Struct;

	//uint32 wakeUpRemainingTimeout;
	//uint32 goToSleepRamainingTimeout;

	uint8 wakeUpTimerActive;
	uint8 goToSleepTimerActive;

	uint32 wakeUpTimeout_sec_global_value;

	uint8 validData;
} timerSNVDataStore_t;

typedef enum ClimbBeaconMode_t {
	BEACON_ONLY = 1,
	COMBO_MODE = 2,
} ClimbBeaconMode_t;

// App event passed from profiles.
typedef struct {
	uint16_t event;  // event type
	uint8_t status; // event status
	uint8_t *pData; // event data pointer
} sbtEvt_t;

// pairing callback event
typedef struct
{
  uint16 connectionHandle; //!< connection Handle
  uint8 state;             //!< state returned from GAPBondMgr
  uint8 status;            //!< status of state
} gapPairStateEvent_t;

// Discovery states
enum {
	BLE_DISC_STATE_IDLE,                // Idle
	BLE_DISC_STATE_MTU,                 // Exchange ATT MTU size
	BLE_DISC_STATE_SVC,                 // Service discovery
	BLE_DISC_STATE_CHAR                 // Characteristic discovery
};
/*********************************************************************
 * LOCAL VARIABLES
 */


// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Discovery state
static uint8_t discState = BLE_DISC_STATE_IDLE;

// Clock instances for internal periodic events.
static Clock_Struct periodicClock;
static Clock_Struct ledTimeoutClock;
static Clock_Struct connectableTimeoutClock;
static Clock_Struct wakeUpClock;
static Clock_Struct goToSleepClock;

static Clock_Struct preAdvClock;
// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

#if defined(FEATURE_OAD)
// Event data from OAD profile.
static Queue_Struct oadQ;
static Queue_Handle hOadQ;
#endif //FEATURE_OAD

// events flag for internal application events.
static uint16_t events;

// Task configuration
Task_Struct sbmTask;
Char sbmTaskStack[SBT_TASK_STACK_SIZE];

#if DEFAULT_BEACON_MODE == BEACON_ONLY
static ChildClimbNodeStateType_t nodeState = BEACON;
#else
static ChildClimbNodeStateType_t nodeState = BY_MYSELF;
#endif
static uint8 Climb_masterNodeName[] = {'C','L','I','M','B','M'};

static uint8 nodeTurnedOn = 0;

static uint8 myAddr[B_ADDR_LEN];
static uint8 myMasterId[MASTER_NODE_ID_LENGTH];

static uint8 defAdvertData[31] = {

		0x07,// length of this data
		GAP_ADTYPE_LOCAL_NAME_COMPLETE,
		'C',
		'L',
		'I',
		'M',
		'B',
		'C',
		0x05,// length of this data
		GAP_ADTYPE_MANUFACTURER_SPECIFIC,
		0x0D,
		0x00,
		0, //ID
		(uint8)BY_MYSELF,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0

};

static uint8 connAdvertData[] = { 0x07,	// length of this data
								GAP_ADTYPE_LOCAL_NAME_COMPLETE,
								'C', 'L', 'I', 'M', 'B', 'C',
								0x02,   // length of this data
								GAP_ADTYPE_FLAGS,
								GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED | DEFAULT_DISCOVERABLE_MODE //

};

 // GAP - SCAN RSP data (max size = 31 bytes)
 static uint8 defScanRspData[] =
 {
//		0x02,   // length of this data
//		GAP_ADTYPE_FLAGS,
//		GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED | DEFAULT_DISCOVERABLE_MODE, //
//
//		0x03,
//		GAP_ADTYPE_APPEARANCE,
//		(uint8) (GAP_APPEARE_GENERIC_TAG & 0xFF),
//		(uint8) ((GAP_APPEARE_GENERIC_TAG >> 8) & 0xFF)
 };

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "CLIMB Node";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

static PIN_Config ClimbAppPinTable[] = {
#ifdef CC2650STK
Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX, /* LED initially off             */
Board_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX, /* LED initially off             */
//		Board_BUZZER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX, /* Buzzer initially off          */
Board_MPU_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,

		PIN_TERMINATE };
#else
Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX, /* LED initially off             */
Board_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX, /* LED initially off             */

		PIN_TERMINATE };
#endif

// Global pin resources
PIN_State pinGpioState;
PIN_Handle hGpioPin;
static uint8 adv_counter = 0;

static uint8 myIDArray[] = {NODE_ID};
static uint8 broadcastID[] = { 0xFF };

static ClimbBeaconMode_t beacon_mode = DEFAULT_BEACON_MODE;

static uint32 batteryLev = 0;

static uint32 wakeUpTimeout_sec_global = 0;

static uint8 scanning = FALSE;

static uint8 devicesHeardDuringLastScan = 0;

static uint8 adv_startNodeIndex = 0;

static myGapDevRec_t* childListArray = NULL;
static myGapDevRec_t* masterListArray = NULL;

// Display Interface
Display_Handle dispHandle = NULL;

// Connection handle of current connection
static uint16_t connHandle = GAP_CONNHANDLE_INIT;
static uint8 mtu_size = DEFAULT_MTU_LENGTH;

static uint8 childInitModeActive = FALSE;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleTopology_init(void);
static void nodeListInit(void);
static void simpleTopology_taskFxn(UArg a0, UArg a1);
static uint8_t simpleTopology_processStackMsg(ICall_Hdr *pMsg);
static uint8_t simpleTopology_processGATTMsg(gattMsgEvent_t *pMsg);
static void simpleTopology_processAppMsg(sbtEvt_t *pMsg);
static void Climb_processCharValueChangeEvt(uint8_t paramID);
static void simpleTopology_processRoleEvent(gapMultiRoleEvent_t *pEvent);
static void climb_processPairState(gapPairStateEvent_t* pairingEvent);
static void climb_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle, uint8_t uiInputs, uint8_t uiOutputs, uint32_t numComparison);
static void climb_pairStateCB(uint16_t connHandle, uint8_t state, uint8_t status);
static void BLE_AdvertiseEventHandler(void);
static void BLE_ConnectionEventHandler_sendAttRsp(void);
static void simpleTopology_freeAttRsp(uint8_t status);
static uint8_t simpleTopology_eventCB(gapMultiRoleEvent_t *pEvent);
static void simpleTopology_charValueChangeCB(uint8_t paramID);
#ifdef FEATURE_OAD
void SimpleBLEPeripheral_processOadWriteCB(uint8_t event, uint16_t connHandle,uint8_t *pData);
#endif //FEATURE_OAD

////CLIMB MANAGEMENT
static ClimbNodeType_t isClimbNode(gapDeviceInfoEvent_t *gapDeviceInfoEvent_a);
static void Climb_addNodeDeviceInfo(gapDeviceInfoEvent_t *gapDeviceInfoEvent, ClimbNodeType_t nodeType);
static uint8 Climb_findNodeById(uint8 *nodeID, ClimbNodeType_t nodeType);
static uint8 Climb_addNode(gapDeviceInfoEvent_t *gapDeviceInfoEvent, ClimbNodeType_t nodeType);
static void Climb_updateNodeMetadata(gapDeviceInfoEvent_t *gapDeviceInfoEvent, uint8 index, ClimbNodeType_t nodeType);
static uint8 Climb_checkNodeState(gapDeviceInfoEvent_t *gapDeviceInfoEvent_a);
static ChildClimbNodeStateType_t Climb_findMyBroadcastedState(gapDeviceInfoEvent_t *gapDeviceInfoEvent_a);
static uint8 Climb_decodeBroadcastedMessage(gapDeviceInfoEvent_t *gapDeviceInfoEvent_a);
static void Climb_updateMyBroadcastedState(ChildClimbNodeStateType_t newState);
static void Climb_nodeTimeoutCheck(void);
static void Climb_removeNode(uint8 indexToRemove, ClimbNodeType_t nodeType);
static void destroyChildNodeList(void);
static void destroyMasterNodeList(void);
static void Climb_enterChildInitMode(void);
static void Climb_exitChildInitMode(void);
#ifdef PRINTF_ENABLED
#ifdef PRINT_NODE_INFO_ENABLED
static void Climb_printfNodeInfo(gapDeviceInfoEvent_t *gapDeviceInfoEvent );
#endif
#endif

// TIMER ROUTINES
static void Climb_periodicTask(void);
static void Climb_preAdvEvtHandler(void);
static void Climb_goToSleepHandler(void);
static void Climb_wakeUpHandler(void);
static void Climb_setWakeUpClock(uint32 wakeUpTimeout_sec_local);

//HARDWARE RELATED FUNCTIONS
static void CLIMB_FlashLed(PIN_Id pinId);
static void Keys_EventCB(keys_Notifications_t notificationType);
static void CLIMB_handleKeys(uint8 keys);
static void startNode(void);
static void stopNode(void);
#ifdef FEATURE_LCD
static void displayInit(void);
#endif
#ifdef WATCHDOGTIMER_EN
static void watchdogTimerInit();
static void timerCallback(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask);
#endif
static void saveTimersConf(void);
static uint8 restoreTimersConf(void);
static void Climb_clockHandler(UArg arg);
static uint8 memcomp(uint8 * str1, uint8 * str2, uint8 len);
static uint8 isNonZero(uint8 * str1, uint8 len);
static uint8_t simpleTopology_enqueueMsg(uint16_t event, uint8_t status, uint8_t *pData);
#ifdef HEAPMGR_METRICS
static void plotHeapMetrics();
#endif
#ifdef STACK_METRICS
static void checkStackMetrics();
#endif
/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t simpleTopology_gapRoleCBs = { simpleTopology_eventCB,        // events to be handled by the app are passed through the GAP Role here
		};

// Simple GATT Profile Callbacks
static climbProfileCBs_t simpleTopology_simpleProfileCBs = { simpleTopology_charValueChangeCB // Characteristic value change callback
		};

// Keys Callbacks
static keysEventCBs_t Keys_EventCBs = { Keys_EventCB, // Profile State Change Callbacks
		};

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs = {
		(pfnPasscodeCB_t)climb_passcodeCB, // Passcode callback
		climb_pairStateCB
		};

#ifdef FEATURE_OAD
static oadTargetCBs_t simpleBLEPeripheral_oadCBs = { (oadWriteCB_t) SimpleBLEPeripheral_processOadWriteCB // Write Callback.
		};
#endif //FEATURE_OAD

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      simpleTopology_createTask
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
 *
 * @param   None.
 *
 * @return  None.
 */
void SimpleTopology_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbmTaskStack;
  taskParams.stackSize = SBT_TASK_STACK_SIZE;
  taskParams.priority = SBT_TASK_PRIORITY;

  Task_construct(&sbmTask, simpleTopology_taskFxn, &taskParams, NULL);
}
/*********************************************************************
 * @fn      simpleTopology_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   None.
 *
 * @return  None.
 */
static void simpleTopology_init(void) {
#ifdef CC2650STK
	// Setup I2C for sensors
	SensorI2C_open();
#endif
	// Handling of buttons, LED, relay
	hGpioPin = PIN_open(&pinGpioState, ClimbAppPinTable);
	//PIN_registerIntCb(hGpioPin, Key_callback);

	//initialize keys
	Keys_Init(&Keys_EventCBs);
	//dispHandle = Display_open(Display_Type_LCD, NULL);
	// ******************************************************************
	// N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
	// ******************************************************************
	// Register the current thread as an ICall dispatcher application
	// so that the application can send and receive messages.
	ICall_registerApp(&selfEntity, &sem);

	// Create an RTOS queue for message from profile to be sent to app.
	appMsgQueue = Util_constructQueue(&appMsg);

	// Create one-shot clocks for internal periodic events.
	Util_constructClock(&periodicClock, Climb_clockHandler,
			PERIODIC_EVT_PERIOD_MSEC, 0, false, PERIODIC_EVT);

	Util_constructClock(&ledTimeoutClock, Climb_clockHandler,
			LED_TIMEOUT_MSEC, 0, false, LED_TIMEOUT_EVT);

	Util_constructClock(&connectableTimeoutClock, Climb_clockHandler,
			CONNECTABLE_TIMEOUT_MSEC, 0, false, CONNECTABLE_TIMEOUT_EVT);

	Util_constructClock(&preAdvClock, Climb_clockHandler,
			PRE_ADV_TIMEOUT, 0, false, PRE_ADV_EVT);

	Util_constructClock(&wakeUpClock, Climb_clockHandler,
			WAKEUP_DEFAULT_TIMEOUT_SEC * 1000, 0, false, WAKEUP_TIMEOUT_EVT);

	Util_constructClock(&goToSleepClock, Climb_clockHandler,
			GOTOSLEEP_DEFAULT_TIMEOUT_SEC * 1000, 0, false, GOTOSLEEP_TIMEOUT_EVT);

//#warning MAC ADDRESS MODIFIED
//	uint8 tempAddr[] = {
//			0x83,
//			0xcd,
//			0xb6,
//			0xf8,
//			0xe6,
//			0xa0,
//	};
//	HCI_EXT_SetBDADDRCmd(tempAddr);
//	GAP_ConfigDeviceAddr(ADDRTYPE_STATIC, tempAddr);

	uint32_t uint32_arg;
	// Setup the GAP
	{

		/*-------------------PERIPHERAL-------------------*/
		// Setup the GAP Peripheral Role Profile
		// Setup the GAP
		GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);

		// For all hardware platforms, device starts advertising upon initialization
		uint32_arg = FALSE;
		GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &uint32_arg, NULL);

		// Set the GAP Role Parameters
		// By setting this to zero, the device will go into the waiting state after
		// being discoverable for 30.72 second, and will not being advertising again
		// until the enabler is set back to TRUE
		uint32_arg = 0;
		GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t), &uint32_arg, NULL);

		uint32_arg = DEFAULT_ENABLE_UPDATE_REQUEST;
		GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t), &uint32_arg, NULL);

		uint32_arg = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
		GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t), &uint32_arg, NULL);

		uint32_arg = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
		GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t), &uint32_arg, NULL);

		uint32_arg = DEFAULT_DESIRED_SLAVE_LATENCY;
		GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t), &uint32_arg, NULL);

		uint32_arg = DEFAULT_DESIRED_CONN_TIMEOUT;
		GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t), &uint32_arg, NULL);

		//set up advertise and scan response data
		GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(defScanRspData), defScanRspData, NULL);
		GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(defAdvertData), defAdvertData, NULL);

		uint32_arg = DEFAULT_NON_CONNECTABLE_ADVERTISING_INTERVAL;

		GAP_SetParamValue(TGAP_CONN_ADV_INT_MIN, uint32_arg);
		GAP_SetParamValue(TGAP_CONN_ADV_INT_MAX, uint32_arg);

		uint32_arg = DEFAULT_CONNECTABLE_ADVERTISING_INTERVAL;

		GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, uint32_arg);
		GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, uint32_arg);

		GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, uint32_arg);
		GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, uint32_arg);

		/*-------------------CENTRAL-------------------*/
		uint8 childScanRes = DEFAULT_MAX_SCAN_RES;
		GAPRole_SetParameter(GAPROLE_MAX_SCAN_RES, sizeof(uint8_t), &childScanRes,NULL);

		GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION);
		GAP_SetParamValue(TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION);
		GAP_SetParamValue(TGAP_GEN_DISC_SCAN_INT, SCAN_INTERVAL);
		GAP_SetParamValue(TGAP_LIM_DISC_SCAN_INT, SCAN_INTERVAL);
		GAP_SetParamValue(TGAP_GEN_DISC_SCAN_WIND, SCAN_WINDOW);
		GAP_SetParamValue(TGAP_LIM_DISC_SCAN_WIND, SCAN_WINDOW);
		GAP_SetParamValue(TGAP_FILTER_ADV_REPORTS, FILTER_ADV_REPORTS);
	}

	// Setup the GAP Role Profile
	{
		/*--------PERIPHERAL-------------*/
		// Register with GAP for HCI/Host messages
		GAP_RegisterForMsgs(selfEntity);
	}

	//GATT
	{
		/*---------------------SERVER------------------------*/
		// Set the GAP Characteristics
		GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);
		gapPeriConnectParams_t connParam = {
				DEFAULT_DESIRED_MIN_CONN_INTERVAL,
				DEFAULT_DESIRED_MAX_CONN_INTERVAL,
				DEFAULT_DESIRED_SLAVE_LATENCY,
				DEFAULT_DESIRED_CONN_TIMEOUT

		};
		GGS_SetParameter(GGS_PERI_CONN_PARAM_ATT, sizeof(gapPeriConnectParams_t), &connParam);
		uint32_arg = GAP_APPEARE_GENERIC_TAG;
		GGS_SetParameter(GGS_APPEARANCE_ATT, sizeof(uint16), &uint32_arg);

		// Initialize GATT Server Services
		GGS_AddService(GATT_ALL_SERVICES);           // GAP
		GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
		//DevInfo_AddService();                        // Device Information Service
		ClimbProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile

		// Add battery service.
#ifdef INCLUDE_BATTERY_SERVICE
		Batt_AddService();
#endif

#ifdef FEATURE_OAD
  VOID OAD_addService();                 // OAD Profile
  OAD_register((oadTargetCBs_t *)&simpleBLEPeripheral_oadCBs);
  hOadQ = Util_constructQueue(&oadQ);
#endif //FEATURE_OAD

#ifdef IMAGE_INVALIDATE
  Reset_addService();
#endif //IMAGE_INVALIDATE

		// Register callback with SimpleGATTprofile
		ClimbProfile_RegisterAppCBs(&simpleTopology_simpleProfileCBs);

		/*-----------------CLIENT------------------*/
		GATT_RegisterForMsgs(selfEntity);

	}
	// Setup the GAP Bond Manager
	{
		uint32_arg = 0; // passkey "000000"
		GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint8_t), &uint32_arg);

		uint32_arg = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ; //GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
		GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &uint32_arg);

		uint32_arg = FALSE;
		GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &uint32_arg);

		uint32_arg = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT;
		GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &uint32_arg);

		uint32_arg = TRUE;
		GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &uint32_arg);
	}

#ifdef CC2650STK
	SensorBmp280_init();
	SensorBmp280_enable(FALSE);
	SensorHdc1000_init();
	PIN_setOutputValue(hGpioPin, Board_MPU_POWER, 0);
	//SensorMpu9250_init(); //ho gia scollegato l'alimentazione all'interno del file Board.c
	//SensorMpu9250_reset();
	SensorOpt3001_init();
	SensorTmp007_init();
#endif
	//SETTING POWER
	HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_0_DBM);

#ifdef WATCHDOGTIMER_EN
	watchdogTimerInit();
#endif
	nodeListInit();

	// Start the Device
	VOID GAPRole_StartDevice(&simpleTopology_gapRoleCBs);

	// Start Bond Manager
	VOID GAPBondMgr_Register(&simpleBLEPeripheral_BondMgrCBs);

	uint8 ret = SUCCESS;
	uint8 snv_stored_conf[STORED_CONFIGURATION_TOTAL_LENGTH];
	//read snv_stored_conf from FLASH
	ret = osal_snv_write(SNV_BASE_ID + CONFIGURATION_SNV_OFFSET_ID, 0, snv_stored_conf);  //this is needed to initialize this ID in SNV
	if (ret == SUCCESS) {
		ret = osal_snv_read(SNV_BASE_ID + CONFIGURATION_SNV_OFFSET_ID, STORED_CONFIGURATION_TOTAL_LENGTH, snv_stored_conf);
		if (ret == SUCCESS) {
			memcpy(myIDArray, &(snv_stored_conf[STORED_CONFIGURATION_NODE_ID_OFFSET]), CHILD_NODE_ID_LENGTH);

			//if the stored configurations contains a valid
			switch (snv_stored_conf[STORED_CONFIGURATION_MODE_OFFSET]) {
			case (uint8) BEACON_ONLY:	//only accept valid data
			case (uint8) COMBO_MODE:
				beacon_mode = (ClimbBeaconMode_t)snv_stored_conf[STORED_CONFIGURATION_MODE_OFFSET];
				//beacon_mode = DEFAULT_BEACON_MODE;
				break;

			default:
				beacon_mode = DEFAULT_BEACON_MODE;//BEACON MODE FIELD CONTAINS A NON-VALID VALUE, start with default one, NB: writing it to flash is useless, since, if the mode isn't changed, at the next startup it will get here anyway.
				break;
			}
		}
	}
	if( beacon_mode == BEACON_ONLY ){
		Climb_updateMyBroadcastedState(BEACON);
	}else if( beacon_mode == COMBO_MODE ){
		Climb_updateMyBroadcastedState(BY_MYSELF);
	}

	if ( restoreTimersConf() ) { // the flash contained a valid timers configuration
		if (Util_isActive(&goToSleepClock)) { //if this timer is active it means that this boot follows a fault, and before the fault the node was active, then start it.
			startNode(); //start the node
		} else {
			//do nothing
		}
	} else {  // the flash did not contain a valid configuration. Proceed with normal boot
//automatically start-up the node
#warning the node is configured to start as childInitModeActive automatically upon power up
		Climb_enterChildInitMode();
		//events |= WAKEUP_TIMEOUT_EVT;
		//Semaphore_post(sem);
	}

//#if defined FEATURE_OAD
//#if defined (HAL_IMAGE_A)
//  Display_print0(dispHandle, 0, 0, "BLE Peripheral A");
//#else
//  Display_print0(dispHandle, 0, 0, "BLE Peripheral B");
//#endif // HAL_IMAGE_A
//#else
//  Display_print0(dispHandle, 0, 0, "BLE Peripheral");
//#endif // FEATURE_OAD

}

/*********************************************************************
 * @fn      nodeListInit
 *
 * @brief   Allocate memory for the nodes' data and set it to 0.
 *
 * @param
 *
 * @return  None.
 */
static void nodeListInit(void) {

	//init children list
	uint16 arraySize = MAX_SUPPORTED_CHILD_NODES * sizeof(myGapDevRec_t);
	childListArray = (myGapDevRec_t*) ICall_malloc(arraySize);
	memset(childListArray, 0, arraySize); //reset the memory

	//init masters list
	arraySize = MAX_SUPPORTED_MASTER_NODES * sizeof(myGapDevRec_t);
	masterListArray = (myGapDevRec_t*) ICall_malloc(arraySize);
	memset(masterListArray, 0, arraySize); //reset the memory

}
/*********************************************************************
 * @fn      simpleTopology_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Multi.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void simpleTopology_taskFxn(UArg a0, UArg a1) {
	// Initialize application
	simpleTopology_init();

	// Application main loop
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
				uint8 safeToDealloc = TRUE;

				if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity)) {
					ICall_Stack_Event *pEvt = (ICall_Stack_Event *) pMsg;

					// Check for BLE stack events first
					if (pEvt->signature == 0xffff) {
						if (pEvt->event_flag & CONN_EVT_END_EVT) {
							// Try to retransmit pending ATT Response (if any)
							BLE_ConnectionEventHandler_sendAttRsp();
						}

						if (pEvt->event_flag & ADVERTISE_EVT) {
							BLE_AdvertiseEventHandler();
						}
					} else {
						// Process inter-task message
						safeToDealloc = simpleTopology_processStackMsg((ICall_Hdr *) pMsg);
					}
				}

				if (pMsg && safeToDealloc) {
					ICall_freeMsg(pMsg);
				}
			}

			// If RTOS queue is not empty, process app message.
			while (!Queue_empty(appMsgQueue)) {
				sbtEvt_t *pMsg = (sbtEvt_t *) Util_dequeueMsg(appMsgQueue);
				if (pMsg) {
					// Process message.
					simpleTopology_processAppMsg(pMsg);

					// Free the space from the message.
					ICall_free(pMsg);
				}
			}
		}
		if (events & PERIODIC_EVT) {
			events &= ~PERIODIC_EVT;
		      if(nodeTurnedOn != 0){
		    	  if(beacon_mode == COMBO_MODE){
					Util_startClock(&periodicClock);
					// Perform periodic application task
					Climb_periodicTask();
		    	  }
		      }
		}

		if (events & LED_TIMEOUT_EVT) {
			events &= ~LED_TIMEOUT_EVT;
			//only turn off leds
			PIN_setOutputValue(hGpioPin, Board_LED1, Board_LED_OFF);
			PIN_setOutputValue(hGpioPin, Board_LED2, Board_LED_OFF);
		}

		if (events & PRE_ADV_EVT) {
			events &= ~PRE_ADV_EVT;

			Climb_preAdvEvtHandler();
		}

	    if (events & CONNECTABLE_TIMEOUT_EVT){
	    	events &= ~CONNECTABLE_TIMEOUT_EVT;
	    	if(childInitModeActive){
	    		Climb_exitChildInitMode();
	    	}
	    }

#ifdef WATCHDOGTIMER_EN
		if (events & WATCHDOG_EVT) {
			events &= ~WATCHDOG_EVT;

			unclearedWatchdogEvents = 0;

		}
#endif

		if (events & GOTOSLEEP_TIMEOUT_EVT) {
			events &= ~GOTOSLEEP_TIMEOUT_EVT;

			Climb_goToSleepHandler();
		}

		if (events & WAKEUP_TIMEOUT_EVT) {
			events &= ~WAKEUP_TIMEOUT_EVT;

			Climb_wakeUpHandler();
		}

#ifdef FEATURE_OAD
		while (!Queue_empty(hOadQ)) {
			oadTargetWrite_t *oadWriteEvt = Queue_dequeue(hOadQ);

			// Identify new image.
			if (oadWriteEvt->event == OAD_WRITE_IDENTIFY_REQ) {
				OAD_imgIdentifyWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
			}
			// Write a next block request.
			else if (oadWriteEvt->event == OAD_WRITE_BLOCK_REQ) {
				OAD_imgBlockWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
			}

			// Free buffer.
			ICall_free(oadWriteEvt);
		}
#endif //FEATURE_OAD
	}
}

/*********************************************************************
 * @fn      simpleTopology_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t simpleTopology_processStackMsg(ICall_Hdr *pMsg) {
	uint8_t safeToDealloc = TRUE;

	switch (pMsg->event) {
	case GATT_MSG_EVENT:
		// Process GATT message
		safeToDealloc = simpleTopology_processGATTMsg((gattMsgEvent_t *) pMsg);
		break;

	case HCI_GAP_EVENT_EVENT: {
		// Process HCI message
		switch (pMsg->status) {
		case HCI_COMMAND_COMPLETE_EVENT_CODE:
			// Process HCI Command Complete Event
			break;

		default:
			break;
		}
	}
		break;

	case GAP_MSG_EVENT:
		simpleTopology_processRoleEvent((gapMultiRoleEvent_t *) pMsg);
		break;

	default:
		// do nothing
		break;
	}

	return (safeToDealloc);
}

/*********************************************************************
 * @fn      simpleTopology_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t simpleTopology_processGATTMsg(gattMsgEvent_t *pMsg) {
	// See if GATT server was unable to transmit an ATT response
	if (pMsg->hdr.status == blePending) {
		// No HCI buffer was available. Let's try to retransmit the response
		// on the next connection event.
		if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
		CONN_EVT_END_EVT) == SUCCESS) {
			// First free any pending response
			simpleTopology_freeAttRsp(FAILURE);

			// Hold on to the response message for retransmission
			pAttRsp = pMsg;

			// Don't free the response message yet
			return (FALSE);
		}
	} else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT) {
		// ATT request-response or indication-confirmation flow control is
		// violated. All subsequent ATT requests or indications will be dropped.
		// The app is informed in case it wants to drop the connection.

		// Display the opcode of the message that caused the violation.
		//LCD_WRITE_STRING_VALUE("FC Violated:", pMsg->msg.flowCtrlEvt.opcode, 10, LCD_PAGE6);
	} else if (pMsg->method == ATT_MTU_UPDATED_EVENT) {
		mtu_size = pMsg->msg.mtuEvt.MTU;
		//PIN_setOutputValue(hGpioPin, Board_LED1, Board_LED_ON);
		// MTU size updated
		//LCD_WRITE_STRING_VALUE("MTU Size:", pMsg->msg.mtuEvt.MTU, 10, LCD_PAGE6);
	}

	//messages from GATT server
	if ((gapRoleNumLinks(GAPROLE_ACTIVE_LINKS) > 0)) {
		if ((pMsg->method == ATT_READ_RSP) || ((pMsg->method == ATT_ERROR_RSP) && (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ))) {
			if (pMsg->method == ATT_ERROR_RSP) {
				//LCD_WRITE_STRING_VALUE("Read Error", pMsg->msg.errorRsp.errCode, 10,  LCD_PAGE6);
			} else {
				// After a successful read, display the read value
				//LCD_WRITE_STRING_VALUE("Read rsp:", pMsg->msg.readRsp.pValue[0], 10, LCD_PAGE6);
			}

		} else if ((pMsg->method == ATT_WRITE_RSP) || ((pMsg->method == ATT_ERROR_RSP) && (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ))) {

			if (pMsg->method == ATT_ERROR_RSP == ATT_ERROR_RSP) {
				//LCD_WRITE_STRING_VALUE("Write Error", pMsg->msg.errorRsp.errCode, 10, LCD_PAGE6);
			} else {
				// After a succesful write, display the value that was written and
				// increment value
				//LCD_WRITE_STRING_VALUE("Write sent:", charVal++, 10, LCD_PAGE6);
			}
		} else if (discState != BLE_DISC_STATE_IDLE) {
			//simpleTopology_processGATTDiscEvent(pMsg);
		}
	} // else - in case a GATT message came after a connection has dropped, ignore it.

	// Free message payload. Needed only for ATT Protocol messages
	GATT_bm_free(&pMsg->msg, pMsg->method);

	// It's safe to free the incoming message
	return (TRUE);
}

/*********************************************************************
 * @fn      simpleTopology_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void simpleTopology_processAppMsg(sbtEvt_t *pMsg) {
	switch (pMsg->event) {
	case SBT_STATE_CHANGE_EVT:
		simpleTopology_processStackMsg((ICall_Hdr *) pMsg->pData);
		// Free the stack message, the SBT_STATE_CHANGE_EVT is the only event that carries a pointer to a valid pData, the others call to simpleTopology_enqueueMsg set pData to NULL
		ICall_freeMsg(pMsg->pData);
		break;

	case SBT_CHAR_CHANGE_EVT:
		Climb_processCharValueChangeEvt(pMsg->status);
		break;

	case KEY_CHANGE_EVT:
		CLIMB_handleKeys(pMsg->status);
		break;

	case MR_PAIRING_STATE_EVT:
	    climb_processPairState((gapPairStateEvent_t*)pMsg->pData);
	    // Free the app data
	    ICall_free(pMsg->pData);
	    break;

	default:
		// Do nothing.
		break;
	}
}

/*********************************************************************
 * @fn      simpleTopology_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void Climb_processCharValueChangeEvt(uint8_t paramID) {
	uint8_t newValue[CLIMBPROFILE_CHAR2_LEN];

	switch (paramID) {
	case CLIMBPROFILE_CHAR1:
		ClimbProfile_GetParameter(CLIMBPROFILE_CHAR1, newValue);
		break;

	case CLIMBPROFILE_CHAR2:
		ClimbProfile_GetParameter(CLIMBPROFILE_CHAR2, newValue);

		if (newValue[0] == GATT_PKT_SET_ID_CMD) {
			uint8 snv_to_store_conf[STORED_CONFIGURATION_TOTAL_LENGTH];

			memcpy(myIDArray, &newValue[1], CHILD_NODE_ID_LENGTH);
			memcpy(&(snv_to_store_conf[STORED_CONFIGURATION_NODE_ID_OFFSET]), &newValue[1], CHILD_NODE_ID_LENGTH);
			snv_to_store_conf[STORED_CONFIGURATION_MODE_OFFSET] = (uint8)beacon_mode;
			osal_snv_write(SNV_BASE_ID+CONFIGURATION_SNV_OFFSET_ID, STORED_CONFIGURATION_TOTAL_LENGTH, snv_to_store_conf);
		} else if (newValue[0] == GATT_PKT_SET_BEACON_MODE_CMD) {
			uint8 snv_to_store_conf[STORED_CONFIGURATION_TOTAL_LENGTH];
			uint8 snv_stored_conf[STORED_CONFIGURATION_TOTAL_LENGTH];

			switch (newValue[1]) {
			case (uint8)BEACON_ONLY:	//only accept valid data
			case (uint8)COMBO_MODE:
				//save the new configuration in ram
				beacon_mode = (ClimbBeaconMode_t)newValue[1];
				snv_to_store_conf[STORED_CONFIGURATION_MODE_OFFSET] = (ClimbBeaconMode_t)newValue[1];
				memcpy(&(snv_to_store_conf[STORED_CONFIGURATION_NODE_ID_OFFSET]), myIDArray, CHILD_NODE_ID_LENGTH);
				//retrieve the stored configuration

				osal_snv_read(SNV_BASE_ID+CONFIGURATION_SNV_OFFSET_ID, STORED_CONFIGURATION_TOTAL_LENGTH, snv_stored_conf);
				if(snv_stored_conf[STORED_CONFIGURATION_MODE_OFFSET] != beacon_mode){ //if the stored and the new configurations don't match store the new version in flash
					osal_snv_write(SNV_BASE_ID+CONFIGURATION_SNV_OFFSET_ID, STORED_CONFIGURATION_TOTAL_LENGTH, snv_to_store_conf);
				}

				if( beacon_mode == BEACON_ONLY ){
					Climb_updateMyBroadcastedState(BEACON);
				}else if( beacon_mode == COMBO_MODE ){
					Climb_updateMyBroadcastedState(BY_MYSELF);
				}
				break;

			default:
				//BEACON MODE FIELD CONTAINS A NON-VALID VALUE
				break;
			}

		}
		break;

	default:
		// COMMAND FIELD CONTAINS A NON VALID VALUE
		break;
	}
}

/*********************************************************************
 * @fn      simpleTopology_processRoleEvent
 *
 * @brief   Multi role event processing function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void simpleTopology_processRoleEvent(gapMultiRoleEvent_t *pEvent) {
	switch (pEvent->gap.opcode) {
	case GAP_DEVICE_INIT_DONE_EVENT: {
		//maxPduSize = pEvent->initDone.dataPktLen;

		memcpy(myAddr, pEvent->initDone.devAddr, B_ADDR_LEN); //salva l'indirizzo del nodo

#ifdef FEATURE_LCD
		char buf[10];
		sprintf(buf,"Me: ");
		devpkLcdText(buf, 1, 0);
		sprintf(buf,Util_convertBdAddr2Str(childDevList[0].devRec.addr));
		devpkLcdText(buf, 1, 5);
#endif

	}
		break;

	case GAP_MAKE_DISCOVERABLE_DONE_EVENT: {

		HCI_EXT_AdvEventNoticeCmd(selfEntity, ADVERTISE_EVT);
	}
		break;

	case GAP_END_DISCOVERABLE_DONE_EVENT: {
		//HCI_EXT_AdvEventNoticeCmd(selfEntity, ADVERTISE_EVT);
	}
		break;

	case GAP_DEVICE_INFO_EVENT: {

		devicesHeardDuringLastScan++;

				if (pEvent->deviceInfo.eventType == GAP_ADRPT_ADV_SCAN_IND | //adv data event (Scannable undirected)
					pEvent->deviceInfo.eventType == GAP_ADRPT_ADV_IND      |
					pEvent->deviceInfo.eventType == GAP_ADRPT_ADV_NONCONN_IND) { //adv data event (Connectable undirected)

		#ifdef PRINTF_ENABLED
					System_printf(
							"\nGAP_DEVICE_INFO_EVENT-GAP_ADRPT_ADV_SCAN_IND, ADV_DATA. address: ");
					System_printf(Util_convertBdAddr2Str(pEvent->deviceInfo.addr));
					System_printf("\nChecking if it is a CLIMB node.\n");
		#endif
					ClimbNodeType_t nodeType = isClimbNode((gapDeviceInfoEvent_t*) &pEvent->deviceInfo);
					if (nodeType == CLIMB_CHILD_NODE || nodeType == CLIMB_MASTER_NODE) {
						Climb_addNodeDeviceInfo(&pEvent->deviceInfo, nodeType);
						if (nodeType == CLIMB_MASTER_NODE){
							Climb_checkNodeState(&pEvent->deviceInfo);
						}
					}else {

		#ifdef PRINTF_ENABLED
						System_printf("\nIt isn't a CLIMB node, device discarded!\n");
		#endif
					}
				} else if(pEvent->deviceInfo.eventType == GAP_ADRPT_SCAN_RSP) {  //scan response data event
		#ifdef PRINTF_ENABLED
					System_printf("\nScan response received!\n");
		#endif
				}

				if (devicesHeardDuringLastScan >= DEFAULT_MAX_SCAN_RES) {
					if (nodeTurnedOn) {
						GAPRole_CancelDiscovery();
						scanning = FALSE;
						if (!scanning) {
							uint8 status = GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE, DEFAULT_DISCOVERY_ACTIVE_SCAN, DEFAULT_DISCOVERY_WHITE_LIST);
							scanning = TRUE;
						}
					}
				}
			}
		break;

	case GAP_DEVICE_DISCOVERY_EVENT: {
		scanning = FALSE;
		devicesHeardDuringLastScan = 0;

		if( beacon_mode == BEACON_ONLY ){
			destroyChildNodeList();
			destroyMasterNodeList();
		}
	}
		break;

	case GAP_LINK_ESTABLISHED_EVENT: {
		if (pEvent->gap.hdr.status == SUCCESS) {
			//NOTE: childInitModeActive/non-connctable adv switch is automatically done in multi.c
			//store connection handle
			connHandle = pEvent->linkCmpl.connectionHandle;

			HCI_EXT_ConnEventNoticeCmd(connHandle, selfEntity, CONN_EVT_END_EVT);

			uint8 adv_active = FALSE;
			GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),&adv_active, NULL);

			if(childInitModeActive){
				Climb_exitChildInitMode();
			}
            // Enable non-childInitModeActive advertising.
			if(nodeTurnedOn){
				adv_active = FALSE;
			}else{
				adv_active = FALSE;
			}
            GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),&adv_active, NULL);

            if (Util_isActive(&connectableTimeoutClock)){
            	Util_stopClock(&connectableTimeoutClock);
            }

		} else {
			connHandle = GAP_CONNHANDLE_INIT;
			discState = BLE_DISC_STATE_IDLE;
		}
	}
		break;

	case GAP_LINK_TERMINATED_EVENT: {
		connHandle = GAP_CONNHANDLE_INIT;

		// Disable connection event end notice
		HCI_EXT_ConnEventNoticeCmd(pEvent->linkTerminate.connectionHandle, selfEntity, 0);

		simpleTopology_freeAttRsp(bleNotConnected);
		if(childInitModeActive){
			Climb_exitChildInitMode();
		}

		// Enable non-childInitModeActive advertising if the node was turned on (otherwise only the childInitMode was active).
		if (nodeTurnedOn) {
			uint8 adv_active = TRUE;
			GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t), &adv_active, NULL);
		}

	}
		break;

	case GAP_LINK_PARAM_UPDATE_EVENT: {

	}
		break;

	default:
		break;
	}
}

/*********************************************************************
 * @fn      multi_role_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void climb_processPairState(gapPairStateEvent_t* pairingEvent)
{
  if (pairingEvent->state == GAPBOND_PAIRING_STATE_STARTED)
  {
    //Display_print1(dispHandle, LCD_PAGE7, 0,"Cxn %d pairing started", pairingEvent->connectionHandle);
  }
  else if (pairingEvent->state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    if (pairingEvent->status == SUCCESS)
    {
      //Display_print1(dispHandle, LCD_PAGE7, 0,"Cxn %d pairing success", pairingEvent->connectionHandle);
    }
    else
    {
      //Display_print2(dispHandle, LCD_PAGE7, 0, "Cxn %d pairing fail: %d", pairingEvent->connectionHandle, pairingEvent->status);
    }
  }
  else if (pairingEvent->state == GAPBOND_PAIRING_STATE_BONDED)
  {
    if (pairingEvent->status == SUCCESS)
    {
      //Display_print1(dispHandle, LCD_PAGE7, 0, "Cxn %d bonding success", pairingEvent->connectionHandle);
    }
  }
  else if (pairingEvent->state == GAPBOND_PAIRING_STATE_BOND_SAVED)
  {
    if (pairingEvent->status == SUCCESS)
    {
      //Display_print1(dispHandle, LCD_PAGE7, 0, "Cxn %d bond save success", pairingEvent->connectionHandle);
    }
    else
    {
      //Display_print2(dispHandle, LCD_PAGE7, 0, "Cxn %d bond save failed: %d", pairingEvent->connectionHandle, pairingEvent->status);
    }
  }
}


/*********************************************************************
 * @fn      multi_role_passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void climb_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs, uint32_t numComparison)
{ //show passcode function
//  gapPasskeyNeededEvent_t *pData;
//
//  // Allocate space for the passcode event.
//  if ((pData = ICall_malloc(sizeof(gapPasskeyNeededEvent_t))))
//  {
//    memcpy(pData->deviceAddr, deviceAddr, B_ADDR_LEN);
//    pData->connectionHandle = connHandle;
//    pData->uiInputs = uiInputs;
//    pData->uiOutputs = uiOutputs;
//    pData->numComparison = numComparison;
//
//    // Enqueue the event.
//    multi_role_enqueueMsg(MR_PASSCODE_NEEDED_EVT, (uint8_t *) pData);
//  }
}

/************************************************************************
 * @fn      multi_role_pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void climb_pairStateCB(uint16_t connHandle, uint8_t state,
                                         uint8_t status)
{
//  gapPairStateEvent_t *pData;
//
//  // Allocate space for the passcode event.
//  if ((pData = ICall_malloc(sizeof(gapPairStateEvent_t))))
//  {
//    pData->connectionHandle = connHandle;
//    pData->state = state;
//    pData->status = status;
//
//    // Enqueue the event.
//    simpleTopology_enqueueMsg(MR_PAIRING_STATE_EVT, state ,(uint8_t *) pData);
//  }
}

/*********************************************************************
 * @fn      BLE_AdvertiseEventHandler
 *
 * @brief   Called after every advertise event
 *
 * @param
 *
 * @return  none
 */
static void BLE_AdvertiseEventHandler(void) {


	if (beacon_mode == BEACON_ONLY) {
		adv_counter++;
		if ((adv_counter - 1) % 60 == 0) {
			batteryLev = AONBatMonBatteryVoltageGet();
			batteryLev = (batteryLev * 125) >> 5;
		}

		Climb_updateMyBroadcastedState(BEACON);

		if (childInitModeActive) { // when it's childInitModeActive enable the preAdvEvt so that the discovery is kept active, otherwise after 2 seconds it will be turned off
			//temp_tick_3 = Clock_getTicks();
			Util_startClock(&preAdvClock);
		}
	} else {
		//temp_tick_3 = Clock_getTicks();
		Util_startClock(&preAdvClock);
	}


#if LED_VERBOSITY > 1
	if( childInitModeActive ){
		CLIMB_FlashLed(Board_LED1);
	}else{
		CLIMB_FlashLed(Board_LED2);
	}
#endif
#ifdef PRINTF_ENABLED
		System_printf("\nAdvertise event\n");
#endif

}

/*********************************************************************
 * @fn      BLE_ConnectionEventHandler_sendAttRsp
 *
 * @brief   Send a pending ATT response message.
 *
 * @param   none
 *
 * @return  none
 */
static void BLE_ConnectionEventHandler_sendAttRsp(void) {

	// See if there's a pending ATT Response to be transmitted
	if (pAttRsp != NULL) {
		uint8_t status;

		// Increment retransmission count
		rspTxRetry++;

		// Try to retransmit ATT response till either we're successful or
		// the ATT Client times out (after 30s) and drops the connection.
		status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, &(pAttRsp->msg));
		if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL)) {
			// Disable connection event end notice
			HCI_EXT_ConnEventNoticeCmd(pAttRsp->connHandle, selfEntity, 0);

			// We're done with the response message
			simpleTopology_freeAttRsp(status);
		} else {
			// Continue retrying
			//LCD_WRITE_STRING_VALUE("Rsp send retry:", rspTxRetry, 10, LCD_PAGE6);
		}
	}
}

/*********************************************************************
 * @fn      simpleTopology_freeAttRsp
 *
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void simpleTopology_freeAttRsp(uint8_t status) {
	// See if there's a pending ATT response message
	if (pAttRsp != NULL) {
		// See if the response was sent out successfully
		if (status == SUCCESS) {
			//LCD_WRITE_STRING_VALUE("Rsp sent, retry:", rspTxRetry, 10, LCD_PAGE6);
		} else {
			// Free response payload
			GATT_bm_free(&pAttRsp->msg, pAttRsp->method);

			//LCD_WRITE_STRING_VALUE("Rsp retry failed:", rspTxRetry, 10, LCD_PAGE6);
		}

		// Free response message
		ICall_freeMsg(pAttRsp);

		// Reset our globals
		pAttRsp = NULL;
		rspTxRetry = 0;
	}
}

/*********************************************************************
 * @fn      simpleTopology_eventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  TRUE if safe to deallocate event message, FALSE otherwise.
 */
static uint8_t simpleTopology_eventCB(gapMultiRoleEvent_t *pEvent) {
	// Forward the role event to the application
	if (simpleTopology_enqueueMsg(SBT_STATE_CHANGE_EVT, SUCCESS, (uint8_t *) pEvent)) {
		// App will process and free the event
		return FALSE;
	}

	// Caller should free the event
	return TRUE;
}

/*********************************************************************
 * @fn      simpleTopology_charValueChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void simpleTopology_charValueChangeCB(uint8_t paramID) {
	simpleTopology_enqueueMsg(SBT_CHAR_CHANGE_EVT, paramID, NULL);
}

#ifdef FEATURE_OAD
/*********************************************************************
 * @fn      SimpleBLEPeripheral_processOadWriteCB
 *
 * @brief   Process a write request to the OAD profile.
 *
 * @param   event      - event type:
 *                       OAD_WRITE_IDENTIFY_REQ
 *                       OAD_WRITE_BLOCK_REQ
 * @param   connHandle - the connection Handle this request is from.
 * @param   pData      - pointer to data for processing and/or storing.
 *
 * @return  None.
 */
void SimpleBLEPeripheral_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData)
{
  oadTargetWrite_t *oadWriteEvt = ICall_malloc( sizeof(oadTargetWrite_t) + \
                                             sizeof(uint8_t) * OAD_PACKET_SIZE);

  if ( oadWriteEvt != NULL )
  {
    oadWriteEvt->event = event;
    oadWriteEvt->connHandle = connHandle;

    oadWriteEvt->pData = (uint8_t *)(&oadWriteEvt->pData + 1);
    memcpy(oadWriteEvt->pData, pData, OAD_PACKET_SIZE);

    Queue_enqueue(hOadQ, (Queue_Elem *)oadWriteEvt);

    // Post the application's semaphore.
    Semaphore_post(sem);
  }
  else
  {
    // Fail silently.
  }
}
#endif //FEATURE_OAD

/*********************************************************************
 * @fn      isClimbNode
 *
 * @brief	checks if the gap event is related to a Climb node or not
 *
 * @param	pointer to gap info event
 *
 * @return  Type of node, see ClimbNodeType_t
 */

static ClimbNodeType_t isClimbNode(gapDeviceInfoEvent_t *gapDeviceInfoEvent_a) {
	uint8 index = 0;

	if (gapDeviceInfoEvent_a->eventType == GAP_ADRPT_ADV_SCAN_IND |
		gapDeviceInfoEvent_a->eventType == GAP_ADRPT_ADV_IND      |
		gapDeviceInfoEvent_a->eventType == GAP_ADRPT_ADV_NONCONN_IND) { //il nome è contenuto solo dentro i pacchetti di advertise, è inutile cercarli dentro le scan response

		while (index < gapDeviceInfoEvent_a->dataLen) {
			if (gapDeviceInfoEvent_a->pEvtData[index + 1]
					== GAP_ADTYPE_LOCAL_NAME_COMPLETE) { //ho trovato il nome del dispositivo

				if (memcomp(&(gapDeviceInfoEvent_a->pEvtData[index + 2]),&(defAdvertData[2]),(gapDeviceInfoEvent_a->pEvtData[index]) - 1) == 0) { //il nome è compatibile con CLIMB

					return CLIMB_CHILD_NODE;

				}else if(memcomp(&(gapDeviceInfoEvent_a->pEvtData[index + 2]),&Climb_masterNodeName[0],(gapDeviceInfoEvent_a->pEvtData[index]) - 1) == 0 ){ //TODO: verificare...
					return CLIMB_MASTER_NODE;
				}
				else {
					return NOT_CLIMB_NODE; //con questo return blocco la ricerca appena trovo un nome, quindi se dentro il pachetto sono definiti due nomi la funzione riconoscerà solo il primo
				}

			} else { // ricerca il nome nella parte successiva del pacchetto
				index = index + gapDeviceInfoEvent_a->pEvtData[index] + 1;
			}

		}
		return NAME_NOT_PRESENT; //

	} else { //sto cercando il nome dentro un scan response
		return WRONG_PARKET_TYPE; //
	}
}

/*********************************************************************
 * @fn      Climb_addNodeDeviceInfo
 *
 * @brief	adds information related to a gap info event, if the node isn't in the list is added, otherwise if the node is already inside the list its data is updated
 *
 * @return  none
 */
static void Climb_addNodeDeviceInfo( gapDeviceInfoEvent_t *gapDeviceInfoEvent , ClimbNodeType_t nodeType) {

	uint8 node_position = 255;

	if(nodeType == CLIMB_CHILD_NODE){
		node_position = Climb_findNodeById(&gapDeviceInfoEvent->pEvtData[ADV_PKT_ID_OFFSET], nodeType);
	}else if(CLIMB_MASTER_NODE){
		node_position = Climb_findNodeById(gapDeviceInfoEvent->addr, nodeType);
		//return; //for now a master don't need to store other master nodes data, this saves some memory....
	}

	if(node_position == 255){	//dispositivo nuovo, aggiungilo!
		Climb_addNode(gapDeviceInfoEvent,nodeType);
	}else{
		Climb_updateNodeMetadata(gapDeviceInfoEvent,node_position,nodeType);
	}

	return;
}

/*********************************************************************
 * @fn      Climb_findNodeById
 *
 * @brief	Find the node inside child lists (it uses only the ID information) and returns the node pointer. If the node cannot be found this function returns null. If the list is empty it returns null.
 *
 * @return  none
 */
static uint8 Climb_findNodeById(uint8 *nodeID, ClimbNodeType_t nodeType) {

	uint8 i;
	if (nodeType == CLIMB_CHILD_NODE) {

		for (i = 0; i < MAX_SUPPORTED_CHILD_NODES; i++) {
			if (memcomp(nodeID, childListArray[i].id, CHILD_NODE_ID_LENGTH) == 0) {
				return i;
			}
		}

	} else if (nodeType == CLIMB_MASTER_NODE) {
		for (i = 0; i < MAX_SUPPORTED_MASTER_NODES; i++) {
			if (memcomp(nodeID, masterListArray[i].id, MASTER_NODE_ID_LENGTH) == 0) {
				return i;
			}
		}

	}
	return 255;

}

/*********************************************************************
 * @fn      Climb_addNode
 *
 * @brief	Adds a node at the end of the child or master list with informations contained in the gap info event. It automatically manages the adding of first node which is critical because it is referenced by childListRootPtr or masterListRootPtr
 *
 * @return  The pointer to the node istance inside the list
 */
static uint8 Climb_addNode(gapDeviceInfoEvent_t *gapDeviceInfoEvent, ClimbNodeType_t nodeType) {

	if (gapDeviceInfoEvent->eventType == GAP_ADRPT_ADV_SCAN_IND | gapDeviceInfoEvent->eventType == GAP_ADRPT_ADV_IND | gapDeviceInfoEvent->eventType == GAP_ADRPT_ADV_NONCONN_IND) {	//adv data
		uint8 freePos = 0;
		if (nodeType == CLIMB_CHILD_NODE) {

			if (gapDeviceInfoEvent->eventType == GAP_ADRPT_ADV_NONCONN_IND) { //child nodes uses only non-connectable advertising, when they are connectable they are in init-mode, and they should not be monitored

				while (isNonZero(childListArray[freePos].id, CHILD_NODE_ID_LENGTH)) {

					freePos++;

					if (freePos >= MAX_SUPPORTED_CHILD_NODES) {
						return 255;
					}

					if (gapDeviceInfoEvent->pEvtData[ADV_PKT_STATE_OFFSET] > 5) {
						return 255;
					}

				}

				childListArray[freePos].rssi = gapDeviceInfoEvent->rssi;
				childListArray[freePos].lastContactTicks = Clock_getTicks();

				memcpy(childListArray[freePos].id, &gapDeviceInfoEvent->pEvtData[ADV_PKT_ID_OFFSET], CHILD_NODE_ID_LENGTH);
				childListArray[freePos].state = (ChildClimbNodeStateType_t) gapDeviceInfoEvent->pEvtData[ADV_PKT_STATE_OFFSET];


				return 1;

			} else {

				return 255;
			}
		} else if (nodeType == CLIMB_MASTER_NODE) {

			while (isNonZero(masterListArray[freePos].id, MASTER_NODE_ID_LENGTH)) {
				freePos++;

				if (freePos >= MAX_SUPPORTED_MASTER_NODES) {
					return 255;
				}

			}

			masterListArray[freePos].rssi = gapDeviceInfoEvent->rssi;
			masterListArray[freePos].lastContactTicks = Clock_getTicks();

			memcpy(masterListArray[freePos].id, &gapDeviceInfoEvent->addr, MASTER_NODE_ID_LENGTH);
			masterListArray[freePos].state = INVALID_STATE;

			return 1;
		}
		return 0;
	}else{ // this event is a scan response, climb system uses only adv data
		return 255;
	}
}

/*********************************************************************
 * @fn      Climb_updateNodeMetadata
 *
 * @brief	updates targetNode metadata with infomation contained in gapDeviceInfoEvent
 *
 * @return  none
 */
static void Climb_updateNodeMetadata(gapDeviceInfoEvent_t *gapDeviceInfoEvent, uint8 index, ClimbNodeType_t nodeType) {

	if (gapDeviceInfoEvent->eventType == GAP_ADRPT_ADV_SCAN_IND | gapDeviceInfoEvent->eventType == GAP_ADRPT_ADV_IND | gapDeviceInfoEvent->eventType == GAP_ADRPT_ADV_NONCONN_IND) {//adv data

		if (nodeType == CLIMB_CHILD_NODE) {

			childListArray[index].rssi = gapDeviceInfoEvent->rssi;
			childListArray[index].lastContactTicks = Clock_getTicks();

			memcpy(childListArray[index].id, &gapDeviceInfoEvent->pEvtData[ADV_PKT_ID_OFFSET], CHILD_NODE_ID_LENGTH);
			childListArray[index].state = (ChildClimbNodeStateType_t) gapDeviceInfoEvent->pEvtData[ADV_PKT_STATE_OFFSET];

			return;

		} else if (nodeType == CLIMB_MASTER_NODE) {

			masterListArray[index].rssi = gapDeviceInfoEvent->rssi;
			masterListArray[index].lastContactTicks = Clock_getTicks();

			memcpy(masterListArray[index].id, &gapDeviceInfoEvent->addr, MASTER_NODE_ID_LENGTH);
			masterListArray[index].state = INVALID_STATE;

			return;

		}

	} else if (gapDeviceInfoEvent->eventType == GAP_ADRPT_SCAN_RSP) {	//scan response data

	}

	return;

}

/*********************************************************************
 * @fn      Climb_checkNodeState
 *
 * @brief   Checks if the state of this node is the same as the one broadcasted by the master, if not it call Climb_updateMyBroadcastedState. Moreover it saves myMasterId!
 *
 * @return  1 if the state has been updated, 0 if not.
 */

static uint8 Climb_checkNodeState(gapDeviceInfoEvent_t *gapDeviceInfoEvent_a) {
	ChildClimbNodeStateType_t broadcastedState;

	//OBTAIN broadcastedState and check where the message is from (myMaster or not)
	if (nodeState == CHECKING || nodeState == ON_BOARD || nodeState == ALERT) {
		if (memcomp(gapDeviceInfoEvent_a->addr, myMasterId, B_ADDR_LEN) == 0) { //sto analizzando un adv del MIO master
			broadcastedState = Climb_findMyBroadcastedState(gapDeviceInfoEvent_a);
		} else {	//sto analizzando l'adv di un altro master,
			return 0;
		}
	}else{ //se sono in by myself cerca in tutti i master visibili (il primo master che mi farà fare il checking diventerà il MIO master)
		broadcastedState = Climb_findMyBroadcastedState(gapDeviceInfoEvent_a);
	}

	//VERIFY STATE TRANSITIONS
	//if( broadcastedState != nodeState || broadcastedState == INVALID_STATE){
	uint8 i;
	switch (broadcastedState) {
	case BY_MYSELF: //CHECK OUT
		for (i = 0; i < MASTER_NODE_ID_LENGTH; i++) { //resetta l'indirizzo del nodo master
			myMasterId[i] = 0;
		}
		if(nodeState == GOING_TO_SLEEP){
			Util_restartClock(&goToSleepClock, GOTOSLEEP_DEFAULT_TIMEOUT_SEC*1000);
		}
		//if (nodeState == ON_BOARD || nodeState == ALERT) { //il passaggio da ON_BOARD o (ALERT) a BY_MYSELF triggera lo spegnimento del nodo
			//Util_restartClock(&goToSleepClock, 20000); //schedula lo spegnimento automatico
			//readyToGoSleeping = 1;
		//}
		//}
		break;
	case CHECKING:
		if (nodeState == BY_MYSELF) {
			//if (readyToGoSleeping == 0) {
			memcpy(myMasterId, gapDeviceInfoEvent_a->addr, MASTER_NODE_ID_LENGTH); //salva l'id del nodo master

		} else { //se il master sta cercando di mettere il nodo in checking ma questo non era in by myself scarta la richiesta
				 //broadcastedState = nodeState;
			return 0;
		}
		break;

	case ON_BOARD:
		if (nodeState == CHECKING || nodeState == ALERT) { //il passaggio in ON BOARD è permesso solo dagli stati checking e alert

		} else { //se il master sta cercando di mettere il nodo in checking ma questo non era in by myself scarta la richiesta
				 //broadcastedState = nodeState;
			return 0;
		}
		break;

	case ALERT:
		return 0;
		//break;

	case GOING_TO_SLEEP:
		if (nodeState == ON_BOARD || nodeState == CHECKING) { //il passaggio da ON_BOARD o (ALERT) a GOING_TO_SLEEP triggera lo spegnimento del nodo
			Util_restartClock(&goToSleepClock, GOING_TO_SLEEP_DELAY_AFTER_REQUEST_MSEC); //schedula lo spegnimento automatico
			//readyToGoSleeping = 1;
		} else {
			//broadcastedState = nodeState;
			return 0;
		}
		break;

	case INVALID_STATE:				//check to see if there is a broadcast message in this packet
		if (nodeState == BY_MYSELF && childInitModeActive == FALSE) { //broadcast message can be received only if I have a myMaster Node or if I'm in BEACON mode
			//broadcastedState = nodeState;
			return 0;
		} else {
			Climb_decodeBroadcastedMessage(gapDeviceInfoEvent_a);
			return 0;
		}
		//break;

	default:
		return 0;
		//break;
	}

	//quando il sistema è in modalità debug si aggiorna solo la variabile nodeState, la funzione Climb_updateMyBroadcastedState che aggiorna anche l'adv verrà chiamata dalla adv_event_handler
	nodeState = broadcastedState;

	return 1;
	//}
	//return 0;
}

/*********************************************************************
 * @fn      Climb_findMyBroadcastedState
 *
 * @brief   Search within broadcasted data to find the state master wants for this node.
 * 			It also find broadcast state (check in all, check out all).
 *
 * @param   none.
 *
 * @return  The found state, INVALID_STATE if not found
 */
static ChildClimbNodeStateType_t Climb_findMyBroadcastedState(gapDeviceInfoEvent_t *gapDeviceInfoEvent_a) {
	uint8 index = 0;

	if (  gapDeviceInfoEvent_a->eventType == GAP_ADRPT_ADV_SCAN_IND
		| gapDeviceInfoEvent_a->eventType == GAP_ADRPT_ADV_IND
		| gapDeviceInfoEvent_a->eventType == GAP_ADRPT_ADV_NONCONN_IND) { //lo stato è contenuto solo dentro i pacchetti di advertise, è inutile cercarli dentro le scan response

		while (index < gapDeviceInfoEvent_a->dataLen) {
			if (gapDeviceInfoEvent_a->pEvtData[index + 1] == GAP_ADTYPE_MANUFACTURER_SPECIFIC) { //ho trovato il campo GAP_ADTYPE_MANUFACTURER_SPECIFIC

				uint8 manufacter_specific_field_end = index + gapDeviceInfoEvent_a->pEvtData[index]; //=15
				index = index + 4; //salto i campi length, adtype_flag e manufacter ID
				//uint8 temp_ID[]= myIDArray;//{myAddr[0]};//{myAddr[1] , myAddr[0]}; //address bytes must be flipped (for little/big endian stuf)
				while (index + 1 < manufacter_specific_field_end) {

					if (memcomp(myIDArray, &gapDeviceInfoEvent_a->pEvtData[index], CHILD_NODE_ID_LENGTH) == 0) { //MESSAGE ADDRESSED TO THIS NODE
						return (ChildClimbNodeStateType_t) (gapDeviceInfoEvent_a->pEvtData[index + 1]);

					} else if(memcomp(broadcastID, &gapDeviceInfoEvent_a->pEvtData[index], CHILD_NODE_ID_LENGTH) == 0) { // BROADCAST MESSAGE FOUND

						if(gapDeviceInfoEvent_a->pEvtData[index + 1] == BROADCAST_MSG_TYPE_STATE_UPDATE_CMD){
							if (!childInitModeActive){
								return (ChildClimbNodeStateType_t) (gapDeviceInfoEvent_a->pEvtData[index + 2]); //return broadcasted state
							}else{
								return INVALID_STATE; //a BROADCAST_MSG_TYPE_STATE_UPDATE_CMD has been found, but the device is now in child init mode, then no state change is allowed
							}
						}else{
							return INVALID_STATE; //a broadcast message has been found but it is not BROADCAST_MSG_TYPE_STATE_UPDATE_CMD, then return since no other message is contained in this adv packet
						}

					}else{

						index = index + CHILD_NODE_ID_LENGTH + 1;
					}

				}

				return INVALID_STATE; //questo blocca la ricerca una volta trovato il flag GAP_ADTYPE_MANUFACTURER_SPECIFIC, quindi se ce ne fossero due il sistema vede solo il primo
			} else { // ricerca il nome nella parte successiva del pacchetto
				index = index + gapDeviceInfoEvent_a->pEvtData[index] + 1;
			}

		}
		return INVALID_STATE; //

	} else { //sto cercando il nome dentro un scan response
		return INVALID_STATE; //
	}
}
/*********************************************************************
 * @fn      Climb_decodeBroadcastedMessage
 *
 * @brief   Search within broadcasted data to find message sent to broadcast ID, if it is found the relative command is launced
 *
 * @param   none.
 *
 * @return  TRUE if some broadcast message have been found, FALSE otherwise
 */
static uint8 Climb_decodeBroadcastedMessage(gapDeviceInfoEvent_t *gapDeviceInfoEvent_a) {
	uint8 index = 0;

	if (gapDeviceInfoEvent_a->eventType == GAP_ADRPT_ADV_SCAN_IND | gapDeviceInfoEvent_a->eventType == GAP_ADRPT_ADV_IND | gapDeviceInfoEvent_a->eventType == GAP_ADRPT_ADV_NONCONN_IND) { //lo stato è contenuto solo dentro i pacchetti di advertise, è inutile cercarli dentro le scan response

		while (index < gapDeviceInfoEvent_a->dataLen) {
			if (gapDeviceInfoEvent_a->pEvtData[index + 1] == GAP_ADTYPE_MANUFACTURER_SPECIFIC) { //ho trovato il campo GAP_ADTYPE_MANUFACTURER_SPECIFIC

				uint8 manufacter_specific_field_end = index + gapDeviceInfoEvent_a->pEvtData[index] - 1; //-1 is beacause the last byte is the counter
				index += 4; //salto i campi length, adtype_flag e manufacter ID
				if (memcomp(broadcastID, &gapDeviceInfoEvent_a->pEvtData[index], CHILD_NODE_ID_LENGTH) == 0) { // BROADCAST MESSAGE FOUND, note that the broadcast message must have the broadcast ID in the first position, otherwise it will be discarded
					uint8 ret = FALSE;
					while (index < manufacter_specific_field_end && index < 29) {

						uint8 broadcastMsgType = gapDeviceInfoEvent_a->pEvtData[index + 1];
						uint32 wakeUpTimeout_sec_local;
						//float randDelay_msec;

						switch (broadcastMsgType) {
						case BROADCAST_MSG_TYPE_STATE_UPDATE_CMD:
							//THIS MESSAGE IS DECODED IN Climb_findMyBroadcastedState(...);
							if (!childInitModeActive) { //BROADCAST_MSG_TYPE_STATE_UPDATE_CMD is valid only if childInitModeActive is not active
								ret = TRUE;
							} else {
								//ret = ret //keep the previous value
							}
							index = index + BROADCAST_MSG_LENGTH_STATE_UPDATE_CMD;

							break;

						case BROADCAST_MSG_TYPE_WU_SCHEDULE_CMD:
							wakeUpTimeout_sec_local = ((gapDeviceInfoEvent_a->pEvtData[index + 2]) << 16) + ((gapDeviceInfoEvent_a->pEvtData[index + 3]) << 8)
									+ (gapDeviceInfoEvent_a->pEvtData[index + 4]);
							Climb_setWakeUpClock(wakeUpTimeout_sec_local);

							ret = TRUE;
							index = index + BROADCAST_MSG_LENGTH_WU_SCHEDULE_CMD;
							break;

						case BROADCAST_MSG_TYPE_MODE_CHANGE_CMD:
							if (childInitModeActive) { // BROADCAST_MSG_TYPE_MODE_CHANGE_CMD is valid only during the child init mode
								uint8 snv_to_store_conf[STORED_CONFIGURATION_TOTAL_LENGTH];
								uint8 snv_stored_conf[STORED_CONFIGURATION_TOTAL_LENGTH];

								switch (gapDeviceInfoEvent_a->pEvtData[index + 2]) {
								case (uint8)BEACON_ONLY:	//only accept valid data
								case (uint8)COMBO_MODE:
									//save the new configuration in ram
									beacon_mode = (ClimbBeaconMode_t)gapDeviceInfoEvent_a->pEvtData[index + 2];
									snv_to_store_conf[STORED_CONFIGURATION_MODE_OFFSET] = (ClimbBeaconMode_t)gapDeviceInfoEvent_a->pEvtData[index + 2];
									memcpy(&(snv_to_store_conf[STORED_CONFIGURATION_NODE_ID_OFFSET]), myIDArray, CHILD_NODE_ID_LENGTH);
									//retrieve the stored configuration

									osal_snv_read(SNV_BASE_ID+CONFIGURATION_SNV_OFFSET_ID, STORED_CONFIGURATION_TOTAL_LENGTH, snv_stored_conf);
									if(snv_stored_conf[STORED_CONFIGURATION_MODE_OFFSET] != (uint8)beacon_mode){ //if the stored and the new configurations don't match store the new version in flash
										osal_snv_write(SNV_BASE_ID+CONFIGURATION_SNV_OFFSET_ID, STORED_CONFIGURATION_TOTAL_LENGTH, snv_to_store_conf);
									}

									if( beacon_mode == BEACON_ONLY ){
										Climb_updateMyBroadcastedState(BEACON);
									}else if( beacon_mode == COMBO_MODE ){
										Climb_updateMyBroadcastedState(BY_MYSELF);
									}
									ret = TRUE;
									break;

								default:
									//BEACON MODE FIELD CONTAINS A NON-VALID VALUE
									break;
								}
							}
							index = index + BROADCAST_MSG_LENGTH_MODE_CHANGE_CMD;
							break;

						default:
							//should not reach here, but if it is the case, exit
							return FALSE;
							break;
						}
					}
					if (childInitModeActive && ret) { //if at least one broadcast packet is received
						Climb_exitChildInitMode();
					}
					return ret;
				} else {
					return FALSE;
					//index = index + NODE_ID_LENGTH + 1;
				}
			} else { // ricerca il nome nella parte successiva del pacchetto
				index += gapDeviceInfoEvent_a->pEvtData[index] + 1;
			}

		}
		return FALSE; //

	} else { //sto cercando il nome dentro un scan response
		return FALSE; //
	}
}
/*********************************************************************
 * @fn      Climb_updateMyBroadcastedState
 *
 * @brief   Updated the node state and adv data
 *
 * @param   new state
 *
 * @return  none
 */

static void Climb_updateMyBroadcastedState(ChildClimbNodeStateType_t newState) {

	if(childInitModeActive){
		GAP_UpdateAdvertisingData(selfEntity, true, sizeof(connAdvertData), connAdvertData);
	}else{

	uint8 newAdvertData[31];

		nodeState = newState;

		newAdvertData[0] = 0x07; // length of this data
		newAdvertData[1] = GAP_ADTYPE_LOCAL_NAME_COMPLETE;
		newAdvertData[2] = defAdvertData[2];
		newAdvertData[3] = defAdvertData[3];
		newAdvertData[4] = defAdvertData[4];
		newAdvertData[5] = defAdvertData[5];
		newAdvertData[6] = defAdvertData[6];
		newAdvertData[7] = defAdvertData[7];
		//newAdvertData[8] = i-9; assigned later
		newAdvertData[9] = GAP_ADTYPE_MANUFACTURER_SPECIFIC;
		newAdvertData[10] = 0x0D;
		newAdvertData[11] = 0x00;
		memcpy(&newAdvertData[ADV_PKT_ID_OFFSET], myIDArray, CHILD_NODE_ID_LENGTH);
		newAdvertData[ADV_PKT_STATE_OFFSET] = (uint8) nodeState;

		uint8 i = ADV_PKT_STATE_OFFSET + 1;
		uint8 nodeArrayIndex = adv_startNodeIndex;
		uint8 roundCompleted = FALSE;

		while (i < 27 && roundCompleted == FALSE) {

			if (isNonZero(childListArray[nodeArrayIndex].id, CHILD_NODE_ID_LENGTH)) { //discard empty spaces

				memcpy(&newAdvertData[i], childListArray[nodeArrayIndex].id, CHILD_NODE_ID_LENGTH);
				i += CHILD_NODE_ID_LENGTH;
				newAdvertData[i++] = childListArray[nodeArrayIndex].rssi;

			}

			nodeArrayIndex++;

			if (nodeArrayIndex >= MAX_SUPPORTED_CHILD_NODES) {
				nodeArrayIndex = 0;
			}

			if (nodeArrayIndex == adv_startNodeIndex) {
				roundCompleted = TRUE;
			}

		}
		adv_startNodeIndex = nodeArrayIndex;

		newAdvertData[i++] = (uint8) (batteryLev >> 8);
		newAdvertData[i++] = (uint8) (batteryLev);
		newAdvertData[i++] = adv_counter; //the counter is always in the last position
		newAdvertData[8] = i - 9;

		//temp_tick_1 = Clock_getTicks();
		GAP_UpdateAdvertisingData(selfEntity, true, i, &newAdvertData[0]);

	}
	//GAP_UpdateAdvertisingData(selfEntity, false, 13,	&advertData[0]);

	return;
}
/*********************************************************************
 * @fn      Climb_nodeTimeoutCheck
 *
 * @brief	Check the timeout for every node contained in child and master lists. If timeout is expired the node is removed.
 *
 *
 * @return  none
 */
static void Climb_nodeTimeoutCheck(void) {
#ifdef PRINTF_ENABLED
	System_printf("\nRunning timeout check!\n");
#endif
	uint32 nowTicks = Clock_getTicks();
	uint16 i = 0;

	//controlla la lista dei child
	while (i < MAX_SUPPORTED_CHILD_NODES) { //NB: ENSURE targetNode IS UPDATED ANY CYCLE, OTHERWISE IT RUNS IN AN INFINITE LOOP
		if (isNonZero(childListArray[i].id, CHILD_NODE_ID_LENGTH)) { //discard empty spaces

			if (nowTicks - childListArray[i].lastContactTicks > NODE_TIMEOUT_OS_TICKS) {

				Climb_removeNode(i,CLIMB_CHILD_NODE); //rimuovi il nodo

			} else { //il nodo non è andato in timeout,

			}
			//NB: ENSURE targetNode IS UPDATED ANY CYCLE, OTHERWISE IT RUNS IN AN INFINITE LOOP
		}
		i++;
	}

	//controlla la lista dei master
	i = 0;
	while (i < MAX_SUPPORTED_MASTER_NODES) { //NB: ENSURE targetNode IS UPDATED ANY CYCLE, OTHERWISE IT RUNS IN AN INFINITE LOOP
		if (isNonZero(masterListArray[i].id, CHILD_NODE_ID_LENGTH)) { //discard empty spaces
			if (nowTicks - masterListArray[i].lastContactTicks > NODE_TIMEOUT_OS_TICKS) {

				//RIMUOVI SOLO I MASTER CHE NON SONO MY_MASTER
				if (memcomp(masterListArray[i].id, myMasterId, MASTER_NODE_ID_LENGTH) != 0) {
					Climb_removeNode(i,CLIMB_MASTER_NODE); //rimuovi il nodo
				} else {
					switch (nodeState) {
					case BY_MYSELF:
						//should not reach here since when this node is in BY_MYSELF the myMasterId should be non-valid, for safety reason I add the next line
						Climb_removeNode(i,CLIMB_MASTER_NODE); //rimuovi il nodo
						break;

					case CHECKING:
						nodeState = BY_MYSELF; //se dopo essere stato passato a checking non vedo più il master torna in BY_MYSELF
						uint8 i;
						for (i = 0; i < MASTER_NODE_ID_LENGTH; i++) { //resetta l'indirizzo del MY_MASTER
							myMasterId[i] = 0;
						}
						Climb_removeNode(i,CLIMB_MASTER_NODE); //rimuovi il nodo
						break;

					case ON_BOARD:
						//nodeState = ALERT; //se non ho visto il mio master vai in alert (solo se ero nello stato ON_BOARD)
						Climb_updateMyBroadcastedState(ALERT);
						break;

					case ALERT:
						//no nothing
						break;

					default:
						break;
					}

				}
			} else { //il nodo non è andato in timeout,

			}
		}

		i++;
		//NB: ENSURE targetNode IS UPDATED ANY CYCLE, OTHERWISE IT RUNS IN AN INFINITE LOOP
	}
}

/*********************************************************************
 * @fn      Climb_removeNode
 *
 * @brief	Removes nodeToRemove node form the list which contains it
 *
 * @return  pointer to the next node
 */
static void Climb_removeNode(uint8 indexToRemove, ClimbNodeType_t nodeType) {

	uint8 i = 0;

	for (i = 0; i < sizeof(myGapDevRec_t); i++) {

		if (nodeType == CLIMB_CHILD_NODE) {

			((uint8*) (&childListArray[indexToRemove]))[i] = 0;

		} else if (nodeType == CLIMB_MASTER_NODE) {

			((uint8*) (&masterListArray[indexToRemove]))[i] = 0;

		}

	}

	return;

}

/*********************************************************************
 * @fn      destroyChildNodeList
 *
 * @brief   Destroy child list
 *

 * @return  none
 */
static void destroyChildNodeList() {

	uint16 arraySize = MAX_SUPPORTED_CHILD_NODES*sizeof(myGapDevRec_t);
	uint16 i = 0;
	//resetta la memoria
	for(i = 0; i < arraySize; i++ ){
		((uint8*)childListArray)[i] = 0;
	}
}
/*********************************************************************
 * @fn      destroyMasterNodeList
 *
 * @brief   Destroy master list
 *

 * @return  none
 */
static void destroyMasterNodeList(void) {

	uint16 arraySize = MAX_SUPPORTED_MASTER_NODES*sizeof(myGapDevRec_t);
	uint16 i = 0;
	//resetta la memoria
	for(i = 0; i < arraySize; i++ ){
		((uint8*)masterListArray)[i] = 0;
	}

}

/*********************************************************************
 * @fn      Climb_enterChildInitMode
 *
 * @brief
 *

 * @return  none
 */
static void Climb_enterChildInitMode(void){

	//HCI_EXT_AdvEventNoticeCmd(selfEntity, 0);
	//Util_stopClock(&preAdvClock);
	childInitModeActive = TRUE;

	uint8 adv_active;
	if(nodeTurnedOn){
		//HCI_EXT_AdvEventNoticeCmd(selfEntity, 0);
		adv_active = 0;
		GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t), &adv_active, NULL);
	}

	GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(connAdvertData), connAdvertData, NULL);

    if (Util_isActive(&connectableTimeoutClock)){
    	Util_stopClock(&connectableTimeoutClock);
    }
    Util_restartClock(&connectableTimeoutClock, CONNECTABLE_TIMEOUT_MSEC);

    uint8 status = GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE, DEFAULT_DISCOVERY_ACTIVE_SCAN, DEFAULT_DISCOVERY_WHITE_LIST); // Discovery is started so that it is possible to set the clock and eventually the ID
    scanning = TRUE;

	adv_active = 1;
	GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &adv_active, NULL);
	HCI_EXT_AdvEventNoticeCmd(selfEntity, ADVERTISE_EVT);

}

/*********************************************************************
 * @fn      Climb_exitChildInitMode
 *
 * @brief
 *

 * @return  none
 */
static void Climb_exitChildInitMode(){

	childInitModeActive = FALSE;

	GAPRole_CancelDiscovery();

	uint8 adv_active = 0;
	uint8 status = GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &adv_active, NULL);

    if (Util_isActive(&connectableTimeoutClock)){
    	Util_stopClock(&connectableTimeoutClock);
    }

	GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(defAdvertData), defAdvertData, NULL);

	if(nodeTurnedOn){
		adv_active = 1;
		status = GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t), &adv_active, NULL);
		HCI_EXT_AdvEventNoticeCmd(selfEntity, ADVERTISE_EVT);
	}
	if (beacon_mode == BEACON_ONLY) {
		destroyChildNodeList();
		destroyMasterNodeList();
	}

}







/*********************************************************************
 * @fn      Climb_periodicTask
 *
 * @brief	Handler associated with Periodic Clock instance.
 *
 * @return  none
 */
static void Climb_periodicTask(void){
	Climb_nodeTimeoutCheck();
#ifdef PRINTF_ENABLED
#ifdef HEAPMGR_METRICS

	uint16 *pBlkMax;
	uint16 *pBlkCnt;
	uint16 *pBlkFree;
	uint16	*pMemAlo;
	uint16 *pMemMax;
	uint16 *pMemUb;

	ICall_heapGetMetrics(pBlkMax, pBlkCnt, pBlkFree, pMemAlo, pMemMax, pMemUb);

	System_printf("\nMax allocated(blocks): %d\n"
			      "Current allocated (blocks): %d\n"
			      "Current free (blocks):%d\n"
		      	  "Current allocated (bytes): %d\n"
	      	  	  "Max allocated (bytes): %d\n"
				  "Furthest allocated: %d\n",
				  *pBlkMax, *pBlkCnt, *pBlkFree, *pMemAlo, *pMemMax, pMemUb);
#endif
#endif
}

static void Climb_preAdvEvtHandler(){
	if(beacon_mode == COMBO_MODE){
#ifdef HIGH_PERFORMANCE
	GAPRole_CancelDiscovery();
	scanning = FALSE;
#endif
	adv_counter++;

	if ((adv_counter - 1) % 60 == 0) {
		batteryLev = AONBatMonBatteryVoltageGet();
		batteryLev = (batteryLev * 125) >> 5;
	}

	Climb_updateMyBroadcastedState(nodeState); //update adv data every adv event to update adv_counter value. Since the argument is nodeState this function call doesn't modify the actual state of this node
	}

	if (!scanning) {
		if (beacon_mode == BEACON_ONLY) {
			if (childInitModeActive) {
				GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE, DEFAULT_DISCOVERY_ACTIVE_SCAN, DEFAULT_DISCOVERY_WHITE_LIST);
				scanning = TRUE;
			}
		} else {
			if (nodeTurnedOn || childInitModeActive) {
				GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE, DEFAULT_DISCOVERY_ACTIVE_SCAN, DEFAULT_DISCOVERY_WHITE_LIST);
				scanning = TRUE;
			}
		}
	}
}
/*********************************************************************
 * @fn      Climb_goToSleepHandler
 *
 * @brief	Handler associated with goToSleep Clock instance.
 *
 * @return  none
 */
static void Climb_goToSleepHandler(){

	stopNode();

	destroyChildNodeList();
	destroyMasterNodeList();

	uint8 i;
	for (i = 0; i < MASTER_NODE_ID_LENGTH; i++) { //resetta l'indirizzo del MY_MASTER
		myMasterId[i] = 0;
	}
	if(beacon_mode == BEACON_ONLY){
		Climb_updateMyBroadcastedState(BEACON);
	}else{
		Climb_updateMyBroadcastedState(BY_MYSELF);
	}

}

/*********************************************************************
 * @fn      Climb_wakeUpHandler
 *
 * @brief	Handler associated with wakeUp Clock instance.
 *
 * @return  none
 */
static void Climb_wakeUpHandler(){

	if(wakeUpTimeout_sec_global == 0){
		startNode();
		Util_restartClock(&goToSleepClock, GOTOSLEEP_DEFAULT_TIMEOUT_SEC*1000);
		Climb_setWakeUpClock(WAKEUP_DEFAULT_TIMEOUT_SEC);

		return;
	}

	if(wakeUpTimeout_sec_global > MAX_ALLOWED_TIMER_DURATION_SEC){
		Util_restartClock(&wakeUpClock, MAX_ALLOWED_TIMER_DURATION_SEC*1000);
		wakeUpTimeout_sec_global = wakeUpTimeout_sec_global - MAX_ALLOWED_TIMER_DURATION_SEC;
	}else{
		//randomizza nell'intorno +/-1 secondo rispetto al valore prestabilito
		float randDelay_msec = 2000 * ((float) Util_GetTRNG()) / 4294967296;
		Util_restartClock(&wakeUpClock, wakeUpTimeout_sec_global*1000 - 1000 + randDelay_msec);
		wakeUpTimeout_sec_global = 0; //reset this so that when the device wakes up, it knows that there is no need to restart timer but it is the actual time to wake up the device
	}

}
/*********************************************************************
 * @fn      Climb_setWakeUpClock
 *
 * @brief	Helper function that sets the wake up clock, it manages the clock overflow for intervals larger than MAX_ALLOWED_TIMER_DURATION_SEC
 *
 * @return  none
 */
static void Climb_setWakeUpClock(uint32 wakeUpTimeout_sec_local){

	if(wakeUpTimeout_sec_local > MAX_ALLOWED_TIMER_DURATION_SEC){ //max timer duration 42949.67sec
		Util_restartClock(&wakeUpClock, MAX_ALLOWED_TIMER_DURATION_SEC*1000);
		wakeUpTimeout_sec_global = wakeUpTimeout_sec_local - MAX_ALLOWED_TIMER_DURATION_SEC;
	}else{
		//randomizza nell'intorno +/-1 secondo rispetto al valore prestabilito
		float randDelay_msec = 2000 * ((float) Util_GetTRNG()) / 4294967296;
		Util_restartClock(&wakeUpClock, wakeUpTimeout_sec_local*1000 - 1000 + randDelay_msec);
		wakeUpTimeout_sec_global = 0; //reset this so that when the device wakes up, it knows that there is no need to restart timer but it is the actual time to wake up the device
	}

}




/*********************************************************************
 * @fn      CLIMB_FlashLed
 *
 * @brief   Turn on a led and start the timer that switch it off;
 *
 * @param   pinId - the led id to turn on

 * @return  none
 */
static void CLIMB_FlashLed(PIN_Id pinId){

	PIN_setOutputValue(hGpioPin, pinId, Board_LED_ON);
	Util_startClock(&ledTimeoutClock); //start the clock that switch the led off

}

/*********************************************************************
 * @fn      Keys_EventCB
 *
 * @brief   Callback from Keys task indicating a role state change.
 *
 * @param   notificationType - type of button press
 *
 * @return  None.
 */
static void Keys_EventCB(keys_Notifications_t notificationType) {

	simpleTopology_enqueueMsg(KEY_CHANGE_EVT, (uint8) notificationType,	NULL);

}

/*********************************************************************
 * @fn      CLIMB_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                HAL_KEY_SW_1
 *
 * @return  none
 */
static void CLIMB_handleKeys(uint8 keys) {

	switch ((keys_Notifications_t) keys) {
	case LEFT_SHORT:
		adv_counter = 0;
		break;

	case RIGHT_SHORT:
		adv_counter = 128;
		break;

	case LEFT_LONG:

		break;

	case RIGHT_LONG:
		if (nodeTurnedOn != 1){
			startNode();

#if LED_VERBOSITY > 0
			CLIMB_FlashLed(Board_LED2);
#endif
			Climb_setWakeUpClock(WAKEUP_DEFAULT_TIMEOUT_SEC);
			Util_restartClock(&goToSleepClock, GOTOSLEEP_DEFAULT_TIMEOUT_SEC*1000);

		}else{ //if manually switched off, no automatic wakeup is setted
			stopNode();

#if LED_VERBOSITY > 0
			CLIMB_FlashLed(Board_LED1);
#endif

			destroyChildNodeList();

			if (nodeState != ON_BOARD) {
				destroyMasterNodeList();

				uint8 i;
				for (i = 0; i < MASTER_NODE_ID_LENGTH; i++) { //resetta l'indirizzo del MY_MASTER
					myMasterId[i] = 0;
				}

				if (beacon_mode == BEACON_ONLY) {
					Climb_updateMyBroadcastedState(BEACON);
				} else {
					Climb_updateMyBroadcastedState(BY_MYSELF);
				}
			} else {
				if (beacon_mode == BEACON_ONLY) {
					Climb_updateMyBroadcastedState(BEACON);
				} else {
					Climb_updateMyBroadcastedState(nodeState);
				}
			}
			Util_stopClock(&wakeUpClock);
			Util_stopClock(&goToSleepClock);
		}
		break;

	case BOTH:
#if LED_VERBOSITY > 0
		CLIMB_FlashLed(Board_LED1);
#endif
		if(!childInitModeActive){
			Climb_enterChildInitMode();
		}else{
			Climb_exitChildInitMode();
		}
		break;

	case REED_SWITCH_LONG:
#if LED_VERBOSITY > 0
		CLIMB_FlashLed(Board_LED1);
#endif
		if(!childInitModeActive){
			Climb_enterChildInitMode();
		}else{
			Climb_exitChildInitMode();
		}
		break;
	default:
		break;
	}
}
/*********************************************************************
 * @fn      startNode
 *
 * @brief   Function to call to start the node.
 *

 * @return  none
 */
static void startNode() {

	if (nodeTurnedOn != 1) {

		nodeTurnedOn = 1;

		uint8 adv_active = 1;
		uint8 status = GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t), &adv_active, NULL);
		HCI_EXT_AdvEventNoticeCmd(selfEntity, ADVERTISE_EVT);

		if(beacon_mode == COMBO_MODE){
			Util_startClock(&periodicClock);
			status |= GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE, DEFAULT_DISCOVERY_ACTIVE_SCAN, DEFAULT_DISCOVERY_WHITE_LIST);
			scanning = TRUE;
		}
	}
}

/*********************************************************************
 * @fn      startNode
 *
 * @brief   Function to call to stop the node.
 *

 * @return  none
 */
static void stopNode() {
	nodeTurnedOn = 0;

	if(childInitModeActive){
		Climb_exitChildInitMode();
	}

	GAPRole_CancelDiscovery();

	//HCI_EXT_AdvEventNoticeCmd(selfEntity, 0);
	scanning = FALSE;
	uint8 adv_active = 0;
	uint8 status = GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t), &adv_active, NULL);

	Util_stopClock(&periodicClock);

}

#ifdef FEATURE_LCD
/*********************************************************************
 * @fn      displayInit
 *
 * @brief
 *
 * @param
 *
 * @return  none
 */
static void displayInit(void) {

	// Initialize LCD
	devpkLcdOpen();
}
#endif
#ifdef WATCHDOGTIMER_EN
static void watchdogTimerInit() {
	GPTimerCC26XX_Params params;
	GPTimerCC26XX_Params_init(&params);
	params.width = GPT_CONFIG_32BIT;
	params.mode = GPT_MODE_PERIODIC_UP;
	params.debugStallMode = GPTimerCC26XX_DEBUG_STALL_OFF;
	hTimer = GPTimerCC26XX_open(CC2650_GPTIMER0A, &params);
	if (hTimer == NULL) {
		//Log_error0("Failed to open GPTimer");
		//Task_exit();
	}

	Types_FreqHz freq;
	BIOS_getCpuFreq(&freq);
	GPTimerCC26XX_Value loadVal = freq.lo / 1 - 1; //47999999
	GPTimerCC26XX_setLoadValue(hTimer, loadVal);
	GPTimerCC26XX_registerInterrupt(hTimer, timerCallback, GPT_INT_TIMEOUT);

}
static void timerCallback(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask) {

	switch (unclearedWatchdogEvents) {
	case 0:
	case 1:
		events |= WATCHDOG_EVT;
		Semaphore_post(sem);
		break;
	case 2:
		unclearedWatchdogEvents--;
		//resetApplication?
		break;
	default:
		//HAL_SYSTEM_RESET();
		//or
		//HCI_EXT_ResetSystemCmd(HCI_EXT_RESET_SYSTEM_HARD);

		break;
	}

	unclearedWatchdogEvents++;
}
#endif

static void saveTimersConf(void){
//	timerSNVDataStore_t timerConfToStore;
//
//
//	//timerConfToStore.wakeUpRemainingTimeout = Util_getClockTimeout(&wakeUpClock);
//	timerConfToStore.wakeUpClock_Struct = wakeUpClock;
//	timerConfToStore.wakeUpTimerActive = Util_isActive(&wakeUpClock);
//	//timerConfToStore.goToSleepRamainingTimeout = Util_getClockTimeout(&goToSleepClock);
//	timerConfToStore.goToSleepClock_Struct = goToSleepClock;
//	timerConfToStore.goToSleepTimerActive = Util_isActive(&goToSleepClock);
//	timerConfToStore.wakeUpTimeout_sec_global_value = wakeUpTimeout_sec_global;
//	timerConfToStore.validData = TRUE;
//
//	osal_snv_write(SNV_BASE_ID+TIMERS_SNV_OFFSET_ID, sizeof(timerSNVDataStore_t), &timerConfToStore);
}

static uint8 restoreTimersConf() {

//	timerSNVDataStore_t timerConfToRestore;
//
//	uint8 ret = osal_snv_write(SNV_BASE_ID + TIMERS_SNV_OFFSET_ID, 0,  &timerConfToRestore);			//this is needed to initialize SNV
//
//	if (ret == SUCCESS) {
//		ret = osal_snv_read(SNV_BASE_ID + TIMERS_SNV_OFFSET_ID, sizeof(timerSNVDataStore_t), &timerConfToRestore);
//
//		if (ret == SUCCESS) {
//
//			if (timerConfToRestore.validData) {
//				Util_stopClock(&wakeUpClock);
//				Util_stopClock(&goToSleepClock);
//
//				//Reload wakeUp timer value
//				wakeUpTimeout_sec_global = timerConfToRestore.wakeUpTimeout_sec_global_value;
//				wakeUpClock = timerConfToRestore.wakeUpClock_Struct;
//				Util_startClock(&wakeUpClock);
//				if (!timerConfToRestore.wakeUpTimerActive) { //if the clock was inactive stop it immediatelly
//					Util_stopClock(&wakeUpClock);
//				}
//
//				//Reload goToSleep timer value
//				goToSleepClock = timerConfToRestore.goToSleepClock_Struct;
//				Util_startClock(&goToSleepClock);
//				if (!timerConfToRestore.goToSleepTimerActive) {
//					Util_stopClock(&goToSleepClock);
//				}
//
//				//unvalidate data so that when no fauls occour it doesn't load an old timers configuration
//				timerConfToRestore.validData = FALSE;
//				osal_snv_write(SNV_BASE_ID + TIMERS_SNV_OFFSET_ID, sizeof(timerSNVDataStore_t), &timerConfToRestore);
//
//				return TRUE;
//			}
//		}
//		return FALSE;
//	}
	return FALSE;
}

/*********************************************************************
 * @fn      Climb_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void Climb_clockHandler(UArg arg) {
	// Store the event.
	events |= arg;

	// Wake up the application.
	Semaphore_post(sem);
}

/*********************************************************************
 * @fn      memcomp
 *
 * @brief   Compares memory locations, same as memcmp, but it returns 0 if two memory sections are identical, 1 otherwise.
 *
 *
 * @return  none
 */

static uint8 memcomp(uint8 * str1, uint8 * str2, uint8 len) { //come memcmp (ma ritorna 0 se è uguale e 1 se è diversa, non dice se minore o maggiore)
	uint8 index;
	for (index = 0; index < len; index++) {
		if (str1[index] != str2[index]) {
			return 1;
		}
	}
	return 0;
}

static uint8 isNonZero(uint8 * str1, uint8 len) { //ritorna 0 se il vettore è zero, 1 altrimenti
	uint8 index;
	for (index = 0; index < len; index++) {
		if (str1[index] != 0) {
			return 1;
		}
	}
	return 0;
}

/*********************************************************************
 * @fn      simpleTopology_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 *
 * @return  None.
 */
static uint8_t simpleTopology_enqueueMsg(uint16_t event, uint8_t status, uint8_t *pData) {
	sbtEvt_t *pMsg = ICall_malloc(sizeof(sbtEvt_t));

	// Create dynamic pointer to message.
	if (pMsg) {
		pMsg->event = event;
		pMsg->status = status;
		pMsg->pData = pData;

		// Enqueue the message.
		return Util_enqueueMsg(appMsgQueue, sem, (uint8*) pMsg);
	}else{
		//CLIMB_FlashLed(Board_LED1);
	}

	return FALSE;
}
#ifdef  HEAPMGR_METRICS
static void plotHeapMetrics() {

#ifdef PRINTF_ENABLED
	System_printf("\nHEAP METRICS\n");
#endif
	uint16_t pBlkMax[1];
	uint16_t pBlkCnt[1];
	uint16_t pBlkFree[1];
	uint16_t pMemAlo[1];
	uint16_t pMemMax[1];
	uint16_t pMemUb[1];
	uint16_t pFailAm[1];
	ICall_heapGetMetrics(pBlkMax, pBlkCnt,pBlkFree,pMemAlo,pMemMax,pMemUb,pFailAm);

#ifdef PRINTF_ENABLED
	System_printf("max cnt of all blocks ever seen at once: %d\ncurrent cnt of all blocks: %d\ncurrent cnt of free blocks: %d\ncurrent total memory allocated: %d\nmax total memory ever allocated at once: %d\nupper bound of memory usage: %d\namount of failed malloc calls: %d\n\n\n", pBlkMax[0], pBlkCnt[0], pBlkFree[0],pMemAlo[0], pMemMax[0], pMemUb[0],pFailAm[0] );
#endif

}
#endif

#ifdef STACK_METRICS
static void checkStackMetrics() {
	static uint16 maxStackUse = 0;

	Task_Stat statbuf; /* declare buffer */
	Task_stat(Task_self(), &statbuf); /* call func to get status */
	System_printf("\nSTACK METRICS\n");

	if(statbuf.used > maxStackUse) {
		maxStackUse = statbuf.used;
	}
#ifdef PRINTF_ENABLED
	System_printf("stack size: %d, stack pointer: 0x%x, stack base: 0x%x, used: %d, absolute max used: %d\n", statbuf.stackSize ,statbuf.sp, statbuf.stack , statbuf.used, maxStackUse);
	if (statbuf.used > (statbuf.stackSize * 9 / 10)) {
		System_printf("Over 90% of task's stack is in use.\n");
	}
#endif

	//RESET UNUSED STACK
	uint32 i;

	for(i = 0; i < (statbuf.sp - statbuf.stack)-1; i++) { //-1 for safety, don't care warning
		sbpTaskStack[i] = 0xBE;
	}

}
#endif

/*********************************************************************
 * @fn      exceptionHandler
 *
 * @brief   Faults handler
 *
 * @param
 *
 * @return  none
 */
/**
 * Exception handler
 */
struct exceptionFrame
{
	unsigned int _r4;
	unsigned int _r5;
	unsigned int _r6;
	unsigned int _r7;
	unsigned int _r8;
	unsigned int _r9;
	unsigned int _r10;
	unsigned int _r11;
	unsigned int _r0;
	unsigned int _r1;
	unsigned int _r2;
	unsigned int _r3;
	unsigned int _r12;
	unsigned int _lr;
	unsigned int _pc;
	unsigned int _xpsr;
};

void exceptionHandler(struct exceptionFrame *e, unsigned int execlR){


	saveTimersConf();

	while(1){
		HAL_SYSTEM_RESET();
	}
}


/*********************************************************************
 *********************************************************************/
