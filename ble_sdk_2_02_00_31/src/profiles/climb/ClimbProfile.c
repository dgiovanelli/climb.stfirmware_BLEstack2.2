/**************************************************************************************************
 Filename:       simpleGATTprofile.c
 Revised:        $Date: 2015-07-20 11:31:07 -0700 (Mon, 20 Jul 2015) $
 Revision:       $Revision: 44370 $

 Description:    This file contains the Simple GATT profile sample GATT service
 profile for use with the BLE sample application.

 Copyright 2010 - 2015 Texas Instruments Incorporated. All rights reserved.

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
 **************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp_giova.h"
#include "gapbondmgr.h"

#include "ClimbProfile.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SERVAPP_NUM_ATTR_SUPPORTED        8

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Simple GATT Profile Service UUID: 0xFFF0
CONST uint8 ClimbProfileServUUID[ATT_BT_UUID_SIZE] = { LO_UINT16(CLIMBPROFILE_SERV_UUID), HI_UINT16(CLIMBPROFILE_SERV_UUID) };

// Characteristic 1 UUID: 0xFFF1
CONST uint8 climbProfilechar1UUID[ATT_BT_UUID_SIZE] = { LO_UINT16(CLIMBPROFILE_CHAR1_UUID), HI_UINT16(CLIMBPROFILE_CHAR1_UUID) };

// Characteristic 1 UUID: 0xFFF1
CONST uint8 climbProfilechar2UUID[ATT_BT_UUID_SIZE] = { LO_UINT16(CLIMBPROFILE_CHAR2_UUID), HI_UINT16(CLIMBPROFILE_CHAR2_UUID) };

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static climbProfileCBs_t *climbProfile_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Simple Profile Service attribute
static CONST gattAttrType_t climbProfileService = { ATT_BT_UUID_SIZE, ClimbProfileServUUID };

// Simple Profile Characteristic 1 Properties
static uint8 climbProfileChar1Props = GATT_PROP_NOTIFY | GATT_PROP_READ; // | GATT_PROP_WRITE;

// Characteristic 1 Value
static uint8 climbProfileChar1[CLIMBPROFILE_CHAR1_LEN];

// Simple Profile Characteristic 1 User Description
static uint8 climbProfileChar1UserDesp[5] = "CIPO";

// Simple Profile Characteristic 1 Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t *climbProfileChar1Config;

// Simple Profile Characteristic 2 Properties
static uint8 climbProfileChar2Props = GATT_PROP_WRITE;

// Characteristic 2 Value
static uint8 climbProfileChar2[CLIMBPROFILE_CHAR2_LEN];

// Simple Profile Characteristic 2 User Description
static uint8 climbProfileChar2UserDesp[5] = "PICO";

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t climbProfileAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] = {
// Simple Profile Service
		{ { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
		GATT_PERMIT_READ, /* permissions */
		0, /* handle */
		(uint8 *) &climbProfileService /* pValue */
		},

		// Characteristic 1 Declaration
		{ { ATT_BT_UUID_SIZE, characterUUID },
		GATT_PERMIT_READ, 0, &climbProfileChar1Props },

		// Characteristic Value 1
		{ { ATT_BT_UUID_SIZE, climbProfilechar1UUID },
		GATT_PERMIT_READ , 0, climbProfileChar1 },

		//GATT_PERMIT_READ | GATT_PERMIT_WRITE, 0, climbProfileChar1 },

		// Characteristic 1 User Description
		{ { ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ, 0, climbProfileChar1UserDesp },

		// Characteristic 1 configuration CCCF
		{ { ATT_BT_UUID_SIZE, clientCharCfgUUID },
		GATT_PERMIT_READ | GATT_PERMIT_WRITE, 0, (uint8 *) &climbProfileChar1Config },

		// Characteristic 2 Declaration
		{ { ATT_BT_UUID_SIZE, characterUUID },
		GATT_PERMIT_READ, 0, &climbProfileChar2Props },

		// Characteristic Value 2
		{ { ATT_BT_UUID_SIZE, climbProfilechar2UUID },
		GATT_PERMIT_WRITE, 0, climbProfileChar2 },

		// Characteristic 2 User Description
		{ { ATT_BT_UUID_SIZE, charUserDescUUID },
		GATT_PERMIT_READ, 0, climbProfileChar2UserDesp },

};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t climbProfile_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr, uint8_t *pValue, uint16_t *pLen, uint16_t offset, uint16_t maxLen, uint8_t method);
static bStatus_t climbProfile_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr, uint8_t *pValue, uint16_t len, uint16_t offset, uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t climbProfileCBs = { climbProfile_ReadAttrCB,  // Read callback function pointer
		climbProfile_WriteAttrCB, // Write callback function pointer
		NULL                      // Authorization callback function pointer
		};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      ClimbProfile_AddService
 *
 * @brief   Initializes the Simple Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t ClimbProfile_AddService(uint32 services) {
	uint8 status;

	// Allocate Client Characteristic Configuration table
	climbProfileChar1Config = (gattCharCfg_t *) ICall_malloc(sizeof(gattCharCfg_t) * linkDBNumConns);
	if (climbProfileChar1Config == NULL) {
		return ( bleMemAllocError);
	}

	// Initialize Client Characteristic Configuration attributes
	GATTServApp_InitCharCfg( INVALID_CONNHANDLE, climbProfileChar1Config);

	if (services & CLIMBPROFILE_SERVICE) {
		// Register GATT attribute list and CBs with GATT Server App
		status = GATTServApp_RegisterService( climbProfileAttrTbl,
											  GATT_NUM_ATTRS(climbProfileAttrTbl),
											  GATT_MAX_ENCRYPT_KEY_SIZE,
											  &climbProfileCBs);
	} else {
		status = SUCCESS;
	}

	return (status);
}

/*********************************************************************
 * @fn      ClimbProfile_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call 
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t ClimbProfile_RegisterAppCBs(climbProfileCBs_t *appCallbacks) {
	if (appCallbacks) {
		climbProfile_AppCBs = appCallbacks;

		return ( SUCCESS);
	} else {
		return ( bleAlreadyInRequestedMode);
	}
}

/*********************************************************************
 * @fn      ClimbProfile_SetParameter
 *
 * @brief   Set a Simple Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t ClimbProfile_SetParameter(uint8 param, uint8 len, void *value) {
	bStatus_t ret = SUCCESS;
	switch (param) {
	case CLIMBPROFILE_CHAR1:

		if (len <= CLIMBPROFILE_CHAR1_LEN) {
//correct way, the len parameter is not used, the notification will be MAX_MTU_SIZE - 3
			VOID memcpy(climbProfileChar1, value, len); //save locally
			ICall_free(value);
			//GATT_bm_free((gattMsg_t *) value, ATT_HANDLE_VALUE_NOTI);

			ret = GATTServApp_ProcessCharCfg_giova(climbProfileChar1Config, climbProfileChar1, len, FALSE, climbProfileAttrTbl, GATT_NUM_ATTRS(climbProfileAttrTbl),
			INVALID_TASK_ID, climbProfile_ReadAttrCB);
//wrong way
//#warning fix this
//			((attHandleValueNoti_t*) value)->handle = climbProfileAttrTbl[2].handle;
//			ret = GATT_Notification(0, (attHandleValueNoti_t*) value, 0);    //attempt to send

		} else {
			ICall_free(value);
			ret = bleInvalidRange;
		}
		break;

	default:
		ret = INVALIDPARAMETER;
		break;
	}

	return (ret);
}

/*********************************************************************
 * @fn      ClimbProfile_GetParameter
 *
 * @brief   Get a Simple Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t ClimbProfile_GetParameter(uint8 param, void *value) {
	bStatus_t ret = SUCCESS;
	switch (param) {
	case CLIMBPROFILE_CHAR1:
		VOID memcpy(value, climbProfileChar1, CLIMBPROFILE_CHAR1_LEN);
		break;

	case CLIMBPROFILE_CHAR2:
		VOID memcpy(value, climbProfileChar2, CLIMBPROFILE_CHAR2_LEN);
		break;

	default:
		ret = INVALIDPARAMETER;
		break;
	}

	return (ret);
}

/*********************************************************************
 * @fn          climbProfile_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t climbProfile_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr, uint8_t *pValue, uint16_t *pLen, uint16_t offset, uint16_t maxLen, uint8_t method) {
	bStatus_t status = SUCCESS;

	// If attribute permissions require authorization to read, return error
	if (gattPermitAuthorRead(pAttr->permissions)) {
		// Insufficient authorization
		return ( ATT_ERR_INSUFFICIENT_AUTHOR);
	}

	if (pAttr->type.len == ATT_BT_UUID_SIZE) {
		// 16-bit UUID
		uint16 uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
		switch (uuid) {
		case CLIMBPROFILE_CHAR1_UUID:

			// verify offset
			if (offset >= CLIMBPROFILE_CHAR1_LEN) {
				status = ATT_ERR_INVALID_OFFSET;

			} else {

				if (maxLen < CLIMBPROFILE_CHAR1_LEN) {
					if (offset > CLIMBPROFILE_CHAR1_LEN - maxLen) { //this is the last read operation for characteristic 1
						*pLen = CLIMBPROFILE_CHAR1_LEN - offset;
						status = SUCCESS;
					} else { //it still need some read operation to read the whole characteristic 1
						*pLen = maxLen;
						status = SUCCESS;
					}
				} else {
					*pLen = CLIMBPROFILE_CHAR1_LEN;
					status = SUCCESS;
				}

				VOID memcpy(pValue, &(pAttr->pValue[offset]), *pLen);
			}
			break;

		default:
			// Should never get here! (characteristics 3 and 4 do not have read permissions)
			*pLen = 0;
			status = ATT_ERR_ATTR_NOT_FOUND;
			break;
		}
	} else {
		// 128-bit UUID
		*pLen = 0;
		status = ATT_ERR_INVALID_HANDLE;
	}

	return (status);
}

/*********************************************************************
 * @fn      climbProfile_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t climbProfile_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr, uint8_t *pValue, uint16_t len, uint16_t offset, uint8_t method) {
	bStatus_t status = SUCCESS;
	uint8 notifyApp = 0xFF;

	// If attribute permissions require authorization to write, return error
	if (gattPermitAuthorWrite(pAttr->permissions)) {
		// Insufficient authorization
		return ( ATT_ERR_INSUFFICIENT_AUTHOR);
	}

	if (pAttr->type.len == ATT_BT_UUID_SIZE) {
		// 16-bit UUID
		uint16 uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
		switch (uuid) {
		case CLIMBPROFILE_CHAR1_UUID:
			//should not get here
			break;

		case CLIMBPROFILE_CHAR2_UUID:

			// verify offset
			if (offset > 0) {
				status = ATT_ERR_INVALID_OFFSET; //do not support blob write (but blob read is supported)
			} else {
				if (status == SUCCESS) {

					VOID memcpy(pAttr->pValue, pValue, len); //VERIFICARE!!!!
					memset(&pAttr->pValue[len], 0, CLIMBPROFILE_CHAR2_LEN - len); //reset the rest of characteristic
					//uint8 i;

					//for (i = len; i < CLIMBPROFILE_CHAR2_LEN; i++) { //azzera il resto della caratteristica
					//	pAttr->pValue[i] = 0;
					//}

					notifyApp = CLIMBPROFILE_CHAR2;

				}
			}
			break;

		case GATT_CLIENT_CHAR_CFG_UUID:
			status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len, offset, GATT_CLIENT_CFG_NOTIFY);
			break;

		default:
			// Should never get here! (characteristics 2 and 4 do not have write permissions)
			status = ATT_ERR_ATTR_NOT_FOUND;
			break;
		}
	} else {
		// 128-bit UUID
		status = ATT_ERR_INVALID_HANDLE;
	}

	// If a characteristic value changed then callback function to notify application of change
	if ((notifyApp != 0xFF) && climbProfile_AppCBs && climbProfile_AppCBs->pfnClimbProfileChange) {
		climbProfile_AppCBs->pfnClimbProfileChange(notifyApp);
	}

	return (status);
}

/*********************************************************************
 *********************************************************************/
