/******************************************************************************

 @file  hal_i2c.h

 @brief Layer added on top of RTOS driver for backwards compatibility with
        CC2541ST

 Group: WCS, LPC, BTS
 Target Device: CC2650, CC2640, CC1350

 ******************************************************************************
 
 Copyright (c) 2013-2016, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: ble_sdk_2_02_00_31
 Release Date: 2016-06-16 18:57:29
 *****************************************************************************/

#ifndef BSP_I2C_H
#define BSP_I2C_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
//#include "comdef.h"
#include "stdbool.h"
#include "stdint.h"
#include "hw_memmap.h"

/*********************************************************************
 * CONSTANTS
 */
#define HAL_I2C_INTERFACE_0     0
#define HAL_I2C_INTERFACE_1     1
   
/*********************************************************************
 * TYPEDEFS
 */
  
/*********************************************************************
 * FUNCTIONS
 */
void HalI2CInit(void);
void HalI2CConfig(uint8_t interface, uint8_t slaveAddress);
bool HalI2CRead(uint8_t *pBuf, uint8_t len);
bool HalI2CWrite(uint8_t *pBuf, uint8_t len);
bool HalI2CWriteSingle(uint8_t data);
bool HalI2CWriteRead(uint8_t *wdata, uint8_t wlen, uint8_t *rdata, uint8_t rlen);
void HalI2CDisable(void);

//
// TEMPORARY until naming convention is decided
//
#define BSP_I2C_INTERFACE_0 HAL_I2C_INTERFACE_0
#define BSP_I2C_INTERFACE_1 HAL_I2C_INTERFACE_1


/////////////////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
}
#endif

#endif /* BSP_IRTEMP_H */
