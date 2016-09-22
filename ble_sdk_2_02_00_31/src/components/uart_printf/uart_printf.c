/**************************************************************************************************
  Filename:       uart_printf.c

  Description:    This file contains the TI-RTOS hooks for printing to UART via
                  System_printf(..).

                  This is a very basic implementation made for the purposes of
                  terminal feedback in workshops, trainings and debug.

  Copyright 2015 Texas Instruments Incorporated. All rights reserved.

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
#include <Board.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>
#include <stdint.h>

/*********************************************************************
 * CONSTANTS
 */
#define UART_PRINTF_BUF_LEN      512

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8_t  uartPrintf_outArray[UART_PRINTF_BUF_LEN];
static uint16_t uartPrintf_head = 0;
static uint16_t uartPrintf_tail = 0;
static UART_Handle hUart = NULL;


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      UartPrintf_init
 *
 * @brief   Initializes the putchar hooks with the handle to the UART.
 *
 * @param   handle - UART driver handle to an initialized and opened UART.
 *
 * @return  None.
 */
void UartPrintf_init(UART_Handle handle)
{
	hUart = handle;
}

/*********************************************************************
 * SYSTEM HOOK FUNCTIONS
 */

/*********************************************************************
 * @fn      uartPrintf_putch
 *
 * @brief   User supplied PutChar function.
 *          typedef Void (*SysCallback_PutchFxn)(Char);
 *
 *          This function is called whenever the System module needs
 *          to output a character.
 *
 *          This implementation fills a very basic ring-buffer, and relies
 *          on another function to flush this buffer out to UART.
 *
 *          Requires SysCallback to be the system provider module.
 *          Initialized via SysCallback.putchFxn = "&uartPrintf_putch"; in the
 *          TI-RTOS configuration script.
 *
 * @param   ch - Character
 *
 * @return  None.
 *
 * @post    ::uartPrintf_head is incremented by one with wrap at UART_PRINTF_BUF_LEN
 *          if there is room.
 */
void uartPrintf_putch(char ch)
{
    // uartPrintf_tail should never catch up with uartPrintf_head. Discard in-between bytes.
	if ( (uartPrintf_head + 1) % UART_PRINTF_BUF_LEN == uartPrintf_tail )
		return;

	uartPrintf_outArray[uartPrintf_head] = ch;
	uartPrintf_head++;

	if (uartPrintf_head >= UART_PRINTF_BUF_LEN)
		uartPrintf_head = 0;
}

/*********************************************************************
 * @fn      uartPrintf_flush
 *
 * @brief   Printf-buffer flush function
 *
 *          In this implementation it is intended to be called by the
 *          Idle task when nothing else is running.
 *
 *          This is achieved by setting up the Idle task in the TI-RTOS
 *          configuration script like so:
 *
 *          var Idle = xdc.useModule('ti.sysbios.knl.Idle');
 *          Idle.addFunc('&uartPrintf_flush');
 *
 * @param   None. Relies on global state.
 *
 * @return  None.
 *
 * @post    ::uartPrintf_tail is incremented to where uartPrintf_head
 *          was at the time the function was called.
  */
void uartPrintf_flush()
{
	// Abort in case UART hasn't been initialized.
	if (NULL == hUart)
		return;

  // Lock head position to avoid race conditions
  uint16_t curHead = uartPrintf_head;

  // Find out how much data must be output, and how to output it.
	bool needWrap = curHead < uartPrintf_tail;
  uint16_t outLen = needWrap?(UART_PRINTF_BUF_LEN-uartPrintf_tail+curHead):(curHead-uartPrintf_tail);

	if (outLen)
	{
		if (needWrap)
		{
			UART_write(hUart, &uartPrintf_outArray[uartPrintf_tail], UART_PRINTF_BUF_LEN - uartPrintf_tail);
			UART_write(hUart, uartPrintf_outArray, curHead);
		}
		else
		{
			UART_write(hUart, &uartPrintf_outArray[uartPrintf_tail], outLen);
		}
	}

	uartPrintf_tail = curHead;
}
