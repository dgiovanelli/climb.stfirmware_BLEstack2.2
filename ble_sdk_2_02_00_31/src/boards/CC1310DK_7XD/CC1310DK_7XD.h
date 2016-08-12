/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** ============================================================================
 *  @file       Board.h
 *
 *  @brief      CC1310EM_7XD_7793 Board Specific header file.
 *              The project options should point to this file if this is the
 *              CC1310EM you are developing code for.
 *
 *  The CC1310 header file should be included in an application as follows:
 *  @code
 *  #include <Board.h>
 *  @endcode
 *
 *  ============================================================================
 */
#ifndef __CC1310EM_7XD_7793_H__
#define __CC1310EM_7XD_7793_H__

#ifdef __cplusplus
extern "C" {
#endif

/** ============================================================================
 *  Symbol by generic Board.c to include the correct kit specific Board.c
 *  ==========================================================================*/
#define CC1310EM_7XD_7793


/** ============================================================================
 *  Includes
 *  ==========================================================================*/
#include <ti/drivers/PIN.h>
#include <driverlib/ioc.h>

/** ============================================================================
 *  Externs
 *  ==========================================================================*/
extern const PIN_Config BoardGpioInitTable[];

/** ============================================================================
 *  Defines
 *  ==========================================================================*/

/* Mapping of pins to board signals using general board aliases
 *      <board signal alias>                <pin mapping>   <comments>
 */
/* Leds */
#define Board_LED_ON                        1               /* LEDs on SmartRF06 EB are active high */
#define Board_LED_OFF                       0
#define Board_DK_LED1                       IOID_25         /* P2.11 */
#define Board_DK_LED2                       IOID_27         /* P2.13 */
#define Board_DK_LED3                       IOID_7          /* P1.2  */
#define Board_DK_LED4                       IOID_6          /* P1.4  */
/* Button Board */
#define Board_KEY_SELECT                    IOID_11         /* P1.14 */
#define Board_KEY_UP                        IOID_19         /* P1.10 */
#define Board_KEY_DOWN                      IOID_12         /* P1.12 */
#define Board_KEY_LEFT                      IOID_15         /* P1.6  */
#define Board_KEY_RIGHT                     IOID_18         /* P1.8  */
/* LCD  Board */
#define Board_LCD_MODE                      IOID_4          /* P1.11 */
#define Board_LCD_RST                       IOID_5          /* P1.13 */
#define Board_LCD_CSN                       IOID_14         /* P1.17 */
/* UART Board */
#define Board_UART_RX                       IOID_2          /* P1.7  */
#define Board_UART_TX                       IOID_3          /* P1.9  */
#define Board_UART_CTS                      IOID_22         /* P1.3  */
#define Board_UART_RTS                      IOID_21         /* P2.18 */
/* SPI Board */
#define Board_SPI0_MISO                     IOID_8          /* P1.20 */
#define Board_SPI0_MOSI                     IOID_9          /* P1.18 */
#define Board_SPI0_CLK                      IOID_10         /* P1.16 */
#define Board_SPI0_CSN                      PIN_UNASSIGNED  /* P1.14, separate CSn for LCD, SDCARD, and ACC */
#define Board_SPI1_MISO                     IOID_24         /* RF2.10 for testing only */
#define Board_SPI1_MOSI                     IOID_23         /* RF2.5  for testing only */
#define Board_SPI1_CLK                      IOID_30         /* RF2.12 for testing only */
#define Board_SPI1_CSN                      PIN_UNASSIGNED  /* RF2.6  for testing only */
/* Ambient Light Sensor */
#define Board_ALS_OUT                       IOID_23         /* P2.5 */
#define Board_ALS_PWR                       IOID_26         /* P2.6 */
/* Accelerometer */
#define Board_ACC_PWR                       IOID_20         /* P2.8 */
#define Board_ACC_CSN                       IOID_24         /* P2.10 */
/* SD Card */
#define Board_SDCARD_CSN                    IOID_30         /* P2.12 */
/* Power Board */
#define Board_3V3_EN                        IOID_13         /* P1.15 */

/** ============================================================================
 *  Instance identifiers
 *  ==========================================================================*/
/* Generic SPI instance identifiers */
#define Board_SPI0                  CC1310DK_7XD_SPI0
/* Generic UART instance identifiers */
#define Board_UART                  CC1310DK_7XD_UART0
/* Generic Crypto instance identifiers */
#define Board_CRYPTO                CC1310DK_7XD_CRYPTO0

/** ============================================================================
 *  Number of peripherals and their names
 *  ==========================================================================*/

/*!
 *  @def    CC2650_CryptoName
 *  @brief  Enum of Crypto names on the CC2650 dev board
 */
typedef enum CC1310DK_7XD_CryptoName {
    CC1310DK_7XD_CRYPTO0 = 0,
    CC1310DK_7XD_CRYPTOCOUNT
} CC1310DK_7XD_CryptoName;

/*!
 *  @def    CC1310DK_7XD_SPIName
 *  @brief  Enum of SPI names on the CC1310 dev board
 */
typedef enum CC1310DK_7XD_SPIName {
    CC1310DK_7XD_SPI0 = 0,
    CC1310DK_7XD_SPI1,
    CC1310DK_7XD_SPICOUNT
} CC1310DK_7XD_SPIName;

/*!
 *  @def    CC1310DK_7XD_UARTName
 *  @brief  Enum of UARTs on the CC1310 dev board
 */
typedef enum CC1310DK_7XD_UARTName {
    CC1310DK_7XD_UART0 = 0,
    CC1310DK_7XD_UARTCOUNT
} CC1310DK_7XD_UARTName;

/*!
 *  @def    CC1310DK_7XD_UdmaName
 *  @brief  Enum of DMA buffers
 */
typedef enum CC1310DK_7XD_UdmaName {
    CC1310DK_7XD_UDMA0 = 0,
    CC1310DK_7XD_UDMACOUNT
} CC1310DK_7XD_UdmaName;

#ifdef __cplusplus
}
#endif

#endif /* __CC1310EM_H__ */
