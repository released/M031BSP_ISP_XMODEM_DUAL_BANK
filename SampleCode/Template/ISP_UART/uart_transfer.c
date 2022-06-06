/**************************************************************************//**
 * @file     uart_transfer.c
 * @version  V1.00
 * $Date: 14/11/17 5:36p $
 * @brief    General UART ISP slave Sample file
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>
#include "targetdev.h"
#include "NuMicro.h"
#include "uart_transfer.h"

#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t uart_rcvbuf[MAX_PKT_SIZE] = {0};
#else
__attribute__((aligned(4))) uint8_t uart_rcvbuf[MAX_PKT_SIZE] = {0};
#endif

uint8_t volatile bUartDataReady = 0;
uint8_t volatile bufhead = 0;


/* please check "targetdev.h" for chip specifc define option */

/*---------------------------------------------------------------------------------------------------------*/
/* INTSTS to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void ISP_UART_IRQHandler(void)
{
    /*----- Determine interrupt source -----*/
    uint32_t u32IntSrc = ISP_UART_PORT->INTSTS;

    if (u32IntSrc & 0x11)   //RDA FIFO interrupt & RDA timeout interrupt
    {
        while (((ISP_UART_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0) && (bufhead < MAX_PKT_SIZE))      //RX fifo not empty
        {
            uart_rcvbuf[bufhead++] = ISP_UART_PORT->DAT;
        }
    }

    if (bufhead == MAX_PKT_SIZE)
    {
        bUartDataReady = TRUE;
        bufhead = 0;
    }
    else if (u32IntSrc & 0x10)
    {
        bufhead = 0;
    }
}
#ifdef __ICCARM__
#pragma data_alignment=4
extern uint8_t response_buff[64];
#else
extern __attribute__((aligned(4))) uint8_t response_buff[64];
#endif 

void PutString(void)
{
    uint32_t i;

    for (i = 0; i < MAX_PKT_SIZE; i++)
    {
        while ((ISP_UART_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk));

        ISP_UART_PORT->DAT = response_buff[i];
    }
}

void ISP_UART_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Select UART function mode */
    UART0->FUNCSEL = ((UART0->FUNCSEL & (~UART_FUNCSEL_FUNCSEL_Msk)) | UART_FUNCSEL_MODE);
    /* Set UART line configuration */
    ISP_UART_PORT->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
    /* Set UART Rx and RTS trigger level */
    ISP_UART_PORT->FIFO = UART_FIFO_RFITL_14BYTES | UART_FIFO_RTSTRGLV_14BYTES;
    /* Set UART baud rate */
    ISP_UART_PORT->BAUD = (UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200));
    /* Set time-out interrupt comparator */
    ISP_UART_PORT->TOUT = (ISP_UART_PORT->TOUT & ~UART_TOUT_TOIC_Msk) | (0x40);
    NVIC_SetPriority(ISP_UART_PORT_IRQn, 2);
    NVIC_EnableIRQ(ISP_UART_PORT_IRQn);
    /* 0x0811 */
    ISP_UART_PORT->INTEN = (UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_RDAIEN_Msk);
}

