/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portserial.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

#include "port.h"
//#include "stm32f10x_conf.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- static functions ---------------------------------*/
static void prvvUARTTxReadyISR( void );
static void prvvUARTRxISR( void );

/* ----------------------- Start implementation -----------------------------*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    /* If xRXEnable enable serial receive interrupts. If xTxENable enable
     * transmitter empty interrupts.
     */
	if( xRxEnable )
    {
        USART_ITConfig( USART3, USART_IT_RXNE, ENABLE );        
    }
    else
    {
        USART_ITConfig( USART3, USART_IT_RXNE, DISABLE );
    }

    if ( xTxEnable )
    {
        USART_ITConfig( USART3, USART_IT_TXE, ENABLE );        

#ifdef RTS_ENABLE
        RTS_HIGH;
#endif
    }
    else
    {
        USART_ITConfig( USART3, USART_IT_TXE, DISABLE );
    }
}

BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
	NVIC_InitTypeDef        NVIC_InitStructure;
    GPIO_InitTypeDef        GPIO_InitStructure;
    USART_InitTypeDef       USART_InitStructure;
    USART_ClockInitTypeDef  USART_ClockInitStructure;

    /* подавляем предупреждение компилятора о неиспользуемой переменной */
    (void) ucPORT;

    RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART3, ENABLE );
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE );

//    GPIO_PinAFConfig( GPIOC, GPIO_PinSource10, GPIO_AF_USART3 );
//    GPIO_PinAFConfig( GPIOC, GPIO_PinSource11, GPIO_AF_USART3 );

    GPIO_StructInit( &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOB, &GPIO_InitStructure );
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init( GPIOB, &GPIO_InitStructure );
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOE, &GPIO_InitStructure );    

    USART_DeInit( USART3 );

    USART_ClockStructInit( &USART_ClockInitStructure );

    USART_ClockInit( USART3, &USART_ClockInitStructure );

    /* настройка скорости обмена */
    USART_InitStructure.USART_BaudRate = (uint32_t)ulBaudRate;

    /* настройка кол-ва битов данных */
    if( ucDataBits == 9 )
        USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    else
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;

    /* кол-во стоп-битов устанавливаем равным 1 */
    USART_InitStructure.USART_StopBits = USART_StopBits_1;

    /* настройка паритета (по умолчанию - его нет) */
    switch( eParity )
    {
    case MB_PAR_NONE:
        USART_InitStructure.USART_Parity = USART_Parity_No;
        break;
    case MB_PAR_ODD:
        USART_InitStructure.USART_Parity = USART_Parity_Odd;
        break;
    case MB_PAR_EVEN:
        USART_InitStructure.USART_Parity = USART_Parity_Even;
        break;
    default:
        USART_InitStructure.USART_Parity = USART_Parity_No;
        break;
    };

    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    USART_Init( USART3, &USART_InitStructure );

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;           /* канал */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   /* приоритет */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;          /* приоритет субгруппы */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             /* включаем канал */
    NVIC_Init(&NVIC_InitStructure);                             /* инициализируем */

    USART_Cmd( USART3, ENABLE );

//    vMBPortSerialEnable( TRUE, TRUE );

#ifdef RTS_ENABLE
    RTS_INIT;
#endif
    //return FALSE;
		return TRUE;
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
    /* Put a byte in the UARTs transmit buffer. This function is called
     * by the protocol stack if pxMBFrameCBTransmitterEmpty( ) has been
     * called. */
    
    // Мои правки
    GPIO_SetBits(GPIOE, GPIO_Pin_15); // для включения режима передачи
    // ------------
	USART_SendData( USART3, (uint16_t) ucByte );
    
    // Мои правки
    GPIO_ResetBits(GPIOE, GPIO_Pin_15); // для включения режима приема
    // ------------
    
    return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    /* Return the byte in the UARTs receive buffer. This function is called
     * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
     */
	*pucByte = (CHAR) USART_ReceiveData( USART3 );
    return TRUE;
}

/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */
static void prvvUARTTxReadyISR( void )
{
    pxMBFrameCBTransmitterEmpty(  );
}

/* Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
static void prvvUARTRxISR( void )
{
    pxMBFrameCBByteReceived(  );
}
