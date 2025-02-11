/**
  ******************************************************************************
  * @file    TIM/TIM10_PWMOutput/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *         This file provides template for all exceptions handler and
  *         peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "mb.h"

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup TIM10_PWMOutput
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

#define MODBUS_MAX_FRAME_LENGTH 32 // Максимальная длина пакета

uint8_t modbus_rx_buffer[MODBUS_MAX_FRAME_LENGTH];
volatile uint8_t modbus_rx_index = 0;

void USART3_IRQHandler( void )
{
    // Проверяем, было ли прерывание вызвано ошибкой
    if (USART_GetITStatus(USART3, USART_IT_ERR) != RESET) 
	{
        // Обработка ошибки переполнения (ORE)
        if (USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET) 
		{
            USART_ClearFlag(USART3, USART_FLAG_ORE); // Очистка флага ORE
        }

        // Обработка ошибки шума (NE)
        if (USART_GetFlagStatus(USART3, USART_FLAG_NE) != RESET) 
		{
            USART_ClearFlag(USART3, USART_FLAG_NE); // Очистка флага NE
        }

        // Обработка ошибки фрейма (FE)
        if (USART_GetFlagStatus(USART3, USART_FLAG_FE) != RESET) 
		{
            USART_ClearFlag(USART3, USART_FLAG_FE); // Очистка флага FE
        }
    }

    // Проверка ошибки паритета (PE)
    if (USART_GetITStatus(USART3, USART_IT_PE) != RESET) 
	{
        if (USART_GetFlagStatus(USART3, USART_FLAG_PE) != RESET) 
		{
            USART_ClearFlag(USART3, USART_FLAG_PE); // Очистка флага PE
        }
        USART_ClearITPendingBit(USART3, USART_IT_PE); // Очистка бита прерывания PE
    }

    if ( USART_GetITStatus( USART3, USART_IT_RXNE ) != RESET )
    {
        USART_ClearITPendingBit( USART3, USART_IT_RXNE );
//        uint8_t data = USART_ReceiveData(USART3); // Читаем байт
//        modbus_rx_buffer[modbus_rx_index++] = data; // Сохраняем в буфер
        modbus_rx_buffer[modbus_rx_index++] = USART_ReceiveData(USART3);		
		// Проверяем, достигнута ли минимальная длина пакета
        if (modbus_rx_index >= 8)  // Минимум: адрес + функция + данные + CRC (2 байта)
		{
            //ParseModbusPacket(modbus_rx_buffer, modbus_rx_index);
			__NOP();
			modbus_rx_index = 0;
        }

    }
    if ( USART_GetITStatus( USART3, USART_IT_TXE ) != RESET )
    {
        USART_ClearITPendingBit( USART3, USART_IT_TXE );
        pxMBFrameCBTransmitterEmpty();
    }
}

//void USART3_IRQHandler( void )
//{
//    if ( USART_GetITStatus( USART3, USART_IT_RXNE ) != RESET )
//    {
//        USART_ClearITPendingBit( USART3, USART_IT_RXNE );
//        pxMBFrameCBByteReceived();

//    }
//    if ( USART_GetITStatus( USART3, USART_IT_TXE ) != RESET )
//    {
//        USART_ClearITPendingBit( USART3, USART_IT_TXE );
//        pxMBFrameCBTransmitterEmpty();
//    }
//}

void TIM6_IRQHandler( void )
{
    if ( TIM_GetITStatus( TIM6, TIM_IT_Update ) != RESET )
    {
        TIM_ClearITPendingBit( TIM6, TIM_IT_Update );

        (void) pxMBPortCBTimerExpired();
    }
}

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
