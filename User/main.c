/**
Настройка Modbus RTU взята отсюда:
https://ctrl-v.biz/ru/blog/stm32f4discovery-modbus-rtu-ascii-mems-lis302dl


**/ 

#define HSE_VALUE ((uint32_t)12000000)

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "mb.h"
#include "mbport.h"
#include "mbcrc.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define nRD_Pin GPIO_Pin_8
#define nWR_Pin GPIO_Pin_9
#define nCS_Pin GPIO_Pin_10

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#define COUNT_READ 55
uint32_t fpga_data[COUNT_READ];
uint32_t send_byte = 0;
/* Private function prototypes -----------------------------------------------*/


/* Private functions ---------------------------------------------------------*/
void GPIO_Config(void);
uint32_t read_fpga(uint32_t addr);
void write_fpga(uint32_t addr,uint32_t data);

/* ModBus --------------------------------------------------------------------*/
#define REG_HOLDING_START   40001
#define REG_HOLDING_NREGS   4

static USHORT   usRegHoldingStart = REG_HOLDING_START;
static USHORT   usRegHoldingBuf[REG_HOLDING_NREGS];
/* ---------------------------------------------------------------------------*/
// Функция для отправки строки через USART
void USART_SendString(USART_TypeDef *USARTx, uint8_t *data) {
    while (*data) { // Перебираем символы строки
        USART_SendData(USARTx, *data); // Отправляем один байт
        while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET); // Ждем, пока регистр передачи станет пустым
        data++; // Переходим к следующему символу
		send_byte++;
    }
}


/**
  * @brief   Main program
  * @param  None
  * @retval None
  */
int main(void)
{     
//#undef HSE_VALUE
//#define HSE_VALUE ((uint32_t)12000000)	
	
	SystemInit();
	
    GPIO_Config();
	
    xMBPortSerialInit(3, 57600, 8, MB_PAR_NONE);
	GPIO_SetBits(GPIOE, GPIO_Pin_15);
	uint8_t message[] = "Hello, World!\r\n";
	USART_SendString(USART3, message);
	GPIO_ResetBits(GPIOE, GPIO_Pin_15);
	
	USART_ClearITPendingBit( USART3, USART_IT_RXNE );
	USART_ITConfig( USART3, USART_IT_RXNE, ENABLE );
	
	USART_ClearFlag(USART3, USART_FLAG_PE); // Очистка флага PE
	USART_ITConfig(USART3, USART_IT_PE, ENABLE);  // Прерывание по ошибке паритета
	
	USART_ClearFlag(USART3, USART_FLAG_ORE); // Очистка флага ORE
	USART_ClearFlag(USART3, USART_FLAG_FE); // Очистка флага FE
	USART_ClearFlag(USART3, USART_FLAG_NE); // Очистка флага NE
	USART_ITConfig(USART3, USART_IT_ERR, ENABLE); // Прерывание по другим ошибкам (ORE, NE, FE)
	
	
	
//   eMBErrorCode eStatus;
//	
//	eStatus = eMBInit( MB_RTU,
//                        0x0A,           /* адрес slave-устройства */
//                        0,
//                        9600,           /* скорость обмена */
//                        MB_PAR_NONE     /* без паритета */
//                     );
//	
//	eStatus = eMBEnable();

    while(1)
    {
//        (void) eMBPoll();
        
		for(uint32_t i = 0; i <= COUNT_READ; i++)
		{
			fpga_data[i] = read_fpga(i);
		}
		
		__NOP();
		
		for(uint32_t i = 0; i <= COUNT_READ; i++)
		{
			fpga_data[i] = 0;
		}
        
        write_fpga(16, 0xFF);
	}
}
//-----------------------------------------------------------------------------
eMBErrorCode eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_HOLDING_START ) &&
        ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegHoldingStart );

        switch ( eMode )
        {
        case MB_REG_READ:
            while( usNRegs > 0 )
            {
                *pucRegBuffer++ = ( UCHAR ) ( usRegHoldingBuf[iRegIndex] >> 8 );
                *pucRegBuffer++ = ( UCHAR ) ( usRegHoldingBuf[iRegIndex] & 0xFF );
                iRegIndex++;
                usNRegs--;
            }
            break;
        case MB_REG_WRITE:
            while( usNRegs > 0 )
            {
                usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;
            }
            break;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}
//-----------------------------------------------------------------------------
eMBErrorCode eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    return MB_ENOREG;
}
//-----------------------------------------------------------------------------
eMBErrorCode eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode )
{
    return MB_ENOREG;
}
//-----------------------------------------------------------------------------
eMBErrorCode eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    return MB_ENOREG;
}
//-----------------------------------------------------------------------------
eMBException eMBFuncReportSlaveID( UCHAR * pucFrame, USHORT * usLen )
{
    // разобратся, что туту делать!!!!!!!!!!!!!!!!!
    return MB_EX_NONE;
}
//-----------------------------------------------------------------------------
void vMBPortTimersDelay( USHORT usTimeOutMS )
{
    // без ее реализации не компилировалось!!!!!!!!!!!!!!!!!
}
//-----------------------------------------------------------------------------
uint32_t read_fpga(uint32_t addr)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
    uint32_t data;
    uint32_t reg;
	
	// set address to read
	// Установить адрес для чтения
	GPIOD->ODR &= ~(0x0000000F);
	GPIOD->ODR |= (addr & 0x0000000F);
    // старшие 2 бита адреса запихиваем в PE7 и PE8
    addr = addr << 3;
    GPIOE->ODR &= ~(0x00000180);
	GPIOE->ODR |= (addr & 0x00000180);
    for(uint32_t i = 0; i <= COUNT_READ; i++) { __NOP(); }	
	
	GPIOF->BSRR |= (nRD_Pin << 16); // Установить в "0" вывод чтения
	GPIOF->BSRR |= (nCS_Pin << 16); // Установить в "0" вывод выбора чипа
	
    for(uint32_t i = 0; i <= COUNT_READ; i++) { __NOP(); }	
    
	data = (GPIOF->IDR & 0x000000FF);
	
	GPIOF->BSRR |= (nRD_Pin | nCS_Pin); // Установить в "1" вывод чтения и вывод выбора чипа
    
    return data;
}
//-----------------------------------------------------------------------------
void write_fpga(uint32_t addr, uint32_t data)
{
    // set address to write
	// Установить адрес для записи
	GPIOD->ODR &= ~(0x0000000F);
	GPIOD->ODR |= (addr & 0x0000000F);
    // старшие 2 бита адреса запихиваем в PE7 и PE8
    addr = addr << 3;
    GPIOE->ODR &= ~(0x00000180);
	GPIOE->ODR |= (addr & 0x00000180);
    for(uint32_t i = 0; i <= COUNT_READ; i++) { __NOP(); }
    
    GPIOF->ODR &= ~(0x000000FF);
    GPIOF->ODR |= (uint8_t)data;
    
    GPIOF->BSRR |= (nWR_Pin << 16); // Установить в "0" вывод записи
	GPIOF->BSRR |= (nCS_Pin << 16); // Установить в "0" вывод выбора чипа
	
    for(uint32_t i = 0; i <= COUNT_READ; i++) { __NOP(); }
    
    GPIOF->BSRR |= (nWR_Pin | nCS_Pin); // Установить в "1" вывод чтения и вывод выбора чипа
}
//-----------------------------------------------------------------------------
void GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF, ENABLE);
    
    // PB14 - светодиод State
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // PD0..PD3 - adr0..adr3
    // PD15 - светодиод Link    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    // set address bit as "0"
	GPIOD->ODR &= ~(0x0000000F);
    
    // PE7 - adr4
    // PE8 - adr5
	// PE14 - "0" - MPU; "1" - ProfiBus
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8  | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	// UART на выход, используем FreeModBus
	GPIO_ResetBits(GPIOE, GPIO_Pin_14);
    
    // set address bit as "0"
	GPIOE->ODR &= ~(GPIO_Pin_7 | GPIO_Pin_8);
    
    // PF0..PF7 - data0..dara7
    // PF8 - nRD, active level "0"
    // PF9 - nWR, active level "0"
    // PF10 - nCS, active level "0"
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
    
    // set data bit as "0"
	GPIOF->ODR &= ~(0x000000FF);
    
	
	// Выводы nCS, nRD и nWR как выход и "1" по умолчанию
	GPIO_WriteBit(GPIOF, GPIO_Pin_8 | GPIO_Pin_9 |  GPIO_Pin_10, Bit_SET);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 |  GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
}
//-----------------------------------------------------------------------------
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  while (1)
  {}
}

#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
