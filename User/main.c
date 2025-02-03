/**

**/ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define nWR_Pin GPIO_Pin_13
#define nRD_Pin GPIO_Pin_14
#define nCS_Pin GPIO_Pin_15
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#define COUNT_READ 60
uint32_t fpga_data[COUNT_READ];

/* Private function prototypes -----------------------------------------------*/


/* Private functions ---------------------------------------------------------*/
void GPIO_Config(void);
uint32_t read_fpga(uint32_t addr);
void write_fpga(uint32_t addr,uint32_t data);


/**
  * @brief   Main program
  * @param  None
  * @retval None
  */
int main(void)
{       
    GPIO_Config();
    
    while(1)
    {
		for(uint32_t i = 0; i <= COUNT_READ; i++)
		{
			fpga_data[i] = read_fpga(i);
		}
		
		__NOP();
		
		for(uint32_t i = 0; i <= COUNT_READ; i++)
		{
			fpga_data[i] = 0;
		}
	}
}
//-----------------------------------------------------------------------------
uint32_t read_fpga(uint32_t addr)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
    uint32_t data;
    uint32_t reg;
	
	// set address to read
	// Установить адрес для чтения
	GPIOD->ODR &= ~(0x0000003F);
	GPIOD->ODR |= (addr & 0x0000003F);
	
	GPIOF->BSRR |= (nRD_Pin << 16); // Установить в "0" вывод чтения
	GPIOF->BSRR |= (nCS_Pin << 16); // Установить в "0" вывод выбора чипа
	
	data = (GPIOF->IDR & 0x000000FF);
	
	GPIOF->BSRR |= (nRD_Pin | nCS_Pin); // Установить в "1" вывод чтения и вывод выбора чипа
    
    return data;
}
//-----------------------------------------------------------------------------
void write_fpga(uint32_t addr,uint32_t data)
{
    // set data ports as input
	// Установить порты данных как вход
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOF, &GPIO_InitStructure);
}
//-----------------------------------------------------------------------------
void GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOF, ENABLE);
    
    // PD0..PD5 - adr0..adr5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    // PF0..PF7 - data0..dara7
    // PF8 - ?
    // PF12 - ?
    // PF13 - nWR, active level "0"
    // PF14 - nRD, active level "0"
    // PF15 - nCS, active level "0"
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	
	// Выводы nCS, nRD и nWR как выход и "1" по умолчанию
	GPIO_WriteBit(GPIOF, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15, Bit_SET);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
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
