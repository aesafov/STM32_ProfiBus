/**

**/ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


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
    {}
}
//-----------------------------------------------------------------------------
uint32_t read_fpga(uint32_t addr)
{
    uint32_t data;
    uint32_t reg;
    
    return data;
}
//-----------------------------------------------------------------------------
void write_fpga(uint32_t addr,uint32_t data)
{
    
}
//-----------------------------------------------------------------------------
void GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOF, ENABLE);
    
    // PF0..PF5 - adr0..adr5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    // PF0..PF7 - data0..dara7
    // PF8 - ?
    // PF12 - ?
    // PF13 - ?
    // PF14 - ?
    // PF15 - ?
	/*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOА, &GPIO_InitStructure);*/
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
