#include "init_periph.h"
#include "stm32f10x.h"
#include "config.h"

#define SPI1_DR_Address       0x4001300C
#define DUMMY_Address         0x20000000

void init_clk(void);
void init_gpio(void);
void init_spi(void);
void init_dma(void);

//***********************************************************


void init_all_periph(void)
{
  init_clk();
  init_gpio();
  init_spi();
  init_dma();
}

void init_clk(void)
{
  ErrorStatus HSEStartUpStatus;
  
  RCC_DeInit(); /* RCC system reset(for debug purpose) */
  RCC_HSEConfig(RCC_HSE_ON);/* Enable HSE */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();/* Wait till HSE is ready */

  if (HSEStartUpStatus == SUCCESS)
  {
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);/* Enable Prefetch Buffer */
    FLASH_SetLatency(FLASH_Latency_2);
    
    RCC_HCLKConfig(RCC_SYSCLK_Div1);/* HCLK = SYSCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1);/* PCLK2 = HCLK */
    RCC_PCLK1Config(RCC_HCLK_Div1);/* PCLK1 = HCLK */
    RCC_PREDIV1Config(RCC_PREDIV1_Source_HSE, RCC_PREDIV1_Div1);
    RCC_PLLConfig(RCC_PLLSource_PREDIV1, MCU_CLK_MULTIPLIER);/* PLLCLK = 8MHz * 8 = 64 MHz*/
    RCC_PLLCmd(ENABLE);/* Enable PLL */
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){};/* Wait till PLL is ready */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);/* Select PLL as system clock source */
    while (RCC_GetSYSCLKSource() != 0x08){}/* Wait till PLL is used as system clock source */
  }
}

void init_gpio(void)
{ 
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|
                         RCC_APB2Periph_GPIOB|
                         RCC_APB2Periph_GPIOC|
                         RCC_APB2Periph_AFIO, ENABLE);
 
   /* Configure SPI_MASTER pins: SCK and MOSI */
   GPIO_InitStructure.GPIO_Pin = SPI_MOSI_PIN | SPI_CLK_PIN;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_Init(SPI_GPIO, &GPIO_InitStructure);

   GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
   
   
  GPIO_InitStructure.GPIO_Pin = SYNC_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(SYNC_GPIO, &GPIO_InitStructure);
}

void init_spi(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

  SPI_InitTypeDef    SPI_InitStructure;
  
  SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;//clock polarity - idle low
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_PRESCALER;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
  SPI_Init(SPI1, &SPI_InitStructure);
  
  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
  SPI_Cmd(SPI1, ENABLE);
}


void init_dma(void)
{
  DMA_InitTypeDef DMA_InitStructure;
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  
  DMA_DeInit(SPI_DMA_CH);
  DMA_InitStructure.DMA_PeripheralBaseAddr =  SPI1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr =      DUMMY_Address;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = 100;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  
  DMA_Init(SPI_DMA_CH, &DMA_InitStructure);
  
  DMA_ITConfig(SPI_DMA_CH, DMA_IT_HT, ENABLE);
  
  NVIC_SetPriority(DMA1_Channel3_IRQn, 15);
  NVIC_EnableIRQ(DMA1_Channel3_IRQn); 
}

