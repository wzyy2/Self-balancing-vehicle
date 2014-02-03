/**********************************************/
/****************ϵͳ�ڲ�����******************/
/**********************************************/
#include "MyF4Config.h"


/**********************************************/
/****************ϵͳʱ������******************/
/**********************************************/
//ϵͳʱ������������ٶ�168MHz
void RCC_Config()
{
	RCC_DeInit();                                  //RCC�Ĵ�����ʼ��
  RCC_HSEConfig(RCC_HSE_ON);                     //ʹ���ⲿʱ��
  if ( RCC_WaitForHSEStartUp() == SUCCESS )      //�ȴ��ⲿʱ������  25MHz
  {
		RCC_PLLCmd(DISABLE);                         //����PLLǰӦ�ȹر���PLL
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);   //ѡ��PLLʱ��Ϊϵͳʱ��  168MHz
		RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 7); //PLLʱ�����ã���ʽ�����system_stm43f4xx.c�� Line149
    RCC_HCLKConfig(RCC_SYSCLK_Div1);             //HCLK(AHB)ʱ��Ϊϵͳʱ��1��Ƶ 168MHz
    RCC_PCLK1Config(RCC_HCLK_Div4);              //PCLK1(APB1)ʱ��ΪHCLKʱ��4��Ƶ 42MHz;TIM2,3,4,5,6,7,12,13,14Ϊ84MHz
    RCC_PCLK2Config(RCC_HCLK_Div2);              //PCLK2(APB2)ʱ��ΪHCLKʱ��2��Ƶ 84Mhz;TIM1,8,9,10,11Ϊ168MHz
  	//FLASH_SetLatency(FLASH_Latency_5);           //����FLASH��ʱ������Ϊ5
		//FLASH_PrefetchBufferCmd(ENABLE);             //ʹ��FLASHԤȡ����
   RCC_PLLCmd(ENABLE);                          //PLLʱ�ӿ���
   while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) { } //�ȴ�PLLʱ��׼����
		while(RCC_GetSYSCLKSource() != 0x08) { }
  }
	//ʹ��GPIOA��GPIOB��GPIOC��GPIOD��GPIOEʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | 
	                              RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE , ENABLE);
	//ʹ��USART1,2��ʹ��TIM2,3,5,6,9ʹ��SPI1ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3 | RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM4|RCC_APB1Periph_SPI3 , ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
}




