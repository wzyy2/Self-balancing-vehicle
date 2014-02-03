/****************************************************/
//    �ڴ�DEMO�м���USART1/USART2�������GPIOE�ӿڵ�1
//��״̬�ƵĶ���,���Ҵ���FPU��������ARM_MATH.H
//��ѧ����ͷ�ļ���ϵͳʱ����ԴΪHSE 25MHz��ϵͳ��
//Ƶ����Ϊ����ٶ�168MHz.
/****************************************************/
 

#include "MyF4Config.h"

#include "5110.h"  
#include "MotorCtl.h"  
#include "24l01.h" 
#include "lcd.h"
#include "counter.h"  
#include "math.h"

int main(void)
{
//  RCC_Config();       //ϵͳʱ������	
	delay_init();
	USART_Config(); 
	MotorInit();
	MotorCtl(10000,0);
	NRF24L01_Init();    	//��ʼ��NRF24L01 
	while(NRF24L01_Check())
	{
		USART_printf(USART3, "%d",SPI3->CR1);
	}
	
			LCD_Init();
	//POINT_COLOR=RED;
//r	LCD_DisplayOn();
 // NRF24L01_TX_Mode();	
  //TIM_Mode_Config();
	//
	//LCD5110_init();
	//LCD5110_clear();

	while(1)
	{	
		USART_printf(USART3, "%d",SPI3->CR1);
	}
}
