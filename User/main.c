/****************************************************/
//    �ڴ�DEMO�м���USART1/USART2�������GPIOE�ӿڵ�1
//��״̬�ƵĶ���,���Ҵ���FPU��������ARM_MATH.H
//��ѧ����ͷ�ļ���ϵͳʱ����ԴΪHSE 25MHz��ϵͳ��
//Ƶ����Ϊ����ٶ�168MHz.
/****************************************************/
 

#include "MyF4Config.h"
//#include "5110.h"  
#include "MotorCtl.h"   // �������
//#include "24l01.h" 
#include "lcd.h"
#include "counter.h"   //������
#include "adc.h"
#include "filter.h" //�˲���
#include "pid.h" 
#include "visual scope.h"  //���ڵ��� ���ι���

#define ANG_center 1800  
#define GYRO_center 2280

int16_t 	velocity[2]; //����
float OutputL,OutputR; //�������

// �ٶȣ��Ƕȿ���
float speedL_accumulate=0,speedR_accumulate=0;
float P_angle=20;   //�Ƕ�
float D_angle=1;   //�Ƕ�
float P_speed=0;   //�ٶ�
float D_speed=0;   //�ٶ�
void AngleControl(xyz *temp,uint16_t angle,int16_t set_speed) 
{

	int16_t speedL_Delta,speedR_Delta;
	float AngleControl;
	
	AngleControl = P_angle * (angle-ANG_center) + D_angle * (temp->g-GYRO_center); //�Ƕȿ���

	//�ٶȿ���
	speedR_Delta=set_speed-velocity[0];
	speedL_Delta=set_speed-velocity[1];
	speedR_accumulate += speedL_Delta;
	speedL_accumulate += speedR_Delta;
	
	
	
	OutputL = AngleControl - P_speed * speedL_accumulate - D_speed * speedL_Delta;
	OutputR = AngleControl - P_speed * speedR_accumulate - D_speed * speedR_Delta;
}



int main(void)
{
	xyz *temp;
	uint16_t angle;
	int16_t 	speed;  //ָ���ٶ�
  RCC_Config();       //ϵͳʱ������	
	ADC_INIT();  //��ʼAD   
	USART_Config(); 
	delay_init();
	MotorInit();  //��ʼ���
//	NRF24L01_Init();    	//��ʼ��NRF24L01 
	TIM_Mode_Config();  //��ʼ������
	LCD_Init();  //��ʼLCD
	filter_init(); // ��ʼ�˲���
	pid_init(); //��ʼPID
	TIM3_Mode_Config();  //���ٶ�ʱ��
	while(1)
	{		
		temp = adcfilter_get();  //�õ�ԭʼ����
		filter_input(temp); //ad�ɼ�������������ֵ�˲�
		angle = comple_filter(temp,GYRO_center); //�����˲��ĽǶ�		
		temp = adcfilter_getCurrent(); //��ֵ�˲�
		

		visual_scope(angle,temp->z,100,10000);  //������λ�� 
		AngleControl(temp,angle,speed); //�Ƕ� �ٶȿ���
		MotorCtl(OutputL, OutputR,0); // ��Ϊ������� 
		//printf(" LCD ID:%x\r\n",lcddev.id); //��ӡLCD ID  
		delay_ms(5);
		
		
		/*********************  ��ʾ *************************/
		if(velocity[0]<0)
			LCD_ShowxNum(0,100,-velocity[0],5,16,0);//��ʾADC��ֵ
		else
			LCD_ShowxNum(0,100,velocity[0],5,16,0);//��ʾADC��ֵ
		if(velocity[1]<0)
			LCD_ShowxNum(0,116,-velocity[1],5,16,0);//��ʾADC��ֵ
		else
			LCD_ShowxNum(0,116,velocity[1],5,16,0);//��ʾADC��ֵ
		LCD_ShowString(140,130,200,16,16,"x");LCD_ShowxNum(156,130,temp->x,4,16,0);//��ʾADC��ֵ
		LCD_ShowString(140,146,200,16,16,"y");LCD_ShowxNum(156,146,temp->y,4,16,0);//��ʾADC��ֵ
	  LCD_ShowString(140,162,200,16,16,"z");LCD_ShowxNum(156,162,temp->z,4,16,0);//��ʾADC��ֵ
		LCD_ShowString(140,178,200,16,16,"g");LCD_ShowxNum(156,178,temp->g,4,16,0);LCD_ShowxNum(250,178,pid_iterate(temp->z-1900),5,16,0);//��ʾADC��ֵ//��ʾADC��ֵ
		LCD_ShowString(140,194,200,16,16,"a");LCD_ShowxNum(156,194,temp->ang,4,16,0);//��ʾADC��ֵ
		LCD_ShowxNum(156,210,temp->z,4,16,0);//��ʾADC��ֵ		
/****************************************************/
	}
}



//����
void TIM3_IRQHandler()
{
  TIM_ClearFlag(TIM3,TIM_FLAG_Update);//???????????	
	velocity[0]=20000-TIM1->CNT ;
	velocity[1]=20000-TIM4->CNT;;	
	TIM1->CNT = 20000;
	TIM4->CNT = 20000;
}

//���ڽ����ж�
void USART3_IRQHandler(void)
{
//			uint8_t data;
    if(USART_GetFlagStatus(USART3,USART_FLAG_RXNE) == SET)
    {
        USART_ClearFlag(USART3,USART_FLAG_RXNE);			
    }
}
