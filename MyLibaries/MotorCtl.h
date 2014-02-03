//*********************************************************************
//
//  Copyright (c) UESTC Roboteam 2011 �� All rights reserved.
// 
//  INCLUDES.H - ��ͷ�ļ�����
//
//      ���ļ��ǵ��ӿƼ�����������Ѳ�߻����˴���������ί���ṩ��ģ��
//  ����ĵ�����ƴ����ļ����û��뽫��Ƭ��Ӳ����ʼ����������ƴ��벹
//  ����������ʹ�á�
//  ע���ٶȵĻ�����ͳһ��������
//
//  2011��10��29��
//
//********************************************************************

#include "MyF4Config.h"

//********************************************************************
// ������: PWMInit() 
// ˵  ��: ��Ƭ��PWM��������ʼ�����ú���
// ʱ  ��: 2011.10.29  
// ��  ��: ��
// ��  ��: ��
// ��  ��: ��
// ע  ��: PWM��Ƶ�� 5-20K
//********************************************************************    
// �޸���:
// ʱ  ��:
// ��  ��:
//********************************************************************
void TIM2_PWM_Init(u16 arr,u16 psc)
{  	
	u16 CCR1_Val = 0;        
	u16 CCR2_Val = 0;
	u16 CCR3_Val = 0;
	u16 CCR4_Val = 0;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_Cmd(TIM2, DISABLE);
 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
   //���ø�����Ϊ�����������,���TIM3 CH2��PWM���岨��	GPIOB.5
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3; //TIM_CH2
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIO
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_TIM2);
   //��ʼ��TIM3
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
 /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //����ΪPWMģʽ1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;	   //��������ֵ�������������������ֵʱ����ƽ��������
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ
//TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
//TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
//TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
//TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;	
	
  TIM_OC1Init(TIM2, &TIM_OCInitStructure);	 //ʹ��ͨ��1

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;	  //����ͨ��2�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
  TIM_OC2Init(TIM2, &TIM_OCInitStructure);	  //ʹ��ͨ��2

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;	//����ͨ��3�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
  TIM_OC3Init(TIM2, &TIM_OCInitStructure);	 //ʹ��ͨ��3

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;	//����ͨ��4�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
  TIM_OC4Init(TIM2, &TIM_OCInitStructure);	//ʹ��ͨ��4


	TIM_Cmd(TIM2, ENABLE);  //ʹ��TIM3
	//TIM_CtrlPWMOutputs(TIM2, ENABLE);

}


void PWMInit(void)
{
		TIM2_PWM_Init(10000,0);
		//TIM_SetCompare3(TIM3,2000);	
}


//********************************************************************
// ������: MotorInit() 
// ˵  ��: ������Ƴ�ʼ������
// ʱ  ��: 2011.10.29    
// ��  ��: ��
// ��  ��: ��
// ��  ��: ��
// ע  ��: 
//********************************************************************    
// �޸���:
// ʱ  ��:
// ��  ��:
//********************************************************************
void MotorInit(void)
{
   PWMInit();

   // ********* �û�������Ƶ��ת��IO�ڳ�ʼ���Ĵ��� **********

}


//********************************************************************
// ������: MotorLCtl()/MotorRCtl() 
// ˵  ��: ��/�� ������ƺ���
// ʱ  ��: 2011.10.29    
// ��  ��: speed: ת�٣�ȡֵ: 0��100
//         direction: ת��ȡֵ: 1:��ת  2:��ת����ʹ����ǰΪ����
// ��  ��: ��
// ��  ��: ��
// ע  ��: 
//********************************************************************    
// �޸���:
// ʱ  ��:
// ��  ��:
//********************************************************************
void MotorLCtl(int16_t speed,unsigned char direction)
{


  // ********* �û�������ߵ�����ٴ��� **********
  
  if(direction == 1) //����
    {
      TIM_SetCompare1(TIM2,speed);	
			TIM_SetCompare2(TIM2,0);	
    }
   
  else if(direction == 2)
    {
      TIM_SetCompare1(TIM2,0);	
			TIM_SetCompare2(TIM2,speed);	
    }
  else
    {
      // ********* �û����������� **********
      // ���ѣ������ӿ����ڻ����˵�·�϶ຸ����LED��Ϊ����ָʾ�ƣ�
      // �Բ�ͬ��ָʾ������ָʾ�������Ӳ����������Ӧ���󣬷����
      // �ԣ�ʹ��������Ͻ�
      while(1);
    }
}

void MotorRCtl(int16_t speed,unsigned char direction)
{


  // ********* �û������ұߵ�����ٴ��� **********
  
  if(direction == 1)
    {
      TIM_SetCompare3(TIM2,speed);	
			TIM_SetCompare4(TIM2,0);	
    }
   
  else if(direction == 2)
    {
      TIM_SetCompare3(TIM2,0);	
			TIM_SetCompare4(TIM2,speed);	
    }
  else
    {
      // ********* �û����������� **********
      // ���ѣ������ӿ����ڻ����˵�·�϶ຸ����LED��Ϊ����ָʾ�ƣ�
      // �Բ�ͬ��ָʾ������ָʾ�������Ӳ����������Ӧ���󣬷����
      // �ԣ�ʹ��������Ͻ�
      while(1);
    }

}


//********************************************************************
// ������: STOP() 
// ˵  ��: ͣ������
// ʱ  ��: 2011.10.29    
// ��  ��: ��
// ��  ��: ��
// ��  ��: ��
// ע  ��: 
//********************************************************************    
// �޸���:
// ʱ  ��:
// ��  ��:
//********************************************************************
void STOP(void)
{
   PWMInit();
}


//********************************************************************
// ������: MotorCtl() 
// ˵  ��: ������ƺ���
// ʱ  ��: 2011.10.29    
// ��  ��: speed: Ѳ�߻����˻��������ٶ�,ȡֵ: 0��100
//         PIDCtlOut: PID�㷨���������ȡֵ: -100��+100
// ��  ��: ��
// ��  ��: ��
// ע  ��: ע���飬���ܷ��������⣡
//********************************************************************    
// �޸���:
// ʱ  ��:
// ��  ��:
//********************************************************************


void MotorCtl( int16_t speedl,int16_t speedr,int16_t PIDCtlOut)// 
{
   int16_t speedL = 0,
                 speedR = 0;
   speedL = speedl - PIDCtlOut;
   speedR = speedr + PIDCtlOut;
   
   if(speedL < 0)
     {
        MotorLCtl(-speedL,2);
     }
   else
    {  
       MotorLCtl(speedL,1);
    }

   if(speedR < 0)
   {
      MotorRCtl(-speedR,2);
   }
   else
    {  
       MotorRCtl(speedR,1);
    }
}
