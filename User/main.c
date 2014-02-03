/****************************************************/
//    在此DEMO中加入USART1/USART2的输出与GPIOE接口的1
//个状态灯的定义,并且打开了FPU，加入了ARM_MATH.H
//数学运算头文件；系统时钟来源为HSE 25MHz，系统主
//频定义为最快速度168MHz.
/****************************************************/
 

#include "MyF4Config.h"
//#include "5110.h"  
#include "MotorCtl.h"   // 电机控制
//#include "24l01.h" 
#include "lcd.h"
#include "counter.h"   //编码器
#include "adc.h"
#include "filter.h" //滤波器
#include "pid.h" 
#include "visual scope.h"  //串口调试 波形工具

#define ANG_center 1800  
#define GYRO_center 2280

int16_t 	velocity[2]; //测速
float OutputL,OutputR; //左右输出

// 速度，角度控制
float speedL_accumulate=0,speedR_accumulate=0;
float P_angle=20;   //角度
float D_angle=1;   //角度
float P_speed=0;   //速度
float D_speed=0;   //速度
void AngleControl(xyz *temp,uint16_t angle,int16_t set_speed) 
{

	int16_t speedL_Delta,speedR_Delta;
	float AngleControl;
	
	AngleControl = P_angle * (angle-ANG_center) + D_angle * (temp->g-GYRO_center); //角度控制

	//速度控制
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
	int16_t 	speed;  //指定速度
  RCC_Config();       //系统时钟配置	
	ADC_INIT();  //初始AD   
	USART_Config(); 
	delay_init();
	MotorInit();  //初始电机
//	NRF24L01_Init();    	//初始化NRF24L01 
	TIM_Mode_Config();  //初始编码器
	LCD_Init();  //初始LCD
	filter_init(); // 初始滤波器
	pid_init(); //初始PID
	TIM3_Mode_Config();  //测速定时器
	while(1)
	{		
		temp = adcfilter_get();  //得到原始数据
		filter_input(temp); //ad采集到的数据做均值滤波
		angle = comple_filter(temp,GYRO_center); //互补滤波的角度		
		temp = adcfilter_getCurrent(); //均值滤波
		

		visual_scope(angle,temp->z,100,10000);  //串口上位机 
		AngleControl(temp,angle,speed); //角度 速度控制
		MotorCtl(OutputL, OutputR,0); // 后为方向控制 
		//printf(" LCD ID:%x\r\n",lcddev.id); //打印LCD ID  
		delay_ms(5);
		
		
		/*********************  显示 *************************/
		if(velocity[0]<0)
			LCD_ShowxNum(0,100,-velocity[0],5,16,0);//显示ADC的值
		else
			LCD_ShowxNum(0,100,velocity[0],5,16,0);//显示ADC的值
		if(velocity[1]<0)
			LCD_ShowxNum(0,116,-velocity[1],5,16,0);//显示ADC的值
		else
			LCD_ShowxNum(0,116,velocity[1],5,16,0);//显示ADC的值
		LCD_ShowString(140,130,200,16,16,"x");LCD_ShowxNum(156,130,temp->x,4,16,0);//显示ADC的值
		LCD_ShowString(140,146,200,16,16,"y");LCD_ShowxNum(156,146,temp->y,4,16,0);//显示ADC的值
	  LCD_ShowString(140,162,200,16,16,"z");LCD_ShowxNum(156,162,temp->z,4,16,0);//显示ADC的值
		LCD_ShowString(140,178,200,16,16,"g");LCD_ShowxNum(156,178,temp->g,4,16,0);LCD_ShowxNum(250,178,pid_iterate(temp->z-1900),5,16,0);//显示ADC的值//显示ADC的值
		LCD_ShowString(140,194,200,16,16,"a");LCD_ShowxNum(156,194,temp->ang,4,16,0);//显示ADC的值
		LCD_ShowxNum(156,210,temp->z,4,16,0);//显示ADC的值		
/****************************************************/
	}
}



//测速
void TIM3_IRQHandler()
{
  TIM_ClearFlag(TIM3,TIM_FLAG_Update);//???????????	
	velocity[0]=20000-TIM1->CNT ;
	velocity[1]=20000-TIM4->CNT;;	
	TIM1->CNT = 20000;
	TIM4->CNT = 20000;
}

//串口接收中断
void USART3_IRQHandler(void)
{
//			uint8_t data;
    if(USART_GetFlagStatus(USART3,USART_FLAG_RXNE) == SET)
    {
        USART_ClearFlag(USART3,USART_FLAG_RXNE);			
    }
}
