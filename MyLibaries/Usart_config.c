#include "MyF4Config.h"

/*****************************************/
/*************ϵͳUSART1/2����************/
/*****************************************/
void USART_Config()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;     //����USART��ʼ���ṹ��	
	NVIC_InitTypeDef nvicParam;
	/*****************************************/
	/************����USART1������*************/
	/*****************************************/
	//TX
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;               //ѡ��GPIOB��6��
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;          //����ѡ�е����Ź���ģʽ��IN������ģʽ��OUT�����ģʽ��AF������ģʽ��AN��ģ��ģʽ
	GPIO_Init(GPIOB, &GPIO_InitStructure);        //��ʼ�������õ�GPIOD����
  //RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;                //ѡ��GPIOB��7��
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;          //����ѡ�е����Ź���ģʽ��IN������ģʽ��OUT�����ģʽ��AF������ģʽ��AN��ģ��ģʽ
                       
  GPIO_Init(GPIOB , &GPIO_InitStructure);
	

/*****************************************/
/**********ϵͳUSART�˿�����**************/
/*****************************************/

	USART_Cmd(USART3 , DISABLE);          //�رմ���
	//GPIOA��2��3�Ÿ���ΪUSART1������
  GPIO_PinAFConfig(GPIOB , GPIO_PinSource10 , GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOB , GPIO_PinSource11 , GPIO_AF_USART3);
	
	//����1��������
	USART_InitStructure.USART_BaudRate = 115200;                                      //�����ʣ�115200��57600��38400��9600��4800��2400��1200
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;                       //��ʾһ��֡�з��ͻ��߽��յ�������λ
  USART_InitStructure.USART_StopBits = USART_StopBits_1;                            //ֹͣλ
  USART_InitStructure.USART_Parity = USART_Parity_No ;                               //��żģʽ
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   //ʹ�ܻ�ر�Ӳ��������ģʽ
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                   //ʹ�ܻ�رշ��ͺͽ���ģʽ   

	USART_Init(USART3 , &USART_InitStructure);
	//ʹ��USART1
	
	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
	
  nvicParam.NVIC_IRQChannel = USART3_IRQn;
  nvicParam.NVIC_IRQChannelPreemptionPriority = 8;
  nvicParam.NVIC_IRQChannelSubPriority = 8;
  nvicParam.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicParam);
		
		
	
	USART_Cmd(USART3, ENABLE);
	
}

/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/
/*
 * ��������itoa
 * ����  ������������ת�����ַ���
 * ����  ��-radix =10 ��ʾ10���ƣ��������Ϊ0
 *         -value Ҫת����������
 *         -buf ת������ַ���
 *         -radix = 10
 * ���  ����
 * ����  ����
 * ����  ����USART2_printf()����
 */
static char *itoa(int value, char *string, int radix)
{
    int     i, d;
    int     flag = 0;
    char    *ptr = string;

    /* This implementation only works for decimal numbers. */
    if (radix != 10)
    {
        *ptr = 0;
        return string;
    }

    if (!value)
    {
        *ptr++ = 0x30;
        *ptr = 0;
        return string;
    }

    /* if this is a negative value insert the minus sign. */
    if (value < 0)
    {
        *ptr++ = '-';

        /* Make the value positive. */
        value *= -1;
    }

    for (i = 10000; i > 0; i /= 10)
    {
        d = value / i;

        if (d || flag)
        {
            *ptr++ = (char)(d + 0x30);
            value -= (d * i);
            flag = 1;
        }
    }

    /* Null terminate the string. */
    *ptr = 0;

    return string;

} /* NCL_Itoa */

/*
 * ��������USART2_printf
 * ����  ����ʽ�������������C���е�printf��������û���õ�C��
 * ����  ��-USARTx ����ͨ��������ֻ�õ��˴���2����USART2
 *		     -Data   Ҫ���͵����ڵ����ݵ�ָ��
 *			   -...    ��������
 * ���  ����
 * ����  ���� 
 * ����  ���ⲿ����
 *         ����Ӧ��USART2_printf( USART2, "\r\n this is a demo \r\n" );
 *            		 USART2_printf( USART2, "\r\n %d \r\n", i );
 *            		 USART2_printf( USART2, "\r\n %s \r\n", j );
 */
void USART_printf(USART_TypeDef* USARTx, uint8_t *Data,...)
{
	const char *s;
  int d;   
  char buf[16];

  va_list ap;
  va_start(ap, Data);

	while ( *Data != 0)     // �ж��Ƿ񵽴��ַ���������
	{				                          
		if ( *Data == 0x5c )  //'\'
		{									  
			switch ( *++Data )
			{
				case 'r':							          //�س���
					USART_SendData(USARTx, 0x0d);
					Data ++;
					break;

				case 'n':							          //���з�
					USART_SendData(USARTx, 0x0a);	
					Data ++;
					break;
				
				default:
					Data ++;
				    break;
			}			 
		}
		else if ( *Data == '%')
		{									  //
			switch ( *++Data )
			{				
				case 's':										  //�ַ���
					s = va_arg(ap, const char *);
          for ( ; *s; s++) 
					{
						USART_SendData(USARTx,*s);
						while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
          }
					Data++;
          break;

        case 'd':										//ʮ����
          d = va_arg(ap, int);
          itoa(d, buf, 10);
          for (s = buf; *s; s++) 
					{
						USART_SendData(USARTx,*s);
						while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
          }
					Data++;
          break;
				 default:
						Data++;
				    break;
			}		 
		} /* end of else if */
		else USART_SendData(USARTx, *Data++);
		while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
	}
}
/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/
