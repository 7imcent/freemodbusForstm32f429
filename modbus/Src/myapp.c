#include "myapp.h"
#include "sys.h"
#include "mb.h"
#include "mbport.h"


//===���ڵ��ԣ�֧��printf
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif 
//===
//����2�Ľ��ջ�������
uint8_t huart2_aRxBuffer[1];
extern void prvvTIMERExpiredISR(void);
extern void prvvUARTTxReadyISR( void );
extern void prvvUARTRxISR( void );
extern volatile UCHAR  ucRTUBuf[256];

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		//int jjj;
	if(htim==(&htim6))
	{	
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0); //PB0 �� 1
		
		prvvTIMERExpiredISR();	
//		for(jjj = 0; jjj<10 ;jjj++)
//		{
//			printf("%x ",ucRTUBuf[jjj]);
//		}
			
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//CHAR  A ='a';
	//���ڵ��Թ���
	if(huart == (&huart1))
	{
		
	}
	//modbus����
	if(huart == (&huart2))
	{
		//xMBPortSerialGetByte(&A);
		//xMBPortSerialPutByte(A);
		
		//HAL_UART_Transmit(&huart2 ,(uint8_t *)&huart2_aRxBuffer,1,0x01);
		//xMBPortSerialPutByte((CHAR)huart2_aRxBuffer[0]);
		
		//xMBPortSerialPutByte(A);
		//HAL_UART_Transmit_IT(&huart2 ,(uint8_t *)&huart2_aRxBuffer,1);
		prvvUARTRxISR();//�����жϴ�����
		HAL_UART_Receive_IT(&huart2, (uint8_t *)huart2_aRxBuffer, 1);
		
	}
}

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart == (&huart2))
//	{
//		
//		printf("dsfsdfsfddd");
//		prvvUARTTxReadyISR();//��������ն˴�����	
//	}
//}

