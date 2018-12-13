#include "myapp.h"
#include "sys.h"
#include "mb.h"
#include "mbport.h"


//===串口调试，支持printf
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif 
//===
//串口2的接收缓存数组
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
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0); //PB0 置 1
		
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
	//串口调试功能
	if(huart == (&huart1))
	{
		
	}
	//modbus串口
	if(huart == (&huart2))
	{
		//xMBPortSerialGetByte(&A);
		//xMBPortSerialPutByte(A);
		
		//HAL_UART_Transmit(&huart2 ,(uint8_t *)&huart2_aRxBuffer,1,0x01);
		//xMBPortSerialPutByte((CHAR)huart2_aRxBuffer[0]);
		
		//xMBPortSerialPutByte(A);
		//HAL_UART_Transmit_IT(&huart2 ,(uint8_t *)&huart2_aRxBuffer,1);
		prvvUARTRxISR();//接收中断处理函数
		HAL_UART_Receive_IT(&huart2, (uint8_t *)huart2_aRxBuffer, 1);
		
	}
}

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart == (&huart2))
//	{
//		
//		printf("dsfsdfsfddd");
//		prvvUARTTxReadyISR();//发送完成终端处理函数	
//	}
//}

