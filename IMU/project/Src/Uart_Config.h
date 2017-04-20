#include "stm32f4xx.h"


/* Cau Hinh Module Uart cho STM32F4 */

/*   Lua Chon Uart
		 Toc Do Giao Tiep. 
		 Lua Chon Bo Uart cua Module Do. 1 hoac 2
		 Cho phep ngat hay khong... ENABLE, DISABLE.
		 Ten Chuong Trinh Ngat Duoc Hieu La Mac Dinh Voi Tat Ca Cac Module.
				-	USART1_IRQn 
				-	USART2_IRQn 
				-	USART3_IRQn 
				- UART4_IRQn 
				- UART5_IRQn 
				-	USART6_IRQn 

*/

typedef enum
{
	E_USART1,
	E_USART2,
	E_USART3,
	E_USART4, 
	E_USART5,
	E_USART6,
	         
} Select_USART;
			  



void Uart_Config( Select_USART UART_Num, uint32_t baudrate , uint8_t Num,FunctionalState State);
 
void USART_puts(USART_TypeDef* USARTx, volatile char *s);
