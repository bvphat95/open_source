#include "Uart_Config.h"

void Uart_Config(  Select_USART UART_Num, uint32_t baudrate, uint8_t Num,FunctionalState State){
	
	GPIO_InitTypeDef GPIO_InitStructure; 
	NVIC_InitTypeDef NVIC_InitStructure; 
	USART_InitTypeDef USART_InitStructure;
	
	
	
//  Config Pin	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)

	
//	Config UART
	USART_InitStructure.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStructure.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStructure.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	
	
	
	// Config NVIC
	
	if(State != DISABLE)
	{	
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART1 interrupts
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
		
	}
	else
	{
	}
	
	switch(UART_Num)
	{	
		case E_USART1:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
			switch(Num)
			{
				case 1:
				{
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
					
					GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
					GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
					GPIO_Init(GPIOA, &GPIO_InitStructure);					
				 
					GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
					GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
					GPIO_Init(GPIOA, &GPIO_InitStructure);		
					
					GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
					GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
					
					break;
				}
				case 2:
				{
				  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
					GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6; 	
					GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
					GPIO_Init(GPIOB, &GPIO_InitStructure);					
				 
					GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
					GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
				
					GPIO_Init(GPIOB, &GPIO_InitStructure);		
					GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1); //
					GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
					break;
				}
				default: break;
			}
			
			
	
			USART_Init(USART1, &USART_InitStructure);				
	
			
			
			
			if(State == ENABLE)
			{
			
			  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn ;
				NVIC_Init(&NVIC_InitStructure);
				USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
			}
			else
			{
				
			}
			USART_Cmd(USART1, ENABLE);

		break;		
		}
		
		case E_USART2:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
			switch(Num)
			{
				case 1:
				{
			  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
					
					GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2; 	
					GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
					GPIO_Init(GPIOA, &GPIO_InitStructure);					
				 
					GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;
					GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
					GPIO_Init(GPIOA, &GPIO_InitStructure);		
					
					GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
					GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
					break;
				}
				case 2:
				{
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
					
					GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5; 	
					GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
					GPIO_Init(GPIOD, &GPIO_InitStructure);				
				 
					GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
					GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
					GPIO_Init(GPIOD, &GPIO_InitStructure);		
					
					GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2); //
					GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
					break;
				}
				default: break;
			}
				
			USART_Init(USART2, &USART_InitStructure);					
			
			if(State != DISABLE)
			{
				NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn ;
				NVIC_Init(&NVIC_InitStructure);
				USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt
			}
			else
			{
				
			}
			USART_Cmd(USART2, ENABLE);

		break;		
		}
		
		
		case E_USART3:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
			switch(Num)
			{
				case 1:
				{
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
				
					GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10; 	
					GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
					GPIO_Init(GPIOB, &GPIO_InitStructure);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers
				 
					GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;
					GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
					GPIO_Init(GPIOB, &GPIO_InitStructure);		

					GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3); //
					GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
					break;
				}
				case 2:
				{
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
					
					
					GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8; 	
					GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
					GPIO_Init(GPIOD, &GPIO_InitStructure);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers
				 
					GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
					GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
					GPIO_Init(GPIOD, &GPIO_InitStructure);		
					
					GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3); //
					GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
					break;
				}
				default: break;
			}
			
			
	
			USART_Init(USART3, &USART_InitStructure);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting
	
			// Cho Phep Ngat Nhan Hay Khong..
			
			
			if(State != DISABLE)
			{
				NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn ;
				NVIC_Init(&NVIC_InitStructure);
				USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt
			}
			else
			{
				
			}
			USART_Cmd(USART3, ENABLE);

			
		break;		
		}
	
		/* Cau Hinh Bo Uart 4 */
		case E_USART4:
		{
			 RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
			// Cau hinh Cho Phep Cung Cap Xung Uart
			
			// Cau Hinh Chan Uart 4
			switch(Num)
			{
				case 1:
				{
					
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
					GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0; 	
					GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
					GPIO_Init(GPIOA, &GPIO_InitStructure);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers
				 
					GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;
					GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
					GPIO_Init(GPIOA, &GPIO_InitStructure);		
					
					GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
					GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);
					break;
				}
				case 2:
				{
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
					GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0; 	
					GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
					GPIO_Init(GPIOA, &GPIO_InitStructure);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers
				 
					GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;
					GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
					GPIO_Init(GPIOA, &GPIO_InitStructure);		
					
					GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
					GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);
					
					break;
				}
				default: break;
			}
			
			
	
			USART_Init(UART4, &USART_InitStructure);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting
	
			// Cho Phep Ngat Nhan Hay Khong..
			
			
			if(State != DISABLE)
			{
				NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn ;
				NVIC_Init(&NVIC_InitStructure);
				USART_ITConfig(UART4, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt
			}
			else
			{
				
			}
			USART_Cmd(UART4, ENABLE);

		break;		
		}
		
		
		//* Cau Hinh Bo Uart 5*/
		
		
				case E_USART5:
		{
			 RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
			// Cau hinh Cho Phep Cung Cap Xung Uart
			
			// Cau Hinh Chan Uart 4
			switch(Num)
			{
				case 1:
				{
					
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
					GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12; 	
					GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
					GPIO_Init(GPIOC, &GPIO_InitStructure);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers
				 
					GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
					GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
					GPIO_Init(GPIOD, &GPIO_InitStructure);		
					
					GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
					GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
					break;
				}
				case 2:
				{
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
					GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12; 	
					GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
					GPIO_Init(GPIOC, &GPIO_InitStructure);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers
				 
					GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
					GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
					GPIO_Init(GPIOD, &GPIO_InitStructure);		
					
					GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
					GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
					
					break;
				}
				default: break;
			}
			
			
	
			USART_Init(UART5, &USART_InitStructure);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting
	
			// Cho Phep Ngat Nhan Hay Khong..
			
			
			if(State != DISABLE)
			{
				NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn ;
				NVIC_Init(&NVIC_InitStructure);
				USART_ITConfig(UART5, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt
			}
			else
			{
				
			}
			USART_Cmd(UART5, ENABLE);

		break;		
		}
		
		/* Cau Hinh Bo Uart 6 */
		case E_USART6:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
			// Cau hinh Cho Phep Cung Cap Xung Uart
		

			// Cau Hinh Chan Uart 6
			switch(Num)
			{
				case 1:
				{
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
					GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6; 	
					GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
					GPIO_Init(GPIOC, &GPIO_InitStructure);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers
				 
					GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
					GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
					GPIO_Init(GPIOC, &GPIO_InitStructure);		
					
					GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6); //
					GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
					break;
				}
				case 2:
				{
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
					GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9; 	
					GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
					GPIO_Init(GPIOG, &GPIO_InitStructure);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers
				 
					GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14;
					GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
					GPIO_Init(GPIOG, &GPIO_InitStructure);		
					
					GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_USART6); //
					GPIO_PinAFConfig(GPIOG, GPIO_PinSource14, GPIO_AF_USART6);
					break;
				}
				default: break;
			}
			
			
	
			USART_Init(USART6, &USART_InitStructure);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting
	
			// Cho Phep Ngat Nhan Hay Khong..
			
			
			if(State != DISABLE)
			{
				NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn ;
				NVIC_Init(&NVIC_InitStructure);
				USART_ITConfig(USART6, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt
			}
			else
			{
				
			}
			USART_Cmd(USART6, ENABLE);

		break;		
		}
		default: break;
	}
}	


void USART_puts(USART_TypeDef* USARTx, volatile char *s){

	while(*s){
		// wait until data register is empty
		while( !(USART_GetFlagStatus(USARTx,USART_FLAG_TXE)));
		USART_SendData(USARTx, *s);
		*s++;
	}
}

		
