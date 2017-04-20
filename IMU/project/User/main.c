/**
  ******************************************************************************
  * @file    main.c 
  * @author  Bui Van Phat
  * @version 1.0
  * @date
  * @brief	Main program body
  ******************************************************************************
  */
#include "main.h"
#include "includes.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "MadgwickAHRS.h"

GPIO_InitTypeDef    GPIO_InitStructure;

#define RAD_TO_DEG 57.295779513082320876798154814105f
#define DEG_TO_RAD 3.14159265358979323846/180.0f
extern void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
extern void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);



int16_t MPU6050data[7],HMC[3];
float GyroMeasError = 3.14159265358979f * (40.0f / 180.0f);

float Gyro_X = 0 ,Gyro_Y = 0 ,Gyro_Z = 0 ,Acc_X = 0 ,Acc_Y = 0 ,Acc_Z = 0 ,Mag_X = 0 ,Mag_Y = 0 ,Mag_Z = 0;
float Roll =0,Pitch =0,Yaw=0;


float beta=0.906900;//betaDef[1];//sqrt(3.0f / 4.0f)*GyroMeasError;//GyroMeasError;
float Delta_T =0.0025;
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

float Gyro_Bias[3]={-0.870229 , 1.7328244 , 0.6946565}; //{0.0304878 , -0.0434939 , -0.0768049};

float Acc_Bias[3] = {0.02002 , -0.02246 , -0.02930};	

float MAG_MATRIX[3][3] ={{0.00262137 , 0.00015803 , 0.0000326624},
                         {0.00015803 , 0.00321909 , 0.0000287375},
                         {0.0000326624 , 0.0000287375 , 0.00264195}};
float MAG_BIAS[3] ={-110.5788 , -10.3471 , -157.9546};


uint8_t Check_Get_Data;
char str[25];
uint16_t time;

void I2C_Configuration(void);
int main(void)
{
	
  I2C_Configuration();
	Uart_Config(E_USART5,115200,1,ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    //khoi tao GPIOD
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);  
	SysTick_Configuration();
	
  Delay(1000);
	MPU6050_Initialize();
	HMC5883L_Initialize();
	
  while(1)
	{		
		 MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS, &Check_Get_Data, MPU6050_RA_INT_STATUS, 1); 
		if((Check_Get_Data & 0x01)== 1)
		 {			 
			if(time == 10)
			{
				time =0;
				GPIO_ToggleBits(GPIOD,GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
				sprintf(str,"%f \t  %f \t  %f \t\r\n",Roll,Pitch,Yaw);
        USART_puts(UART5, str);
			}
      
			 MPU6050_GetRawAccelTempGyro(MPU6050data);	
	     HMC5883L_GetHeading(HMC);			 
		  
			 Acc_X = ((float)MPU6050data[0]/2048.0f)-Acc_Bias[0];
			 Acc_Y = ((float)MPU6050data[1]/2048.0f)-Acc_Bias[1];
			 Acc_Z = ((float)MPU6050data[2]/2048.0f)-Acc_Bias[2];
				
			 Gyro_X = ((float)MPU6050data[4])/16.4f - Gyro_Bias[0];
			 Gyro_Y = ((float)MPU6050data[5])/16.4f - Gyro_Bias[1];
			 Gyro_Z = ((float)MPU6050data[6])/16.4f - Gyro_Bias[2];
		   
			 Mag_X = (HMC[0]- MAG_BIAS[0])*MAG_MATRIX[0][0] + (HMC[1]- MAG_BIAS[1])*MAG_MATRIX[0][1]+(HMC[2]- MAG_BIAS[2]) * MAG_MATRIX[0][2];
			 Mag_Z = (HMC[0]- MAG_BIAS[0])*MAG_MATRIX[1][0] + (HMC[1]- MAG_BIAS[1])*MAG_MATRIX[1][1]+(HMC[2]- MAG_BIAS[2]) * MAG_MATRIX[1][2];
			 Mag_Y = (HMC[0]- MAG_BIAS[0])*MAG_MATRIX[2][0] + (HMC[1]- MAG_BIAS[1])*MAG_MATRIX[2][1]+(HMC[2]- MAG_BIAS[2]) * MAG_MATRIX[2][2];
			
			 MadgwickAHRSupdate(Gyro_X*DEG_TO_RAD, Gyro_Y*DEG_TO_RAD, Gyro_Z*DEG_TO_RAD,Acc_X,Acc_Y,Acc_Z,Mag_X ,Mag_Y,Mag_X);
			
			Roll  = RAD_TO_DEG*atan2(2.0f * (q0 * q1 + q2 * q3),1-2*q1*q1-2*q2*q2);
      Pitch = RAD_TO_DEG*asin(2.0f * (q0*q2-q3*q1));			    
      Yaw  = RAD_TO_DEG*atan2(2.0f * (q0 * q3 + q1 * q2),1-2*q2*q2-2*q3*q3);
			 time++;			 
		 }
  }
} 		
		
	

void I2C_Configuration(void)
{
#ifdef FAST_I2C_MODE
 #define I2C_SPEED 400000
 #define I2C_DUTYCYCLE I2C_DutyCycle_16_9  
#else /* STANDARD_I2C_MODE*/
 #define I2C_SPEED 100000
 #define I2C_DUTYCYCLE I2C_DutyCycle_2
#endif /* FAST_I2C_MODE*/
	
  GPIO_InitTypeDef  GPIO_InitStructure;
  I2C_InitTypeDef   I2C_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_I2C1);
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_I2C1);	

  I2C_DeInit(I2C1);
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DUTYCYCLE;
  I2C_InitStructure.I2C_OwnAddress1 = 0;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_ClockSpeed = 100000;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init(I2C1, &I2C_InitStructure);
  I2C_Cmd(I2C1, ENABLE); 

}

/*******************************************************************************
Function name: USE_FULL_ASSERT
Decription: None
Input: None
Output: None
*******************************************************************************/
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
