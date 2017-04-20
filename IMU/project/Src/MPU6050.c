/* Includes */
#include "MPU6050.h"
//#include "stm32f10x_i2c.h"
#include "SysTick.h"


#define TIME_OUT 5000
uint32_t I2C_TIME_OUT = 0;
/** @defgroup MPU6050_Library
* @{
*/

/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
 * the clock source to use the X Gyro for reference, which is slightly better than
 * the default internal clock source.
 */
void MPU6050_Initialize() 
{
	 uint8_t REG_0x6AB = 0,REG_0x37 = 2;
   uint8_t MPLRT_DIV = 0x04,C,Clear_Seft_Test,DLPF_BW_42 = 0x03;
	 uint8_t AFS,INT_ENABLE_W = 0x01 ,INT_PIN_CFG_W = 0x22;
	
	  MPU6050_SetClockSource(MPU6050_CLOCK_PLL_XGYRO);

	  MPU6050_I2C_ByteWrite(MPU6050_DEFAULT_ADDRESS,&DLPF_BW_42,MPU6050_RA_CONFIG);  // Set DLPF Bandwith 42 HZ.

  	MPU6050_I2C_ByteWrite(MPU6050_DEFAULT_ADDRESS,&MPLRT_DIV,MPU6050_RA_SMPLRT_DIV); // Sample Rate 200 HZ.
	
    MPU6050_SetSleepModeStatus(DISABLE);
	  
	
	  MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS, &C ,MPU6050_RA_GYRO_CONFIG,1);
	  // Clear Seft Test;
	  Clear_Seft_Test = C & ~0xE0;
	  MPU6050_I2C_ByteWrite(MPU6050_DEFAULT_ADDRESS, &Clear_Seft_Test,MPU6050_RA_GYRO_CONFIG);
	  
	  // Clear AFS bit;
	  AFS = C & ~0x18;
	  MPU6050_I2C_ByteWrite(MPU6050_DEFAULT_ADDRESS, &AFS,MPU6050_RA_GYRO_CONFIG); 
	  MPU6050_SetFullScaleGyroRange(MPU6050_GYRO_FS_2000);
	
	  
    MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS, &C ,MPU6050_RA_ACCEL_CONFIG,1);
	  // Clear Seft Test;
	  Clear_Seft_Test = C & ~0xE0;
	  MPU6050_I2C_ByteWrite(MPU6050_DEFAULT_ADDRESS, &Clear_Seft_Test,MPU6050_RA_ACCEL_CONFIG);
	  
	  // Clear AFS bit;
	  AFS = C & ~0x18;
	  MPU6050_I2C_ByteWrite(MPU6050_DEFAULT_ADDRESS, &AFS,MPU6050_RA_ACCEL_CONFIG); 	
    MPU6050_SetFullScaleAccelRange(MPU6050_ACCEL_FS_16);
	  
	
	  // INT Enable read data.
    MPU6050_I2C_ByteWrite(MPU6050_DEFAULT_ADDRESS,&INT_ENABLE_W,MPU6050_RA_INT_ENABLE); 
  	MPU6050_I2C_ByteWrite(MPU6050_DEFAULT_ADDRESS,&REG_0x6AB,MPU6050_RA_USER_CTRL);
		MPU6050_I2C_ByteWrite(MPU6050_DEFAULT_ADDRESS,&INT_PIN_CFG_W,MPU6050_RA_INT_PIN_CFG);
		
		
	
        	  


}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, FALSE otherwise
 */
bool MPU6050_TestConnection() 
{
    if(MPU6050_GetDeviceID() == 0x34) //0b110100; 8-bit representation in hex = 0x34
      return TRUE;
    else
      return FALSE;
}
// WHO_AM_I register

/** Get Device ID.
 * This register is used to verify the identity of the device (0b110100).
 * @return Device ID (should be 0x68, 104 dec, 150 oct)
 * @see MPU6050_RA_WHO_AM_I
 * @see MPU6050_WHO_AM_I_BIT
 * @see MPU6050_WHO_AM_I_LENGTH
 */
uint8_t MPU6050_GetDeviceID()
{
    uint8_t tmp;
    MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, &tmp);
    return tmp; 
}
/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source New clock source setting
 * @see MPU6050_GetClockSource()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CLKSEL_BIT
 * @see MPU6050_PWR1_CLKSEL_LENGTH
 */
void MPU6050_SetClockSource(uint8_t source) 
{
    MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see MPU6050_GetFullScaleGyroRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_SetFullScaleGyroRange(uint8_t range) 
{
    MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

// GYRO_CONFIG register

/** Get full-scale gyroscope range.
 * The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below.
 *
 * <pre>
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 * </pre>
 *
 * @return Current full-scale gyroscope range setting
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
uint8_t MPU6050_GetFullScaleGyroRange() 
{
    uint8_t tmp;
    MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, &tmp);
    return tmp;
}
/** Get full-scale accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 * </pre>
 *
 * @return Current full-scale accelerometer range setting
 * @see MPU6050_ACCEL_FS_2
 * @see MPU6050_RA_ACCEL_CONFIG
 * @see MPU6050_ACONFIG_AFS_SEL_BIT
 * @see MPU6050_ACONFIG_AFS_SEL_LENGTH
 */
uint8_t MPU6050_GetFullScaleAccelRange() 
{
    uint8_t tmp;
    MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, &tmp);
    return tmp;
}
/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see MPU6050_GetFullScaleAccelRange()
 */
void MPU6050_SetFullScaleAccelRange(uint8_t range) 
{
    MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/** Get sleep mode status.
 * Setting the SLEEP bit in the register puts the device into very low power
 * sleep mode. In this mode, only the serial interface and internal registers
 * remain active, allowing for a very low standby current. Clearing this bit
 * puts the device back into normal mode. To save power, the individual standby
 * selections for each of the gyros should be used if any gyro axis is not used
 * by the application.
 * @return Current sleep mode enabled status
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
bool MPU6050_GetSleepModeStatus() 
{
    uint8_t tmp;
    MPU6050_ReadBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, &tmp);
    if(tmp == 0x00)
      return FALSE;
    else
      return TRUE;    
}
/** Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see MPU6050_GetSleepModeStatus()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
void MPU6050_SetSleepModeStatus(FunctionalState NewState) 
{
    MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, NewState);
}

/** Get raw 6-axis motion sensor readings (accel/temp/gyro).
 * Retrieves all currently available motion sensor values.
 * @param AccelGyro 16-bit signed integer array of length 7
 * @see MPU6050_RA_ACCEL_XOUT_H
 * data 0 -> 2 : Accel
 * data 3      : Temp
 * data 4 -> 6 : Gyro
 */
void MPU6050_GetRawAccelTempGyro(s16* AccelGyro) 
{
    u8 dataR[14],i; 
    MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS, dataR, MPU6050_RA_ACCEL_XOUT_H, 14);

    for(i=0;i<7;i++) 
    AccelGyro[i]=(dataR[i*2]<<8)|dataR[i*2+1];   

}

/** Write multiple bits in an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 */
void MPU6050_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) 
{
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t tmp, mask;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);  
    mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    tmp &= ~(mask); // zero all important bits in existing byte
    tmp |= data; // combine data with existing byte
    MPU6050_I2C_ByteWrite(slaveAddr,&tmp,regAddr);   
}
/** write a single bit in an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 */
void MPU6050_WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) 
{
    uint8_t tmp;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);  
    tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
    MPU6050_I2C_ByteWrite(slaveAddr,&tmp,regAddr); 
}
/** Read multiple bits from an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 */
void MPU6050_ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) 
{
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t tmp, mask;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1); 
    mask = ((1 << length) - 1) << (bitStart - length + 1);
    tmp &= mask;
    tmp >>= (bitStart - length + 1);
    *data = tmp;
}

/** Read a single bit from an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 */
void MPU6050_ReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data) 
{
    uint8_t tmp;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);  
    *data = tmp & (1 << bitNum);
}

/**
* @brief  Initializes the I2C peripheral used to drive the MPU6050
* @param  None
* @return None
*/
/**
* @brief  Writes one byte to the  MPU6050.
* @param  slaveAddr : slave address MPU6050_DEFAULT_ADDRESS
* @param  pBuffer : pointer to the buffer  containing the data to be written to the MPU6050.
* @param  writeAddr : address of the register in which the data will be written
* @return None
*/
void MPU6050_I2C_ByteWrite(u8 slaveAddr, u8* pBuffer, u8 writeAddr)
{
//  ENTR_CRT_SECTION();

  /* Send START condition */
  I2C_GenerateSTART(MPU6050_I2C, ENABLE);

  /* Test on EV5 and clear it */
  I2C_TIME_OUT = TIME_OUT;
  while(!(I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT)) &&  (I2C_TIME_OUT != 0))
	{
		I2C_TIME_OUT--;
	}
	
	
	if(I2C_TIME_OUT == 0)
	{
		Stop_Communication();
	}
	
	

  /* Send MPU6050 address for write */
  I2C_Send7bitAddress(MPU6050_I2C, slaveAddr, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  I2C_TIME_OUT = TIME_OUT;
  while(!(I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) && (I2C_TIME_OUT != 0))
	{
		I2C_TIME_OUT--;
	}

	if(I2C_TIME_OUT == 0)
	{
		Stop_Communication();
	}
  /* Send the MPU6050's internal address to write to */
  I2C_SendData(MPU6050_I2C, writeAddr);
  I2C_TIME_OUT = TIME_OUT;
  /* Test on EV8 and clear it */
  while((!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) && (I2C_TIME_OUT != 0))
	{
		I2C_TIME_OUT--;
	}
	
	if(I2C_TIME_OUT == 0)
	{
		Stop_Communication();
	}
	
	
  /* Send the byte to be written */
  I2C_SendData(MPU6050_I2C, *pBuffer);

  /* Test on EV8 and clear it */
	I2C_TIME_OUT = TIME_OUT;
  while((!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) && (I2C_TIME_OUT != 0))
	{
		I2C_TIME_OUT--;
	}
	
	
	if(I2C_TIME_OUT == 0)
	{
		Stop_Communication();
	}
  /* Send STOP condition */
  I2C_GenerateSTOP(MPU6050_I2C, ENABLE);

 // EXT_CRT_SECTION();

}


void writeByte(u8 slaveAddr, u8* pBuffer, u8 writeAddr)
{
//  ENTR_CRT_SECTION();

  /* Send START condition */
  I2C_GenerateSTART(MPU6050_I2C, ENABLE);

  /* Test on EV5 and clear it */
  I2C_TIME_OUT = TIME_OUT;
  while(!(I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT)) &&  (I2C_TIME_OUT != 0))
	{
		I2C_TIME_OUT--;
	}
	
	
	if(I2C_TIME_OUT == 0)
	{
		Stop_Communication();
	}
	
	

  /* Send MPU6050 address for write */
  I2C_Send7bitAddress(MPU6050_I2C, slaveAddr, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  I2C_TIME_OUT = TIME_OUT;
  while(!(I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) && (I2C_TIME_OUT != 0))
	{
		I2C_TIME_OUT--;
	}

	if(I2C_TIME_OUT == 0)
	{
		Stop_Communication();
	}
  /* Send the MPU6050's internal address to write to */
  I2C_SendData(MPU6050_I2C, writeAddr);
  I2C_TIME_OUT = TIME_OUT;
  /* Test on EV8 and clear it */
  while((!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) && (I2C_TIME_OUT != 0))
	{
		I2C_TIME_OUT--;
	}
	
	if(I2C_TIME_OUT == 0)
	{
		Stop_Communication();
	}
	
	
  /* Send the byte to be written */
  I2C_SendData(MPU6050_I2C, *pBuffer);

  /* Test on EV8 and clear it */
	I2C_TIME_OUT = TIME_OUT;
  while((!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) && (I2C_TIME_OUT != 0))
	{
		I2C_TIME_OUT--;
	}
	
	
	if(I2C_TIME_OUT == 0)
	{
		Stop_Communication();
	}
  /* Send STOP condition */
  I2C_GenerateSTOP(MPU6050_I2C, ENABLE);

 // EXT_CRT_SECTION();

}

/**
* @brief  Reads a block of data from the MPU6050.
* @param  slaveAddr  : slave address MPU6050_DEFAULT_ADDRESS
* @param  pBuffer : pointer to the buffer that receives the data read from the MPU6050.
* @param  readAddr : MPU6050's internal address to read from.
* @param  NumByteToRead : number of bytes to read from the MPU6050 ( NumByteToRead >1  only for the Mgnetometer readinf).
* @return None
*/

void MPU6050_I2C_BufferRead(u8 slaveAddr, u8* pBuffer, u8 readAddr, u16 NumByteToRead)
{
 // ENTR_CRT_SECTION();

  /* While the bus is busy */

	I2C_TIME_OUT = TIME_OUT;
  while((I2C_GetFlagStatus(MPU6050_I2C, I2C_FLAG_BUSY)) && I2C_TIME_OUT != 0 );

	
  /* Send START condition */
  I2C_GenerateSTART(MPU6050_I2C, ENABLE);
	if(I2C_TIME_OUT == 0)
	{
		Stop_Communication();
	}
	
	
  /* Test on EV5 and clear it */
	I2C_TIME_OUT = TIME_OUT;
  while((!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT)) && (I2C_TIME_OUT != 0))
  {
		I2C_TIME_OUT--;
	}
	
	if(I2C_TIME_OUT == 0)
	{
		Stop_Communication();
	}
	
	/* Send MPU6050 address for write */
	
	
  I2C_Send7bitAddress(MPU6050_I2C, slaveAddr, I2C_Direction_Transmitter); 

  /* Test on EV6 and clear it */
  I2C_TIME_OUT = TIME_OUT;
  while((!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) && (I2C_TIME_OUT != 0))
	{
		I2C_TIME_OUT--;
	}
	
	
	if(I2C_TIME_OUT == 0)
	{
		Stop_Communication();
	}
	
  /* Clear EV6 by setting again the PE bit */
  I2C_Cmd(MPU6050_I2C, ENABLE);

  /* Send the MPU6050's internal address to write to */
  I2C_SendData(MPU6050_I2C, readAddr);
	
  I2C_TIME_OUT = TIME_OUT;
  /* Test on EV8 and clear it */
  while((!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))&& (I2C_TIME_OUT != 0))
	{
		I2C_TIME_OUT--;
	}

	
	if(I2C_TIME_OUT == 0)
	{
		Stop_Communication();
	}
	
  /* Send STRAT condition a second time */
  I2C_GenerateSTART(MPU6050_I2C, ENABLE);
	
  /* Test on EV5 and clear it */
	
	
	I2C_TIME_OUT = TIME_OUT;
  while((!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT)) && (I2C_TIME_OUT != 0))
	{
		I2C_TIME_OUT--;
	}
		
		
	if(I2C_TIME_OUT == 0)
	{
		Stop_Communication();
	}
	

  /* Send MPU6050 address for read */
  I2C_Send7bitAddress(MPU6050_I2C, slaveAddr, I2C_Direction_Receiver);
  I2C_TIME_OUT = TIME_OUT;
  /* Test on EV6 and clear it */
  while((!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) && (I2C_TIME_OUT != 0))
	{
		I2C_TIME_OUT--;
	}
	
	
	if(I2C_TIME_OUT == 0)
	{
		Stop_Communication();
		NumByteToRead = 1;
	}
	
  /* While there is data to be read */
  while(NumByteToRead)
  {
    if(NumByteToRead == 1)
    {
      /* Disable Acknowledgement */
      I2C_AcknowledgeConfig(MPU6050_I2C, DISABLE);

      /* Send STOP Condition */
      I2C_GenerateSTOP(MPU6050_I2C, ENABLE);
    }
    /* Test on EV7 and clear it */
    if(I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
    {
      /* Read a byte from the MPU6050 */
      *pBuffer = I2C_ReceiveData(MPU6050_I2C);

      /* Point to the next location where the byte read will be saved */
      pBuffer++;

      /* Decrement the read bytes counter */
      NumByteToRead--;
    }
  }

  /* Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(MPU6050_I2C, ENABLE);
//  EXT_CRT_SECTION();

}

uint8_t Stop_Communication()
{
      I2C_GenerateSTOP(MPU6050_I2C, ENABLE);
		//	Stop = 1;
		 I2C_ClearITPendingBit(I2C1,I2C_IT_TIMEOUT);
		 I2C_ClearITPendingBit(I2C1,I2C_IT_PECERR);
	    I2C_ClearITPendingBit(I2C1,I2C_IT_OVR);
			I2C_ClearITPendingBit(I2C1,I2C_IT_AF);
	  I2C_ClearITPendingBit(I2C1,I2C_IT_ARLO);
	  I2C_ClearITPendingBit(I2C1,I2C_IT_BERR);
		
	  I2C_ClearFlag(I2C1,I2C_FLAG_SMBALERT);
	
	     I2C_ClearFlag(I2C1,I2C_FLAG_TIMEOUT);
	
									I2C_ClearFlag(I2C1,I2C_FLAG_PECERR);
									I2C_ClearFlag(I2C1,I2C_FLAG_OVR);
									I2C_ClearFlag(I2C1,I2C_FLAG_AF);
	
									I2C_ClearFlag(I2C1,I2C_FLAG_ARLO);
									I2C_ClearFlag(I2C1,I2C_FLAG_BERR);
	      I2C_AcknowledgeConfig(MPU6050_I2C, DISABLE);
      I2C_GenerateSTOP(MPU6050_I2C, ENABLE);
	
	
	    MPU6050_Initialize();
	return 1;
}












void MPU6050SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[4],i;
   uint8_t selfTest[6],ACCEL = 0xF0,GYRO = 0xE0;
   float factoryTrim[6];
   
   // Configure the accelerometer for self-test
   MPU6050_I2C_ByteWrite(MPU6050_DEFAULT_ADDRESS, &ACCEL, MPU6050_RA_ACCEL_CONFIG); // Enable self test on all three axes and set accelerometer range to +/- 8 g
   MPU6050_I2C_ByteWrite(MPU6050_DEFAULT_ADDRESS, &GYRO,  MPU6050_RA_GYRO_CONFIG); // Enable self test on all three axes and set accelerometer range to +/- 8 g
   
	
	 Delay(250);  // Delay a while to let the device execute the self-test
   
	 // Read Selt Test Results X-axis, Y-axis, Z-axis, A-axis.
	 MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS,rawData, SELF_TEST_X,4); // X-axis self-test results
  // rawData[1] = readByte(MPU6050_ADDRESS, SELF_TEST_Y); // Y-axis self-test results
  // rawData[2] = readByte(MPU6050_ADDRESS, SELF_TEST_Z); // Z-axis self-test results
  // rawData[3] = readByte(MPU6050_ADDRESS, SELF_TEST_A); // Mixed-axis self-test results
   
	
	// Extract the acceleration test results first
   selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
   selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 4 ; // YA_TEST result is a five-bit unsigned integer
   selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) >> 4 ; // ZA_TEST result is a five-bit unsigned integer
  

	// Extract the gyration test results first
   selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
   selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
   selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer   
  

// Process results to allow final comparison with factory set values
   factoryTrim[0] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[0] - 1.0)/30.0))); // FT[Xa] factory trim calculation
   factoryTrim[1] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[1] - 1.0)/30.0))); // FT[Ya] factory trim calculation
   factoryTrim[2] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[2] - 1.0)/30.0))); // FT[Za] factory trim calculation
   factoryTrim[3] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[3] - 1.0) ));             // FT[Xg] factory trim calculation
   factoryTrim[4] =  (-25.0*131.0)*(pow( 1.046 , ((float)selfTest[4] - 1.0) ));             // FT[Yg] factory trim calculation
   factoryTrim[5] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[5] - 1.0) ));             // FT[Zg] factory trim calculation
   
 
 
   for (i = 0; i < 6; i++) {
     destination[i] = 100.0 + 100.0*((float)selfTest[i] - factoryTrim[i])/factoryTrim[i]; // Report percent differences
   }
   
}


void calibrateMPU6050(float * dest1, float * dest2)
{  
  uint8_t data[12], PWR_MGMT_2_W = 0x00,PWR_MGMT_1_W = 0x80,Data_W = 0x00; // data array to hold accelerometer and gyro x, y, z data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g
  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
	
  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
	
// reset device, reset all registers, clear gyro and accelerometer bias registers
  writeByte(MPU6050_ADDRESS,&PWR_MGMT_1_W, MPU6050_RA_PWR_MGMT_1); // Write a one to bit 7 reset bit; toggle reset device
  Delay(100);  
   
// get stable time source
// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	PWR_MGMT_1_W = 0x01; 
  writeByte(MPU6050_ADDRESS, &PWR_MGMT_1_W, MPU6050_RA_PWR_MGMT_1);  
	
  writeByte(MPU6050_ADDRESS, &PWR_MGMT_2_W, MPU6050_RA_PWR_MGMT_2); 
  Delay(200);
  
// Configure device for bias calculation
  writeByte(MPU6050_ADDRESS, &Data_W,INT_ENABLE);   // Disable all interrupts
  writeByte(MPU6050_ADDRESS, &Data_W,FIFO_EN);      // Disable FIFO
  writeByte(MPU6050_ADDRESS, &Data_W,PWR_MGMT_1);   // Turn on internal clock source
  writeByte(MPU6050_ADDRESS, &Data_W,I2C_MST_CTRL); // Disable I2C master
  writeByte(MPU6050_ADDRESS, &Data_W,USER_CTRL);    // Disable FIFO and I2C master modes
  writeByte(MPU6050_ADDRESS, &Data_W,USER_CTRL);    // Reset FIFO and DMP
  Delay(15);
  
// Configure MPU6050 gyro and accelerometer for bias calculation
  Data_W = 0x01;
  writeByte(MPU6050_ADDRESS, &Data_W,CONFIG);      // Set low-pass filter to 188 Hz
	Data_W = 0x00;
  writeByte(MPU6050_ADDRESS, &Data_W,SMPLRT_DIV);  // Set sample rate to 1 kHz
  writeByte(MPU6050_ADDRESS, &Data_W,GYRO_CONFIG);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(MPU6050_ADDRESS, &Data_W,ACCEL_CONFIG); // Set accelerometer full-scale to 2 g, maximum sensitivity
 


// Configure FIFO to capture accelerometer and gyro data for bias calculation

  Data_W = 0x40;
  writeByte(MPU6050_ADDRESS,&Data_W, USER_CTRL);   // Enable FIFO  
	
	Data_W = 0x78;
  writeByte(MPU6050_ADDRESS, &Data_W,FIFO_EN);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
  Delay(80); // accumulate 80 samples in 80 milliseconds = 960 bytes

// At end of sample accumulation, turn off FIFO sensor read
  Data_W = 0x00;
  writeByte(MPU6050_ADDRESS,&Data_W, FIFO_EN);        // Disable gyro and accelerometer sensors for FIFO
  
	MPU6050_I2C_BufferRead(MPU6050_ADDRESS,data,FIFO_COUNTH,2);
//	readBytes(MPU6050_ADDRESS, FIFO_COUNTH, 2, &data[0]); 
	
	// read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    MPU6050_I2C_BufferRead(MPU6050_ADDRESS,data,FIFO_R_W,12);
		
		//readBytes(MPU6050_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
    
    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
            
}
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
    
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}
 
// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

// Push gyro biases to hardware registers
  writeByte(MPU6050_ADDRESS , &data[0], XG_OFFS_USRH);// might not be supported in MPU6050
  writeByte(MPU6050_ADDRESS , &data[1], XG_OFFS_USRL);
  writeByte(MPU6050_ADDRESS , &data[2], YG_OFFS_USRH);
  writeByte(MPU6050_ADDRESS , &data[3], YG_OFFS_USRL);
  writeByte(MPU6050_ADDRESS , &data[4], ZG_OFFS_USRH);
  writeByte(MPU6050_ADDRESS , &data[5], ZG_OFFS_USRL);

  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.



  
	MPU6050_I2C_BufferRead(MPU6050_ADDRESS, &data[0], XA_OFFSET_H, 2); // Read factory accelerometer trim values
	
  accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  
	MPU6050_I2C_BufferRead(MPU6050_ADDRESS, &data[0], YA_OFFSET_H, 2);
  
	accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  
	MPU6050_I2C_BufferRead(MPU6050_ADDRESS, &data[0], ZA_OFFSET_H, 2);
  
	accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  
  
  for(ii = 0; ii < 3; ii++) {
    if(accel_bias_reg[ii] & mask) 
			mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);
 
  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

  // Push accelerometer biases to hardware registers
  writeByte(MPU6050_ADDRESS, &data[0], XA_OFFSET_H); // might not be supported in MPU6050
  writeByte(MPU6050_ADDRESS, &data[1], XA_OFFSET_L_TC);
  writeByte(MPU6050_ADDRESS, &data[2], YA_OFFSET_H);
  writeByte(MPU6050_ADDRESS, &data[3], YA_OFFSET_L_TC);  
  writeByte(MPU6050_ADDRESS, &data[4], ZA_OFFSET_H);
  writeByte(MPU6050_ADDRESS, &data[5], ZA_OFFSET_L_TC);

// Output scaled accelerometer biases for manual subtraction in the main program
   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}





