/* ===========================================================================================
  * �ļ�����:  sht3x.c
  * ����ʱ��:  2017-7-3
  * ��������:  ��ʪ�ȴ���(�Ż������ṩ���룬���õ���ֲ�ԣ���ܾ���)
  * ��           ��:  ����
  * ��           ˾:  �������ܿƼ����޹�˾
  * ���ļ�¼:  
  * ʹ��˵��:  ��ȡ�¶�ʱ���ú���SHT3X_GetTempAndHumi() 
  *                         ��ʼ��ʱҪ���ú���SHT3X_Init()
  *                         ��ֲʱ����Ҫ�޸Ĵ��ļ�����Ӧͷ�ļ���
  *                         ��Ҫ��SHT_3X_I2C.c ���ʹ��
  * ========================================================================================== */
  
#define SHT3X_TYPE_CLCYLE
#include "SHT_3X_I2C.h"
#include "sht3x.h"
#include "delay.h"
unsigned char _i2cWriteHeader;
unsigned char _i2cReadHeader;
/* ====================================================================================
 * ��������:  SHT3X_SetI2cAdr
 * ����˵��:  ѡ���ַ
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
void SHT3X_SetI2cAdr(unsigned char i2cAdr)
{
	_i2cWriteHeader = i2cAdr << 1;
	_i2cReadHeader = _i2cWriteHeader | 0x01;
}
/* ====================================================================================
 * ��������:  SHT3X_Init
 * ����˵��:  ��ʼ��
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
void SHT3X_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	//ʹ��GPIOBʱ��
	   
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_10|GPIO_Pin_11); 	//PB6,PB7 �����
	
	f_Sht3xI2cInit();
	SHT3X_SetI2cAdr(SHT_3X_ADD);
}

/* ====================================================================================
 * ��������:  SHT3X_WriteCommand
 * ����˵��:  дָ��
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
unsigned char SHT3X_Read2BytesAndCrc(unsigned short *data, unsigned char finaleAckNack, unsigned char timeout)
{
	unsigned char     errorbuf;    // error code
	unsigned char     bytes[2]; // read data array
	unsigned char     checksum; // checksum byte

	// read two data bytes and one checksum byte
	errorbuf = SHT_3X_I2c_ReadByte(&bytes[0], ACK, timeout);
	if(errorbuf == NO_ERROR) 
		errorbuf = SHT_3X_I2c_ReadByte(&bytes[1], ACK, 0);
	if(errorbuf == NO_ERROR) 
		errorbuf = SHT_3X_I2c_ReadByte(&checksum, finaleAckNack, 0);

	// verify checksum
	if(errorbuf == NO_ERROR) 
		errorbuf = SHT3X_CheckCrc(bytes, 2, checksum);

	// combine the two bytes to a 16-bit value
	*data = (bytes[0] << 8) | bytes[1];

	return  errorbuf;
}

/* ====================================================================================
 * ��������:  SHT3X_Write2BytesAndCrc
 * ����˵��:  дָ��
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
unsigned char SHT3X_Write2BytesAndCrc(unsigned short data)
{
	unsigned char     errorbuf;    // error code
	unsigned char     bytes[2]; // read data array
	unsigned char     checksum; // checksum byte
	
	bytes[0] = data >> 8;
	bytes[1] = data & 0xFF;
	checksum = SHT3X_CalcCrc(bytes, 2);
 
	// write two data bytes and one checksum byte
	errorbuf = SHT_3X_I2c_WriteByte(bytes[0]); // write data MSB
	if(errorbuf == NO_ERROR) 
		errorbuf = SHT_3X_I2c_WriteByte(bytes[1]); // write data LSB
	if(errorbuf == NO_ERROR) 
		errorbuf = SHT_3X_I2c_WriteByte(checksum); // write checksum
  
	return errorbuf ;
}

/* ====================================================================================
 * ��������:  SHT3x_ReadSerialNumber
 * ����˵��:  
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
unsigned char SHT3x_ReadSerialNumber(unsigned int *serialNbr)
{
	unsigned char errorbuf;                         // error code
	unsigned short serialNumWords[2];

	errorbuf = SHT3X_StartWriteAccess();

	// write "read serial number" command
	errorbuf |= SHT3X_WriteCommand(CMD_READ_SERIALNBR);
	// if no error, start read access
	if(errorbuf == NO_ERROR) 
		errorbuf = SHT3X_StartReadAccess();
	// if no error, read first serial number word
	if(errorbuf == NO_ERROR) 
		errorbuf = SHT3X_Read2BytesAndCrc(&serialNumWords[0], ACK, 100);
	// if no error, read second serial number word
	if(errorbuf == NO_ERROR) 
		errorbuf = SHT3X_Read2BytesAndCrc(&serialNumWords[1], NACK, 0);

	SHT3X_StopAccess();

	// if no error, calc serial number as 32-bit integer
	if(errorbuf == NO_ERROR)
	{
		*serialNbr = (serialNumWords[0] << 16) | serialNumWords[1];
	}
	return errorbuf;
}
/* ====================================================================================
 * ��������:  SHT3X_ReadStatus
 * ����˵��:  
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
unsigned char SHT3X_ReadStatus(unsigned short *status)
{
	unsigned char errorbuf;                                       // error code
	errorbuf = SHT3X_StartWriteAccess();

	// if no error, write "read status" command
	if(errorbuf == NO_ERROR) 
		errorbuf = SHT3X_WriteCommand(CMD_READ_STATUS);
	// if no error, start read access
	if(errorbuf == NO_ERROR) 
		errorbuf = SHT3X_StartReadAccess(); 
	// if no error, read status
	if(errorbuf == NO_ERROR) 
		errorbuf = SHT3X_Read2BytesAndCrc(status, NACK, 0);

	SHT3X_StopAccess();

	return errorbuf;
}
/* ====================================================================================
 * ��������:  SHT3X_ClearAllAlertFlags
 * ����˵��:  
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
unsigned char SHT3X_ClearAllAlertFlags(void)
{
	unsigned char errorbuf;                           // error code

	errorbuf = SHT3X_StartWriteAccess();

	// if no error, write clear status register command
	if(errorbuf == NO_ERROR) 
		errorbuf = SHT3X_WriteCommand(CMD_CLEAR_STATUS);

	SHT3X_StopAccess();

	return errorbuf;
}

/* ====================================================================================
 * ��������:  SHT3X_ReadMeasurementBuffer
 * ����˵��:  
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
unsigned char SHT3X_ReadMeasurementBuffer(float *temp, float *humi)
{
	unsigned char      errorbuf;        // error code
	unsigned short     rawValueTemp; // temperature raw value from sensor
	unsigned short     rawValueHumi; // humidity raw value from sensor

	errorbuf = SHT3X_StartWriteAccess();

	// if no error, read measurements
	if(errorbuf == NO_ERROR)	
		errorbuf = SHT3X_WriteCommand(CMD_FETCH_DATA);
	if(errorbuf == NO_ERROR)	
		errorbuf = SHT3X_StartReadAccess();	
	if(errorbuf == NO_ERROR)	
		errorbuf = SHT3X_Read2BytesAndCrc(&rawValueTemp, ACK, 0);
	if(errorbuf == NO_ERROR) 
		errorbuf = SHT3X_Read2BytesAndCrc(&rawValueHumi, NACK, 0);
	
	// if no error, calculate temperature in �C and humidity in %RH
	if(errorbuf == NO_ERROR)
	{
		*temp = SHT3X_CalcTemperature(rawValueTemp);
		*humi = SHT3X_CalcHumidity(rawValueHumi);
	}
	
	SHT3X_StopAccess();
	
	return   errorbuf;
}

/* ====================================================================================
 * ��������:  SHT3X_EnableHeater
 * ����˵��:  
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
unsigned char SHT3X_EnableHeater(void)
{
	unsigned char errorbuf; // error code
	errorbuf = SHT3X_StartWriteAccess();
	// if no error, write heater enable command
	if(errorbuf == NO_ERROR) 
		errorbuf = SHT3X_WriteCommand(CMD_HEATER_ENABLE);
	SHT3X_StopAccess();
	return errorbuf;
}
/* ====================================================================================
 * ��������:  SHT3X_DisbaleHeater
 * ����˵��:  
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
unsigned char SHT3X_DisbaleHeater(void)
{
	unsigned char errorbuf;                          // error code
	errorbuf = SHT3X_StartWriteAccess();
	// if no error, write heater disable command
	if(errorbuf == NO_ERROR) 
		errorbuf = SHT3X_WriteCommand(CMD_HEATER_DISABLE);
	SHT3X_StopAccess();
	return errorbuf;
}

/* ====================================================================================
 * ��������:  SHT3X_SofloatReset
 * ����˵��:  
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
unsigned char SHT3X_SofloatReset(void)
{
	unsigned char errorbuf; // error code
	errorbuf = SHT3X_StartWriteAccess();
	// write reset command
	errorbuf |= SHT3X_WriteCommand(CMD_SOFT_RESET);
	SHT3X_StopAccess();
	// if no error, wait 50 ms afloater reset
	if(errorbuf == NO_ERROR) 
		delay_us(50000);
	return errorbuf;
}
/* ====================================================================================
 * ��������:  SHT3X_StartWriteAccess
 * ����˵��:  
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
unsigned char SHT3X_StartWriteAccess(void)
{
	unsigned char errorbuf; // error code
	// write a start condition
	SHT_3X_I2c_StartCondition();
	// write the sensor I2C address with the write flag
	errorbuf = SHT_3X_I2c_WriteByte(_i2cWriteHeader);
	return errorbuf;
}
/* ====================================================================================
 * ��������:  SHT3X_StartReadAccess
 * ����˵��:  
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
unsigned char SHT3X_StartReadAccess(void)
{
	unsigned char errorbuf;               // error code

	// write a start condition
	SHT_3X_I2c_StartCondition();

	// write the sensor I2C address with the read flag
	errorbuf = SHT_3X_I2c_WriteByte(_i2cReadHeader);

	return errorbuf;
}
/* ====================================================================================
 * ��������:  SHT3X_StopAccess
 * ����˵��:  
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
void SHT3X_StopAccess(void)
{
	SHT_3X_I2c_StopCondition();
}

/* ====================================================================================
 * ��������:  SHT3X_WriteCommand
 * ����˵��:  дָ��
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
unsigned char SHT3X_WriteCommand(unsigned short cmd)
{
	unsigned char errorbuf; // error code
	// write the upper 8 bits of the command to the sensor
	errorbuf  = SHT_3X_I2c_WriteByte(cmd >> 8);
	// write the lower 8 bits of the command to the sensor
	errorbuf |= SHT_3X_I2c_WriteByte(cmd & 0xFF);
	return  errorbuf;
}

/* ====================================================================================
 * ��������:  SHT3X_CalcCrc
 * ����˵��:  
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
unsigned char SHT3X_CalcCrc(unsigned char data[], unsigned char nbrOfBytes)
{
	unsigned char bit;        // bit mask
	unsigned char crc = 0xFF; // calculated checksum
	unsigned char byteCtr;    // byte counter

	// calculates 8-Bit checksum with given polynomial
	for(byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++)
	{
		crc ^= (data[byteCtr]);
		for(bit = 8; bit > 0; --bit)
		{
			if(crc & 0x80) 
				crc = (crc << 1) ^ POLYNOMIAL;
			else 
				crc = (crc << 1);
		}
	}
	return crc;
}
/* ====================================================================================
 * ��������:  SHT3X_CheckCrc
 * ����˵��:  
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
unsigned char SHT3X_CheckCrc(unsigned char data[], unsigned char nbrOfBytes, unsigned char checksum)
{
	unsigned char crc;     // calculated checksum

	// calculates 8-Bit checksum
	crc = SHT3X_CalcCrc(data, nbrOfBytes);

	// verify checksum
	if(crc != checksum) 
		return CHECKSUM_ERROR;
	else 
		return NO_ERROR;
}
/* ====================================================================================
 * ��������:  SHT3X_CalcTemperature
 * ����˵��:  
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
float SHT3X_CalcTemperature(unsigned short rawValue)
{
	// calculate temperature [�C]
	// T = -45 + 175 * rawValue / (2^16-1)
	return 175 * (float)rawValue / 65535 - 45;
}
/* ====================================================================================
 * ��������:  SHT3X_CalcTemperature
 * ����˵��:  
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
float SHT3X_CalcHumidity(unsigned short rawValue)
{
	// calculate relative humidity [%RH]
	// RH = rawValue / (2^16-1) * 100
	return 100 * (float)rawValue / 65535;
}

/* ====================================================================================
 * ��������:  SHT3X_CalcRawTemperature
 * ����˵��:  
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
unsigned short SHT3X_CalcRawTemperature(float temperature)
{
	// calc raw value from a temperature [�C]
	// rawValue = (T + 45) / 175 * (2^16-1)
	return (unsigned short)((temperature + 45) / 175 * 65535);
}

/* ====================================================================================
 * ��������:  SHT3X_CalcRawHumidity
 * ����˵��:  
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
unsigned short SHT3X_CalcRawHumidity(float humidity)
{
	// calc raw value from a relative humidity [%RH]
	// rawValue = RH / 100 * (2^16-1)
	return (unsigned short)(humidity / 100 * 65535);
}
/* ====================================================================================
 * ��������:  SHT3X_CalcRawHumidity
 * ����˵��:  
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
//BOOL SHT3X_ReadAlert(void)
//{
//	// read alert pin
//	return (ALERT_READ != 0) ? TRUE : FALSE;
//}
/* ====================================================================================
 * ��������:  SHT3X_GetTempAndHumi
 * ����˵��:  ��ȡ��ʪ��
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
unsigned char SHT3X_GetTempAndHumi(float *temp,float *humi,unsigned char repeatab,unsigned char mode,unsigned char timeout)
{
	unsigned char errorbuf;
	
	switch(mode)
	{    
		case MODE_CLKSTRETCH:          // get temperature with clock stretching mode
			errorbuf = SHT3X_GetTempAndHumiClkStretch(temp, humi, repeatab, timeout); 
			break;
		case MODE_POLLING:                // get temperature with polling mode
			errorbuf = SHT3X_GetTempAndHumiPolling(temp, humi, repeatab, timeout); 
			break;
		default:              
			errorbuf = PARM_ERROR;
			break;
	}
	return errorbuf;
}
/* ====================================================================================
 * ��������:  SHT3X_GetTempAndHumiClkStretch
 * ����˵��:  
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
 unsigned char SHT3X_GetTempAndHumiClkStretch(float *temp,float *humi,unsigned char repeatab,unsigned char timeout)
{
	unsigned char     errorbuf;        // error code
	unsigned short    rawValueTemp; // temperature raw value from sensor
	unsigned short    rawValueHumi; // humidity raw value from sensor

	errorbuf = SHT3X_StartWriteAccess();

	if(errorbuf == NO_ERROR)
	{
		// start measurement in clock stretching mode
		// use depending on the required repeatability, the corresponding command
		switch(repeatab)
		{
			case REPEATAB_LOW:    
				errorbuf = SHT3X_WriteCommand(CMD_MEAS_CLOCKSTR_L); 
				break;
			case REPEATAB_MEDIUM: 
				errorbuf = SHT3X_WriteCommand(CMD_MEAS_CLOCKSTR_M); 
				break;
			case REPEATAB_HIGH:  
				errorbuf = SHT3X_WriteCommand(CMD_MEAS_CLOCKSTR_H); 
				break;
			default:              
				errorbuf = PARM_ERROR; 
				break;
		}
	}

	// if no error, start read access
	if(errorbuf == NO_ERROR) 
		errorbuf = SHT3X_StartReadAccess();
	// if no error, read temperature raw values
	if(errorbuf == NO_ERROR) 
		errorbuf = SHT3X_Read2BytesAndCrc(&rawValueTemp, ACK, timeout);
	// if no error, read humidity raw values
	if(errorbuf == NO_ERROR) 
		errorbuf = SHT3X_Read2BytesAndCrc(&rawValueHumi, NACK, 0);

	SHT3X_StopAccess();

	// if no error, calculate temperature in �C and humidity in %RH
	if(errorbuf == NO_ERROR)
	{
		 *temp = SHT3X_CalcTemperature(rawValueTemp);
		 *humi = SHT3X_CalcHumidity(rawValueHumi);
	}

	return errorbuf;
}
/* ====================================================================================
 * ��������:  SHT3X_GetTempAndHumiPolling
 * ����˵��:  
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
unsigned char SHT3X_GetTempAndHumiPolling(float *temp,float *humi,unsigned char repeatab,unsigned char timeout)
{
	unsigned char     errorbuf;           // error code
	unsigned short    rawValueTemp;    // temperature raw value from sensor
	unsigned short    rawValueHumi;    // humidity raw value from sensor

	errorbuf  = SHT3X_StartWriteAccess();

	// if no error ...
	if(errorbuf == NO_ERROR)
	{
		// start measurement in polling mode
		// use depending on the required repeatability, the corresponding command
		switch(repeatab)
		{
			case REPEATAB_LOW:    
				errorbuf = SHT3X_WriteCommand(CMD_MEAS_POLLING_L); 
				break;
			case REPEATAB_MEDIUM: 
				errorbuf = SHT3X_WriteCommand(CMD_MEAS_POLLING_M); 
				break;
			case REPEATAB_HIGH:		
				errorbuf = SHT3X_WriteCommand(CMD_MEAS_POLLING_H); 
				break;
			default:         			
				errorbuf = PARM_ERROR; 
				break;
		}
	}

	// if no error, wait until measurement ready
	if(errorbuf == NO_ERROR)
	{
		// poll every 1ms for measurement ready until timeout
		while(timeout--)
		{
			// check if the measurement has finished
			errorbuf = SHT3X_StartReadAccess();
			// if measurement has finished -> exit loop
			if(errorbuf == NO_ERROR) 
				break;
			// delay 1ms
			delay_us(1000);
		}
			
		// check for timeout error
		if(timeout == 0) 
			errorbuf = TIMEOUT_ERROR;
	}
		
		// if no error, read temperature and humidity raw values
	if(errorbuf == NO_ERROR)
	{
		errorbuf |= SHT3X_Read2BytesAndCrc(&rawValueTemp, ACK, 0);
		errorbuf |= SHT3X_Read2BytesAndCrc(&rawValueHumi, NACK, 0);
	}

	SHT3X_StopAccess();

	// if no error, calculate temperature in �C and humidity in %RH
	if(errorbuf == NO_ERROR)
	{
		*temp = SHT3X_CalcTemperature(rawValueTemp);
		*humi = SHT3X_CalcHumidity(rawValueHumi);
	}

	return errorbuf;
}

/* ====================================================================================
 * ��������:  SHT3X_StartPeriodicMeasurment
 * ����˵��:  
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
unsigned char SHT3X_StartPeriodicMeasurment(unsigned char repeatab,unsigned char freq)
{
	unsigned char errorbuf;                               // error code
	errorbuf = SHT3X_StartWriteAccess();

	// if no error, start periodic measurement 
	if(errorbuf == NO_ERROR)
	{
		// use depending on the required repeatability and frequency,
		// the corresponding command
		switch(repeatab)
		{
			case REPEATAB_LOW: // low repeatability
				switch(freq)
				{
					case FREQUENCY_HZ5:  // low repeatability,  0.5 Hz
						errorbuf |= SHT3X_WriteCommand(CMD_MEAS_PERI_05_L); 
						break;					
					case FREQUENCY_1HZ:  // low repeatability,  1.0 Hz
						errorbuf |= SHT3X_WriteCommand(CMD_MEAS_PERI_1_L); 
						break;					
					case FREQUENCY_2HZ:  // low repeatability,  2.0 Hz
						errorbuf |= SHT3X_WriteCommand(CMD_MEAS_PERI_2_L); 
						break;					
					case FREQUENCY_4HZ:  // low repeatability,  4.0 Hz
						errorbuf |= SHT3X_WriteCommand(CMD_MEAS_PERI_4_L);
						break;					
					case FREQUENCY_10HZ: // low repeatability, 10.0 Hz
						errorbuf |= SHT3X_WriteCommand(CMD_MEAS_PERI_10_L); 
						break;					
					default:
						errorbuf |= PARM_ERROR; break;
				}
			  break;
				
			case REPEATAB_MEDIUM: // medium repeatability
				switch(freq)
				{
					case FREQUENCY_HZ5:  // medium repeatability,  0.5 Hz
						errorbuf |= SHT3X_WriteCommand(CMD_MEAS_PERI_05_M); 
						break;
					case FREQUENCY_1HZ:  // medium repeatability,  1.0 Hz
						errorbuf |= SHT3X_WriteCommand(CMD_MEAS_PERI_1_M); 
						break;				
					case FREQUENCY_2HZ:  // medium repeatability,  2.0 Hz
						errorbuf |= SHT3X_WriteCommand(CMD_MEAS_PERI_2_M); 
						break;				
					case FREQUENCY_4HZ:  // medium repeatability,  4.0 Hz
						errorbuf |= SHT3X_WriteCommand(CMD_MEAS_PERI_4_M); 
						break;			
					case FREQUENCY_10HZ: // medium repeatability, 10.0 Hz
						errorbuf |= SHT3X_WriteCommand(CMD_MEAS_PERI_10_M); 
						break;
					default:
						errorbuf |= PARM_ERROR; break;
				}
			  break;
				
			case REPEATAB_HIGH: // high repeatability
				switch(freq)
				{
					case FREQUENCY_HZ5:  // high repeatability,  0.5 Hz
						errorbuf |= SHT3X_WriteCommand(CMD_MEAS_PERI_05_H); 
						break;
					case FREQUENCY_1HZ:  // high repeatability,  1.0 Hz
						errorbuf |= SHT3X_WriteCommand(CMD_MEAS_PERI_1_H); 
						break;
					case FREQUENCY_2HZ:  // high repeatability,  2.0 Hz
						errorbuf |= SHT3X_WriteCommand(CMD_MEAS_PERI_2_H); 
						break;
					case FREQUENCY_4HZ:  // high repeatability,  4.0 Hz
						errorbuf |= SHT3X_WriteCommand(CMD_MEAS_PERI_4_H); 
						break;
					case FREQUENCY_10HZ: // high repeatability, 10.0 Hz
						errorbuf |= SHT3X_WriteCommand(CMD_MEAS_PERI_10_H); 
						break;
					default:
						errorbuf |= PARM_ERROR; break;
				}
			  break;
			default:
				errorbuf |= PARM_ERROR; 
				break;
		}
	}
	SHT3X_StopAccess();
	return errorbuf;
}

