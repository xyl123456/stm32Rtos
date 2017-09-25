/* ===========================================================================================
  * �ļ�����:  SHT_3X_I2C.c
  * ����ʱ��:  2017-7-3
  * ��������:  ��ʪ��i2c �ײ������ļ�(�Ż������ṩ���룬���õ���ֲ�ԣ���ܾ���)
  * ��           ��:  ����
  * ��           ˾:  �������ܿƼ����޹�˾
  * ���ļ�¼:  
  * ��           ע:  ��ֲ��������Ҫ����MCU �޸ĵ�������
  *                          c �ļ�:
  *                            1) SHT_3X_I2C_SDA_OUT
  *                            2) SHT_3X_I2C_SDA_IN
  *                            3) SHT_3X_I2C_SCL_OUT
  *                            4) SHT_3X_I2C_SCL_IN
  *                          h �ļ�:
  *                            1) #define  SHT_3X_SDA_L
  *                            2) #define  SHT_3X_SDA_H
  *                            3) #define  SHT_3X_SCL_L
  *                            4) #define  SHT_3X_SCL_H
  *                            5) #define  SHT_3X_SDA_IN
  *                            6) #define  SHT_3X_SCL_IN
  *                          ��ֲ�����������������ɸ���Ӳ����·��MCU ��Ƶ�޸Ĳ���
  *                                   #define  SHT_3X_DELAY_COUNT 
  * ========================================================================================== */
#define  SHT_3X_I2C_TYPE_CLYCLE
#include "SHT_3X_I2C.h"
#include "delay.h"
#include "sys.h"
#include "sht3x.h"
/* ====================================================================================
 * ��������:  I2C_SDA_OUT
 * ����˵��:  ����SDA Ϊ�����
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
void SHT_3X_I2C_SDA_OUT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
/* ====================================================================================
 * ��������:  I2C_SDA_IN
 * ����˵��:  ����SDA Ϊ�����
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
void SHT_3X_I2C_SDA_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	//GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
/* ====================================================================================
 * ��������:  I2C_SCL_OUT
 * ����˵��:  ����SCL Ϊ�����
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
void SHT_3X_I2C_SCL_OUT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
/* ====================================================================================
 * ��������:  I2C_SCL_IN
 * ����˵��:  ����SCL Ϊ�����
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
void SHT_3X_I2C_SCL_IN(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	//GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/* ====================================================================================
 * ��������:  SDA_HIHG
 * ����˵��:  SDA �����
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
void SHT_3X_SDA_HIHG(void)
{
	SHT_3X_SDA_H;
}
/* ====================================================================================
 * ��������:  SDA_LOW
 * ����˵��:  SDA �����
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
void SHT_3X_SDA_LOW(void)
{
	SHT_3X_SDA_L;
}
/* ====================================================================================
 * ��������:  SCL_HIGH
 * ����˵��:  SCL �����
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
void SHT_3X_SCL_HIGH(void)
{
	SHT_3X_SCL_H;
}
/* ====================================================================================
 * ��������:  SCL_LOW
 * ����˵��:  SCL �����
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
void SHT_3X_SCL_LOW(void)
{
	SHT_3X_SCL_L;
}
/* ====================================================================================
 * ��������:  SDA_READ
 * ����˵��:  SDA �����ƽ
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
unsigned char SHT_3X_SDA_READ(void)
{
	return (SHT_3X_SDA_IN);
}
/* ====================================================================================
 * ��������:  SCL_READ
 * ����˵��:  SCL �����ƽ
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
unsigned char SHT_3X_SCL_READ(void)
{
	return (SHT_3X_SCL_IN);
}
/* ====================================================================================
 * ��������:  f_Sht3xI2cInit
 * ����˵��:  ��ʼ��SDA  / SCL
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
void f_Sht3xI2cInit(void)
{
	SHT_3X_I2C_SDA_OUT();
	SHT_3X_I2C_SCL_OUT();
	SHT_3X_SDA_HIHG();
	SHT_3X_SCL_HIGH();
}

/* ====================================================================================
 * ��������:  I2c_StartCondition
 * ����˵��:  ��ʼ�ź�
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
void SHT_3X_I2c_StartCondition(void)
 {
 	SHT_3X_I2C_SDA_OUT();
	SHT_3X_SDA_HIHG();
	SHT_3X_SCL_HIGH();
	delay_us(SHT_3X_DELAY_COUNT);
	SHT_3X_SDA_LOW();
	delay_us(SHT_3X_DELAY_COUNT);
}

/* ====================================================================================
 * ��������:  I2c_StopCondition
 * ����˵��:  ֹͣ�ź�
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
void SHT_3X_I2c_StopCondition(void)
{
	SHT_3X_SCL_HIGH();
	SHT_3X_SDA_LOW();
	delay_us(SHT_3X_DELAY_COUNT);
	SHT_3X_SDA_HIHG();
	delay_us(SHT_3X_DELAY_COUNT);
}
/* ====================================================================================
 * ��������:  SHT_3X_I2c_WriteByte
 * ����˵��:  дһ���ֽ�
 * ����˵��:  txByte -- Ҫд����ֽ�
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
unsigned char SHT_3X_I2c_WriteByte(unsigned char txByte)
{
	unsigned char errorbuf = NO_ERROR;
	unsigned char i;
	SHT_3X_SCL_LOW();
	SHT_3X_I2C_SDA_OUT();
	for(i = 0; i < 8; i ++)
	{
		SHT_3X_SCL_LOW();
		delay_us(SHT_3X_DELAY_COUNT);          // data hold time(t_HD;DAT)
		if(txByte & 0x80)
			SHT_3X_SDA_HIHG();
		else                    
			SHT_3X_SDA_LOW();
		delay_us(SHT_3X_DELAY_COUNT);          // data set-up time (t_SU;DAT)
		SHT_3X_SCL_HIGH();                         // generate clock pulse on SCL
		delay_us(SHT_3X_DELAY_COUNT);          // SCL high time (t_HIGH)
		txByte <<= 1;
	}
	SHT_3X_SCL_LOW();
	
	delay_us(SHT_3X_DELAY_COUNT);  
	SHT_3X_I2C_SDA_IN();
	delay_us(SHT_3X_DELAY_COUNT);  
	SHT_3X_SCL_HIGH();
	
	delay_us(SHT_3X_DELAY_COUNT);                 // data set-up time (t_SU;DAT)
	if(SHT_3X_SDA_READ()) 
		errorbuf = ACK_ERROR;                         // check ack from i2c slave
	SHT_3X_SCL_LOW();
	delay_us(SHT_3X_DELAY_COUNT);                 // wait to see byte package on scope
	return errorbuf;                                          // return error code
}

/* ====================================================================================
 * ��������:  SHT_3X_I2c_ReadByte
 * ����˵��:  ��һ���ֽ�
 * ����˵��:  rxByte -- �����ֽ�
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
unsigned char SHT_3X_I2c_ReadByte(unsigned char *rxByte, unsigned char ack, unsigned char timeout)
{
	unsigned char errorbuf = NO_ERROR;
	unsigned char i,readbuf;
	
	readbuf = 0x00;
	SHT_3X_SCL_LOW();
	SHT_3X_I2C_SDA_IN();                                             // release SDA-line
	for(i = 0; i < 8; i++)                                                   // shift bit for masking (8 times)
	{ 
		readbuf <<= 1;
		SHT_3X_SCL_HIGH();                                          // start clock on SCL-line
		delay_us(SHT_3X_DELAY_COUNT);                           // clock set-up time (t_SU;CLK)
		errorbuf = I2c_WaitWhileClockStreching(timeout);  // wait while clock streching
		delay_us(SHT_3X_DELAY_COUNT);                           // SCL high time (t_HIGH)
		if(SHT_3X_SDA_READ()) 
			readbuf |= 0x01;                                           // read bit
		SHT_3X_SCL_LOW();
		delay_us(SHT_3X_DELAY_COUNT);                            // data hold time(t_HD;DAT)
	}
	*rxByte = readbuf;
	SHT_3X_I2C_SDA_OUT();  
	delay_us(SHT_3X_DELAY_COUNT);
	if(ack == ACK) 
		SHT_3X_SDA_LOW();                                          // send acknowledge if necessary
	else           
		SHT_3X_SDA_HIHG();
	delay_us(SHT_3X_DELAY_COUNT);                                  // data set-up time (t_SU;DAT)
	SHT_3X_SCL_HIGH();                                                 // clk #9 for ack
	delay_us(SHT_3X_DELAY_COUNT);                                  // SCL high time (t_HIGH)
	SHT_3X_SCL_LOW();
	SHT_3X_SDA_HIHG();                                                // release SDA-line
	delay_us(SHT_3X_DELAY_COUNT);                                  // wait to see byte package on scope
		
	return errorbuf;                                                           // return with no error
}
/* ====================================================================================
 * ��������:  I2c_WaitWhileClockStreching
 * ����˵��:  
 * ����˵��:  timeout -- ��ʱʱ��
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
unsigned char I2c_WaitWhileClockStreching(unsigned char timeout)
{
	unsigned char errorbuf = NO_ERROR;
	while(SHT_3X_SCL_READ() == 0)
	{
		if(timeout-- == 0)	
			return TIMEOUT_ERROR;
		delay_us(100);
	}
	return errorbuf;
}

/* ====================================================================================
 * ��������:  I2c_GeneralCallReset
 * ����˵��:  
 * ����˵��:  
 * ���˵��:
 * ��           ��:  ����
 * �༭ʱ��:  2017-7-3
 * ��           ע:  
 ====================================================================================*/
unsigned char I2c_GeneralCallReset(void)
{
	unsigned char errorbuf;
	SHT_3X_I2c_StartCondition();
	errorbuf = SHT_3X_I2c_WriteByte(0x00);
	if(errorbuf == NO_ERROR) 
		errorbuf = SHT_3X_I2c_WriteByte(0x06);
	return errorbuf;
}


