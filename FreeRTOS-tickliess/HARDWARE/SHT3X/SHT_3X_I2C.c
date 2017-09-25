/* ===========================================================================================
  * 文件名称:  SHT_3X_I2C.c
  * 建立时间:  2017-7-3
  * 功能描述:  温湿度i2c 底层驱动文件(优化厂家提供代码，更好的移植性，规避警告)
  * 作           者:  王聪
  * 公           司:  艾佳智能科技有限公司
  * 更改记录:  
  * 备           注:  移植过程中需要根据MCU 修改的配置有
  *                          c 文件:
  *                            1) SHT_3X_I2C_SDA_OUT
  *                            2) SHT_3X_I2C_SDA_IN
  *                            3) SHT_3X_I2C_SCL_OUT
  *                            4) SHT_3X_I2C_SCL_IN
  *                          h 文件:
  *                            1) #define  SHT_3X_SDA_L
  *                            2) #define  SHT_3X_SDA_H
  *                            3) #define  SHT_3X_SCL_L
  *                            4) #define  SHT_3X_SCL_H
  *                            5) #define  SHT_3X_SDA_IN
  *                            6) #define  SHT_3X_SCL_IN
  *                          移植过程中如果遇到问题可根据硬件电路及MCU 主频修改参数
  *                                   #define  SHT_3X_DELAY_COUNT 
  * ========================================================================================== */
#define  SHT_3X_I2C_TYPE_CLYCLE
#include "SHT_3X_I2C.h"
#include "delay.h"
#include "sys.h"
#include "sht3x.h"
/* ====================================================================================
 * 函数名称:  I2C_SDA_OUT
 * 功能说明:  设置SDA 为输出口
 * 输入说明:  
 * 输出说明:
 * 作           者:  王聪
 * 编辑时间:  2017-7-3
 * 备           注:  
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
 * 函数名称:  I2C_SDA_IN
 * 功能说明:  设置SDA 为输入口
 * 输入说明:  
 * 输出说明:
 * 作           者:  王聪
 * 编辑时间:  2017-7-3
 * 备           注:  
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
 * 函数名称:  I2C_SCL_OUT
 * 功能说明:  设置SCL 为输出口
 * 输入说明:  
 * 输出说明:
 * 作           者:  王聪
 * 编辑时间:  2017-7-3
 * 备           注:  
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
 * 函数名称:  I2C_SCL_IN
 * 功能说明:  设置SCL 为输入口
 * 输入说明:  
 * 输出说明:
 * 作           者:  王聪
 * 编辑时间:  2017-7-3
 * 备           注:  
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
 * 函数名称:  SDA_HIHG
 * 功能说明:  SDA 输出高
 * 输入说明:  
 * 输出说明:
 * 作           者:  王聪
 * 编辑时间:  2017-7-3
 * 备           注:  
 ====================================================================================*/
void SHT_3X_SDA_HIHG(void)
{
	SHT_3X_SDA_H;
}
/* ====================================================================================
 * 函数名称:  SDA_LOW
 * 功能说明:  SDA 输出低
 * 输入说明:  
 * 输出说明:
 * 作           者:  王聪
 * 编辑时间:  2017-7-3
 * 备           注:  
 ====================================================================================*/
void SHT_3X_SDA_LOW(void)
{
	SHT_3X_SDA_L;
}
/* ====================================================================================
 * 函数名称:  SCL_HIGH
 * 功能说明:  SCL 输出高
 * 输入说明:  
 * 输出说明:
 * 作           者:  王聪
 * 编辑时间:  2017-7-3
 * 备           注:  
 ====================================================================================*/
void SHT_3X_SCL_HIGH(void)
{
	SHT_3X_SCL_H;
}
/* ====================================================================================
 * 函数名称:  SCL_LOW
 * 功能说明:  SCL 输出低
 * 输入说明:  
 * 输出说明:
 * 作           者:  王聪
 * 编辑时间:  2017-7-3
 * 备           注:  
 ====================================================================================*/
void SHT_3X_SCL_LOW(void)
{
	SHT_3X_SCL_L;
}
/* ====================================================================================
 * 函数名称:  SDA_READ
 * 功能说明:  SDA 输入电平
 * 输入说明:  
 * 输出说明:
 * 作           者:  王聪
 * 编辑时间:  2017-7-3
 * 备           注:  
 ====================================================================================*/
unsigned char SHT_3X_SDA_READ(void)
{
	return (SHT_3X_SDA_IN);
}
/* ====================================================================================
 * 函数名称:  SCL_READ
 * 功能说明:  SCL 输入电平
 * 输入说明:  
 * 输出说明:
 * 作           者:  王聪
 * 编辑时间:  2017-7-3
 * 备           注:  
 ====================================================================================*/
unsigned char SHT_3X_SCL_READ(void)
{
	return (SHT_3X_SCL_IN);
}
/* ====================================================================================
 * 函数名称:  f_Sht3xI2cInit
 * 功能说明:  初始化SDA  / SCL
 * 输入说明:  
 * 输出说明:
 * 作           者:  王聪
 * 编辑时间:  2017-7-3
 * 备           注:  
 ====================================================================================*/
void f_Sht3xI2cInit(void)
{
	SHT_3X_I2C_SDA_OUT();
	SHT_3X_I2C_SCL_OUT();
	SHT_3X_SDA_HIHG();
	SHT_3X_SCL_HIGH();
}

/* ====================================================================================
 * 函数名称:  I2c_StartCondition
 * 功能说明:  开始信号
 * 输入说明:  
 * 输出说明:
 * 作           者:  王聪
 * 编辑时间:  2017-7-3
 * 备           注:  
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
 * 函数名称:  I2c_StopCondition
 * 功能说明:  停止信号
 * 输入说明:  
 * 输出说明:
 * 作           者:  王聪
 * 编辑时间:  2017-7-3
 * 备           注:  
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
 * 函数名称:  SHT_3X_I2c_WriteByte
 * 功能说明:  写一个字节
 * 输入说明:  txByte -- 要写入的字节
 * 输出说明:
 * 作           者:  王聪
 * 编辑时间:  2017-7-3
 * 备           注:  
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
 * 函数名称:  SHT_3X_I2c_ReadByte
 * 功能说明:  读一个字节
 * 输入说明:  rxByte -- 读的字节
 * 输出说明:
 * 作           者:  王聪
 * 编辑时间:  2017-7-3
 * 备           注:  
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
 * 函数名称:  I2c_WaitWhileClockStreching
 * 功能说明:  
 * 输入说明:  timeout -- 超时时间
 * 输出说明:
 * 作           者:  王聪
 * 编辑时间:  2017-7-3
 * 备           注:  
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
 * 函数名称:  I2c_GeneralCallReset
 * 功能说明:  
 * 输入说明:  
 * 输出说明:
 * 作           者:  王聪
 * 编辑时间:  2017-7-3
 * 备           注:  
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


