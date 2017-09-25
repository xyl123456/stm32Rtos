/* ===========================================================================================
  * 文件名称:  SHT_3X_I2C.h
  * 建立时间:  2017-7-3
  * 功能描述:  温湿度i2c 底层驱动头文件(优化厂家提供代码，更好的移植性，规避警告)
  * 作           者:  王聪
  * 公           司:  艾佳智能科技有限公司
  * 更改记录:  
  * ========================================================================================== */

#ifndef   SHT_3X_I2C_H
#define SHT_3X_I2C_H

#ifdef    SHT_3X_I2C_TYPE_CLYCLE
	#define   SHT_3X_I2C_TYPE
#else
	#define   SHT_3X_I2C_TYPE   extern
#endif

#define  SHT_3X_SDA_L                                                   GPIO_ResetBits(GPIOB,GPIO_Pin_11)
#define  SHT_3X_SDA_H                                                   GPIO_SetBits(GPIOB,GPIO_Pin_11)
#define  SHT_3X_SCL_L                                                  	GPIO_ResetBits(GPIOB,GPIO_Pin_10)
#define  SHT_3X_SCL_H                                                   GPIO_SetBits(GPIOB,GPIO_Pin_10)


#define  SHT_3X_SDA_IN                                                  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11)
#define  SHT_3X_SCL_IN                                                   GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10)

#define  SHT_3X_DELAY_COUNT                                      (unsigned char)5

//==============================================================================
extern void f_Sht3xI2cInit(void);
extern void SHT_3X_I2c_StartCondition(void);
extern void SHT_3X_I2c_StopCondition(void);
extern unsigned char SHT_3X_I2c_WriteByte(unsigned char txByte);
extern unsigned char SHT_3X_I2c_ReadByte(unsigned char *rxByte, unsigned char ack, unsigned char timeout);
extern unsigned char I2c_WaitWhileClockStreching(unsigned char timeout);
extern unsigned char I2c_GeneralCallReset(void);

#endif

