/* ===========================================================================================
  * �ļ�����:  SHT_3X_I2C.h
  * ����ʱ��:  2017-7-3
  * ��������:  ��ʪ��i2c �ײ�����ͷ�ļ�(�Ż������ṩ���룬���õ���ֲ�ԣ���ܾ���)
  * ��           ��:  ����
  * ��           ˾:  �������ܿƼ����޹�˾
  * ���ļ�¼:  
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

