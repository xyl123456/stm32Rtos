/* ===========================================================================================
  * 文件名称:  sht3x.h
  * 建立时间:  2017-7-3
  * 功能描述:  温湿度处理(优化厂家提供代码，更好的移植性，规避警告)
  * 作           者:  王聪
  * 公           司:  艾佳智能科技有限公司
  * 更改记录:  
  * ========================================================================================== */

#ifndef SHTX_H
#define SHTX_H

#ifdef SHT3X_TYPE_CLCYLE
	#define SHT3X_TYPE
#else
	#define SHT3X_TYPE    extern
#endif

#define   SHT_3X_ADD                    (unsigned char)0x44
#define   POLYNOMIAL                    0x131 // P(x) = x^8 + x^5 + x^4 + 1 = 100110001

extern unsigned char _i2cWriteHeader;
extern unsigned char _i2cReadHeader;

enum etCommands
{
	CMD_READ_SERIALNBR  = 0x3780,     // read serial number
	CMD_READ_STATUS     = 0xF32D,       // read status register
	CMD_CLEAR_STATUS    = 0x3041,       // clear status register
	CMD_HEATER_ENABLE   = 0x306D,       // enabled heater
	CMD_HEATER_DISABLE  = 0x3066,       // disable heater
	CMD_SOFT_RESET      = 0x30A2,           // sofloat reset
	CMD_MEAS_CLOCKSTR_H = 0x2C06,     // meas. clock stretching, high rep.
	CMD_MEAS_CLOCKSTR_M = 0x2C0D,    // meas. clock stretching, medium rep.
	CMD_MEAS_CLOCKSTR_L = 0x2C10, // meas. clock stretching, low rep.
	CMD_MEAS_POLLING_H  = 0x2400, // meas. no clock stretching, high rep.
	CMD_MEAS_POLLING_M  = 0x240B, // meas. no clock stretching, medium rep.
	CMD_MEAS_POLLING_L  = 0x2416, // meas. no clock stretching, low rep.
	CMD_MEAS_PERI_05_H  = 0x2032, // meas. periodic 0.5 mps, high rep.
	CMD_MEAS_PERI_05_M  = 0x2024, // meas. periodic 0.5 mps, medium rep.
	CMD_MEAS_PERI_05_L  = 0x202F, // meas. periodic 0.5 mps, low rep.
	CMD_MEAS_PERI_1_H   = 0x2130, // meas. periodic 1 mps, high rep.
	CMD_MEAS_PERI_1_M   = 0x2126, // meas. periodic 1 mps, medium rep.
	CMD_MEAS_PERI_1_L   = 0x212D, // meas. periodic 1 mps, low rep.
	CMD_MEAS_PERI_2_H   = 0x2236, // meas. periodic 2 mps, high rep.
	CMD_MEAS_PERI_2_M   = 0x2220, // meas. periodic 2 mps, medium rep.
	CMD_MEAS_PERI_2_L   = 0x222B, // meas. periodic 2 mps, low rep.
	CMD_MEAS_PERI_4_H   = 0x2334, // meas. periodic 4 mps, high rep.
	CMD_MEAS_PERI_4_M   = 0x2322, // meas. periodic 4 mps, medium rep.
	CMD_MEAS_PERI_4_L   = 0x2329, // meas. periodic 4 mps, low rep.
	CMD_MEAS_PERI_10_H  = 0x2737, // meas. periodic 10 mps, high rep.
	CMD_MEAS_PERI_10_M  = 0x2721, // meas. periodic 10 mps, medium rep.
	CMD_MEAS_PERI_10_L  = 0x272A, // meas. periodic 10 mps, low rep.
	CMD_FETCH_DATA      = 0xE000, // readout measurements for periodic mode
	CMD_R_AL_LIM_LS     = 0xE102, // read alert limits, low set
	CMD_R_AL_LIM_LC     = 0xE109, // read alert limits, low clear
	CMD_R_AL_LIM_HS     = 0xE11F, // read alert limits, high set
	CMD_R_AL_LIM_HC     = 0xE114, // read alert limits, high clear
	CMD_W_AL_LIM_LS     = 0x6100, // write alert limits, low set
	CMD_W_AL_LIM_LC     = 0x610B, // write alert limits, low clear
	CMD_W_AL_LIM_HS     = 0x611D, // write alert limits, high set
	CMD_W_AL_LIM_HC     = 0x6116, // write alert limits, high clear
	CMD_NO_SLEEP        = 0x303E,
};

enum etError
{
	NO_ERROR                = 0x00, // no error
	ACK_ERROR              = 0x01, // no acknowledgment error
	CHECKSUM_ERROR    = 0x02, // checksum mismatch error
	TIMEOUT_ERROR      = 0x04, // timeout error
	PARM_ERROR           = 0x80, // parameter out of range error
};

enum etI2cAck
{
	ACK  = 0,
	NACK = 1
};

enum etRepeatab
{
	REPEATAB_HIGH,   // high repeatability
	REPEATAB_MEDIUM, // medium repeatability
	REPEATAB_LOW    // low repeatability
};

enum etMode
{
	MODE_CLKSTRETCH, // clock stretching
	MODE_POLLING    // polling
};

enum etFrequency
{
	FREQUENCY_HZ5,  //  0.5 measurements per seconds
	FREQUENCY_1HZ,  //  1.0 measurements per seconds
	FREQUENCY_2HZ,  //  2.0 measurements per seconds
	FREQUENCY_4HZ,  //  4.0 measurements per seconds
	FREQUENCY_10HZ // 10.0 measurements per seconds
};

//==============================================================================
extern void SHT3X_SetI2cAdr(unsigned char i2cAdr);
extern void SHT3X_Init(void);
extern unsigned char SHT3X_Read2BytesAndCrc(unsigned short *data, unsigned char finaleAckNack, unsigned char timeout);
extern unsigned char SHT3X_Write2BytesAndCrc(unsigned short data);
extern unsigned char SHT3x_ReadSerialNumber(unsigned int *serialNbr);
extern unsigned char SHT3X_ReadStatus(unsigned short *status);
extern unsigned char SHT3X_ClearAllAlertFlags(void);
extern unsigned char SHT3X_ReadMeasurementBuffer(float *temp, float *humi);
extern unsigned char SHT3X_EnableHeater(void);
extern unsigned char SHT3X_DisbaleHeater(void);
extern unsigned char SHT3X_SofloatReset(void);
extern unsigned char SHT3X_StartWriteAccess(void);
extern unsigned char SHT3X_StartReadAccess(void);
extern void SHT3X_StopAccess(void);
extern unsigned char SHT3X_WriteCommand(unsigned short cmd);
extern unsigned char SHT3X_CalcCrc(unsigned char data[], unsigned char nbrOfBytes);
extern unsigned char SHT3X_CheckCrc(unsigned char data[], unsigned char nbrOfBytes, unsigned char checksum);
extern float SHT3X_CalcTemperature(unsigned short rawValue);
extern float SHT3X_CalcHumidity(unsigned short rawValue);
extern unsigned short SHT3X_CalcRawTemperature(float temperature);
extern unsigned short SHT3X_CalcRawHumidity(float humidity);
extern unsigned char SHT3X_GetTempAndHumi(float *temp,float *humi,unsigned char repeatab,unsigned char mode,unsigned char timeout);
extern unsigned char SHT3X_GetTempAndHumiClkStretch(float *temp,float *humi,unsigned char repeatab,unsigned char  timeout);
extern unsigned char SHT3X_GetTempAndHumiPolling(float *temp,float *humi,unsigned char repeatab,unsigned char timeout);
extern unsigned char SHT3X_StartPeriodicMeasurment(unsigned char  repeatab,unsigned char freq);

#endif



