#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "timer.h"
#include "lcd.h"
#include "key.h"
#include "beep.h"
#include "malloc.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "types.h"
#include "timers.h"
#include "stmflash.h"
#include "sht3x.h"

PM25_up_t PM25_data;
Dev_control_t DEVCON_data;//设备控制
Dev_dp_t  DEVIDSET_data;
Dev_up_t  DEVUP_data;
Head_up_t HEAD_data;
SHT30_DATA_STRUCT SHT30_data;
Data_up_t DATAUP_data;

u16 DEV_ID[2];

u8 Head_line[2]={0xEB,0x90};
u8 Tail_line[2]={0x0D,0x0A};

u8 Dataup_line[2]={0x00,0x1D};
u8 Power_data[3]={0x0A,0x00,0x00};
u8 Power_state[3]={0x0F,0x00,0x00};

//命令处理函数，将字符串命令转换成命令值
//str：命令
//返回值: 0XFF，命令错误；其他值，命令值
void CommandProcess(u8 buf[],u8 len)
{
	u8 CMD_data=buf[4];
	u8 data_buf[40];
	int i;
	u8 chec_buf[2];
	u16 chec_int=0;
	
	mymemcpy(data_buf,buf,len);
	switch(len)
	{
		case 0x0d:
			if(CMD_data == 0x01)
			{
				//心跳
				mymemcpy(HEAD_data.data_buf,buf,len);
				
			}
			if(CMD_data == 0xFD)
			{
				//写入设备ID
				mymemcpy(DEVIDSET_data.data_buf,buf,len);
				DEV_ID[0]=0x0000;
				DEV_ID[1]=0x0000+DEVIDSET_data.data_core.MAC_addr[3];
				
				STMFLASH_Write(FLASH_SAVE_ADDR,DEV_ID,2);
				
				//处理设备数据返回
				mymemcpy(DEVUP_data.data_buf,buf,len);
				
				DEVUP_data.data_core.Data_type=0xFE;
				
				for(i=0;i<len-6;i++)
				{
					chec_int=chec_int + DEVUP_data.data_buf[2+i];
				} 
				chec_buf[0]=chec_int>>8;
				chec_buf[1]=chec_int;
				
				mymemcpy(DEVUP_data.data_core.Check_code,chec_buf,2);
				myuart_send(1,DEVUP_data.data_buf,len);
				
			}
			break;
		case 0x18:
			if(CMD_data == 0x05)
			{
				mymemcpy(DEVCON_data.data_buf,buf,len);
				//控制
				if(DEVCON_data.data_core.Cmd_code[0]==0x81)
				{
					//开关机,关闭屏幕，继电器，PM25等外设
					
				}
				if(DEVCON_data.data_core.Cmd_code[0]==0x82)
				{
					//负离子，紫外灯
				}
				if(DEVCON_data.data_core.Cmd_code[0]==0x84)
				{
					//风机0-4
					FAN_set(DEVCON_data.data_core.Cmd_code[2]);
				}
			}
			break;
			default:
			break;	
	}
}
//将字符串中的小写字母转换为大写
//str:要转换的字符串
//len：字符串长度
void LowerToCap(u8 *str,u8 len)
{
	u8 i;
	for(i=0;i<len;i++)
	{
		if((96<str[i])&&(str[i]<123))	//小写字母
		str[i]=str[i]-32;				//转换为大写
	}
}

//数据上报命令
void Send_data()
{
	u8 i;
	u8 temp;
	u16 chec_int=0;
	u8 chec_buf[2];
	u8 Dev_data[4];
	Dev_data[0]=DEV_ID[0]>>8;
	Dev_data[1]=DEV_ID[0];
	Dev_data[2]=DEV_ID[1]>>8;
	Dev_data[3]=DEV_ID[1];
	mymemcpy(DATAUP_data.data_core.Head_byte,Head_line,2);
	DATAUP_data.data_core.Data_type=0x04;
	
	mymemcpy(DATAUP_data.data_core.MAC_addr,Dev_data,4);
	
	mymemcpy(DATAUP_data.data_core.Data_length,Dataup_line,2);
	mymemcpy(DATAUP_data.data_core.Tial,Tail_line,2);
	//temp and humi

	temp = SHT3X_GetTempAndHumi(&SHT30_data.Temp,&SHT30_data.Humi, REPEATAB_HIGH, MODE_POLLING, 50);
	if(temp==NO_ERROR){
			SHT30_data.Temp_byte[0]=0x03;
			SHT30_data.Temp_byte[1]=((int)SHT30_data.Temp>>8)&0xff;
			SHT30_data.Temp_byte[2]=(int)SHT30_data.Temp&0xff;
			
			SHT30_data.Humi_byte[0]=0x04;
			SHT30_data.Humi_byte[1]=((int)SHT30_data.Humi>>8)&0xff;
			SHT30_data.Humi_byte[2]= (int)SHT30_data.Humi&0xff;
		#ifdef __DEBUG
			myuart_send(1,SHT30_data.Temp_byte,3);
			myuart_send(1,SHT30_data.Humi_byte,3);
		#endif
		mymemcpy(DATAUP_data.data_core.TEM,SHT30_data.Temp_byte,3);
		mymemcpy(DATAUP_data.data_core.HUM,SHT30_data.Humi_byte,4);
	}
	//pm25
	DATAUP_data.data_core.PM03[0]=0x0E;
	mymemcpy(DATAUP_data.data_core.PM03+1,PM25_data.data_core.pm10_bz,2);
	
	DATAUP_data.data_core.PM25[0]=0x01;
	mymemcpy(DATAUP_data.data_core.PM25+1,PM25_data.data_core.pm25_bz,2);
	
	mymemcpy(DATAUP_data.data_core.POW,Power_data,3);
	mymemcpy(DATAUP_data.data_core.POW_STA,Power_state,3);
	
	for(i=0;i<25;i++){
		chec_int=chec_int + DATAUP_data.data_buf[2+i];
	}
	chec_buf[0]=chec_int>>8;
	chec_buf[1]=chec_int;
	mymemcpy(DATAUP_data.data_core.Check_code,chec_buf,2);
	
	myuart_send(1,DATAUP_data.data_buf,31);
}

