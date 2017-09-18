#ifndef __TYPES_H
#define __TYPES_H
//激光PM2.5
typedef union PM25_up
{
   unsigned char data_buf[32];
   struct pm25_up_t
   {
       unsigned char Head_byte[2];//0x42 0x4D
       unsigned char Data_length[2];//0x00 0x1c
       unsigned char pm1_bz[2]; //pm1.0标准颗粒物ug/m3
       unsigned char pm25_bz[2];   //pm2.5标准颗粒物ug/m3
       unsigned char pm10_bz[2];   //pm10标准颗粒物ug/m3
       unsigned char pm1_dq[2]; //pm1.0大气环境ug/m3
       unsigned char pm25_dq[2];   //pm2.5大气环境ug/m3
       unsigned char pm10_dq[2];   //pm10大气环境ug/m3
       unsigned char pm03_kq[2]; //pm0.3 0.1升空气颗粒数
       unsigned char pm05_kq[2]; //pm0.5 0.1升空气颗粒数
       unsigned char pm1_kq[2]; //pm1.0 0.1升空气颗粒数
       unsigned char pm25_kq[2];//pm2.5 0.1升空气颗粒数
       unsigned char pm5_kq[2];//pm5 0.1升空气颗粒数
       unsigned char pm10_kq[2];//pm10 0.1升空气颗粒数
       unsigned char bb_code;  //版本号0x91
       unsigned char error_code;  //错误码0x00
       unsigned char Check_code[2];//校验码，除去包头和包尾所有字节的累加和
   }data_core;
}PM25_up_t;

//监测数据上传,该代码为温度、湿度、PM2.5值、PM03值、电量
typedef union Data_up
{
   unsigned char data_buf[31];
   struct data_up_t
   {
       unsigned char Head_byte[2];//0xEB 0x90
       unsigned char Data_length[2];//0x00 0x17
       unsigned char Data_type; //数据的类型0x04
       unsigned char MAC_addr[4];   //设备地址 
       unsigned char PM25[3];//第一个字节0x01,后两个是数据，数据为整数
       unsigned char PM03[3];//第一个字节0x0E,后两个是数据，数据为整数
       unsigned char TEM[3];//第一个字节0x03,后两个是数据,数据为1000+数据准换16
       unsigned char HUM[3];//第一个字节0x04,后两个是数据,数据为1000+数据准换16
       unsigned char POW[3];//第一个字节0x0A,后两个是数据，数据为百分比
       unsigned char POW_STA[3];//如果充电则为0X01，不充电为0x00
       unsigned char Check_code[2];//校验码，除去包头和包尾所有字节的累加和
       unsigned char Tial[2];//0x0D 0x0A
   }data_core;
}Data_up_t;

//控制命令
typedef union Dev_control
{
   unsigned char data_buf[24];
   struct dec_ctl_t
   {
       unsigned char Head_byte[2];//0xEB 0x90
       unsigned char Data_length[2];//0x00 0x18
       unsigned char Data_type; //数据的类型0x05
       unsigned char MAC_addr[4];   //设备地址 
       unsigned char Serial_code[8];//命令流水号
       unsigned char Cmd_code[3];//命令字，开机 81 00 0A ，关机81 00 0B  ...
       unsigned char Check_code[2];//校验码，除去包头和包尾所有字节的累加和
       unsigned char Tial[2];//0x0D 0x0A
   }data_core;
}Dev_control_t;

//控制命令响应
typedef union Dev_control_res
{
   unsigned char data_buf[24];
   struct dec_ctlres_t
   {
       unsigned char Head_byte[2];//0xEB 0x90
       unsigned char Data_length[2];//0x00 0x18
       unsigned char Data_type; //数据的类型0x06
       unsigned char MAC_addr[4];   //设备地址 
       unsigned char Serial_code[8];//命令流水号
       unsigned char Cmd_code[3];//命令字，开机 81 00 0A ，关机81 00 0B
       unsigned char Check_code[2];//校验码，除去包头和包尾所有字节的累加和
       unsigned char Tial[2];//0x0D 0x0A
   }data_core;
}Dev_control_res_t;

//心跳命令
typedef union Head_up
{
   unsigned char data_buf[13];
   struct head_data_t
   {
       unsigned char Head_byte[2];//0xEB 0x90
       unsigned char Data_length[2];//0x00 0x0B
       unsigned char Data_type; //数据的类型0x01
       unsigned char MAC_addr[4];   //设备地址 
       unsigned char Check_code[2];//校验码，除去包头和包尾所有字节的累加和
       unsigned char Tial[2];//0x0D 0x0A
   }data_core;
}Head_up_t;

//写入设备ID
typedef union Dev_dp
{
   unsigned char data_buf[13];
   struct dev_dp_t
   {
       unsigned char Head_byte[2];//0xEB 0x90
       unsigned char Data_length[2];//0x00 0x0B
       unsigned char Data_type; //数据的类型0xFD
       unsigned char MAC_addr[4];   //设备地址,高位在前地位在后
       unsigned char Check_code[2];//校验码，除去包头和包尾所有字节的累加和
       unsigned char Tial[2];//0x0D 0x0A
   }data_core;
}Dev_dp_t;

//写入设备ID返回
typedef union Dev_up
{
   unsigned char data_buf[13];
   struct dec_up_t
   {
       unsigned char Head_byte[2];//0xEB 0x90
       unsigned char Data_length[2];//0x00 0x0D
       unsigned char Data_type; //数据的类型0xFE
       unsigned char MAC_addr[4];   //设备地址 
       unsigned char Check_code[2];//校验码，除去包头和包尾所有字节的累加和
       unsigned char Tial[2];//0x0D 0x0A
   }data_core;
}Dev_up_t;
#endif
