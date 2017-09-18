#ifndef __TYPES_H
#define __TYPES_H
//����PM2.5
typedef union PM25_up
{
   unsigned char data_buf[32];
   struct pm25_up_t
   {
       unsigned char Head_byte[2];//0x42 0x4D
       unsigned char Data_length[2];//0x00 0x1c
       unsigned char pm1_bz[2]; //pm1.0��׼������ug/m3
       unsigned char pm25_bz[2];   //pm2.5��׼������ug/m3
       unsigned char pm10_bz[2];   //pm10��׼������ug/m3
       unsigned char pm1_dq[2]; //pm1.0��������ug/m3
       unsigned char pm25_dq[2];   //pm2.5��������ug/m3
       unsigned char pm10_dq[2];   //pm10��������ug/m3
       unsigned char pm03_kq[2]; //pm0.3 0.1������������
       unsigned char pm05_kq[2]; //pm0.5 0.1������������
       unsigned char pm1_kq[2]; //pm1.0 0.1������������
       unsigned char pm25_kq[2];//pm2.5 0.1������������
       unsigned char pm5_kq[2];//pm5 0.1������������
       unsigned char pm10_kq[2];//pm10 0.1������������
       unsigned char bb_code;  //�汾��0x91
       unsigned char error_code;  //������0x00
       unsigned char Check_code[2];//У���룬��ȥ��ͷ�Ͱ�β�����ֽڵ��ۼӺ�
   }data_core;
}PM25_up_t;

//��������ϴ�,�ô���Ϊ�¶ȡ�ʪ�ȡ�PM2.5ֵ��PM03ֵ������
typedef union Data_up
{
   unsigned char data_buf[31];
   struct data_up_t
   {
       unsigned char Head_byte[2];//0xEB 0x90
       unsigned char Data_length[2];//0x00 0x17
       unsigned char Data_type; //���ݵ�����0x04
       unsigned char MAC_addr[4];   //�豸��ַ 
       unsigned char PM25[3];//��һ���ֽ�0x01,�����������ݣ�����Ϊ����
       unsigned char PM03[3];//��һ���ֽ�0x0E,�����������ݣ�����Ϊ����
       unsigned char TEM[3];//��һ���ֽ�0x03,������������,����Ϊ1000+����׼��16
       unsigned char HUM[3];//��һ���ֽ�0x04,������������,����Ϊ1000+����׼��16
       unsigned char POW[3];//��һ���ֽ�0x0A,�����������ݣ�����Ϊ�ٷֱ�
       unsigned char POW_STA[3];//��������Ϊ0X01�������Ϊ0x00
       unsigned char Check_code[2];//У���룬��ȥ��ͷ�Ͱ�β�����ֽڵ��ۼӺ�
       unsigned char Tial[2];//0x0D 0x0A
   }data_core;
}Data_up_t;

//��������
typedef union Dev_control
{
   unsigned char data_buf[24];
   struct dec_ctl_t
   {
       unsigned char Head_byte[2];//0xEB 0x90
       unsigned char Data_length[2];//0x00 0x18
       unsigned char Data_type; //���ݵ�����0x05
       unsigned char MAC_addr[4];   //�豸��ַ 
       unsigned char Serial_code[8];//������ˮ��
       unsigned char Cmd_code[3];//�����֣����� 81 00 0A ���ػ�81 00 0B  ...
       unsigned char Check_code[2];//У���룬��ȥ��ͷ�Ͱ�β�����ֽڵ��ۼӺ�
       unsigned char Tial[2];//0x0D 0x0A
   }data_core;
}Dev_control_t;

//����������Ӧ
typedef union Dev_control_res
{
   unsigned char data_buf[24];
   struct dec_ctlres_t
   {
       unsigned char Head_byte[2];//0xEB 0x90
       unsigned char Data_length[2];//0x00 0x18
       unsigned char Data_type; //���ݵ�����0x06
       unsigned char MAC_addr[4];   //�豸��ַ 
       unsigned char Serial_code[8];//������ˮ��
       unsigned char Cmd_code[3];//�����֣����� 81 00 0A ���ػ�81 00 0B
       unsigned char Check_code[2];//У���룬��ȥ��ͷ�Ͱ�β�����ֽڵ��ۼӺ�
       unsigned char Tial[2];//0x0D 0x0A
   }data_core;
}Dev_control_res_t;

//��������
typedef union Head_up
{
   unsigned char data_buf[13];
   struct head_data_t
   {
       unsigned char Head_byte[2];//0xEB 0x90
       unsigned char Data_length[2];//0x00 0x0B
       unsigned char Data_type; //���ݵ�����0x01
       unsigned char MAC_addr[4];   //�豸��ַ 
       unsigned char Check_code[2];//У���룬��ȥ��ͷ�Ͱ�β�����ֽڵ��ۼӺ�
       unsigned char Tial[2];//0x0D 0x0A
   }data_core;
}Head_up_t;

//д���豸ID
typedef union Dev_dp
{
   unsigned char data_buf[13];
   struct dev_dp_t
   {
       unsigned char Head_byte[2];//0xEB 0x90
       unsigned char Data_length[2];//0x00 0x0B
       unsigned char Data_type; //���ݵ�����0xFD
       unsigned char MAC_addr[4];   //�豸��ַ,��λ��ǰ��λ�ں�
       unsigned char Check_code[2];//У���룬��ȥ��ͷ�Ͱ�β�����ֽڵ��ۼӺ�
       unsigned char Tial[2];//0x0D 0x0A
   }data_core;
}Dev_dp_t;

//д���豸ID����
typedef union Dev_up
{
   unsigned char data_buf[13];
   struct dec_up_t
   {
       unsigned char Head_byte[2];//0xEB 0x90
       unsigned char Data_length[2];//0x00 0x0D
       unsigned char Data_type; //���ݵ�����0xFE
       unsigned char MAC_addr[4];   //�豸��ַ 
       unsigned char Check_code[2];//У���룬��ȥ��ͷ�Ͱ�β�����ֽڵ��ۼӺ�
       unsigned char Tial[2];//0x0D 0x0A
   }data_core;
}Dev_up_t;
#endif
