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
/************************************************
 ALIENTEK ս��STM32F103������ FreeRTOSʵ��18-1
 FreeRTOS�͹���Ticklessģʽʵ��-�⺯���汾
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/

//�������ȼ�
#define START_TASK_PRIO		1
//�����ջ��С	
#define START_STK_SIZE 		256  
//������
TaskHandle_t StartTask_Handler;
//������
void start_task(void *pvParameters);

/*����1����*/
//�������ȼ�
#define UART1_TASK_PRIO 2
//�����ջ��С	
#define UART1_STK_SIZE  256 
//������
TaskHandle_t UART1_Handler;
//������
void UART1_task(void *pvParameters);

PM25_up_t PM25_data;
Dev_control_t DEVCON_data;//�豸����
Dev_dp_t  DEVIDSET_data;
Head_up_t HEAD_data;

/*����2����*/
//�������ȼ���
#define UART2_TASK_PRIO		3
//�����ջ��С	
#define UART2_STK_SIZE 		256  
//������
TaskHandle_t UART2Task_Handler;
//������
void UART2_task(void *pvParameters);

//��ֵ�ź������
SemaphoreHandle_t BinarySemaphore;	//��ֵ�ź������
SemaphoreHandle_t BinarySemaphore_uart2;	//����2��ֵ�ź������

//������������õ���������
#define SYSTEM	  0x01    
#define PM25			0x02
#define FAN1  		0x03
#define RFD   		0x04

#define COMMANDERR	0XFF

//����͹���ģʽǰ��Ҫ���������
//ulExpectedIdleTime���͹���ģʽ����ʱ��
void PreSleepProcessing(uint32_t ulExpectedIdleTime)
{
	//�ر�ĳЩ�͹���ģʽ�²�ʹ�õ�����ʱ�ӣ��˴�ֻ����ʾ�Դ���
/*
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,DISABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,DISABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,DISABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,DISABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF,DISABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG,DISABLE);
	*/
}

//�˳��͹���ģʽ�Ժ���Ҫ���������
//ulExpectedIdleTime���͹���ģʽ����ʱ��
void PostSleepProcessing(uint32_t ulExpectedIdleTime)
{
	//�˳��͹���ģʽ�Ժ����Щ���رյ�����ʱ�ӣ��˴�ֻ����ʾ�Դ���
	/*
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG,ENABLE);	 
*/	
}

//���ַ����е�Сд��ĸת��Ϊ��д
//str:Ҫת�����ַ���
//len���ַ�������
void LowerToCap(u8 *str,u8 len)
{
	u8 i;
	for(i=0;i<len;i++)
	{
		if((96<str[i])&&(str[i]<123))	//Сд��ĸ
		str[i]=str[i]-32;				//ת��Ϊ��д
	}
}

//������������ַ�������ת��������ֵ
//str������
//����ֵ: 0XFF�������������ֵ������ֵ
void CommandProcess(u8 buf[],u8 len)
{
	u8 CMD_data=buf[4];
	u8 data_buf[40];
	myuart_send(1,buf,len);
	mymemcpy(data_buf,buf,len);
	
	switch(len)
	{
		case 13:
			if(CMD_data == 0x01)
			{
				//����
				mymemcpy(HEAD_data.data_buf,buf,len);
				
			}
			if(CMD_data == 0xFD)
			{
				//д���豸ID
				mymemcpy(DEVIDSET_data.data_buf,buf,len);
			}
			break;
		case 24:
			if(CMD_data == 0x05)
			{
				mymemcpy(DEVCON_data.data_buf,buf,len);
				//����
				if(DEVCON_data.data_core.Cmd_code[0]==0x81)
				{
					//���ػ�,�ر���Ļ���̵�����PM25������
					
					
				}
				if(DEVCON_data.data_core.Cmd_code[0]==0x82)
				{
					//�����ӣ������
				}
				if(DEVCON_data.data_core.Cmd_code[0]==0x84)
				{
					//���0-4
					if(DEVCON_data.data_core.Cmd_code[2]==0x0B)
					{
						//0��������
					}
					if(DEVCON_data.data_core.Cmd_code[2]==0x11)
					{
						//1��
						RELAY1=1;
						RELAY2=0;
						RELAY3=0;
						RELAY4=0;
					}
					if(DEVCON_data.data_core.Cmd_code[2]==0x12)
					{
						//2��
						RELAY1=1;
						RELAY2=1;
						RELAY3=0;
						RELAY4=0;
					}
					if(DEVCON_data.data_core.Cmd_code[2]==0x13)
					{
						//3��
						RELAY1=1;
						RELAY2=1;
						RELAY3=1;
						RELAY4=0;
					}
					if(DEVCON_data.data_core.Cmd_code[2]==0x14)
					{
						//4��
						RELAY1=1;
						RELAY2=1;
						RELAY3=1;
						RELAY4=1;
					}
					
				}
			}
			break;
			default:
			break;
		
	}
}

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4	 
	delay_init();	    				//��ʱ������ʼ��	 
	//uart_init(115200);					//��ʼ������
	myuart_init(1,115200);//��ʼ������1
	myuart_init(2,9600);//��ʼ������2
	LED_Init();		  					//��ʼ��LED
	KEY_Init();							//��ʼ������
	BEEP_Init();						//��ʼ��������
	my_mem_init(SRAMIN);            	//��ʼ���ڲ��ڴ��
	
	//������ʼ����
    xTaskCreate((TaskFunction_t )start_task,            //������
                (const char*    )"start_task",          //��������
                (uint16_t       )START_STK_SIZE,        //�����ջ��С
                (void*          )NULL,                  //���ݸ��������Ĳ���
                (UBaseType_t    )START_TASK_PRIO,       //�������ȼ�
                (TaskHandle_t*  )&StartTask_Handler);   //������              
    vTaskStartScheduler();          //�����������
}

//��ʼ����������
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //�����ٽ���
	
	//������ֵ�ź���
	BinarySemaphore=xSemaphoreCreateBinary();
	//����2��ֵ�ź���
	BinarySemaphore_uart2=xSemaphoreCreateBinary();
    //����UART2���񣬼��PM2.5
    xTaskCreate((TaskFunction_t )UART2_task,             
                (const char*    )"UART2_task",           
                (uint16_t       )UART2_STK_SIZE,        
                (void*          )NULL,                  
                (UBaseType_t    )UART2_TASK_PRIO,        
                (TaskHandle_t*  )&UART2Task_Handler);   
    //����UART1����
    xTaskCreate((TaskFunction_t )UART1_task,     
                (const char*    )"UART1_task",   
                (uint16_t       )UART1_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )UART1_TASK_PRIO,
                (TaskHandle_t*  )&UART1_Handler); 
    vTaskDelete(StartTask_Handler); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���
}

//UART2������
void UART2_task(void *pvParameters)
{
	u8 len=0;
	BaseType_t err=pdFALSE;
	
	while(1)
	{
		err=xSemaphoreTake(BinarySemaphore_uart2,portMAX_DELAY);	//��ȡ�ź���
		if(err==pdTRUE)										//��ȡ�ź����ɹ�
		{
			len=USART2_RX_STA&0x3fff;						//�õ��˴ν��յ������ݳ���
			//CommandStr=mymalloc(SRAMIN,len);			myfree(SRAMIN,CommandStr); //�ͷ��ڴ�	//�����ڴ�
			mymemcpy(PM25_data.data_buf,USART2_RX_BUF,len);
			//#ifdef __DEBUG
			myuart_send(2,PM25_data.data_buf,len);
			//#endif
		
			USART2_RX_STA=0;
			memset(USART2_RX_BUF,0,USART_REC_LEN);			//���ڽ��ջ���������
			
		}
	}
}

//UART1_task����
void UART1_task(void *pvParameters)
{
	u8 len=0;
	BaseType_t err=pdFALSE;
	
	u8 CommandStr[40];
	while(1)
	{
		err=xSemaphoreTake(BinarySemaphore,portMAX_DELAY);	//��ȡ�ź���
		if(err==pdTRUE)										//��ȡ�ź����ɹ�
		{
			len=USART_RX_STA&0x3fff;						//�õ��˴ν��յ������ݳ���
			//CommandStr=mymalloc(SRAMIN,len);				//�����ڴ�
			//sprintf((char*)CommandStr,"%s",USART_RX_BUF);
			mymemcpy(CommandStr,USART_RX_BUF,len);
			//CommandStr[len]='\0';							//�����ַ�����β����
			//LowerToCap(CommandStr,len);						//���ַ���ת��Ϊ��д		
			CommandProcess(CommandStr,len);		//�������
		
			USART_RX_STA=0;
			memset(USART_RX_BUF,0,USART_REC_LEN);			//���ڽ��ջ���������
			myfree(SRAMIN,CommandStr);						//�ͷ��ڴ�
		}
	}
}



