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

//������������õ�����ֵ
#define LED1ON	1
#define LED1OFF	2
#define BEEPON	3
#define BEEPOFF	4
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
u8 CommandProcess(u8 *str)
{
	u8 CommandValue=COMMANDERR;
	if(strcmp((char*)str,"LED1ON")==0) CommandValue=LED1ON;
	else if(strcmp((char*)str,"LED1OFF")==0) CommandValue=LED1OFF;
	else if(strcmp((char*)str,"BEEPON")==0) CommandValue=BEEPON;
	else if(strcmp((char*)str,"BEEPOFF")==0) CommandValue=BEEPOFF;
	return CommandValue;
}

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4	 
	delay_init();	    				//��ʱ������ʼ��	 
	//uart_init(115200);					//��ʼ������
	myuart_init(1,115200);//��ʼ������1
	myuart_init(2,115200);//��ʼ������2
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
    //����UART2����
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
	
	u8 *CommandStr;
	while(1)
	{
		err=xSemaphoreTake(BinarySemaphore_uart2,portMAX_DELAY);	//��ȡ�ź���
		if(err==pdTRUE)										//��ȡ�ź����ɹ�
		{
			len=USART2_RX_STA&0x3fff;						//�õ��˴ν��յ������ݳ���
			CommandStr=mymalloc(SRAMIN,len);				//�����ڴ�
			mymemcpy((char *)CommandStr,USART2_RX_BUF,len);
			
			myuart_send(2,CommandStr,len);
		
			USART2_RX_STA=0;
			memset(USART2_RX_BUF,0,USART_REC_LEN);			//���ڽ��ջ���������
			myfree(SRAMIN,CommandStr);				        //�ͷ��ڴ�
		}
	}
}

//UART1_task����
void UART1_task(void *pvParameters)
{
	u8 len=0;
	u8 CommandValue=COMMANDERR;
	BaseType_t err=pdFALSE;
	
	u8 *CommandStr;
	while(1)
	{
		err=xSemaphoreTake(BinarySemaphore,portMAX_DELAY);	//��ȡ�ź���
		if(err==pdTRUE)										//��ȡ�ź����ɹ�
		{
			len=USART_RX_STA&0x3fff;						//�õ��˴ν��յ������ݳ���
			CommandStr=mymalloc(SRAMIN,len+1);				//�����ڴ�
			sprintf((char*)CommandStr,"%s",USART_RX_BUF);
			CommandStr[len]='\0';							//�����ַ�����β����
			LowerToCap(CommandStr,len);						//���ַ���ת��Ϊ��д		
			CommandValue=CommandProcess(CommandStr);		//�������
			if(CommandValue!=COMMANDERR)
			{
				printf("����Ϊ:%s\r\n",CommandStr);
				switch(CommandValue)						//��������
				{
					case LED1ON: 
						LED1=0;
						break;
					case LED1OFF:
						LED1=1;
						break;
					case BEEPON:
						BEEP=1;
						break;
					case BEEPOFF:
						BEEP=0;
						break;
				}
			}
			else
			{
				printf("��Ч���������������!!\r\n");
			}
			USART_RX_STA=0;
			memset(USART_RX_BUF,0,USART_REC_LEN);			//���ڽ��ջ���������
			myfree(SRAMIN,CommandStr);						//�ͷ��ڴ�
		}
	}
}



