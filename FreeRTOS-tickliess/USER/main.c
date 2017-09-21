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

/************************************************
 ALIENTEK ս��STM32F103������ FreeRTOSʵ��18-1
 FreeRTOS�͹���Ticklessģʽʵ��-�⺯���汾
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/
//������************************************************/
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
TaskHandle_t UART1Task_Handler;
//������
void UART1_task(void *pvParameters);

/*����2����*/
//�������ȼ�
#define UART2_TASK_PRIO		3
//�����ջ��С	
#define UART2_STK_SIZE 		256  
//������
TaskHandle_t UART2Task_Handler;
//������
void UART2_task(void *pvParameters);

/*������������*/
//�������ȼ���
#define KEY_TASK_PRIO		4
//�����ջ��С	
#define KEY_STK_SIZE 		256  
//������
TaskHandle_t KEYTask_Handler;
//������
void KEY_task(void *pvParameters);


PM25_up_t PM25_data;
Dev_control_t DEVCON_data;//�豸����
Dev_dp_t  DEVIDSET_data;
Head_up_t HEAD_data;

u16 DEV_ID[2];

void PreSleepProcessing(uint32_t ulExpectedIdleTime);
void PostSleepProcessing(uint32_t ulExpectedIdleTime);

void LowerToCap(u8 *str,u8 len);
void CommandProcess(u8 buf[],u8 len);

void LowerToCap(u8 *str,u8 len);
//��ֵ�ź������
SemaphoreHandle_t BinarySemaphore;	//��ֵ�ź������
SemaphoreHandle_t BinarySemaphore_uart2;	//����2��ֵ�ź������

TimerHandle_t 	AutoReloadTimer_Handle;			//���ڶ�ʱ�����
TimerHandle_t	OneShotTimer_Handle;			//���ζ�ʱ�����

void AutoReloadCallback(TimerHandle_t xTimer); 	//���ڶ�ʱ���ص�����
void OneShotCallback(TimerHandle_t xTimer);		//���ζ�ʱ���ص�����

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
	*/
}

//�˳��͹���ģʽ�Ժ���Ҫ���������
//ulExpectedIdleTime���͹���ģʽ����ʱ��
void PostSleepProcessing(uint32_t ulExpectedIdleTime)
{
	//�˳��͹���ģʽ�Ժ����Щ���رյ�����ʱ�ӣ��˴�ֻ����ʾ�Դ���
	/*
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE); 
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
	mymemcpy(data_buf,buf,len);
	switch(len)
	{
		case 0x0d:
			if(CMD_data == 0x01)
			{
				//����
				mymemcpy(HEAD_data.data_buf,buf,len);
				
			}
			if(CMD_data == 0xFD)
			{
				//д���豸ID
				
			}
			break;
		case 0x18:
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
					FAN_set(DEVCON_data.data_core.Cmd_code[2]);
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
	myuart_init(1,115200);//��ʼ������1
	myuart_init(2,9600);//��ʼ������2
	LED_Init();		  					//��ʼ��LED
	KEY_Init();							//��ʼ������
	BEEP_Init();						//��ʼ��������
	my_mem_init(SRAMIN);            	//��ʼ���ڲ��ڴ��
	//STMFLASH_Read(FLASH_SAVE_ADDR,DEV_ID,2);
	
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
	
	 //����������ڶ�ʱ��
  AutoReloadTimer_Handle=xTimerCreate((const char*		)"AutoReloadTimer",
									    (TickType_t			)10000,
							            (UBaseType_t		)pdTRUE,
							            (void*				)1,
							            (TimerCallbackFunction_t)AutoReloadCallback); //���ڶ�ʱ��������1s(1000��ʱ�ӽ���)������ģʽ
												
	/*												
    //�������ζ�ʱ��
	OneShotTimer_Handle=xTimerCreate((const char*			)"OneShotTimer",
							         (TickType_t			)2000,
							         (UBaseType_t			)pdFALSE,
							         (void*					)2,
							         (TimerCallbackFunction_t)OneShotCallback); //���ζ�ʱ��������2s(2000��ʱ�ӽ���)������ģʽ		
	*/										
		if(AutoReloadTimer_Handle!=NULL)
		{
			xTimerStart(AutoReloadTimer_Handle,0);	//�������ڶ�ʱ��
		}
	
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
                (TaskHandle_t*  )&UART1Task_Handler); 
		//������������
    xTaskCreate((TaskFunction_t )KEY_task,     
                (const char*    )"KEY_task",   
                (uint16_t       )KEY_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )KEY_TASK_PRIO,
                (TaskHandle_t*  )&KEYTask_Handler); 
							
    vTaskDelete(StartTask_Handler); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���						
}

//���ڶ�ʱ���Ļص�����
void AutoReloadCallback(TimerHandle_t xTimer)
{
	//myuart_send(1,"1234",4);
	
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
			#ifdef __DEBUG
			myuart_send(2,PM25_data.data_buf,len);
			#endif
		
			USART2_RX_STA=0;
			memset(USART2_RX_BUF,0,USART_REC_LEN);			//���ڽ��ջ���������
		}
		 vTaskDelay(10); //��ʱ10ms��Ҳ����10��ʱ�ӽ���
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

			mymemcpy(CommandStr,USART_RX_BUF,len);
			CommandStr[len]=0x0D;
			CommandStr[len+1]=0x0A;
			
			CommandProcess(CommandStr,len+2);		//�������
		
			USART_RX_STA=0;
			memset(USART_RX_BUF,0,USART_REC_LEN);			//���ڽ��ջ���������
			memset(CommandStr,0,40);	
		}
		 vTaskDelay(10); //��ʱ10ms��Ҳ����10��ʱ�ӽ���
	}
}

void KEY_task(void *pvParameters)
{
	u8 key;
	u8 mode_temp;
	while(1){
		key=KEY_Scan(0);
		switch(key)
		{
			case WKUP_PRES:
				//���ػ�
			if(SYS_STA&0x80)
			{
				FAN_set(0x0B);//����״̬
			}else
			{
				FAN_set(0x11);//Ĭ��һ����
			}
			#ifdef __BUG
			myuart_send(1,&SYS_STA,1);
			#endif
				break;
			
			case KEY0_PRES:
				//����ģʽ
				break;
			
			case KEY1_PRES:
				//���÷��������е�
				mode_temp=SYS_STA&0x8F;
				switch(mode_temp)
				{
					case 0x81:
						FAN_set(0x12);
						break;
					case 0x83:
						FAN_set(0x13);
					break;
					case 0x87:
						FAN_set(0x14);
					break;
					case 0x8F:
						FAN_set(0x11);
					break;
					default:
						break;
				}
				#ifdef __DEBUG
					myuart_send(1,&SYS_STA,1);
				#endif
				break;
				
			case KEY2_PRES:
				//��������
				break;
			
			default:
			break;					
		}
		 vTaskDelay(10); //��ʱ10ms��Ҳ����10��ʱ�ӽ���
	}
}
