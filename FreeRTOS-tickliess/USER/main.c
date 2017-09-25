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

/*����3����*/
//�������ȼ�
#define UART3_TASK_PRIO		4
//�����ջ��С	
#define UART3_STK_SIZE 		256  
//������
TaskHandle_t UART3Task_Handler;
//������
void UART3_task(void *pvParameters);

/*������������*/
//�������ȼ���
#define KEY_TASK_PRIO		5
//�����ջ��С	
#define KEY_STK_SIZE 		256  
//������
TaskHandle_t KEYTask_Handler;
//������
void KEY_task(void *pvParameters);


void PreSleepProcessing(uint32_t ulExpectedIdleTime);
void PostSleepProcessing(uint32_t ulExpectedIdleTime);

void LowerToCap(u8 *str,u8 len);
//��ֵ�ź������
SemaphoreHandle_t BinarySemaphore;	//��ֵ�ź������
SemaphoreHandle_t BinarySemaphore_uart2;	//����2��ֵ�ź������
SemaphoreHandle_t BinarySemaphore_uart3;	//����3��ֵ�ź������

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

//LCDˢ��ʱʹ�õ���ɫ
int lcd_discolor[14]={	WHITE, BLACK, BLUE,  BRED,      
						GRED,  GBLUE, RED,   MAGENTA,       	 
						GREEN, CYAN,  YELLOW,BROWN, 			
						BRRED, GRAY };

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4	 
	delay_init();	    				//��ʱ������ʼ��	 
	myuart_init(1,115200);//��ʼ������1
	myuart_init(2,9600);//��ʼ������2
	myuart_init(3,115200);//��ʼ������3
	LED_Init();		  					//��ʼ��LED
//	LCD_Init();
	KEY_Init();							//��ʼ������
	BEEP_Init();						//��ʼ��������
	SHT3X_Init();           //��ʼ��I2C��ʪ��ģ��
	my_mem_init(SRAMIN);            	//��ʼ���ڲ��ڴ��
	STMFLASH_Read(FLASH_SAVE_ADDR,DEV_ID,2);
	
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
		//����3��ֵ�ź���
	BinarySemaphore_uart3=xSemaphoreCreateBinary();
	
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
	
    //����UART1����
    xTaskCreate((TaskFunction_t )UART1_task,
                (const char*    )"UART1_task",   
                (uint16_t       )UART1_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )UART1_TASK_PRIO,
                (TaskHandle_t*  )&UART1Task_Handler); 
		//����UART2���񣬼��PM2.5
    xTaskCreate((TaskFunction_t )UART2_task,             
                (const char*    )"UART2_task",           
                (uint16_t       )UART2_STK_SIZE,        
                (void*          )NULL,                  
                (UBaseType_t    )UART2_TASK_PRIO,        
                (TaskHandle_t*  )&UART2Task_Handler); 
    //����UART3���񣬴�����Ļ
    xTaskCreate((TaskFunction_t )UART3_task,             
                (const char*    )"UART3_task",           
                (uint16_t       )UART3_STK_SIZE,        
                (void*          )NULL,                  
                (UBaseType_t    )UART3_TASK_PRIO,        
                (TaskHandle_t*  )&UART3Task_Handler);   								
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
	
	Send_data();
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

//UART3_task����
void UART3_task(void *pvParameters)
{
	u8 len=0;
	BaseType_t err=pdFALSE;
	u8 CommandStr[40];
	while(1)
	{
		err=xSemaphoreTake(BinarySemaphore_uart3,portMAX_DELAY);	//��ȡ�ź���
		if(err==pdTRUE)										//��ȡ�ź����ɹ�
		{
			len=USART3_RX_STA&0x3fff;						//�õ��˴ν��յ������ݳ���

			mymemcpy(CommandStr,USART3_RX_BUF,len);
			CommandStr[len]=0x0D;
			CommandStr[len+1]=0x0A;
			
			//CommandProcess(CommandStr,len+2);		//�������
			myuart_send(3,CommandStr,len+2);
			USART3_RX_STA=0;
			memset(USART3_RX_BUF,0,USART_REC_LEN);			//���ڽ��ջ���������
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
			#ifdef __DEBUG
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
