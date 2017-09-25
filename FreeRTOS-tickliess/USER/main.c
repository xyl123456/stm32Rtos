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
 ALIENTEK 战舰STM32F103开发板 FreeRTOS实验18-1
 FreeRTOS低功耗Tickless模式实验-库函数版本
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/
//任务定义************************************************/
//任务优先级
#define START_TASK_PRIO		1
//任务堆栈大小	
#define START_STK_SIZE 		256  
//任务句柄
TaskHandle_t StartTask_Handler;
//任务函数
void start_task(void *pvParameters);

/*串口1任务*/
//任务优先级
#define UART1_TASK_PRIO 2
//任务堆栈大小	
#define UART1_STK_SIZE  256 
//任务句柄
TaskHandle_t UART1Task_Handler;
//任务函数
void UART1_task(void *pvParameters);

/*串口2任务*/
//任务优先级
#define UART2_TASK_PRIO		3
//任务堆栈大小	
#define UART2_STK_SIZE 		256  
//任务句柄
TaskHandle_t UART2Task_Handler;
//任务函数
void UART2_task(void *pvParameters);

/*串口3任务*/
//任务优先级
#define UART3_TASK_PRIO		4
//任务堆栈大小	
#define UART3_STK_SIZE 		256  
//任务句柄
TaskHandle_t UART3Task_Handler;
//任务函数
void UART3_task(void *pvParameters);

/*按键处理任务*/
//任务优先级，
#define KEY_TASK_PRIO		5
//任务堆栈大小	
#define KEY_STK_SIZE 		256  
//任务句柄
TaskHandle_t KEYTask_Handler;
//任务函数
void KEY_task(void *pvParameters);


void PreSleepProcessing(uint32_t ulExpectedIdleTime);
void PostSleepProcessing(uint32_t ulExpectedIdleTime);

void LowerToCap(u8 *str,u8 len);
//二值信号量句柄
SemaphoreHandle_t BinarySemaphore;	//二值信号量句柄
SemaphoreHandle_t BinarySemaphore_uart2;	//串口2二值信号量句柄
SemaphoreHandle_t BinarySemaphore_uart3;	//串口3二值信号量句柄

TimerHandle_t 	AutoReloadTimer_Handle;			//周期定时器句柄
TimerHandle_t	OneShotTimer_Handle;			//单次定时器句柄

void AutoReloadCallback(TimerHandle_t xTimer); 	//周期定时器回调函数
void OneShotCallback(TimerHandle_t xTimer);		//单次定时器回调函数

//用于命令解析用的命令类型
#define SYSTEM	  0x01    
#define PM25			0x02
#define FAN1  		0x03
#define RFD   		0x04

#define COMMANDERR	0XFF

//进入低功耗模式前需要处理的事情
//ulExpectedIdleTime：低功耗模式运行时间
void PreSleepProcessing(uint32_t ulExpectedIdleTime)
{
	//关闭某些低功耗模式下不使用的外设时钟，此处只是演示性代码
/*
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,DISABLE);
	*/
}

//退出低功耗模式以后需要处理的事情
//ulExpectedIdleTime：低功耗模式运行时间
void PostSleepProcessing(uint32_t ulExpectedIdleTime)
{
	//退出低功耗模式以后打开那些被关闭的外设时钟，此处只是演示性代码
	/*
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE); 
*/	
}

//LCD刷屏时使用的颜色
int lcd_discolor[14]={	WHITE, BLACK, BLUE,  BRED,      
						GRED,  GBLUE, RED,   MAGENTA,       	 
						GREEN, CYAN,  YELLOW,BROWN, 			
						BRRED, GRAY };

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4	 
	delay_init();	    				//延时函数初始化	 
	myuart_init(1,115200);//初始化串口1
	myuart_init(2,9600);//初始化串口2
	myuart_init(3,115200);//初始化串口3
	LED_Init();		  					//初始化LED
//	LCD_Init();
	KEY_Init();							//初始化按键
	BEEP_Init();						//初始化蜂鸣器
	SHT3X_Init();           //初始化I2C温湿度模块
	my_mem_init(SRAMIN);            	//初始化内部内存池
	STMFLASH_Read(FLASH_SAVE_ADDR,DEV_ID,2);
	
	//创建开始任务
    xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄              
    vTaskStartScheduler();          //开启任务调度
}

//开始任务任务函数
void start_task(void *pvParameters)
{
  taskENTER_CRITICAL();           //进入临界区
	//创建二值信号量
	BinarySemaphore=xSemaphoreCreateBinary();
	//串口2二值信号量
	BinarySemaphore_uart2=xSemaphoreCreateBinary();
		//串口3二值信号量
	BinarySemaphore_uart3=xSemaphoreCreateBinary();
	
	 //创建软件周期定时器
  AutoReloadTimer_Handle=xTimerCreate((const char*		)"AutoReloadTimer",
									    (TickType_t			)10000,
							            (UBaseType_t		)pdTRUE,
							            (void*				)1,
							            (TimerCallbackFunction_t)AutoReloadCallback); //周期定时器，周期1s(1000个时钟节拍)，周期模式
												
	/*												
    //创建单次定时器
	OneShotTimer_Handle=xTimerCreate((const char*			)"OneShotTimer",
							         (TickType_t			)2000,
							         (UBaseType_t			)pdFALSE,
							         (void*					)2,
							         (TimerCallbackFunction_t)OneShotCallback); //单次定时器，周期2s(2000个时钟节拍)，单次模式		
	*/										
		if(AutoReloadTimer_Handle!=NULL)
		{
			xTimerStart(AutoReloadTimer_Handle,0);	//开启周期定时器
		}
	
    //创建UART1任务
    xTaskCreate((TaskFunction_t )UART1_task,
                (const char*    )"UART1_task",   
                (uint16_t       )UART1_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )UART1_TASK_PRIO,
                (TaskHandle_t*  )&UART1Task_Handler); 
		//创建UART2任务，检测PM2.5
    xTaskCreate((TaskFunction_t )UART2_task,             
                (const char*    )"UART2_task",           
                (uint16_t       )UART2_STK_SIZE,        
                (void*          )NULL,                  
                (UBaseType_t    )UART2_TASK_PRIO,        
                (TaskHandle_t*  )&UART2Task_Handler); 
    //创建UART3任务，串口屏幕
    xTaskCreate((TaskFunction_t )UART3_task,             
                (const char*    )"UART3_task",           
                (uint16_t       )UART3_STK_SIZE,        
                (void*          )NULL,                  
                (UBaseType_t    )UART3_TASK_PRIO,        
                (TaskHandle_t*  )&UART3Task_Handler);   								
		//创建按键任务
    xTaskCreate((TaskFunction_t )KEY_task,     
                (const char*    )"KEY_task",   
                (uint16_t       )KEY_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )KEY_TASK_PRIO,
                (TaskHandle_t*  )&KEYTask_Handler); 
							
    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区						
}

//周期定时器的回调函数
void AutoReloadCallback(TimerHandle_t xTimer)
{
	
	Send_data();
}

//UART1_task函数
void UART1_task(void *pvParameters)
{
	u8 len=0;
	BaseType_t err=pdFALSE;
	u8 CommandStr[40];
	while(1)
	{
		err=xSemaphoreTake(BinarySemaphore,portMAX_DELAY);	//获取信号量
		if(err==pdTRUE)										//获取信号量成功
		{
			len=USART_RX_STA&0x3fff;						//得到此次接收到的数据长度

			mymemcpy(CommandStr,USART_RX_BUF,len);
			CommandStr[len]=0x0D;
			CommandStr[len+1]=0x0A;
			
			CommandProcess(CommandStr,len+2);		//命令解析
		
			USART_RX_STA=0;
			memset(USART_RX_BUF,0,USART_REC_LEN);			//串口接收缓冲区清零
			memset(CommandStr,0,40);	
		}
		 vTaskDelay(10); //延时10ms，也就是10个时钟节拍
	}
}

//UART2任务函数
void UART2_task(void *pvParameters)
{
	u8 len=0;
	BaseType_t err=pdFALSE;
	while(1)
	{
		err=xSemaphoreTake(BinarySemaphore_uart2,portMAX_DELAY);	//获取信号量
		if(err==pdTRUE)										//获取信号量成功
		{
			len=USART2_RX_STA&0x3fff;						//得到此次接收到的数据长度
			//CommandStr=mymalloc(SRAMIN,len);			myfree(SRAMIN,CommandStr); //释放内存	//申请内存
			mymemcpy(PM25_data.data_buf,USART2_RX_BUF,len);
			#ifdef __DEBUG
			myuart_send(2,PM25_data.data_buf,len);
			#endif
		
			USART2_RX_STA=0;
			memset(USART2_RX_BUF,0,USART_REC_LEN);			//串口接收缓冲区清零
		}
		 vTaskDelay(10); //延时10ms，也就是10个时钟节拍
	}
}

//UART3_task函数
void UART3_task(void *pvParameters)
{
	u8 len=0;
	BaseType_t err=pdFALSE;
	u8 CommandStr[40];
	while(1)
	{
		err=xSemaphoreTake(BinarySemaphore_uart3,portMAX_DELAY);	//获取信号量
		if(err==pdTRUE)										//获取信号量成功
		{
			len=USART3_RX_STA&0x3fff;						//得到此次接收到的数据长度

			mymemcpy(CommandStr,USART3_RX_BUF,len);
			CommandStr[len]=0x0D;
			CommandStr[len+1]=0x0A;
			
			//CommandProcess(CommandStr,len+2);		//命令解析
			myuart_send(3,CommandStr,len+2);
			USART3_RX_STA=0;
			memset(USART3_RX_BUF,0,USART_REC_LEN);			//串口接收缓冲区清零
			memset(CommandStr,0,40);	
		}
		 vTaskDelay(10); //延时10ms，也就是10个时钟节拍
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
				//开关机
			if(SYS_STA&0x80)
			{
				FAN_set(0x0B);//待机状态
			}else
			{
				FAN_set(0x11);//默认一档风
			}
			#ifdef __DEBUG
			myuart_send(1,&SYS_STA,1);
			#endif
				break;
			
			case KEY0_PRES:
				//设置模式
				break;
			
			case KEY1_PRES:
				//设置风量：高中低
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
				//其他待定
				break;
			
			default:
			break;					
		}
		 vTaskDelay(10); //延时10ms，也就是10个时钟节拍
	}
}
