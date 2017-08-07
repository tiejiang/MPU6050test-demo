#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "lcd.h"
#include "usart.h"	 
#include "dma.h"

//#define SEND_BUF_SIZE 8200	//发送数据长度,最好等于sizeof(TEXT_TO_SEND)+2的整数倍.
#define SEND_BUF_SIZE 44

u8 SendBuff[SEND_BUF_SIZE];	//发送数据缓冲区
//const u8 TEXT_TO_SEND[]={"ALIENTEK WarShip STM32F1 DMA 串口实验"};
 int main(void)
 {	 
	u16 i;
	u8 t=0;
	u8 j,mask=0;
	float pro=0;//进度

	delay_init();	    	 //延时函数初始化	  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	uart_init(115200);	 	//串口初始化为115200
	LED_Init();		  		//初始化与LED连接的硬件接口
	//LCD_Init();			   	//初始化LCD 	
	//KEY_Init();				//按键初始化		 	
	 //DMA1通道4,外设为串口1,存储器为SendBuff,长度SEND_BUF_SIZE.
 	//MYDMA_Config(DMA1_Channel4,(u32)&USART1->DR,(u32)SendBuff,SEND_BUF_SIZE);
	 MYDMA_Config(DMA1_Channel5,(u32)&USART1->DR,(u32)SendBuff,SEND_BUF_SIZE);
	 	    
	 USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE); //使能串口1的DMA接收
/*	
	 
 	POINT_COLOR=RED;//设置字体为红色 
	LCD_ShowString(30,50,200,16,16,"WarShip STM32");	
	LCD_ShowString(30,70,200,16,16,"DMA TEST");	
	LCD_ShowString(30,90,200,16,16,"ATOM@ALIENTEK");
	LCD_ShowString(30,110,200,16,16,"2015/1/15");	
 	LCD_ShowString(30,130,200,16,16,"KEY0:Start");
	//显示提示信息	
	j=sizeof(TEXT_TO_SEND);	   
	for(i=0;i<SEND_BUF_SIZE;i++)//填充数据到SendBuff
    {
		if(t>=j)//加入换行符
		{
			if(mask)
			{
				SendBuff[i]=0x0a;
				t=0;
			}else 
			{
				SendBuff[i]=0x0d;
				mask++;
			}	
		}else//复制TEXT_TO_SEND语句
		{
			mask=0;
			SendBuff[i]=TEXT_TO_SEND[t];
			t++;
		}    	   
    }		 
	POINT_COLOR=BLUE;//设置字体为蓝色	
*/  
	i=0;
	while(1)
	{
		 printf("\r\n DMA DATA Receive Begin\r\n");
//		t=KEY_Scan(0); 
//		if(t==KEY0_PRES)//KEY0按下   PA_4
//		{
//			LCD_ShowString(30,150,200,16,16,"Start Transimit....");
//			LCD_ShowString(30,170,200,16,16,"   %");//显示百分号
			     
			//MYDMA_Enable(DMA1_Channel4);//开始一次DMA传输！	  
			  
		    //等待DMA传输完成
		    //实际应用中，传输数据期间，可以执行另外的任务
//		    while(1)
//		    {
//					if(DMA_GetFlagStatus(DMA1_FLAG_TC4)!=RESET)	//判断通道4传输完成
//					{
//						//DMA_ClearFlag(DMA1_FLAG_TC4);//清除通道4传输完成标志
//						DMA_ClearFlag(DMA1_FLAG_TC5);//清除通道4传输完成标志
//						break; 
//					}
//					//pro=DMA_GetCurrDataCounter(DMA1_Channel4);//得到当前还剩余多少个数据
//					pro=DMA_GetCurrDataCounter(DMA1_Channel5);//得到当前还剩余多少个数据
//					pro=1-pro/SEND_BUF_SIZE;//得到百分比	  
//					pro*=100;      //扩大100倍
//					//LCD_ShowNum(30,170,pro,3,16);	  
//					printf("传输百分比： %f\n", pro);
//		    }			    
//			LCD_ShowNum(30,170,100,3,16);//显示100%	  
//			LCD_ShowString(30,150,200,16,16,"Transimit Finished!");//提示传送完成
//		}
		i++;
		delay_ms(10);
		if(i==20)
		{
			LED0=!LED0;//提示系统正在运行	
			i=0;
		}		   
	}
}
