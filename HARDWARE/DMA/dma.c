#include "dma.h"
#include "delay.h"
#define IMU_BAUD_RATE				115200
#define IMU_RX_BUFFER_SIZE	44
#define IMU_RX_PROTOCOL_LEN	11

#define DEBUG

u8 imuCurBufferIndex = 0;
u8 imuRxBuffer0[IMU_RX_BUFFER_SIZE] = {0};
u8 imuRxBuffer1[IMU_RX_BUFFER_SIZE] = {0};

u32 temp_x_axis_anglespeed;


volatile u8 newImuData = 0;

s16 gAcc[3] = {0};
s16 angleSpeed[3] = {0};
s16 angle[3] = {0};
s16 tempture = 0;
u8 start_flag;
u8 end_flag;
u16 one_flag;
u16 check_sum;

u32 imu_package = 0;
u32 imu_error = 0;

DMA_InitTypeDef DMA_InitStructure;

u16 DMA1_MEM_LEN;//保存DMA每次数据传送的长度 	    
//DMA1的各通道配置
//这里的传输形式是固定的,这点要根据不同的情况来修改
//从存储器->外设模式/8位数据宽度/存储器增量模式
//DMA_CHx:DMA通道CHx
//cpar:外设地址
//cmar:存储器地址
//cndtr:数据传输量 
void MYDMA_Config(DMA_Channel_TypeDef* DMA_CHx,u32 cpar,u32 cmar,u16 cndtr)
{
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//使能DMA传输
	
  DMA_DeInit(DMA_CHx);   //将DMA的通道1寄存器重设为缺省值

	DMA1_MEM_LEN=cndtr;
	DMA_InitStructure.DMA_PeripheralBaseAddr = cpar;  //DMA外设基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = cmar;  //DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //数据传输方向，从内存读取发送到外设
	DMA_InitStructure.DMA_BufferSize = cndtr;  //DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //数据宽度为8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //数据宽度为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //工作在正常模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMA通道 x拥有中优先级 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA_CHx, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道USART1_Tx_DMA_Channel所标识的寄存器
	  	
} 

void USART1_IRQHandler(void)        
{
	s16 temp;
	u8 imuRecvBufferLen = 0;
	u8 *imuData;
	//printf("GO TO INTERUPT \n");
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		//printf("GO TO INTERUPT \n");
		//clear interrupt
		USART1->SR;
		USART1->DR;
		DMA_Cmd(DMA1_Channel5,DISABLE);
		imuRecvBufferLen = IMU_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Channel5); //剩余空间大小=存储空间-空间中还未传输数据量
		//交替进入两个不同的缓存 ：imuRxBuffer0  和  imuRxBuffer1
		if (0 == imuCurBufferIndex)
		{
			imuData = imuRxBuffer0;
		  DMA1_Channel5->CNDTR = IMU_RX_BUFFER_SIZE;
		  DMA1_Channel5->CMAR = (u32)imuRxBuffer1;
			imuCurBufferIndex = 1;
		}
		else
		{
			imuData = imuRxBuffer1;
		  DMA1_Channel5->CNDTR = IMU_RX_BUFFER_SIZE;  //传输数据量
		  DMA1_Channel5->CMAR = (u32)imuRxBuffer0;		//存储器
			imuCurBufferIndex = 0;
		}
		DMA_Cmd(DMA1_Channel5, ENABLE);  //使能DMA1通道5 接收数据
		printf("GET DATA AGAIN \n");
		// 22 为加速度、角速度、角度数据包数据总长度		
			if ((22 == imuRecvBufferLen) && ('S' == *(imuData+0)) && 'E' == *(imuData + 21))	
		{
			imu_package++;
			newImuData = 1;
			//gAcc[0] = (*(imuData + 3) << 8) + *(imuData + 2);
			//gAcc[1] = (*(imuData + 5) << 8) + *(imuData + 4);
			//gAcc[2] = (*(imuData + 7) << 8) + *(imuData + 6);
			start_flag = *imuData;
		
			angleSpeed[0] = (*(imuData + 2) << 8) + *(imuData + 1);   //组合X轴高低字节
			angleSpeed[1] = (*(imuData + 4) << 8) + *(imuData + 3);   //Y~
			angleSpeed[2] = (*(imuData + 6) << 8) + *(imuData + 5);     //Z~
			temp = (*(imuData + 14) << 8) + *(imuData + 13);
			angle[0] = temp / 182;
			//angle[0] = temp;
			temp = (*(imuData + 16) << 8) + *(imuData + 15);
			angle[1] = temp / 182;
			//angle[1] = temp;
			//temp = (*(imuData + 18) << 8) + *(imuData + 17);
			//angle[2] = temp / 182;
			
			one_flag = *(imuData + 19);
			check_sum = *(imuData + 20);
			end_flag = *(imuData + 21);
			
			#ifdef DEBUG
			printf("TEST OUTPUT\r\n");
			printf("TEST start_flag: %c\r\n", start_flag);
			printf("X-角速度:  %d\r\n", angleSpeed[0]);
			printf("Y-角速度:  %d\r\n", angleSpeed[1]);
			printf("Z-角速度:  %d\r\n", angleSpeed[2]);
			printf("X-角度:  %d\r\n", angle[0]);
			printf("Y-角度:  %d\r\n", angle[1]);
			//printf("Z-角度:  %d\r\n", angle[2]);
			printf("TEST one_flag: %d\r\n", one_flag);
			printf("TEST check_sum: %d\r\n", check_sum);
			printf("TEST end_flag: %c\r\n", end_flag);
			//tempture = (*(imuData + 31) << 8) + *(imuData + 30); 
			
			
			printf("temp_x_axis_anglespeed : %d\n", temp_x_axis_anglespeed);
			printf("angleSpeed : %d\n", angleSpeed[0]);
			if(abs(temp_x_axis_anglespeed-angleSpeed[0])>400){
				printf("knock!!! \n");
			}else{
				printf("safe status  \n");
			}
			
			DMA_Cmd(DMA1_Channel5,DISABLE);
			
			temp_x_axis_anglespeed = angleSpeed[0];
			printf("temp_x_axis_anglespeed_second : %d\n", temp_x_axis_anglespeed);
			
			delay_ms(300);
			#endif
			
		}
		
		else
			imu_error++;
	}
	if(USART_GetITStatus(USART1, USART_IT_PE | USART_IT_FE | USART_IT_NE) != RESET)//出错处理
	{
		printf("USART ClearITPendingBit");
		USART_ClearITPendingBit(USART1, USART_IT_PE | USART_IT_FE | USART_IT_NE);
	}
		
}
 

























