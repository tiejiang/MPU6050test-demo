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

u16 DMA1_MEM_LEN;//����DMAÿ�����ݴ��͵ĳ��� 	    
//DMA1�ĸ�ͨ������
//����Ĵ�����ʽ�ǹ̶���,���Ҫ���ݲ�ͬ��������޸�
//�Ӵ洢��->����ģʽ/8λ���ݿ��/�洢������ģʽ
//DMA_CHx:DMAͨ��CHx
//cpar:�����ַ
//cmar:�洢����ַ
//cndtr:���ݴ����� 
void MYDMA_Config(DMA_Channel_TypeDef* DMA_CHx,u32 cpar,u32 cmar,u16 cndtr)
{
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//ʹ��DMA����
	
  DMA_DeInit(DMA_CHx);   //��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ

	DMA1_MEM_LEN=cndtr;
	DMA_InitStructure.DMA_PeripheralBaseAddr = cpar;  //DMA�������ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = cmar;  //DMA�ڴ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //���ݴ��䷽�򣬴��ڴ��ȡ���͵�����
	DMA_InitStructure.DMA_BufferSize = cndtr;  //DMAͨ����DMA����Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //�����ַ�Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ��ַ�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //����������ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMAͨ�� xӵ�������ȼ� 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
	DMA_Init(DMA_CHx, &DMA_InitStructure);  //����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��USART1_Tx_DMA_Channel����ʶ�ļĴ���
	  	
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
		imuRecvBufferLen = IMU_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Channel5); //ʣ��ռ��С=�洢�ռ�-�ռ��л�δ����������
		//�������������ͬ�Ļ��� ��imuRxBuffer0  ��  imuRxBuffer1
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
		  DMA1_Channel5->CNDTR = IMU_RX_BUFFER_SIZE;  //����������
		  DMA1_Channel5->CMAR = (u32)imuRxBuffer0;		//�洢��
			imuCurBufferIndex = 0;
		}
		DMA_Cmd(DMA1_Channel5, ENABLE);  //ʹ��DMA1ͨ��5 ��������
		printf("GET DATA AGAIN \n");
		// 22 Ϊ���ٶȡ����ٶȡ��Ƕ����ݰ������ܳ���		
			if ((22 == imuRecvBufferLen) && ('S' == *(imuData+0)) && 'E' == *(imuData + 21))	
		{
			imu_package++;
			newImuData = 1;
			//gAcc[0] = (*(imuData + 3) << 8) + *(imuData + 2);
			//gAcc[1] = (*(imuData + 5) << 8) + *(imuData + 4);
			//gAcc[2] = (*(imuData + 7) << 8) + *(imuData + 6);
			start_flag = *imuData;
		
			angleSpeed[0] = (*(imuData + 2) << 8) + *(imuData + 1);   //���X��ߵ��ֽ�
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
			printf("X-���ٶ�:  %d\r\n", angleSpeed[0]);
			printf("Y-���ٶ�:  %d\r\n", angleSpeed[1]);
			printf("Z-���ٶ�:  %d\r\n", angleSpeed[2]);
			printf("X-�Ƕ�:  %d\r\n", angle[0]);
			printf("Y-�Ƕ�:  %d\r\n", angle[1]);
			//printf("Z-�Ƕ�:  %d\r\n", angle[2]);
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
	if(USART_GetITStatus(USART1, USART_IT_PE | USART_IT_FE | USART_IT_NE) != RESET)//������
	{
		printf("USART ClearITPendingBit");
		USART_ClearITPendingBit(USART1, USART_IT_PE | USART_IT_FE | USART_IT_NE);
	}
		
}
 

























