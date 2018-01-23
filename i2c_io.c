
#include "SI32L0xx_conf.h"
#include "stdio.h"
#include "math.h"


////////////////////////////////////////////////////
#define DEVICEADDR    (0x77)

#define PORT_SDA      (PORT_3)
#define PORT_SCL      (PORT_3)

#define PIN_SDA       (PORT_Pin_2)
#define PIN_SCL       (PORT_Pin_3)

#define SDA_H         (((PORT_SDA)->Pn) |= (PIN_SDA))
#define SDA_L         (((PORT_SDA)->Pn) &= ~(PIN_SDA))

#define SCL_H         (((PORT_SCL)->Pn) |= (PIN_SCL)) 
#define SCL_L         (((PORT_SCL)->Pn) &= ~(PIN_SCL))

#define SDA_I         (((PORT_SDA)->Pn_DIR) |= (PIN_SDA))
#define SDA_O         (((PORT_SDA)->Pn_DIR) &= ~(PIN_SDA))

#define SDA_READ      ((((PORT_SDA)->Pn) & (PIN_SDA)) ? 1 : 0) 
//////////////////////////////////////////////////////


void DelayMs(uint16_t ms)
{
	uint16_t i, j;
	
	for (i = 0; i < ms; i++)
	{
		for (j = 0; j < 2500; j++);
	}
}

void LSE32K_Trim_HSI16MIRC(void)
{
	uint8_t i, k = 1;
	
	// IRC 16M test
	PORT_0->Pn_SEL0 |= ( (0x1 << 14) | (0x1 << 15) );
	XLF_CTRL = 0x3F;
	IRC16M_CTRL = 0xE0;
	SYSCLK0 |= 0x8;
	
	XLF_CTRL &= ~0x40;
	while (!(XLF_CTRL & 0x40));
	
	for (i = 0; i < 7; i++)
	{
		IRC16M_CTRL &= ~0x100;
		while (!(IRC16M_CTRL & 0x100));
		
		CLKCAL_CTRL = 0x0;
		REFCNT = 200;
		CALCNT = 0;
		CLKCAL_CTRL = 0x7;
		
		while (!(CLKCAL_CTRL & 0x40));
		
		if (CALCNT < 101250)
		{
			if (k < 6)
			{
				IRC16M_CTRL = IRC16M_CTRL + (0x20 >> k++);
			}
			else
			{
				IRC16M_CTRL = IRC16M_CTRL + 1;
			}
		}
		else
		{
			if (k < 6)
			{
				IRC16M_CTRL = IRC16M_CTRL - (0x20 >> k++);
			}
			else
			{
				IRC16M_CTRL = IRC16M_CTRL - 1;
			}
		}
	}
	
	IRC16M_CTRL &= ~0x100;
	while (!(IRC16M_CTRL & 0x100));
}

void I2C_Delay(void)
{ 
	uint8_t i = 10;
	
	while(i--);
}
 
int I2C_Start(void)  
{  
	SDA_O,SDA_H;  
	I2C_Delay();
	
	SCL_H;  
	I2C_Delay();
	
	SDA_I;
	if (!SDA_READ) 
		return -1;
    
	SDA_O,SDA_L;  
	I2C_Delay();

	SDA_I;		
	if (!SDA_READ)   
		return -1;
		
	return 0;
}  
  
void I2C_Stop(void)  
{  
	SCL_L;  
	I2C_Delay();
  
	SDA_O,SDA_L;  
	I2C_Delay();
  
	SCL_H;  
	I2C_Delay();
  
	SDA_O,SDA_H;  
	I2C_Delay();  
}  
 
void I2C_Ack(void)  
{     
	SCL_L;  
	I2C_Delay(); 
	
	SDA_O,SDA_L;  
	I2C_Delay(); 
	
	SCL_H;  
	I2C_Delay(); 
	
	SCL_L;  
	I2C_Delay();  
}  
 
void I2C_NoAck(void)  
{     
	SCL_L;  
	I2C_Delay(); 
	
	SDA_O,SDA_H;  
	I2C_Delay();
  
	SCL_H;  
	I2C_Delay(); 
	
	SCL_L;  
	I2C_Delay();  
}  

int I2C_WaitAck(void)     
{  
	SCL_L;  
	I2C_Delay();
  
	SDA_O,SDA_H;
	I2C_Delay();
  
	SCL_H;  
	I2C_Delay();

	SDA_I; 
	if (SDA_READ)  
	{  
		SCL_L;
		I2C_Delay();
		
		return -1;  
	} 
		
	SCL_L;
	I2C_Delay();
		
	return 0;  
} 

void I2C_SendByte(u8 SendByte)   
{  
  u8 i; 	
	
	SDA_O;
	for (i = 0; i < 8; i++)  
	{ 
		SCL_L;  
		I2C_Delay();
						
		if (SendByte & 0x80)
		{					
			SDA_H; 
		}					
		else
		{					
			SDA_L;
		}				
		SendByte <<= 1;  
		I2C_Delay();
					
		SCL_H;  
		I2C_Delay();  
	} 
		
	SCL_L;
	I2C_Delay();
		
	SDA_O,SDA_H; 
	I2C_Delay(); 		
}  
 
uint8_t I2C_ReceiveByte(void)    
{   
	u8 i;  
	u8 ReceiveByte;  
     
	SCL_L;
	I2C_Delay();
	
	SDA_O,SDA_H;
	I2C_Delay();
	
	SDA_I;
	for (i = 0; i < 8; i++)  
	{
		SCL_H;
		I2C_Delay();
			
		ReceiveByte = (ReceiveByte << 1) | SDA_READ; 
			
		SCL_L;  
		I2C_Delay();				
	}  
 
	return ReceiveByte;  
}

int I2C_WriteByte(u8 SendByte, u16 WriteAddress, u8 DeviceAddress)  
{         
	if (I2C_Start() == -1)
	{		
		I2C_Stop(); 
		return -1;
	}
		
	I2C_SendByte(DeviceAddress & 0xFFFE); 
	if (I2C_WaitAck() == -1)  
	{  
		I2C_Stop();   
		return -1;  
	} 
		
	I2C_SendByte((u8)(WriteAddress & 0x00FF));          
	if (I2C_WaitAck() == -1)
	{
		I2C_Stop(); 
		return -1;
	}
		
	I2C_SendByte(SendByte);  
	if (I2C_WaitAck() == -1)
	{
		I2C_Stop(); 
		return -1;
	}
		
	I2C_Stop();
	
	return 0;  
} 

int I2C_ReadByte(u8* pBuffer, u8 length, u16 ReadAddress, u8 DeviceAddress)  
{         
	if (I2C_Start() == -1)
	{		
		I2C_Stop(); 
		return -1; 
	}
		
	I2C_SendByte(DeviceAddress & 0xFFFE);
	if (I2C_WaitAck() == -1)
	{
		I2C_Stop(); 
		return -1;
	}
		
	I2C_SendByte((u8)(ReadAddress & 0x00FF));       
	if (I2C_WaitAck() == -1)
	{
		I2C_Stop(); 
		return -1;
	}
		
	if (I2C_Start() == -1)
	{
		I2C_Stop(); 
		return -1;
	}
		
	I2C_SendByte(DeviceAddress | 0x0001);  
	if (I2C_WaitAck() == -1)
	{
		I2C_Stop(); 
		return -1;
	}
		
	while (length)  
	{  
		*pBuffer = I2C_ReceiveByte();
		if (length == 1)
			I2C_NoAck();  
		else 
			I2C_Ack();   
		pBuffer++;  
		length--;  
	} 
		
	I2C_Stop();
		
	return 0;  
}

void uf_GPIO_I2C_Init(void)
{
	PORT_InitTypeDef PORT_InitStruct;
	
	CMU_APBPeriph1ClockCmd(CMU_APBPeriph1_PORT, ENABLE);
	
	PORT_StructInit(&PORT_InitStruct);
	PORT_InitStruct.PORT_Pin = PIN_SDA; // SDA
	PORT_InitStruct.PORT_Properity = PORT_Properity_Digital;
	PORT_InitStruct.PORT_Mode = PORT_Mode_OUT;	
	PORT_InitStruct.PORT_OutType = PORT_OutType_OD;
	PORT_InitStruct.PORT_PullHigh = PORT_PH_PullHigh;
	PORT_InitStruct.PORT_DriveSink = PORT_DS_DriveSinkNormal;	
	PORT_Init(PORT_SDA, &PORT_InitStruct);

	PORT_StructInit(&PORT_InitStruct);
	PORT_InitStruct.PORT_Pin = PIN_SCL; //SCL
	PORT_InitStruct.PORT_Properity = PORT_Properity_Digital;
	PORT_InitStruct.PORT_Mode = PORT_Mode_OUT;
	PORT_InitStruct.PORT_OutType = PORT_OutType_OD;
	PORT_InitStruct.PORT_PullHigh = PORT_PH_PullHigh;
	PORT_InitStruct.PORT_DriveSink = PORT_DS_DriveSinkNormal;	
	PORT_Init(PORT_SCL, &PORT_InitStruct);
	
	PORT_WriteBit(PORT_SDA, PIN_SDA, Bit_SET);
	PORT_WriteBit(PORT_SCL, PIN_SCL, Bit_SET);	
}

void uf_UART_Init(UART_TypeDef* UARTx)
{
	UART_InitTypeDef UART_InitStruct;
	
	PORT_InitTypeDef PORT_InitStruct;
	
	CMU_APBPeriph1ClockCmd(CMU_APBPeriph1_PORT, ENABLE);
	PORT_InitStruct.PORT_Pin = PORT_Pin_6; // RX = P1.6
	PORT_InitStruct.PORT_Properity = PORT_Properity_Digital;
	PORT_InitStruct.PORT_Mode = PORT_Mode_IN;
	PORT_InitStruct.PORT_OutType = PORT_OutType_OD;
	PORT_InitStruct.PORT_PullHigh = PORT_PH_PullHigh;
	PORT_InitStruct.PORT_DriveSink = PORT_DS_DriveSinkNormal;	
	PORT_Init(PORT_1, &PORT_InitStruct);	
	PORT_InitStruct.PORT_Pin = PORT_Pin_7; // TX = P1.7
	PORT_InitStruct.PORT_Properity = PORT_Properity_Digital;
	PORT_InitStruct.PORT_Mode = PORT_Mode_OUT;
	PORT_InitStruct.PORT_OutType = PORT_OutType_PP;
	PORT_InitStruct.PORT_PullHigh = PORT_PH_NoPullHigh;
	PORT_InitStruct.PORT_DriveSink = PORT_DS_DriveSinkNormal;	
	PORT_Init(PORT_1, &PORT_InitStruct);
	// Port AF
	PORT_PinAFConfig(PORT_1, PORT_PinSource6, PORT_AF_1); 
	PORT_PinAFConfig(PORT_1, PORT_PinSource7, PORT_AF_1);	

	CMU_APBPeriph0ClockCmd(CMU_APBPeriph0_UART1, ENABLE);
	UART_DeInit(UART_1);	
	UART_InitStruct.UART_BaudRate = 9600;
	UART_InitStruct.UART_DataLength = UART_DataLength_8Bits;
	UART_InitStruct.UART_StopBits = UART_StopBits_1Bit;
	UART_InitStruct.UART_Parity = UART_Parity_None;
	UART_InitStruct.UART_HardwareFlowControl = UART_HardwareFlowControl_None; 	
	UART_Init(UARTx, &UART_InitStruct);

	// Config FIFO
	UART_FIFOModeConfig(UARTx, UART_TXFIFOThreshValue_2_32FULL, UART_RXFIFOThreshValue_30_32FULL, ENABLE);
	UART_PTXREModeConfig(UARTx, ENABLE);
	
	UART_ITConfig(UARTx, ( UART_IT_TXREmptyIE | UART_IT_ReceiveData_RXFIFOTimeOutIE ), DISABLE);
}

int fputc(int ch, FILE *stream)
{
	while (UART_GetLineStatus(UART_1, UART_LSFLAG_TXREmpty_TXFIFOFull) == SET);	
	UART_SendData(UART_1, ch);	
	
	return ch;
}

void NVIC_Init(void)
{
	__disable_irq();
	
	NVIC_DisableIRQ(UART1_IRQn);
	NVIC_ClearPendingIRQ(UART1_IRQn);
	NVIC_SetPriority(UART1_IRQn, 0x0);
	//NVIC_EnableIRQ(UART1_IRQn);	
	
	__enable_irq();
}

int main(void)
{	
	uint8_t Buffer[22];
	
	s32 UT, UP;
	s16 AC1,AC2,AC3;
	u16 AC4,AC5,AC6;
	s16 B1,B2,MB,MC,MD;
	s32 X1,X2,X3;
	u32 B4;
	s32 B3,B5,B6,B7;
	s32 T, P;	
	float Temperature;
	float Pressure;
	uint8_t OSS = 0x3;

	LSE32K_Trim_HSI16MIRC();
	
	uf_GPIO_I2C_Init();	
	
	NVIC_Init();

	SystemCoreClockUpdate();
	uf_UART_Init(UART_1);
	
	printf("SystemCoreClock = %d...\n\r", SystemCoreClock);
	
	while (1)
	{
			// Read adjust data from BMP180
			/*
			while (I2C_ReadByte(Buffer, 2, 0xAA, (DEVICEADDR << 1)) == -1);
			AC1 = (Buffer[0] << 8) | Buffer[1];
			while (I2C_ReadByte(Buffer, 2, 0xAC, (DEVICEADDR << 1)) == -1);
			AC2 = (Buffer[0] << 8) | Buffer[1];
			while (I2C_ReadByte(Buffer, 2, 0xAE, (DEVICEADDR << 1)) == -1);
			AC3 = (Buffer[0] << 8) | Buffer[1];
			while (I2C_ReadByte(Buffer, 2, 0xB0, (DEVICEADDR << 1)) == -1);
			AC4 = (Buffer[0] << 8) | Buffer[1];
			while (I2C_ReadByte(Buffer, 2, 0xB2, (DEVICEADDR << 1)) == -1);
			AC5 = (Buffer[0] << 8) | Buffer[1];
			while (I2C_ReadByte(Buffer, 2, 0xB4, (DEVICEADDR << 1)) == -1);
			AC6 = (Buffer[0] << 8) | Buffer[1];
			while (I2C_ReadByte(Buffer, 2, 0xB6, (DEVICEADDR << 1)) == -1);
			B1 = (Buffer[0] << 8) | Buffer[1];
			while (I2C_ReadByte(Buffer, 2, 0xB8, (DEVICEADDR << 1)) == -1);
			B2 = (Buffer[0] << 8) | Buffer[1];
			while (I2C_ReadByte(Buffer, 2, 0xBA, (DEVICEADDR << 1)) == -1);
			MB = (Buffer[0] << 8) | Buffer[1];
			while (I2C_ReadByte(Buffer, 2, 0xBC, (DEVICEADDR << 1)) == -1);
			MC = (Buffer[0] << 8) | Buffer[1];
			while (I2C_ReadByte(Buffer, 2, 0xBE, (DEVICEADDR << 1)) == -1);
			MD = (Buffer[0] << 8) | Buffer[1];
			*/
			
			while (I2C_ReadByte(Buffer, 22, 0xAA, (DEVICEADDR << 1)) == -1);
			AC1 = (Buffer[0] << 8) | Buffer[1];
			AC2 = (Buffer[2] << 8) | Buffer[3];
			AC3 = (Buffer[4] << 8) | Buffer[5];
			AC4 = (Buffer[6] << 8) | Buffer[7];
			AC5 = (Buffer[8] << 8) | Buffer[9];
			AC6 = (Buffer[10] << 8) | Buffer[11];
			B1 = (Buffer[12] << 8) | Buffer[13];
			B2 = (Buffer[14] << 8) | Buffer[15];
			MB = (Buffer[16] << 8) | Buffer[17];
			MC = (Buffer[18] << 8) | Buffer[19];			
			MD = (Buffer[20] << 8) | Buffer[21];
			
			printf("BMP180 adjust data:\n\r");
			printf("AC1 = 0x%X...\n\r", AC1);	
			printf("AC2 = 0x%X...\n\r", AC2);
			printf("AC3 = 0x%X...\n\r", AC3);
			printf("AC4 = 0x%X...\n\r", AC4);
			printf("AC5 = 0x%X...\n\r", AC5);
			printf("AC6 = 0x%X...\n\r", AC6);
			printf("B1 = 0x%X...\n\r", B1);
			printf("B2 = 0x%X...\n\r", B2);
			printf("MB = 0x%X...\n\r", MB);
			printf("MC = 0x%X...\n\r", MC);
			printf("MD = 0x%X...\n\r", MD);	
			
			// Test temperature
			while (I2C_WriteByte(0x2E, 0xF4, (DEVICEADDR << 1)) == -1);
			DelayMs(5);
			while (I2C_ReadByte(Buffer, 2, 0xF6, (DEVICEADDR << 1)) == -1);
			UT = (Buffer[0] << 8) | Buffer[1];
			X1 = ((UT - AC6) * AC5) >> 15;
			X2 = (MC << 11 ) / (X1 + MD);
			B5 = X1 + X2;
			T = (B5 + 8) >> 4;
			Temperature = T * 0.1;	
			printf("Temperatue of BMP180 = %.1f oC...\n\r", Temperature);

			// Test pressure
			while (I2C_WriteByte(0xF4 + (OSS << 6), 0xF4, (DEVICEADDR << 1)) == -1);	
			DelayMs(30);
			while (I2C_ReadByte(Buffer, 3, 0xF6, (DEVICEADDR << 1)) == -1);
			UP = (Buffer[0] << 16) | (Buffer[1] << 8) | Buffer[2];
			UP = UP >> (8 - OSS);
			B6 = B5 - 4000;
			X1 = ( B2 * ( B6 * B6 >> 12 ) ) >> 11;
			X2 = AC2 * B6 >> 11;
			X3 = X1 + X2;
			B3 = ( ( ( (AC1 << 2 ) + X3 ) << OSS ) + 2 ) >> 2;
			X1 = AC3 * B6 >> 13;
			X2 = ( B1 * ( B6 * B6 >> 12 ) ) >> 16;
			X3 = ( ( X1 + X2 ) + 2 ) >> 2;
			B4 = AC4 * (u32)( X3 + 32768 ) >> 15;
			B7 = ( (u32)UP - B3 ) * (50000 >> OSS);
			if (B7 < 0x80000000)
			{
				P = (B7 << 1) / B4;
			}
			else
			{
				P = (B7 / B4) << 1;
			}
			X1 = (P >> 8) * (P >> 8);
			X1 = (X1 * 3038) >> 16;
			X2 = (-7357 * P) >> 16;
			P = P + ( (X1 + X2 + 3791) >> 4 );
			Pressure = P / 100.0;
			printf("Pressure of BMP180 = %.1f hPa...\n\r", Pressure);	

			DelayMs(3000);
	}
}
