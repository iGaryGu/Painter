#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_dcmi.h"
#include "stm32f4xx_dma.h"
#include "i2c_ops.h"
#include "ov7670.h"
#include "stm32f4xx.h"
#include "misc.h"
#include "ov7670_regsmap.h"

volatile int32_t dma_handler_counter = 0;

uint32_t camera_frame[80*60]; // for 320*240 resolution

void RCC_Configuration(void)
{
	/* --------------------------- System Clocks Configuration -----------    ------*/
	/* USART1 clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	/* GPIOA clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
}

void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/*-------------------------- GPIO Configuration ------------------------    ----*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Connect USART pins to AF */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);   // USART1_TX
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);  // USART1_RX
}

void USART1_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;

	/* USARTx configuration ------------------------------------------------------*/
	/* USARTx configured as follow:
	 *  - BaudRate = 9600 baud
	 *  - Word Length = 8 Bits
	 *  - One Stop Bit
	 *  - No parity
	 *  - Hardware flow control disabled (RTS and CTS signals)
	 *  - Receive and transmit enabled
	 */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);
}

void USART1_puts(char* s)
{
	while(*s) {
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
		USART_SendData(USART1, *s);
		s++;
	}
}


void camera_init(void)
{
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_DCMI, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	/* Clocking of  GPIOB is enabled in i2c_init function */

	/* PA4 - HSYNC
	   PA6 - PIXCLK
	   PB7 - DCMI_VSYNC
	   PC6 - DCMI_D0
	   PC7 - DCMI_D1
	   PC8 - DCMI_D2
	   PC9 - DCMI_D3
	   PC11 - DCMI_D4
	   PB6 - DCMI_D5
	   PB8 - DCMI_D6
	   PB9 - DCMI_D7

	   PB10 - I2C_SCL
	   PB11 - I2C_SDA
	 */

	/* set  gpio pins to provide DCMI access */
	GPIO_InitTypeDef dcmi_gpioa_init;
	dcmi_gpioa_init.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6;
	dcmi_gpioa_init.GPIO_Mode = GPIO_Mode_AF;
	dcmi_gpioa_init.GPIO_OType = GPIO_OType_PP;
	dcmi_gpioa_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
	dcmi_gpioa_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &dcmi_gpioa_init);

	GPIO_InitTypeDef dcmi_gpiob_init;
	dcmi_gpiob_init.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_9;
	dcmi_gpiob_init.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOB, &dcmi_gpiob_init);

	GPIO_InitTypeDef dcmi_gpioc_init;
	dcmi_gpioc_init.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_11;
	dcmi_gpioc_init.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOC, &dcmi_gpioc_init);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_DCMI);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_DCMI);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_DCMI);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_DCMI);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_DCMI);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_DCMI);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_DCMI);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_DCMI);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_DCMI);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_DCMI);

	/* dcmi interface init */
	DCMI_InitTypeDef camera_init;

	camera_init.DCMI_CaptureMode = DCMI_CaptureMode_SnapShot;
	camera_init.DCMI_CaptureRate = DCMI_CaptureRate_All_Frame;
	camera_init.DCMI_ExtendedDataMode = DCMI_ExtendedDataMode_8b;
	camera_init.DCMI_HSPolarity = DCMI_HSPolarity_High;
	camera_init.DCMI_PCKPolarity = DCMI_PCKPolarity_Falling;
	camera_init.DCMI_SynchroMode = DCMI_SynchroMode_Hardware;
	camera_init.DCMI_VSPolarity = DCMI_VSPolarity_High;

	DCMI_StructInit(&camera_init);
	// enable DCMI interface

	DMA_DeInit(DMA2_Stream1);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	DMA_InitTypeDef dma_init;

	dma_init.DMA_Channel = DMA_Channel_1;
	dma_init.DMA_PeripheralBaseAddr = (uint32_t) &(DCMI->DR);
	dma_init.DMA_Memory0BaseAddr = (uint32_t) camera_frame;
	dma_init.DMA_DIR = DMA_DIR_PeripheralToMemory;
	dma_init.DMA_BufferSize = 80*60/2;
	dma_init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma_init.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma_init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	dma_init.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	dma_init.DMA_Mode = DMA_Mode_Normal;
	dma_init.DMA_Priority = DMA_Priority_High;
	dma_init.DMA_FIFOMode = DMA_FIFOMode_Disable;
	dma_init.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	dma_init.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dma_init.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_Init(DMA2_Stream1, &dma_init);

	DMA_ITConfig(DMA2_Stream1, DMA_IT_TC | DMA_IT_FE, ENABLE);

	NVIC_InitTypeDef dma_stream1_irq;

	dma_stream1_irq.NVIC_IRQChannel = DMA2_Stream1_IRQn;
	dma_stream1_irq.NVIC_IRQChannelPreemptionPriority = 0;
	dma_stream1_irq.NVIC_IRQChannelSubPriority = 0;
	dma_stream1_irq.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&dma_stream1_irq);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

	NVIC_InitTypeDef camera_irq;
	camera_irq.NVIC_IRQChannel = DCMI_IRQn;
	camera_irq.NVIC_IRQChannelPreemptionPriority = 0;
	camera_irq.NVIC_IRQChannelSubPriority = 1;
	camera_irq.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&camera_irq);

	DCMI_ITConfig(DCMI_IT_ERR | DCMI_IT_FRAME | DCMI_IT_LINE |
			DCMI_IT_OVF | DCMI_IT_VSYNC, ENABLE);

	/* i2c interface init */
	GPIO_InitTypeDef i2c_gpio_init;

	i2c_gpio_init.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	i2c_gpio_init.GPIO_Mode = GPIO_Mode_AF;
	i2c_gpio_init.GPIO_OType = GPIO_OType_OD;
	i2c_gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
	i2c_gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init (GPIOB, &i2c_gpio_init);

	GPIO_PinAFConfig (GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);    // I2C2_SCL
	GPIO_PinAFConfig (GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);    // I2C2_SDA
	I2C_InitTypeDef i2c_init;

	i2c_init.I2C_Ack = I2C_Ack_Enable;
	i2c_init.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	i2c_init.I2C_ClockSpeed = 100000;
	i2c_init.I2C_DutyCycle = I2C_DutyCycle_2;
	i2c_init.I2C_Mode = I2C_Mode_I2C;
	i2c_init.I2C_OwnAddress1 = 21;

	I2C_Init(I2C2, &i2c_init);
	// enable I2C interface
	I2C_Cmd(I2C2, ENABLE);
/*********************************************************/

	/* gpio init to provide mco frequency */
	GPIO_InitTypeDef mco_Init;

	mco_Init.GPIO_Pin = GPIO_Pin_8;
	mco_Init.GPIO_Mode = GPIO_Mode_AF;
	mco_Init.GPIO_Speed = GPIO_Speed_100MHz;
	mco_Init.GPIO_OType = GPIO_OType_PP;

	GPIO_Init(GPIOA, &mco_Init);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO);

}
void SendPicture(void)
{
	int i;
	for(i=0;i<10;i++)
	{
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
		USART_SendData(USART1, camera_frame[i]);
	}
}

char* itoa(int value, char* result, int base)
{
	if (base < 2 || base > 36) {
		*result = '\0';
		return result;
	}
	char *ptr = result, *ptr1 = result, tmp_char;
	int tmp_value;

	do {
		tmp_value = value;
		value /= base;
		*ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value         - value * base)];
	} while (value);

	if (tmp_value < 0) *ptr++ = '-';
	*ptr-- = '\0';
	while (ptr1 < ptr) {
		tmp_char = *ptr;
		*ptr-- = *ptr1;
		*ptr1++ = tmp_char;
	}
	return result;
}
int count = 0;
char strr[10] = "";
int flag = 0;
int tt = 0;
void DCMI_IRQHandler(void)
{
	if(DCMI_GetFlagStatus(DCMI_FLAG_VSYNCRI) == SET)
	{
		if(flag!=1){
//			USART1_puts("dcmi VSYNCR1\r\n");
		}
		DCMI_ClearFlag(DCMI_FLAG_VSYNCRI);
	}
	else if(DCMI_GetFlagStatus(DCMI_FLAG_LINERI) == SET)
	{
		if(flag!=1){
//			USART1_puts("dcmi LINERI\r\n");
		}
		DCMI_ClearFlag(DCMI_FLAG_LINERI);
	}
	else if(DCMI_GetFlagStatus(DCMI_FLAG_FRAMERI) == SET)                                                   
	{
		char st[10] = "";
		char t[10] = "";
		flag = 1;
/*		uint32_t dr = DCMI_ReadData();
		itoa(dr,t,10);
		USART1_puts("DR = ");
		USART1_puts(t);
		USART1_puts("\r\n");
		*/
		if(tt < 80*60/2-1 ){
			uint16_t count = DMA_GetCurrDataCounter(DMA2_Stream1);
			USART1_puts("count");
			itoa(count,st,10);
			USART1_puts(st);
			USART1_puts("\r\n");
			flag = 1;
			USART1_puts("DCMI_FLAG_FRAMERI \n\n");
			tt++;
		}
		DCMI_ClearFlag(DCMI_FLAG_FRAMERI);
	}
	else if(DCMI_GetFlagStatus(DCMI_FLAG_ERRRI) == SET)
	{
//		USART1_puts("DCMI_FLAG_ERRRI \n\n");
		DCMI_ClearFlag(DCMI_FLAG_ERRRI);
	}
	else if(DCMI_GetFlagStatus(DCMI_FLAG_OVFRI) == SET)
	{
		

		flag = 1;
//		USART1_puts("DCMI_FLAG_OVFRI \n\n");
		DCMI_ClearFlag(DCMI_FLAG_OVFRI);
	}
}


char st[10]="";
void DMA2_Stream1_IRQHandler(void)
{
	USART1_puts("ys");
	dma_handler_counter++;
	itoa(dma_handler_counter,st,10);
	USART1_puts(st);
	USART1_puts("\r\n");
	
	if(DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF1) != RESET)
	{
		
		DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);
	}
	if(DMA_GetITStatus(DMA2_Stream1,DMA_IT_TEIF1) == SET){
		USART1_puts("DMA2 error\r\n");
		DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TEIF1);
	}
}


int main(void)
{
	uint8_t c = 0;
	SystemInit();
	RCC_Configuration();
	GPIO_Configuration();
	USART1_Configuration();

	RCC_APB2PeriphClockCmd (RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

	GPIO_InitTypeDef GPIOG_Init;

	GPIOG_Init.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
	GPIOG_Init.GPIO_Mode = GPIO_Mode_OUT;
	GPIOG_Init.GPIO_OType = GPIO_OType_PP;
	GPIOG_Init.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_Init(GPIOG, &GPIOG_Init);
	camera_init();
	c = I2C_readreg(0x0a);

	if(c == 0x76)
	{
		ov7670_init();
		GPIO_SetBits(GPIOG, GPIO_Pin_13);
	}

	DMA_Cmd(DMA2_Stream1, ENABLE);
	DCMI_Cmd(ENABLE);
	DCMI_CaptureCmd(ENABLE);
	int time = 0;
	while(1)
	{
		if(dma_handler_counter == 1){
			USART1_puts("pic");
			SendPicture();
			GPIO_SetBits(GPIOG, GPIO_Pin_14);
		}
	}

	return 0;
}
