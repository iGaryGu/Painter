#include "i2c_ops.h"
#include "main.h"
#include "stm32f4xx_i2c.h"

#define SLAVE_ADDRESS 0x42

/* This function issues a start condition and
 * transmits the slave address + R/W bit
 *
 * Parameters:
 * 		I2Cx --> the I2C peripheral e.g. i2c_dev
 * 		address --> the 7 bit slave address
 * 		direction --> the transmission direction can be:
 * 						I2C_Direction_Tranmitter for Master transmitter mode
 * 						I2C_Direction_Receiver for Master receiver
 */

void delay(uint32_t volatile DelayTime_uS)
{
	uint32_t DelayTime = 0;
	DelayTime = (SystemCoreClock/1000000)*DelayTime_uS;
	for (; DelayTime!=0; DelayTime--);
}

void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction)
{
	// wait until i2c_dev is not busy anymore
//	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
	// Send i2c_dev START condition
	I2C_GenerateSTART(I2Cx, ENABLE);
	
	USART1_puts("send start!\r\n");
	// wait for i2c_dev EV5 --> Slave has acknowledged start condition
//	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
	// Send slave Address for write
	I2C_Send7bitAddress(I2Cx, address, direction);

	USART1_puts("send address!\r\n");
	/* wait for i2c_dev EV6, check if
	 * either Slave has acknowledged Master transmitter or
	 * Master receiver mode, depending on the transmission
	 * direction
	 */
	if(direction == I2C_Direction_Transmitter)
	{
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
//		Delay(100);

		USART1_puts("master transmit!\r\n");
	}
	else if(direction == I2C_Direction_Receiver)
	{
//		Delay(100);
//		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
		USART1_puts("master receive!\r\n");
	}
}


/* This function transmits one byte to the slave device
 * Parameters:
 *		I2Cx --> the I2C peripheral e.g. i2c_dev
 *		data --> the data byte to be transmitted
 */
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data)
{
	I2C_SendData(I2Cx, data);
	USART1_puts("i2c send!\r\n");
	// wait for i2c_dev EV8_2 --> byte has been transmitted
//	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
//	Delay(100);
}

/* This function reads one byte from the slave device
 * and acknowledges the byte (requests another byte)
 */
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx){
	uint8_t data;
	// enable acknowledge of received data
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	data = I2C_ReceiveData(I2Cx);
	return data;
}


/* This function reads one byte from the slave device
 * and doesn't acknowledge the received data
 */
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx){
	uint8_t data;
	// disable acknowledge of received data
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
//	Delay(100);
	// read data from I2C data register and return data byte
	data = I2C_ReceiveData(I2Cx);
	USART1_puts("i2c received!\r\n");
	return data;
}


/* This function issues a stop condition and therefore
 * releases the bus
 */
void I2C_stop(I2C_TypeDef* I2Cx){
	// Send i2c_dev STOP Condition
	I2C_GenerateSTOP(I2Cx, ENABLE);
}


uint8_t I2C_writereg(uint8_t reg, uint8_t data)
{
	uint8_t tmp;

	USART1_puts("start!\r\n");

	I2C_start(i2c_dev, SLAVE_ADDRESS, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	USART1_puts("write start!\r\n");
	I2C_write(i2c_dev, (uint8_t) reg); // write one byte to the slave
	USART1_puts("write!one \r\n");
	delay(100);
	I2C_write(i2c_dev, (uint8_t) data); // write one byte to the slave
	USART1_puts("write!two \r\n");
	I2C_stop(i2c_dev); // stop the transmission
	USART1_puts("stop!\r\n");
	delay(100);
	I2C_start(i2c_dev, SLAVE_ADDRESS, I2C_Direction_Receiver); // start a transmission in Master receiver mode
	USART1_puts("start!\r\n");
	tmp = I2C_read_nack(i2c_dev);
	USART1_puts("write nack!\r\n");
	I2C_stop(i2c_dev); // stop the transmission
	USART1_puts("stop!\r\n");
	delay(100);

	return tmp;
}


uint8_t I2C_readreg(uint8_t reg)
{
	uint8_t tmp;
	I2C_start(i2c_dev, SLAVE_ADDRESS, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	USART1_puts("start!\r\n");

	I2C_write(i2c_dev, (uint8_t) reg); // write one byte to the slave
	USART1_puts("write!\r\n");
	I2C_stop(i2c_dev); // stop the transmission
	USART1_puts("stop!\r\n");

	delay(100);

	I2C_start(i2c_dev, SLAVE_ADDRESS, I2C_Direction_Receiver); // start a transmission in Master receiver mode
	USART1_puts("master received!\r\n");
	tmp = I2C_read_nack(i2c_dev);
	USART1_puts("read nack!\r\n");

	I2C_stop(i2c_dev); // stop the transmission
	USART1_puts("stop!\r\n");


	delay(100);

	return tmp;
}
