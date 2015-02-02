#define USE_STDPERIPH_DRIVER
#include "stm32f4xx.h"
#include "stm32_p103.h"
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
/************************/
#include <string.h>
#include <math.h>

/* Filesystem includes */
#include "filesystem.h"
#include "fio.h"
#include "romfs.h"

#include "clib.h"
#include "shell.h"
#include "host.h"
#include "dataQuick.h"

#define PHASE_DELAY 500.0
#define PHASE_DELAY_MIN 100.0
/* _sromfs symbol can be found in main.ld linker script
 * it contains file system structure of test_romfs directory
 */

//static void setup_hardware();

/* motor A, B cycles */
int mot_num = 0;
int motA = 0;
int motB = 0;
/*delay of motor A, B */
float dA = 0.0;
float dB = 0.0;
/* handler */
xTaskHandle xHandle;
xTaskHandle xHandle_motorA;
xTaskHandle xHandle_motorB;
SemaphoreHandle_t xSemaphoreA;
SemaphoreHandle_t xSemaphoreB;
SemaphoreHandle_t xSemaphoreC;
SemaphoreHandle_t xSemaphoreD;
/* Add for serial input */
volatile xSemaphoreHandle serial_tx_wait_sem = NULL;
/* Add for serial input */
volatile xQueueHandle serial_rx_queue = NULL;
int positionX = 0;
int positionY = 0;
/* IRQ handler to handle USART2 interruptss (both transmit and receive
 * interrupts). */
void USART1_IRQHandler()
{
	static signed portBASE_TYPE xHigherPriorityTaskWoken;

	/* If this interrupt is for a transmit... */
	if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET) {
		/* "give" the serial_tx_wait_sem semaphore to notfiy processes
		 * that the buffer has a spot free for the next byte.
		 */
		xSemaphoreGiveFromISR(serial_tx_wait_sem, &xHigherPriorityTaskWoken);

		/* Diables the transmit interrupt. */
		USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
		/* If this interrupt is for a receive... */
	}else if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){
		char msg = USART_ReceiveData(USART1);

		/* If there is an error when queueing the received byte, freeze! */
		if(!xQueueSendToBackFromISR(serial_rx_queue, &msg, &xHigherPriorityTaskWoken))
			while(1);
	}
	else {
		/* Only transmit and receive interrupts should be enabled.
		 * If this is another type of interrupt, freeze.
		 */
		while(1);
	}

	if (xHigherPriorityTaskWoken) {
		taskYIELD();
	}
}

void send_byte(char ch)
{
	/* Wait until the RS232 port can receive another byte (this semaphore
	 * is "given" by the RS232 port interrupt when the buffer has room for
	 * another byte.
	 */
	while (!xSemaphoreTake(serial_tx_wait_sem, portMAX_DELAY));

	/* Send the byte and enable the transmit interrupt (it is disabled by
	 * the interrupt).
	 */
	USART_SendData(USART1, ch);
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
}

char recv_byte()
{
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	char msg;
	while(!xQueueReceive(serial_rx_queue, &msg, portMAX_DELAY));
	return msg;
}

void Delay(uint32_t volatile DelayTime_uS){
	uint32_t DelayTime = 0;
	DelayTime = SystemCoreClock/1000000*DelayTime_uS;
	for(;DelayTime != 0 ; DelayTime--)
		__NOP();
}

void clockwise(int n, float delay)
{
	GPIO_ResetBits(GPIOG, GPIO_Pin_9 | GPIO_Pin_10 | \
	               GPIO_Pin_13);
	GPIO_SetBits(GPIOG, GPIO_Pin_14);
	int i = 0;
	for(i = 0; i < n; i++) {
		GPIO_ResetBits(GPIOG, GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_13);
		Delay(delay);
		GPIO_SetBits(GPIOG, GPIO_Pin_13);
		Delay(delay);
		GPIO_ToggleBits(GPIOG, GPIO_Pin_14);
		Delay(delay);
		GPIO_SetBits(GPIOG, GPIO_Pin_10);
		Delay(delay);
		GPIO_ToggleBits(GPIOG, GPIO_Pin_13);
		Delay(delay);
		GPIO_SetBits(GPIOG, GPIO_Pin_9);
		Delay(delay);
		GPIO_ToggleBits(GPIOG, GPIO_Pin_10);
		Delay(delay);
		GPIO_SetBits(GPIOG, GPIO_Pin_14);
		Delay(delay);
	}
	fio_printf(1, "cw A %d\r\n", i);
}
void counterClockwise(int n, float delay)
{
	GPIO_ResetBits(GPIOG, GPIO_Pin_10 | \
	               GPIO_Pin_13 | GPIO_Pin_14);
	GPIO_SetBits(GPIOG, GPIO_Pin_9);
	int i = 0;
	for(i = 0; i < n; i++) {
		GPIO_ResetBits(GPIOG, GPIO_Pin_14);
		Delay(delay);
		GPIO_SetBits(GPIOG, GPIO_Pin_10);
		Delay(delay);
		GPIO_ToggleBits(GPIOG, GPIO_Pin_9);
		Delay(delay);
		GPIO_SetBits(GPIOG, GPIO_Pin_13);
		Delay(delay);
		GPIO_ToggleBits(GPIOG, GPIO_Pin_10);
		Delay(delay);
		GPIO_SetBits(GPIOG, GPIO_Pin_14);
		Delay(delay);
		GPIO_ToggleBits(GPIOG, GPIO_Pin_13);
		Delay(delay);
		GPIO_SetBits(GPIOG, GPIO_Pin_9);
		Delay(delay);
	}
	fio_printf(1, "ccw A %d\r\n", i);
}
void clockwiseB(int n, float delay)
{
	GPIO_ResetBits(GPIOE, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4);
	GPIO_SetBits(GPIOE, GPIO_Pin_5);
	int i = 0;
	for(i = 0; i < n; i++) {
		GPIO_ResetBits(GPIOE, GPIO_Pin_2);
		Delay(delay);
		GPIO_SetBits(GPIOE, GPIO_Pin_4);
		Delay(delay);
		GPIO_ToggleBits(GPIOE, GPIO_Pin_5);
		Delay(delay);
		GPIO_SetBits(GPIOE, GPIO_Pin_3);
		Delay(delay);
		GPIO_ToggleBits(GPIOE, GPIO_Pin_4);
		Delay(delay);
		GPIO_SetBits(GPIOE, GPIO_Pin_2);
		Delay(delay);
		GPIO_ToggleBits(GPIOE, GPIO_Pin_3);
		Delay(delay);
		GPIO_SetBits(GPIOE, GPIO_Pin_5);
		Delay(delay);
	}
	fio_printf(1, "cw B %d\r\n", i);
}
void counterClockwiseB(int n, float delay)
{
	GPIO_ResetBits(GPIOE, GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);
	GPIO_SetBits(GPIOE, GPIO_Pin_2);
	int i = 0;
	for(i = 0; i < n; i++) {
		GPIO_ResetBits(GPIOE, GPIO_Pin_5);
		Delay(delay);
		GPIO_SetBits(GPIOE, GPIO_Pin_3);
		Delay(delay);
		GPIO_ToggleBits(GPIOE, GPIO_Pin_2);
		Delay(delay);
		GPIO_SetBits(GPIOE, GPIO_Pin_4);
		Delay(delay);
		GPIO_ToggleBits(GPIOE, GPIO_Pin_3);
		Delay(delay);
		GPIO_SetBits(GPIOE, GPIO_Pin_5);
		Delay(delay);
		GPIO_ToggleBits(GPIOE, GPIO_Pin_4);
		Delay(delay);
		GPIO_SetBits(GPIOE, GPIO_Pin_2);
		Delay(delay);
	}
	fio_printf(1, "ccw B %d\r\n", i);
}

void parse(char* str,char* argv[]){
	int b_quote=0, b_dbquote=0;
	int count = 0;
	int p = 0;
	int i;
	for(i=0; str[i]; i++){
		if(str[i]=='\'')
			++b_quote;
		if(str[i]=='"')
			++b_dbquote;
		if(str[i]==' '&&b_quote%2==0&&b_dbquote%2==0){
			str[i]='\0';
			argv[count++]=&str[p];
			p=i+1;
		}
	}
}

void gpio_init(){
	// AHB clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_InitTypeDef GPIO_E;
	GPIO_E.GPIO_Pin = GPIO_Pin_All;//2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_E.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_E.GPIO_OType = GPIO_OType_PP;
	GPIO_E.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_E.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOE, &GPIO_E);
}

void command_prompt(void *pvParameters)
{
	char buf[128];
	char *argv[100];
	char hint[] = USER_NAME "@" USER_NAME "-STM32:~$ ";

	xSemaphoreA = xSemaphoreCreateMutex();
	xSemaphoreB = xSemaphoreCreateMutex();
	xSemaphoreC = xSemaphoreCreateMutex();
	xSemaphoreD = xSemaphoreCreateMutex();
	if(xSemaphoreA == NULL || xSemaphoreB == NULL || xSemaphoreC == NULL || xSemaphoreD == NULL)
		fio_printf(1, "create semaphore failed!\r\n");
	
	else {
		xSemaphoreTake(xSemaphoreA, (TickType_t) 10);
		xSemaphoreTake(xSemaphoreB, (TickType_t) 10);
		xSemaphoreTake(xSemaphoreC, (TickType_t) 10);
		xSemaphoreTake(xSemaphoreD, (TickType_t) 10);
		fio_printf(1, "command get semaphoreAB\r\n");
	}
	fio_printf(1,"Painter control!\r\n");
	while(1){
		fio_printf(1, "%s", hint);
		fio_read(0, buf, 127);
		//parse f code	
		parse(buf,argv);
		/* this task control pen */
		/*motA = atoi(argv[0]);
		motB = atoi(argv[1]);
		int penCtrl = atoi(argv[2]);*/
		int penCtrl = 0;
		if(atoi(argv[0]) == 1) {
		  for(int i = 0; i < data3num; i++) {
		    //doneA = 0;
		    //doneB = 0;
		    motA = data3[i][0];
		    motB = data3[i][1];
		    penCtrl = data3[i][2];
			fio_printf(1, "%d %d %d\r\n", motA, motB, penCtrl);
			if(penCtrl == 0) {
				/* up */
				TIM4 -> CCR1 = 37;
			}
			else if(penCtrl == 1) {
				/* down */
				TIM4 -> CCR1 = 115;
			}
			/* convert coordinate to steps of motor A, B */
			int tempA = motA;
			int tempB = motB;
			motA = motA - positionX;
			motB = motB - positionY;
			positionX = tempA;
			positionY = tempB;
			if(motA != 0) {
				mot_num++;
			}
			if(motB != 0) {
				mot_num++;
			}
			fio_printf(1, "%d\r\n", mot_num);
			if(mot_num == 2){
				dB = ((float)motA/(float)motB) * PHASE_DELAY_MIN;
				if(dB < 0)
					dB = 0 - dB;
				if(dB < PHASE_DELAY_MIN && dB > 0) {
					dB = PHASE_DELAY_MIN;
					dA = ((float)motB / (float)motA) * dB;
					if(dA < 0)
						dA = 0 - dA;
				}
				else {
					dA = PHASE_DELAY_MIN;
				}
				xSemaphoreGive(xSemaphoreA);
				xSemaphoreGive(xSemaphoreB);
			}
			else {
				if(motA != 0){
					/* only motorA moving */
					dA = PHASE_DELAY_MIN;
					xSemaphoreGive(xSemaphoreA);
				}
				else{
					/* only motorB moving */
					dB = PHASE_DELAY_MIN;
					xSemaphoreGive(xSemaphoreB);
				}
			}
			//vTaskDelay(100);
			vTaskSuspend(xHandle);
			if(mot_num != 2) {
				if(motA != 0)
					while(xSemaphoreTake(xSemaphoreC, (TickType_t) 10) != pdTRUE);
				else
					while(xSemaphoreTake(xSemaphoreD, (TickType_t) 10) != pdTRUE);
			}
			else {
				while(xSemaphoreTake(xSemaphoreC, (TickType_t) 10) != pdTRUE);
				while(xSemaphoreTake(xSemaphoreD, (TickType_t) 10) != pdTRUE);
			}
			mot_num = 0;
			motA = 0;
			motB = 0;
			fio_printf(1, "finish %d times , A, B\r\n", i+1);
		  }
		}
	}
}

void motorA_handler(void *pvParameters){
	while(1){
		//vTaskResume(xHandle);
		if(xSemaphoreTake(xSemaphoreA, (TickType_t) 10) == pdTRUE){
			fio_printf(1, "A got semaphore\r\n");
			if(mot_num==2){
				/* function handle */
				if(motA < 0)
					clockwise(0 - motA, dA);
				else
					counterClockwise(motA, dA);
			}else{
				/* function handle */
				if(motA < 0)
					clockwise(0 - motA, dA);
				else
					counterClockwise(motA, dA);
			}
			fio_printf(1,"A finished\r\n");
			xSemaphoreGive(xSemaphoreC);
			if(mot_num == 2 || (mot_num == 1 && motA != 0))
				vTaskResume(xHandle);
			//vTaskSuspend(xHandle_motorA);
			//vTaskDelay(100);
		}
	}
}

void motorB_handler(void *pvParameters){
	while(1){
		//vTaskResume(xHandle);
		if(xSemaphoreTake(xSemaphoreB, (TickType_t) 10) == pdTRUE){
			fio_printf(1, "B got semaphore\r\n");
			if(mot_num==2){
				/* function handle */
				if(motB < 0)
					clockwiseB(0 - motB, dB);
				else
					counterClockwiseB(motB, dB);
			}else{
				/* function handle */
				if(motB < 0)
					clockwiseB(0 - motB, dB);
				else
					counterClockwiseB(motB, dB);
			}
			fio_printf(1,"B finished\r\n");
			xSemaphoreGive(xSemaphoreD);
			if(mot_num == 1 && motB != 0)
				vTaskResume(xHandle);
			//vTaskSuspend(xHandle_motorB);
			//vTaskDelay(100);
		}
	}

}

int main()
{
	gpio_init();
	init_rs232();
	enable_rs232_interrupts();
	enable_rs232();
	TIM4 -> CCR1 = 37;
	fs_init();
	fio_init();
	/* Create the queue used by the serial task.  Messages for write to
	 * the RS232. */
	vSemaphoreCreateBinary(serial_tx_wait_sem);
	/* Reference: www.freertos.org/a00116.html */
	serial_rx_queue = xQueueCreate(1, sizeof(char));
	/* Start running the tasks. */
	xTaskCreate(command_prompt,
	            (signed portCHAR *) "CLI",
	            512 /* stack size */, NULL, tskIDLE_PRIORITY + 2, &xHandle);
	xTaskCreate(motorA_handler,(signed portCHAR *) "motorA",512,NULL,tskIDLE_PRIORITY+1,&xHandle_motorA);
	xTaskCreate(motorB_handler,(signed portCHAR *) "motorB",512,NULL,tskIDLE_PRIORITY+1,&xHandle_motorB);
	vTaskStartScheduler();

	return 0;
}

void vApplicationTickHook()
{
}
