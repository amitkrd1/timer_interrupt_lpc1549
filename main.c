#include "board.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
//#include "gps.h"
//#include <time.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#define USE_INTEGER_CLOCK
STATIC RINGBUFF_T txring, rxring;
//STATIC RINGBUFF_T txring1, rxring1;               ///Ringbuffer variables
#define UART_RB_SIZE 128


#if defined(BOARD_NXP_LPCXPRESSO_1549)
#define LPC_USART       LPC_USART0
#define LPC_IRQNUM      UART0_IRQn
#define LPC_UARTHNDLR   UART0_IRQHandler

#else
#error "No UART setup defined"
#endif
#define NUM_FIELDS
#define UART_TEST_DEFAULT_BAUDRATE 9600
 char stringbuffer[1024];

static uint8_t rxbuff[UART_RB_SIZE], txbuff[UART_RB_SIZE];
static uint8_t rxbuff1[UART_RB_SIZE],txbuff1[UART_RB_SIZE];
static uint8_t rxbuff2[UART_RB_SIZE],txbuff2[UART_RB_SIZE];


const char inst1[] = "LPC15xx UART1 example using ring buffers\r\n";
const char inst2[] = "Press a key to echo it back or ESC to quit\r\n";
const char inst3[]="UART0 testing TASK-4  \r\n";
const char inst4[]="Its working\r\n";
const char inst5[]="UART2 testing TASK-5  \r\n";
const char inst6[]="its sucessful\r\n";
const char msg[]="interrupt based timer tick \r\n";


uint32_t clkrate;
uint32_t systemclkrate;
uint32_t pllinclkrate;
uint32_t plloutclkrate;
uint32_t systickrate;
uint32_t pllfreq;


void NVIC_config(void)
{
	
	//NVIC_EnableIRQ(UART0_IRQn);
	//NVIC_EnableIRQ(UART1_IRQn);
	
	//Chip_UART_IntEnable(LPC_USART1, UART_INTEN_RXRDY);
	//Chip_UART_IntDisable(LPC_USART1, UART_INTEN_TXRDY);
	
	NVIC_EnableIRQ(UART0_IRQn );
	NVIC_EnableIRQ(UART1_IRQn );                       //IRQ number for UART0 UART1, UART2 are 21,22,23 respectively
	NVIC_EnableIRQ(UART2_IRQn );
	NVIC_SetPriority(21,0);                 	//0 highest interrupt priority, 7 lowest interrupt priority
	NVIC_SetPriority(22,7); 
	NVIC_SetPriority(23,2);
	
}
static void pinmux(void)
{
	
 Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 13, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));
 Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 18, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));
 Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 22, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));
 Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 23, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 6, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));
 Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 7, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	
	Chip_SWM_MovablePortPinAssign(SWM_UART1_RXD_I, 0, 13);
  Chip_SWM_MovablePortPinAssign(SWM_UART1_TXD_O, 0, 18);
	Chip_SWM_MovablePortPinAssign(SWM_UART0_RXD_I, 0, 22);
  Chip_SWM_MovablePortPinAssign(SWM_UART0_TXD_O, 0, 23);
	Chip_SWM_MovablePortPinAssign(SWM_UART2_RXD_I, 0, 6);
  Chip_SWM_MovablePortPinAssign(SWM_UART2_TXD_O, 0, 7);
	
}



static void uart_init(void)
{
	Chip_UART_Init(LPC_USART0);
	Chip_UART_Init(LPC_USART1);
	Chip_UART_Init(LPC_USART2);
	
	Chip_UART_ConfigData(LPC_USART1, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1);
	Chip_UART_SetBaud(LPC_USART1, UART_TEST_DEFAULT_BAUDRATE);
	Chip_UART_Enable(LPC_USART1);
	Chip_UART_TXEnable(LPC_USART1);
	
	
	Chip_UART_ConfigData(LPC_USART0, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1);
	Chip_UART_SetBaud(LPC_USART0, UART_TEST_DEFAULT_BAUDRATE);
	Chip_UART_Enable(LPC_USART0);
	Chip_UART_TXEnable(LPC_USART0);
	
		Chip_UART_ConfigData(LPC_USART2, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1);
	 Chip_UART_SetBaud(LPC_USART2, UART_TEST_DEFAULT_BAUDRATE);
	 Chip_UART_Enable(LPC_USART2);
	 Chip_UART_TXEnable(LPC_USART2);
  
	//Chip_UART_IntEnable(LPC_USART1, UART_INTEN_RXRDY);
	//Chip_UART_IntDisable(LPC_USART1, UART_INTEN_TXRDY);
	
	//NVIC_EnableIRQ(UART0_IRQn);
	//NVIC_EnableIRQ(UART1_IRQn);
}

void SCT0_Init(void)
{

	Chip_SCT_Init(LPC_SCT0);			                   						// enable the SCT2 clock

	Chip_SCT_Config(LPC_SCT0, 	SCT_CONFIG_32BIT_COUNTER  |						// unified timers,
                              SCT_CONFIG_AUTOLIMIT_L    );						// auto limit

	Chip_SCT_SetMatchCount(LPC_SCT0, SCT_MATCH_0,	(SystemCoreClock/100) - 1);	//match 0 @ 100 Hz = 10 msec
	Chip_SCT_SetMatchReload(LPC_SCT0, SCT_MATCH_0, (SystemCoreClock/100) - 1);


	Chip_SCT_EventState(LPC_SCT0, SCT_EVENT_0, ENABLE_ALL_STATES);	 			// event 0 happens in all states
	Chip_SCT_EventControl(LPC_SCT0, SCT_EVENT_0, SCT_COMBMODE_MATCH);			// match 0 only condition

	Chip_SCT_EnableEventInt(LPC_SCT0,SCT_EVT_0);								// event 0 generates an interrupt

    NVIC_EnableIRQ(SCT0_IRQn);                             						// enable SCT0 interrupt

    Chip_SCT_ClearControl(LPC_SCT0,SCT_CTRL_HALT_L);							// start timer

}

void SCT0_IRQHandler(void)										// SCT0 Interrupt Handler (10 msec)
{
	 static uint8_t cnt = 0;

	    if (++cnt > 20)                                			// 20 * 10 msec = 200 msec
	    {
	        cnt = 0;
					Chip_UART_SendBlocking(LPC_USART0, msg, sizeof(msg) - 1);
	        Chip_GPIO_SetPinToggle(LPC_GPIO , 1, 1); 			// toggle pin P1_1
	    }
	    Chip_SCT_SetEventFlag(LPC_SCT0,SCT_EVT_0);  			// clear event 0 flag
}




static void prvSetupHardware(void)
{

	SystemCoreClockUpdate();
	Board_Init();
	/* Initial LED0 state is off */
	Board_LED_Set(0,true);
}


static void vLEDTask1(void *pvParameters) {
	bool LedState = false;

	while (1) {
		Board_LED_Set(2, LedState);
	
		//LedState = (bool) !LedState;
  //printf("hello world \n");
		// About a 3Hz on/off toggle rate 
		vTaskDelay(configTICK_RATE_HZ /6);
	}
}

static void vLEDTask2(void *pvParameters) {
	bool LedState = false;

	while (1) {
		Board_LED_Set(1, LedState);
		LedState = (bool) !LedState;
		clkrate=Chip_Clock_GetMainClockRate();
		systemclkrate=Chip_Clock_GetSystemClockRate();
	 	pllinclkrate=	Chip_Clock_GetSystemPLLInClockRate();
		plloutclkrate=Chip_Clock_GetSystemPLLOutClockRate();
		//systickrate=Chip_Clock_GetSysTickClockRate();
	
		  printf("%d \r\n",plloutclkrate);
	  //printf("%d \t %d \n",pllinclkrate,plloutclkrate);

		
		//printf("Hello World !!..\n\r");

		/* About a 7Hz on/off toggle rate */
		vTaskDelay(configTICK_RATE_HZ / 14);
	}
}

static void UART1_task3(void *pvParameters)
{
	
	

	 RingBuffer_Init(&rxring, rxbuff1, 1, UART_RB_SIZE);
	 RingBuffer_Init(&txring, txbuff1, 1, UART_RB_SIZE);


	  
	   while(1)
		 {
			 
			//clkrate=Chip_Clock_GetIntOscRate();
		//Chip_UART_SendBlocking(LPC_USART1,(const char *)clkrate, 1000);
				//Chip_UART_ReadRB(LPC_USART,&rxbuff,stringbuffer,50);
				Chip_UART_SendBlocking(LPC_USART1, inst1, sizeof(inst1) - 1);
			
			 
     Chip_UART_SendBlocking(LPC_USART1, inst2, sizeof(inst2) - 1);
		 }
}
static void UART0_Task4(void *pvParameters)
{
	uint8_t key;
	int bytes;
	
	  RingBuffer_Init(&rxring, rxbuff, 1, UART_RB_SIZE);
	  RingBuffer_Init(&txring, txbuff, 1, UART_RB_SIZE);
	  Chip_UART_TXIntHandlerRB(LPC_USART0,&txring);
		//Chip_UART_IntEnable(LPC_USART0, UART_INTEN_RXRDY);
	  //Chip_UART_IntDisable(LPC_USART0, UART_INTEN_TXRDY);
		
	 
	
			while(1)
			{
			//Chip_UART_ReadBlocking(LPC_USART0,stringbuffer,100);
			//Chip_UART_SendBlocking(LPC_USART0, inst3, 100);
			//Chip_UART_ReadRB(LPC_USART0,&rxring,stringbuffer,50);
       //vTaskDelay(configTICK_RATE_HZ);
			//Chip_UART_SendRB(LPC_USART0,&txring,inst3, sizeof(inst3)-1);
				//Chip_UART_SendBlocking(LPC_USART0, inst4, sizeof(inst4) - 1);
	   
}
			}

/*static void UART2_task5(void *pvParameters)
{
	 RingBuffer_Init(&rxring, rxbuff2, 1, UART_RB_SIZE);
	 RingBuffer_Init(&txring, txbuff2, 1, UART_RB_SIZE);

	  
	   while(1)
		 {
				//Chip_UART_ReadRB(LPC_USART,&rxbuff,stringbuffer,50);
				Chip_UART_SendBlocking(LPC_USART2, inst5, sizeof(inst5) - 1);
			  //vTaskDelay(configTICK_RATE_HZ);
			 
				//Chip_UART_SendBlocking(LPC_USART2, inst6, sizeof(inst6) - 1);
		 }
}
*/
static void timer_task(void *pvParameters)
{
	
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_UART1 | SYSCTL_CLOCK_GPIO1);   	// enable GPIO1 and UART clock

	Chip_GPIO_SetPinDIR(LPC_GPIO, 1, 1, true);				                        // configure P1_1 (BLUE LED) as output

	SCT0_Init();											                                        // Initialize SCT

	while (1)                                         	                     	// loop forever
	{
		__WFI();
	}	
}





 int main(void)
 {	
	int i;
	 prvSetupHardware();
	 pinmux();
	 uart_init();
	 //clock_setup();
	 
	 //xTaskCreate(vLEDTask1, "vTaskLed1",
			//	configMINIMAL_STACK_SIZE, NULL, (0 ),
			//	(TaskHandle_t *) NULL);
	 
	// xTaskCreate(vLEDTask2, "vTaskLed2",
			//	configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL ),
				//(TaskHandle_t *) NULL);
	 
	xTaskCreate(UART1_task3, "UART1Test",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL ),
				(TaskHandle_t *) NULL);
  
	 xTaskCreate(UART0_Task4, "UARTTest",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL ),
				(TaskHandle_t *) NULL); 
	 
	 xTaskCreate(timer_task, "TimerTest",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL ),
				(TaskHandle_t *) NULL); 

	 vTaskStartScheduler();
	return 1;		

	 	 
//the code should never reach here
}
