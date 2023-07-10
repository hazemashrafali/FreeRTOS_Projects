/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "lpc21xx.h"
#include "queue.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"


/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )
	
/*TASK MACROS*/
#define ULONG_MAX 		0xFFFFFFFF
#define TRUE	1
#define FALSE	0

/*BTN configuration*/
#define BTN1_PORT		PORT_0
#define BTN2_PORT		PORT_0
#define BTN1_PIN		PIN0
#define BTN2_PIN		PIN1

/*PRIORITIE TYPEs*/
#define _1ST_PRIORITIE_RANK		1
#define _2ND_PRIORITIE_RANK		2

/*Tasks Handlers*/
TaskHandle_t  BTN1_Task_Handler 		= NULL;
TaskHandle_t  BTN2_Task_Handler 		= NULL;
TaskHandle_t  Periodic_Task_Handler = NULL;
TaskHandle_t  UART_Task_Handler 		= NULL;

/*Queue Handler*/
QueueHandle_t UART_queue_Handler		= NULL;

/*QUEUE Length*/
#define UART_QUEUE_LENGTH		20

/*message notify ID*/
#define BTN1_PRESS_NOTIFY_ID		0x01
#define BTN1_RELEASE_NOTIFY_ID	0x02
#define BTN2_PRESS_NOTIFY_ID		0x03
#define BTN2_RELEASE_NOTIFY_ID	0x04
#define BTN1_PRESS_NOTIFY_ID		0x01
#define PERIODIC_NOTIFY_ID			0x05


/*TASKs DELAY*/
#define DEBOUNCE_DELAY 			50
#define BTN_TASK_DELAY			5
#define PERIODIC_TASK_DELAY	100

/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/


/*task to be created*/
void BTN1_Task (void * pvParameters)
{
	unsigned char BTN_press_notify 		= 0x01;
	unsigned char BTN_release_notify 	= 0x02;
	for( ;; )
    {
			// check button state
			if(GPIO_read(BTN1_PORT, BTN1_PIN) == PIN_IS_HIGH)
			{
				if( UART_queue_Handler != NULL )
				{
						/* Send an unsigned char.  Wait for MAX ticks for space to become available if necessary. */
						if( xQueueSend( UART_queue_Handler,&BTN_press_notify,portMAX_DELAY ) != pdPASS )
						{
								/* Failed to post the message, even after MAX ticks. */
						}
				}
				// debounce delay
				vTaskDelay(DEBOUNCE_DELAY);

				// recheck if button is released
				while(GPIO_read(BTN1_PORT, BTN1_PIN) == PIN_IS_HIGH) // wait until btn is released
				{
					vTaskDelay(BTN_TASK_DELAY);
				}

				// button released, notify falling edge (released)
				if( xQueueSend( UART_queue_Handler,&BTN_release_notify,portMAX_DELAY ) != pdPASS )
				{
						/* Failed to post the message, even after MAX ticks. */
				}
				
			}
			else
			{
				// Do Nothing
			}
    }
		vTaskDelete(NULL);
}

void BTN2_Task (void * pvParameters)
{
		unsigned char BTN_press_notify 		= 0x03;
		unsigned char BTN_release_notify 	= 0x04;
		for( ;; )
    {
			// check button state
			if(GPIO_read(BTN2_PORT, BTN2_PIN) == PIN_IS_HIGH)
			{
				if( UART_queue_Handler != NULL )
				{
						/* Send an unsigned char.  Wait for MAX ticks for space to become available if necessary. */
						if( xQueueSend( UART_queue_Handler,&BTN_press_notify,portMAX_DELAY ) != pdPASS )
						{
								/* Failed to post the message, even after MAX ticks. */
						}
				}
				// debounce delay
				vTaskDelay(DEBOUNCE_DELAY);

				// recheck if button is released
				while(GPIO_read(BTN2_PORT, BTN2_PIN) == PIN_IS_HIGH) // wait until btn is released
				{
					vTaskDelay(BTN_TASK_DELAY);
				}

				// button released, notify falling edge (released)
				if( xQueueSend( UART_queue_Handler,&BTN_release_notify,portMAX_DELAY ) != pdPASS )
				{
						/* Failed to post the message, even after MAX ticks. */
				}
				
			}
			else
			{
				// Do Nothing
			}
    }
		vTaskDelete(NULL);
}

void Periodic_Task (void * pvParameters)
{
	unsigned char periodic_notify = 0x05;
	for( ;; )
    {
				if( UART_queue_Handler != NULL )
				{
						/* Send an unsigned char.  Wait for MAX ticks for space to become available if necessary. */
						if( xQueueSend( UART_queue_Handler,&periodic_notify,portMAX_DELAY ) != pdPASS )
						{
								/* Failed to post the message, even after 5 ticks. */
						}
				}
				vTaskDelay(PERIODIC_TASK_DELAY);
    }
		vTaskDelete(NULL);
}

void UART_Task (void * pvParameters)
{
	unsigned char receive;
	for( ;; )
    {
				/* receive now contains a copy of xMessage. */	
			 if( xQueueReceive( UART_queue_Handler,&receive,portMAX_DELAY ) == pdPASS )
			 {
					if (receive == BTN1_PRESS_NOTIFY_ID)		 			//BTN1 pressed
					{
						while(vSerialPutString( ( signed char * ) "Button_1 pressed\n", sizeof("Button_1 pressed\n") ) == pdFALSE);
					}
					else if (receive == BTN1_RELEASE_NOTIFY_ID)		 	//BTN1 released
					{
						while(vSerialPutString( ( signed char * ) "Button_1 released\n", sizeof("Button_1 released\n") ) == pdFALSE);
					}
					else if (receive == BTN2_PRESS_NOTIFY_ID) 		//BTN2 pressed
					{
						while(vSerialPutString( ( signed char * ) "Button_2 pressed\n", sizeof("Button_2 pressed\n") ) == pdFALSE);
					}
					else if (receive == BTN2_RELEASE_NOTIFY_ID)			//BTN2 released
					{
						while(vSerialPutString( ( signed char * ) "Button_2 released\n", sizeof("Button_2 released\n") ) == pdFALSE);
					}
					else	if (receive == PERIODIC_NOTIFY_ID)		//periodic task
					{
						while(vSerialPutString( ( signed char * ) "PERIODIC MESSAGE\n", sizeof("PERIODIC MESSAGE\n") ) == pdFALSE);
					}
					else
					{
						//do nothing
					}
			 }
    }
}


/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();
		
	/*
	The semaphore is created in the 'empty' state, meaning the semaphore must first be
	given using the xSemaphoreGive() API function before it can subsequently be taken
	(obtained) using the xSemaphoreTake() function.
	*/
	
    /* Create Tasks here */
							xTaskCreate(  BTN1_Task,
                            "BTN1_Task",
                            configMINIMAL_STACK_SIZE,
                            NULL,
                            _1ST_PRIORITIE_RANK ,
                            &BTN1_Task_Handler
                          );
							xTaskCreate(  BTN2_Task,
                            "BTN2_Task",
                            configMINIMAL_STACK_SIZE,
                            NULL,
                            _1ST_PRIORITIE_RANK ,
                            &BTN2_Task_Handler
                          );
							xTaskCreate(  Periodic_Task,
                            "Periodic_Task",
                            configMINIMAL_STACK_SIZE,
                            NULL,
                            _1ST_PRIORITIE_RANK ,
                            &Periodic_Task_Handler
                          );
							xTaskCreate(  UART_Task,
                            "UART_Task",
                            configMINIMAL_STACK_SIZE,
                            NULL,
                            _1ST_PRIORITIE_RANK ,
                            &UART_Task_Handler
                          );

		UART_queue_Handler = xQueueCreate (UART_QUEUE_LENGTH,sizeof( unsigned char )); 

	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/


