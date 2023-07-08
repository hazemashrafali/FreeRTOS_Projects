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
#define NOTIFY_VALUE	0x01
#define TRUE	1
#define FALSE	0
#define MAX_LOOP_INDEX		10
#define HEAVY_LOAD_CYCLE	100000
#define WAIT1_DELAY	5
#define WAIT2_DELAY	2
#define TASK1_DELAY	100
#define TASK2_DELAY	500

/*PRIORITIE TYPEs*/
#define _1ST_PRIORITIE_RANK		1
#define _2ND_PRIORITIE_RANK		2

/*Tasks Handlers*/
TaskHandle_t  UART_Task1_Handler = NULL;
TaskHandle_t  UART_Task2_Handler = NULL;
SemaphoreHandle_t UART_semaphore = NULL;

/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/


/*task to be created*/
void UART_Task1 (void * pvParameters)
{
	uint8_t u8_index = 0;
	for( ;; )
    {
			if(UART_semaphore != NULL)
			{
				 /* See if we can obtain the semaphore.  If the semaphore is not
						available wait 10 ticks to see if it becomes free. */
					//portMAX_DELAY 
				if( xSemaphoreTake( UART_semaphore,  portMAX_DELAY ) == pdTRUE )
				{
						GPIO_write(PORT_0,PIN2,PIN_IS_HIGH);
						while ( vSerialPutString( ( signed char * ) "-----TASK1-----\n", sizeof("-----TASK1-----\n")) == pdFALSE);
				
						for ( u8_index = 0; u8_index < MAX_LOOP_INDEX; u8_index++ )
						{
							xSerialPutChar (u8_index + '0');
							vTaskDelay( WAIT2_DELAY );
							while(vSerialPutString( ( signed char * ) "-this is task 1 string\n", sizeof("-this is task 1 string\n") ) == pdFALSE);
						}
						GPIO_write(PORT_0,PIN2,PIN_IS_LOW);
					xSemaphoreGive( UART_semaphore );
				}
				else
				{
					   /* We could not obtain the semaphore and can therefore not access
            the shared resource safely. */
				}		
			}
			vTaskDelay( TASK1_DELAY );
    }
}

void UART_Task2 (void * pvParameters)
{
	uint8_t u8_index = 0;
	uint32_t u32_index = 0;
	for( ;; )
    {
			if(UART_semaphore != NULL)
			{
				 /* See if we can obtain the semaphore.  If the semaphore is not
        available wait max ticks to see if it becomes free. */
				if( xSemaphoreTake( UART_semaphore, portMAX_DELAY ) == pdTRUE )
				{
						GPIO_write(PORT_0,PIN1,PIN_IS_HIGH);
						while ( vSerialPutString( ( signed char * ) "-----TASK2-----\n", sizeof("-----TASK2-----\n") )== pdFALSE);			
						for ( u8_index = 0; u8_index < MAX_LOOP_INDEX; u8_index++ )
						{
							xSerialPutChar (u8_index + '0');
							vTaskDelay( WAIT2_DELAY );
							while( vSerialPutString( ( signed char * ) "-this is task 2 string\n", sizeof("-this is task 2 string\n") ) == pdFALSE);
							/*heavy load simulation*/
							for(u32_index=0; u32_index < HEAVY_LOAD_CYCLE; u32_index++);
						}
						GPIO_write(PORT_0,PIN1,PIN_IS_LOW);
					xSemaphoreGive( UART_semaphore );
				}
				else
				{
					   /* We could not obtain the semaphore and can therefore not access
            the shared resource safely. */
				}
			}
			vTaskDelay( TASK2_DELAY );
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
	
	 /* Create the semaphore to guard a shared resource.*/
	UART_semaphore = xSemaphoreCreateBinary();
	
	/*
	The semaphore is created in the 'empty' state, meaning the semaphore must first be
	given using the xSemaphoreGive() API function before it can subsequently be taken
	(obtained) using the xSemaphoreTake() function.
	*/
	xSemaphoreGive(UART_semaphore);
	
    /* Create Tasks here */
							xTaskCreate(  UART_Task1,
                            "UART_Task1",
                            configMINIMAL_STACK_SIZE,
                            NULL,
                            _2ND_PRIORITIE_RANK ,
                            &UART_Task1_Handler
                          );
							xTaskCreate(  UART_Task2,
                            "UART_Task2",
                            configMINIMAL_STACK_SIZE,
                            NULL,
                            _1ST_PRIORITIE_RANK ,
                            &UART_Task2_Handler
                          );
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


