/*** RTOS CLASS 17 ***/
/*
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * @file    main_T3A_RTOS.c
 * @brief   Send lower-case and receive upper-case letter via
 	 	 	UART0 on NXP FRDM K64 with FreeRTOS (3 tasks).
 * @author	Félix Armenta A - PADTS IECA 3
 * @date	Aug 10th, 2021
 * @vers	v0.1
 */

//#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"

/*** INCLUDE FILES ***/
// For FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// For UART0
#include "fsl_uart.h"

/*** OTHER DEFINITIONS AND DECLARATIONS ***/
// For UART0
#define UART_BAUDRATE 		115200
#define UART0_RX 			16U			// PTB
#define UART0_TX 			17U			// PTB
#define RING_BUFFER_SIZE 	16
#define SEMIHOST			1
#define DEBUG_TEST			0

uint8_t UartRingBuffer[RING_BUFFER_SIZE];
volatile uint8_t txIndex = 0;
volatile uint8_t rxIndex = 0;

// For Queues
#define GET_ARGS(args,type) *((type*)args)

typedef struct {
	QueueHandle_t q1;
	QueueHandle_t q2;
}twoQueues_t;

// For Tasks
void task_1 (void* args);
void task_2 (void* args);
void task_3 (void* args);

void UART0_RX_TX_IRQHandler(void)
{
	uint8_t data;

	if ((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & UART_GetStatusFlags(UART0))
	{
		data = UART_ReadByte(UART0);

		if (((rxIndex + 1) % RING_BUFFER_SIZE) != txIndex)
		{
			UartRingBuffer[rxIndex] = data;
			rxIndex++;
			rxIndex %= RING_BUFFER_SIZE;
		}
	}
	SDK_ISR_EXIT_BARRIER;
}

int main(void) {
    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

#if SEMIHOST
   	//UART0 INIT
	CLOCK_EnableClock(kCLOCK_PortB);                           	// Port B Clock Gate Control: Clock enabled

	PORT_SetPinMux(PORTB, UART0_RX, kPORT_MuxAlt3);           	// PORTB16 (pin 62) is configured as UART0_RX
	PORT_SetPinMux(PORTB, UART0_TX, kPORT_MuxAlt3);           	// PORTB17 (pin 63) is configured as UART0_TX
	SIM->SOPT5 = ((SIM->SOPT5 &
			(~(SIM_SOPT5_UART0TXSRC_MASK)))                     // Mask bits to zero which are setting
			| SIM_SOPT5_UART0TXSRC(SOPT5_UART0TXSRC_UART_TX)	// UART 0 transmit data source select: UART0_TX pin
	);

    uart_config_t UARTconfig;

    UART_GetDefaultConfig(&UARTconfig);
    UARTconfig.baudRate_Bps = UART_BAUDRATE;
    UARTconfig.enableTx     = true;
    UARTconfig.enableRx     = true;

    UART_Init(UART0, &UARTconfig, CLOCK_GetFreq(UART0_CLK_SRC));
#endif

    /* Send g_tipString out. */
    const uint8_t exampleString[46] = "\n\rUART0 is working from UART_WriteBlocking\n";
    UART_WriteBlocking(UART0, exampleString, sizeof(exampleString) / sizeof(exampleString[0]));

    /* Enable RX interrupt. */
    UART_EnableInterrupts(UART0, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable);
    EnableIRQ(UART0_RX_TX_IRQn);

    PRINTF("\rAnd also UART0 is working from debug console\r\n");

    static QueueHandle_t queue1;
    static QueueHandle_t queue2;

    queue1 = xQueueCreate(9, sizeof(uint8_t));
    queue2 = xQueueCreate(9, sizeof(uint8_t));

    vQueueAddToRegistry(queue1, "LowerCase Queue");
    vQueueAddToRegistry(queue2, "UpperCase Queue");

    twoQueues_t tasksParam = {queue1, queue2};

    xTaskCreate(task_1, "Task 1", (configMINIMAL_STACK_SIZE*2), (void*)&tasksParam, (configMAX_PRIORITIES-4), NULL);
    xTaskCreate(task_2, "Task 2", (configMINIMAL_STACK_SIZE*2), (void*)&tasksParam, (configMAX_PRIORITIES-3), NULL);
    xTaskCreate(task_3, "Task 3", (configMINIMAL_STACK_SIZE*2), (void*)&tasksParam, (configMAX_PRIORITIES-2), NULL);

    vTaskStartScheduler();

    /* Force the counter to be placed into memory. */
    volatile static int i = 0 ;
    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {
        i++ ;
        /* 'Dummy' NOP to allow source level single stepping of
            tight while() loop */
        __asm volatile ("nop");
    }
    return 0 ;
}


void task_1 (void* args)
{
	twoQueues_t t1;
	t1 = *((twoQueues_t*) args);
	uint8_t letter = 0;

	for(;;)
	{
		if(rxIndex != txIndex)
		{
#if DEBUG_TEST
			PRINTF("\r\n%c", UartRingBuffer[txIndex]);
#endif
			letter = UartRingBuffer[txIndex];
			txIndex++;
			txIndex %= RING_BUFFER_SIZE;

			if ((letter > 96) && (letter < 123))
			{
				xQueueSend(t1.q1, &letter, portMAX_DELAY);
			}
		}
	}
}

void task_2 (void* args)
{
	twoQueues_t t2;
	t2 = *((twoQueues_t*) args);
	uint8_t letter = 0;

	for(;;)
	{
		xQueueReceive(t2.q1, &letter, portMAX_DELAY);
#if DEBUG_TEST
		PRINTF("\t%c", letter);
#endif
		letter -= 0x20;
#if DEBUG_TEST
		PRINTF("\t%c\n", letter);
#endif
		xQueueSend(t2.q2, &letter, portMAX_DELAY);
	}
}

void task_3 (void* args)
{
	twoQueues_t t3;
	t3 = *((twoQueues_t*) args);
	uint8_t msg[4] = {'\r', '\t', 0, '\n'};

	for(;;)
	{
		xQueueReceive(t3.q2, &msg[2], portMAX_DELAY);
		UART_WriteBlocking(UART0, msg, sizeof(msg));
	}
}
