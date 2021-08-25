/*** RTOS CLASS 14 (Theory) & 15 (Code) ***/
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

/**
 * @file    queue04.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"

/* TODO: insert other include files here. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* TODO: insert other definitions and declarations here. */
typedef struct
{
	uint32_t inc_counter;
	uint32_t dec_counter;
} msg_t;

void task_producer(void *args);

void task_consumer(void *args);

/*
 * @brief   Application entry point.
 */
int main(void) {

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    PRINTF("Hello World\n");

    static QueueHandle_t mailbox;

	mailbox = xQueueCreate(10,sizeof(msg_t*));

	vQueueAddToRegistry(mailbox, "Queue mailbox");

	xTaskCreate(task_consumer, "consumer", 110, (void*)&mailbox, 1, NULL);
	xTaskCreate(task_producer, "producer", 110, (void*)&mailbox, 1, NULL);

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

void task_producer(void *args)
{
//	QueueHandle_t mailbox = GET_ARGS(args,QueueHandle_t);
	QueueHandle_t mailbox = *((QueueHandle_t*)args);
	msg_t msg = {0,0xffff};
	msg_t *pmsg;
	for(;;)
	{
		msg.inc_counter++;
		msg.dec_counter--;
		pmsg = pvPortMalloc(sizeof(msg_t));
		*pmsg = msg;
		xQueueSend(mailbox,&pmsg,portMAX_DELAY);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void task_consumer(void *args)
{
//	QueueHandle_t mailbox = GET_ARGS(args,QueueHandle_t);
	QueueHandle_t mailbox = *((QueueHandle_t*)args);
	msg_t *pmsg;
	for(;;)
	{
		xQueueReceive(mailbox,&pmsg,portMAX_DELAY);
		PRINTF("\rMessage received| inc: %i dec: %i\n",pmsg->inc_counter,pmsg->dec_counter);
		vPortFree(pmsg);
	}
}
