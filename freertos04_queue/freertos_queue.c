/*** RTOS CLASS 13, 14 & 15 Fix ***/
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
 * @file    freertos_queue.c
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
	uint8_t id;
	uint32_t data;
} queueData_t;

static void rxTask(void *parameters);
static void txTask(void *parameters);

xQueueHandle queueTxRx;

static const queueData_t dataTX [2] =
{
		{1, 111},
		{2, 222}
};

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

    queueTxRx = xQueueCreate(5, sizeof(queueData_t));

    vQueueAddToRegistry(queueTxRx, "Queue Test");	// Agregar la queueTxRx al Queue List

    xTaskCreate(txTask, "TX 1 task", 500, (void*) &(dataTX[0]), 1, NULL);

    xTaskCreate(txTask, "TX 2 task", 500, (void*) &(dataTX[1]), 1, NULL);

    xTaskCreate(rxTask, "RX task", 500, NULL, 2, NULL);

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

/*
// OPCION FELIX
static void rxTask(void *parameters)
{
	queueData_t dataRXQ; 	// Variable para el dato de la queue
	BaseType_t statusQueue;

	for(;;)
	{
		statusQueue = xQueueReceive(queueTxRx, &dataRXQ, portMAX_DELAY);

		if (statusQueue == pdTRUE)
		{
			PRINTF("\rFrom TX task %d = %d\n", dataRXQ.id, dataRXQ.data);
		}
		else
		{
			PRINTF("\rData not received\n");
		}
	}
}

static void txTask(void *parameters)
{
	queueData_t *data2TX;				// Parametro que vamos a recibir

	data2TX = (queueData_t*) parameters;	// Casteo de void a int

	for(;;)
	{
		xQueueSend(queueTxRx, data2TX, 0); // Se tiene que poner sin ampersand porque ya es un puntero
		//taskYIELD();
		vTaskDelay(500);
	}
}
// FIN OPCION FELIX
*/

//OPCION SERNA
static void rxTask(void *parameters)
{
	queueData_t *dataRXQ; 	// Variable para el dato de la queue
	BaseType_t statusQueue;

	for(;;)
	{
		statusQueue = xQueueReceive(queueTxRx, &dataRXQ, portMAX_DELAY);

		if (statusQueue == pdTRUE)
		{
			PRINTF("\rFrom TX task %d = %d\n", dataRXQ->id, dataRXQ->data);
		}
		else
		{
			PRINTF("\rData not received\n");
		}
	}
}

static void txTask(void *parameters)
{
	queueData_t *data2TX;				// Parametro que vamos a recibir

	data2TX = (queueData_t*) parameters;	// Casteo de void a int

	for(;;)
	{
		xQueueSend(queueTxRx, &data2TX, 0); // Se tiene que poner sin ampersand porque ya es un puntero
		//taskYIELD();
		vTaskDelay(500);
	}
}
// FIN OPCION SERNA
