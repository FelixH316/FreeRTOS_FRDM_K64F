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
 * @file    MK64F12_Project.c
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
/* TODO: insert other definitions and declarations here. */
#define DELAY (0xffffff)

void vTask1 (void *pvParameters);
void vTask2 (void *pvParameters);
void vTaskFunction (void *pvParameters);

static unsigned long ulIdleCycleCount = 0;

const char *pcTextForTask1 = "Task 1 is running\r\n";
const char *pcTextForTask2 = "Task 2 is running\r\n";

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

    //xTaskCreate(vTask1, "Task 1", 1000, NULL, 1, NULL);

    //xTaskCreate(vTask2, "Task 2", 1000, NULL, 0, NULL);

    xTaskCreate(vTaskFunction, "Task 1", 1000, (void*) pcTextForTask1, configMAX_PRIORITIES-4, NULL);

    xTaskCreate(vTaskFunction, "Task 2", 1000, (void*) pcTextForTask2, configMAX_PRIORITIES-4, NULL);

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

void vTask1 (void *pvParameters){
	const char *pcTaskName = "Task 1 is running\r\n";
	//volatile unsigned long ul;

	for(;;)
	{
		PRINTF(pcTaskName);

		PRINTF("%d\n", ulIdleCycleCount);
		/*
		for(ul = 0; ul < DELAY; ul++)
		{

		}
		*/
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void vTask2 (void *pvParameters){
	const char *pcTaskName = "Task 2 is running\r\n";

	portTickType xLastWakeTime;				// Variable del tipo tick
	xLastWakeTime = xTaskGetTickCount();	// Func para alamcenar el tick actual

	for(;;)
	{
		PRINTF(pcTaskName);

		PRINTF("%d\n", ulIdleCycleCount);
		//vTaskDelay(pdMS_TO_TICKS(2000));

		//vTaskDelayUntil(&xLastWakeTime, 400);	// (Valor actual de tick), (2 seg/Ttick)
		//vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2000));
	}
}

void vTaskFunction (void *pvParameters){
	char *pcTaskName;
	//volatile unsigned long ul;

	pcTaskName = (char*) pvParameters; // Casteamos el parametro recibido

	for(;;)
	{
		PRINTF(pcTaskName);
		/*
		for(ul = 0; ul < DELAY; ul++)
		{

		}
		*/
		//vTaskDelay(500);	// Delay de 2.5 seg
		taskYIELD();
	}
}

/* Idle hook functions MUST be called vApplicationIdleHook(), take no parameters,
 * and return void. */
void vApplicationIdleHook(void)
{
	/* This hook function does nothing but increment a counter. */
	ulIdleCycleCount++;
	PRINTF("Task IDLE is running\r\n");
}
