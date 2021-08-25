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
 * @file    fr13_demo_sysView.c
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
#include "semphr.h"		// Agregamos libreria de semaforos

/* TODO: insert other definitions and declarations here. */
#define DWR_CTRL (*(volatile uint32_t*)0xE0001000)

SemaphoreHandle_t xMutex;

const TickType_t xMaxBlockTimeTicks = 0x20;

#define BOARD_SW_GPIO BOARD_SW3_GPIO
#define BOARD_SW_PORT BOARD_SW3_PORT
#define BOARD_SW_GPIO_PIN BOARD_SW3_GPIO_PIN
#define BOARD_SW_IRQ BOARD_SW3_IRQ

static void prvPrintTask(void *pvParameters);

static void prvNewPrintString(const char *pcString);

SemaphoreHandle_t semphr1;

/*
 * @brief   Application entry point.
 */
int main(void)
{
    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    DWR_CTRL |= (1 << 0);

    SEGGER_SYSVIEW_Conf();
    SEGGER_SYSVIEW_Start();

    PRINTF("Hello World\n");

    SEGGER_SYSVIEW_Print("Hello World\n");

//    uint8_t x = 10;
//    SEGGER_SYSVIEW_PrintfTarget("Hello %d World\n", x);

    xMutex = xSemaphoreCreateMutex();

    xTaskCreate(prvPrintTask, "Print1", 1000,
    		"\n\rTask 1 *******************************", 1, NULL);

    xTaskCreate(prvPrintTask, "Print2", 1000,
        	"\n\rTask 2 *******************************", 1, NULL);

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

static void prvNewPrintString(const char *pcString)
{
	xSemaphoreTake(xMutex, portMAX_DELAY);
	PRINTF("%s", pcString);
	xSemaphoreGive(xMutex);
}

static void prvPrintTask(void *pvParameters)
{
	char *pcStringToPrint;
	const TickType_t xSlowDownDelay = pdMS_TO_TICKS(5UL);


	pcStringToPrint = (char*) pvParameters;

	for(;;)
	{
		prvNewPrintString(pcStringToPrint);

		vTaskDelay(rand() % xMaxBlockTimeTicks);

		vTaskDelay(xSlowDownDelay);
	}
}
