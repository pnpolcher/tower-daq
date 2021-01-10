/*
 * Copyright 2016-2020 NXP
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
 * @file    daq.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK40D10.h"
/* TODO: insert other include files here. */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"
#include "queue.h"


#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_dspi.h"
#include "fsl_dspi_freertos.h"
#include "fsl_gpio.h"

#include "ltc1859.h"
#include "ltc2498.h"
#include "twr_adcdac.h"

/* TODO: insert other definitions and declarations here. */

#define TWR_SPI_BASE (SPI0_BASE)
#define TWR_SPI_BASE_IRQN (SPI0_IRQn)

#define TWR_SPI_BASE_ADDRESS ((SPI_Type*)TWR_SPI_BASE)

#define TWR_SPI_CLK_SRC (DSPI0_CLK_SRC)
#define TWR_SPI_CLK_FREQ CLOCK_GetFreq((DSPI0_CLK_SRC))

#define DSPI_NVIC_PRIO 2

#define mainQUEUE_RECEIVE_TASK_PRIORITY     ( tskIDLE_PRIORITY + 2 )
#define mainQUEUE_SEND_TASK_PRIORITY        ( tskIDLE_PRIORITY + 1 )
#define mainEVENT_SEMAPHORE_TASK_PRIORITY   ( configMAX_PRIORITIES - 1 )

#define mainQUEUE_SEND_PERIOD_MS            pdMS_TO_TICKS( 200 )
#define mainSOFTWARE_TIMER_PERIOD_MS        pdMS_TO_TICKS( 1000 )

#define mainQUEUE_LENGTH                    ( 1 )

static void prvQueueReceiveTask(void *pvParameters);
static void vLtc1859SampleTimerCallback(TimerHandle_t xTimer);
static void vLtc2498SampleTimerCallback(TimerHandle_t xTimer);

void init_spi();
void init_gpio();

static QueueHandle_t xQueue = NULL;
static SemaphoreHandle_t xEventSemaphore = NULL;
static SemaphoreHandle_t xSpiSemaphore = NULL;
static TimerHandle_t xLtc1859SampleTimer = NULL;
static TimerHandle_t xLtc2498SampleTimer = NULL;

static dspi_rtos_handle_t twr_spi_handle;

uint16_t ltc1859_result;
uint32_t ltc2498_result;

/*
 * @brief   Application entry point.
 */
int main(void) {
  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    init_spi();

    xLtc1859SampleTimer = xTimerCreate(
		"Ltc1859SampleTimer",
		pdMS_TO_TICKS(1000),
		pdTRUE,
		(void *)0,
		vLtc1859SampleTimerCallback
	);

    xLtc2498SampleTimer = xTimerCreate(
		"Ltc2498SampleTimer",
		pdMS_TO_TICKS(1000),
		pdTRUE,
		(void *)0,
		vLtc2498SampleTimerCallback
	);

    xQueue = xQueueCreate(
		mainQUEUE_LENGTH,
		sizeof(uint32_t)
	);

    xEventSemaphore = xSemaphoreCreateBinary();
    xSpiSemaphore = xSemaphoreCreateMutex();

    xTaskCreate(
		prvQueueReceiveTask,
		(char *)"Rx",
		configMINIMAL_STACK_SIZE,
		NULL,
		mainQUEUE_RECEIVE_TASK_PRIORITY,
		NULL
	);


    printf("Hello World\n");

    xTimerStart(xLtc1859SampleTimer, 0);
    xTimerStart(xLtc2498SampleTimer, 0);

    vTaskStartScheduler();
    for(;;);
}

void init_spi()
{
	dspi_master_config_t masterConfig;
    status_t status;

	masterConfig.whichCtar = kDSPI_Ctar0;
	masterConfig.ctarConfig.baudRate = 1000U;
	masterConfig.ctarConfig.bitsPerFrame = 8;
	masterConfig.ctarConfig.cpol = kDSPI_ClockPolarityActiveHigh; // (CPOL = 0)
	masterConfig.ctarConfig.cpha = kDSPI_ClockPhaseFirstEdge; // (CPHA = 0)
	masterConfig.ctarConfig.direction = kDSPI_MsbFirst;

    masterConfig.whichPcs = kDSPI_Pcs0;
    masterConfig.pcsActiveHighOrLow = kDSPI_PcsActiveLow;

	NVIC_SetPriority(TWR_SPI_BASE_IRQN, DSPI_NVIC_PRIO + 1);

	status = DSPI_RTOS_Init(&twr_spi_handle, TWR_SPI_BASE_ADDRESS, &masterConfig, TWR_SPI_CLK_FREQ);
    if (status != kStatus_Success)
    {
        // PRINTF("DSPI master: error during initialization. \r\n");
        vTaskSuspend(NULL);
    }

	masterConfig.whichCtar = kDSPI_Ctar1;
	masterConfig.ctarConfig.bitsPerFrame = 16;

	DSPI_MasterInit(TWR_SPI_BASE_ADDRESS, &masterConfig, TWR_SPI_CLK_FREQ);
}

static void prvQueueReceiveTask(void *pvParameters)
{
	uint32_t ulReceivedValue;

	for(;;)
	{
		xQueueReceive(xQueue, &ulReceivedValue, portMAX_DELAY);
		if (ulReceivedValue == 100UL)
		{
			// Do something.
		}
	}
}

static void vLtc1859SampleTimerCallback(TimerHandle_t xTimer)
{
	dspi_transfer_t masterXfer;
	status_t status;

	if (xSemaphoreTake(xSpiSemaphore, (TickType_t)10) == pdTRUE)
	{
		uint16_t adc_command = (LTC1859_CH0 | LTC1859_UNIPOLAR_MODE | LTC1859_LOW_RANGE_MODE | LTC1859_NORMAL_MODE);

		printf("Sending command 0x%x to LTC1859.\n", adc_command);

		ltc1859_cs_enable();

		masterXfer.dataSize = 2;
		masterXfer.txData = (uint8_t *)&adc_command;
		masterXfer.rxData = (uint8_t *)&ltc1859_result;
		masterXfer.configFlags = kDSPI_MasterCtar0;
		status = DSPI_RTOS_Transfer(&twr_spi_handle, &masterXfer);

		if (status == kStatus_Success)
		{
			float voltage = ((float)ltc1859_result) / 65535.0f * 5.0f;
			printf("Value received = %x\n", ltc1859_result);
			printf("LTC1859 voltage = %d\n", (int32_t)(voltage * 1000.0f));
		}

		cs_disable();
		xSemaphoreGive(xSpiSemaphore);
	}
	else
	{

	}
}

static void vLtc2498SampleTimerCallback(TimerHandle_t xTimer)
{
	dspi_transfer_t masterXfer;
	status_t status;

	ltc2498_result = 0;
	if (xSemaphoreTake(xSpiSemaphore, (TickType_t)10) == pdTRUE)
	{

		//uint32_t adc_command = (LTC2498_CH0 << 24);// | (LTC2498_R50 << 16);
		// uint32_t adc_command = (LTC2498_R50 << 8) | LTC2498_CH0;
		uint32_t adc_command = 0x90B0;
		printf("Sending command 0x%x to LTC2498.\n", adc_command);

		ltc2498_cs_enable();

		masterXfer.dataSize = 4;
		masterXfer.txData = (uint8_t *)&adc_command;
		masterXfer.rxData = (uint8_t *)&ltc2498_result;
		masterXfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0 | kDSPI_MasterPcsContinuous;
		status = DSPI_RTOS_Transfer(&twr_spi_handle, &masterXfer);

		if (status == kStatus_Success)
		{
			ltc2498_result =
					((ltc2498_result & 0x000000ff) << 24) |
					((ltc2498_result & 0x0000ff00) << 8) |
					((ltc2498_result & 0x00ff0000) >> 8) |
					((ltc2498_result & 0xff000000) >> 24);
			float voltage = (float)(ltc2498_result - 0x20000000) / 268435456.0f * 2.5f;
			printf("LTC2498 voltage = %d\n", (int32_t)(voltage * 1000.0f));
		}

		cs_disable();
		xSemaphoreGive(xSpiSemaphore);
	}
	else
	{

	}
}
