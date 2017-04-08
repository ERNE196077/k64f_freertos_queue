/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
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

/*System includes.*/
#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "board.h"

#include "pin_mux.h"
#include "clock_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define MAX_LOG_LENGTH 20
/*******************************************************************************
* Globals
******************************************************************************/
/* Logger queue handle */
static QueueHandle_t log_queue = NULL;

gpio_pin_config_t gpio_config_sw2 = {
		kGPIO_DigitalInput,
		0
};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* Logger API */
void log_init(uint32_t queue_length, uint32_t max_log_lenght);
static void log_task(void *pvParameters);
/*******************************************************************************
 * Code
 ******************************************************************************/




/*!
 * @brief Main function
 */
int main(void)
{
     BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    GPIO_PinInit (GPIOC, 6u, &gpio_config_sw2);

    /* Initialize logger for 10 logs with maximum lenght of one log 20 B */
    log_init(10, MAX_LOG_LENGTH);
    vTaskStartScheduler();
    for (;;)
        ;
}

/*******************************************************************************
 * Logger functions
 ******************************************************************************/
/*!
 * @brief log_init function
 */
void log_init(uint32_t queue_length, uint32_t max_log_lenght)
{
    log_queue = xQueueCreate(queue_length, max_log_lenght);
    xTaskCreate(log_task, "log_task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
}

/*!
 * @brief log_print_task function
 */
static void log_task(void *pvParameters)
{
    uint32_t counter = 0;
    char log[MAX_LOG_LENGTH + 1];
    while (1)
    {
        xQueueReceive(log_queue, log, portMAX_DELAY);
        PRINTF("Log %d: %s\r\n", counter, log);
        counter++;
    }
}


void PORTC_IRQHandler (void){
	BaseType_t xHigherPriorityTaskWoken;

	uint32_t ulCurrentInterrupt;
	__asm volatile( "mrs %0, ipsr" : "=r"( ulCurrentInterrupt ) );

	PRINTF("Interrupt SW2!!! %d. \r\n",ulCurrentInterrupt);
	char log[MAX_LOG_LENGTH + 1];
	 sprintf(log, "ISR Msg to log_task");
	 xQueueSendToBackFromISR(log_queue, &log, &xHigherPriorityTaskWoken);

	NVIC_DisableIRQ(PORTC_IRQn);
	PORTC->PCR[6] = ((PORTC->PCR[6] &
	          (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK))) /* Mask bits to zero which are setting */
	            | PORT_PCR_PS(0x1)                               /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the corresponding PE field is set. */
	            | PORT_PCR_PE(0x1)                          /* Pull Enable: Internal pullup or pulldown resistor is enabled on the corresponding pin, if the pin is configured as a digital input. */
				| PORT_PCR_ISF_MASK
				| PORT_PCR_IRQC(0xA)
	          );
	PORTC->ISFR = 0x0;
	NVIC_ClearPendingIRQ(PORTC_IRQn);
	NVIC_EnableIRQ(PORTC_IRQn);

	if( xHigherPriorityTaskWoken )
		{
			taskYIELD ();
		}


}
