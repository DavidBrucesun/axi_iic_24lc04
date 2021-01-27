/******************************************************************************
 *
 * File name: main.c
 * Data     : 2021.01.27
 *
 * Version  : 1.0
 * Author   : SW
 *****************************************************************************/

/*---- includes file --------------------------------------------------------*/
/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
/* Xilinx includes. */
#include "xil_printf.h"
#include "xparameters.h"
/* Device includes. */
#include "hw_init/mio_emio.h"
#include "hw_init/axi_iic.h"
/*-----------------------------------------------------------*/
static TaskHandle_t xLedTask;
static TaskHandle_t xAxiIicTask;

int main(void)
{
	xil_printf("Hello from Freertos example main\r\n");

	xTaskCreate(BlinkLedTask,
		(const char*)"LED",
		configMINIMAL_STACK_SIZE,
		NULL,
		tskIDLE_PRIORITY,
		&xLedTask);

	xTaskCreate(Axi_Iic_Task,
		(const char*)"AxiIic",
		configMINIMAL_STACK_SIZE,
		NULL,
		tskIDLE_PRIORITY + 1,
		&xAxiIicTask);

	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	/* If all is well, the scheduler will now be running, and the following line
	will never be reached.  If the following line does execute, then there was
	insufficient FreeRTOS heap memory available for the idle and/or timer tasks
	to be created.  See the memory management section on the FreeRTOS web site
	for more details. */
	for (;; );
}

/*---- end of mian.c --------------------------------------------------------*/
