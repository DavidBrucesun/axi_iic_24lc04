/******************************************************************************
 *
 * File name: mio_emio.c
 * Data     : 2020.12.2
 *
 * Version  : 1.0
 * Author   : SW
 *****************************************************************************/

/*---- include files --------------------------------------------------------*/
#include "user.h"

#ifdef FreeRTOS
/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#endif

/* Xilinx includes. */
#include "xgpiops.h"
#include "mio_emio.h"
/*---- macro definition -----------------------------------------------------*/
//MIO and EMIO
#define GPIO_DEVICE_ID    XPAR_XGPIOPS_0_DEVICE_ID

XGpioPs Gpio;    // GPIO device driver instance

#ifdef FreeRTOS
static void psLED1Task( void *pvParameters );  //blink PS LED1
static void psLED2Task( void *pvParameters );  //blink PS LED2

static TaskHandle_t xPsLED1Task;
static TaskHandle_t xPsLED2Task;
#endif
/*---- private function -----------------------------------------------------*/
// initial Demo port
/*
 * Initialize MIO and EMIO GPIO
 */
int Gpio_Init(void)
{
	int status;
	XGpioPs_Config *ConfigPtr;

	// find GPIO device
	ConfigPtr = XGpioPs_LookupConfig(GPIO_DEVICE_ID);
	status = XGpioPs_CfgInitialize(&Gpio, ConfigPtr,
			ConfigPtr->BaseAddr);
	if(status != XST_SUCCESS){
		return XST_FAILURE;
	}
	// set pin direction: 0-input, 1-output
	XGpioPs_SetDirectionPin(&Gpio, PSLED1, 1);
	XGpioPs_SetDirectionPin(&Gpio, PSLED2, 1);

	// enable pin output: 0-disable, 1-enable
	XGpioPs_SetOutputEnablePin(&Gpio, PSLED1, 1);
	XGpioPs_SetOutputEnablePin(&Gpio, PSLED2, 1);

	// turn off all leds
	XGpioPs_WritePin(&Gpio, PSLED1, 0x1);
	XGpioPs_WritePin(&Gpio, PSLED2, 0x1);

	return XST_SUCCESS;
}

//control LED
void Led_Control(u32 led, u32 cmd)
{
	XGpioPs_WritePin(&Gpio, led, cmd);
}

#ifdef FreeRTOS
//blink PS LED1
static void psLED1Task( void *pvParameters )
{
	int ledStatus = 1;
	while (1)
	{
		Led_Control(PSLED1, ledStatus);
		ledStatus = ~ledStatus;
		vTaskDelay(50);  //delay 500ms
	}
	
}

//blink PS LED2
static void psLED2Task( void *pvParameters )
{
	int ledStatus = 1;
	while (1)
	{
		Led_Control(PSLED2, ledStatus);
		ledStatus = ~ledStatus;
		vTaskDelay(100);  //delay 1s
	}
}

//blink ps led
void BlinkLedTask( void *pvParameters )
{
	int status;

	status = Gpio_Init();

	if (status == XST_SUCCESS)
	{
		xTaskCreate( psLED1Task,
				     ( const char * ) "PSLED1",
				     configMINIMAL_STACK_SIZE,
				     NULL,
				     tskIDLE_PRIORITY,
				     &xPsLED1Task );
		
		xTaskCreate( psLED2Task,
				     ( const char * ) "PSLED2",
				     configMINIMAL_STACK_SIZE,
				     NULL,
				     tskIDLE_PRIORITY,
				     &xPsLED2Task );
	}

	vTaskDelete( NULL );
}
#endif
/*---- end of demo.c --------------------------------------------------------*/
