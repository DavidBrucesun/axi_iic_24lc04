/******************************************************************************
 *
 * File name: mio_emio.h
 * Data     : 2020.12.02
 *
 * Version  : 1.0
 * Author   : SW
 *****************************************************************************/
#ifndef __MIO_EMIO_H_
#define __MIO_EMIO_H_

/*---- macro definition -----------------------------------------------------*/
#define PSLED1    0   // LED D1, MIO0
#define PSLED2    13  // LED D2, MIO13

#define LED_ON    0  // GPIO turn low, LED turn on
#define LED_OFF   1  // GPIO turn high, LED turn off
// exported function
int Gpio_Init(void);  // initial demo port
void Led_Control(u32 led, u32 cmd);  //control LED

void BlinkLedTask( void *pvParameters );  //blink ps led

#endif
/*---- end of demo.h --------------------------------------------------------*/
