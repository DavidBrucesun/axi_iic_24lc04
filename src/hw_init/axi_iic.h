/******************************************************************************
 *
 * File name: axi_iic.h
 * Data     : 2021.01.27
 *
 * Version  : 1.0
 * Author   : SW
 *****************************************************************************/
#ifndef __AXI_IIC_H_
#define __AXI_IIC_H_

 /*---- macro definition -----------------------------------------------------*/

// exported function
int Axi_Iic_Init(void);  // initial axi-iic port

void Axi_Iic_Task(void* pvParameters);  //axi-iic task

#endif
/*---- end of axi_iic.h -----------------------------------------------------*/