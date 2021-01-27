/******************************************************************************
 *
 * File name: axi_iic.c
 * Data     : 2021.01.27
 * Description :
 * AXI-IIC port for 24LC04 EEPROM, sw, 2021.01.27
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
#include "xiic.h"
#include "xil_printf.h"
#include "xscugic.h"

/* Device includes. */
#include "axi_iic.h"

/************************** Constant Definitions *****************************/
#define IIC_DEVICE_ID	XPAR_IIC_0_DEVICE_ID

#define INTC_DEVICE_ID		XPAR_SCUGIC_SINGLE_DEVICE_ID
#define IIC_INTR_ID		XPAR_FABRIC_IIC_0_VEC_ID

#ifndef FreeRTOS
 #define INTC			 	XScuGic
 #define INTC_HANDLER		XScuGic_InterruptHandler
#endif // !FreeRTOS

/*
 * The following constant defines the address of the IIC Slave device on the
 * IIC bus. Note that since the address is only 7 bits, this constant is the
 * address divided by 2.
 */
#define EEPROM_ADDRESS 		0x50	/* 0xA0 as an 8 bit number. */

/*
 * The page size determines how much data should be written at a time.
 * The write function should be called with this as a maximum byte count.
 */
#define PAGE_SIZE   16

/*
 * The Starting address in the IIC EEPROM on which this test is performed.
 */
#define EEPROM_TEST_START_ADDRESS   128

/**************************** Type Definitions *******************************/

/*
 * The AddressType should be u8 as the address pointer in the on board EEPROM is
 * 1 bytes.
 */
typedef u8 AddressType;

/***************** Macros (Inline Functions) Definitions *********************/

/************************** Function Prototypes ******************************/

static int SetupInterruptSystem(XIic* IicInstPtr);

int EepromWriteData(u16 ByteCount);  //write data to eeprom

int EepromReadData(u8 *BufferPtr, u16 ByteCount);  //read data from eeprom

static void SendHandler(XIic *InstancePtr);

static void ReceiveHandler(XIic *InstancePtr);

static void StatusHandler(XIic *InstancePtr, int Event);

/************************** Variable Definitions *****************************/

XIic IicInstance;	/* The instance of the IIC device. */

#ifndef FreeRTOS
INTC Intc; 	/* The instance of the Interrupt Controller Driver */
#else
extern XScuGic xInterruptController;
static XScuGic *Intc = &xInterruptController;/* The Instance of the Interrupt Controller Driver */
#endif  //!FreeRTOS
/*
 * Write buffer for writing a page.
 */
u8 WriteBuffer[sizeof(AddressType) + PAGE_SIZE];

u8 ReadBuffer[PAGE_SIZE];	/* Read buffer for reading a page. */

volatile u8 TransmitComplete;	/* Flag to check completion of Transmission */
volatile u8 ReceiveComplete;	/* Flag to check completion of Reception */

u8 EepromIicAddr;		/* Variable for storing Eeprom IIC address */
/************************** Function Definitions *****************************/

/*****************************************************************************/
/**
* AXI-IIC initial function for EEPROM.
*
* @param	None.
*
* @return	XST_SUCCESS if successful else XST_FAILURE.
*
* @note		None.
*
******************************************************************************/
int Axi_Iic_Init(void)
{
	int Status;
	XIic_Config* ConfigPtr;	/* Pointer to configuration data */

	/*
	 * Initialize the IIC driver so that it is ready to use.
	 */
	ConfigPtr = XIic_LookupConfig(IIC_DEVICE_ID);
	if (ConfigPtr == NULL) {
		return XST_FAILURE;
	}

	Status = XIic_CfgInitialize(&IicInstance, ConfigPtr,
		ConfigPtr->BaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Setup the Interrupt System.
	 */
	Status = SetupInterruptSystem(&IicInstance);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Set the Handlers for transmit and reception.
	 */
	XIic_SetSendHandler(&IicInstance, &IicInstance,
				(XIic_Handler) SendHandler);
	XIic_SetRecvHandler(&IicInstance, &IicInstance,
				(XIic_Handler) ReceiveHandler);
	XIic_SetStatusHandler(&IicInstance, &IicInstance,
				  (XIic_StatusHandler) StatusHandler);

	return XST_SUCCESS;
}
/*****************************************************************************/
/**
* This function writes a buffer of data to the IIC serial EEPROM.
*
* @param	ByteCount contains the number of bytes in the buffer to be
*		written.
*
* @return	XST_SUCCESS if successful else XST_FAILURE.
*
* @note		The Byte count should not exceed the page size of the EEPROM as
*		noted by the constant PAGE_SIZE.
*
******************************************************************************/
int EepromWriteData(u16 ByteCount)
{
	int Status;

	/*
	 * Set the defaults.
	 */
	TransmitComplete = 1;
	IicInstance.Stats.TxErrors = 0;

	/*
	 * Start the IIC device.
	 */
	Status = XIic_Start(&IicInstance);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Send the Data.
	 */
	Status = XIic_MasterSend(&IicInstance, WriteBuffer, ByteCount);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Wait till the transmission is completed.
	 */
	while ((TransmitComplete) || (XIic_IsIicBusy(&IicInstance) == TRUE)) {
		/*
		 * This condition is required to be checked in the case where we
		 * are writing two consecutive buffers of data to the EEPROM.
		 * The EEPROM takes about 2 milliseconds time to update the data
		 * internally after a STOP has been sent on the bus.
		 * A NACK will be generated in the case of a second write before
		 * the EEPROM updates the data internally resulting in a
		 * Transmission Error.
		 */
		if (IicInstance.Stats.TxErrors != 0) {


			/*
			 * Enable the IIC device.
			 */
			Status = XIic_Start(&IicInstance);
			if (Status != XST_SUCCESS) {
				return XST_FAILURE;
			}


			if (!XIic_IsIicBusy(&IicInstance)) {
				/*
				 * Send the Data.
				 */
				Status = XIic_MasterSend(&IicInstance,
							 WriteBuffer,
							 ByteCount);
				if (Status == XST_SUCCESS) {
					IicInstance.Stats.TxErrors = 0;
				}
				else {
				}
			}
		}
	}

	/*
	 * Stop the IIC device.
	 */
	Status = XIic_Stop(&IicInstance);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
* This function reads data from the IIC serial EEPROM into a specified buffer.
*
* @param	BufferPtr contains the address of the data buffer to be filled.
* @param	ByteCount contains the number of bytes in the buffer to be read.
*
* @return	XST_SUCCESS if successful else XST_FAILURE.
*
* @note		None.
*
******************************************************************************/
int EepromReadData(u8 *BufferPtr, u16 ByteCount)
{
	int Status;
	AddressType Address = EEPROM_TEST_START_ADDRESS;

	/*
	 * Set the Defaults.
	 */
	ReceiveComplete = 1;

	/*
	 * Position the Pointer in EEPROM.
	 */
	if (sizeof(Address) == 1) {
		WriteBuffer[0] = (u8) (EEPROM_TEST_START_ADDRESS);
	}
	else {
		WriteBuffer[0] = (u8) (EEPROM_TEST_START_ADDRESS >> 8);
		WriteBuffer[1] = (u8) (EEPROM_TEST_START_ADDRESS);
	}

	Status = EepromWriteData(sizeof(Address));
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Start the IIC device.
	 */
	Status = XIic_Start(&IicInstance);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Receive the Data.
	 */
	Status = XIic_MasterRecv(&IicInstance, BufferPtr, ByteCount);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Wait till all the data is received.
	 */
	while ((ReceiveComplete) || (XIic_IsIicBusy(&IicInstance) == TRUE)) {

	}

	/*
	 * Stop the IIC device.
	 */
	Status = XIic_Stop(&IicInstance);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
* This function setups the interrupt system so interrupts can occur for the
* IIC device. The function is application-specific since the actual system may
* or may not have an interrupt controller. The IIC device could be directly
* connected to a processor without an interrupt controller. The user should
* modify this function to fit the application.
*
* @param	IicInstPtr contains a pointer to the instance of the IIC device
*		which is going to be connected to the interrupt controller.
*
* @return	XST_SUCCESS if successful else XST_FAILURE.
*
* @note		None.
*
******************************************************************************/
static int SetupInterruptSystem(XIic* IicInstPtr)
{
	int Status;

	XScuGic_Config* IntcConfig;

	/*
	 * Initialize the interrupt controller driver so that it is ready to
	 * use.
	 */
	IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
	if (NULL == IntcConfig) {
		return XST_FAILURE;
	}

#ifndef FreeRTOS

	Status = XScuGic_CfgInitialize(&Intc, IntcConfig,
		IntcConfig->CpuBaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	XScuGic_SetPriorityTriggerType(&Intc, IIC_INTR_ID,
			0xA0, 0x3);

	/*
	 * Connect the interrupt handler that will be called when an
	 * interrupt occurs for the device.
	 */
	Status = XScuGic_Connect(&Intc, IIC_INTR_ID,
		(Xil_InterruptHandler)XIic_InterruptHandler,
		IicInstPtr);
	if (Status != XST_SUCCESS) {
		return Status;
	}

	/*
	 * Enable the interrupt for the IIC device.
	 */
	XScuGic_Enable(&Intc, IIC_INTR_ID);

	/*
	 * Initialize the exception table and register the interrupt
	 * controller handler with the exception table
	 */
	Xil_ExceptionInit();

	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
		(Xil_ExceptionHandler)INTC_HANDLER, &Intc);

	/* Enable non-critical exceptions */
	Xil_ExceptionEnable();

#else

	XScuGic_SetPriorityTriggerType(Intc, IIC_INTR_ID,
		0xA0, 0x3);

	/*
	 * Connect the interrupt handler that will be called when an
	 * interrupt occurs for the device.
	 */
	Status = XScuGic_Connect(Intc, IIC_INTR_ID,
		(Xil_InterruptHandler)XIic_InterruptHandler,
		IicInstPtr);
	if (Status != XST_SUCCESS) {
		return Status;
	}

	/*
	 * Enable the interrupt for the IIC device.
	 */
	XScuGic_Enable(Intc, IIC_INTR_ID);

#endif // FreeRTOS

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
* This Send handler is called asynchronously from an interrupt
* context and indicates that data in the specified buffer has been sent.
*
* @param	InstancePtr is not used, but contains a pointer to the IIC
*		device driver instance which the handler is being called for.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
static void SendHandler(XIic *InstancePtr)
{
	TransmitComplete = 0;
}

/*****************************************************************************/
/**
* This Receive handler is called asynchronously from an interrupt
* context and indicates that data in the specified buffer has been Received.
*
* @param	InstancePtr is not used, but contains a pointer to the IIC
*		device driver instance which the handler is being called for.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
static void ReceiveHandler(XIic *InstancePtr)
{
	ReceiveComplete = 0;
}

/*****************************************************************************/
/**
* This Status handler is called asynchronously from an interrupt
* context and indicates the events that have occurred.
*
* @param	InstancePtr is a pointer to the IIC driver instance for which
*		the handler is being called for.
* @param	Event indicates the condition that has occurred.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
static void StatusHandler(XIic *InstancePtr, int Event)
{

}

//axi-iic task
void Axi_Iic_Task(void* pvParameters)
{
	u32 Index;
	int Status;
	AddressType Address = EEPROM_TEST_START_ADDRESS;
	EepromIicAddr = EEPROM_ADDRESS;

    Status = Axi_Iic_Init();
	if(Status != XST_SUCCESS){
		xil_printf("AXI-IIC initialize failed!\r\n");
		vTaskDelete(NULL);
	}
	xil_printf("AXI-IIC initialize success!\r\n");

    /*
	 * Initialize the data to write and the read buffer.
	 */
	if (sizeof(Address) == 1) {
		WriteBuffer[0] = (u8) (EEPROM_TEST_START_ADDRESS);
		EepromIicAddr |= (EEPROM_TEST_START_ADDRESS >> 8) & 0x7;
	} else {
		WriteBuffer[0] = (u8) (EEPROM_TEST_START_ADDRESS >> 8);
		WriteBuffer[1] = (u8) (EEPROM_TEST_START_ADDRESS);
		ReadBuffer[Index] = 0;
	}

	for (Index = 0; Index < PAGE_SIZE; Index++) {
		WriteBuffer[sizeof(Address) + Index] = 0xFF;
		ReadBuffer[Index] = 0;
	}

	/*
	 * Set the Slave address.
	 */
	Status = XIic_SetAddress(&IicInstance, XII_ADDR_TO_SEND_TYPE,
				 EepromIicAddr);
	if (Status != XST_SUCCESS) {
		xil_printf("Set slave address failed!\r\n");
		vTaskDelete(NULL);
	}


	/*
	 * Write to the EEPROM.
	 */
	Status = EepromWriteData(sizeof(Address) + PAGE_SIZE);
	if (Status != XST_SUCCESS) {
		xil_printf("Write data to EEPROM failed!\r\n");
		vTaskDelete(NULL);
	}

	/*
	 * Read from the EEPROM.
	 */
	Status = EepromReadData(ReadBuffer, PAGE_SIZE);
	if (Status != XST_SUCCESS) {
		xil_printf("Read data from EEPROM failed!\r\n");
		vTaskDelete(NULL);
	}

	/*
	 * Verify the data read against the data written.
	 */
	for (Index = 0; Index < PAGE_SIZE; Index++) {
		if (ReadBuffer[Index] != WriteBuffer[Index + sizeof(Address)]) {
			xil_printf("IIC eeprom Example Failed!\r\n");
		    vTaskDelete(NULL);
		}

		ReadBuffer[Index] = 0;
	}

	xil_printf("Successfully ran IIC eeprom Example!\r\n");
	vTaskDelete(NULL);
}
/*---- end of axi_iic.c -----------------------------------------------------*/
