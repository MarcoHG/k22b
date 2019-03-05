#include "board.h"
#include "fsl_uart.h"

#include "pin_mux.h"
#include "clock_config.h"
#include <stdio.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_UART UART1
#define DEMO_UART_CLKSRC SYS_CLK
#define DEMO_UART_CLK_FREQ CLOCK_GetFreq(SYS_CLK)
#define TX_BUFFER_LENGTH 128
#define RX_BUFFER_LENGTH 128

typedef struct _stTransferUart
{
uart_handle_t handle;
uart_config_t config;
UART_Type *base;

// Receiver
uint8_t rxByte;
uint8_t rxRingBuffer[RX_BUFFER_LENGTH];
uint16_t rxTail;
uint16_t rxHead;
uart_transfer_t rxTransfer;

// Transmitter
uint8_t txBuffer[TX_BUFFER_LENGTH];
uart_transfer_t txTransfer;

// Status
volatile bool txOnGoing;
volatile bool rxFlag;

} stTransferUart;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/* UART user callback */
void UART_DebugCallback(UART_Type *base, uart_handle_t *handle, status_t status,
    void *userData);
void UART_AuxCallback(UART_Type *base, uart_handle_t *handle, status_t status,
    void *userData);
bool getRxRingBuffer(stTransferUart *psUart, uint8_t *data);
int __sys_write(int iFileHandle, char *pcBuffer, int iLength);
int __sys_readc(void);
void initUarts(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
stTransferUart sDbgUartTransfer;
stTransferUart sAuxUartTransfer;

/*******************************************************************************
 * Code
 ******************************************************************************/
/* Debug UART user callback */
void UART_DebugCallback(UART_Type *base, uart_handle_t *handle, status_t status,
    void *userData)
{
	// userData = userData;	// ?
	stTransferUart *psUart = (stTransferUart *) userData;	//

	if (kStatus_UART_TxIdle == status)
	{
		psUart->txOnGoing = false;
	}

	if (kStatus_UART_RxIdle == status)
	{
		psUart->rxFlag = true;
		psUart->rxRingBuffer[psUart->rxHead++] = psUart->rxByte;
		psUart->rxHead %= RX_BUFFER_LENGTH;
		UART_TransferReceiveNonBlocking(base, handle, &psUart->rxTransfer, NULL);
	}
}

// Primitive to re-target printf
int __sys_write(int iFileHandle, char *pcBuffer, int iLength)
{
	// Assume we have enough buffer size
	assert(iLength < TX_BUFFER_LENGTH);	/// add assert later

	sDbgUartTransfer.txTransfer.data = sDbgUartTransfer.txBuffer;
	sDbgUartTransfer.txTransfer.dataSize = iLength;

	memcpy(sDbgUartTransfer.txBuffer, pcBuffer, iLength);
	sDbgUartTransfer.txOnGoing = true;
	UART_TransferSendNonBlocking(sDbgUartTransfer.base, &sDbgUartTransfer.handle,
	    &sDbgUartTransfer.txTransfer);
	return 0;	// printf is a streamed output - assume everything was sent to tx buffer
}
int __sys_readc(void)
{
	if (sDbgUartTransfer.rxHead == sDbgUartTransfer.rxTail)
		return EOF;
	int data = sDbgUartTransfer.rxRingBuffer[sDbgUartTransfer.rxTail++];
	sDbgUartTransfer.rxTail %= RX_BUFFER_LENGTH;

	return data;
}

/*!
 * @brief getRxRingBuffer Check and get character from the top of RX ring buffer
 *
 * Function checks for tail and head of received ring buffer member of the
 * transfer structure.
 *
 * @param psUart pointer to uart transfer structure with the rx buffer
 * @param data contains the next (fifo) character of the buffer
 *
 * @retval true  if a character is being returned
 * @retval false if there is no elemnt toget from the buffer
 */
bool getRxRingBuffer(stTransferUart *psUart, uint8_t *data)
{
	if (psUart->rxTail != psUart->rxHead)
	{
		*data = psUart->rxRingBuffer[psUart->rxTail++];
		psUart->rxTail %= RX_BUFFER_LENGTH;
		return true;
	}
	else
		return false;
}

/*!
 * @brief initTransferUart initializes the uart transfer structure members
 *
 * Initialization sets pointer members to the memory locations within the uart
 * transfer structure, creates the handle and starts the receiving mechanism.
 *
 * Function expects the psUart->config member to be set before the call
 *
 * @param psUart pointer to the UART transfer method structure
 * @param base the UART peripheral base address \sa K22: UART0 UART1 UART2
 * @param callback the pointer to the callback function, called in IRQ
 * @param clock UART clock source \sa K22: UART0,UART1 uses SYS_CLK and UART2 is BUS_CLK
 */
void initTransferUart(stTransferUart *psUart, UART_Type *base,
    uart_transfer_callback_t callback, clock_name_t clock)
{
	// Initialize bss
	psUart->txBuffer[0] = 0;
	psUart->rxRingBuffer[0] = 0;
	psUart->rxHead = 0;
	psUart->rxTail = 0;
	psUart->rxByte = 0;
	psUart->txOnGoing = false;
	psUart->rxFlag = false;
	psUart->base = base;

	assert(psUart->config.baudRate_Bps); // basic check

	UART_Init(psUart->base, &psUart->config, CLOCK_GetFreq(clock));


	UART_TransferCreateHandle(base, &psUart->handle, callback, psUart);

	// Prepare to receive ONE character
	psUart->rxTransfer.data = &psUart->rxByte;
	psUart->rxTransfer.dataSize = 1;

	UART_TransferReceiveNonBlocking(psUart->base, &psUart->handle,
	    &psUart->rxTransfer, NULL);

}

/* Auxiliary UART user callback */
void UART_AuxCallback(UART_Type *base, uart_handle_t *handle, status_t status,
    void *userData)
{
	// userData = userData;	// ?
	stTransferUart *psUart = (stTransferUart *) userData;	//

	if (kStatus_UART_TxIdle == status)
	{
		psUart->txOnGoing = false;
	}

	if (kStatus_UART_RxIdle == status)
	{
		psUart->rxFlag = true;
		psUart->rxRingBuffer[psUart->rxHead++] = psUart->rxByte;
		psUart->rxHead %= RX_BUFFER_LENGTH;
		UART_TransferReceiveNonBlocking(base, handle, &psUart->rxTransfer, NULL);
	}
}


/*!
 * @brief Main function
 */
int main(void)
{

	BOARD_InitPins();
	BOARD_BootClockRUN();

	// Init UART
	uart_config_t *pConfig;
	/*
	 * UART_GetDefaultConfig returns
	 * config.baudRate_Bps = 115200U;
	 * config.parityMode = kUART_ParityDisabled;
	 * config.stopBitCount = kUART_OneStopBit;
	 * config.txFifoWatermark = 0;
	 * config.rxFifoWatermark = 1;
	 * config.enableTx = false;
	 * config.enableRx = false;
	 */

	// Debug Uart = UART1
	pConfig = &sDbgUartTransfer.config;
	UART_GetDefaultConfig(pConfig);
	// Change from defaults
	pConfig->enableTx = true;
	pConfig->enableRx = true;
	pConfig->baudRate_Bps = 115200;
	initTransferUart(&sDbgUartTransfer, UART1, UART_DebugCallback, UART1_CLK_SRC);

	// Auxiliary Uart = UART0
	pConfig = &sAuxUartTransfer.config;
	UART_GetDefaultConfig(pConfig);
	// Change from defaults
	pConfig->enableTx = true;
	pConfig->enableRx = true;
	pConfig->baudRate_Bps = 9600;
	initTransferUart(&sAuxUartTransfer, UART0, UART_DebugCallback, UART0_CLK_SRC);


	printf("Debug UART - count rx chars, displays every 10\r");// Test retarget at UART1
	uint8_t ch, cha;
	uint16_t nbrRec = 0, nbrRecCpy = 0;

	// UART0 Auxiliary
	initTransferUart(&sAuxUartTransfer, UART0, UART_AuxCallback, SYS_CLK);

	uint8_t msg[] = "Hello from Auxiliary Uart - echo input\r";
	sAuxUartTransfer.txTransfer.data = sAuxUartTransfer.txBuffer;
	sAuxUartTransfer.txTransfer.dataSize = sizeof(msg) - 1;
	memcpy(sAuxUartTransfer.txBuffer, msg, sizeof(msg) - 1);
	sAuxUartTransfer.txOnGoing = true;
	UART_TransferSendNonBlocking(sAuxUartTransfer.base, &sAuxUartTransfer.handle,
	    &sAuxUartTransfer.txTransfer);

	while (1)
	{
		// Handle Debug Console
		while (getRxRingBuffer(&sDbgUartTransfer, &ch))
			++nbrRec;
		if (nbrRec != nbrRecCpy)
		{
			if(!(nbrRec %10))
				printf("Received %d\r", nbrRec);
			nbrRecCpy = nbrRec;
		}
		// Handle Aux
		if(getRxRingBuffer(&sAuxUartTransfer, &cha))
		{
			sAuxUartTransfer.txTransfer.data[0] 	= cha;
			sAuxUartTransfer.txTransfer.dataSize 	= 1;
			UART_TransferSendNonBlocking(sAuxUartTransfer.base, &sAuxUartTransfer.handle,
				    &sAuxUartTransfer.txTransfer);
		}

		// Simulate a long delay
		for (uint16_t i = 0; i < 65000U; ++i)
		{
			// Debug
		}

	}
}
