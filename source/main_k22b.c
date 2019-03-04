
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
#define TX_BUFFER_LENGTH 50
#define RX_BUFFER_LENGTH 128


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/* UART user callback */
void UART_UserCallback(UART_Type *base, uart_handle_t *handle, status_t status, void *userData);
void prepareRxDbgCharacter(void);
int __sys_write(int iFileHandle, char *pcBuffer, int iLength);
int __sys_readc(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
uart_handle_t g_uartDbgHandle;
uint8_t g_txDbgBuffer[TX_BUFFER_LENGTH] = {0};
uint8_t g_rxDbgByte = 0;
uint8_t g_rxDbgRingBuffer[RX_BUFFER_LENGTH] = {0};
uint16_t	rxDbgTail =0;
uint16_t	rxDbgHead =0;

volatile bool txDbgOnGoing = false;
volatile bool rxDbgFlag = false;

/*******************************************************************************
 * Code
 ******************************************************************************/
/* UART user callback */
void UART_UserCallback(UART_Type *base, uart_handle_t *handle, status_t status, void *userData)
{
    userData = userData;

    if (kStatus_UART_TxIdle == status)
    {
        txDbgOnGoing = false;
    }

    if (kStatus_UART_RxIdle == status)
    {
        rxDbgFlag = true;
        g_rxDbgRingBuffer[rxDbgHead++] = g_rxDbgByte;
        rxDbgHead %= RX_BUFFER_LENGTH;
        prepareRxDbgCharacter();
    }
}

// Primitive to re-target printf
int __sys_write(int iFileHandle, char *pcBuffer, int iLength)
{
	// Assume we have enough buffer size
	assert(iLength < TX_BUFFER_LENGTH);	/// add assert later
	static uart_transfer_t sendXfer;
	sendXfer.data 		=	g_txDbgBuffer;
	sendXfer.dataSize = iLength;
	memcpy(g_txDbgBuffer, pcBuffer, iLength);
	txDbgOnGoing = true;
	UART_TransferSendNonBlocking(DEMO_UART, &g_uartDbgHandle, &sendXfer);
	return 0;	// printf is a streamed output - assume everything was sent to tx buffer
}
int __sys_readc(void)
{
	if( rxDbgHead == rxDbgTail )
		return EOF;
	int data = g_rxDbgRingBuffer[rxDbgTail++];
	rxDbgTail %= RX_BUFFER_LENGTH;

	return data;
}

void prepareRxDbgCharacter(void)
{
	static uart_transfer_t receiveXfer;
	receiveXfer.data 			= &g_rxDbgByte;
	receiveXfer.dataSize 	= 1;	// Get notifications every char
	UART_TransferReceiveNonBlocking(DEMO_UART, &g_uartDbgHandle, &receiveXfer, NULL);

}

bool	getRxDbg(uint8_t *data)
{
	if( rxDbgTail	!= rxDbgHead )
	{
		*data = g_rxDbgRingBuffer[rxDbgTail++];
		rxDbgTail %= RX_BUFFER_LENGTH;
		return true;
	}
	else
		return false;
}

/*!
 * @brief Main function
 */
int main(void)
{
    uart_config_t config;
    // uart_transfer_t xfer;


    BOARD_InitPins();
    BOARD_BootClockRUN();

    /*
     * config.baudRate_Bps = 115200U;
     * config.parityMode = kUART_ParityDisabled;
     * config.stopBitCount = kUART_OneStopBit;
     * config.txFifoWatermark = 0;
     * config.rxFifoWatermark = 1;
     * config.enableTx = false;
     * config.enableRx = false;
     */
    UART_GetDefaultConfig(&config);
    config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
    config.enableTx = true;
    config.enableRx = true;

    UART_Init(DEMO_UART, &config, DEMO_UART_CLK_FREQ);
    UART_TransferCreateHandle(DEMO_UART, &g_uartDbgHandle, UART_UserCallback, NULL);

    //	dataSize will be set at printf

    // Prepare to receive one
    prepareRxDbgCharacter();

    printf("God is good ");
    uint8_t ch;
    int nbrRec=0, nbrRecCpy=0;
    while (1)
    {
    	while(getRxDbg(&ch))
    		++nbrRec;
    	if(nbrRec != nbrRecCpy)
    	{
    		printf("Received %d\r", nbrRec);
    		nbrRecCpy = nbrRec;
    	}

    	// Simulate a long delay
    	for(uint16_t i=0; i < 65000U; ++i);

    }
}
