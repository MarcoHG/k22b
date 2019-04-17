#include "board.h"
#include "fsl_uart.h"
#include "fsl_ftm.h" // FTM_GetQuadDecoderCounterValue
#include "peripherals.h"


#include "pin_mux.h"
#include "clock_config.h"
#include <stdio.h>
#include <stdlib.h>	//strtol

#include <string.h>
#include "settings.h"



/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_UART UART1
#define DEMO_UART_CLKSRC SYS_CLK
#define DEMO_UART_CLK_FREQ CLOCK_GetFreq(SYS_CLK)
#define TX_BUFFER_LENGTH 128
#define RX_BUFFER_LENGTH 128

#define PIT_MILLI_SEC				5		// Match the PIT component
#define AMPLIFIER_TIME_OUT 5000

#define NOT_MOVING_POS	5				// Counts to declare 'not moving'
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

bool  buildStdInArg(uint8_t ch, char *str);
void 	parseAndSendHex(char *cmd);
/*******************************************************************************
 * Variables
 ******************************************************************************/
stTransferUart sDbgUartTransfer;
stTransferUart sAuxUartTransfer;

volatile bool pitIsrFlag = false;
volatile uint32_t encoder_count = 0U;

inputSet_t 	gsSettings[] =
{
		// Read and write
	{ "CHKS",     0,	127,		&gSets.checkSpeed     },	// 0 to 127
	{ "CLOSES",   0,	127,    &gSets.closingSpeed   },	// 0 to 127
	{ "APERTS",   0,	127,    &gSets.apertureSpeed  },	// 0 to 127
	{ "WAITO",    0,	40000U, &gSets.waitAtOpenMsec },	// 5 to 40,000
	{ "CHKP",  		5,	80,     &gSets.checkPercent   },	// 5 to 80
	{ "PARTOP", 	5,	50,     &gSets.partialPercent },	// 5 to 50
	{ "CHKDIR",   0,	1,   	  &gSets.checkSpeedDir  },	// 0 to 1
	{ "MODE",			0,	2,		  &gSets.mode           }, //  07 Mode 0: run,	1:manual, 2:amplifier
	{ "OPE",			0,	2,		  &gSets.operation      }, //  08 Manual Operation 0: Init,	1:Open, 2:close
		// Read Only settings

};

uint16_t	nbr_settings = sizeof(gsSettings)/sizeof(inputSet_t);	// Variable dimension
stSettings gSets;



/*******************************************************************************
 * Code
 ******************************************************************************/
/* PIT IRQ */
void PIT1_0_IRQHANDLER (void )
{
	/* Clear interrupt flag.*/
	    PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
	    pitIsrFlag = true;



	    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
	  exception return operation might vector to incorrect interrupt */
	#if defined __CORTEX_M && (__CORTEX_M == 4U)
	    __DSB();
	#endif
}

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

	if (kStatus_UART_TxIdle == status)	// kUART_TxBusy
	{
		psUart->txOnGoing = false;	// There at least two more bytes on transit
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
 * 	\brief parses the c-string for hex codes and send them to serial port
 *
 * 	Assign
 * 	a =  b
 * 	hex1 hex2
 *
 *
 * 	returns a c-string is being built and return true when user hits \r or \n
 */
void parseAndSendHex(char *cmd)
{
	static uint8_t sendArray[20], index;

	char *pToken = strtok(cmd, " ");
	index =0;
	while (pToken != NULL)
	{
		int32_t data = strtol(pToken, NULL, 16);
		if(data <= 255)
			sAuxUartTransfer.txBuffer[index++] = data;
		pToken = strtok (NULL, " ,.-");
	}
	if(index)
	{
		sAuxUartTransfer.txTransfer.dataSize = index;
		sAuxUartTransfer.txOnGoing = true;
		UART_TransferSendNonBlocking(sAuxUartTransfer.base, &sAuxUartTransfer.handle,
		    &sAuxUartTransfer.txTransfer);
	}
	cmd[0]=0;
}
/*!
 * 	buildStdInArg
 * 	Build the string in the indicated pointer, user makes sure it has enough memory
 *
 * 	returns a c-string is being built and return true when user hits \r or \n
 */
bool  buildStdInArg(uint8_t ch, char *str)
{
	int i = strlen((char *)str);
	if('\r' == ch  || '\n' == ch)
	{
		/// Was sending directly
		//parseAndSendHex((char *)str);
		//str[0] = 0;
		str[i]= 0;	// c-str
		return true;
	}
	else
		if(0x08 == ch && i > 0)
		{
			printf("%c %c",ch,ch);
			str[--i] = 0;
		}
		else
		{
			str[i++] = ch;
			str[i] =0;	// c-string
			putchar(ch);
		}
	return false;

}

void initHardware(void)
{
  /* Define the init structure for the output LED pin*/
  gpio_pin_config_t led_config = {
      kGPIO_DigitalOutput, 0,
  };
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

	printf("Debug UART - Input hex commands and 'enter' to send it to auxport\r\n");// Test retarget at UART1

	// Initialize PIT & Quad Decoder
  BOARD_InitPeripherals(); //	This initializes UART0 on *.mex 

	// Auxiliary Uart = UART0
	pConfig = &sAuxUartTransfer.config;
	UART_GetDefaultConfig(pConfig);
	// Change from defaults
	pConfig->enableTx = true;
	pConfig->enableRx = true;
	pConfig->baudRate_Bps = 9600;
	initTransferUart(&sAuxUartTransfer, UART0, UART_AuxCallback, UART0_CLK_SRC);
	//
	uint8_t msg[] = "Hello from Auxiliary Uart - echo input\r";
	sAuxUartTransfer.txTransfer.data = sAuxUartTransfer.txBuffer;
	sAuxUartTransfer.txTransfer.dataSize = sizeof(msg) - 1;
	memcpy(sAuxUartTransfer.txBuffer, msg, sizeof(msg) - 1);
	sAuxUartTransfer.txOnGoing = true;
	UART_TransferSendNonBlocking(sAuxUartTransfer.base, &sAuxUartTransfer.handle,
	    &sAuxUartTransfer.txTransfer);

	//	-- how to set an interrupt on Transmission complete?
	// volatile UART_Type *base = UART0;
	// base->C2 |= UART_C2_TCIE_MASK;	// need to add the status kStatus_UART_TxBusy



  /* Init output LED GPIO. */
  GPIO_PinInit(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_GPIO_PIN, &led_config);

}
/*!
 * @brief Send data to the Aux serial port
 *
 * The routine assumes that the amplifier is connected to the Aux
 * serial port. On enter flushes the Aux serial port and sends the
 * indicated amount of data. The routine wait for a response, at least
 * one character from the amplifier.
 *
 * @return true if the amplifier responses, false if no response is received
 * after AMPLIFIER_TIME_OUT mSecs
 *
 */
bool sendAmpDataWait(uint8_t *data, uint8_t len)
{
	uint8_t	cha;
	uint16_t timeout =0;
	// Flush receiver buffer
	while(getRxRingBuffer(&sAuxUartTransfer, &cha));

	sAuxUartTransfer.txTransfer.dataSize = len;
	for(int i=0; i< len; ++i)
		sAuxUartTransfer.txBuffer[i] = data[i];

	GPIO_PortSet(BOARD_LED_BLUE_GPIO, 1u << BOARD_LED_BLUE_GPIO_PIN);
	sAuxUartTransfer.txOnGoing = true;
	UART_TransferSendNonBlocking(sAuxUartTransfer.base, &sAuxUartTransfer.handle,
			&sAuxUartTransfer.txTransfer);
	// Wait for TX to end
	while(sAuxUartTransfer.txOnGoing);
	pitIsrFlag = false;
	while(!getRxRingBuffer(&sAuxUartTransfer, &cha) && timeout < AMPLIFIER_TIME_OUT) // wait for amplifier to respond
	if(pitIsrFlag)
	{
		pitIsrFlag = false;
		timeout += PIT_MILLI_SEC;
	}
	GPIO_PortClear(BOARD_LED_BLUE_GPIO, 1u << BOARD_LED_BLUE_GPIO_PIN);
	if(timeout == AMPLIFIER_TIME_OUT)
		return false;
	return true;

}

/*!
 * @brief Send data to the Aux serial port
 *
 * The routine assumes that the amplifier is connected to the Aux
 * serial port. Then it first flushes the Aux serial port and sends the
 * indicated amount of data. The routine expects a character from the
 * amplifier after the command is sent
 *
 * @return true if the amplifier responses, false if no response is received
 * after AMPLIFIER_TIME_OUT mSecs
 *
 */
void sendAmpData(uint8_t *data, uint8_t len)
{
	sAuxUartTransfer.txTransfer.dataSize = len;
	for(int i=0; i< len; ++i)
		sAuxUartTransfer.txBuffer[i] = data[i];
	UART_TransferSendNonBlocking(sAuxUartTransfer.base, &sAuxUartTransfer.handle,
			&sAuxUartTransfer.txTransfer);
}

void sendAmpByte(uint8_t data)
{
	sendAmpData(&data,1);
}



bool sendAuxCommand(uint8_t cmd)
{
	static uint8_t command = 0;
	command = cmd;
	return sendAmpDataWait(&command,1);
}
bool sendAuxSetCommand(uint8_t cmd, uint8_t value)
{
	static uint8_t	command[2];
	command[0] = cmd;
	command[1] = value;
	return sendAmpDataWait(command,2);
}

int32_t	absVal(int32_t v)
{
	return v < 0	? -v	: v;
}

void initSequence(void)
{

	// Set to UART mode
	sendAuxCommand(0xE1);
	// Set Current Limit
	sendAuxSetCommand(0x82, 0x05);
	// Set Acel
	sendAuxSetCommand(0x84, 0x20);
	// Set Decel
	sendAuxSetCommand(0x85, 0x20);
	// Set Fwd
	sendAuxSetCommand(0x8A, 0x01);

	uint32_t position = 0;
	uint32_t position_old = 0;
	FTM_ClearQuadDecoderCounterValue(QUAD1_PERIPHERAL);

	// Set Speed
	sendAuxSetCommand(0x80, 0x1F);
	// Monitor POsition
	pitIsrFlag = false;
	bool moving = true;

	uint8_t cha;
	uint16_t zeroSpeed=0;
	while(getRxRingBuffer(&sAuxUartTransfer, &cha));	// Flush RX
	printf("Reaching full open position\r\n");
	while(moving)
	{
		static uint16_t counts=0;
		position = FTM_GetQuadDecoderCounterValue(QUAD1_PERIPHERAL);
		if(pitIsrFlag)
		{
			pitIsrFlag = false;
			if(++counts >=25)	// 125mS
			{
				counts =0;
				if(absVal(position_old - position) < NOT_MOVING_POS )
					++zeroSpeed;
				else
					zeroSpeed=0;
				position_old = position;
				printf("\rPos= %ld, %d, Clim:", position, zeroSpeed);
				sendAmpByte(0xCA);	// Read Current
			}
		}
		// Read response from Amplifier
		if(getRxRingBuffer(&sAuxUartTransfer, &cha))
			printf(" [%x]",0xFF&cha);
		if(zeroSpeed >= 40)	// 5 secs
			moving = false;



	}
	// Set Speed
	sendAuxSetCommand(0x80, 0x00);
	// Set to stop
	sendAuxSetCommand(0x8A, 0x00);




}
/*!
 * @brief Main function
 */
int main(void)
{


	uint32_t counts;
	uint8_t ch, cha;
	uint16_t nbrRec = 0, nbrRecCpy = 0;
	char		command[10] = {0};
	uint8_t cmd_index=0;

	initHardware();
	char inStr[50]={0}, outStr[50]={0};

	parse_result_t result;

	// Start with RUN mode
	gSets.mode = MODE_RUN;
	uint8_t index;
	getRxRingBuffer(&sDbgUartTransfer, &ch); // test
	while (1)
	{
		// GPIO_PortToggle(BOARD_LED_BLUE_GPIO, 1u << BOARD_LED_BLUE_GPIO_PIN);
		//GPIO_PortSet(BOARD_LED_BLUE_GPIO, 1u << BOARD_LED_BLUE_GPIO_PIN);
		// Handle Debug Console
		if(getRxRingBuffer(&sDbgUartTransfer, &ch))
		{
		  if(ch == '~' && gSets.mode == MODE_AMPLIFIER)
		  {
		  	counts = FTM_GetQuadDecoderCounterValue(QUAD1_PERIPHERAL);	// encoder_count = FTM_GetQuadDecoderCounterValue(DEMO_FTM_BASEADDR);
		  	printf("E: %d\r\n", counts);
		  }
		  else
		  if (buildStdInArg(ch, inStr) )
		  {
		  	if(gSets.mode == MODE_AMPLIFIER)
		  	{
		  		if(strstr(inStr,"end") != NULL)
		  		{
		  			printf("Mode set to MANUAL\r\n");
		  			gSets.mode=MODE_MANUAL;
		  		}
		  		else
		  			parseAndSendHex(inStr);

		  	}
		  	else
		  	switch(result = parseInStr(gsSettings, inStr, outStr, &index) )
		  	{
				case PARSE_ASSIGN:
					if(strcmp(gsSettings[index].pToken,"OPE")==0 && *gsSettings[index].pData == 0)
					{
						// Ope=0 init
						initSequence();

					}

					printf(" OK\r\n");
					break;
				case PARSE_READ:
					printf(" %s\r\n",outStr);
					break;
				case PARSE_LIM_ERR:
					printf("Value not in range\r\n");
					break;
				case PARSE_UNKOWN_TOKEN:
					printf("unknown command %s\r\n",inStr);
					break;
				default:
					printf("Wrong input\r\n");
					break;
		  	}
		  	inStr[0] =0;	// clear input string
		  }
		}


		// Handle Aux
		// if(mode == MODE_AMPLIFIER)
		if(getRxRingBuffer(&sAuxUartTransfer, &cha))
		{
			printf("[%x]",0xFF&cha);
			//sAuxUartTransfer.txTransfer.data[0] 	= cha;
			//sAuxUartTransfer.txTransfer.dataSize 	= 1;
			//UART_TransferSendNonBlocking(sAuxUartTransfer.base, &sAuxUartTransfer.handle, &sAuxUartTransfer.txTransfer);
		}

		if(pitIsrFlag)	// Every 5 mS
		{
			pitIsrFlag = false;

		}

		// Simulate a long delay
		//for (uint16_t i = 0; i < 65000U; ++i)
		{
			// Debug
		}

	}
}
