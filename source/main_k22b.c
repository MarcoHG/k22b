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
#include "version.h"
#include "pflash.h"		// imported from pflash example
#include "fsl_crc.h"	// Added crc support


/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_UART UART1
#define DEMO_UART_CLKSRC SYS_CLK
#define DEMO_UART_CLK_FREQ CLOCK_GetFreq(SYS_CLK)
#define TX_BUFFER_LENGTH 128
#define RX_BUFFER_LENGTH 128

#define PIT_MILLI_SEC				5			// Match the PIT component
#define QUAD_MODULO					5000	// From component 0..QUAD_MODULO
#define AMPLIFIER_TIME_OUT  5000
#define READ_CMD_TIME_OUT  	50

#define MAX_SPEED_CMD			127
#define FULL_OPEN_MARGIN	100		// Counts to stop before hit the open-end

#define	TOK_SAVE	"SAVE"
#define	TOK_LIST	"LIST"


#define NOT_MOVING_POS	5				// Counts to declare 'not moving'
typedef struct transfer_uart
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
void 	defaultValues(void);

void delay_ms(uint32_t delay);
/*******************************************************************************
 * Variables
 ******************************************************************************/
stTransferUart sDbgUartTransfer;
stTransferUart sAuxUartTransfer;

volatile bool pitIsrFlag = false;
volatile uint32_t	pit_counter =0;
volatile int32_t encoder_count = 0U;
volatile bool req_reset_encoder = false;
volatile int32_t	gDoor_length =0;
volatile int32_t	gVolt_supply;

// commands
int32_t 	gMode;
int32_t 	gOperation;
int32_t		dummy;

CRC_Type 			*crc_base = CRC0;
crc_config_t 	crc_config;


inputSet_t 	gUsrInput[] =
{
				// Read and write parameters - mapped to gData
/* 0  */	{ "CHKS",     0,	127,		&gData.checkSpeed     },	// 0 to 127
/* 1  */	{ "CLOSES",   0,	127,    &gData.closingSpeed   },	// 0 to 127
/* 2  */	{ "APERTS",   0,	127,    &gData.apertureSpeed  },	// 0 to 127
/* 3  */	{ "WAITO",    0,	40000U, &gData.waitAtOpenMsec },	// 5 to 40,000
/* 4  */	{ "CHKP",  		5,	80,     &gData.checkPercent   },	// 5 to 80 speeds
/* 5  */	{ "PARTOP", 	5,	50,     &gData.partialPercent },	// 5 to 50
/* 6  */	{ "CLIM", 		0,	127,    &gData.currentLimit		},	// 0 to 127
/* 7  */	{ "RLIM", 		0,	127,    &gData.regenLimit			},	// 0 to 127
/* 8  */	{ "ACEL", 		0,	127,    &gData.acel						},	// 0 to 127
/* 9  */	{ "DECEL", 		0,	127,    &gData.decel					},	// 0 to 127
/* 10 */	{ "CHKDIR",   0,	1,   	  &gData.checkSpeedDir  },	// 0 to 1
/* 11 */	{ "OCAUTO",   0,	1,   	  &gData.openCloseAuto	},	// 0 to 1,	0:Normal, 1:Auto open-close
/* 12 */	{ "OCWAIT",  	0,	50000U, &gData.waitRepeat			},	// 0 to 50,000	Wait to re-start open-close sequence

/* 13 */	{ "MODE",			0,	2,		  &gMode           			}, 	//  07 Mode 0: run,	1:manual, 2:amplifier
/* 14 */	{ "OPE",			0,	2,		  &gOperation      			}, 	//  08 Manual Operation 0: Init,	1:Open, 2:close
/* 15 */	{ "RANGE",		0,	50000U,	&gDoor_length    			}, 	//  Read Only - set at runtime
/* 16 */	{ "VOLT",		  0,	50000U,	&gVolt_supply    			}, 	//  Read Only - set at runtime



// These are operations only
/* 15 */	{ TOK_SAVE,		0,	0,		  &dummy     						}, 	//  Save tp pflash
/* 16 */	{ TOK_LIST,		0,	0,		  &dummy     						}, 	//  List commands
		// Read Only settings

};

uint16_t	gNbrUsrInput = sizeof(gUsrInput)/sizeof(inputSet_t);	// Variable dimension
stPrmData gData;
uint16_t	gNbrData 			= sizeof(gData)/sizeof(uint32_t);		// Variable dimension	gData gNbrData





/*******************************************************************************
 * Code
 ******************************************************************************/
void trap_error(char *pMsg)
{
	volatile char *pDebug = pMsg;	// Debug point
	while(1);
}

/* Init CRC-16-CCIT
  *
  * @details Init CRC peripheral module for CRC-16/CCIT-FALSE protocol:
  * 		width=16 poly=0x1021 init=0xffff refin=false refout=false xorout=0x0000 check=0x29b1
  *    	http://reveng.sourceforge.net/crc-catalogue/
  *    	name="CRC-16/CCITT-FALSE"
  */

void crc_init(void)
{
	/*
	 * config.polynomial = 0x1021;
	 * config.seed = 0xFFFF;
	 * config.reflectIn = false;
	 * config.reflectOut = false;
	 * config.complementChecksum = false;
	 * config.crcBits = kCrcBits16;
	 * config.crcResult = kCrcFinalChecksum;
	 */
	CRC_GetDefaultConfig(&crc_config);
	crc_config.seed = 0xFFFFU;
}
/*!
 * @brief Calculate the 16-bit crc
 *
 * must initialize the base
 *
 */
uint16_t get_crc(uint8_t *data, uint16_t length)
{
	CRC_Init(crc_base, &crc_config);
  CRC_WriteData(crc_base, data, length);
  return CRC_Get16bitResult(crc_base);
}

/* PIT IRQ */
void PIT1_0_IRQHANDLER (void )
{
	/* Clear interrupt flag.*/
	PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
	pitIsrFlag = true;
	++pit_counter;
	static uint32_t previous_count;
	uint32_t	counts = 0;
	int32_t	delta_counts = 0;
	if(req_reset_encoder)
	{
		req_reset_encoder = false;
		encoder_count =0;
		FTM_ClearQuadDecoderCounterValue(QUAD1_PERIPHERAL);

	}
	else
	{
		//uint32_t flags 	= FTM_GetQuadDecoderFlags(QUAD1_PERIPHERAL);
		counts 						=		FTM_GetQuadDecoderCounterValue(QUAD1_PERIPHERAL);
		bool overflow_pos = 	previous_count > QUAD_MODULO/2  && counts < QUAD_MODULO/2;
		bool overflow_neg = 	previous_count < QUAD_MODULO/2  && counts > QUAD_MODULO/2;
		if(overflow_pos)
			delta_counts = counts - (QUAD_MODULO - previous_count) +1;
		else
		if(overflow_neg)
			delta_counts = 	-((counts - QUAD_MODULO) + 1 - previous_count);
		else
			delta_counts = 	(int32_t)counts - (int32_t)previous_count;
	}
	previous_count = counts;
	encoder_count += delta_counts;



	    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
	  exception return operation might vector to incorrect interrupt */
	#if defined __CORTEX_M && (__CORTEX_M == 4U)
	    __DSB();
	#endif
}
/*!
 * \brief Create a blocking delay using PIT isr
 *
 *
 */
void delay_ms(uint32_t delay)
{
	uint32_t counts = delay / PIT_MILLI_SEC;
	pitIsrFlag = false;
	for(int i =0; i <= counts ;)
		if(pitIsrFlag)
		{
			pitIsrFlag = false;
			++i;
		}


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
  gpio_pin_config_t sw_config = {
  		kGPIO_DigitalInput,
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

	printf("Linear Doors Control - WCC Version %d.%d.%d\r\n", SW_MAJOR, SW_MINOR, SW_BUILD);// Test retarget at UART1

	// Initialize PIT & Quad Decoder
  BOARD_InitPeripherals(); //	This initializes UART0 on *.mex 
  FTM_SetQuadDecoderModuloValue(QUAD1_PERIPHERAL, 0, QUAD_MODULO);	// over-ride QUAD_MODULO peripherals

	// Auxiliary Uart = UART0
	pConfig = &sAuxUartTransfer.config;
	UART_GetDefaultConfig(pConfig);
	// Change from defaults
	pConfig->enableTx = true;
	pConfig->enableRx = true;
	pConfig->baudRate_Bps = 9600;
	initTransferUart(&sAuxUartTransfer, UART0, UART_AuxCallback, UART0_CLK_SRC);
	//
	// msg to test aux port
	uint8_t msg[] = "HelloAuxUart";
	sAuxUartTransfer.txTransfer.data = sAuxUartTransfer.txBuffer;
	sAuxUartTransfer.txTransfer.dataSize = sizeof(msg) - 1;
	memcpy(sAuxUartTransfer.txBuffer, msg, sizeof(msg) - 1);
	sAuxUartTransfer.txOnGoing = true;
	UART_TransferSendNonBlocking(sAuxUartTransfer.base, &sAuxUartTransfer.handle,
	    &sAuxUartTransfer.txTransfer);

	// Flush AUX to avoid echo Amplifier
	pitIsrFlag =false;
	for(uint8_t pitIsr=0; pitIsr < 10; )
	{
		if(pitIsrFlag)
		{
			pitIsrFlag = false;
			++pitIsr;
		}
		uint8_t cha;
		getRxRingBuffer(&sAuxUartTransfer, &cha);	// Flush RX
	}
	//	-- how to set an interrupt on Transmission complete?
	// volatile UART_Type *base = UART0;
	// base->C2 |= UART_C2_TCIE_MASK;	// need to add the status kStatus_UART_TxBusy

  /* Init output LED GPIO. */
  GPIO_PinInit(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_GPIO_PIN, &led_config);
  GPIO_PortSet(BOARD_LED_BLUE_GPIO, 1U << BOARD_LED_BLUE_GPIO_PIN);	// Turn LED off

  /* Init SW2	- also arduino A2*/
  GPIO_PinInit(BOARD_SW2_GPIO, BOARD_SW2_GPIO_PIN, &sw_config);	// enable pin also in pins.h

  // Init CRC polynomial
  crc_init();

  // Get data from Memory
  // test my crc
  //char testData[] = "123456789";
  //volatile uint16_t ccrc = get_crc((uint8_t *)&testData, sizeof(testData) - 1);	// 0x29b1
  //ccrc = get_crc((uint8_t *)&testData, sizeof(testData) - 1);	// 0x29b1

  delay_ms(50);
  if( !pflash_read((uint32_t *)&gData, gNbrData) )
 		printf("\r\nError Reading pflash!\r\n");
  else
  {
  	// Calculate CRC
  	uint32_t crc 	= get_crc((uint8_t *)&gData, (gNbrData-1)*4);	// AL in 32 bits
  	if(crc != gData.crc)
  	{
  		// Memory corrupted, loading defaults
  		defaultValues();
  		printf("\r\nMemory BAD, using defaults\r\n");

  	}
  	else
  		printf("\n\rData from pFlash\r\n");
  	delay_ms(50);
  }
  volatile int a= 1234;	// debug


}

void defaultValues(void)
{
	gData.checkSpeed		  = 40;
	gData.closingSpeed 	  = 65;
	gData.apertureSpeed	  =	80;
	gData.waitAtOpenMsec	= 2000;
	gData.checkPercent  	= 50;
	gData.partialPercent  = 50;
	gData.currentLimit 	  = 20;		//
	gData.regenLimit 		  = 20;		//
	gData.acel					  = 20;
	gData.decel					  = 20;
	gData.checkSpeedDir	  = 1;
	gData.openCloseAuto	  = 0;			//	Normal
	gData.waitRepeat  	  = 3000;		//

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
/*!
 * Compensate PWM output with supply
 * Norm = 115VAC
 */
bool setSpeed(uint32_t spd)
{
	uint32_t	speed = (spd * 160)/gVolt_supply;
	if(speed >MAX_SPEED_CMD)
		speed = MAX_SPEED_CMD;
	return sendAuxSetCommand(0x80, speed);
}
int32_t	absVal(int32_t v)
{
	return v < 0	? -v	: v;
}

bool closeSequence(void)
{
	// Set Speed
	if( !setSpeed(gData.closingSpeed))		// 0.1.7
		return false;

	// Monitor Zero speed
	pitIsrFlag = false;
	uint16_t zeroSpeed=0;
	int32_t position 			= 0;
	int32_t position_old 	= 0;

	for (uint8_t cha; getRxRingBuffer(&sAuxUartTransfer, &cha); );	// Flush RX
	// Start Moving REV
	if( !sendAuxSetCommand(0x8A, 0x02))
		return false;
	bool moving = true;
	int32_t closeSpdPos = gDoor_length * gData.checkPercent/100;
	bool atCheckSpeed = false;
	printf("\r\nClosing Doors..%ld, %ld@%ld->0\r\n", gData.closingSpeed, gData.checkSpeed, closeSpdPos);
	while(moving)
	{
		static uint16_t counts=0;
		uint8_t cha;
		// Every 5mS
		if(pitIsrFlag)
		{
			pitIsrFlag = false;
			position = encoder_count;
			if(++counts >=25)	// 125mS
			{
				counts =0;
				if(absVal(position_old - position) < NOT_MOVING_POS )
					++zeroSpeed;
				else
					zeroSpeed=0;
				position_old = position;
				if(encoder_count < closeSpdPos && !atCheckSpeed)
				{
					atCheckSpeed = true;
					setSpeed(gData.checkSpeed);		// 0.1.7

				}
				char speed = atCheckSpeed ? 'C' : 'O';
				printf("\rPos= %ld, %d, '%c' Clim:", position, zeroSpeed, speed );
				sendAmpByte(0xCA);	// Read Current
			}
		}
		// Read response from Amplifier
		if(getRxRingBuffer(&sAuxUartTransfer, &cha))
			printf(" [%x]",0xFF&cha);
		if(zeroSpeed >= 40)	// 5 secs
		{
			moving = false;
			zeroSpeed = 0;
		}
	}
	// Set Speed to zero
	sendAuxSetCommand(0x80, 0x00);
	// Set to stop mode
	sendAuxSetCommand(0x8A, 0x00);
	return true;
}


/*!
 * Open sequence
 *
 */
bool openSequence(void)
{
	// Set Speed
	if( !setSpeed(gData.apertureSpeed))		// 0.1.7
		return false;

	// Monitor Zero speed
	pitIsrFlag = false;
	uint16_t zeroSpeed=0;
	int32_t position 			= 0;
	int32_t position_old 	= 0;

	for (uint8_t cha; getRxRingBuffer(&sAuxUartTransfer, &cha); );	// Flush RX
	// Start Moving FWD
	if( !sendAuxSetCommand(0x8A, 0x01))
		return false;
	bool moving = true;
	int32_t openSpdPos = gDoor_length * gData.checkPercent/100;
	bool atCheckSpeed = false;
	printf("\r\nOpening Doors..%ld, %ld@%ld->%ld\r\n", gData.apertureSpeed, gData.checkSpeed, openSpdPos,gDoor_length );
	while(moving)
	{
		static uint16_t counts=0;
		uint8_t cha;
		// Every 5mS
		if(pitIsrFlag)
		{
			pitIsrFlag = false;
			position = encoder_count;
			if(++counts >=25)	// 125mS
			{
				counts =0;
				// This condition is for jamming 0.1.8+
				if(absVal(position_old - position) < NOT_MOVING_POS )
					++zeroSpeed;
				else
					zeroSpeed=0;
				position_old = position;
				if(encoder_count > openSpdPos)
				if(!atCheckSpeed)		// Change speed
				{
					atCheckSpeed = true;
					setSpeed(gData.checkSpeed);	// 0.1.7

				}
				else
				{
					if(encoder_count > (gDoor_length - FULL_OPEN_MARGIN))	// close to hit the end-stop?
					{
						setSpeed(0);	// 0.1.7
						moving = false;
					}

				}
				char speed = atCheckSpeed ? 'C' : 'O';
				printf("\rPos= %ld, %d, %c Clim:", position, zeroSpeed, speed );
				sendAmpByte(0xCA);	// Read Current
			}
		}
		// Read response from Amplifier
		if(getRxRingBuffer(&sAuxUartTransfer, &cha))
			printf(" [%x]",0xFF&cha);
		if(zeroSpeed >= 40)	// 5 secs - jamming -- stop and re-init
		{
			moving = false;
			zeroSpeed = 0;
		}
	}
	// Set Speed to zero
	sendAuxSetCommand(0x80, 0x00);
	// Set to stop mode
	sendAuxSetCommand(0x8A, 0x00);
	return true;
}


bool initSequence(int32_t *position_range)
{
	// Set to UART mode
	if( !sendAuxCommand(0xE1))
		return false;
	// Set Current Limit
	if(!sendAuxSetCommand(0x82, gData.currentLimit))		// TBD
		return false;
	// Set Acel
	if( !sendAuxSetCommand(0x84, gData.acel) )		// gData.acel, gData.decel
		return false;
	// Set Decel
	if( !sendAuxSetCommand(0x85, gData.decel) )
		return false;
	// Set STOP
	if( !sendAuxSetCommand(0x8A, 0x00) )
		return false;

	// Prepare position
	int32_t position 			= 0;
	int32_t position_old 	= 0;
	int32_t position_at_close;
	req_reset_encoder = true;	// reset the position

	// Set Speed normalized 0.1.7
	if( !setSpeed(gData.checkSpeed) )
		return false;

	// Monitor POsition
	pitIsrFlag = false;

	uint16_t zeroSpeed=0;
	for (uint8_t cha; getRxRingBuffer(&sAuxUartTransfer, &cha); );	// Flush RX
	printf("\r\nInit Doors..");
	bool moving = false;
	typedef enum direction_t {OPEN=0, CLOSE, DONE} direction_t;
	direction_t motorDirection = OPEN;
	while(motorDirection == OPEN || motorDirection == CLOSE)
	if(!moving)
	{
		// Start moving
		uint8_t dir;
		char moving_str[2][10] = {"Opening", "Closing"};
		dir = (motorDirection == OPEN) ?  0x01 : 0x02;
		if( !sendAuxSetCommand(0x8A, dir) )	// Start moving FWD
			return false;
		printf("\r\n%s...\r\n",moving_str[dir-1]);
		moving = true;
	}
	else
	while(moving)
	{
		static uint16_t counts=0;
		uint8_t cha;
		// Every 5mS
		if(pitIsrFlag)
		{
			pitIsrFlag = false;
			position = encoder_count;
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
		{
			moving = false;
			zeroSpeed = 0;
			if(motorDirection == OPEN)
			{
				req_reset_encoder = true;
				// delay_ms(10);
			}
			else
				position_at_close = position;
			++motorDirection;
		}
	}
	// Wait for any previous reading to display
	// delay_ms(50); // full duplex not needed

	// Set current position as "Home"
	req_reset_encoder = true;
	// Set Speed to zero
	sendAuxSetCommand(0x80, 0x00);
	// Set to stop mode
	sendAuxSetCommand(0x8A, 0x00);
	*position_range = absVal(position_at_close);
	return true;

}

/*!
 *	@brief	Execute the indicated command
 *
 *	User has input a value to the gUsrInput[index] and this function performs
 *	the desired operation
 *
 */
void executeCommand(index)
{
	switch(index)
	{
	case 0 : //	 "CHKS",
	case 1 : //	 "CLOSES"
	case 2 : //	 "APERTS"
	case 3 : //	 "WAITO",
	case 4 : //	 "CHKP",
	case 5 : //	 "PARTOP"
	case 6 : //	 "CLIM",
	case 7 : //	 "RLIM",
	case 8 : //	 "ACEL",
	case 9 : //	 "DECEL",
  case 10: //	 "CHKDIR"
  case 11: //	 spare1
  case 12: //	 spare2
  	printf(" [%ld]",*gUsrInput[index].pData);
  	break;
  case 13: //	 "MODE"
  	switch(*gUsrInput[index].pData)
  	{
  	case	0:
  		printf("\r\nRun mode\r\n");
  		break;
  	case 	1:
  		printf("\r\nManual mode\r\n");
  		break;
  	case	2:
  		printf("\r\nAmplifier commands accepted - enter 'end' to finish this mode\r\n");
  		break;
  	}
  	break;
  case 14: //	 "OPE"
  	switch(*gUsrInput[index].pData)	//
  	{
  	case	0:	// OPE=0 to HOME
  		if( initSequence(&gDoor_length) )
  			printf("\r\nInit complete, range: %ld counts\r\n", gDoor_length);
  		else
  			printf("\r\nInit Sequence Failed!");
  		break;
  	case	1:	// OPE=1 to OPEN
  		if(openSequence() )
  			printf("\r\nDoor is open..");
  		else
  			printf("\r\nOPEN Sequence Failed!");
  		break;
  	case	2:	// OPE=2 to CLOSE
  		if(closeSequence() )
  		{
  		  printf("\r\nDoor is closed OK, setting this position as HOME\r\n");
  		  req_reset_encoder=true;
  		}
  		else
  			printf("\r\nOPEN Sequence Failed!");
  		break;
  		break;
  	default:
  		trap_error("Unkown OPE");
  		break;
  	}
  	break;


  default:
  	trap_error("Invalid index");
  	break;
	}	// Index
}

bool readCmd(uint8_t cmd, uint8_t *value)
{
	uint8_t	cha;
	uint16_t timeout =0;

	// Flush receiver buffer
	while(getRxRingBuffer(&sAuxUartTransfer, &cha));

	sAuxUartTransfer.txTransfer.dataSize = 1;
	sAuxUartTransfer.txBuffer[0] = cmd;

	sAuxUartTransfer.txOnGoing = true;
	UART_TransferSendNonBlocking(sAuxUartTransfer.base, &sAuxUartTransfer.handle,
			&sAuxUartTransfer.txTransfer);

	// Wait for TX to end
	while(sAuxUartTransfer.txOnGoing);
	pitIsrFlag = false;
	while(!getRxRingBuffer(&sAuxUartTransfer, value) && timeout < READ_CMD_TIME_OUT) // wait for amplifier to respond
	if(pitIsrFlag)
	{
		pitIsrFlag = false;
		timeout += PIT_MILLI_SEC;
	}
	if(timeout == READ_CMD_TIME_OUT)
		return false;

	return true;

}
/*!
 * 	gVolt_supply
 * 	Reads voltage from amplifier and if response,
 * 	value is set in parameter
 *
 * 	\returns true if amplifier responds correclty
 */
bool read_supply( uint32_t *volt_in)
{
	uint8_t volt;

	if( readCmd(0xCC, &volt))
	{
		*volt_in = (22 * volt)/10;	// Command CC gain is 2.2V per count
		return true;
	}
	return false;
}


void do_open_close_sequence(void)
{
	static uint32_t counter=0;
	printf("\r\n*** Open-Close sequence #%u ***", ++counter);
	req_reset_encoder = true;
	while(req_reset_encoder);
	printf(" home ");

	// Open - close sequence
	openSequence();
	// Wait at open position
	printf("\r\nDelay at open");
	uint32_t	wait_ticks = gData.waitAtOpenMsec/PIT_MILLI_SEC;
	uint32_t pit_counter_local;
	for(pit_counter=0;	pit_counter < wait_ticks;)
	{
		if(pit_counter_local != pit_counter)
		{
			pit_counter_local = pit_counter;
			if((pit_counter_local % 100)==99 )
				printf(".");
		}
	}
	closeSequence();
	printf(" completed.");
	delay_ms(100);

}

/*!
 * 	Test the demo open-close flag
 */
bool start_open_close_sequence(uint32_t wait_ms)
{
	bool run_open_close = false;
	if(gData.openCloseAuto)
	{
		run_open_close = true;
	  uint8_t ch;
	  pit_counter = 0;
	  uint32_t pit_local =0;
	  //
	  while( getRxRingBuffer(&sDbgUartTransfer, &ch) );	// flush serial port
	  delay_ms(100);
	  printf("\r\nOpen-Close is set continuous, press 'y' to cancel\n");
	  delay_ms(100);
	  // Wait for wait_ms for cancellation
	  while(pit_counter < (wait_ms/PIT_MILLI_SEC) && run_open_close)
	  if(getRxRingBuffer(&sDbgUartTransfer, &ch))
	  {
	  	if(ch == 'y')
	  	{
	  		run_open_close = false;
	  	}
	  }
	  else
	  {
	  	if(pit_local != pit_counter)
	  	{
	  		pit_local = pit_counter;
	  		if(!(pit_local%200))	// every sec
	  			printf("\r%u", pit_local*PIT_MILLI_SEC/1000);
	  	}

	  }
	}
	return run_open_close;
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
	gMode = MODE_RUN;
	uint8_t index;
	getRxRingBuffer(&sDbgUartTransfer, &ch); // test
	bool init_sequence_done = false;

#if 0
	// Quick test 0.1.7
	gDoor_length = 2000;	// \todo remove on commit
	init_sequence_done	= true;
#endif

	gVolt_supply = 160;
	read_supply(&gVolt_supply);
	setSpeed(0);

	printf("\r\nVoltage Supply %lu", gVolt_supply);
	// Test for Open-Close on powerup
	bool run_open_close_sequence = start_open_close_sequence(5000);	//


	while (1)
	{
		// GPIO_PortToggle(BOARD_LED_BLUE_GPIO, 1u << BOARD_LED_BLUE_GPIO_PIN);
		//GPIO_PortSet(BOARD_LED_BLUE_GPIO, 1u << BOARD_LED_BLUE_GPIO_PIN);
		// Handle Debug Console
		if(getRxRingBuffer(&sDbgUartTransfer, &ch))
		{
		  if(ch == '`' && gMode == MODE_AMPLIFIER)
		  {
		  	//counts = FTM_GetQuadDecoderCounterValue(QUAD1_PERIPHERAL);	// encoder_count = FTM_GetQuadDecoderCounterValue(DEMO_FTM_BASEADDR);
		  	printf("E: %ld\r\n", encoder_count);
		  }
		  else
		  if (buildStdInArg(ch, inStr) )
		  {
		  	if(gMode == MODE_AMPLIFIER)
		  	{
		  		if(strstr(inStr,"end") != NULL)
		  		{
		  			printf("Mode set to MANUAL\r\n");
		  			gMode=MODE_MANUAL;
		  		}
		  		else
		  			parseAndSendHex(inStr);

		  	}
		  	else
		  	switch(result = parseInStr(gUsrInput, inStr, outStr, &index) )
		  	{
				case PARSE_ASSIGN:	// The setting gUsrInput[index] has been assigned successfully
					executeCommand(index);
					break;
				case PARSE_READ:
					if(!strcmp(gUsrInput[index].pToken, TOK_SAVE) )
					{
						gData.crc = get_crc((uint8_t *)&gData, sizeof(gData)-4);
						if( pflash_write((uint32_t *)&gData, gNbrData) )	// gData
							printf("\r\nSettings saved...");
						else
							printf("\r\nERROR saving parameters ");
					}
					else
					if(!strcmp(gUsrInput[index].pToken, TOK_LIST))
					{
						//sDbgUartTransfer.txOnGoing = false;
						for(int i=0; i<gNbrData-1; ++i)	// don't display last (CRC)
						{
							while(sDbgUartTransfer.txOnGoing);	// wait before sending
							printf("\r\n%02d\t%s\t%ld",i,gUsrInput[i].pToken, *gUsrInput[i].pData );
						}
						while(sDbgUartTransfer.txOnGoing);	// wait before sending
						printf("\r\n");
					}
					else
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



		// SW2 has been pressed
		if(!GPIO_PinRead(BOARD_SW2_GPIO, BOARD_SW2_GPIO_PIN))
		{
			// Debounce
			delay_ms(10);
			if( GPIO_PinRead(BOARD_SW2_GPIO, BOARD_SW2_GPIO_PIN) )
				break;	// Noise
			if(!init_sequence_done)
			{
				//
				printf("Initialize by SW2 request");
				initSequence(&gDoor_length);
				init_sequence_done = true;
			}
			else
				do_open_close_sequence();

		}
		// Open close set by powerup
		if(run_open_close_sequence)
		{
			if(!init_sequence_done)
			{
				//
				printf("Initialize by AUTO flag");
				initSequence(&gDoor_length);
				init_sequence_done = true;
			}
			else
			{
				do_open_close_sequence();
				printf("\r\nWaiting %lu mS to repeat...",gData.waitRepeat);
				delay_ms(gData.waitRepeat);
				while(getRxRingBuffer(&sAuxUartTransfer, &cha));	// flush amplifier serial port garbage
			}


		}

		int a = 123;

		if(pitIsrFlag)	// Every 5 mS
		{
			static uint32_t run_counter =0;
			pitIsrFlag = false;

			// Monitor voltage in RUN mode,
			if(gMode == MODE_RUN)
			if(++run_counter > 200)	// every second
			{
				if( read_supply(&gVolt_supply) )
				{
					// printf("\r\nSupply %u ", gVolt_supply);	// test OK
				}
				run_counter =0;
			}




		}

	}
}
