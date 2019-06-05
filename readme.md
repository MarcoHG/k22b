# Developing with MCUXpresso and SDK
Develop a project using the SDK components, record insights to share with colleagues and create a framework with at least two serial ports. One of the serial will be used for Debug and shall re-target printf to provide debug trace.

Target is the FRDM-K22 board. Initially the debug port will be UART1 as it can by access using OSDA.

## Open Sequence and Ajdusts are operaative 0.1.7


## Arduino shield candidate to ship 0.1.6
- Change motor wires to match encoder wiring 
- Added pflash and crc drivers
- save to flash
- list
- SW2 starts a 'init sequence' and after it will do "operate'

## Completed Open and close sequence at command line 0.1.5
- Added openSequence and closeSequence
- Using parameters from the gsSet settings,i.e. can be changed at user's input


## Completed Init sequence 0.1.4
- Encoder position is int32_t and is updated at PIT
- Init sequence uses global settings

## Added QUAD setting 0.1.3
- added FTM2 for QuadDecoder
- added structure to input parameters from serial console
- support for GPIO PTD5 to drive BLUE_LED benchmark and observe serial port usage
- start init (home) routine
- version.h
 

## Add Second UART 0.2
-	Added a structure `stTransferUart` to organize variables of transfer-Uart method 
-	UART1 is Debug 115200-n-8-1 and UART0 is 9600-n-8-1
- Increase TX_BUFFER_LENGTH to 128, asserts also are routed here

Use UART0, pins are accesible at the Arduino connector [PTA1-RX-D9](Arduino J2 Connector) and [PTA2-TX-D3](Arduino J1 Connector)

Add the SDK component for Uart0 `Open MCUXpresso Configuration` button and add pins
In the `Peripherals` tools add UART0 component and configure for:
- Name: UART_AUX
- Mode:	Transfer
- Peripheral:	UART0
- UART configuration leave the defaults 115200,N,8,1 for this commit
	+ SYS_CLK 80MHz, 
	+ Clock Freq CLOCK_GetFreq()  
	+ 115200, TX water 0, RX water 1, Idle after valid start bit
	+ Enable TX
	+ Enable RX
- Leave the Transfer configuraton as default (these are const data that we won't be able to modify anyway)
	+ Transfer handle ID:	UART_AUX_handle
	+ Check init 

- Update the project code and the peripheral.* files should be created and pin_mux.* and clock.* should be updated 


## Base project 0.1 
3/4/2019
Base project uses transfer interrupt method for UART1. Goal is to retarget printf at this UART.

The rx transfer length is one and we receive an event every character which is moved to a g_rxDbgRingBuffer.
Function `getRxDbg` returns true if rx ring buffer is not empty and returns the character at the tail, this is 
a non blocking function returning false if rx ring buffer is empty
 
The printf is retargeted as explained in doc/ and printf buffer is moved to the g_txDbgBuffer transfer buffer, 
making the printf a non-blocking. The tx transfer event is serviced after the last character is sent.

### Create the project
Use a `Import SDK example` wizard generated project as the starting point .
- Name it `k22b`
- Provide the freedom board support file, this will generate the board.* files
- The SDK Debug console to UART
- Select the uart `uart_interrupt_transfer` driver example 
- This is a C - project and the SDK console is in UART

_In the Advanced Setting page_
- Select Library Redlib (nohost)  - Newlib will be for C++ but couldn't retarget printf
- sdk PRINTF -> printf
- printf/scanf -> uart
- leave all other options with default

### Edit 
Compile and run, verify that UART1 works as expected
- rename project to k22b
- Edit the main file with starting needs and rename it `main_k22b.c` 
- Start git version control and add: `*.[ch] .cproject .project readme.md doc/*`
- commit and create a github to push to and add the remote.

### Tests
Create a delay loop inside the main loop to stress serial port by sending 50 chars
01234567890123456789012345678901234567890123456789

Verify that number of received chars is multiple of 50





