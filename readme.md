# Developing with MCUXpresso and SDK
Develop a project using the SDK components, record insights to share with colleagues and create a framework with at least two sreial ports. One of the serial will be used for Debug and shall retarget printf to provide debug trace.

Target is the FRDM-K22 board. Initially the debug port will be UART1 as it can by access using OSDA.


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


