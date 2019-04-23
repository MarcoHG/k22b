# Operation Mode
----------------------
There are three operation modes: RUN, COMMAND and AMPLIFIER mode. Select the operation mode that best fit the parameter you want to test or calibrate the system parameters and exercise the different amplifier commands. The controller starts in RUN mode, waiting for the sw

The amplifier and command mode use a host terminal through the usb port to communicate with the controller. The Run mode uses the connector with switch inputs to start the operations equivalent to duraglide, still +5V power need to be supplied, a USB power supply can be used.

## Amplifier Mode
This mode allows to manually enter commands that are sent directly to the amplifier to evaluate the effect on the motor. See the power solution reference manual for amplifier comands
Data received from the amplifier are displayed in the terminal as bytes enclosed in square brackets []  
Command and responses are shown in hexadecimal code

## Command Mode 
This mode allows to read and write the door parameters settings using a host computer. This mode also allows to sstart operations using the terminal like init, open, close.

## Command list
These are the commands supported for the operation of the controller, the token is the identifier to type to get access the parameter. To start a Init Sequence, type 
`Init=1`
followed bt a return
Token	|	min | Max | variable | description	
---|---|---|---|---
CHKS		|		0	|	127			|  checkSpeed     | 	Speed when reach end or close position
CLOSES	|   0	|	127    	|  closingSpeed   | 	Max speed during closing sequence      
APERTS	|   0	|	127    	|  apertureSpeed  | 	Max speed when openning
WAITO		|   0	|	40000U	|  waitAtOpenMsec | 	time to wait at full open position 
CHKP		|  	5	|	80    	|  checkPercent   | 	percent to change speed to checkSpeed
PARTOP	| 	5	|	50    	|  partialPercent | 	5 to 50                                     
CHKDIR	|   0	|	1   	  |  checkSpeedDir  | 	0 to 1 
CLOSECL |   0	|	127  	  |  closeCurrLim   | 	0 to 127                                      
MODE		|		0	|	2			  |  mode           |   Set operating mode 0:run, 1:manual, 2:amplifier      
OPE			|		0	|	2			  |  operation      |   Start sequence: 0:Init, 1:Open, 2:close

## Init Sequence
Type ope=0\r in mode 0 or 1
		+	Init	
			1- Start at _checkSpeed_ direction fwd
			2- End position when same encoder position at current limit, position recorded. Motor de-energized
			3- wait for 1 sec
			4- move at _checkSpeed_ in the opposite direction of 1
			5- End position when same encoder position at current limit, position recorded. Motor de-energized

		+ Open:						
	- run				- run using external inputs to simulate duraglide using loaded settings

Send and wait to receive a 0 or FF
send E1 and wait to receive 0
Send 8A 01	wait to receive FF 
Send 82 closeCurrLim 
Send 80 checkSpeed


		 block to auxSerial
setDirection(uint8_t dir)
{
	0	
}

## Macros teraterm
F2 : E1
F3 : 8A 01^M
F4 : 8A 02^M
F5 : 80 7F^M  
F6 : 80 15^M  
F7 : 80 0^M   
 
## Amplifier LOG
E1[0]
8A 01[ff]
80 7F[ff]80


set txOnGoing true, it will be cleared on complete
volatile bool rxFlag;






Estimate door length


lowspeed = 
start
E1
8A 1

measure 
82	05						// Set current limit
84	20						// Acel (1=step)
85	20						// Decel
8a  01						// Move Forward
80  lowspeed			// move at low speed
pos=0


until delta_pos < 5	and time_wall < 2 sec

 


 
open - 
E1^M8A 01^M80 5F^M




Current resistor R5 and R7 from 0.001 Ohm to 0.005




# Wiring
------------------------------------------
Motor Black to Amplifier motor -1
Motor White to amplifier motor +1
Encoder increases with positive on Black

## Arduino J2 Connector
Pin| Arduino | K22 Port | function
---|---|---|---
20 	| D15 	| PTE1 	| UART1_RX	(R71) (OSDA)
18 	| D14 	| PTE0 	| UART1_TX
16	| AREF 	| AREF 	| 
14	| GND 	| GND  	| 
12	| D13 	| PTD5 	| BLUE_LED 	
10	| D12 	| PTD7 	| 
9		| D11 	| PTD6 	| 	
6		| D10 	| PTD4 	| UART0_RTS
4		| D9 		| PTA1 	| UART0_RX to K4-2 TX (RED LED)
2		| D8 		| PTB19	| FTM2_QD_PHB	ENC-WHITE

## Arduino J1 Connector
pin | Arduino | K22 Port | function
----|---|---|---
16	| D7 	| PTC6 	| 
14	| D6 	| PTC3 	| 
12	| D5 	| PTB18 | FTM2_QD_PHA	ENC-GREEN
10	| D4 	| PTA4	| 
9		| D3 	| PTA2 	| UART0_TX	K4-3 RX
6		| D2 	| PTB16	| 
4		| D1 	| PTD3 	| UART2_TX
2		| D0 	| PTD2	| UART2_RX

## Arduino J25 Connector
pin | Arduino | K22 Port | function
----|---|---|---
2   |  NC   |  
4   |  RST  |  
8   |  3.3V |  
10  |  5V   |  
12  |  GND  |  
14  |  GND  |  
16  |  VIN  |  

## Arduino J24 Connector
pin | Arduino | K22 Port | function
----|---|---|---
2   | A0  |  PTB0  | 	Reduce Opening
4   | A1  |  PTB1  |	Breakout
6   | A2  |  PTC1  |	Operate	(SW2)
8   | A3  |  PTC2  |
10  | A4  |  PTB3  |	SDA EEPROM
12  | A5  |  PTB2  |	SCL

PTC1	SW2		A2
PTB17	SW3


PTA1	RED_LED			// UART0_RX to K4-2 
PTA2	GREEN_LED		// Added P{
PTD5	BLUE_LED






	
