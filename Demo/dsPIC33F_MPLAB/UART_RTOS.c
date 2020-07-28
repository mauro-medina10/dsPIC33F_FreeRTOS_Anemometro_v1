/*
 * File:   UART_RTOS.c
 * Author: MWA692
 *
 * Created on July 28, 2020, 10:22 AM
 */


#include "UART_RTOS.h"

void UART_initRTOS(void){
	// configure U2MODE
	U1MODEbits.UARTEN = 0;	// Bit15 TX, RX DISABLED, ENABLE at end of func
	//U2MODEbits.notimplemented;	// Bit14
	U1MODEbits.USIDL = 0;	// Bit13 Continue in Idle
	U1MODEbits.IREN = 0;	// Bit12 No IR translation
	U1MODEbits.RTSMD = 0;	// Bit11 Simplex Mode
	//U2MODEbits.notimplemented;	// Bit10
	U1MODEbits.UEN = 0;		// Bits8,9 TX,RX enabled, CTS,RTS not
	U1MODEbits.WAKE = 0;	// Bit7 No Wake up (since we don't sleep here)
	U1MODEbits.LPBACK = 0;	// Bit6 No Loop Back
	U1MODEbits.ABAUD = 0;	// Bit5 No Autobaud (would require sending '55')
	U1MODEbits.URXINV = 0;	// Bit4 IdleState = 1  (for dsPIC)
	U1MODEbits.BRGH = 1;	// Bit3 16 clocks per bit period
	U1MODEbits.PDSEL = 0;	// Bits1,2 8bit, No Parity
	U1MODEbits.STSEL = 0;	// Bit0 One Stop Bit
	
	// Load a value into Baud Rate Generator.  Example is for 9600.
	// See section 19.3.1 of datasheet.
	//  U2BRG = (Fcy/(4*BaudRate))-1
	//  U2BRG = (35M/(4*115200))-1		
	//  U2BRG = 75
	U1BRG = 75;	// 40Mhz osc, 9600 Baud

	// Load all values in for U1STA SFR
	U1STAbits.UTXISEL1 = 0;	//Bit15 Int when Char is transferred (1/2 config!)
	U1STAbits.UTXINV = 0;	//Bit14 N/A, IRDA config
	U1STAbits.UTXISEL0 = 0;	//Bit13 Other half of Bit15
	//U2STAbits.notimplemented = 0;	//Bit12
	U1STAbits.UTXBRK = 0;	//Bit11 Disabled
	U1STAbits.UTXEN = 0;	//Bit10 TX pins controlled by periph
	U1STAbits.UTXBF = 0;	//Bit9 *Read Only Bit*
	U1STAbits.TRMT = 0;	//Bit8 *Read Only bit*
	U1STAbits.URXISEL = 0;	//Bits6,7 Int. on character recieved
	U1STAbits.ADDEN = 0;	//Bit5 Address Detect Disabled
	U1STAbits.RIDLE = 0;	//Bit4 *Read Only Bit*
	U1STAbits.PERR = 0;		//Bit3 *Read Only Bit*
	U1STAbits.FERR = 0;		//Bit2 *Read Only Bit*
	U1STAbits.OERR = 0;		//Bit1 *Read Only Bit*
	U1STAbits.URXDA = 0;	//Bit0 *Read Only Bit*

	IPC7 = 0x4400;	// Mid Range Interrupt Priority level 4, no urgent reason

	IFS1bits.U2TXIF = 0;	// Clear the Transmit Interrupt Flag
	IEC1bits.U2TXIE = 1;	// Enable Transmit Interrupts
	IFS1bits.U2RXIF = 0;	// Clear the Recieve Interrupt Flag
	IEC1bits.U2RXIE = 1;	// Enable Recieve Interrupts

	U1MODEbits.UARTEN = 1;	// And turn the peripheral on

	U1STAbits.UTXEN = 1;
	// I think I have the thing working now. lol
}

void __attribute__ ((interrupt, no_auto_psv)) _U1RXInterrupt(void) {
	LATA = U1RXREG;
	IFS0bits.U1RXIF = 0;
}

void __attribute__ ((interrupt, no_auto_psv)) _U1TXInterrupt(void) {
	IFS0bits.U1TXIF = 0;
}