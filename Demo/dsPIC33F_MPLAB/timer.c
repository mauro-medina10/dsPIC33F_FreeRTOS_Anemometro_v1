/*
 * File:   timer.c
 * Author: MWA692
 *
 * Created on August 8, 2020, 11:12 AM
 */


#include "xc.h"
#include <stdint.h>
#include <stdio.h> 

/*anemometro headers*/
#include <anemometroDef.h>
#include <timer.h>

/*Configura los timers 4 y 5 para trabajar como uno de 32bits*/
void timerInit(void) {
    T5CONbits.TON = 0; // Stop any 16-bit Timer5 operation
    T4CONbits.TON = 0; // Stop any 16/32-bit Timer4 operation
    T4CONbits.T32 = 1; // Enable 32-bit Timer mode
    T4CONbits.TCS = 0; // Select internal instruction cycle clock
    T4CONbits.TGATE = 0; // Disable Gated Timer mode
    T4CONbits.TCKPS = 0b00; // Select 1:1 Prescaler
    TMR5 = 0x00; // Clear 32-bit Timer (msw)
    TMR4 = 0x00; // Clear 32-bit Timer (lsw)
    PR5 = 0x0002; // Load 32-bit period value (msw)
    PR4 = 0x0000; // Load 32-bit period value (lsw)

    /*No necesito interrupciones por ahora*/
    //    IPC7bits.T5IP = 0x01; // Set Timer5 interrupt Priority Level
    //    IFS1bits.T5IF = 0; // Clear Timer5 Interrupt Flag
    //    IEC1bits.T5IE = 1; // Enable Timer5 interrupt
    //    T4CONbits.TON = 1; // Start 32-bit Timer
}

void timerStart(void) {
    TMR5 = 0x00; // Clear 32-bit Timer (msw)
    TMR4 = 0x00; // Clear 32-bit Timer (lsw)
    T4CONbits.TON = 1; // Start 32-bit Timer
}

void timerStop(void) {
    T4CONbits.TON = 0; // Start 32-bit Timer
}

uint32_t timerCount(void) {
    uint32_t lsw = 0, msw = 0;
    uint32_t timerVal = 0;

    // Reading from 32-bit timer
    lsw = TMR4; // Read lsw from the Type B timer register
    msw = TMR5HLD; // Read msw from the Type C timer holding register

    timerVal = (msw << 16) | lsw;
    
    return timerVal;
}

//void __attribute__((__interrupt__, no_auto_psv)) _T5Interrupt(void) {
//    /* Interrupt Service Routine code goes here */
//    IFS1bits.T5IF = 0; // Clear Timer3 Interrupt Flag
//}