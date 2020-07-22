/*
 * File:   pwm.c
 * Author: MWA692
 *
 * Created on July 13, 2020, 2:40 PM
 */


#include "xc.h"


uint8_t pulseCount = 0;

void pwm_init(void) {

    //Configuro Pin 15 como RP6 10010
    RPOR3bits.RP7R = 18;
//    //Configuro Pin 6 como RP2 ==> OC1 = 10010
//    RPOR1bits.RP2R = 18;

    /* T3 is used to generate interrupts.  T4 is used to provide an accurate
    time measurement. */
    T3CON = 0;
    T3CONbits.TCKPS = 0;
    T3CONbits.TCS = 0;
    T3CONbits.TGATE = 0;
    //El timer cuenta hasta 407 que equivale a medio periodo de 43kHz
    PR3 = 407;
    //Pongo en 0 el valor del timer
    TMR3 = 0;

    //Output Compare 1 will halt in CPU Idle mode
    OC1CONbits.OCSIDL = 1;
    //Timer3 is the clock source for Output Compare 1
    OC1CONbits.OCTSEL = 1;
    //SINGLE COMPARE MATCH MODE TOGGLE OUTPUT 
    OC1CONbits.OCM = 3;
    //Habilito interrupcion para cada toggle
    IPC0bits.OC1IP = 1; // Setup Output Compare 1 interrupt for
    IFS0bits.OC1IF = 0; // Clear Output Compare 1 interrupt flag
    IEC0bits.OC1IE = 1; // Enable Output Compare 1 interrupts
    
    //Configura pwm a 43kHz
    OC1R = 380;
}

void pwm_start(void) {
    //Reinicio timer
    TMR3 = 0;
    //Activo pwm
    OC1CONbits.OCM = 3;
    /* Start timer. */
    T3CONbits.TON = 1;
}

void pwm_stop(void) {
    //Apaga pwm
    OC1CONbits.OCM = 0;
    /* Apago timer. */
    T3CONbits.TON = 0;
}

/*Configura el comparador para enviar un tren de pulsos de longitud definida.
 No se debe iniciar el comparador antes con pwm_start();*/
void pwm_pulseTrain_bloq(uint8_t n) {
    //Reinicio timer
    TMR3 = 0;
    //Activo pwm
    OC1CONbits.OCM = 3;
    /* Start timer. */
    T3CONbits.TON = 1;
    while (pulseCount < (2*n)); //Bloquea hasta que se den n pulsos
    //Apaga pwm
    OC1CONbits.OCM = 0;
    /* Apago timer. */
    T3CONbits.TON = 0;
    //Reinicio contador global
    pulseCount = 0;
}

// Example code for Output Compare 1 ISR:

void __attribute__((__interrupt__)) _OC1Interrupt(void) {
    //Contador global de pulsos (cuenta cada semi-periodo)
    pulseCount++;
    //Limpio bandera
    IFS0bits.OC1IF = 0;
}
    