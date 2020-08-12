/*
 * File:   adc.c
 * Author: MWA692
 *
 * Created on July 16, 2020, 10:06 AM
 */

#include "adc.h"

//uint16_t ADCvalue[1500];
//uint16_t indice1 = 0;

anemometro_deteccion_enum estadoDeteccion = PRIMER_LIMITE;
//uint16_t medicionesADC[1500];
//uint32_t indexADC = 0;
uint16_t LIMIT_SUP = 0;
uint16_t LIMIT_INF = 0;

void adc_init(void) {

    //AD1CON1 Register
    AD1CON1bits.FORM = 0; // Data Output Format: Unsigned Int (Dennis' Mod)
    AD1CON1bits.SSRC = 7; // Internal Counter (SAMC) ends sampling and starts convertion
    AD1CON1bits.ASAM = 1; // ADC Sample Control: Sampling begins immediately after conversion
    AD1CON1bits.SIMSAM = 0; // Sequencial Sampling/conversion
    AD1CON1bits.AD12B = 0; // 10-bit 2/4-channel operation
    AD1CON2bits.CHPS = 1; // Converts CH0/CH1


    //AD1CON3 Register        
    AD1CON3bits.ADRC = 0; // ADC Clock is derived from Systems Clock
    AD1CON3bits.SAMC = 0; // Auto Sample Time = 0*Tad
    AD1CON3bits.ADCS = 2; // ADC Conversion Clock TAD = TCY * (ADCS + 1) = (1/40M) * 3 =
    //                       75 ns (13.3 MHz)
    //                       ADC Conversion Time for 10-bit Tconv = 12 * TAD = 900 ns (1.1 MHz)

    //AD1CON4 Register  
    AD1CON4 = 0; // Allocate 1 words of buffer to each analog input
    // This register is not used in conversion order mode
    // This is required only in the scatter/gather mode

    //AD1CHS0/AD1CHS123: A/D Input Select Register
    AD1CHS0bits.CH0SA = 3; // MUXA +ve input selection (AIN5) for CH0
    AD1CHS0bits.CH0NA = 0; // MUXA -ve input selection (Vref-) for CH0

    AD1CHS123bits.CH123SA = 1; // CH1 positive input is AN3
    AD1CHS123bits.CH123NA = 0; // MUXA -ve input selection (Vref-) for CH1

    //AD1PCFGH/AD1PCFGL: Port Configuration Register
    AD1PCFGL = 0xFFFF;
    //AD1PCFGH=0xFFFF;
    //Configura pin AN5 para usar el ADC
    //    AD1PCFGLbits.PCFG5 = 0;
    //Configura pin AN3 para usar el ADC
    AD1PCFGLbits.PCFG3 = 0;

    //AD1CSSH/AD1CSSL: A/D Input Scan Selection Register
    AD1CSSL = 0x0000; // Channel Scan is disabled, default state

    IPC3bits.AD1IP = 3; //Interrupt priority 1
    IFS0bits.AD1IF = 0; // Clear the A/D interrupt flag bit
    IEC0bits.AD1IE = 0; // Do Not Enable A/D interrupt 

    INTCON1bits.NSTDIS = 0; // Enable interrupt nesting
}

void adc_start(void) {
    IEC0bits.AD1IE = 1; // Enable ADC1 interrupts
    AD1CON1bits.ADON = 1; // Turn on ADC1
}

void adc_stop(void) {
    AD1CON1bits.ADON = 0; // Turn off ADC1
    IFS0bits.AD1IF = 0; // Clear the A/D interrupt flag bit
    IEC0bits.AD1IE = 0; // Do Not Enable A/D interrupt 
}

void adc_transdSelect(mux_transSelect_enum transd) {
    switch (transd) {
        case TRANS_EMISOR_NORTE:
            LIMIT_SUP = LIMIT_SUP_N;
            LIMIT_INF = LIMIT_INF_N;
            break;
        case TRANS_EMISOR_SUR:
            LIMIT_SUP = LIMIT_SUP_S;
            LIMIT_INF = LIMIT_INF_S;
            break;
        case TRANS_EMISOR_ESTE:
            LIMIT_SUP = LIMIT_SUP_E;
            LIMIT_INF = LIMIT_INF_E;
            break;
        case TRANS_EMISOR_OESTE:
            LIMIT_SUP = LIMIT_SUP_O;
            LIMIT_INF = LIMIT_INF_O;
            break;
        default: LIMIT_SUP = LIMIT_SUP_N;
            LIMIT_INF = LIMIT_INF_N;
    }
}

void __attribute__((interrupt, no_auto_psv)) _ADC1Interrupt(void) {
    BaseType_t xTaskWoken = pdFALSE;
    uint16_t ADCval = 0;

    /*Detecto el tren de pulsos directamente en la ISR*/
    ADCval = ADC1BUF0;

    switch (estadoDeteccion) {
        case PRIMER_LIMITE:
            if (ADCval > LIMIT_SUP) estadoDeteccion = SEGUNDO_LIMITE;
            break;
        case SEGUNDO_LIMITE:
            if (ADCval < LIMIT_INF) {
                IEC0bits.AD1IE = 0; // Do Not Enable A/D interrupt
                estadoDeteccion = PRIMER_LIMITE;
                anemometroTdetect(&xTaskWoken);
            }
            break;
        default: estadoDeteccion = PRIMER_LIMITE;
    }
    IFS0bits.AD1IF = 0; // Clear ADC1 interrupt flag

    if (xTaskWoken != pdFALSE) {
        taskYIELD();
    }
}

