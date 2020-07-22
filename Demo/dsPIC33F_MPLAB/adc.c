/*
 * File:   adc.c
 * Author: MWA692
 *
 * Created on July 16, 2020, 10:06 AM
 */

#include "xc.h"

uint16_t ADCvalue[128];
uint8_t indice = 0;

void adc_init(void) {

    //AD1CON1 Register
    AD1CON1bits.FORM = 0; // Data Output Format: Unsigned Int (Dennis' Mod)
    AD1CON1bits.SSRC = 7; // Internal Counter (SAMC) ends sampling and starts convertion
    AD1CON1bits.ASAM = 1; // ADC Sample Control: Sampling begins immediately after conversion
    AD1CON1bits.SIMSAM = 0; // Sequencial Sampling/conversion
    AD1CON1bits.AD12B = 0; // 10-bit 2/4-channel operation
    //AD1CON1bits.ADDMABM = 1; // DMA buffers are built in conversion order mode

    //AD1CON2 Register
    //AD1CON2bits.SMPI = 0; // Increment DMA address every 1 sample/conversion
    AD1CON2bits.BUFM = 0;
    AD1CON2bits.CHPS = 3; // Converts CH0/CH1


    //AD1CON3 Register        
    AD1CON3bits.ADRC = 0; // ADC Clock is derived from Systems Clock
    AD1CON3bits.SAMC = 0; // Auto Sample Time = 0*Tad

    AD1CON3bits.ADCS = 5; // ADC Conversion Clock Tad=Tcy*(ADCS+1)

    //AD1CON4 Register  
    AD1CON4 = 0; // Allocate 1 words of buffer to each analog input
    // This register is not used in conversion order mode
    // This is required only in the scatter/gather mode

    //AD1CHS0/AD1CHS123: A/D Input Select Register
    AD1CHS0bits.CH0SA = 5; // MUXA +ve input selection (AIN0) for CH0
    AD1CHS0bits.CH0NA = 0; // MUXA -ve input selection (Vref-) for CH0

    AD1CHS123bits.CH123SA = 1; // CH3 positive input is AN5
    AD1CHS123bits.CH123NA = 0; // MUXA -ve input selection (Vref-) for CH1


    //AD1PCFGH/AD1PCFGL: Port Configuration Register
    AD1PCFGL = 0xFFFF;
    //AD1PCFGH=0xFFFF;
    //Configura pin AN5 para usar el ADC
    AD1PCFGLbits.PCFG5 = 0;

    //AD1CSSH/AD1CSSL: A/D Input Scan Selection Register
    AD1CSSL = 0x0000; // Channel Scan is disabled, default state


    IFS0bits.AD1IF = 0; // Clear the A/D interrupt flag bit
    IEC0bits.AD1IE = 0; // Do Not Enable A/D interrupt 

    INTCON1bits.NSTDIS = 1; // Disable interrupt nesting
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
void __attribute__((interrupt, no_auto_psv)) _ADC1Interrupt(void) {
    //Leo valor ADC
    ADCvalue[indice] = ADC1BUF0;
    indice++;
    if(indice == 128){
        AD1CON1bits.ADON = 0; // Turn off ADC1
    }
    
    IFS0bits.AD1IF = 0; // Clear ADC1 interrupt flag
}