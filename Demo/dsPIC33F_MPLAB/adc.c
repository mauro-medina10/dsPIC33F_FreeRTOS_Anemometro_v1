/*
 * File:   adc.c
 * Author: MWA692
 *
 * Created on July 16, 2020, 10:06 AM
 */

#include "adc.h"

anemometro_deteccion_enum estadoDeteccion = PRIMERA_SAMPLE;
//uint16_t medicionesADC[500];
uint32_t indexADC = 0;
////Nuevo metodo de deteccion
uint16_t ADCmaximosCount = 0;
uint16_t ADClastRead = 0;
uint8_t ADCsecondMaxFlag = 0;
uint8_t ADCsafetyFlag = 1;

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

    //ADMA config
    //    AD1CON1bits.ADDMABM = 1; // DMA buffers are built in conversion order mode
    //    AD1CON2bits.SMPI = 0; // SMPI must be 0

    //AD1CHS0/AD1CHS123: A/D Input Select Register
    AD1CHS0bits.CH0SA = 5; // MUXA +ve input selection (AIN5) for CH0
    AD1CHS0bits.CH0NA = 0; // MUXA -ve input selection (Vref-) for CH0

    AD1CHS123bits.CH123SA = 1; // CH1 positive input is AN3
    AD1CHS123bits.CH123NA = 0; // MUXA -ve input selection (Vref-) for CH1

    //AD1PCFGH/AD1PCFGL: Port Configuration Register
    AD1PCFGL = 0xFFFF;
    //AD1PCFGH=0xFFFF;
    //Configura pin AN5 para usar el ADC
    AD1PCFGLbits.PCFG5 = 0;
    //Configura pin AN3 para usar el ADC
    AD1PCFGLbits.PCFG3 = 0;

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
    uint8_t i = 0;
    wind_medicion_type aux;

    AD1CON1bits.ADON = 0; // Turn off ADC1
    IFS0bits.AD1IF = 0; // Clear the A/D interrupt flag bit
    IEC0bits.AD1IE = 0; // Do Not Enable A/D interrupt 
    ADCmaximosCount = 0;
    estadoDeteccion = PRIMERA_SAMPLE;
    //    ADCsecondMaxFlag = 0;
    ADClastRead = 0;
    //    ADCsafetyFlag = 0;

    //    for (i = 0; i < indexADC; i++) {
    //        aux.mag = (float) medicionesADC[i];
    //        aux.deg = 0;
    //        uartSendMed(aux);
    //    }
    //    indexADC = 0;
}

void __attribute__((interrupt, no_auto_psv)) _ADC1Interrupt(void) {
    BaseType_t xTaskWoken = pdFALSE;
    uint16_t ADCval = 0;

    /*Detecto el tren de pulsos directamente en la ISR*/
    ADCval = ADC1BUF0;

    //    medicionesADC[indexADC] = ADCval;
    //
    //    if (indexADC == 400) {
    //        //                while (1);
    //        IEC0bits.AD1IE = 0; // Do Not Enable A/D interrupt
    //        anemometroTdetected(&xTaskWoken, 1);
    //    }
    //    indexADC++;

    /*Detecto Tercer maximo*/
    switch (estadoDeteccion) {
        case SEMI_POSITIVO:
            if (ADCval > ADClastRead) {
                estadoDeteccion = SEMI_NEGATIVO;
            }
            break;
        case SEMI_NEGATIVO:
            if (ADCval < ADClastRead) { //Maximo detectado
                ADCmaximosCount++;
                estadoDeteccion = SEMI_POSITIVO;
            }
            if (ADCmaximosCount == 3) {
                if (ADCval > LIMIT_SAFETY) {
                    IEC0bits.AD1IE = 0; // Do Not Enable A/D interrupt
                    /*Seteo el bit 0*/
                    anemometroTdetected(&xTaskWoken, 0x01);
                } else {
                    IEC0bits.AD1IE = 0;
                    /*Seteo bit 1*/
                    anemometroTdetected(&xTaskWoken, 0x02);
                }
            }
            break;
        case PRIMERA_SAMPLE:
            estadoDeteccion = SEMI_POSITIVO;
            break;
        default: estadoDeteccion = PRIMERA_SAMPLE;
    }
    ADClastRead = ADCval;

    IFS0bits.AD1IF = 0; // Clear ADC1 interrupt flag

    if (xTaskWoken != pdFALSE) {
        taskYIELD();
    }
}