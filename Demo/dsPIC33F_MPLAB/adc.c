/*
 * File:   adc.c
 * Author: MWA692
 *
 * Created on July 16, 2020, 10:06 AM
 */

#include "adc.h"

//static anemometro_deteccion_enum estadoDeteccion = PRIMERA_SAMPLE;
//uint16_t medicionesADC[200];
//static uint32_t indexADC = 0;
////Nuevo metodo de deteccion
//static uint16_t ADCmaximosCount = 0;
//static uint16_t ADClastRead = 0;
//unsigned int DmaBuffer = 0;

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
    AD1CON1bits.ADDMABM = 1; // DMA buffers are built in conversion order mode
    AD1CON2bits.SMPI = 0; // SMPI must be 0

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

    initDma0();
}

void adc_start(void) {
    //    IEC0bits.AD1IE = 1; // Enable ADC1 interrupts
    DMA0CONbits.CHEN = 1; //enable DMA
    AD1CON1bits.ADON = 1; // Turn on ADC1
}

void initDma0(void) {
    DMA0CONbits.AMODE = 0; // Configure DMA for Register indirect with post increment	
    DMA0CONbits.MODE = 1; // One-Shot, Ping-Pong modes disabled
    //    DMA0CONbits.MODE = 2; // Configure DMA for Continuous Ping-Pong mode
    DMA0PAD = (volatile unsigned int) &ADC1BUF0;
    DMA0CNT = (N_DMA_SAMP - 1);
    DMA0REQ = 13;
    DMA0STA = __builtin_dmaoffset(BufferA);
    //    DMA0STB = __builtin_dmaoffset(BufferB);
    IFS0bits.DMA0IF = 0; //Clear the DMA interrupt flag bit	
    IEC0bits.DMA0IE = 1; //Set the DMA interrupt enable bit	
    //    DMA0CONbits.CHEN = 1; //enable	
}

void adc_stop(void) {
    AD1CON1bits.ADON = 0; // Turn off ADC1

    IFS0bits.AD1IF = 0; // Clear the A/D interrupt flag bit

    IEC0bits.AD1IE = 0; // Do Not Enable A/D interrupt 
}

//void __attribute__((interrupt, no_auto_psv)) _ADC1Interrupt(void) {
//    static BaseType_t xTaskWoken = pdFALSE;
//    static uint16_t ADCval = 0;
//
//    /*Detecto el tren de pulsos directamente en la ISR*/
//    ADCval = ADC1BUF0;
//
//    //    medicionesADC[indexADC] = ADCval;
//    //
//    //    if (indexADC == 200) {
//    //        //                while (1);
//    //        IEC0bits.AD1IE = 0; // Do Not Enable A/D interrupt
//    //        anemometroTdetected(&xTaskWoken, 1);
//    //    }
//    indexADC++;
//
//    /*Detecto Tercer maximo*/
//    switch (estadoDeteccion) {
//        case SEMI_POSITIVO:
//            if (ADCval > ADClastRead) {
//                estadoDeteccion = SEMI_NEGATIVO;
//            }
//            break;
//        case SEMI_NEGATIVO:
//            if (ADCval < ADClastRead) { //Maximo detectado
//                ADCmaximosCount++;
//                estadoDeteccion = SEMI_POSITIVO;
//            }
//            if (ADCmaximosCount == 3) {
//                if (ADCval > LIMIT_SAFETY) {
//                    IEC0bits.AD1IE = 0; // Do Not Enable A/D interrupt
//                    /*Seteo el bit 0*/
//                    anemometroTdetected(&xTaskWoken, 0x01);
//                } else {
//                    IEC0bits.AD1IE = 0;
//                    /*Seteo bit 1*/
//                    anemometroTdetected(&xTaskWoken, 0x02);
//                }
//            }
//            break;
//        case PRIMERA_SAMPLE:
//            estadoDeteccion = SEMI_POSITIVO;
//            break;
//        default: estadoDeteccion = PRIMERA_SAMPLE;
//    }
//    ADClastRead = ADCval;
//
//    IFS0bits.AD1IF = 0; // Clear ADC1 interrupt flag
//
//    if (xTaskWoken != pdFALSE) {
//        taskYIELD();
//    }
//    xTaskWoken = pdFALSE;
//}

BaseType_t dma_ceroAligned(mux_transSelect_enum coordAligned) {
    if ((BufferA[0] < LIMIT_SUPERIOR && BufferA[0] > LIMIT_INF) &&
            BufferA[0] > BufferA[1]) {
        return pdPASS;
    } else {
        if (dma_ceroCalib(coordAligned) == pdPASS) return pdPASS;
    }
    return pdFAIL;
}

BaseType_t dma_ceroCalib(mux_transSelect_enum coordCalib) {
    uint8_t indexCalib = 0;
    uint8_t validDerivadaFlag = 0;

    //Busco el primer cero con derivada negativa
    while (validDerivadaFlag != 1) {
        while ((BufferA[indexCalib] >= LIMIT_SUPERIOR || BufferA[indexCalib] <= LIMIT_INF)) {
            indexCalib++;
            if (indexCalib > 255) return pdFAIL;
        }
        if (BufferA[indexCalib] > BufferA[indexCalib + 1]) {
            validDerivadaFlag = 1;
        } else {
            indexCalib++;
        }
    }

    switch (coordCalib) {
        case TRANS_EMISOR_OESTE:
            detect_sample_O = indexCalib;
            break;
        case TRANS_EMISOR_ESTE:
            detect_sample_E = indexCalib;
            break;
        case TRANS_EMISOR_NORTE:
            detect_sample_N = indexCalib;
            break;
        case TRANS_EMISOR_SUR:
            detect_sample_S = indexCalib;
            break;
        default: return pdFAIL;
    }
    return pdPASS;
}

float dma_detectPulse(mux_transSelect_enum coordDetect) {
    anemometro_deteccion_enum estadoDeteccion = PRIMERA_SAMPLE;
    uint8_t ADCcrucesCount = 0;
    uint8_t i = 0;
    unsigned int* buff = BufferA;
    float timeMed = 0;
    wind_medicion_type aux;
    unsigned int safetyFlag = 0;

    //Envio muestras por UART para graficar
    //    for (i = 0; i < N_DMA_SAMP; i++) {
    //        aux.mag = (float) BufferA[i];
    //        aux.deg = 0;
    //        uartSendMed(aux);
    //    }
    switch (coordDetect) {
        case TRANS_EMISOR_OESTE:
            buff = &BufferA[detect_sample_O];
            timeMed = (float) detect_sample_O / DMA_FREQ;
            break;
        case TRANS_EMISOR_ESTE:
            buff = &BufferA[detect_sample_E];
            timeMed = (float) detect_sample_E / DMA_FREQ;
            break;
        case TRANS_EMISOR_NORTE:
            buff = &BufferA[detect_sample_N];
            timeMed = (float) detect_sample_N / DMA_FREQ;
            break;
        case TRANS_EMISOR_SUR:
            buff = &BufferA[detect_sample_S];
            timeMed = (float) detect_sample_S / DMA_FREQ;
            break;
        default: buff = BufferA;
    }

    //Detecto el segundo cruce por cero
    for (i = 0; i < N_DMA_SAMP; i++) {
        switch (estadoDeteccion) {
            case SEMI_POSITIVO:
                if (*buff < LIMIT_INF) {
                    ADCcrucesCount++;
                    estadoDeteccion = SEMI_NEGATIVO;
                }
                break;
            case SEMI_NEGATIVO:
                if (*buff > LIMIT_SUPERIOR) { //Maximo detectado
                    ADCcrucesCount++;
                    estadoDeteccion = SEMI_POSITIVO;
                }
                if (ADCcrucesCount == 4) {
                    if (safetyFlag == 1) {
                        timeMed += (float) (i + 1) / DMA_FREQ;
                        return timeMed;
                    } else {
                        return 776.77;
                    }
                }
                break;
            case PRIMERA_SAMPLE:
                /* Si la señal adelanta cuento los cruces por cero,
                 * si la señal esta atrasada necesito contar un cruce de mas.
                 */
                if (*buff > LIMIT_SUPERIOR) {
                    estadoDeteccion = SEMI_POSITIVO;
                } else {
                    ADCcrucesCount++;
                    estadoDeteccion = SEMI_NEGATIVO;
                }
                break;
            default: estadoDeteccion = PRIMERA_SAMPLE;
        }
        if (*buff > LIMIT_SAFETY) safetyFlag = 1;
        buff++;
    }
    return 775.77;
}

void __attribute__((interrupt, no_auto_psv))_DMA0Interrupt(void) {
    BaseType_t xTaskWoken = pdFALSE;

    RB_9_SET(1);

    anemometroTdetected(&xTaskWoken, 0x01);

    IFS0bits.DMA0IF = 0;

    if (xTaskWoken != pdFALSE) {
        taskYIELD();
    }
} 