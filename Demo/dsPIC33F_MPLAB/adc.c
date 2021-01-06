/*
 * File:   adc.c
 * Author: MWA692
 *
 * Created on July 16, 2020, 10:06 AM
 */

#include "adc.h"

/*Global variables*/
uint16_t DmaBuffer = 0;
static unsigned int dataN[1024];
uint16_t dataSamples = 0;

//uint8_t thE = 1;
//uint8_t thO = 0;

static unsigned int BufferA[N_DMA_SAMP] __attribute__((space(dma)));
static unsigned int BufferB[N_DMA_SAMP] __attribute__((space(dma)));
static uint8_t detect_sample_O = 0;
static uint8_t detect_sample_E = 0;
static uint8_t detect_sample_N = 0;
static uint8_t detect_sample_S = 0;

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
    DmaBuffer = 0;

    //    IEC0bits.AD1IE = 1; // Enable ADC1 interrupts
    DMA0CONbits.CHEN = 1; //enable DMA
    AD1CON1bits.ADON = 1; // Turn on ADC1
}

void initDma0(void) {
    DMA0CONbits.AMODE = 0; // Configure DMA for Register indirect with post increment	
    DMA0CONbits.MODE = 2; // Configure DMA for Continuous Ping-Pong mode
    //    DMA0CONbits.MODE = 2; // Configure DMA for Continuous Ping-Pong mode
    DMA0PAD = (volatile unsigned int) &ADC1BUF0;
    DMA0CNT = (N_DMA_SAMP - 1);
    DMA0REQ = 13;
    DMA0STA = __builtin_dmaoffset(BufferA);
    DMA0STB = __builtin_dmaoffset(BufferB);
    IFS0bits.DMA0IF = 0; //Clear the DMA interrupt flag bit	
    IEC0bits.DMA0IE = 1; //Set the DMA interrupt enable bit	
    //    DMA0CONbits.CHEN = 1; //enable	
}

void adc_stop(void) {
    AD1CON1bits.ADON = 0; // Turn off ADC1

    IFS0bits.AD1IF = 0; // Clear the A/D interrupt flag bit

    IEC0bits.AD1IE = 0; // Do Not Enable A/D interrupt 

    DMA0CONbits.CHEN = 0; //Disable DMA
}

BaseType_t dma_ceroAligned(mux_transSelect_enum coordAligned) {
    uint8_t indexAligned = 0;

    switch (coordAligned) {
        case TRANS_EMISOR_OESTE:
            indexAligned = detect_sample_O;
            break;
        case TRANS_EMISOR_ESTE:
            indexAligned = detect_sample_E;
            break;
        case TRANS_EMISOR_NORTE:
            indexAligned = detect_sample_N;
            break;
        case TRANS_EMISOR_SUR:
            indexAligned = detect_sample_S;
            break;
        default: indexAligned = 0;
    }

    //Verifico si estoy en un cero con derivada negativa
    if ((BufferA[indexAligned] < LIMIT_SUPERIOR && BufferA[indexAligned] > LIMIT_INF) &&
            BufferA[indexAligned] > BufferA[indexAligned + 1]) {
        return pdPASS;
    } else {
        if (dma_ceroCalib(coordAligned) == pdPASS) return pdPASS;
    }
    return pdFAIL;
}

BaseType_t dma_ceroCalib(mux_transSelect_enum coordCalib) {
    uint16_t indexCalib = 0;
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

BaseType_t dma_capturePulse(mux_transSelect_enum coordCapture) {
    uint8_t i = 0;
    uint32_t ulNotificationValue;
    BaseType_t notifyStatus = pdFAIL;
    char msgN[6];
    uint16_t j = 0;

    dataSamples = 0;

    while (dataSamples < DMA_TOTAL_SAMP) {
        notifyStatus = xTaskNotifyWait(0, UINT32_MAX, &ulNotificationValue, 2 / portTICK_PERIOD_MS);

        if ((ulNotificationValue & 0x01) != 0 && notifyStatus == pdPASS) {
            if (DmaBuffer & 0x01) {
                for (i = 0; i < N_DMA_SAMP; i++) {
                    dataN[dataSamples] = BufferA[i];
                    dataSamples++;
                }
            } else {
                for (i = 0; i < N_DMA_SAMP; i++) {
                    dataN[dataSamples] = BufferB[i];
                    dataSamples++;
                }
            }
        } else {
            return pdFAIL;
        }
    }

    adc_stop();
    //datos por UART
    for (j = 0; j < dataSamples; j++) {
        sprintf(msgN, "%3.0d\r\n%c", dataN[j], '\0');
        uartSend((uint8_t *) msgN, sizeof (msgN), portMAX_DELAY);
    }
    return pdPASS;
}

BaseType_t dma_detectPulse(mux_transSelect_enum coordDetect, float* time) {
    anemometro_deteccion_enum maxDetectionState = PRIMERA_MUESTRA;
    uint16_t i = 0;
    uint16_t maxIndex = 0;
    uint16_t minIndex = 0;
    unsigned int lastMax = dataN[dataSamples];
    unsigned int lastMin = dataN[dataSamples - 3];
    //    unsigned int lastVal = dataN[0];
    char msgN[9];

    //Analizo de atras para adelante porque el ruido es demasiado 
    //N-S: aparentemente miden bien asi
    //O: probar detectando el minimo
    i = dataSamples;

    //Detecto el MAXIMO
    while (1) {
        switch (maxDetectionState) {
            case PRIMERA_MUESTRA:
                //Veo si estoy bajando o subiendo
                if (dataN[i] > dataN[i - 1]) {
                    maxDetectionState = MINIMO_LOCAL;
                } else if (dataN[i] < dataN[i - 1]) {
                    maxDetectionState = MAXIMO_LOCAL;
                } else {
                    i--;
                    if (i == 0) return pdFAIL;
                }
                break;
            case MAXIMO_LOCAL:
                //Busco el maximo local
                while (dataN[i - 1] >= dataN[i]) {
                    i--;
                    if (i == 0) return pdFAIL;
                }
                //Busco el maximo global (comparo con el mayor 'maximo local' encontrado)

                if ((coordDetect == TRANS_EMISOR_OESTE && (dataN[i] >= (lastMax + MAX_THRESHOLD_O)))
                        || (coordDetect == TRANS_EMISOR_ESTE && (dataN[i] >= (lastMax + MAX_THRESHOLD_E)))
                        || (coordDetect == TRANS_EMISOR_NORTE && (dataN[i] >= (lastMax + MAX_THRESHOLD_N)))
                        || (coordDetect == TRANS_EMISOR_SUR && (dataN[i] >= (lastMax + MAX_THRESHOLD_S)))
                        ) {
                    lastMax = dataN[i];
                    maxIndex = i;
                    i--;
                    if (i == 0) maxDetectionState = MAXIMO_GLOBAL;
                    maxDetectionState = MINIMO_LOCAL;
                } else if (((coordDetect == TRANS_EMISOR_OESTE && dataN[i] < (lastMax * (DETECTION_THRESHOLD_O)))
                        || (coordDetect == TRANS_EMISOR_ESTE && dataN[i] < (lastMax * (DETECTION_THRESHOLD_E)))
                        || (coordDetect == TRANS_EMISOR_NORTE && dataN[i] < (lastMax * (DETECTION_THRESHOLD_N)))
                        || (coordDetect == TRANS_EMISOR_SUR && dataN[i] < (lastMax * (DETECTION_THRESHOLD_S))))
                        && lastMax > LIMIT_SAFETY) { //Si los maximos empiezan a decaer dejo de buscar
                    //                    maxIndex = i;
                    maxDetectionState = MAXIMO_GLOBAL;
                } else {
                    i--;
                    maxDetectionState = MINIMO_LOCAL;
                    if (i == 0) return pdFAIL;
                }
                break;
            case MINIMO_LOCAL:
                //Avanzo hasta que cambie la derivada a positiva
                while (dataN[i - 1] <= dataN[i]) {
                    i--;
                    if (i == 0) return pdFAIL;
                }
                maxDetectionState = MAXIMO_LOCAL;
                break;
            case MAXIMO_GLOBAL:
                //muestro indice (debug)
                //                    sprintf(msgN, "\r\nI: %3.0d%c", maxIndex + 1, '\0');
                //                    uartSend((uint8_t *) msgN, sizeof (msgN), portMAX_DELAY);
                //termino la busqueda 
                if (lastMax > LIMIT_SAFETY) {
                    *time = (float) (maxIndex + 1) / DMA_FREQ;

                    return pdPASS;
                }
                return pdFAIL;

                break;
            default: maxDetectionState = PRIMERA_MUESTRA;
        }
    }
    return pdFAIL;
}

void __attribute__((interrupt, no_auto_psv))_DMA0Interrupt(void) {
    BaseType_t xTaskWoken = pdFALSE;

    DmaBuffer++;

    anemometroTdetectedFromISR(&xTaskWoken, 0x01);

    IFS0bits.DMA0IF = 0;

    if (xTaskWoken != pdFALSE) {
        taskYIELD();
    }
} 