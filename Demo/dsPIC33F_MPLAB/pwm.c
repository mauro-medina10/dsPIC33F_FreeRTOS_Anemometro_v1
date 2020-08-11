/*
 * File:   pwm.c
 * Author: MWA692
 *
 * Created on July 13, 2020, 2:40 PM
 */

#include "pwm.h"

/*FreeRTOS definitions*/
static void comparador_task(void *pvParameters);

SemaphoreHandle_t xSemaphoreComparadorPulsos;

QueueHandle_t xQueueComparadorNumPulsos;

/*Global variables definitions*/
uint8_t pulseCount = 0;

void comparadorInit(void) {
    //Semaforo usado en la interrupcion
    xSemaphoreComparadorPulsos = xSemaphoreCreateBinary();

    if(xSemaphoreComparadorPulsos == NULL) while(1);
    
    /* T3 is used to generate interrupts.  T4 is used to provide an accurate
       time measurement. */
    T3CONbits.TON = 0;
    T3CON = 0;
    T3CONbits.TCKPS = 0;
    T3CONbits.TCS = 0;
    T3CONbits.TGATE = 0;
    //El timer cuenta hasta 407 que equivale a medio periodo de 43kHz
    PR3 = (uint16_t) (configCPU_CLOCK_HZ / OUTPUT_FREQ) / 2;
    //Pongo en 0 el valor del timer
    TMR3 = 0;

    //Output Compare 1 will halt in CPU Idle mode
    OC1CONbits.OCSIDL = 1;
    //Timer3 is the clock source for Output Compare 1
    OC1CONbits.OCTSEL = 1;
    //SINGLE COMPARE MATCH MODE TOGGLE OUTPUT 
    OC1CONbits.OCM = 3;

    //Configura pwm a 43kHz
    OC1R = 20;

    //Habilito interrupcion para cada toggle
    IPC0bits.OC1IP = 3; // Setup Output Compare 1 interrupt for
    IFS0bits.OC1IF = 0; // Clear Output Compare 1 interrupt flag
    IEC0bits.OC1IE = 1; // Enable Output Compare 1 interrupts

    //Configuro Pin 6 como RP2: OC1 = 10010
    RPOR1bits.RP2R = 18;
}

void comparador_rtos_init(void) {

    xQueueComparadorNumPulsos = xQueueCreate(10, sizeof ( uint8_t));

    if (xQueueComparadorNumPulsos == NULL) {
        while (1);
    };

    if (xTaskCreate(comparador_task, "comparador_task", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 3, NULL) != pdPASS) {
        while (1);
    }

    comparadorInit();
}

void comparadorStart(void) {
    //Reinicio timer
    TMR3 = 0;
    //Activo pwm
    OC1CONbits.OCM = 3;
    /* Start timer. */
    T3CONbits.TON = 1;
}

void comparadorStop(void) {
    //Apaga pwm
    OC1CONbits.OCM = 0;
    /* Apago timer. */
    T3CONbits.TON = 0;
}

/*Configura el comparador para enviar un tren de pulsos de longitud definida.
 No se debe iniciar el comparador antes con pwm_start();*/
void comparadorPulseTrain_bloq(uint8_t n) {
    //Desactivo filtro receptor
    filtroDisable();

    comparadorStart();

    while (pulseCount < (2 * n)); //Bloquea hasta que se den n pulsos   

    comparadorStop();
    //Activo filtro receptor
    filtroEnable();

    //Reinicio contador global
    pulseCount = 0;
}

void comparadorPulseTrain_NObloq(uint8_t nPulsos) {
    pulseCount = 0;

    comparadorStart();

    while (pulseCount < (2 * nPulsos)) {
        xSemaphoreTake(xSemaphoreComparadorPulsos, portMAX_DELAY);
    }
    comparadorStop();

    //Activo Mux 2
    MUX_INPUT_INH(0);
    
    pulseCount = 0;
}

void comparadorPulseTrainRTOS(uint8_t n) {
    uint8_t nmrPulsos = n;

    if (xQueueSend(xQueueComparadorNumPulsos, &nmrPulsos, 0) != pdPASS) {
        while (1);
    }
}

static void comparador_task(void *pvParameters) {
    uint8_t nPulsos = 0;

    while (1) {

        xQueueReceive(xQueueComparadorNumPulsos, &nPulsos, portMAX_DELAY);
        //Desactivo filtro receptor
        //filtroDisable();

        comparadorStart();

        while (pulseCount < (2 * nPulsos)) {
            xSemaphoreTake(xSemaphoreComparadorPulsos, portMAX_DELAY);
        }
        //Activo filtro receptor
        //filtroEnable();

        comparadorStop();


        pulseCount = 0;
    }
}

void filtroEnable(void) {
    TRISBbits.TRISB0 = 1;

    //    AD1PCFGLbits.PCFG2 = 0;
}

void filtroDisable(void) {
    //    AD1PCFGLbits.PCFG2 = 1;

    TRISBbits.TRISB0 = 0;

    PORTBbits.RB0 = 0;
}

void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void) {
#ifdef RTOS_AVAILABLE
    BaseType_t xTaskWoken = pdFALSE;

    //Contador global de pulsos (cuenta cada semi-periodo)
    pulseCount++;

    xSemaphoreGiveFromISR(xSemaphoreComparadorPulsos, &xTaskWoken);

    //Limpio bandera
    IFS0bits.OC1IF = 0;

    if (xTaskWoken != pdFALSE) {
        taskYIELD();
    }
#else
    //Contador global de pulsos (cuenta cada semi-periodo)
    pulseCount++;

    //Limpio bandera
    IFS0bits.OC1IF = 0;
#endif
}

