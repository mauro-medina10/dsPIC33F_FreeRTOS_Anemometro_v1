/*
 * File:   main.c
 * Author: MWA692
 *
 * Created on July 20, 2020, 2:46 PM
 */
#ifndef __dsPIC33FJ128GP802__
#define __dsPIC33FJ128GP802__
#endif

/*Standars includes*/
#include <stdint.h>
#include <stdio.h> 


/*Config includes*/
#include "xc.h"
#include <config.h>

/*FreeRTOS includes*/
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "list.h"
#include "queue.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/*Peripherals drivers includes*/
#include <anemometroDef.h>
#include <adc.h>
#include <pwm.h>
#include <UART_RTOS.h>
#include <timer.h>

/*Funciones locales*/
static void prvSetupHardware(void);

/*--------Tasks declaration---------*/
//static void led_test_task(void *pvParameters);
//static void transductor_test_task(void *pvParameters);

static void anemometro_main_task(void *pvParameters);

/*FreeRTOS declarations*/
static TaskHandle_t xTaskToNotify = NULL;
static QueueHandle_t qRecf;

/*Global variables*/
float detect_delta_O = DETECTION_ERROR_O;
float detect_delta_E = DETECTION_ERROR_E;
float detect_delta_N = DETECTION_ERROR_N;
float detect_delta_S = DETECTION_ERROR_S;

anemometro_mode_enum anemometroModoActivo = Menu;

uint8_t medProgFlag = 0;

int main(void) {
    //Inicio Hardware
    prvSetupHardware();

    //Output compare
    comparadorInit();
    //    comparador_rtos_init();

    //ADC init
    adc_init();

    //UART init
    uartInit_RTOS();
    //    uartInit();
    //Timer 32bits
    timerInit();

    //Queues init
    qRecf = xQueueCreate(5, sizeof (float));

    if (qRecf == NULL) {
        while (1);
    }

    //FreeRTOS inits
    if (xTaskCreate(anemometro_main_task,
            "anemometro_main_task",
            configMINIMAL_STACK_SIZE * 3,
            NULL,
            tskIDLE_PRIORITY + 3,
            &xTaskToNotify) != pdPASS) {
        while (1);
    }

    vTaskStartScheduler();

    while (1);

    return 0;
}

static void anemometro_main_task(void *pvParameters) {
    anemometro_config_enum configOption = ExitConfig;
    wind_medicion_type simpleMed = {0, 0};
    mux_transSelect_enum emisorSelect = TRANS_EMISOR_OESTE;
    uint16_t auxV = 0;
    TickType_t xLastWakeTime;
    unsigned int firstPeriodFlag = 1;
    float soundSpeed = 345.7;
    uint16_t medPeriod = 5;

    DELAY_100uS;

    //Desactivo MUX
    MUX_INPUT_INH(1);

    DELAY_100uS;

    while (1) {
        switch (anemometroModoActivo) {
            case Menu:
                //Bandera para inciar medicion periodica
                firstPeriodFlag = 1;
                //Enum del menu configuracion
                configOption = ExitConfig;
                //Espero a que se elija un modo de func
                anemometroModoActivo = uartGetMode();
                //Variables auxiliares para mediciones varias
                if (emisorSelect > TRANS_EMISOR_SUR) emisorSelect = TRANS_EMISOR_OESTE;
                auxV = 0;

                break;
            case Medicion_Simple:

                simpleMed = anemometroGetMed();
                //                simpleMed.mag = anemometroGetCoordTime(emisorSelect) * 1000000;

                uartSendMed(simpleMed);
                //                emisorSelect++;
                //                if (emisorSelect > TRANS_EMISOR_SUR) emisorSelect = TRANS_EMISOR_OESTE;

                anemometroModoActivo = Menu;
                break;
            case Medicion_Continua:

                if (firstPeriodFlag == 1) {
                    xLastWakeTime = xTaskGetTickCount();
                    firstPeriodFlag = 0;
                }

                medProgFlag = 1;

                simpleMed = anemometroGetMed();
                //                                simpleMed.mag = anemometroGetVcoord(emisorSelect);
                //                simpleMed.mag = anemometroGetCoordTime(emisorSelect) * 1000000;

                uartSendMed(simpleMed);

                //                auxV++;
                //                if (auxV >= 50) {
                //                    auxV = 0;
                //                    emisorSelect++;
                //                    //                    emisorSelect += 2;
                //                    if (emisorSelect > TRANS_EMISOR_SUR) {
                //                        emisorSelect = TRANS_EMISOR_OESTE;
                //                        uartEndMode();
                //                    }
                //                    //                    uartEndMode();
                //                }
                vTaskDelay(10 / portTICK_PERIOD_MS);

                anemometroModoActivo = uartGetMode();

                medProgFlag = 0;

                if (anemometroModoActivo == Medicion_Continua) {
                    vTaskDelayUntil(&xLastWakeTime, (medPeriod * 1000) / portTICK_PERIOD_MS);
                }
                break;
            case Configuracion:
                configOption = uartGetModeConfig();

                switch (configOption) {
                    case CalCero:
                        xQueueReceive(qRecf, &soundSpeed, portMAX_DELAY);

                        //                        anemometroDelayTest();

                        anemometroCalibCero(soundSpeed);

                        simpleMed.mag = detect_delta_O * 1000000;
                        simpleMed.deg = detect_delta_E * 1000000;
                        uartSendMed(simpleMed);

                        simpleMed.mag = detect_delta_N * 1000000;
                        simpleMed.deg = detect_delta_S * 1000000;
                        uartSendMed(simpleMed);
                        simpleMed.mag = 0;
                        simpleMed.deg = 0;

                        configOption = ExitConfig;
                        break;
                    case SetEmi:
                        configOption = uartGetModeConfig();
                        if (configOption == 1) emisorSelect = TRANS_EMISOR_NORTE;
                        if (configOption == 2) emisorSelect = TRANS_EMISOR_SUR;
                        if (configOption == 3) emisorSelect = TRANS_EMISOR_ESTE;
                        if (configOption == 4) emisorSelect = TRANS_EMISOR_OESTE;
                        configOption = ExitConfig;
                        break;
                    case SetPeriod:
                        configOption = uartGetModeConfig();
                        if (configOption == 1) medPeriod = 5;
                        if (configOption == 2) medPeriod = 10;
                        if (configOption == 3) medPeriod = 30;
                        if (configOption == 4) medPeriod = 60;
                        if (configOption == 5) medPeriod = 600;
                        configOption = ExitConfig;
                        break;
                    case ExitConfig:
                        anemometroModoActivo = Menu;
                        break;
                    default: anemometroModoActivo = Menu;
                }

                anemometroModoActivo = Menu;
                break;
            default: anemometroModoActivo = Menu;
        }
    }
}

float anemometroGetVcoord(mux_transSelect_enum coordV) {
    float valMed = 0;
    float timeMed[] = {0, 0};
    float timeNmediciones[N_TIMER_MODE];
    uint8_t i = 0;
    uint8_t n = 0;
    float timeCorr[2];

    for (i = 0; i < 2; i++) {
        for (n = 0; n < N_TIMER_MODE; n++) {

            timeNmediciones[n] = anemometroGetCoordTime(coordV + i);

            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        timeMed[i] = anemometroCalcMode(timeNmediciones, N_TIMER_MODE);
    }

    if (timeMed[0] == 0 || timeMed[1] == 0) {
        valMed = 666.66;
    } else {
        //Calculo 
        switch (coordV) {
            case TRANS_EMISOR_OESTE:
                //Corrijo los tiempos medidos
                timeCorr[0] = timeMed[0] - detect_delta_O; //taOE
                timeCorr[1] = timeMed[1] - detect_delta_E; //trOE

                if (timeCorr[0] == timeCorr[1]) {
                    return 0;
                }
                //Si los dos tiempos desfasan para el mismo lado la medicion carece de sentido
                //                if ((timeCorr[0] < DETECTION_CERO_MIN && timeCorr[1] < DETECTION_CERO_MIN) ||
                //                        (timeCorr[0] > DETECTION_CERO_MAX && timeCorr[1] > DETECTION_CERO_MAX)) {
                //                    return 555.55;
                //                }

                /*La definicion de las coordenadas es tal que si el viento sopla 
                 * DESDE el NORTE el angulo es 0°
                 *si sopla DESDE el OESTE el angulo es 90°
                 * El angulo va de 0° a 360°
                 */
                valMed = (((float) DISTANCE_EO / 2) * (1 / timeCorr[0] - 1 / timeCorr[1])) - OFFSET_ERROR_EO;
                valMed = (valMed - DETECT_OFFSET_OE) / DETECT_SCALING_OE;
                break;
            case TRANS_EMISOR_NORTE:
                //Corrijo los tiempos medidos
                timeCorr[0] = timeMed[0] - detect_delta_N;
                timeCorr[1] = timeMed[1] - detect_delta_S;

                if (timeCorr[0] == timeCorr[1]) {
                    return 0;
                }

                //                if ((timeCorr[0] < DETECTION_CERO_MIN && timeCorr[1] < DETECTION_CERO_MIN) ||
                //                        (timeCorr[0] > DETECTION_CERO_MAX && timeCorr[1] > DETECTION_CERO_MAX)) {
                //                    return 555.55;
                //                }

                valMed = (((float) DISTANCE_NS / 2) * (1 / timeCorr[0] - 1 / timeCorr[1])) - OFFSET_ERROR_NS;
                valMed = (valMed - DETECT_OFFSET_NS) / DETECT_SCALING_NS;
                break;
            default:
                valMed = 444.44;
        }
    }
    return valMed;
}

BaseType_t anemometroGetProm(float* medOE, float* medNS, uint8_t prom) {
    float medAcumOE = 0;
    float medAcumNS = 0;
    uint8_t medNpromOE = prom;
    uint8_t medNpromNS = prom;
    uint8_t i = 0;
    float medCoordOE = 0;
    float medCoordNS = 0;

    for (i = 0; i < prom; i++) {
        medCoordOE = anemometroGetVcoord(TRANS_EMISOR_OESTE);

        vTaskDelay(1 / portTICK_PERIOD_MS);

        medCoordNS = anemometroGetVcoord(TRANS_EMISOR_NORTE);

        if (medCoordOE < 100) {
            medAcumOE += medCoordOE;
        } else {
            medNpromOE--;
        }
        if (
                medCoordNS < 100) {
            medAcumNS += medCoordNS;
        } else {
            medNpromNS--;
        }
    }

    if (medNpromOE > (prom / 2) && medNpromNS > (prom / 2)) {
        *medOE = medAcumOE / medNpromOE;
        *medNS = medAcumNS / medNpromNS;

        return pdPASS;
    } else {
        *medOE = 666.66;
        *medNS = 666.66;
    }

    return pdFAIL;
}

float anemometroGetCoordTime(mux_transSelect_enum coordTime) {
    float timeMed = 999.99;
    uint32_t ulNotificationValue;
    BaseType_t notifyStatus = pdFAIL;
    BaseType_t pulseDetected = pdFALSE;
    BaseType_t pulseCaptured = pdFALSE;

    RB_9_SET(0);

    if (anemometroMuxOutputSelect(coordTime) != pdTRUE) {
        return timeMed;
    }

    DELAY_50uS;

    //    RB_9_SET(1);

    timerStart();

    comparadorPulseTrain_NObloq(TRAIN_PULSE_LENGTH);

    //Necesitaria esperar 400us
    DELAY_400uS;
    DELAY_100uS;
    //    DELAY_T;
    //    anemometroReceptorDelay(coordTime);

    timerStop();

    adc_start();

    pulseCaptured = dma_capturePulse(coordTime);

    adc_stop();

    if (pulseCaptured == pdPASS) {
        pulseDetected = dma_detectPulse(coordTime, &timeMed);

        timeMed += (((float) timerCount() / configCPU_CLOCK_HZ));

    } else {
        timeMed = 888.88;
    }

    return timeMed;
}

wind_medicion_type anemometroGetMed(void) {
    wind_medicion_type valMed = {0, 0};
    float VcoordOE = 0;
    float VcoordNS = 0;
    BaseType_t medResult = pdFAIL;

    //    //Mido en la coordenada O-E
    //    VcoordOE = anemometroGetMedProm(TRANS_EMISOR_OESTE, N_MED_PROM);
    //    //Mido en la coordenada N-S
    //    VcoordNS = anemometroGetMedProm(TRANS_EMISOR_NORTE, N_MED_PROM);
    //Tomo 10 promedios de cada coordenada 
    medResult = anemometroGetProm(&VcoordOE, &VcoordNS, N_MED_PROM);

    //Calculo 
    //    valMed.mag = sqrtf(powf(VcoordOE, 2) + powf(VcoordNS, 2
    //    if (VcoordOE < 555 && VcoordNS < 555) {
    if (medResult == pdPASS) {

        valMed.mag = sqrtf(VcoordOE * VcoordOE + VcoordNS * VcoordNS);

        if (valMed.mag == 0) {
            valMed.deg = 0;
            sprintf(valMed.coord, "   ");
            return valMed;
        }

        //Correccion lineal de la medicion
        valMed.mag = (valMed.mag - MED_OFFSET) / MED_SCALING;

        if (VcoordNS != 0) {
            //Viento desde NorOeste: 0 < deg < 90
            if (VcoordOE > 0 && VcoordNS > 0) {
                valMed.deg = (180 / 3.14159) * atanf(VcoordOE / VcoordNS);
                if (valMed.deg < 20) {
                    sprintf(valMed.coord, " N ");
                } else if (valMed.deg > 70) {
                    sprintf(valMed.coord, " O ");
                } else {
                    sprintf(valMed.coord, "N-O");
                }
            }
            //Viento desde Sur: 90 < deg < 270
            if (VcoordNS < 0) {
                valMed.deg = (180 / 3.14159) * atanf(VcoordOE / VcoordNS) + 180;
                if (valMed.deg < 110) {
                    sprintf(valMed.coord, " O ");
                } else if (valMed.deg > 160 && valMed.deg < 200) {
                    sprintf(valMed.coord, " S ");
                } else if (valMed.deg > 250) {
                    sprintf(valMed.coord, " E ");
                } else {
                    if (valMed.deg < 180) {
                        sprintf(valMed.coord, "S-O");
                    } else {
                        sprintf(valMed.coord, "S-E");
                    }
                }
            }
            //Viento desde NorEste: 270 < deg < 360
            if (VcoordOE < 0 && VcoordNS > 0) {
                valMed.deg = (180 / 3.14159) * atanf(VcoordOE / VcoordNS) + 360;
                if (valMed.deg > 340) {
                    sprintf(valMed.coord, " N ");
                } else if (valMed.deg < 290) {
                    sprintf(valMed.coord, " E ");
                } else {
                    sprintf(valMed.coord, "N-E");
                }
            }
        } else if (VcoordOE > 0) {
            //Viento desde Oeste
            valMed.deg = 90;
            sprintf(valMed.coord, " O ");
        } else {
            //Viento desde Este
            valMed.deg = 270;
            sprintf(valMed.coord, " E ");
        }
        //Correccion lineal del angulo
        if (valMed.deg >= ANGLE_OFFSET) {
            valMed.deg = (valMed.deg - ANGLE_OFFSET) / ANGLE_SCALING;
        }
    } else {
        //Error lectura
        valMed.mag = 999.99;
        valMed.deg = 999.99;
        return valMed;
    }

    if (valMed.deg == 180) {
        sprintf(valMed.coord, " S ");
    } else if (valMed.deg == 0) {
        sprintf(valMed.coord, " N ");
    }

    return valMed;
}

void anemometroTdetectedFromISR(BaseType_t * pxHigherPriorityTaskWoken, uint32_t val) {
    xTaskNotifyFromISR(xTaskToNotify, val, eSetBits, pxHigherPriorityTaskWoken);
}

void anemometroEmiterSelect(mux_transSelect_enum transd) {

    anemometroMuxOutputSelect(transd);

    //    adc_transdSelect(transd);
}

void anemometroSendFloat(float* dat) {
    xQueueSend(qRecf, dat, portMAX_DELAY);
}

void anemometroRecvFloat(float* dat) {
    xQueueReceive(qRecf, dat, portMAX_DELAY);
}

void anemometroReceptorDelay(mux_transSelect_enum emisor) {
    switch (emisor) {
        case TRANS_EMISOR_OESTE:
            DELAY_O;
            break;
        case TRANS_EMISOR_ESTE:
            DELAY_E;
            break;
        case TRANS_EMISOR_NORTE:
            DELAY_N;
            break;
        case TRANS_EMISOR_SUR:
            DELAY_S;

            break;
        default: DELAY_N;
    }
}

/*Configura los pines de salida del PORTA: RA1 = A y RA0 = B*/
BaseType_t anemometroMuxOutputSelect(mux_transSelect_enum ch) {
    switch (ch) {
        case TRANS_EMISOR_OESTE:
            MUX_INPUT_A(0);
            DELAY_50uS;
            //            vTaskDelay(20 / portTICK_PERIOD_MS);
            MUX_INPUT_B(0);
            break;
        case TRANS_EMISOR_ESTE:
            MUX_INPUT_A(1);
            DELAY_50uS;
            //            vTaskDelay(20 / portTICK_PERIOD_MS);
            MUX_INPUT_B(0);
            break;
        case TRANS_EMISOR_NORTE:
            MUX_INPUT_A(0);
            DELAY_50uS;
            //            vTaskDelay(20 / portTICK_PERIOD_MS);
            MUX_INPUT_B(1);
            break;
        case TRANS_EMISOR_SUR:
            MUX_INPUT_A(1);
            DELAY_50uS;
            //            vTaskDelay(20 / portTICK_PERIOD_MS);
            MUX_INPUT_B(1);
            break;
        default:
            MUX_INPUT_A(0);
            DELAY_50uS;
            //            vTaskDelay(20 / portTICK_PERIOD_MS);
            MUX_INPUT_B(0);
            break;
    }
    DELAY_50uS;
    //    vTaskDelay(20 / portTICK_PERIOD_MS);

    return pdTRUE;
}

float anemometroCalcMode(float * pData, uint16_t nData) {
    float c [N_TIMER_MODE];
    uint8_t i, j;

    //Inicializo matriz auxiliar
    for (i = 0; i < nData; i++) {
        c [ i ] = 0;
    }
    /* Comparo cada elemento de los datos
     * si uno se repite cuento en el mismo indice de la matriz auxiliar
     * y borro de los datos el segundo de los elementos repetidos
     */
    for (i = 0; i < nData; i++) {
        for (j = 0; j < nData; j++) {
            if ((*(pData + i) == *(pData + j)) && (i != j) && (*(pData + i) != 0)) {
                c [ i ] += 1;
                *(pData + j) = 0;
            }
        }
    }

    i = 0;

    /* Recorro la matriz auxiliar
     * el indice con el numero mas alto sera el indice de la moda (i)
     */
    for (j = 0; j < nData; j++) {
        if (c[i] > c[j] && (i != j)) {
            c[j] = 0;
        } else if (i != j) {

            c[i] = 0;
            i = j;
        }
    }

    return *(pData + i);
}

void anemometroCalibCero(float Sspeed) {
    float ceroMed[N_TIMER_MODE];
    uint8_t i = 0, j = 0;
    float ceroMode[4];

    for (j = 0; j < 4; j++) {
        for (i = 0; i < N_TIMER_MODE; i++) {

            ceroMed[i] = anemometroGetCoordTime((mux_transSelect_enum) j);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        //Mido el tiempo de pulso sin viento para cada coordenada
        ceroMode[j] = anemometroCalcMode(ceroMed, N_TIMER_MODE);
    }
    detect_delta_O = ceroMode[0] - DISTANCE_EO / Sspeed;
    detect_delta_E = ceroMode[1] - DISTANCE_EO / Sspeed;
    detect_delta_N = ceroMode[2] - DISTANCE_NS / Sspeed;
    detect_delta_S = ceroMode[3] - DISTANCE_NS / Sspeed;
}

void anemometroAbortMed(void) {
    if (medProgFlag == 0) {
        anemometroModoActivo = Menu;
        xTaskAbortDelay(xTaskToNotify);
    }
}

void anemometroDelayTest(void) {
    char msg[30];

    anemometroGetCoordTime(TRANS_EMISOR_OESTE);
    if (dma_ceroAligned(TRANS_EMISOR_OESTE) == pdPASS) {
        sprintf(msg, "\r\n Delay emisor OESTE: OK%c", '\0');
        uartSend((uint8_t *) msg, sizeof (msg), portMAX_DELAY);
    } else {
        sprintf(msg, "\r\n Delay emisor OESTE: error%c", '\0');
        uartSend((uint8_t *) msg, sizeof (msg), portMAX_DELAY);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);

    anemometroGetCoordTime(TRANS_EMISOR_ESTE);
    if (dma_ceroAligned(TRANS_EMISOR_ESTE) == pdPASS) {
        sprintf(msg, "  ESTE: OK%c", '\0');
        uartSend((uint8_t *) msg, sizeof (msg), portMAX_DELAY);
    } else {
        sprintf(msg, "  ESTE: error%c", '\0');
        uartSend((uint8_t *) msg, sizeof (msg), portMAX_DELAY);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);

    anemometroGetCoordTime(TRANS_EMISOR_NORTE);
    if (dma_ceroAligned(TRANS_EMISOR_NORTE) == pdPASS) {
        sprintf(msg, "  NORTE: OK%c", '\0');
        uartSend((uint8_t *) msg, sizeof (msg), portMAX_DELAY);
    } else {
        sprintf(msg, "  NORTE: error%c", '\0');
        uartSend((uint8_t *) msg, sizeof (msg), portMAX_DELAY);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);

    anemometroGetCoordTime(TRANS_EMISOR_SUR);
    if (dma_ceroAligned(TRANS_EMISOR_SUR) == pdPASS) {
        sprintf(msg, "  SUR: OK\r\n%c", '\0');
        uartSend((uint8_t *) msg, sizeof (msg), portMAX_DELAY);
    } else {
        sprintf(msg, "  SUR: error\r\n%c", '\0');
        uartSend((uint8_t *) msg, sizeof (msg), portMAX_DELAY);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

static void prvSetupHardware(void) {
#ifdef _PLLACTIVATED_
    // Configure Oscillator to operate the device at 40Mhz
    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2 = 40MHz
    // Fosc= 20M*32/(4*2)=80Mhz for 20M input clock
    PLLFBD = 30; // M=32
    CLKDIVbits.PLLPRE = 2; // N1=4
    CLKDIVbits.PLLPOST0 = 0; // N2=2
    CLKDIVbits.PLLPOST1 = 0;

    // Clock Switch to incorporate PLL
    __builtin_write_OSCCONH(0x03); // Initiate Clock Switch to 
    // FRC with PLL (NOSC=0b001)
    __builtin_write_OSCCONL(OSCCON | 0x01); // Start clock switching
    while (OSCCONbits.COSC != 0b011); // Wait for Clock switch to occur	

    // Wait for PLL to lock
    while (OSCCONbits.LOCK != 1) {
    };
#endif
    //Desactivo watchdog

    RCONbits.SWDTEN = 0;

    /* set LED0 pins as outputs */
    TRISAbits.TRISA4 = 0;
    DELAY_50uS;
    //Set mux control pins as outputs
    TRISAbits.TRISA0 = 0;
    DELAY_50uS;
    TRISAbits.TRISA1 = 0;
    DELAY_50uS;
    TRISBbits.TRISB4 = 0;
    DELAY_50uS;
    TRISBbits.TRISB9 = 0;
    DELAY_50uS;
    //Apago el LED
    //    PORTAbits.RA4 = 0;
}

void vApplicationIdleHook(void) {

}