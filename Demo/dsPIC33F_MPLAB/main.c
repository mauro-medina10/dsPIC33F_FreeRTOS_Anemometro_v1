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
#include <dsp.h>

/*Config includes*/
#include "xc.h"
#include "config.h"

/*FreeRTOS includes*/
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "list.h"
#include "queue.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/*Peripherals drivers includes*/
#include "anemometroDef.h"
#include "adc.h"
#include "pwm.h"
#include "UART_RTOS.h"
#include "timer.h"

/*Funciones locales*/
static void prvSetupHardware(void);

/*--------Tasks declaration---------*/
//static void led_test_task(void *pvParameters);
//static void transductor_test_task(void *pvParameters);

static void anemometro_main_task(void *pvParameters);

/*FreeRTOS declarations*/
static TaskHandle_t xMainTaskHandle = NULL;
static QueueHandle_t qRecf;

/*Global variables*/
float detect_delta_O = DETECTION_ERROR_O;
float detect_delta_E = DETECTION_ERROR_E;
float detect_delta_N = DETECTION_ERROR_N;
float detect_delta_S = DETECTION_ERROR_S;

static anemometro_mode_enum anemometroModoActivo = Menu;

uint8_t medProgFlag = 0;
uint8_t medAborted = 0;

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
            &xMainTaskHandle) != pdPASS) {
        while (1);
    }

    LED_OFF;

    vTaskStartScheduler();

    while (1);

    return 0;
}

static void anemometro_main_task(void *pvParameters) {
    wind_medicion_type abortMed = {9995.99, 9995.99};
    anemometro_config_enum configOption = ExitConfig;
    wind_medicion_type simpleMed = {0, 0};
    mux_transSelect_enum emisorSelect = TRANS_EMISOR_OESTE;
    uint16_t auxV = 0;
    TickType_t xLastWakeTime;
    unsigned int firstPeriodFlag = 1;
    float soundSpeed = 345.7;
    float coordVmed[2];
    uint32_t medPeriod = 10;
    BaseType_t medState = pdFAIL;

    MUX_INPUT_INH(0);

    DELAY_50uS;

    LED_ON;

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
                //Mide las componentes de cada coordenada de la velocidad
                medState = anemometroVmedian(&coordVmed[0], &coordVmed[1]);

                //Calcula el modulo y direccion de la velocidad
                if (medState == pdPASS) {
                    simpleMed = anemometroGetMed(coordVmed[0], coordVmed[1]);

                    medState = pdFAIL;
                } else {
                    sprintf(simpleMed.coord, " F ");
                }

                uartSendMed(&simpleMed);

                anemometroModoActivo = Menu;
                break;
            case Medicion_Continua:

                //Mide periodicamente la velocidad y direccion del viento
                if (firstPeriodFlag == 1) {
                    xLastWakeTime = xTaskGetTickCount();
                    firstPeriodFlag = 0;
                }
                //Inicia medicion
                medProgFlag = 1;

                if (anemometroVmedian(&coordVmed[0], &coordVmed[1]) == pdPASS) {

                    simpleMed = anemometroGetMed(coordVmed[0], coordVmed[1]);

                } else {
                    //Error de medicion
                    sprintf(simpleMed.coord, " F ");
                }

                uartSendMed(&simpleMed);

                anemometroModoActivo = uartGetMode();

                //Finaliza medicion
                medProgFlag = 0;

                //Delay definido por configuracion
                if (anemometroModoActivo == Medicion_Continua) {
                    vTaskDelayUntil(&xLastWakeTime, (medPeriod * 1000) / portTICK_PERIOD_MS);
                }

                break;
            case Configuracion:

                configOption = uartGetModeConfig();

                switch (configOption) {
                    case CalCero:
                        /*Calibracion con viento nulo: 
                         *identifica el error en la deteccion 
                         *del tren de pulsos con el tiempo teorico */

                        xQueueReceive(qRecf, &soundSpeed, portMAX_DELAY);

                        if (soundSpeed == 999.99) {
                            //Deja los valores definidos originalmente
                            detect_delta_O = (float) DETECTION_ERROR_O;
                            detect_delta_E = (float) DETECTION_ERROR_E;
                            detect_delta_N = (float) DETECTION_ERROR_N;
                            detect_delta_S = (float) DETECTION_ERROR_S;
                        } else {
                            anemometroCalibCero(soundSpeed);
                        }

                        //Muestra al usuario los valores obtenidos
                        simpleMed.mag = detect_delta_O * 1000000;
                        simpleMed.deg = detect_delta_E * 1000000;
                        uartSendMed(&simpleMed);

                        simpleMed.mag = detect_delta_N * 1000000;
                        simpleMed.deg = detect_delta_S * 1000000;
                        uartSendMed(&simpleMed);
                        simpleMed.mag = 0;
                        simpleMed.deg = 0;

                        configOption = ExitConfig;
                        break;
                    case SetEmi:
                        /*Configura MUX manualmente
                         *configuracion usada en etapa de pruebas/debug
                         *irrelevante ahora*/
                        configOption = uartGetModeConfig();
                        if (configOption == 1) emisorSelect = TRANS_EMISOR_NORTE;
                        if (configOption == 2) emisorSelect = TRANS_EMISOR_SUR;
                        if (configOption == 3) emisorSelect = TRANS_EMISOR_ESTE;
                        if (configOption == 4) emisorSelect = TRANS_EMISOR_OESTE;
                        configOption = ExitConfig;
                        break;
                    case SetPeriod:
                        //Permite elegir el periodo de medicion en el modo automatico
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

/*Calcula velocidad y direccion a partir de las componentes de cada coordenada*/
BaseType_t anemometroVmedian(float* medOE, float* medNS) {
    BaseType_t medState = pdFAIL;
    float timeMedian[4];
    float timeCorr[2];

    //Mide los tiempos en cada coordenada
    medState = anemometroGetTmedian(&timeMedian[0], &timeMedian[1], TRANS_EMISOR_OESTE);

    medState *= anemometroGetTmedian(&timeMedian[2], &timeMedian[3], TRANS_EMISOR_NORTE);

    if (medState == pdFAIL) return pdFAIL;

    //Calculo V para OE
    timeCorr[0] = timeMedian[0] - detect_delta_O; //taOE
    timeCorr[1] = timeMedian[1] - detect_delta_E; //trOE

    if (timeCorr[0] == timeCorr[1]) {
        *medOE = 0;
    } else {
        *medOE = (((float) DISTANCE_EO / 2) * (1 / timeCorr[0] - 1 / timeCorr[1])) - OFFSET_ERROR_EO;
        *medOE = (*medOE - DETECT_OFFSET_OE) / DETECT_SCALING_OE;
    }
    //Calculo V para NS
    timeCorr[0] = timeMedian[2] - detect_delta_N; //taNS
    timeCorr[1] = timeMedian[3] - detect_delta_S; //trNS

    if (timeCorr[0] == timeCorr[1]) {
        *medNS = 0;
    } else {
        *medNS = (((float) DISTANCE_NS / 2) * (1 / timeCorr[0] - 1 / timeCorr[1])) - OFFSET_ERROR_NS;
        *medNS = (*medNS - DETECT_OFFSET_NS) / DETECT_SCALING_NS;
    }

    return medState;
}

/*Calcula la mediana del ToF doble del tren de pulsos en una coordenada*/
BaseType_t anemometroGetTmedian(float* medianA, float* medianB, mux_transSelect_enum coordV) {
    float timeMed[] = {0, 0};
    uint8_t i = 0;
    uint8_t n = 0;
    uint8_t correctMeds = 0;
    float timeNmediciones[N_TIMER_MEDIAN];
    BaseType_t medState = pdPASS;

    for (i = 0; i < 2; i++) {
        for (n = 0; n < N_TIMER_MEDIAN; n++) {

            correctMeds += (uint8_t) anemometroGetCoordTime(&timeNmediciones[n], coordV + i);

            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        //Si menos de la mitad de las mediciones dieron error calcula el tiempo de llegada del tren de pulsos
        if (correctMeds > (N_TIMER_MEDIAN / 2)) {
            medState = anemometroFindMedian(timeNmediciones, &timeMed[i], N_TIMER_MEDIAN);
            correctMeds = 0;
        } else {
            return pdFAIL;
        }
    }

    if (medState == pdPASS) {
        //DEBUG: ajuste ventanas
        //        *medianA = timeMed[0] * 1000000;
        //        *medianB = timeMed[1] * 1000000;
        *medianA = timeMed[0];
        *medianB = timeMed[1];
    }

    return medState;
}

/*Utiliza la moda para determinar el ToF
 *ya no se usa */
BaseType_t anemometroGetTmode(float* modeA, float* modeB, mux_transSelect_enum coordV) {
    float timeMed[] = {0, 0};
    float timeNmediciones[N_TIMER_MODE];
    uint8_t i = 0;
    uint8_t n = 0;
    BaseType_t medState = pdFAIL;

    for (i = 0; i < 2; i++) {
        for (n = 0; n < N_TIMER_MODE; n++) {

            anemometroGetCoordTime(&timeNmediciones[n], coordV + i);

            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        medState = anemometroCalcMode(timeNmediciones, &timeMed[i], N_TIMER_MODE);
    }
    //DEBUG: ajuste ventanas
    //    *modeA = timeMed[0] * 1000000;
    //    *modeB = timeMed[1] * 1000000;

    *modeA = timeMed[0];
    *modeB = timeMed[1];

    return medState;
}

/*NO USADA
 *calcula la velocidad con el metodo que usa la moda*/
float anemometroGetVcoord(mux_transSelect_enum coordV) {
    float valMed = 0;
    float timeMed[] = {0, 0};
    float timeNmediciones[N_TIMER_MODE];
    uint8_t i = 0;
    uint8_t n = 0;
    float timeCorr[2];

    for (i = 0; i < 2; i++) {
        for (n = 0; n < N_TIMER_MODE; n++) {

            anemometroGetCoordTime(&timeNmediciones[n], coordV + i);

            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        anemometroCalcMode(timeNmediciones, &timeMed[i], N_TIMER_MODE);
    }

    if (timeMed[0] == 0 || timeMed[1] == 0 || timeMed[0] > 100 || timeMed[1] > 100) {
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
                 * DESDE el NORTE el angulo es 0�
                 *si sopla DESDE el OESTE el angulo es 90�
                 * El angulo va de 0� a 360�
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

/*NO USADA
 *calcula velocidad a partir de la moda*/
BaseType_t anemometroVmode(float* medOE, float* medNS, uint8_t Nmode) {
    BaseType_t modeState = pdPASS;
    float velNmedicionesR[N_MED_MODE];
    float velNmedicionesT[N_MED_MODE];
    float timeMode[4];
    uint8_t n = 0, i = 0;
    float timeCorr[] = {0, 0};

    for (i = 0; i < 4; i += 2) {
        for (n = 0; n < N_MED_MODE; n++) {

            anemometroGetTmode(&velNmedicionesR[n], &velNmedicionesT[n], TRANS_EMISOR_OESTE + i);

            vTaskDelay(1 / portTICK_PERIOD_MS);

        }
        modeState *= anemometroCalcMode(velNmedicionesR, &timeMode[i], N_MED_MODE);

        modeState *= anemometroCalcMode(velNmedicionesT, &timeMode[1 + i], N_MED_MODE);
    }

    if (modeState == pdFAIL) return modeState;

    //Calculo V para OE
    timeCorr[0] = timeMode[0] - detect_delta_O; //taOE
    timeCorr[1] = timeMode[1] - detect_delta_E; //trOE

    if (timeCorr[0] == timeCorr[1]) {
        *medOE = 0;
    } else {
        *medOE = (((float) DISTANCE_EO / 2) * (1 / timeCorr[0] - 1 / timeCorr[1])) - OFFSET_ERROR_EO;
        *medOE = (*medOE - DETECT_OFFSET_OE) / DETECT_SCALING_OE;
    }
    //Calculo V para NS
    timeCorr[0] = timeMode[2] - detect_delta_N; //taNS
    timeCorr[1] = timeMode[3] - detect_delta_S; //trNS

    if (timeCorr[0] == timeCorr[1]) {
        *medNS = 0;
    } else {
        *medNS = (((float) DISTANCE_NS / 2) * (1 / timeCorr[0] - 1 / timeCorr[1])) - OFFSET_ERROR_NS;
        *medNS = (*medNS - DETECT_OFFSET_NS) / DETECT_SCALING_NS;
    }

    return modeState;
}

 /*NO USADA
  *calcula el promedio de las mediciones en cada coordenada*/
BaseType_t anemometroVprom(float* medOE, float* medNS, uint8_t Nprom) {
    float medAcumOE = 0;
    float medAcumNS = 0;
    uint8_t medNpromOE = Nprom;
    uint8_t medNpromNS = Nprom;
    uint8_t i = 0;
    float medCoordOE = 0;
    float medCoordNS = 0;

    for (i = 0; i < Nprom; i++) {
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

    if (medNpromOE > (Nprom / 2) && medNpromNS > (Nprom / 2)) {
        *medOE = medAcumOE / medNpromOE;
        *medNS = medAcumNS / medNpromNS;

        return pdPASS;
    } else {
        *medOE = 666.66;
        *medNS = 666.66;
    }

    return pdFAIL;
}

/*Mide el ToF del tren de pulsos en una coordenada en particular 
 *con un emisor definido*/
BaseType_t anemometroGetCoordTime(float* timeMed, mux_transSelect_enum coordTime) {
    float timeCalc = 999.99;
    uint32_t ulNotificationValue;
    BaseType_t notifyStatus = pdFAIL;
    BaseType_t pulseDetected = pdFALSE;
    BaseType_t pulseCaptured = pdFALSE;

    if (anemometroMuxOutputSelect(coordTime) != pdTRUE) {
        return pdFAIL;
    }
    //Desactivo MUX 2
    MUX_INPUT_INH(1);

    DELAY_50uS;

    timerStart();

    //Se�al de exitacion del emisor
    comparadorPulseTrain_NObloq(2);
    DELAY_100uS;
    comparadorPulseTrain_NObloq(4);

    DELAY_400uS;
    DELAY_50uS;

    //Activo Mux 2
    MUX_INPUT_INH(0);
    timerStop();

    adc_start();

    pulseCaptured = dma_capturePulse(coordTime);

    if (pulseCaptured == pdPASS) {
        pulseDetected = dma_detectPulse(coordTime, &timeCalc);

        if (pulseDetected == pdPASS) {
            //Calcula el ToF
            timeCalc += (((float) timerCount() / configCPU_CLOCK_HZ));

            *timeMed = timeCalc;

            return pdPASS;
        }
    }

    return pdFAIL;
}

/*Calcula la direccion y velocidad del viento a partir de las componentes*/
wind_medicion_type anemometroGetMed(float VcoordOE, float VcoordNS) {
    wind_medicion_type valMed = {0, 0};

    valMed.mag = sqrtf(VcoordOE * VcoordOE + VcoordNS * VcoordNS);

    if (valMed.mag == 0) {
        valMed.deg = 0;
        sprintf(valMed.coord, "   ");

        return valMed;
    }

    //Correccion lineal de la medicion
    valMed.mag = (valMed.mag - MED_OFFSET) / MED_SCALING;

    if (VcoordNS != 0) {
        //Calculo Angulo
        valMed.deg = (180 / 3.14159265) * atanf(VcoordOE / VcoordNS);

        //Correccion lineal del angulo
        if (fabsf(valMed.deg) >= ANGLE_OFFSET) {
            valMed.deg = (valMed.deg - ANGLE_OFFSET) / ANGLE_SCALING;
        }

        //Viento desde NorOeste: 0 < deg < 90
        if (VcoordOE > 0 && VcoordNS > 0) {
            if (valMed.deg < 20) {
                sprintf(valMed.coord, " N ");
                valMed.mag = valMed.mag / PARALLEL_SCALING;
            } else if (valMed.deg > 70) {
                sprintf(valMed.coord, " O ");
                valMed.mag = valMed.mag / PARALLEL_SCALING;
            } else {
                sprintf(valMed.coord, "N-O");
            }
        }
        //Viento desde Sur: 90 < deg < 270
        if (VcoordNS < 0) {
            valMed.deg += 180;

            if (valMed.deg < 110) {
                sprintf(valMed.coord, " O ");
                valMed.mag = valMed.mag / PARALLEL_SCALING;
            } else if (valMed.deg > 160 && valMed.deg < 200) {
                sprintf(valMed.coord, " S ");
                valMed.mag = valMed.mag / PARALLEL_SCALING;
            } else if (valMed.deg > 250) {
                sprintf(valMed.coord, " E ");
                valMed.mag = valMed.mag / PARALLEL_SCALING;
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
            valMed.deg += 360;

            if (valMed.deg > 340) {
                sprintf(valMed.coord, " N ");
                valMed.mag = valMed.mag / PARALLEL_SCALING;
            } else if (valMed.deg < 290) {
                sprintf(valMed.coord, " E ");
                valMed.mag = valMed.mag / PARALLEL_SCALING;
            } else {
                sprintf(valMed.coord, "N-E");
            }
        }
    } else if (VcoordOE > 0) {
        //Viento desde Oeste
        valMed.deg = 90;
        sprintf(valMed.coord, " O ");
        valMed.mag = valMed.mag / PARALLEL_SCALING;
    } else {
        //Viento desde Este
        valMed.deg = 270;
        sprintf(valMed.coord, " E ");
        valMed.mag = valMed.mag / PARALLEL_SCALING;
    }

    if (valMed.deg == 180) {
        sprintf(valMed.coord, " S ");
        valMed.mag = valMed.mag / PARALLEL_SCALING;
    } else if (valMed.deg == 0) {
        sprintf(valMed.coord, " N ");
        valMed.mag = valMed.mag / PARALLEL_SCALING;
    }

    return valMed;
}

void anemometroTdetectedFromISR(BaseType_t * pxHigherPriorityTaskWoken, uint32_t val) {
    xTaskNotifyFromISR(xMainTaskHandle, val, eSetBits, pxHigherPriorityTaskWoken);
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

BaseType_t anemometroFindMedian(float* pData, float* pMedian, uint16_t nData) {
    uint16_t i, j;
    float aux = 0;
    uint16_t indexMedian = 0;

    for (i = 0; i < nData - 1; i++) {
        for (j = 0; j < nData - i - 1; j++) {
            if (pData[j] > pData[j + 1]) {
                aux = *(pData + j);
                *(pData + j) = *(pData + j + 1);
                *(pData + j + 1) = aux;
            }
        }
    }
    indexMedian = (nData + 1) / 2 - 1;

    if (indexMedian >= 0 && indexMedian <= nData) {
        *pMedian = pData[indexMedian];
        return pdPASS;
    } else {
        return pdFAIL;
    }
}

BaseType_t anemometroCalcMode(float* pData, float* pMode, uint16_t nData) {
    float c[N_TIMER_MODE];
    uint8_t i, j;

    if (nData > N_TIMER_MODE) return pdFAIL;

    //Inicializo matriz auxiliar
    for (i = 0; i < N_TIMER_MODE; i++) {
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

    *pMode = *(pData + i);

    return pdPASS;
}

void anemometroCalibCero(float Sspeed) {
    float ceroMed[N_TIMER_MEDIAN];
    uint8_t i = 0, j = 0;
    float ceroMedian[4];
    BaseType_t calState = pdFAIL;

    for (j = 0; j < 4; j++) {
        for (i = 0; i < N_TIMER_MEDIAN; i++) {

            anemometroGetCoordTime(&ceroMed[i], (mux_transSelect_enum) j);
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        //Mido el tiempo de pulso sin viento para cada coordenada
        calState *= anemometroFindMedian(ceroMed, &ceroMedian[j], N_TIMER_MEDIAN);
    }
    //Se usa la diferencia entre el tiempo medido y el teorico como correccion en todas las mediciones
    if (calState == pdTRUE) {
        detect_delta_O = (float) ceroMedian[0] - DISTANCE_EO / Sspeed;
        detect_delta_E = (float) ceroMedian[1] - DISTANCE_EO / Sspeed;
        detect_delta_N = (float) ceroMedian[2] - DISTANCE_NS / Sspeed;
        detect_delta_S = (float) ceroMedian[3] - DISTANCE_NS / Sspeed;
    }
}

void anemometroAbortMed(void) {
    /*Si no hay medicion en progreso sale del modo automatico*/
    if (medProgFlag == 0) {
        anemometroModoActivo = Menu;
        medAborted = 1;
        xTaskAbortDelay(xMainTaskHandle);
    }
}

void anemometroDelayTest(void) {
    /*FUNCION NO UTILIZBLE CON ESTE METODO DE DeTECCION*/
    char msg[30];
    float medAux = 0;

    anemometroGetCoordTime(&medAux, TRANS_EMISOR_OESTE);
    if (dma_ceroAligned(TRANS_EMISOR_OESTE) == pdPASS) {
        sprintf(msg, "\r\n Delay emisor OESTE: OK%c", '\0');
        uartSend((uint8_t *) msg, sizeof (msg), portMAX_DELAY);
    } else {
        sprintf(msg, "\r\n Delay emisor OESTE: error%c", '\0');
        uartSend((uint8_t *) msg, sizeof (msg), portMAX_DELAY);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);

    anemometroGetCoordTime(&medAux, TRANS_EMISOR_ESTE);
    if (dma_ceroAligned(TRANS_EMISOR_ESTE) == pdPASS) {
        sprintf(msg, "  ESTE: OK%c", '\0');
        uartSend((uint8_t *) msg, sizeof (msg), portMAX_DELAY);
    } else {
        sprintf(msg, "  ESTE: error%c", '\0');
        uartSend((uint8_t *) msg, sizeof (msg), portMAX_DELAY);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);

    anemometroGetCoordTime(&medAux, TRANS_EMISOR_NORTE);
    if (dma_ceroAligned(TRANS_EMISOR_NORTE) == pdPASS) {
        sprintf(msg, "  NORTE: OK%c", '\0');
        uartSend((uint8_t *) msg, sizeof (msg), portMAX_DELAY);
    } else {
        sprintf(msg, "  NORTE: error%c", '\0');
        uartSend((uint8_t *) msg, sizeof (msg), portMAX_DELAY);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);

    anemometroGetCoordTime(&medAux, TRANS_EMISOR_SUR);
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
    DELAY_100uS;
    //Set mux control pins as outputs
    TRISAbits.TRISA0 = 0;
    DELAY_100uS;
    TRISAbits.TRISA1 = 0;
    DELAY_100uS;
    TRISBbits.TRISB4 = 0;
    DELAY_100uS;
    TRISBbits.TRISB9 = 0;
    DELAY_100uS;
    //    RB_8_SET_MODE(1);
    //    DELAY_100uS;
    RB_9_SET_MODE(0);
    DELAY_100uS
}

void vApplicationIdleHook(void) {

}