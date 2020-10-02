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
#include <Math.h>

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

float anemometroGetVcoord(mux_transSelect_enum coord);

float anemometroGetMedProm(mux_transSelect_enum coord, uint8_t Nprom);

float anemometroGetCoordTime(mux_transSelect_enum coord);

void anemometroReceptorDelay(mux_transSelect_enum emisor);

float anemometroCalcMode(float * pData, uint16_t nData);

void anemometroCalibCero(void);

void anemometroAbortMed(void);
/*--------Tasks declaration---------*/
//static void led_test_task(void *pvParameters);
//static void transductor_test_task(void *pvParameters);

static void anemometro_main_task(void *pvParameters);

/*FreeRTOS declarations*/
static TaskHandle_t xTaskToNotify = NULL;

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

    //FreeRTOS inits

    if (xTaskCreate(anemometro_main_task,
            "anemometro_main_task",
            configMINIMAL_STACK_SIZE * 3,
            NULL,
            tskIDLE_PRIORITY + 3,
            &xTaskToNotify) != pdPASS) {
        while (1);
    }

    //        if (xTaskCreate(led_test_task, "led_test_task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL) != pdPASS) {
    //            while (1);
    //        }

    //    if (xTaskCreate(transductor_test_task, "transductor_test_task", configMINIMAL_STACK_SIZE * 3, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
    //        while (1);
    //    }

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
    //    wind_medicion_type lineMed = {9999.99, 999.99};

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
                
                if (emisorSelect > TRANS_EMISOR_SUR) emisorSelect = TRANS_EMISOR_OESTE;

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

                //                simpleMed = anemometroGetMed();


                simpleMed.mag = anemometroGetVcoord(emisorSelect);
                //                simpleMed.mag = anemometroGetCoordTime(emisorSelect) * 1000000;

                uartSendMed(simpleMed);

                auxV++;
                if (auxV >= 10) {
                    auxV = 0;
                    //                    emisorSelect++;
                    emisorSelect += 2;
                    //                    //                    uartSendMed(lineMed);
                    //                    if (emisorSelect > TRANS_EMISOR_SUR) {
                    //                        emisorSelect = TRANS_EMISOR_OESTE;
                    //                        //                        //                        uartSendMed(lineMed);
                    //                        //                        vTaskDelay(2 / portTICK_PERIOD_MS);
                    uartEndMode();
                    //                        vTaskDelay(10 / portTICK_PERIOD_MS);
                    //                        //                        //                        while (1);
                    //                }
                }
                anemometroModoActivo = uartGetMode();

                medProgFlag = 0;

                if (anemometroModoActivo == Medicion_Continua) {
                    //                    vTaskDelayUntil(&xLastWakeTime, (MED_PERIOD * 1000) / portTICK_PERIOD_MS);
                }
                break;
            case Configuracion:
                configOption = uartGetModeConfig();

                switch (configOption) {
                    case CalCero:
                        anemometroCalibCero();

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
                        if (configOption == 1) MED_PERIOD = 5;
                        if (configOption == 2) MED_PERIOD = 10;
                        if (configOption == 3) MED_PERIOD = 30;
                        if (configOption == 4) MED_PERIOD = 60;
                        if (configOption == 5) MED_PERIOD = 600;
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

/*CADA 5 SEGUNDOS ENVIARÁ UN TREN DE PULSOS POR UNO DE LOS CANALES*/
//static void transductor_test_task(void *pvParameters) {
//
//    while (1) {
//
//        vTaskDelay(10 / portTICK_PERIOD_MS);
//
//        LED_ON;
//        //CANAL 0
//        muxOutputSelect(TRANS_EMISOR_OESTE);
//
//        comparadorPulseTrain_bloq(TRAIN_PULSE_LENGTH);
//
//        vTaskDelay(5000 / portTICK_PERIOD_MS);
//        //CANAL 1
//        LED_OFF;
//
//        muxOutputSelect(TRANS_EMISOR_ESTE);
//
//        comparadorPulseTrain_bloq(TRAIN_PULSE_LENGTH);
//
//        vTaskDelay(5000 / portTICK_PERIOD_MS);
//        //CANAL 2
//        LED_ON;
//
//        muxOutputSelect(TRANS_EMISOR_NORTE);
//
//        comparadorPulseTrain_bloq(TRAIN_PULSE_LENGTH);
//
//        vTaskDelay(5000 / portTICK_PERIOD_MS);
//        //CANAL 3
//        LED_OFF;
//
//        muxOutputSelect(TRANS_EMISOR_SUR);
//
//        comparadorPulseTrain_bloq(TRAIN_PULSE_LENGTH);
//
//        vTaskDelay(5000 / portTICK_PERIOD_MS);
//    }
//}

//static void led_test_task(void *pvParameters) {
//    uint8_t flag = 0;
//    char str[] = "TEST\r\n";
//    char strRev[10];
//    uint16_t mediciones[1500];
//    uint32_t index = 0;
//
//    while (1) {
//        muxOutputSelect(TRANS_EMISOR_NORTE);
//        vTaskDelay(5 / portTICK_PERIOD_MS);
//        //Prendo 2do mux
//        MUX_INPUT_INH(0);
//        vTaskDelay(5 / portTICK_PERIOD_MS);
//        //adc_start();
//
//        if (flag) {
//            //Apaga led
//            LED_OFF;
//            //Apaga pwm
//            //            MUX_INPUT_INH(0);
//            vTaskDelay(5 / portTICK_PERIOD_MS);
//            //            filtroEnable();
//            //            uartRecv((uint8_t *) strRev, 5, portMAX_DELAY);
//            //comparadorStop();
//            flag = 0;
//        } else {
//            //Prende Led
//            LED_ON;
//            vTaskDelay(5 / portTICK_PERIOD_MS);
//            //            MUX_INPUT_INH(1);
//            //            filtroDisable();
//            //            vTaskDelay(5 / portTICK_PERIOD_MS);
//            //UART
//            //            uartSendMenu(menuTemplate);
//            //            uartSend((uint8_t *) str, 6, portMAX_DELAY);
//            //Arranca pwm
//            //comparadorStart();
//            //            comparadorPulseTrainRTOS(TRAIN_PULSE_LENGTH);
//            //ADC
//            //            adc_start();
//            //            simpleMed = anemometroGetMed();
//            //            DELAY_50uS;
//            //            vTaskDelay(1 / portTICK_PERIOD_MS);
//            //            adc_stop();
//            flag = 1;
//        }
//        vTaskDelay(5000 / portTICK_PERIOD_MS);
//    }
//}

float anemometroGetVcoord(mux_transSelect_enum coord) {
    float valMed = 0;
    float timeMed[] = {0, 0};
    float timeNmediciones[N_TIMER_MODE];
    uint8_t i = 0;
    uint8_t n = 0;
    float timeCorr[2];

    for (i = 0; i < 2; i++) {
        for (n = 0; n < N_TIMER_MODE; n++) {

            timeNmediciones[n] = anemometroGetCoordTime(coord + i);

            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        timeMed[i] = anemometroCalcMode(timeNmediciones, N_TIMER_MODE);
    }

    if (timeMed[0] == 0 || timeMed[1] == 0) {
        valMed = 666.66;
    } else {
        //Calculo 
        switch (coord) {
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
//float anemometroGetVcoord(mux_transSelect_enum coord) {
//    float valMed = 0;
//    float valAcum = 0;
//    float valProm = 0;
//    uint32_t timeMed[] = {0, 0};
//    uint32_t ulNotificationValue;
//    uint8_t i = 0;
//    uint8_t n = 0;
//    uint8_t nMedProm = N_MED_PROM;
//    float timeConv[2];
//    BaseType_t notifyStatus = pdFAIL;
//
//    for (n = 0; n < N_MED_PROM; n++) {
//        for (i = 0; i < 2; i++) {
//            anemometroEmiterSelect(coord + i);
//
//            DELAY_50uS;
//
//            timerStart();
//
//            comparadorPulseTrain_NObloq(TRAIN_PULSE_LENGTH);
//
//            //Necesitaria esperar 400us
//            DELAY_400uS;
//            DELAY_400uS;
//            anemometroReceptorDelay(coord + i);
//
//            adc_start();
//
//            //        ulNotificationValue = ulTaskNotifyTake(pdTRUE, 1 / portTICK_PERIOD_MS);
//            notifyStatus = xTaskNotifyWait(0, UINT32_MAX, &ulNotificationValue, 1 / portTICK_PERIOD_MS);
//
//            if ((ulNotificationValue & 0x1) != 0 && notifyStatus == pdPASS) {
//                timeMed[i] = timerCount();
//            } else {
//                timeMed[i] = 0;
//            }
//            adc_stop();
//
//            timerStop();
//        }
//
//        if (timeMed[0] == 0 || timeMed[1] == 0) {
//            valMed = 0;
//            nMedProm--;
//        } else {
//            //    Convierto tiempos a us
//            timeConv[0] = timerCount2s(timeMed[0]); // N u O
//            timeConv[1] = timerCount2s(timeMed[1]); // S o E
//
//            //Calculo 
//            switch (coord) {
//                case TRANS_EMISOR_OESTE:
//                    //Corrijo los tiempos medidos
//                    timeConv[0] = timeConv[0] - DETECTION_ERROR_O; //taOE
//                    timeConv[1] = timeConv[1] - DETECTION_ERROR_E; //trOE
//
//                    /*La definicion de las coordenadas es tal que si el viento sopla 
//                     * DESDE el NORTE el angulo es 0°
//                     *si sopla DESDE el OESTE el angulo es 90°
//                     * El angulo va de 0° a 360°
//                     */
//                    valMed = (((float) DISTANCE_EO / 2) * (1 / timeConv[0] - 1 / timeConv[1])) - OFFSET_ERROR_EO;
//                    break;
//                case TRANS_EMISOR_NORTE:
//                    //Corrijo los tiempos medidos
//                    timeConv[0] = timeConv[0] - DETECTION_ERROR_N;
//                    timeConv[1] = timeConv[1] - DETECTION_ERROR_S;
//
//                    valMed = (((float) DISTANCE_NS / 2) * (1 / timeConv[0] - 1 / timeConv[1])) - OFFSET_ERROR_NS;
//                    break;
//                default:
//                    nMedProm--;
//                    valMed = 0;
//            }
//            valAcum += valMed;
//        }
//        vTaskDelay(1 / portTICK_PERIOD_MS);
//
//    }
//    if (nMedProm > 0) {
//        valProm = valAcum / nMedProm;
//    } else {
//        valProm = 999.99;
//    }
//
//    return valProm;
//    //    return valMed;
//}

float anemometroGetMedProm(mux_transSelect_enum coord, uint8_t prom) {
    float medAcum = 0;
    uint8_t medNprom = prom;
    uint8_t i = 0;
    float medCoord = 0;

    for (i = 0; i < prom; i++) {
        medCoord = anemometroGetVcoord(coord);
        if (medCoord < 100) {
            medAcum += medCoord;
        } else {
            medNprom--;
        }
    }

    if (medNprom > (prom / 2)) {
        return medAcum / medNprom;
    } else {
        return 666.66;
    }
}

float anemometroGetCoordTime(mux_transSelect_enum coord) {
    float timeMed = 999.99;
    uint32_t ulNotificationValue;
    BaseType_t notifyStatus = pdFAIL;

    RB_9_SET(0);

    if (muxOutputSelect(coord) != pdTRUE) {
        return timeMed;
    }

    DELAY_50uS;

    //    RB_9_SET(1);

    timerStart();

    comparadorPulseTrain_NObloq(TRAIN_PULSE_LENGTH);

    //Necesitaria esperar 400us
    DELAY_400uS;
    //    DELAY_T;
    anemometroReceptorDelay(coord);

    timerStop();

    adc_start();

    //    RB_9_SET(0);

    notifyStatus = xTaskNotifyWait(0, UINT32_MAX, &ulNotificationValue, 2 / portTICK_PERIOD_MS);

    adc_stop();

    //    RB_9_SET(0);

    if ((ulNotificationValue & 0x01) != 0 && notifyStatus == pdPASS) {
        /*Detecto Segundo maximo*/
        timeMed = dma_detectPulse();
    } else {
        timeMed = 888.88;
    }

    switch (coord) {
        case TRANS_EMISOR_OESTE:
            timeMed += (((float) timerCount() / configCPU_CLOCK_HZ));
            break;
        case TRANS_EMISOR_ESTE:
            timeMed += (((float) timerCount() / configCPU_CLOCK_HZ));
            break;
        case TRANS_EMISOR_NORTE:
            timeMed += (((float) timerCount() / configCPU_CLOCK_HZ));
            break;
        case TRANS_EMISOR_SUR:
            timeMed += (((float) timerCount() / configCPU_CLOCK_HZ));

            break;
        default: timeMed = 777.77;
    }

    //    RB_9_SET(0);

    return timeMed;
}

wind_medicion_type anemometroGetMed(void) {
    wind_medicion_type valMed = {0, 0};
    float VcoordOE = 0;
    float VcoordNS = 0;

    //Mido en la coordenada O-E
    VcoordOE = anemometroGetMedProm(TRANS_EMISOR_OESTE, N_MED_PROM);
    //Mido en la coordenada N-S
    VcoordNS = anemometroGetMedProm(TRANS_EMISOR_NORTE, N_MED_PROM);

    //Calculo 
    //    valMed.mag = sqrtf(powf(VcoordOE, 2) + powf(VcoordNS, 2
    if (VcoordOE < 555 && VcoordNS < 555) {
        valMed.mag = sqrtf(VcoordOE * VcoordOE + VcoordNS * VcoordNS);

        if (valMed.mag == 0) {
            valMed.deg = 0;
            return valMed;
        }

        if (VcoordNS != 0) {
            if (VcoordOE > 0 && VcoordNS > 0) {
                valMed.deg = (180 / 3.14159) * atanf(VcoordOE / VcoordNS);
            }
            if (VcoordNS < 0) {
                valMed.deg = (180 / 3.14159) * atanf(VcoordOE / VcoordNS) + 180;
            }
            if (VcoordOE < 0 && VcoordNS > 0) {
                valMed.deg = (180 / 3.14159) * atanf(VcoordOE / VcoordNS) + 360;
            }
        } else if (VcoordOE > 0) {
            valMed.deg = 90;
        } else {
            valMed.deg = 270;
        }
    } else {
        valMed.mag = 999.99;
        valMed.deg = 999.99;
        return valMed;
    }

    return valMed;
}

//void anemometroTdetect(BaseType_t * pxHigherPriorityTaskWoken) {
//    xSemaphoreGiveFromISR(xSemaphoreTrenDetectado, pxHigherPriorityTaskWoken);
//}

void anemometroTdetected(BaseType_t * pxHigherPriorityTaskWoken, uint32_t val) {
    //    vTaskNotifyGiveFromISR(xTaskToNotify, pxHigherPriorityTaskWoken);

    xTaskNotifyFromISR(xTaskToNotify, val, eSetBits, pxHigherPriorityTaskWoken);
}

void anemometroEmiterSelect(mux_transSelect_enum transd) {

    muxOutputSelect(transd);

    //    adc_transdSelect(transd);
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
BaseType_t muxOutputSelect(mux_transSelect_enum ch) {
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
    //    float mode[N_TIMER_MODE];
    float c [N_TIMER_MODE];
    uint8_t i, j;

    for (i = 0; i < nData; i++) {
        c [ i ] = 0;
    }

    for (i = 0; i < nData; i++) {
        for (j = 0; j < nData; j++) {
            if ((*(pData + i) == *(pData + j)) && (i != j) && (*(pData + i) != 0)) {
                c [ i ] += 1;
                *(pData + j) = 0;
            }
        }
    }

    i = 0;

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

void anemometroCalibCero(void) {
    float ceroMed[30];
    uint8_t i = 0, j = 0;
    float ceroMode[4];

    for (j = 0; j < 4; j++) {
        for (i = 0; i < 30; i++) {

            ceroMed[i] = anemometroGetCoordTime((mux_transSelect_enum) j);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        ceroMode[j] = anemometroCalcMode(ceroMed, 30);
    }
    detect_delta_O = ceroMode[0] - DISTANCE_EO / SOUND_SPEED;
    detect_delta_E = ceroMode[1] - DISTANCE_EO / SOUND_SPEED;
    detect_delta_N = ceroMode[2] - DISTANCE_NS / SOUND_SPEED;
    detect_delta_S = ceroMode[3] - DISTANCE_NS / SOUND_SPEED;
}

void anemometroAbortMed(void) {
    if (medProgFlag == 0) {
        anemometroModoActivo = Menu;
        xTaskAbortDelay(xTaskToNotify);
    }
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