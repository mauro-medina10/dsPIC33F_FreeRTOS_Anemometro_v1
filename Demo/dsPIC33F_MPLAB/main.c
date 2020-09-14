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

float anemometroGetMedProm(uint8_t Nprom);

wind_medicion_type anemometroTestTransd(mux_transSelect_enum coord);

void anemometroReceptorDelay(mux_transSelect_enum emisor);

/*--------Tasks declaration---------*/
//static void led_test_task(void *pvParameters);
//static void transductor_test_task(void *pvParameters);

static void anemometro_main_task(void *pvParameters);

/*FreeRTOS declarations*/
static TaskHandle_t xTaskToNotify = NULL;

/*Global variables*/
//float velMed[15];
//uint8_t indMed = 0;
//uint32_t timeMedVal[50];
//uint16_t indiceAux = 0;

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

    if (xTaskCreate(anemometro_main_task, "anemometro_main_task", configMINIMAL_STACK_SIZE * 3, NULL, tskIDLE_PRIORITY + 3, &xTaskToNotify) != pdPASS) {
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
    anemometro_mode_enum anemometroModoActivo = Menu;
    wind_medicion_type simpleMed = {0, 0};
    mux_transSelect_enum emisorSelect = TRANS_EMISOR_OESTE;
    uint16_t auxV = 0;
    float medCoord = 0;
    wind_medicion_type lineMed = {999.99, 999.99};
    float medAcum = 0;
    uint8_t i = 0;
    uint8_t medNprom = N_TIMER_PROM;

    LED_ON;
    vTaskDelay(5 / portTICK_PERIOD_MS);

    //Desactivo MUX
    MUX_INPUT_INH(1);

    while (1) {
        switch (anemometroModoActivo) {
            case Menu:
                auxV = 0;
                medAcum = 0;
                anemometroModoActivo = uartGetMode();

                if (anemometroModoActivo == Medicion_Continua) emisorSelect = TRANS_EMISOR_OESTE;

                break;
            case Medicion_Simple:
                //                simpleMed = anemometroGetMed();
                //                simpleMed.mag = anemometroGetMedProm(N_MED_PROM);
                //                medCoord = anemometroGetVcoord(TRANS_EMISOR_NORTE);
                //                simpleMed.mag = medCoord;
                simpleMed = anemometroTestTransd(TRANS_EMISOR_NORTE);
                //                simpleMed = anemometroTestTransd(emisorSelect);
                emisorSelect++;
                //                simpleMed = anemometroGetVcoord(emisorSelect);

                //                emisorSelect += 2;

                if (emisorSelect > TRANS_EMISOR_SUR) emisorSelect = TRANS_EMISOR_OESTE;

                uartSendMed(simpleMed);

                anemometroModoActivo = Menu;
                break;
            case Medicion_Continua:
                vTaskDelay(10 / portTICK_PERIOD_MS);
                //                anemometroEmiterSelect(emisorSelect);
                //                simpleMed = anemometroGetMed();
                //                simpleMed = anemometroTestTransd(TRANS_EMISOR_SUR);
                //                simpleMed = anemometroTestTransd(emisorSelect);
                //                medCoord = anemometroGetVcoord(TRANS_EMISOR_NORTE);
                medCoord = anemometroGetVcoord(emisorSelect);

                simpleMed.mag = medCoord;

                emisorSelect += 2;
                if (emisorSelect > TRANS_EMISOR_SUR) emisorSelect = TRANS_EMISOR_OESTE;

                //                simpleMed.mag = anemometroGetMedProm(N_MED_PROM);

                uartSendMed(simpleMed);

                //                auxV++;
                //                if (auxV == 50) {
                //                    auxV = 0;
                //                    //                    emisorSelect++;
                //                    emisorSelect += 2;
                //                    //                    uartSendMed(lineMed);
                //                    if (emisorSelect > TRANS_EMISOR_SUR) {
                //                        emisorSelect = TRANS_EMISOR_OESTE;
                //                        uartSendMed(lineMed);
                //                        //                        uartEndMode();
                //                        //                        while (1);
                //                    }
                //                }
                anemometroModoActivo = uartGetMode();
                if (anemometroModoActivo == Menu) {
                    emisorSelect = TRANS_EMISOR_OESTE;
                    auxV = 0;
                }
                break;
            case Configuracion:
                /*TODO*/
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
    //Apago el LED
    //    PORTAbits.RA4 = 0;
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

float anemometroGetVcoord(mux_transSelect_enum coord) {
    float valMed = 0;
    float valAcum = 0;
    float valProm = 0;
    uint32_t timeMed[] = {0, 0};
    uint32_t ulNotificationValue;
    uint8_t i = 0;
    uint8_t n = 0;
    uint8_t m = 0;
    uint8_t nTimerProm[2] = {N_TIMER_PROM, N_TIMER_PROM};
    uint8_t nMedProm = N_MED_PROM;
    float timeConv[2];
    BaseType_t notifyStatus = pdFAIL;

    for (m = 0; m < N_MED_PROM; m++) {
        for (n = 0; n < N_TIMER_PROM; n++) {
            for (i = 0; i < 2; i++) {
                anemometroEmiterSelect(coord + i);

                DELAY_50uS;

                timerStart();

                comparadorPulseTrain_NObloq(TRAIN_PULSE_LENGTH);

                //Necesitaria esperar 400us
                DELAY_400uS;
                anemometroReceptorDelay(coord + i);

                adc_start();

                //        ulNotificationValue = ulTaskNotifyTake(pdTRUE, 1 / portTICK_PERIOD_MS);
                notifyStatus = xTaskNotifyWait(0, UINT32_MAX, &ulNotificationValue, 1 / portTICK_PERIOD_MS);

                if ((ulNotificationValue & 0x1) != 0 && notifyStatus == pdPASS) {
                    timeMed[i] += timerCount();
                } else {
                    nTimerProm[i]--;
                }
                adc_stop();

                timerStop();
            }
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }

        if (timeMed[0] == 0 || timeMed[1] == 0 || nTimerProm[0] == 0 || nTimerProm[1] == 0) {
            valMed = 999.99;
        } else {
            timeMed[0] = timeMed[0] / nTimerProm[0];
            timeMed[1] = timeMed[1] / nTimerProm[1];

            //    Convierto tiempos a us
            timeConv[0] = timerCount2s(timeMed[0]); // N u O
            timeConv[1] = timerCount2s(timeMed[1]); // S o E

            //Calculo 
            switch (coord) {
                case TRANS_EMISOR_OESTE:
                    //Corrijo los tiempos medidos
                    timeConv[0] = timeConv[0] - DETECTION_ERROR_O; //taOE
                    timeConv[1] = timeConv[1] - DETECTION_ERROR_E; //trOE

                    /*La definicion de las coordenadas es tal que si el viento sopla 
                     * DESDE el NORTE el angulo es 0°
                     *si sopla DESDE el OESTE el angulo es 90°
                     * El angulo va de 0° a 360°
                     */
                    valMed = (((float) DISTANCE_EO / 2) * (1 / timeConv[0] - 1 / timeConv[1])) - OFFSET_ERROR_EO;
                    break;
                case TRANS_EMISOR_NORTE:
                    //Corrijo los tiempos medidos
                    timeConv[0] = timeConv[0] - DETECTION_ERROR_N;
                    timeConv[1] = timeConv[1] - DETECTION_ERROR_S;

                    valMed = (((float) DISTANCE_NS / 2) * (1 / timeConv[0] - 1 / timeConv[1])) - OFFSET_ERROR_NS;
                    break;
                default:
                    valMed = 888.88;
            }
        }
        if (valMed < 555) {
            valAcum += valMed;
        } else {
            nMedProm--;
        }
        timeMed[0] = 0;
        timeMed[1] = 0;
        nTimerProm[0] = N_TIMER_PROM;
        nTimerProm[1] = N_TIMER_PROM;
    }
    if (nMedProm > 0) {
        valMed = valAcum / nMedProm;
    } else {
        valMed = 666.66;
    }
    return valMed;
    //    return valMed;
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

wind_medicion_type anemometroTestTransd(mux_transSelect_enum coord) {
    wind_medicion_type valMed = {0, 0};
    uint32_t timeMed = 0;
    uint32_t ulNotificationValue;
    BaseType_t notifyStatus = pdFAIL;

    //    anemometroEmiterSelect(coord);
    if (muxOutputSelect(coord) != pdTRUE) {
        return valMed;
    }

    DELAY_50uS;

    timerStart();

    comparadorPulseTrain_NObloq(TRAIN_PULSE_LENGTH);

    //Necesitaria esperar 400us
    DELAY_400uS;
    anemometroReceptorDelay(coord);

    adc_start();

    //    ulNotificationValue = ulTaskNotifyTake(pdTRUE, 2 / portTICK_PERIOD_MS);
    notifyStatus = xTaskNotifyWait(0, UINT32_MAX, &ulNotificationValue, 2 / portTICK_PERIOD_MS);

    if ((ulNotificationValue & 0x01) != 0 && notifyStatus == pdPASS) {
        timeMed = timerCount();
    } else {
        timeMed = 0;
    }

    adc_stop();

    timerStop();

    valMed.mag = (float) timeMed;

    return valMed;
}

wind_medicion_type anemometroGetMed(void) {
    wind_medicion_type valMed = {0, 0};
    float VcoordOE = 0;
    float VcoordNS = 0;

    //Mido en la coordenada O-E
    VcoordOE = anemometroGetVcoord(TRANS_EMISOR_OESTE);
    //Mido en la coordenada N-S
    VcoordNS = anemometroGetVcoord(TRANS_EMISOR_NORTE);

    //Calculo 
    //    valMed.mag = sqrtf(powf(VcoordOE, 2) + powf(VcoordNS, 2
    if (VcoordOE < 555 && VcoordNS < 555) {
        valMed.mag = sqrtf(VcoordOE * VcoordOE + VcoordNS * VcoordNS);

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
            valMed.deg = -90;
        }
    } else {
        valMed.mag = 999.99;
        valMed.deg = 999.99;
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

float anemometroGetMedProm(uint8_t prom) {
    uint32_t medAcum = 0;
    uint8_t medNprom = prom;
    uint8_t i = 0;
    float medCoord = 0;
    wind_medicion_type promMed = {0, 0};

    for (i = 0; i < prom; i++) {
        medCoord = anemometroGetVcoord(TRANS_EMISOR_NORTE);
        if (medCoord < 1000) {
            medAcum += medCoord;
        } else {
            medNprom--;
        }
    }

    if (medNprom > 0) {
        return medAcum / medNprom;
    } else {
        return 777.77;
    }
}

void vApplicationIdleHook(void) {

}