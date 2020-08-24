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

wind_medicion_type anemometroTestCoord(mux_transSelect_enum coord);

wind_medicion_type anemometroTestTransd(mux_transSelect_enum coord);

/*--------Tasks declaration---------*/
//static void led_test_task(void *pvParameters);
//static void transductor_test_task(void *pvParameters);

static void anemometro_main_task(void *pvParameters);

/*FreeRTOS declarations*/
static TaskHandle_t xTaskToNotify = NULL;

/*Global variables*/
float velMed[15];
uint8_t indMed = 0;
uint32_t timeMedVal[50];
uint16_t indiceAux = 0;

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
    //    xSemaphoreTrenDetectado = xSemaphoreCreateBinary();

    //    if (xSemaphoreTrenDetectado == NULL) while (1);

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

    LED_ON;
    vTaskDelay(5 / portTICK_PERIOD_MS);

    //Desactivo MUX
    MUX_INPUT_INH(1);

    while (1) {
        switch (anemometroModoActivo) {
            case Menu:
                anemometroModoActivo = uartGetMode();
                break;
            case Medicion_Simple:
                //                simpleMed = anemometroGetMed();
                simpleMed = anemometroTestTransd(TRANS_EMISOR_NORTE);

                uartSendMed(simpleMed);

                anemometroModoActivo = Menu;
                break;
            case Medicion_Continua:
                vTaskDelay(10 / portTICK_PERIOD_MS);
                //                anemometroEmiterSelect(emisorSelect);

                //                simpleMed = anemometroGetMed();

                //                simpleMed = anemometroTestTransd(emisorSelect);

                simpleMed = anemometroTestCoord(emisorSelect);

                uartSendMed(simpleMed);

                auxV++;
                if (auxV == 50) {
                    //                    emisorSelect++;
                    auxV = 0;
                    emisorSelect += 2;
                    if (emisorSelect > 3) {
                        while (1);
                    }
                }
                anemometroModoActivo = uartGetMode();
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
void muxOutputSelect(mux_transSelect_enum ch) {
    switch (ch) {
        case TRANS_EMISOR_OESTE:
            MUX_INPUT_A(0);
            vTaskDelay(20 / portTICK_PERIOD_MS);
            MUX_INPUT_B(0);
            break;
        case TRANS_EMISOR_ESTE:
            MUX_INPUT_A(1);
            vTaskDelay(20 / portTICK_PERIOD_MS);
            MUX_INPUT_B(0);
            break;
        case TRANS_EMISOR_NORTE:
            MUX_INPUT_A(0);
            vTaskDelay(20 / portTICK_PERIOD_MS);
            MUX_INPUT_B(1);
            break;
        case TRANS_EMISOR_SUR:
            MUX_INPUT_A(1);
            vTaskDelay(20 / portTICK_PERIOD_MS);
            MUX_INPUT_B(1);
            break;
        default:
            MUX_INPUT_A(0);
            vTaskDelay(20 / portTICK_PERIOD_MS);
            MUX_INPUT_B(0);
            break;
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
}

wind_medicion_type anemometroTestCoord(mux_transSelect_enum coord) {
    wind_medicion_type valMed = {0, 0};
    uint32_t timeMed[2];
    uint32_t ulNotificationValue;
    uint8_t i = 1;
    float timeConv[2];

    for (i = 1; i < 3; i++) {
        anemometroEmiterSelect(coord + (mux_transSelect_enum) i - 1);

        DELAY_50uS;

        timerStart();

        comparadorPulseTrain_NObloq(TRAIN_PULSE_LENGTH);

        //Necesitaria esperar 300us
        DELAY_300uS;

        adc_start();

        ulNotificationValue = ulTaskNotifyTake(pdTRUE, 2 / portTICK_PERIOD_MS);

        if (ulNotificationValue == 1) {
            timeMed[i - 1] = timerCount();
        } else {
            timeMed[i - 1] = 0;
            i--;
        }
        adc_stop();

        timerStop();
    }
    //    Convierto tiempos a us
    timeConv[0] = timerCount2s(timeMed[0]); // N u O
    timeConv[1] = timerCount2s(timeMed[1]); // S o E

    //Calculo 
    switch (coord) {
        case TRANS_EMISOR_OESTE:
            //Corrijo los tiempos medidos
            timeConv[0] = timeConv[0] - DETECTION_ERROR_O; //taOE
            timeConv[1] = timeConv[1] - DETECTION_ERROR_E; //trOE
            valMed.mag = (((float) DISTANCE_EO / 2) * (1 / timeConv[0] - 1 / timeConv[1])); //Agregar offset
            break;
        case TRANS_EMISOR_NORTE:
            //Corrijo los tiempos medidos
            timeConv[0] = timeConv[0] - DETECTION_ERROR_N;
            timeConv[1] = timeConv[1] - DETECTION_ERROR_S;
            valMed.mag = (((float) DISTANCE_NS / 2) * (1 / timeConv[0] - 1 / timeConv[1])); //Agregar offset
            break;
        default: valMed.mag = 0;
    }

    return valMed;
}

wind_medicion_type anemometroTestTransd(mux_transSelect_enum coord) {
    wind_medicion_type valMed = {0, 0};
    uint32_t timeMed = 0;
    uint32_t ulNotificationValue;

    anemometroEmiterSelect(coord);

    DELAY_50uS;

    timerStart();

    comparadorPulseTrain_NObloq(TRAIN_PULSE_LENGTH);

    //Necesitaria esperar 300us
    DELAY_300uS;

    adc_start();

    ulNotificationValue = ulTaskNotifyTake(pdTRUE, 2 / portTICK_PERIOD_MS);

    if (ulNotificationValue == 1) {
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
    uint32_t ulNotificationValue;
    wind_medicion_type valMed = {0, 0};
    uint32_t timeMed[4]; //Guardo las mediciones en orden O-E-N-S
    float timeConv[4];
    mux_transSelect_enum transdSelect = TRANS_EMISOR_NORTE;

    for (transdSelect = 0; transdSelect < 4; transdSelect++) {
        anemometroEmiterSelect(transdSelect);

        DELAY_300uS;

        timerStart();

        comparadorPulseTrain_NObloq(TRAIN_PULSE_LENGTH);

        //Necesitaria esperar 300us
        DELAY_300uS;

        adc_start();

        ulNotificationValue = ulTaskNotifyTake(pdTRUE, 2 / portTICK_PERIOD_MS);

        if (ulNotificationValue == 1) {
            timeMed[transdSelect] = timerCount();
        } else {
            timeMed[transdSelect] = 0;
        }

        adc_stop();

        timerStop();
    }
    //    Convierto tiempos a us
    timeConv[0] = timerCount2s(timeMed[0]); //O
    timeConv[1] = timerCount2s(timeMed[1]); //E
    timeConv[2] = timerCount2s(timeMed[2]); //N
    timeConv[3] = timerCount2s(timeMed[3]); //S
    //Corrijo los tiempos medidos
    timeConv[0] = timeConv[0] - DETECTION_ERROR_O; //taOE
    timeConv[1] = timeConv[1] - DETECTION_ERROR_E; //trOE
    timeConv[2] = timeConv[2] - DETECTION_ERROR_N; //taNS
    timeConv[3] = timeConv[3] - DETECTION_ERROR_S; //trNS
    //Calculo 
    //    valMed.mag = sqrtf(powf((DISTANCE_EO / 2) * (1 / timeConv[0] - 1 / timeConv[1]), 2) + powf((DISTANCE_NS / 2) * (1 / timeConv[2] - 1 / timeMed[3]), 2));

    //    valMed.deg = atanf((DISTANCE_EO / DISTANCE_NS) * ((1 / timeConv[0] - 1 / timeConv[1]) / (1 / timeConv[2] - 1 / timeConv[3]))) * 180 / 3.14159;

    //MIDO en la coordenada E-O
    valMed.mag = (((float) DISTANCE_EO / 2) * (1 / timeConv[0] - 1 / timeConv[1])) - OFFSET_ERROR_EO;
    //MIDO en la coordenada N-S
    valMed.deg = (((float) DISTANCE_NS / 2) * (1 / timeConv[2] - 1 / timeConv[3])) - OFFSET_ERROR_NS;

    return valMed;
}

//void anemometroTdetect(BaseType_t * pxHigherPriorityTaskWoken) {
//    xSemaphoreGiveFromISR(xSemaphoreTrenDetectado, pxHigherPriorityTaskWoken);
//}

void anemometroTdetected(BaseType_t * pxHigherPriorityTaskWoken) {
    vTaskNotifyGiveFromISR(xTaskToNotify, pxHigherPriorityTaskWoken);
}

void anemometroEmiterSelect(mux_transSelect_enum transd) {
    muxOutputSelect(transd);

    adc_transdSelect(transd);
}

void vApplicationIdleHook(void) {

}
