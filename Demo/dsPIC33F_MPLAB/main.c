/*
 * File:   main.c
 * Author: MWA692
 *
 * Created on July 20, 2020, 2:46 PM
 */
#ifndef __dsPIC33FJ128GP802__
#define __dsPIC33FJ128GP802__
#endif

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
#include <adc.h>
#include <pwm.h>
#include <UART_RTOS.h>

// PLL activado
#define _PLLACTIVATED_
//Delay
#define DELAY_50uS asm volatile ("REPEAT, #1751"); Nop(); // 50uS delay
//Entradas mux
#define MUX_INPUT_A(b) (PORTAbits.RA1 = (b))
#define MUX_INPUT_B(b) (PORTAbits.RA0 = (b))
#define MUX_INPUT_INH(b)    (PORTBbits.RB4 = (b))
//Manejo led
#define LED_ON() (PORTAbits.RA4 = 1)
#define LED_OFF() (PORTAbits.RA4 = 0)
//Longitud tren de pulsos
#define TRAIN_PULSE_LENGTH 10

/*Typedef definitions*/
typedef enum {
    TRANS_EMISOR_OESTE = 0,
    TRANS_EMISOR_ESTE,
    TRANS_EMISOR_NORTE,
    TRANS_EMISOR_SUR
} mux_transSelect_enum;

/*Funciones locales*/
static void prvSetupHardware(void);

void muxOutputSelect(mux_transSelect_enum ch);

/*--------Tasks declaration---------*/
static void led_test_task(void *pvParameters);
static void transductor_test_task(void *pvParameters);

int main(void) {
    //Inicio Hardware
    prvSetupHardware();

    //Output compare
    comparador_rtos_init();

    //ADC init
    adc_init();

    //UART init
    uartInit();

    if (xTaskCreate(led_test_task, "led_test_task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        while (1);
    }

    //    if (xTaskCreate(transductor_test_task, "transductor_test_task", configMINIMAL_STACK_SIZE * 3, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
    //        while (1);
    //    }

    vTaskStartScheduler();


    while (1);

    return 0;
}

/*CADA 5 SEGUNDOS ENVIARÁ UN TREN DE PULSOS POR UNO DE LOS CANALES*/
static void transductor_test_task(void *pvParameters) {

    while (1) {

        vTaskDelay(10 / portTICK_PERIOD_MS);

        LED_ON();
        //CANAL 0
        muxOutputSelect(TRANS_EMISOR_OESTE);

        comparadorPulseTrain_bloq(TRAIN_PULSE_LENGTH);

        vTaskDelay(5000 / portTICK_PERIOD_MS);
        //CANAL 1
        LED_OFF();

        muxOutputSelect(TRANS_EMISOR_ESTE);

        comparadorPulseTrain_bloq(TRAIN_PULSE_LENGTH);

        vTaskDelay(5000 / portTICK_PERIOD_MS);
        //CANAL 2
        LED_ON();

        muxOutputSelect(TRANS_EMISOR_NORTE);

        comparadorPulseTrain_bloq(TRAIN_PULSE_LENGTH);

        vTaskDelay(5000 / portTICK_PERIOD_MS);
        //CANAL 3
        LED_OFF();

        muxOutputSelect(TRANS_EMISOR_SUR);

        comparadorPulseTrain_bloq(TRAIN_PULSE_LENGTH);

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

static void led_test_task(void *pvParameters) {
    uint8_t flag = 0;
    char str[] = "TEST\r\n";
    char strRev[10];

    while (1) {
        muxOutputSelect(TRANS_EMISOR_NORTE);
        vTaskDelay(5 / portTICK_PERIOD_MS);
        //Prendo 2do mux
        MUX_INPUT_INH(0);
        vTaskDelay(5 / portTICK_PERIOD_MS);
        //adc_start();

        if (flag) {
            //Apaga led
            LED_OFF();
            //Apaga pwm
            //            MUX_INPUT_INH(0);
            vTaskDelay(5 / portTICK_PERIOD_MS);
            //            filtroEnable();
            uartRecv((uint8_t *) strRev, 5, portMAX_DELAY);
            //comparadorStop();
            flag = 0;
        } else {
            //Prende Led
            LED_ON();
            vTaskDelay(5 / portTICK_PERIOD_MS);
            //            MUX_INPUT_INH(1);
            //            filtroDisable();
            //            vTaskDelay(5 / portTICK_PERIOD_MS);
            //UART
            uartSend((uint8_t *) str, 6, portMAX_DELAY);
            //Arranca pwm
            //comparadorStart();
            //            comparadorPulseTrainRTOS(TRAIN_PULSE_LENGTH);

            flag = 1;
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

static void prvSetupHardware(void) {
#ifdef _PLLACTIVATED_
    // Configure Oscillator to operate the device at 40Mhz
    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2 = 35MHz
    // Fosc= 20M*28/(4*2)=70Mhz for 20M input clock
    PLLFBD = 26; // M=28
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

    //Set mux control pins as outputs
    TRISAbits.TRISA0 = 0;
    TRISAbits.TRISA1 = 0;
    TRISBbits.TRISB4 = 0;

    //Apago el LED
    PORTAbits.RA4 = 0;

}

/*Configura los pines de salida del PORTA: RA1 = A y RA0 = B*/
void muxOutputSelect(mux_transSelect_enum ch) {
    switch (ch) {
        case TRANS_EMISOR_OESTE:
            MUX_INPUT_A(0);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            MUX_INPUT_B(0);
            break;
        case TRANS_EMISOR_ESTE:
            MUX_INPUT_A(1);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            MUX_INPUT_B(0);
            break;
        case TRANS_EMISOR_NORTE:
            MUX_INPUT_A(0);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            MUX_INPUT_B(1);
            break;
        case TRANS_EMISOR_SUR:
            MUX_INPUT_A(1);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            MUX_INPUT_B(1);
            break;
        default:
            MUX_INPUT_A(0);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            MUX_INPUT_B(0);
            break;
    }
}

void vApplicationIdleHook(void) {

}
