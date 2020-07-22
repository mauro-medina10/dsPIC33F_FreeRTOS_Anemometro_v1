/*
 * File:   main.c
 * Author: MWA692
 *
 * Created on July 20, 2020, 2:46 PM
 */

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

// PLL activado
#define _PLLACTIVATED_

/*Funciones locales*/
static void prvSetupHardware(void);

/*--------Tasks declaration---------*/
static void led_test_task(void *pvParameters);


int main(void) {
    //Inicio Hardware
    prvSetupHardware();
    
    //Output compare
    pwm_init();

    //ADC init
    adc_init();

    if (xTaskCreate(led_test_task, "led_test_task", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        while (1);
    }

    vTaskStartScheduler();

    
    while(1);
    
    return 0;
}

static void led_test_task(void *pvParameters) {
    uint32_t count = 0;
    uint8_t flag = 0;

    while (1) {
        
        adc_start();
        
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        
        if (flag) {
            //Apaga led
            PORTAbits.RA4 = 0;

            //Apaga pwm
            //pwm_stop();

            flag = 0;
        } else {
            //Prende Led
            PORTAbits.RA4 = 1;

            //Arranca pwm
            //pwm_start();
            //pwm_pulseTrain_bloq(TRAIN_PULSE_LENGTH);
            
            flag = 1;
        }
    }
}

static void prvSetupHardware(void){
    #ifdef _PLLACTIVATED_
    // Configure Oscillator to operate the device at 40Mhz
    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2 = 35MHz
    // Fosc= 20M*28/(4*2)=70Mhz for 20M input clock
    PLLFBD = 26; // M=28
    CLKDIVbits.PLLPRE = 2; // N1=4
    CLKDIVbits.PLLPOST0 = 0; // N2=2  //Aveces se cuelga acá? se "soluciona" con un breakpoint despues de prvSetupHardware();)

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
    RCONbits.SWDTEN=0;
    
    /* set LED0 pins as outputs */
    TRISAbits.TRISA4 = 0;

    //Apago el LED
    PORTAbits.RA4 = 0;
}
void vApplicationIdleHook( void )
{

}
