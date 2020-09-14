/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef ANEMOMETRO_DEF_HEADER_TEMPLATE_H
#define	ANEMOMETRO_DEF_HEADER_TEMPLATE_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>
#include <stdio.h> 

/*FreeRTOS includes*/
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "list.h"
#include "queue.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

//Distancias entre transductores
#define DISTANCE_NS 0.276  //Norte-Sur (blanco-negro)
#define DISTANCE_EO 0.276   //Este-Oeste (Rosa-Nada)

//Valor ADC sin excitacion
#define ADC_CERO 377
//Limites para deteccion de tren de pulsos
#define LIMIT_SUPERIOR 377
#define LIMIT_INF 376
#define LIMIT_SAFETY 382

#define DETECTION_ERROR_O 0.00026255
#define DETECTION_ERROR_E 0.00026108						   						  
#define DETECTION_ERROR_N 0.00026940   
#define DETECTION_ERROR_S 0.00026949 

#define OFFSET_ERROR_EO 0//4.867876	
#define OFFSET_ERROR_NS 0//-2.3815

//Numero de mediciones que se promedian
#define N_TIMER_PROM 20
#define N_MED_PROM 5

// PLL activado
#define _PLLACTIVATED_

//Rtos
#define RTOS_AVAILABLE //El periferico se usará en contexto de un RTOS

//Delay
#define DELAY_50uS asm volatile ("REPEAT, #2001"); Nop(); // 50uS delay
#define DELAY_100uS asm volatile ("REPEAT, #4001"); Nop(); // 100uS delay
#define DELAY_400uS asm volatile ("REPEAT, #16001"); Nop(); // 400uS delay
#define DELAY_O asm volatile ("REPEAT, #14181"); Nop();
#define DELAY_E asm volatile ("REPEAT, #14081"); Nop();
#define DELAY_N asm volatile ("REPEAT, #14401"); Nop();
#define DELAY_S asm volatile ("REPEAT, #14411"); Nop();
//#define DELAY_N asm volatile ("REPEAT, #2081"); Nop();
//#define DELAY_S asm volatile ("REPEAT, #2091"); Nop();
//Entradas mux
#define MUX_INPUT_A(b) (PORTAbits.RA1 = (b))
#define MUX_INPUT_B(b) (PORTAbits.RA0 = (b))
#define MUX_INPUT_INH(b)    (PORTBbits.RB4 = (b))
//Manejo led
#define LED_ON (PORTAbits.RA4 = 1)
#define LED_OFF (PORTAbits.RA4 = 0)
//Longitud tren de pulsos
#define TRAIN_PULSE_LENGTH 10

/*typedef definitions*/
typedef enum {
    Menu = 0,
    Medicion_Simple,
    Medicion_Continua,
    Configuracion
} anemometro_mode_enum;

typedef enum {
    SEMI_POSITIVO = 0,
    SEMI_NEGATIVO,
    PRIMERA_SAMPLE
} anemometro_deteccion_enum;

typedef enum {
    TRANS_EMISOR_OESTE = 0,
    TRANS_EMISOR_ESTE,
    TRANS_EMISOR_NORTE,
    TRANS_EMISOR_SUR
} mux_transSelect_enum;

/*Structs definitions*/
typedef struct {
    float mag;
    float deg;
} wind_medicion_type;

/*FreeRTOS definitions*/

/*Funciones*/
BaseType_t muxOutputSelect(mux_transSelect_enum ch);

wind_medicion_type anemometroGetMed(void);

void anemometroTdetected(BaseType_t *pxHigherPriorityTaskWoken, uint32_t val);

void anemometroEmiterSelect(mux_transSelect_enum transd);

/*FreeRTOS declarations*/


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

