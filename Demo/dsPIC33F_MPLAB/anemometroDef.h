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
#define DISTANCE_NS 0.176  //Norte-Sur (blanco-negro)
#define DISTANCE_EO 0.185   //Este-Oeste (Rosa-Nada)

//Limites para deteccion de tren de pulsos
#define LIMIT_SUP_E 392 
#define LIMIT_INF_E 351 
#define LIMIT_SUP_O 393 
#define LIMIT_INF_O 352 
#define LIMIT_SUP_N 393 
#define LIMIT_INF_N 351 
#define LIMIT_SUP_S 396 
#define LIMIT_INF_S 353 

#define DETECTION_ERROR_E 278.43
#define DETECTION_ERROR_O 226.31
#define DETECTION_ERROR_N 244.81
#define DETECTION_ERROR_S 239.61

// PLL activado
#define _PLLACTIVATED_

//Rtos
#define RTOS_AVAILABLE //El periferico se usará en contexto de un RTOS

//Delay
#define DELAY_50uS asm volatile ("REPEAT, #2001"); Nop(); // 50uS delay
#define DELAY_300uS asm volatile ("REPEAT, #12001"); Nop(); // 300uS delay
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
    PRIMER_LIMITE = 0,
    SEGUNDO_LIMITE
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


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

