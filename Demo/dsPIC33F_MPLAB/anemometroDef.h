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
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h> 
#include <Math.h>

/*FreeRTOS includes*/
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "list.h"
#include "queue.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

//Distancias entre transductores
#define DISTANCE_NS 0.271  //Norte-Sur (blanco-negro)
#define DISTANCE_EO 0.276   //Este-Oeste (Rosa-Nada)

//Valor ADC sin excitacion
#define ADC_CERO 384
//Limites para deteccion de tren de pulsos
#define LIMIT_SUPERIOR 700
#define LIMIT_INF 100
#define LIMIT_SAFETY 465
#define LIMIT_SAFETY_NORTE 600
#define LIMIT_HIGH 386
#define MAX_LOSE_LIMIT 30

#define DETECTION_CERO_NS 0.0008
#define DETECTION_CERO_OE 0.0007942
#define DETECTION_CERO_MAX 0.00080397
#define DETECTION_CERO_MIN 0.00080395

#define DETECT_LIMIT_HIGH_O 384
#define DETECT_LIMIT_HIGH_E 385			   						  
#define DETECT_LIMIT_HIGH_N 382   
#define DETECT_LIMIT_HIGH_S 387

//Retardo de cada coordenada respecto al tiempo teorico (calc y medido para viento cero)
#define DETECTION_ERROR_O 0.0003390458
#define DETECTION_ERROR_E 0.0003163158		   						  
#define DETECTION_ERROR_N 0.0002907765
#define DETECTION_ERROR_S 0.0003144165

//Histeresis para tomar el maximo del tren de pulsos
#define MAX_THRESHOLD_O 1//0
#define MAX_THRESHOLD_E 4//6//1
#define MAX_THRESHOLD_N 8//1
#define MAX_THRESHOLD_S 6//0

#define DETECT_SCALING_OE 1//0.7841		
#define DETECT_SCALING_NS 1//0.739			   						  

#define DETECT_OFFSET_OE 0//-0.1361		
#define DETECT_OFFSET_NS 0//-0.0264

#define ANGLE_SCALING 1//1.031
#define ANGLE_OFFSET 0//9.5

#define OFFSET_ERROR_EO 0
#define OFFSET_ERROR_NS 0

#define MED_SCALING 1//0.976558333
#define MED_OFFSET 0//-0.356591667

//Numero de mediciones que se promedian
#define N_MED_PROM 20
#define N_TIMER_MODE 32

//definiciones tiempos
#define DELAY400 0.0004

// PLL activado
#define _PLLACTIVATED_

//Rtos
#define RTOS_AVAILABLE //El periferico se usará en contexto de un RTOS

//Delay
#define DELAY_50uS asm volatile ("REPEAT, #2001"); Nop(); // 50uS delay
#define DELAY_100uS asm volatile ("REPEAT, #4001"); Nop(); // 100uS delay
#define DELAY_400uS asm volatile ("REPEAT, #16001"); Nop(); // 400uS delay
#define DELAY_O asm volatile ("REPEAT, #14831"); Nop(); //14746
#define DELAY_E asm volatile ("REPEAT, #14831"); Nop(); //14708
#define DELAY_N asm volatile ("REPEAT, #15031"); Nop(); //14971
#define DELAY_S asm volatile ("REPEAT, #15001"); Nop(); //15031
#define DELAY_T asm volatile ("REPEAT, #15031"); Nop();

#define Ous_DELAY 0.0003687 //14676 / 40000000
#define Eus_DELAY 0.0003677 //14708 / 40000000
#define Nus_DELAY 0.0003743 //14955 / 40000000
#define Sus_DELAY 0.0003758  //14960 / 40000000

//Entradas mux
#define MUX_INPUT_A(b) (PORTAbits.RA1 = (b))
#define MUX_INPUT_B(b) (PORTAbits.RA0 = (b))
#define MUX_INPUT_INH(b) (PORTBbits.RB4 = (b))
//#define MUX_INPUT_INH(b) (PORTBbits.RB4 = (b))
//Manejo led
#define RB_9_SET(b) (PORTBbits.RB9 = (b))
#define RB_8_SET_VAL(b) (PORTBbits.RB8 = (b))
#define RB_8_SET_MODE(b) (TRISBbits.TRISB8 = (b))
#define LED_ON (PORTAbits.RA4 = 1)
#define LED_OFF (PORTAbits.RA4 = 0)
//Longitud tren de pulsos
#define TRAIN_PULSE_LENGTH 10
//CALCULO TEORICO VELOCIDAD DEL SONIDO
#define SOUND_VEL(b) (float)(331.228 * sqrtf((b + 273.16)/273.16))

/*typedef definitions*/
typedef enum {
    Menu = 0,
    Medicion_Simple,
    Medicion_Continua,
    Configuracion,
    Exit
} anemometro_mode_enum;

typedef enum {
    CalCero = 0,
    SetEmi,
    SetPeriod,
    ExitConfig
} anemometro_config_enum;

typedef enum {
    MAXIMO_GLOBAL = 0,
    MAXIMO_LOCAL,
    MINIMO_LOCAL,
    CRUCE_CERO,
    PRIMERA_MUESTRA
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
    char coord[3];
} wind_medicion_type;

/*FreeRTOS definitions*/

/*Funciones*/
BaseType_t anemometroMuxOutputSelect(mux_transSelect_enum ch);

wind_medicion_type anemometroGetMed(void);

void anemometroTdetectedFromISR(BaseType_t *pxHigherPriorityTaskWoken, uint32_t val);

void anemometroEmiterSelect(mux_transSelect_enum transd);

float anemometroGetVcoord(mux_transSelect_enum coordV);

BaseType_t anemometroGetProm(float* medOE, float* medNS, uint8_t prom);

float anemometroGetCoordTime(mux_transSelect_enum coordTime);

void anemometroReceptorDelay(mux_transSelect_enum emisor);

BaseType_t anemometroCalcMode(float* pData, float* pMode, uint16_t nData);

void anemometroCalibCero(float Sspeed);

void anemometroAbortMed(void);

void anemometroDelayTest(void);

void anemometroSendFloat(float* dat);

void anemometroRecvFloat(float* dat);
/*FreeRTOS declarations*/

/*Global variables*/
//static float DETECTION_ERROR_O = 0.0002165070;
//static float DETECTION_ERROR_E = 0.0002155070;
//static float DETECTION_ERROR_N = 0.0002216887;
//static float DETECTION_ERROR_S = 0.0002226887;

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

