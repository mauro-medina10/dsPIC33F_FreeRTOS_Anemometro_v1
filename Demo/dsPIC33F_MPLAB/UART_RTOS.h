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
#ifndef UART_RTOS_HEADER_H
#define	UART_RTOS_HEADER_H

#include <xc.h> // include processor files - each processor file is guarded.  

#include <stdint.h>
#include <stdio.h> 
#include <stdbool.h>

/*anemometro headers*/
#include <anemometroDef.h>

/*FreeRTOS includes*/
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "list.h"
#include "queue.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/*Defines*/
#define BAUDRATE 115200
#define BRGVAL ((35000000/BAUDRATE)/4)-1

/*Global variables*/
static const char MENU[] = "1- Medicion Simple\r\n2- Medicion Continua\r\n3- Configuracion\r\n\nIngrese opcion:\r\n";
static const char MENU_COORDENADAS[] = "1- Norte\r\n2- Sur\r\n3- Este\r\n4- Oeste\r\n\nIngrese opcion:\r\n";

/*Typedef*/
typedef enum {
    menuTemplate = 0,
    menuTemplate_config,
} uart_menu_enum;

/* FreeRTOS declarations*/
static SemaphoreHandle_t xSemaphoreUartSend;
static QueueHandle_t qRecv;
static QueueHandle_t qSendMedicion;
static QueueHandle_t qMenuOpcion;
static QueueHandle_t qAnemometroModo;

void uartInit(void);

void uartInit_RTOS(void);

void uartSendMenu(uart_menu_enum opcion);

uint32_t uartSend(uint8_t *pBuf, int32_t size, uint32_t blockTime);

uint32_t uartRecv(uint8_t *pBuf, int32_t size, uint32_t blockTime);

anemometro_mode_enum uartGetMode(void);

void uartSendMed(wind_medicion_type med);

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

