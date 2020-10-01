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
#include <stdlib.h>

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
#define BRGVAL ((40000000/BAUDRATE)/16)-1

/*Global variables*/
static const char MENU[] = "\r\n1- Medicion Simple\r\n2- Medicion Continua\r\n3- Configuracion\r\n\nIngrese opcion:\r\n\0";
static const char MENU_COORDENADAS[] = "1- Norte\r\n2- Sur\r\n3- Este\r\n4- Oeste\r\n\nIngrese opcion:\r\n\0";
static const char MENU_CONFIG[] = "\r\n1- Calibracion Cero\r\n2- Set Emisor\r\n3- Periodo Medicion\r\n4- Salir\r\n\nIngrese opcion:\r\n\0";
static const char MENU_PERIODOS[] = "\r\n1- 5s\r\n2- 10s\r\n3- 30s\r\n4- 1m\r\n5- 10m\r\n6- Salir\r\n\nIngrese opcion:\r\n\0";

/*Typedef*/
typedef enum {
    menuTemplate = 0,
    menuTemplate_config,
    menuTemplate_period,
    menuTemplate_coord
} uart_menu_enum;

void uartInit(void);

void uartInit_RTOS(void);

void uartSendMenu(uart_menu_enum opcion);

uint32_t uartSend(uint8_t *pBuf, int32_t size, uint32_t blockTime);

uint32_t uartRecv(uint8_t *pBuf, int32_t size, uint32_t blockTime);

anemometro_mode_enum uartGetMode(void);

anemometro_config_enum uartGetModeConfig(void);

void uartSendMed(wind_medicion_type med);

void uartEndMode(void);

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

