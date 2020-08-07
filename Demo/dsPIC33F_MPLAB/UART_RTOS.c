/*
 * File:   UART_RTOS.c
 * Author: MWA692
 *
 * Created on July 28, 2020, 10:22 AM
 */

#include "UART_RTOS.h"

/*Defines*/
#define BAUDRATE 115200
#define BRGVAL ((35000000/BAUDRATE)/4)-1

#define MENU "1- Medir\r\n2- Elegir emisor\r\n\nIngrese opcion: "
#define MENU_COORDENADAS "1- Norte\r\n2- Sur\r\n3- Este\r\n4- Oeste\r\n\nIngrese opcion: "

/*Global variables*/
bool txHasEnded = false;

/* FreeRTOS declarations*/
static void uart_task(void *pvParameters);

static SemaphoreHandle_t xSemaphoreUartSend;
static QueueHandle_t qRecv;

void uartInit_RTOS(void) {
    if (xTaskCreate(uart_task, "uart_task", configMINIMAL_STACK_SIZE * 3, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        while (1);
    }

    uartInit();
}

void uartSendMenu(uint8_t o) {
    uint8_t opcion = o;
    //    static const char MENU[] = "1- Medir\r\n2- Elegir emisor\r\n\nIngrese opcion: ";
    //    static const char MENU_COORDENADAS[] = "1- Norte\r\n2- Sur\r\n3- Este\r\n4- Oeste\r\n\nIngrese opcion: ";
    switch (opcion) {
        case 1:
            uartSend(&MENU, sizeof(MENU), portMAX_DELAY);
            break;
        case 2:
            uartSend(&MENU_COORDENADAS, sizeof(MENU_COORDENADAS), portMAX_DELAY);
            break;
        default: uartSend(&MENU, sizeof(MENU), portMAX_DELAY);
    }
}

void uartInit(void) {
    xSemaphoreUartSend = xSemaphoreCreateBinary();

    qRecv = xQueueCreate(16, sizeof (char));

    if (xSemaphoreUartSend == NULL || qRecv == NULL) {
        while (1);
    };

    U1MODEbits.STSEL = 0; // 1-Stop bit
    U1MODEbits.PDSEL = 0; // No Parity, 8-Data bits
    U1MODEbits.ABAUD = 0; // Auto-Baud disabled
    U1MODEbits.BRGH = 1; // High-Speed mode

    U1BRG = BRGVAL; // Baud Rate setting for 115200

    U1STAbits.UTXISEL0 = 0; // Interrupt after one TX character is transmitted
    U1STAbits.URXISEL = 0; // Interrupt after one RX character is received;
    IPC2bits.U1RXIP = 1; // Setup RX priryty 1 interrupt for
    IPC3bits.U1TXIP = 1; // Setup TX priryty 1 interrupt for
    IFS0bits.U1RXIF = 0; // Clear the Recieve Interrupt Flag
    IEC0bits.U1RXIE = 1; // Enable Recieve Interrupt
    U1STAbits.UTXISEL1 = 0; // Interrupt when a character is transferred to the Transmit Shift register 
    U1STAbits.UTXISEL0 = 0;
    IFS0bits.U1TXIF = 0; // Clear the Transmiter Interrupt Flag
    IEC0bits.U1TXIE = 1; // Enable UART TX interrupt

    U1MODEbits.UARTEN = 1; // Enable UART
    U1STAbits.UTXEN = 1; // Enable UART TX

    //Configuro Pin 15 como RP6: U1RX 
    RPINR18bits.U1RXR = 6;
    //Configuro pin 16 como RP7 : U1TX = 00011 
    RPOR3bits.RP7R = 3;
}

uint32_t uartRecv(uint8_t *pBuf, int32_t size, uint32_t blockTime) {
    int32_t ret = 0;
    TickType_t waitTick;

    if (blockTime != portMAX_DELAY)
        waitTick = blockTime / portTICK_PERIOD_MS;
    else
        waitTick = portMAX_DELAY;

    while ((ret < size) && (xQueueReceive(qRecv, &pBuf[ret], waitTick) == pdTRUE)) {
        ret++;
        //        waitTick = 0;
    }

    return ret;
}

uint32_t uartSend(uint8_t *pBuf, int32_t size, uint32_t blockTime) {
    int32_t ret = 0;
    TickType_t waitTick;

    if (blockTime != portMAX_DELAY)
        waitTick = blockTime / portTICK_PERIOD_MS;
    else
        waitTick = portMAX_DELAY;

    while (ret < size) {
        U1TXREG = pBuf[ret];
        xSemaphoreTake(xSemaphoreUartSend, waitTick);
        waitTick = 0;
        ret++;
    }

    return ret;
}

static void uart_task(void *pvParameters) {

    while (1) {

    }
}

void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void) {
    uint8_t data;
    BaseType_t xHigherPriorityTaskWoken;

    /* obtiene dato recibido por puerto serie */
    data = U1RXREG;

    /* pone dato en queue */
    xQueueSendFromISR(qRecv, &data, &xHigherPriorityTaskWoken);

    IFS0bits.U1RXIF = 0;

    if (xHigherPriorityTaskWoken != pdFALSE) {
        taskYIELD();
    }
}

void __attribute__((__interrupt__)) _U1TXInterrupt(void) {
    BaseType_t xHigherPriorityTaskWoken;

    /* We have not woken a task at the start of the ISR. */
    xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(xSemaphoreUartSend, &xHigherPriorityTaskWoken);

    IFS0bits.U1TXIF = 0;

    if (xHigherPriorityTaskWoken != pdFALSE) {
        taskYIELD();
    }
}