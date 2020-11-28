/*
 * File:   UART_RTOS.c
 * Author: MWA692
 *
 * Created on July 28, 2020, 10:22 AM
 */

#include "UART_RTOS.h"

/*declarations*/

/*Tasks*/
static void uart_task(void *pvParameters);

/* FreeRTOS declarations*/
static SemaphoreHandle_t xSemaphoreUartSend;
static QueueHandle_t qRecv;
static QueueHandle_t qSendMedicion;
static QueueHandle_t qMenuOpcion;
static QueueHandle_t qAnemometroModo;
static QueueHandle_t qAnemometroModoConfig;

TaskHandle_t pxUartTask;
wind_medicion_type abortMed = {9995.99, 9995.99};

anemometro_mode_enum modoActivo = Menu;

void uartInit_RTOS(void) {

    //Queue para pasar el modo de trabajo a la main task
    qMenuOpcion = xQueueCreate(5, sizeof (char));

    if (qMenuOpcion == NULL) {
        while (1);
    }
    //Queue para recibir los valores medidos en la main task
    qAnemometroModo = xQueueCreate(5, sizeof ( anemometro_mode_enum));

    if (qAnemometroModo == NULL) {
        while (1);
    }
    //Queue para pasar las opciones de configuracion a la main task
    qAnemometroModoConfig = xQueueCreate(5, sizeof ( anemometro_config_enum));

    if (qAnemometroModoConfig == NULL) {
        while (1);
    }

    if (xTaskCreate(uart_task, "uart_task", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, &pxUartTask) != pdPASS) {
        while (1);
    }

    uartInit();
}

void uartSendMenu(uart_menu_enum opcion) {
    uart_menu_enum op = opcion;

    switch (op) {
        case menuTemplate:
            uartSend((uint8_t *) MENU, sizeof (MENU), portMAX_DELAY);
            break;
        case menuTemplate_config:
            uartSend((uint8_t *) MENU_CONFIG, sizeof (MENU_CONFIG), portMAX_DELAY);
            break;
        case menuTemplate_coord:
            uartSend((uint8_t *) MENU_COORDENADAS, sizeof (MENU_COORDENADAS), portMAX_DELAY);
            break;
        case menuTemplate_period:
            uartSend((uint8_t *) MENU_PERIODOS, sizeof (MENU_PERIODOS), portMAX_DELAY);
            break;
        default: uartSend((uint8_t *) MENU, sizeof (MENU), portMAX_DELAY);
    }
}

void uartInit(void) {
    xSemaphoreUartSend = xSemaphoreCreateBinary();

    qRecv = xQueueCreate(16, sizeof (char));

    qSendMedicion = xQueueCreate(32, sizeof (wind_medicion_type));

    if (xSemaphoreUartSend == NULL || qRecv == NULL || qSendMedicion == NULL) {
        while (1);
    };

    U1MODEbits.STSEL = 0; // 1-Stop bit
    U1MODEbits.PDSEL = 0; // No Parity, 8-Data bits
    U1MODEbits.ABAUD = 0; // Auto-Baud disabled
    U1MODEbits.BRGH = 0; // High-Speed mode

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

    while ((ret < size) && (xQueueReceive(qRecv, &pBuf[ret], waitTick) == pdTRUE) && pBuf[ret] != '\n' && pBuf[ret] != '\r') {
        //eco
        if ((char) pBuf[ret] != '¬') uartSend(&pBuf[ret], 1, 0);
        ret++;
        //        waitTick = 0;
    }

    return ret;
}

uint32_t uartSend(uint8_t *pBuf, int32_t size, uint32_t blockTime) {
    int32_t ret = 0;
    TickType_t waitTick;

    //Para poder usar un blocktime hay que modificarlo
    if (blockTime != portMAX_DELAY)
        waitTick = blockTime / portTICK_PERIOD_MS;
    else
        waitTick = portMAX_DELAY;

    while (ret < size && pBuf[ret] != '\0') {
        U1TXREG = pBuf[ret];
        xSemaphoreTake(xSemaphoreUartSend, waitTick);
        //        waitTick = 0;
        ret++;
    }

    return ret;
}

static void uart_task(void *pvParameters) {
    anemometro_config_enum modoConfig = Exit;
    wind_medicion_type medSimple;
    char msg[50];
    char comando = 'z';
    char vel[6];
    float datFaux = 0;
    uint8_t notifyFlag = 1;
    float tempAmbiente = 25;

    while (1) {
        switch (modoActivo) {
            case Menu:
                xQueueSend(qAnemometroModo, &modoActivo, portMAX_DELAY);

                while (modoActivo == Menu || modoActivo == Exit) {
                    uartSendMenu(menuTemplate);

                    uartRecv((uint8_t *) & comando, 1, portMAX_DELAY);
                    if (comando < 53 && comando > 48) {
                        modoActivo = comando - 48;
                    }
                }
                notifyFlag = 1;
                break;
            case Medicion_Simple:
                if (notifyFlag == 1) {
                    xQueueSend(qAnemometroModo, &modoActivo, portMAX_DELAY);
                    notifyFlag = 0;
                }

                if (xQueueReceive(qSendMedicion, &medSimple, portMAX_DELAY) == pdTRUE) {
                    if (medSimple.mag < 55500) {
                        sprintf(msg, "\r\nMedición: %5.2f m/s ; %3f deg  %s\r\n%c",
                                (double) medSimple.mag, (double) medSimple.deg, medSimple.coord, '\0');
                    } else {
                        sprintf(msg, "\r\n NULL     NULL%c", '\0');
                    }
                    uartSend((uint8_t *) msg, sizeof (msg), portMAX_DELAY);
                    modoActivo = Menu;
                }
                //                if (xQueueReceive(qSendMedicion, &medSimple, 50 / portTICK_PERIOD_MS) == pdTRUE) {
                //                    sprintf(msg, "Medición: %4.4f m/s - %5.2f deg\r\n\0", medSimple.mag, medSimple.deg);
                //                    uartSend((uint8_t *) msg, sizeof (msg), portMAX_DELAY);
                //                } else {
                //                    modoActivo = Menu;
                //                }
                break;
            case Medicion_Continua:
                xQueueSend(qAnemometroModo, &modoActivo, portMAX_DELAY);

                if (xQueueReceive(qSendMedicion, &medSimple, portMAX_DELAY) == pdTRUE) {
                    if (medSimple.mag < 55500) {
                        //                    sprintf(msg, "\r\nMedición: %4.2f m/s - %4.2f deg\r\n", medSimple.mag, medSimple.deg);
                        sprintf(msg, "\r\n %3.2f     %3.2f  %s%c",
                                (double) medSimple.mag, (double) medSimple.deg, medSimple.coord, '\0');
                    } else {
                        sprintf(msg, "\r\n NULL     NULL%c", '\0');
                    }
                    uartSend((uint8_t *) msg, sizeof (msg), portMAX_DELAY);
                }
                /*Si recibo cualquier caracter termino el modo continuo*/
                if (uartRecv((uint8_t *) & exit, 1, 0) == 1) {
                    modoActivo = Menu;
                    //                    anemometroAbortMed();
                }
                break;
            case Configuracion:
                xQueueSend(qAnemometroModo, &modoActivo, portMAX_DELAY);

                uartSendMenu(menuTemplate_config);

                uartRecv((uint8_t *) & comando, 1, portMAX_DELAY);
                if (comando < 53 && comando > 48) {
                    modoConfig = comando - 49;
                }
                switch (modoConfig) {
                    case CalCero:
                        sprintf(vel,"FFFFFF");
                        
                        xQueueSend(qAnemometroModoConfig, &modoConfig, portMAX_DELAY);

                        sprintf(msg, "\r\n Ingrese Temperatura ambiente:\r\n%c", '\0');
                        uartSend((uint8_t *) msg, sizeof (msg), portMAX_DELAY);
                        uartRecv((uint8_t *) vel, 6, portMAX_DELAY);

                        tempAmbiente = atof(vel);
                        if (tempAmbiente == 0.0 && vel[0] != 48) {
                            datFaux = 999.99;
                            anemometroSendFloat(&datFaux);
                        } else {
                            datFaux = SOUND_VEL(tempAmbiente);

                            anemometroSendFloat(&datFaux);
                        }

                        if (xQueueReceive(qSendMedicion, &medSimple, portMAX_DELAY) == pdTRUE) {
                            sprintf(msg, "\r\n Deltas:\r\n%c", '\0');
                            uartSend((uint8_t *) msg, sizeof (msg), portMAX_DELAY);

                            sprintf(msg, " Oeste: %3.2f    Este: %3.2f%c",
                                    (double) medSimple.mag, (double) medSimple.deg, '\0');
                            uartSend((uint8_t *) msg, sizeof (msg), portMAX_DELAY);

                            xQueueReceive(qSendMedicion, &medSimple, portMAX_DELAY);
                            sprintf(msg, " Norte: %3.2f    Sur: %3.2f\r\n%c",
                                    (double) medSimple.mag, (double) medSimple.deg, '\0');
                            uartSend((uint8_t *) msg, sizeof (msg), portMAX_DELAY);
                        }

                        modoConfig = ExitConfig;
                        break;
                    case SetEmi:
                        xQueueSend(qAnemometroModoConfig, &modoConfig, portMAX_DELAY);

                        uartSendMenu(menuTemplate_coord);

                        uartRecv((uint8_t *) & comando, 1, portMAX_DELAY);
                        if (comando < 53 && comando > 48) {
                            modoConfig = comando - 48;
                            xQueueSend(qAnemometroModoConfig, &modoConfig, portMAX_DELAY);
                        }
                        modoConfig = ExitConfig;
                        break;
                    case SetPeriod:
                        xQueueSend(qAnemometroModoConfig, &modoConfig, portMAX_DELAY);

                        uartSendMenu(menuTemplate_period);

                        uartRecv((uint8_t *) & comando, 1, portMAX_DELAY);
                        if (comando < 54 && comando > 48) {
                            modoConfig = comando - 48;
                            xQueueSend(qAnemometroModoConfig, &modoConfig, portMAX_DELAY);
                        }
                        modoConfig = ExitConfig;
                        break;
                    case ExitConfig:
                        xQueueSend(qAnemometroModoConfig, &modoConfig, portMAX_DELAY);
                        break;
                    default: modoConfig = ExitConfig;
                }
                modoConfig = ExitConfig;
                modoActivo = Menu;
                break;
            default: modoActivo = Menu;
        }
    }
}

anemometro_mode_enum uartGetMode(void) {
    anemometro_mode_enum mod = Menu;

    if (xQueueReceive(qAnemometroModo, &mod, portMAX_DELAY) == pdTRUE) {
        return mod;
    } else {
        return Menu;
    }
}

anemometro_config_enum uartGetModeConfig(void) {
    anemometro_config_enum mod = Menu;

    if (xQueueReceive(qAnemometroModoConfig, &mod, portMAX_DELAY) == pdTRUE) {
        return mod;
    } else {
        return ExitConfig;
    }
}

void uartSendMed(wind_medicion_type med) {
    xQueueSend(qSendMedicion, &med, portMAX_DELAY);
}

void uartEndMode(void) {
    uint8_t end = '¬';
    xQueueSend(qRecv, &end, portMAX_DELAY);
}

void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void) {
    uint8_t data;
    BaseType_t xHigherPriorityTaskWoken;

    /* obtiene dato recibido por puerto serie */
    data = U1RXREG;

    /* pone dato en queue */
    xQueueSendFromISR(qRecv, &data, &xHigherPriorityTaskWoken);

    if (modoActivo == Medicion_Continua) {
        //            uartSendMed(abortMed);
        anemometroAbortMed();
        xHigherPriorityTaskWoken = pdTRUE;
    }

    IFS0bits.U1RXIF = 0;

    if (xHigherPriorityTaskWoken != pdFALSE) {
        taskYIELD();
    }
}

void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void) {
    BaseType_t xHigherPriorityTaskWoken;

    /* We have not woken a task at the start of the ISR. */
    xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(xSemaphoreUartSend, &xHigherPriorityTaskWoken);

    IFS0bits.U1TXIF = 0;

    if (xHigherPriorityTaskWoken != pdFALSE) {
        taskYIELD();
    }
}