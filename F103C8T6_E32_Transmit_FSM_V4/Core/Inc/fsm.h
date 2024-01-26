/*
 * fsm.h
 *
 * Файл h с объявлениями основных функций и переменных
 *
 */
#ifndef INC_FSM_H_
#define INC_FSM_H_
//======================================================================//
//======================================================================//
//======================================================================//
//======================================================================//
#include "stm32f1xx_hal.h"
#include "main.h"
#include <stdio.h>
#include "fonts.h"
#include "ssd1306.h"
//========================================================================================//
//=============================StateFsm===================================================//
//========================================================================================//

#define Debug

#define ADC_CHANNELS_NUM 2
#define RX_BUF_SIZE 20

#define STATE_START 0
#define	STATE_ERROR_FOREWER 1
#define	STATE_WAIT 2
#define	STATE_BUTTON_CHOESE_DETONATOR 3
#define	STATE_BUTTON_MENU 4
#define	STATE_RX_MESSAGE 5
#define	STATE_TX_MESSAGE 6
#define	STATE_WAIT_FOREVER 7
#define	STATE_WAIT_MENU 8
#define	STATE_BUTTON_TEST 9
#define	STATE_BUTTON_DEMOLITION 10
#define	STATE_WAIT_TX 11

#define	NUMBER_STATE 12

#define NUMBER_EXPLODER 5

#define ADRES_NUMBER_BYTE_EXPLODER 3
#define ADRES_NUMBER_BYTE_DETONATOR_ON 4
#define ADRES_NUMBER_BYTE_VOLT 5
#define ADRES_NUMBER_BYTE_DEMOLITION_ON_EXPLODER 4

#define TIME_TX_DEMOLITION_WEAKE 6
#define TIME_TX_TEST_1_WEAKE 6

#define NUMBER_NOD 1
#define RX_BUF_SIZE0 6
#define RX_BUF_SIZE1 20
#define TIME_UART_CALLBACK_ERROR 500//10ms/cycle X cycle. cycle = 500
#define NUMBER_ERROR_SETUP_E32_MESSAGE_COMPARE 5
#define NUMBER_ERROR_INIT_E32_RF_CALLBACK 1
#define NUMBER_ERROR_READ_DET 15
#define	NUMBER_ERROR_SETUP_E32_PIN 10
#define	NUMBER_ERROR_HANDLE_FSM 20
#define FLASH_ADDRES_START 0x08010000
#define FLASH_ADDRES_START_ERROR 0x08010010
#define FLASH_ADDRES_READ_DETON_ERROR 0x08010020
#define FLASH_ADDRES_FSM_HANDLE_ERROR 0x08010030
#define MESSAGE_NUMBER 2 //Номер байта в ма�?�?иве отвечающего за имя �?ообщения. 1 - Демолитион, 2 - Лонг, 3 - Те�?т, 4 - Лог.
#define MESSAGE_NUMBER_DEMOLITION 1
#define MESSAGE_NUMBER_DEMOLITION_LONG 2
#define MESSAGE_NUMBER_TEST 3
#define MESSAGE_NUMBER_LOG 4
//#define BYTE_PULT_RESIV_DEMOLITION {0, 1, 2, 3, 4, 5, 6, 7, 8}
#define BYTE_PULT_RESIV_DEMOLITION {1, 3, 200, 3, 0, 5}
//#define BYTE_PULT_RESIV_DEMOLITION {0x30, 0x03, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30}
#define BYTE_PULT_RESIV_DEMOLITION_LONG {0x10, 0x02, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10}
#define BYTE_PULT_RESIV_TEST {3, 125, 2, 3, 4, 9}
#define BYTE_PULT_RESIV_LOG {0x20, 0x04, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20}
//#define BYTE_RESIV_PULT_DEMOLITION {0x60,0x01,0x60,0x60,0x60,0x60,0x60,0x60,0x60,0x60,0x60,0x60,0x60,0x60,0x60,0x60}
//#define BYTE_RESIV_PULT_DEMOLITION_LONG {0x02, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10}
#define BYTE_RESIV_PULT_TEST {3, 3, 2, 3, 4, 9}
#define BYTE_RESIV_PULT_LOG {0x20, 0x04, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00}

#define BYTE_RESIV_PULT_TEST {3, 3, 2, 3, 4, 9}
#define BYTE_RESIV_PULT_LOG {0x20, 0x04, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00}

#define STATE_INTERRUPS
// Перечисление для состояний
typedef enum {
    error,
    buttonDetonator,
    rxMessage,
    txMessage,
    buttonTest,
    buttonDemolition,
    buttonMenu,
    waitForev,
    waitMenu,
    waitTx,
    start,
    wait
} NextState;

NextState nextState;

// Структура для буфера дисплея
typedef struct {
    uint8_t bfDetonator[NUMBER_EXPLODER];
    uint8_t numberRxExploder[NUMBER_EXPLODER];
    uint8_t numberLogExploder[NUMBER_EXPLODER];
    uint8_t numberRxExploderOnDetonator[NUMBER_EXPLODER];
    uint8_t numberRxExploderVolt[NUMBER_EXPLODER];
    uint8_t bfReadDetonator;
    uint8_t bfPowerPult;
    uint8_t bfPowerResiv;
    uint8_t bfArm;
    uint8_t bfTest;
    uint8_t bfDemolition;
    uint8_t timeTxWake;
} BufferDisplay;

BufferDisplay bufferDisplay;

// Массивы данных для записи в память без знака шириной 32 бита
uint32_t flashData[];
uint32_t flashRxData[1];

// Переменные без знака шириной 16 бит
uint16_t adcVal;
uint16_t adcData[ADC_CHANNELS_NUM];
uint32_t adcVoltage[ADC_CHANNELS_NUM];

// Перечисление для состояний передачи
typedef enum {
    txDemolition,
    txTest,
    txNoTx
} Tx;

extern Tx txStatus;

// Перечисление для типов ошибок
typedef enum {
    e32FunctionInit = 1,
    startE32Error,
	startE32RxError,
    startDisplayError,
    txLogicError,
    txIntruptError,
    readVoltageError,
} ErrorType;

// Перечисление для состояний внешних прерываний
typedef enum {
    NoUsaNoExtern,
    NoUsaExtern,
    UsaNoExtern,
    UsaExtern
} NumberStateInterrupt;

// Перечисление для состояний пинов E32
typedef enum {
    normalMode,
    wakeUpMode,
    powerSavingMode,
    sleepMode
} E32PinState;

// Функции для управления прерываниями
void setInterruptBeginState(NumberStateInterrupt sInterruptbeginState);
void setInterruptEndState(NumberStateInterrupt sInterruptEndState);
void setInterruptState(NumberStateInterrupt sInterruptState);

// Функции вывода на дисплей
void outDisplay(void);
void outBufferDisplay(void);
void outDisplayUser(void);
void outDisplayWakeTx(void);
void outDisplayUser(void);

void startFsm(void);
void outError(void);
void outLoadMenu(void);
void outRxMessage(void);
void outTxMessage(void);
void outBlank(void);

// Функция чтения прерываний
uint32_t readInterrupt(void);

// Функции обработки ошибок
void handlerError(ErrorType error);
uint32_t handlerRxMessage(void);
uint32_t ForewerHandlerError(void);

// Функции управления временем
void waitShort(void);
void waitLong(void);
void waitLongMenu(void);
void waitLongTx(void);
void waitForever(void);

// Функции проверки сообщений
uint8_t checkRxStartMessage(uint8_t* pTx, uint8_t n);

// Функция чтения напряжения
void readPower(void);

// Функции проверки сообщений
uint8_t cheсkRxMessage(uint8_t* pTx, uint8_t* pTx2, uint8_t n);
uint8_t compareMessage(uint8_t* pTx, uint8_t n);
uint8_t compareAdressMessage(uint8_t* pTx);

// Глобальные переменные
extern uint8_t flagInterrupt; // Флаг прерывания
extern uint8_t flagButtonDemolition; // Флаг прерывания от кнопки демолитион
extern uint8_t flagButtonTest; // Флаг прерывания от кнопки тест

// Функция управления пинами E32
uint8_t e32PinSetup(E32PinState mode);

// Функция разрешения прерываний
void setInterruptState(NumberStateInterrupt sInterruptState);

// Массив переменных без знака шириной 8 бит
extern uint8_t receiveBuffer[RX_BUF_SIZE1];
extern uint8_t rxByteSize;

// Константы для приема сообщений
extern const uint8_t rxByteLog[];
extern const uint8_t rxByteTest[];

// Массивы для хранения начала байта сообщения
extern uint8_t txByteDemolition[];
extern const uint8_t txByteTest[];

// Переменные без знака шириной 32 бита
uint32_t errorHandleError;

// Массивы для инициализации
extern const uint8_t txByteSetup0[];
extern const uint8_t txByteSetup1[];

//Структура для управления переходами в конечном автомате
typedef struct tag_point {
    NumberStateInterrupt interruptBeginState; // Прерывания в начале текущего состояния
    NumberStateInterrupt interruptEndState;   // Прерывания в конце текущего состояния
    void (*out)(void);                        // Указатель на функцию вывода информации в текущем состоянии
    uint32_t (*in)(void);                     // Указатель на функцию ввода информации. Возвращает следующее состояние
    void (*time)(void);                       // Указатель на функция времени.
    uint32_t next[NUMBER_STATE];              // Массив следующих состояний
} tag_point;



#endif /* INC_FSM_H_ */
