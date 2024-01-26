/*
 * fsm.c
 *
 * Файл с основными функциями фсм
 *
 */
#include "fsm.h"


//====================================================================//
//====================================================================//
//====================================================================//
//====================================================================//

/**
 * @brief Замена стандартной функции вывода данных для использования с микросхемой Stm32.
 *
 * @param file - Идентификатор файла (не используется).
 * @param ptr - Указатель на массив символов для вывода.
 * @param len - Длина массива.
 * @return int - Длина выводимых данных.
 */
int _write(int file, char *ptr, int len)
{
  (void)file; // Подавление предупреждения о неиспользуемом параметре.
  int DataIdx;

  // Цикл по каждому символу в массиве.
  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    ITM_SendChar(*ptr++); // Отправка символа.
  }
  return len; // Возвращается длина выводимых данных.
}

NumberStateInterrupt InterruptState; // Глобальная переменная для хранения текущего состояния прерывания.

/**
 * @brief Устанавливает прерывания в начале состояния и вызывает функцию установки прерывания.
 *
 * @param sInterruptbeginState - Начальное состояние прерывания.
 */
void setInterruptBeginState(NumberStateInterrupt sInterruptbeginState)
{
#ifdef Debug
    printf("setInterruptBeginState.\n"); // Вывод отладочного сообщения (если включен режим отладки)
#endif
    InterruptState = sInterruptbeginState; // Установка начального состояния прерывания
    setInterruptState(InterruptState);      // Вызов функции установки прерывания
}
/**
 * @brief Устанавливает прерывания в конце состояния и вызывает функцию установки прерывания
 *
 * @param sInterruptbeginState - Начальное состояние прерывания
 */
void setInterruptEndState(NumberStateInterrupt sInterruptEndState){
#ifdef Debug
	printf("setInterruptEndState().\n"); //Вывод отладочного сообщения
#endif
	InterruptState= sInterruptEndState; // Установка конченого состояния прерывания
	setInterruptState(InterruptState); // Вызов функции установки прерывания
}

//=============================================================================================================================//
//===============================================setInterruptState=============================================================//
//=============================================================================================================================//
/**
 * @brief Устанавливает состояние прерывания в соответствии с переданным значением.
 *
 * @param sInterruptState - Состояние прерывания, которое необходимо установить.
 */
void setInterruptState(NumberStateInterrupt sInterruptState)
{
    switch (sInterruptState)
    {
    case NoUsaNoExtern:
        HAL_NVIC_DisableIRQ(USART1_IRQn); // Отключение прерывания USART1
        HAL_NVIC_DisableIRQ(EXTI15_10_IRQn); // Отключение прерывания EXTI15_10
        HAL_NVIC_DisableIRQ(EXTI9_5_IRQn); // Отключение прерывания EXTI9_5
        break;
    case NoUsaExtern:
        HAL_NVIC_DisableIRQ(USART1_IRQn); // Отключение прерывания USART1
        HAL_NVIC_EnableIRQ(EXTI15_10_IRQn); // Включение прерывания EXTI15_10
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn); // Включение прерывания EXTI9_5
        break;
    case UsaNoExtern:
        HAL_NVIC_EnableIRQ(USART1_IRQn); // Включение прерывания USART1
        HAL_NVIC_DisableIRQ(EXTI15_10_IRQn); // Отключение прерывания EXTI15_10
        HAL_NVIC_DisableIRQ(EXTI9_5_IRQn); // Отключение прерывания EXTI9_5
        break;
    case UsaExtern:
        HAL_NVIC_EnableIRQ(USART1_IRQn); // Включение прерывания USART1
        HAL_NVIC_EnableIRQ(EXTI15_10_IRQn); // Включение прерывания EXTI15_10
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn); // Включение прерывания EXTI9_5
        break;
    default:
        break;
    }
}

//Пустая функция для вечного цикла ошибки
void outError(void){
#ifdef Debug
	printf("---------------------------ERROR FOREVER-------------().\n");
#endif
	HAL_Delay(1);
}
//Пустая функция
void outBlank(void){
	HAL_Delay(1);
}
//Пустая функция для состояния приема сообщения
void outRxMessage(void){
	HAL_Delay(1);
}
//Функция короткого ожидания. 1 мс
void waitShort(void){
#ifdef Debug
	printf("waitShort().\n");
#endif
	HAL_Delay(1);
}
//Пустая функция ожидания
void waitLongMenu(void){
	HAL_Delay(1);
}

//=============================================================================================================================//
//=====================================================checkRxStartMessage=====================================================//
//=============================================================================================================================//


/**
 * @brief Проверяет, совпадает ли начальное сообщение с ожидаемым.
 *
 * @param pTx - Указатель на массив ожидаемого сообщения.
 * @param n - Размер ожидаемого сообщения.
 * @return uint8_t - Статус проверки (0 - совпадение, 1 - несовпадение).
 */
uint8_t checkRxStartMessage(uint8_t* pTx, uint8_t n) {
#ifdef Debug
    printf("checkRxStartMessage().\n");
#endif
    HAL_Delay(500);

    for (uint8_t i = 0; i < n; i++) { // Проверяет совпадения первых n символов с ожиданием
        if (receiveBuffer[i] != *(pTx + i)) {
            nextState = error;
#ifdef Debug
            printf("compareRxMessage_Error.\n");
            for (int k = 0; k < 5; k++) {
                printf("compareRxMessage_Error.%d\n", receiveBuffer[k]); // Выводит первые пять символов при несовпадении для отладки
            }
#endif
            handlerError(startE32RxError);
            return 1; // Несовпадение сообщений
        }
    }

    return 0; // Совпадение сообщений
}

//=============================================================================================================================//
//=====================================================handlerError============================================================//
//=============================================================================================================================//
/**
 * @brief Обрабатывает ошибку и выполняет соответствующие действия в зависимости от типа ошибки.
 *
 * @param error - Тип ошибки (перечисление ErrorType).
 */
void handlerError(ErrorType error) {
    switch (error) {
        case e32FunctionInit:
#ifdef Debug
            printf("++++++++++++++++++++++++++++Error in e32PinSetup\n");
#endif
            errorReboot((uint8_t)error);
            break;
        case txLogicError:
#ifdef Debug
            printf("+++++++++++++++++++++++Error in outTxMessage.\n");
#endif
            errorReboot((uint8_t)error);
            break;
        case startE32Error:
#ifdef Debug
            printf("+++++++++++++++++++++++Error in Start.\n");
#endif
            errorReboot((uint8_t)error);
            break;
        case startDisplayError:
#ifdef Debug
            printf("+++++++++++++++++++++++Error in startDisplay.\n");
#endif
            errorReboot((uint8_t)error);
            break;
        case txIntruptError:
#ifdef Debug
            printf("+++++++++++++++++++++++Error in outTxMessage.\n");
#endif
            errorReboot((uint8_t)error);
            break;
        case readVoltageError:
#ifdef Debug
            printf("+++++++++++++++++++++++Error in ReadVolt.\n");
#endif
            break;
        case startE32RxError:
#ifdef Debug
            printf("+++++++++++++++++++++++Error in outTxMessage.\n");
#endif
            errorReboot((uint8_t)error);
            break;
        default:
            break;
    }
    HAL_Delay(1);
}

//=============================================================================================================================//
//=====================================================errorReboot============================================================//
//=============================================================================================================================//
/**
 * @brief Конечный цикл мигания светодиодом в случае ошибки, с последующим сбросом системы.
 *
 * @param numError - Номер ошибки (используется для определения числа миганий).
 */
void errorReboot(uint8_t numError) {
    while (1) {
        for (int i = 0; i < (numError * 2); i++) {
            // Мигаем светодиодом
            togleLed();
            HAL_Delay(400);
        }
        HAL_Delay(500);
        // Включаем сброс устройства
        //RCC_DeInit();//-------------------------------------------------------
        // Инициализируем системные настройки (тактовую частоту и т.д.)
        SystemInit();
        // Включаем сброс системы
        NVIC_SystemReset();
    }
}
//=============================================================================================================================//
//===================================================cheсkRxMessage============================================================//
//=============================================================================================================================//
/**
 * @brief Проверяет принятое сообщение и переходит в соответствующее состояние.
 *
 * @param pTx Указатель на первый байт принятого сообщения.
 * @param pTx2 Указатель на первый байт жидаемого сообщения.
 * @param n Размер сообщения.
 * @return NextState - Следующее состояние автомата.
 */
uint8_t cheсkRxMessage(uint8_t* pTx, uint8_t* pTx2, uint8_t n) {
    if (bufferDisplay.timeTxWake >= 1) {  // Проверка условия времени передачи
        if ((compareMessage(pTx, n) == 1) && (compareAdressMessage(pTx) == 1)) { // Проверка совпадения сообщения и адреса
            // Обновление данных в буфере дисплея
            bufferDisplay.numberRxExploder[receiveBuffer[ADRES_NUMBER_BYTE_EXPLODER]] = 1;
            bufferDisplay.numberRxExploderOnDetonator[receiveBuffer[ADRES_NUMBER_BYTE_EXPLODER]] = receiveBuffer[ADRES_NUMBER_BYTE_DETONATOR_ON];
            bufferDisplay.numberRxExploderVolt[receiveBuffer[ADRES_NUMBER_BYTE_EXPLODER]] = receiveBuffer[ADRES_NUMBER_BYTE_VOLT];
            // Возврат состояния передачии
            return waitTx;
        }
    }
    // Проверка условия времени передачи (если не выполнено предыдущее условие)
    else if (bufferDisplay.timeTxWake < 1) {
        // Проверка совпадения сообщения и адреса (второе сообщение)
        if ((compareMessage(pTx2, n) == 1) && (compareAdressMessage(pTx2) == 1)) {
            // Возврат состояния меню
            return waitMenu;
        }
    }
    // Возврат состояния ожидания
    return wait;
}

//=============================================================================================================================//
//===================================================compareMessage============================================================//
//=============================================================================================================================//
/**
 * @brief Сравнивает принятое сообщение с ожидаемым.
 *
 * @param pTx Указатель на первый байт ожидаемого сообщения.
 * @param n Размер сообщения.
 * @return uint8_t - 1, если сообщения совпадают, 0 в противном случае.
 */
uint8_t compareMessage(uint8_t* pTx, uint8_t n) {
    // Цикл сравнения байтов сообщений
    for (uint8_t i = 0; i < n; i++) {
        // Проверка совпадения байтов
        if (receiveBuffer[i] != *(pTx + i)) {
            #ifdef Debug
                // Вывод отладочной информации об несовпадении
                printf("compareRxMessage_Error.\n");
                for (int k = 0; k < 16; k++) {
                    printf("compareRxMessage_Error.%d\n", receiveBuffer[k]);
                }
            #endif
            return 0; // Возврат 0 в случае несовпадения
        }
    }
    return 1; // Возврат 1, если сообщения совпадают
}

//=============================================================================================================================//
//===================================================compareAdressMessage============================================================//
//=============================================================================================================================//
/**
 * @brief Сравнивает адрес полученного сообщения с допустимыми значениями.
 *
 * @param pTx Указатель на байт адреса сообщения.
 * @return uint8_t - 1, если адрес в пределах допустимых значений, 0 в противном случае.
 */
uint8_t compareAdressMessage(uint8_t* pTx) {
    // Проверка адреса на превышение максимального значения
    if (receiveBuffer[ADRES_NUMBER_BYTE_EXPLODER] > NUMBER_EXPLODER) {
        #ifdef Debug
            // Вывод отладочной информации об ошибке
            printf("+++++++++++++++ERROR NUMBAR EXPLODER RX MESSAGE TEST.%d\n", receiveBuffer[ADRES_NUMBER_BYTE_EXPLODER]);
        #endif
        return 0;
    } else {
        return 1; // Возврат 1, если адрес в пределах допустимых значений
    }
}
//=============================================================================================================================//
//=====================================================ForewerHandlerError=====================================================//
//=============================================================================================================================//
/**
 * @brief Обработчик ошибок, выполняющий бесконечный цикл мигания светодиода и запись ошибки во флеш-память.
 *
 * @return uint32_t Возвращает 0 (не достигается, так как функция выполняет бесконечный цикл).
 */
uint32_t ForewerHandlerError(void) {
    #ifdef Debug
        // Вывод отладочной информации
        printf("ERROR_FOREVER\n");
    #endif

    // Запись ошибки во флеш-память
    Flash_Write_Data(FLASH_ADDRES_START_ERROR, (uint32_t *)flashData, 1);

    // Бесконечный цикл мигания светодиода
    while (1) {
            HAL_GPIO_TogglePin(PIN_LED_GPIO_Port, PIN_LED_Pin);
            HAL_Delay(200);

    			}

    // Возврат 0 (не достигается, так как функция выполняет бесконечный цикл)
    return 0;
    HAL_Delay(1); // Этот код недостижим, так как предыдущий цикл бесконечный
}
