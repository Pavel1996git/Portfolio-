/*
 * fsm.c
 *
 * Файл с основными функциями e32
 *
 */
#include "fsm.h"

//===============================================================================================================================//
//==============================================e32PinSetup(E32PinState mode)====================================================//
//===============================================================================================================================//
/**
 * @brief Настройка пинов E32 в соответствии с режимом работы.
 *
 * @param mode Режим работы E32.
 * @return uint8_t Возвращает 0 при успешной настройке, 1 в случае ошибки.
 */
uint8_t e32PinSetup(E32PinState mode) {
    uint8_t status = 0;

    // Выбор режима в зависимости от значения mode
    switch (mode) {
        case sleepMode:
            HAL_GPIO_WritePin(PIN_M0_GPIO_Port, PIN_M0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(PIN_M1_GPIO_Port, PIN_M1_Pin, GPIO_PIN_SET);
            break;
        case normalMode:
            HAL_GPIO_WritePin(PIN_M0_GPIO_Port, PIN_M0_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(PIN_M1_GPIO_Port, PIN_M1_Pin, GPIO_PIN_RESET);
            break;
        case wakeUpMode:
            HAL_GPIO_WritePin(PIN_M0_GPIO_Port, PIN_M0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(PIN_M1_GPIO_Port, PIN_M1_Pin, GPIO_PIN_RESET);
            break;
        case powerSavingMode:
            HAL_GPIO_WritePin(PIN_M0_GPIO_Port, PIN_M0_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(PIN_M1_GPIO_Port, PIN_M1_Pin, GPIO_PIN_SET);
            break;
        default:
            status = 1;
            #ifdef Debug
                // Вывод отладочной информации в случае ошибки
                printf("NUMBER_ERROR_SETUP_E32_PIN\n");
            #endif
            break;
    }

    // Возврат статуса (0 при успешной настройке, 1 в случае ошибки)
    return status;
}


