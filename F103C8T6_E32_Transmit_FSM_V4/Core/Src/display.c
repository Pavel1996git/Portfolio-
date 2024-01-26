/*
 * fsm.c
 *
 *  Created on: Nov 11, 2023
 *      Author: semen
 */
#include "fsm.h"

// Структуры и перечисления

//===============================================================================================================================//
//=================================================outDisplayUser()===============================================================//
//===============================================================================================================================//
void outDisplayUser(void)
{
	//HAL_Delay(200);
	 //SSD1306_Clear();
	char buffer[20];
	setInterruptState(NoUsaNoExtern);
	//HAL_NVIC_DisableIRQ(USART1_IRQn);
	//HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	//HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
	SSD1306_Fill (0);

	for(uint8_t i = 0; i< NUMBER_EXPLODER; i++ ){
		if (bufferDisplay.bfDetonator[i]==1){
			SSD1306_GotoXY (0,(i*12)+12);
			//bufferDisplay.bfDetonator[i];
			sprintf(buffer, "%d", (i+1));
			SSD1306_Puts ("Det ", &Font_7x10, 1);
			SSD1306_Puts (buffer, &Font_7x10, 1);
			SSD1306_Puts (":", &Font_7x10, 1);
		}
		if (bufferDisplay.numberRxExploder[i] == 1){
			SSD1306_GotoXY (40,(i*12));
			SSD1306_Puts (" +", &Font_7x10, 1);
			bufferDisplay.numberRxExploder[i] = 0;
		}
		if (bufferDisplay.numberRxExploderOnDetonator[i] == 1){
			SSD1306_GotoXY (50,(i*12));
			SSD1306_Puts ("D", &Font_7x10, 1);
			bufferDisplay.numberRxExploderOnDetonator[i] = 0;
		}
		if (bufferDisplay.numberRxExploderVolt[i] >6){
			SSD1306_GotoXY (60,(i*12));
			sprintf(buffer, "%d", bufferDisplay.numberRxExploderVolt[i]);
			SSD1306_Puts (buffer, &Font_7x10, 1);
			//SSD1306_GotoXY (67,i*12+12);
			//char* til[1];
			//til[0] = ',';
			SSD1306_Puts (",", &Font_7x10, 1);
			//SSD1306_GotoXY (72,i*12+12);
			sprintf(buffer, "%d", bufferDisplay.numberRxExploderVolt[i]);
			SSD1306_Puts (buffer, &Font_7x10, 1);
			//SSD1306_GotoXY (79,i*12+12);
			//char* charVolt = " volt";
			SSD1306_Puts (" volt", &Font_7x10, 1);
			bufferDisplay.numberRxExploderVolt[i] = 0;
		}

	}

	SSD1306_GotoXY (0,0);
	SSD1306_Puts ("Menu", &Font_7x10, 1);

if(bufferDisplay.bfArm == 1){
	SSD1306_GotoXY (30,0);
	SSD1306_Puts ("ARM", &Font_7x10, 1);
}

	SSD1306_GotoXY (45,0);
	sprintf(buffer, "%d", bufferDisplay.bfPowerPult);
	SSD1306_Puts ("Volt: ", &Font_7x10, 1);
	SSD1306_Puts (buffer, &Font_7x10, 1);
	SSD1306_Puts (",", &Font_7x10, 1);
	sprintf(buffer, "%d", bufferDisplay.bfPowerPult);
	SSD1306_Puts (buffer, &Font_7x10, 1);

	SSD1306_UpdateScreen();
	HAL_Delay(50);

/*
		        SSD1306_GotoXY (0,0);
		        SSD1306_Puts ("NIZAR", &Font_7x10, 1);
		        SSD1306_GotoXY (0, 30);
		        SSD1306_Puts ("MOHIDEEN", &Font_11x18, 1);
		        SSD1306_UpdateScreen();
		        HAL_Delay (1000);
		        */
}

//============================================================================================================================//
//=================================================outDisplayWakeTx===========================================================//
//============================================================================================================================//
/**
 * @brief Вывод информации на дисплей в режиме ожидания передачи (WakeTx).
 */
void outDisplayWakeTx(void) {
    char buffer[20];

    // Установка состояния прерываний в NoUsaNoExtern (запрет всех прерываний)
    setInterruptState(NoUsaNoExtern);

    #ifdef Debug
        // Вывод отладочной информации
        printf("outDisplayWakeTx.\n");
    #endif

    // Очистка экрана
    SSD1306_Fill(0);

    // Установка курсора в начало экрана
    SSD1306_GotoXY(0, 0);

    // Вывод информации в зависимости от состояния кнопок
    if (flagButtonDemolition == SET) {
        SSD1306_Puts(" DEMOLITION", &Font_11x18, 1);
        flagButtonDemolition = RESET;
    } else if (flagButtonTest == SET) {
        SSD1306_Puts("   TEST", &Font_11x18, 1);
        flagButtonTest = RESET;
    } else {
        SSD1306_Puts("   WAIT", &Font_11x18, 1);
        SSD1306_GotoXY(40, 30);
        // Преобразование числа времени в строку и вывод на дисплей
        sprintf(buffer, "%d", bufferDisplay.timeTxWake);
        SSD1306_Puts(buffer, &Font_11x18, 1);
    }

    // Обновление содержимого экрана
    SSD1306_UpdateScreen();

    // Задержка 1 миллисекунда
    HAL_Delay(1);
}


//============================================================================================================================//
//=================================================outBufferDisplay===========================================================//
//============================================================================================================================//
/**
 * @brief Обновление данных в структуре bufferDisplay.
 *
 * Функция считывает состояние кнопок, напряжение питания и обновляет соответствующие поля в структуре bufferDisplay.
 */
void outBufferDisplay(void) {
    // Считывание состояния кнопок и обновление соответствующих полей в структуре
    bufferDisplay.bfDetonator[0] = !HAL_GPIO_ReadPin(PIN_BUTTON_10_GPIO_Port, PIN_BUTTON_10_Pin);
    bufferDisplay.bfDetonator[1] = !HAL_GPIO_ReadPin(PIN_BUTTON_11_GPIO_Port, PIN_BUTTON_11_Pin);
    bufferDisplay.bfDetonator[2] = !HAL_GPIO_ReadPin(PIN_BUTTON_12_GPIO_Port, PIN_BUTTON_12_Pin);
    bufferDisplay.bfDetonator[3] = !HAL_GPIO_ReadPin(PIN_BUTTON_13_GPIO_Port, PIN_BUTTON_13_Pin);
    bufferDisplay.bfDetonator[4] = !HAL_GPIO_ReadPin(PIN_BUTTON_14_GPIO_Port, PIN_BUTTON_14_Pin);

    // Считывание состояния кнопки ARM и обновление соответствующего поля в структуре
    bufferDisplay.bfArm = !HAL_GPIO_ReadPin(PIN_BUTTON_ARM_GPIO_Port, PIN_BUTTON_ARM_Pin);

    // Считывание напряжения питания и обновление соответствующего поля в структуре
    readPower();
    bufferDisplay.bfPowerPult = adcVoltage[0];

    // Задержка 1 миллисекунда (возможно, ошибка в оригинальном коде)
    HAL_Delay(1);
}
