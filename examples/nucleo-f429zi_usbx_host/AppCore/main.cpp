/*
 * SPDX-FileCopyrightText: 2025 Roland Rusch, easy-smart solution GmbH <roland.rusch@easy-smart.ch>
 * SPDX-License-Identifier: AGPL-3.0-only
 */

/**
 * This file holds the main setup() and loop() functions for C++ code.
 * If a RTOS is used, loop() is called in the main task and setup() is called before RTOS initialization.
 * @see App_ThreadX_Init() in Core/Src/app_threadx.c
 */

#include "main.hpp"
#include "globals.hpp"
#include "RunEvery.hpp"
#include "RunOnce.hpp"
#include "RunThreadOnce.hpp"
#include "Command/RegisterCommands.hpp"


/**
 * @brief Setup function.
 * This function is called once at the beginning of the program before ThreadX is initialized.
 * @see main() in Core/Src/main.c
 */
void setup() {
    Stm32ItmLogger::logger.setSeverity(Stm32ItmLogger::LoggerInterface::Severity::INFORMATIONAL)
            ->println("::setup()");

    dummyCpp = 0;
    dummyCandCpp = 0;

    ::AppCore::Command::RegisterCommands()();

    Serial3.begin();

    // print welcome message
    // Serial3.print(F("startup "));
    // Serial3.print(FIRMWARE_NAME);
    // Serial3.print(F(" v"));
    // Serial3.print(FIRMWARE_VERSION);
    // Serial3.print(F(" "));
    // Serial3.println(FIRMWARE_COPY);
    // Serial3.flush();
    delay(500);
    Serial3.print('\0');
    Serial3.flush();
}


void loopOnce() {
    Stm32ItmLogger::logger.setSeverity(Stm32ItmLogger::LoggerInterface::Severity::INFORMATIONAL)
            ->println("::loopOnce()");

#ifdef TX_ENABLE_STACK_CHECKING
    tx_thread_stack_error_notify(Stack_Error_Handler);
#endif
}

/**
 * @brief This function is the main loop that executes continuously.
 * The function is called inside the mainLoopThread().
 * @see mainLoopThread() in AZURE_RTOS/App/app_azure_rtos.c
 */
void loop() {
    Serial3.loop();

    dummyCpp++;
    dummyCandCpp++;


    static Stm32Common::RunEvery blinker;
    if (blinker.loop(300)) {
        HAL_GPIO_TogglePin(LED1_GRN_GPIO_Port, LED1_GRN_Pin);
    }
}


/**
 * @brief This function handles fatal errors.
 * @see Error_Handler() in Core/Src/main.c
 */
void errorHandler() {
    HAL_GPIO_WritePin(LED1_GRN_GPIO_Port, LED1_GRN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED2_BLU_GPIO_Port, LED2_BLU_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED3_RED_GPIO_Port, LED3_RED_Pin, GPIO_PIN_RESET);

    while (true) {
        for (uint32_t i = (SystemCoreClock / 10); i > 0; i--) { UNUSED(i); }
        HAL_GPIO_TogglePin(LED1_GRN_GPIO_Port, LED1_GRN_Pin);
        HAL_GPIO_TogglePin(LED2_BLU_GPIO_Port, LED2_BLU_Pin);
        HAL_GPIO_TogglePin(LED3_RED_GPIO_Port, LED3_RED_Pin);
    }
}


[[noreturn]] void Stack_Error_Handler(TX_THREAD *thread_ptr) {
    Logger.print("==> Stack_Error_Handler() called in thread ");
    Logger.println(thread_ptr->tx_thread_name);
    Logger.print("    Stack size: ");
    Logger.println(thread_ptr->tx_thread_stack_size);
    Error_Handler();
    __disable_irq();
    for (;;) { ; }
}
