/*
 * SPDX-FileCopyrightText: 2025 Roland Rusch, easy-smart solution GmbH <roland.rusch@easy-smart.ch>
 * SPDX-License-Identifier: AGPL-3.0-only
 */

#include "InterruptHandlerCallback.hpp"
#include "globals.hpp"
#include "Helper.hpp"

#if defined(HAL_EXTI_MODULE_ENABLED)
/**
 * @brief This function is a callback invoked by the HAL_GPIO_EXTI_IRQHandler.
 *
 * It is intended to handle external GPIO interrupt events.
 *
 * @param GPIO_Pin The GPIO pin number that caused the interrupt.
 */
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

    switch (GPIO_Pin) {
        case USER_Btn_Pin: // EXTI15
            Logger.printf("HAL_GPIO_EXTI_Callback(%d): USER_Btn_Pin \n", GPIO_Pin);
            break;

        default:
            Logger.printf("HAL_GPIO_EXTI_Callback(%d): UNKNOWN \n", GPIO_Pin);
    }
}
#endif


#if defined(HAL_IWDG_MODULE_ENABLED)
/**
 * @brief Callback function invoked by the HAL when the Independent Watchdog
 *        (IWDG) triggers an early wakeup interrupt.
 *
 * @param hiwdg Pointer to the IWDG_HandleTypeDef structure that contains
 *        the configuration information for the IWDG.
 */
extern "C" void HAL_IWDG_EarlyWakeupCallback(IWDG_HandleTypeDef *hiwdg) {
    LIBSMART_UNUSED(hiwdg);
    // Logger.printf("HAL_IWDG_EarlyWakeupCallback()\n");
    errorHandler();
}
#endif


/**
 * @brief This function is a callback invoked by the HAL_SYSTICK_IRQHandler.
 *
 * It is intended to be implemented by the user to handle system tick interrupts.
 * By default, it is an empty function and can be modified by the user as needed.
 */
extern "C" void HAL_SYSTICK_Callback(void) {

}
