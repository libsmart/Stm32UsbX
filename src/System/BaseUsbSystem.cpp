/*
 * SPDX-FileCopyrightText: 2025 Roland Rusch, easy-smart solution GmbH <roland.rusch@easy-smart.ch>
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "BaseUsbSystem.hpp"

using namespace Stm32UsbX;

UINT BaseUsbSystem::initialize(void *regular_memory_pool_start, ULONG regular_memory_size,
                               void *cache_safe_memory_pool_start, ULONG cache_safe_memory_size) {
    log(Stm32ItmLogger::LoggerInterface::Severity::DEBUGGING)
            ->printf("Stm32UsbX::BaseUsbSystem[%s]::initialize(%p, %d, %p, %d)\r\n", getName(),
                     regular_memory_pool_start, regular_memory_size, cache_safe_memory_pool_start,
                     cache_safe_memory_size);

    if (isInitialized()) return UX_SUCCESS;

    // @see https://github.com/eclipse-threadx/rtos-docs/blob/main/rtos-docs/usbx/usbx-host-stack-2.md#initialization-of-usbx-resources
    const auto ret = ux_system_initialize(
        regular_memory_pool_start,
        regular_memory_size,
        cache_safe_memory_pool_start,
        cache_safe_memory_size);

    if (ret != UX_SUCCESS) {
        constexpr char fmt[] = "Stm32UsbX::BaseUsbSystem[%s]: ux_system_initialize() = 0x%02x";
        LIBSMART_HANDLE_ERROR(fmt, getName(), ret);
    }

    initialized = true;

    return ret;
}

UINT BaseUsbSystem::uninitialize() {
    log(Stm32ItmLogger::LoggerInterface::Severity::DEBUGGING)
            ->printf("Stm32UsbX::BaseUsbSystem[%s]::uninitialize(%p, %d, %p, %d)\r\n", getName());

    // @see https://github.com/eclipse-threadx/rtos-docs/blob/main/rtos-docs/usbx/usbx-host-stack-2.md#uninitialization-of-usbx-resources
    const auto ret = ux_system_uninitialize();

    initialized = false;

    if (ret != UX_SUCCESS) {
        constexpr char fmt[] = "Stm32UsbX::BaseUsbSystem[%s]: ux_system_uninitialize() = 0x%02x";
        LIBSMART_HANDLE_ERROR(fmt, getName(), ret);
    }
    return ret;
}

void BaseUsbSystem::error_callback_register(error_callback_cb error_callback) {
    log(Stm32ItmLogger::LoggerInterface::Severity::DEBUGGING)
        ->printf("Stm32UsbX::BaseUsbSystem[%s]::error_callback_register(%p)\r\n", getName(), error_callback);

    ux_utility_error_callback_register(error_callback);
}

#if !defined(UX_HOST_SIDE_ONLY) && false
UINT BaseUsbSystem::tasks_run() {
    log(Stm32ItmLogger::LoggerInterface::Severity::DEBUGGING)
            ->printf("Stm32UsbX::BaseUsbSystem[%s]::tasks_run(%p, %d, %p, %d)\r\n", getName());

    const auto ret = ux_system_tasks_run();

    if (ret != UX_SUCCESS) {
        constexpr char fmt[] = "Stm32UsbX::BaseUsbSystem[%s]: ux_system_tasks_run() = 0x%02x";
        LIBSMART_HANDLE_ERROR(fmt, getName(), ret);
    }
    return ret;
}
#endif

bool BaseUsbSystem::isInitialized() {
    return initialized;
}
