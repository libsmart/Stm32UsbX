/*
 * SPDX-FileCopyrightText: 2025 Roland Rusch, easy-smart solution GmbH <roland.rusch@easy-smart.ch>
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "BaseUsbHost.hpp"

#include "ux_host_class_hid.h"

#if __EXCEPTIONS
#include <stdexcept>
#define LIBSMART_HANDLE_ERROR(fmt, ...)                                          \
do {                                                                    \
char buffer[snprintf(nullptr, 0, fmt, __VA_ARGS__) + 1]{};              \
snprintf(buffer, sizeof(buffer), fmt, __VA_ARGS__);                     \
log(Stm32ItmLogger::LoggerInterface::Severity::ERROR)->println(buffer); \
throw std::runtime_error(buffer);                                       \
} while (0);
#else
#define LIBSMART_HANDLE_ERROR(fmt, ...)                                          \
do {                                                                    \
char buffer[snprintf(nullptr, 0, fmt, __VA_ARGS__) + 1]{};              \
snprintf(buffer, sizeof(buffer), fmt, __VA_ARGS__);                     \
log(Stm32ItmLogger::LoggerInterface::Severity::ERROR)->println(buffer); \
return ret;                                                             \
} while (0);
#endif

using namespace Stm32UsbX;

UINT BaseUsbHost::initialize(system_change_function_cb system_change_function) {
    log(Stm32ItmLogger::LoggerInterface::Severity::DEBUGGING)
            ->printf("Stm32UsbX::BaseUsbHost[%s]::initialize(%p)\r\n", getName(), system_change_function);

    if (isInitialized()) return UX_SUCCESS;

    // @see https://github.com/eclipse-threadx/rtos-docs/blob/main/rtos-docs/usbx/usbx-host-stack-4.md#ux_host_stack_initialize
    const auto ret = ux_host_stack_initialize(system_change_function);

    if (ret != UX_SUCCESS) {
        constexpr char fmt[] = "Stm32UsbX::BaseUsbHost[%s]: ux_host_stack_initialize() = 0x%02x";
        LIBSMART_HANDLE_ERROR(fmt, getName(), ret);
    }

    initialized = true;

    return ret;
}

UINT BaseUsbHost::endpoint_transfer_abort(UX_ENDPOINT *endpoint) {
    log(Stm32ItmLogger::LoggerInterface::Severity::DEBUGGING)
            ->printf("Stm32UsbX::BaseUsbHost[%s]::endpoint_transfer_abort(%p)\r\n", getName(), endpoint);

    if (isInitialized()) return UX_SUCCESS;

    // @see https://github.com/eclipse-threadx/rtos-docs/blob/main/rtos-docs/usbx/usbx-host-stack-4.md#ux_host_stack_endpoint_transfer_abort
    const auto ret = ux_host_stack_endpoint_transfer_abort(endpoint);

    if (ret != UX_SUCCESS) {
        constexpr char fmt[] = "Stm32UsbX::BaseUsbHost[%s]: ux_host_stack_endpoint_transfer_abort() = 0x%02x";
        LIBSMART_HANDLE_ERROR(fmt, getName(), ret);
    }
    return ret;
}

UINT BaseUsbHost::class_get(UCHAR *class_name, UX_HOST_CLASS **class_struct) {
    log(Stm32ItmLogger::LoggerInterface::Severity::DEBUGGING)
            ->printf("Stm32UsbX::BaseUsbHost[%s]::class_get(%s, %p)\r\n", getName(), class_name, class_struct);

    // @see https://github.com/eclipse-threadx/rtos-docs/blob/main/rtos-docs/usbx/usbx-host-stack-4.md#ux_host_stack_class_get
    const auto ret = ux_host_stack_class_get(class_name, class_struct);

    if (ret != UX_SUCCESS) {
        constexpr char fmt[] = "Stm32UsbX::BaseUsbHost[%s]: ux_host_stack_class_get() = 0x%02x";
        LIBSMART_HANDLE_ERROR(fmt, getName(), ret);
    }
    return ret;
}

UINT BaseUsbHost::class_register(UCHAR *class_name, class_entry_address_cb class_entry_address) {
    log(Stm32ItmLogger::LoggerInterface::Severity::DEBUGGING)
            ->printf("Stm32UsbX::BaseUsbHost[%s]::class_register(%s, %p)\r\n", getName(), class_name,
                     class_entry_address);

    // @see https://github.com/eclipse-threadx/rtos-docs/blob/main/rtos-docs/usbx/usbx-host-stack-4.md#ux_host_stack_class_register
    const auto ret = ux_host_stack_class_register(class_name, class_entry_address);

    if (ret != UX_SUCCESS) {
        constexpr char fmt[] = "Stm32UsbX::BaseUsbHost[%s]: ux_host_stack_class_register() = 0x%02x";
        LIBSMART_HANDLE_ERROR(fmt, getName(), ret);
    }
    return ret;
}


UINT BaseUsbHost::hcd_register(UCHAR *hcd_name, hcd_function_cb hcd_function, ULONG hcd_param1, ULONG hcd_param2) {
    log(Stm32ItmLogger::LoggerInterface::Severity::DEBUGGING)
            ->printf("Stm32UsbX::BaseUsbHost[%s]::hcd_register(%s, %p, %d, %d)\r\n", getName(), hcd_name,
                     hcd_function, hcd_param1, hcd_param2);

    // https://github.com/eclipse-threadx/rtos-docs/blob/main/rtos-docs/usbx/usbx-host-stack-4.md#ux_host_stack_hcd_register
    const auto ret = ux_host_stack_hcd_register(hcd_name, hcd_function, hcd_param1, hcd_param2);

    if (ret != UX_SUCCESS) {
        constexpr char fmt[] = "Stm32UsbX::BaseUsbHost[%s]: ux_host_stack_hcd_register() = 0x%02x";
        LIBSMART_HANDLE_ERROR(fmt, getName(), ret);
    }
    return ret;
}

UINT BaseUsbHost::class_hid_client_register(UCHAR *hid_client_name, hid_client_handler_cb hid_client_handler) {
    log(Stm32ItmLogger::LoggerInterface::Severity::DEBUGGING)
            ->printf("Stm32UsbX::BaseUsbHost[%s]::class_hid_client_register(%s, %p)\r\n", getName(),
                     hid_client_name, hid_client_handler);

    // https://github.com/eclipse-threadx/rtos-docs/blob/main/rtos-docs/usbx/usbx-host-stack-4.md#ux_host_stack_hcd_register
    const auto ret = ux_host_class_hid_client_register(hid_client_name, hid_client_handler);

    if (ret != UX_SUCCESS) {
        constexpr char fmt[] = "Stm32UsbX::BaseUsbHost[%s]: ux_host_stack_hcd_register() = 0x%02x";
        LIBSMART_HANDLE_ERROR(fmt, getName(), ret);
    }
    return ret;
}

bool BaseUsbHost::isInitialized() {
    return initialized;
}
