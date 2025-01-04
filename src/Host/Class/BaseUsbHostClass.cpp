/*
 * SPDX-FileCopyrightText: 2025 Roland Rusch, easy-smart solution GmbH <roland.rusch@easy-smart.ch>
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "BaseUsbHostClass.hpp"

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

/*
UINT BaseUsbHostClass::endpoint_transfer_abort(UX_ENDPOINT *endpoint) {
    log(Stm32ItmLogger::LoggerInterface::Severity::DEBUGGING)
        ->printf("Stm32UsbX::BaseUsbHostClass[%s]::endpoint_transfer_abort(%p)\r\n", getName(), endpoint);

    if (isInitialized()) return UX_SUCCESS;

    // @see https://github.com/eclipse-threadx/rtos-docs/blob/main/rtos-docs/usbx/usbx-host-stack-4.md#ux_host_stack_endpoint_transfer_abort
    const auto ret = ux_host_stack_endpoint_transfer_abort(endpoint);

    if (ret != UX_SUCCESS) {
        constexpr char fmt[] = "Stm32UsbX::BaseUsbHostClass[%s]: ux_host_stack_endpoint_transfer_abort() = 0x%02x";
        LIBSMART_HANDLE_ERROR(fmt, getName(), ret);
    }

    initialized = true;

    return ret;

}
*/
