/*
 * SPDX-FileCopyrightText: 2025 Roland Rusch, easy-smart solution GmbH <roland.rusch@easy-smart.ch>
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <libsmart_config.hpp>
#include "Loggable.hpp"
#include "Nameable.hpp"
#include "ux_api.h"

namespace Stm32UsbX {
    class BaseUsbSystem : protected TX_EVENT_FLAGS_GROUP,
                          public Stm32ItmLogger::Loggable,
                          public Stm32Common::Nameable {
    public:
        BaseUsbSystem() : BaseUsbSystem(&Stm32ItmLogger::emptyLogger) { ; }

        explicit BaseUsbSystem(const char *name)
            : BaseUsbSystem(name, &Stm32ItmLogger::emptyLogger) { ; }

        explicit BaseUsbSystem(Stm32ItmLogger::LoggerInterface *logger)
            : BaseUsbSystem("Stm32UsbX::BaseUsbSystem", logger) { ; }

        BaseUsbSystem(const char *name, Stm32ItmLogger::LoggerInterface *logger)
            : TX_EVENT_FLAGS_GROUP(), Loggable(logger), Nameable(name) { ; }


        virtual UINT initialize(VOID *regular_memory_pool_start, ULONG regular_memory_size,
                                VOID *cache_safe_memory_pool_start, ULONG cache_safe_memory_size);


        virtual UINT uninitialize();

#if !defined(UX_HOST_SIDE_ONLY)
        virtual UINT tasks_run();
#endif


        using error_callback_cb = VOID (*)(UINT system_level, UINT system_context, UINT error_code);
        virtual VOID error_callback_register(error_callback_cb error_callback);

        virtual bool isInitialized();

    private:
        bool initialized = false;
    };
}
