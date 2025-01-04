/*
 * SPDX-FileCopyrightText: 2025 Roland Rusch, easy-smart solution GmbH <roland.rusch@easy-smart.ch>
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <libsmart_config.hpp>
#include "Loggable.hpp"
#include "Nameable.hpp"

extern "C" {
#include "ux_api.h"
}

namespace Stm32UsbX {
    class BaseUsbHostClass : protected TX_EVENT_FLAGS_GROUP,
                        public Stm32ItmLogger::Loggable,
                        public Stm32Common::Nameable {
    public:
        BaseUsbHostClass() : BaseUsbHostClass(&Stm32ItmLogger::emptyLogger) { ; }

        explicit BaseUsbHostClass(const char *name)
            : BaseUsbHostClass(name, &Stm32ItmLogger::emptyLogger) { ; }

        explicit BaseUsbHostClass(Stm32ItmLogger::LoggerInterface *logger)
            : BaseUsbHostClass("Stm32UsbX::BaseUsbHostClass", logger) { ; }

        BaseUsbHostClass(const char *name, Stm32ItmLogger::LoggerInterface *logger)
            : TX_EVENT_FLAGS_GROUP(), Loggable(logger), Nameable(name) { ; }


    };
}
