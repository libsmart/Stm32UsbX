/*
 * SPDX-FileCopyrightText: 2025 Roland Rusch, easy-smart solution GmbH <roland.rusch@easy-smart.ch>
 * SPDX-License-Identifier: AGPL-3.0-only
 */

#pragma once
#include "BaseUsbHost.hpp"

namespace Stm32UsbX {
    class UsbHost : public BaseUsbHost {
    public:
        UsbHost() = default;

        explicit UsbHost(const char *name)
            : BaseUsbHost(name) {
        }

        explicit UsbHost(Stm32ItmLogger::LoggerInterface *logger)
            : BaseUsbHost(logger) {
        }

        UsbHost(const char *name, Stm32ItmLogger::LoggerInterface *logger)
            : BaseUsbHost(name, logger) {
        }


        virtual UINT initialize();

        using BaseUsbHost::initialize;
    };
}
