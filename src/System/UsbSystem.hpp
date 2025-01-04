/*
 * SPDX-FileCopyrightText: 2025 Roland Rusch, easy-smart solution GmbH <roland.rusch@easy-smart.ch>
 * SPDX-License-Identifier: AGPL-3.0-only
 */

#pragma once
#include "BaseUsbSystem.hpp"

namespace Stm32UsbX {
    class UsbSystem : public BaseUsbSystem {
    public:
        UsbSystem() = default;

        explicit UsbSystem(const char *name)
            : BaseUsbSystem(name) {
        }

        explicit UsbSystem(Stm32ItmLogger::LoggerInterface *logger)
            : BaseUsbSystem(logger) {
        }

        UsbSystem(const char *name, Stm32ItmLogger::LoggerInterface *logger)
            : BaseUsbSystem(name, logger) {
        }
    };
}
