/*
 * SPDX-FileCopyrightText: 2025 Roland Rusch, easy-smart solution GmbH <roland.rusch@easy-smart.ch>
 * SPDX-License-Identifier: AGPL-3.0-only
 */

#include "UsbHost.hpp"

UINT Stm32UsbX::UsbHost::initialize() {
    return BaseUsbHost::initialize(nullptr);
}
