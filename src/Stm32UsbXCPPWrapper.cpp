/*
 * SPDX-FileCopyrightText: 2024 Roland Rusch, easy-smart solution GmbH <roland.rusch@easy-smart.ch>
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "Stm32UsbXCPPWrapper.hpp"
#include "Stm32UsbX.hpp"

UINT Stm32UsbX_setupThread(TX_BYTE_POOL *byte_pool) {
    return Stm32UsbX::setupThread(byte_pool);
}
