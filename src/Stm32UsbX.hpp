/*
 * SPDX-FileCopyrightText: 2024 Roland Rusch, easy-smart solution GmbH <roland.rusch@easy-smart.ch>
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef LIBSMART_STM32USBX_STM32USBX_HPP
#define LIBSMART_STM32USBX_STM32USBX_HPP

#include "tx_api.h"

namespace Stm32UsbX {
    UINT setupThread(TX_BYTE_POOL *byte_pool);
}

#endif //LIBSMART_STM32USBX_STM32USBX_HPP
