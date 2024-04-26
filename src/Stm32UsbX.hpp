/*
 * SPDX-FileCopyrightText: 2024 Roland Rusch, easy-smart solution GmbH <roland.rusch@easy-smart.ch>
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef LIBSMART_STM32USBX_STM32USBX_HPP
#define LIBSMART_STM32USBX_STM32USBX_HPP

#include "tx_api.h"
#include "ux_api.h"
#include "Stm32UsbXCdcAcm.hpp"


namespace Stm32UsbX {
    /**
     * @brief Sets up the UsbX thread.
     * @param byte_pool A pointer to a TX_BYTE_POOL struct to be used by the thread (size=UX_DEVICE_APP_MEM_POOL_SIZE).
     * @return The return value of the setup thread function call.
     */
    UINT setupThread(TX_BYTE_POOL *byte_pool);
    extern Stm32UsbXCdcAcm *cdcAcmWorker;

}

#endif //LIBSMART_STM32USBX_STM32USBX_HPP
