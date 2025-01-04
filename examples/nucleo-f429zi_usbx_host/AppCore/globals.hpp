/*
 * SPDX-FileCopyrightText: 2025 Roland Rusch, easy-smart solution GmbH <roland.rusch@easy-smart.ch>
 * SPDX-License-Identifier: AGPL-3.0-only
 */

/**
 * This file holds exports for the global variables, defined in globals.cpp.
 * @see globals.cpp
 */

#ifndef APPCORE_GLOBALS_HPP
#define APPCORE_GLOBALS_HPP

#include "globals.h"
#include <cstdint>
#include <Stm32UsbX.hpp>
#include <Host/UsbHost.hpp>
#include <System/UsbSystem.hpp>

#include "Stm32ItmLogger.hpp"

#include "usart.h"
#include "Driver/Stm32HalUartItDriver.hpp"
#include "EventFlags/EventFlags.hpp"
#include "ezShell/Shell.hpp"


#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t dummyCpp;

inline Stm32ItmLogger::Stm32ItmLogger &Logger = Stm32ItmLogger::logger;

inline Stm32Common::StreamSession::Manager<Stm32Shell::ezShell::Shell, 1> microrlStreamSessionManager(&Logger);
inline Stm32Serial::Stm32HalUartItDriver uart3Driver(&huart3, "uart3Driver");
inline Stm32Serial::Stm32Serial Serial3(&uart3Driver, &microrlStreamSessionManager);

inline Stm32UsbX::UsbSystem usbSystem("USB", &Logger);
inline Stm32UsbX::UsbHost usbHost("USB-Host", &Logger);
inline Stm32ThreadX::EventFlags hcdFlags("hcdFlags");

#ifdef __cplusplus
}
#endif

#endif
