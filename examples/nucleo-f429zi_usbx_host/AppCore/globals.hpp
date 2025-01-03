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
#include "Stm32ItmLogger.hpp"

#include "usart.h"
#include "Driver/Stm32HalUartItDriver.hpp"
#include "ezShell/Shell.hpp"


#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t dummyCpp;

inline Stm32ItmLogger::Stm32ItmLogger &Logger = Stm32ItmLogger::logger;

inline Stm32Common::StreamSession::Manager<Stm32Shell::ezShell::Shell, 1> microrlStreamSessionManager(&Logger);
inline Stm32Serial::Stm32HalUartItDriver uart3Driver(&huart3, "uart3Driver");
inline Stm32Serial::Stm32Serial Serial3(&uart3Driver, &microrlStreamSessionManager);



#ifdef __cplusplus
}
#endif

#endif
