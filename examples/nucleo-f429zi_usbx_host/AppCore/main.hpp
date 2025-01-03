/*
 * SPDX-FileCopyrightText: 2025 Roland Rusch, easy-smart solution GmbH <roland.rusch@easy-smart.ch>
 * SPDX-License-Identifier: AGPL-3.0-only
 */

/**
 * This file holds the headers for main.cpp.
 * @see main.cpp
 */

#ifndef APPCORE_MAIN_HPP
#define APPCORE_MAIN_HPP

#include <main.h>

#ifdef __cplusplus
extern "C" {
#endif

    void setup();
    void errorHandler();
    void loop();

#ifdef __cplusplus
}
#endif

#endif
