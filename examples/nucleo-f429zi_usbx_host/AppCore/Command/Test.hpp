/*
 * SPDX-FileCopyrightText: 2025 Roland Rusch, easy-smart solution GmbH <roland.rusch@easy-smart.ch>
 * SPDX-License-Identifier: AGPL-3.0-only
 */

#pragma once

#include "globals.hpp"
#include "Command/AbstractCommand.hpp"
#include "ezShell/Shell.hpp"

namespace AppCore::Command {
    class Test : public Stm32Shell::Command::AbstractCommand {
    public:
        Test() {
            Nameable::setName("test");
            isSync = true;
            setLogger(&Logger);
        }

        runReturn run() override {
            auto ret = AbstractCommand::run();

            try {
                if (std::strcmp(argv[1], "timestamp") == 0) {
                    ret = runTimestamp();
                }

                // if (std::strcmp(argv[1], "post") == 0) {
                //     ret = runPost();
                // }

                // if (std::strcmp(argv[1], "time") == 0) {
                //     ret = runTime();
                // }
            } catch (const std::exception &e) {
                ret = runReturn::ERROR;
                log()->setSeverity(Stm32ItmLogger::LoggerInterface::Severity::ERROR)
                        ->printf("ERROR: %s\r\n", e.what());
                out()->printf("ERROR: %s\r\n", e.what());
            }

            return ret;
        }

    private:
        runReturn runTimestamp() {
            Stm32ItmLogger::logger.setSeverity(Stm32ItmLogger::LoggerInterface::Severity::INFORMATIONAL)
                    ->println("Test::runTimestamp()");

            return runReturn::ERROR;
        }
    };
}
