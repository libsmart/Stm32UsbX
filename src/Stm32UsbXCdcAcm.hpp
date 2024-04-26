/*
 * SPDX-FileCopyrightText: 2024 Roland Rusch, easy-smart solution GmbH <roland.rusch@easy-smart.ch>
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef LIBSMART_STM32USBX_STM32USBXCDCACM_HPP
#define LIBSMART_STM32USBX_STM32USBXCDCACM_HPP

#include <limits.h>

#include "main.h"
#include "tx_api.h"
#include "ux_api.h"
#include "Stm32ThreadxThread.hpp"
#include "StringBuffer.hpp"
#include "ux_device_descriptors.h"
#include "ux_device_class_cdc_acm.h"

namespace Stm32UsbX {

    class Stm32UsbXCdcAcm : public Stm32ThreadxThread::thread, public Stm32Common::Stream {
    public:
        Stm32UsbXCdcAcm(void *pstack,
                        uint32_t stackSize,
                        const priority &prio,
                        const char *name) :
                thread(pstack,
                       stackSize,
                       &Stm32ThreadxThread::BOUNCE(Stm32UsbXCdcAcm, workerThread),
                       (ULONG) this,
                       prio,
                       name) {};


        void loop() {
            // if(rxBuffer.getLength() > 0) {
                // Debugger_log(DBG, "received '%.*s'", rxBuffer.getLength(), rxBuffer.getReadPointer());
                // rxBuffer.clear();
            // }
        }


        size_t getWriteBuffer(uint8_t *&buffer) override {
            buffer=txBuffer.getWritePointer();
            return txBuffer.getRemainingSpace();
        }

        size_t setWrittenBytes(size_t size) override {
            return txBuffer.add(size);
        }

        size_t write(uint8_t data) override {
            return txBuffer.write(data);
        }

        int availableForWrite() override {
            return static_cast<int>(std::min(txBuffer.getRemainingSpace(), static_cast<size_t>(INT_MAX)));
        }

        void flush() override {

        }

        int available() override {
            return static_cast<int>(std::min(rxBuffer.getLength(), static_cast<size_t>(INT_MAX)));
        }

        int read() override {
            return rxBuffer.read();
        }

        int peek() override {
            return rxBuffer.peek();
        }


        static VOID USBD_CDC_ACM_Activate(VOID *cdc_acm_instance);
        static VOID USBD_CDC_ACM_Deactivate(VOID *cdc_acm_instance);
        static VOID USBD_CDC_ACM_ParameterChange(VOID *cdc_acm_instance);

        static UINT _cdc_acm_write_callback(UX_SLAVE_CLASS_CDC_ACM *cdc_acm, UINT status, ULONG length);
        static UINT _cdc_acm_read_callback(UX_SLAVE_CLASS_CDC_ACM *cdc_acm, UINT status, UCHAR *data_pointer, ULONG length);




    protected:
        [[noreturn]] VOID workerThread();

        TX_EVENT_FLAGS_GROUP flags;
        enum class eventFlags {
            TX_DATA_READY = 1,
            RX_DATA_READY = 2,
            READ_DONE = 4,
            WRITE_DONE = 8,
            WRITE_LOCK = 16
        };


    private:
        UX_SLAVE_CLASS_CDC_ACM_PARAMETER cdc_acm_parameter;
        ULONG cdc_acm_interface_number;
        ULONG cdc_acm_configuration_number;
        static UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARAMETER CDC_VCP_LineCoding;
        static Stm32UsbXCdcAcm *self;


        class txBufferClass final: public Stm32Common::StringBuffer<256> {
        protected:
            void onEmpty() override {
                tx_event_flags_set(&self->flags, ~static_cast<ULONG>(eventFlags::TX_DATA_READY), TX_AND);
            }

            void onNonEmpty() override {
                tx_event_flags_set(&self->flags, static_cast<ULONG>(eventFlags::TX_DATA_READY), TX_OR);
            }
        } txBuffer;

        class rxBufferClass final: public Stm32Common::StringBuffer<256> {
        protected:
            void onEmpty() override {
                tx_event_flags_set(&self->flags, ~static_cast<ULONG>(eventFlags::RX_DATA_READY), TX_AND);
            }

            void onNonEmpty() override {
                tx_event_flags_set(&self->flags, static_cast<ULONG>(eventFlags::RX_DATA_READY), TX_OR);
            }
        } rxBuffer;

        // Stm32Common::StringBuffer<256> rxBuffer;
        // Stm32Common::StringBuffer<256> txBuffer;
    };

}

#endif //LIBSMART_STM32USBX_STM32USBXCDCACM_HPP
