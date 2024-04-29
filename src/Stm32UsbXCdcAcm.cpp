/*
 * SPDX-FileCopyrightText: 2024 Roland Rusch, easy-smart solution GmbH <roland.rusch@easy-smart.ch>
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "Stm32UsbXCdcAcm.hpp"
#include <sys/stat.h>

#include "Stm32GcodeRunner.hpp"
#include "Stm32UsbX.hpp"
#include "Stm32UsbXDeviceInit.h"


using namespace Stm32UsbX;

UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARAMETER Stm32UsbXCdcAcm::CDC_VCP_LineCoding =
{
    115200, /* baud rate */
    0x00, /* stop bits-1 */
    0x00, /* parity - none */
    0x08 /* nb. of bits 8 */
};

Stm32UsbXCdcAcm *Stm32UsbXCdcAcm::self = nullptr;

VOID Stm32UsbXCdcAcm::workerThread() {
    Debugger_log(DBG, "Stm32UsbX::Stm32UsbXCdcAcm::workerThread() starting");

    UINT ret = TX_SUCCESS;

    // /////////////////////////////////////////////////////////////////////////////
    // Initialize the cdc acm class parameters for the device
    cdc_acm_parameter.ux_slave_class_cdc_acm_instance_activate = USBD_CDC_ACM_Activate;
    // Deinitialize the cdc class parameters for the device
    cdc_acm_parameter.ux_slave_class_cdc_acm_instance_deactivate = USBD_CDC_ACM_Deactivate;
    // Manage the CDC class requests
    cdc_acm_parameter.ux_slave_class_cdc_acm_parameter_change = USBD_CDC_ACM_ParameterChange;


    // Get cdc acm configuration number
    cdc_acm_configuration_number = 1;
    // Find cdc acm interface number
    //  cdc_acm_interface_number = (USBD_CDCACM_EPINCMD_ADDR & 0x0FU) - 1;
    cdc_acm_interface_number = 0;

    // Initialize the device cdc acm class
    ret = ux_device_stack_class_register(_ux_system_slave_class_cdc_acm_name,
                                         ux_device_class_cdc_acm_entry,
                                         cdc_acm_configuration_number,
                                         cdc_acm_interface_number,
                                         (VOID *) &cdc_acm_parameter);

    if (ret != UX_SUCCESS) {
        Debugger_log(DBG, "%lu: ux_device_stack_class_register = 0x%02x", HAL_GetTick(), ret);
        Error_Handler();
    }

    tx_thread_sleep(UX_MS_TO_TICK(500));
    USBX_APP_Device_Init();


    tx_event_flags_create(&flags, const_cast<CHAR *>("USB CDC flags"));


    ULONG actual_length = 0;
    UX_SLAVE_DEVICE *device = {};
    UX_SLAVE_INTERFACE *data_interface = {};
    UX_SLAVE_CLASS_CDC_ACM *cdc_acm = {};
    device = &_ux_system_slave->ux_system_slave_device;

    for (;;) {
        // Check, if device is configured
        if (device->ux_slave_device_state != UX_DEVICE_CONFIGURED) {
            tx_thread_sleep(UX_MS_TO_TICK(10));
            continue;
        }

        data_interface = device->ux_slave_device_first_interface[0].ux_slave_interface_next_interface;
        // Compares two memory blocks ux_slave_class_name and _ux_system_slave_class_cdc_acm_name
        ret = ux_utility_memory_compare(data_interface->ux_slave_interface_class->ux_slave_class_name,
                                        _ux_system_slave_class_cdc_acm_name,
                                        ux_utility_string_length_get(_ux_system_slave_class_cdc_acm_name));
        if (ret != UX_SUCCESS) {
            cdc_acm = nullptr;
            Debugger_log(DBG, "%lu: RX: ux_utility_memory_compare = 0x%02x", HAL_GetTick(), ret);
            tx_thread_sleep(UX_MS_TO_TICK(1000));
            continue;
        }
        cdc_acm = (UX_SLAVE_CLASS_CDC_ACM *) data_interface->ux_slave_interface_class_instance;

        // Set the callback for read and write
        self = this;
        UX_SLAVE_CLASS_CDC_ACM_CALLBACK_PARAMETER callback_parameter;
        callback_parameter.ux_device_class_cdc_acm_parameter_read_callback = _cdc_acm_read_callback;
        callback_parameter.ux_device_class_cdc_acm_parameter_write_callback = _cdc_acm_write_callback;
        ret = ux_device_class_cdc_acm_ioctl(cdc_acm, UX_SLAVE_CLASS_CDC_ACM_IOCTL_TRANSMISSION_START,
                                            &callback_parameter);
        // ret = ux_device_class_cdc_acm_ioctl(cdc_acm, UX_SLAVE_CLASS_CDC_ACM_IOCTL_TRANSMISSION_STOP, nullptr);
        if (ret != UX_SUCCESS) {
            Debugger_log(DBG, "%lu: ux_device_class_cdc_acm_ioctl = 0x%02x", HAL_GetTick(), ret);
            tx_thread_sleep(UX_MS_TO_TICK(1000));
            continue;
        }

        for (;;) {
            ULONG actual_events;
            tx_event_flags_get(&flags,
                               static_cast<ULONG>(eventFlags::TX_DATA_READY)
                               | static_cast<ULONG>(eventFlags::RX_DATA_READY),
                               TX_OR, &actual_events, TX_WAIT_FOREVER);


            if (actual_events & static_cast<ULONG>(eventFlags::RX_DATA_READY)) {
                // Data waiting in rxBuffer, ready to be parsed

                Stm32Common::buf_size_signed_t posR = rxBuffer.findPos('\r');
                Stm32Common::buf_size_signed_t posN = rxBuffer.findPos('\n');
                const Stm32Common::buf_size_signed_t eolPos = std::max(posR, posN);
                if (eolPos > 0) {
                    Stm32GcodeRunner::AbstractCommand *cmd{};
                    const size_t cmdLen = std::min(posR < 0 ? SIZE_MAX : posR, posN < 0 ? SIZE_MAX : posN);
                    auto parserRet = Stm32GcodeRunner::parser.parseString(
                        cmd, reinterpret_cast<const char *>(rxBuffer.getReadPointer()), cmdLen);
                    rxBuffer.remove(eolPos + 1);

                    if (parserRet == Stm32GcodeRunner::Parser::parserReturn::OK) {
                        Debugger_log(DBG, "Found command: %s", cmd->getName());
                        Stm32GcodeRunner::CommandContext *cmdCtx{};
                        Stm32GcodeRunner::worker->createCommandContext(cmdCtx);
                        if(cmdCtx == nullptr) {
                            txBuffer.println("ERROR: Command buffer full");
                            continue;
                        }
                        cmdCtx->setCommand(cmd);

                        cmdCtx->registerOnWriteFunction([cmdCtx, this]() {
                            // Debugger_log(DBG, "onWriteFn()");
                            if (cmdCtx->outputLength() > 0) {
                                const auto result = cmdCtx->outputRead(
                                    reinterpret_cast<char *>(txBuffer.getWritePointer()),
                                    txBuffer.getRemainingSpace());
                                txBuffer.setWrittenBytes(result);
                            }
                        });

                        cmdCtx->registerOnCmdEndFunction([cmdCtx]() {
                            Debugger_log(DBG, "onCmdEndFn()");
                            Stm32GcodeRunner::worker->deleteCommandContext(cmdCtx);
                        });

                        Stm32GcodeRunner::worker->enqueueCommandContext(cmdCtx);
                    } else if(parserRet == Stm32GcodeRunner::Parser::parserReturn::UNKNOWN_COMMAND) {
                        txBuffer.println("ERROR: UNKNOWN COMMAND");
                    } else if(parserRet == Stm32GcodeRunner::Parser::parserReturn::GARBAGE_STRING) {
                        txBuffer.println("ERROR: UNKNOWN COMMAND");
                    } else {
                        // txBuffer.println("ERROR: ONLY WHITESPACE");
                    }
                }
            }


            if (actual_events & static_cast<ULONG>(eventFlags::TX_DATA_READY) && !(
                    actual_events & static_cast<ULONG>(eventFlags::WRITE_LOCK))) {
                // Data waiting in txBuffer but no write lock set

                // Set WRITE_LOCK
                tx_event_flags_set(&flags, static_cast<ULONG>(eventFlags::WRITE_LOCK), TX_OR);
                ret = ux_device_class_cdc_acm_write_with_callback(cdc_acm,
                                                                  const_cast<UCHAR *>(txBuffer.getReadPointer()),
                                                                  txBuffer.getLength());
                if (ret != UX_SUCCESS) {
                    Debugger_log(DBG, "%lu: ux_device_class_cdc_acm_write_with_callback = 0x%02x", HAL_GetTick(),
                                 ret);
                    tx_thread_sleep(UX_MS_TO_TICK(500));
                }
            }
            tx_thread_sleep(1);
        }

        tx_thread_sleep(1);
    }
}

UINT Stm32UsbXCdcAcm::_cdc_acm_write_callback(UX_SLAVE_CLASS_CDC_ACM *cdc_acm, UINT status, ULONG length) {
    // Debugger_log(DBG, "%lu: Stm32UsbX::Stm32UsbXCdcAcm::_cdc_acm_write_callback( status=0x%02x, length=%d )",
    // HAL_GetTick(), status, length);
    UINT ret = UX_SUCCESS;

    if (status == UX_SUCCESS) {
        ret = (self->txBuffer.remove(length) == length) ? UX_SUCCESS : UX_ERROR;
    }

    // Set WRITE_DONE flag
    tx_event_flags_set(&self->flags, static_cast<ULONG>(eventFlags::WRITE_DONE), TX_OR);
    // Remove WRITE_LOCK flag
    tx_event_flags_set(&self->flags, ~static_cast<ULONG>(eventFlags::WRITE_LOCK), TX_AND);

    return ret;
}


UINT Stm32UsbXCdcAcm::_cdc_acm_read_callback(UX_SLAVE_CLASS_CDC_ACM *cdc_acm, UINT status, UCHAR *data_pointer,
                                             ULONG length) {
    // Debugger_log(DBG, "%lu: Stm32UsbX::Stm32UsbXCdcAcm::_cdc_acm_read_callback( status=0x%02x, length=%d )",
    // HAL_GetTick(), status, length);
    UINT ret = UX_SUCCESS;

    if (status == UX_SUCCESS) {
        self->txBuffer.write(data_pointer, length); // remote echo
        ret = (self->rxBuffer.write(data_pointer, length) == length) ? UX_SUCCESS : UX_ERROR;
    }

    tx_event_flags_set(&self->flags, static_cast<ULONG>(eventFlags::READ_DONE), TX_OR);

    return ret;
}


/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @param  cdc Instance
  * @retval none
  */
VOID Stm32UsbXCdcAcm::USBD_CDC_ACM_Activate(VOID *cdc_acm_instance_void) {
    auto cdc_acm_instance = (UX_SLAVE_CLASS_CDC_ACM *) cdc_acm_instance_void;
    UINT ux_status = UX_SUCCESS;

    Debugger_log(DBG, "%lu: USBD_CDC_ACM_Activate()", HAL_GetTick());

    // Set device_class_cdc_acm with default parameters
    ux_status = ux_device_class_cdc_acm_ioctl(cdc_acm_instance,
                                              UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_LINE_CODING,
                                              &CDC_VCP_LineCoding);
    if (ux_status != UX_SUCCESS) {
        Debugger_log(DBG, "%lu: ux_device_class_cdc_acm_ioctl() = 0x%02x", HAL_GetTick(), ux_status);
        Error_Handler();
    }
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
VOID Stm32UsbXCdcAcm::USBD_CDC_ACM_Deactivate(VOID *cdc_acm_instance_void) {
    auto cdc_acm_instance = (UX_SLAVE_CLASS_CDC_ACM *) cdc_acm_instance_void;
    Debugger_log(DBG, "%lu: USBD_CDC_ACM_Deactivate()", HAL_GetTick());
}

/**
  * @brief  Manage the CDC class requests
  * @param  cdc Instance
  * @retval none
  */
VOID Stm32UsbXCdcAcm::USBD_CDC_ACM_ParameterChange(VOID *cdc_acm_instance_void) {
    auto cdc_acm_instance = (UX_SLAVE_CLASS_CDC_ACM *) cdc_acm_instance_void;
    UX_SLAVE_TRANSFER *transfer_request;
    UX_SLAVE_DEVICE *device;
    ULONG request;
    UINT ux_status = UX_SUCCESS;

    //  Debugger_log(DBG, "%lu: USBD_CDC_ACM_ParameterChange()", HAL_GetTick());

    // Get the pointer to the device
    device = &_ux_system_slave->ux_system_slave_device;

    // Get the pointer to the transfer request associated with the control endpoint
    transfer_request = &device->ux_slave_device_control_endpoint.ux_slave_endpoint_transfer_request;

    // Extract all necessary fields of the request
    request = *(transfer_request->ux_slave_transfer_request_setup + UX_SETUP_REQUEST);

    //  Debugger_log(DBG, "request = %02x", request);

    // Here we proceed only the standard request we know of at the device level
    switch (request) {
        // Set Line Coding Command
        case UX_SLAVE_CLASS_CDC_ACM_SET_LINE_CODING: {
            // Get the Line Coding parameters
            ux_status = ux_device_class_cdc_acm_ioctl(cdc_acm_instance,
                                                      UX_SLAVE_CLASS_CDC_ACM_IOCTL_GET_LINE_CODING,
                                                      &CDC_VCP_LineCoding);
            if (ux_status != UX_SUCCESS) Error_Handler();
            break;
        }

        // Get Line Coding Command
        case UX_SLAVE_CLASS_CDC_ACM_GET_LINE_CODING: {
            ux_status = ux_device_class_cdc_acm_ioctl(cdc_acm_instance,
                                                      UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_LINE_CODING,
                                                      &CDC_VCP_LineCoding);
            if (ux_status != UX_SUCCESS) Error_Handler();
            break;
        }

        // Set the control line state
        case UX_SLAVE_CLASS_CDC_ACM_SET_CONTROL_LINE_STATE:
        default:
            break;
    }

    //  Debugger_log(DBG, "OK");
}

