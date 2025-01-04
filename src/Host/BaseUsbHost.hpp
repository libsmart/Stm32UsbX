/*
 * SPDX-FileCopyrightText: 2025 Roland Rusch, easy-smart solution GmbH <roland.rusch@easy-smart.ch>
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <libsmart_config.hpp>
#include "Loggable.hpp"
#include "Nameable.hpp"

extern "C" {
#include "ux_api.h"
#include "ux_host_class_hid.h"
}

namespace Stm32UsbX {
    class BaseUsbHost : protected TX_EVENT_FLAGS_GROUP,
                        public Stm32ItmLogger::Loggable,
                        public Stm32Common::Nameable {
    public:
        BaseUsbHost() : BaseUsbHost(&Stm32ItmLogger::emptyLogger) { ; }

        explicit BaseUsbHost(const char *name)
            : BaseUsbHost(name, &Stm32ItmLogger::emptyLogger) { ; }

        explicit BaseUsbHost(Stm32ItmLogger::LoggerInterface *logger)
            : BaseUsbHost("Stm32UsbX::BaseUsbHost", logger) { ; }

        BaseUsbHost(const char *name, Stm32ItmLogger::LoggerInterface *logger)
            : TX_EVENT_FLAGS_GROUP(), Loggable(logger), Nameable(name) { ; }


        using system_change_function_cb = UINT (*)(ULONG, UX_HOST_CLASS *,VOID *);

        /**
         * @brief Initializes the USB host stack with a custom system change callback function.
         *
         * This method sets up the USB host stack, allowing the system to handle USB events.
         * The `system_change_function` parameter should point to a callback function that will be
         * triggered on system state changes. If the stack is already initialized, the method
         * returns success without reinitializing.
         *
         * @param system_change_function The callback function to handle system state changes.
         * @return Returns `UX_SUCCESS` on successful initialization or an error code otherwise.
         */
        virtual UINT initialize(system_change_function_cb system_change_function);

        virtual UINT endpoint_transfer_abort(UX_ENDPOINT *endpoint);

        virtual UINT class_get(UCHAR *class_name, UX_HOST_CLASS **class_struct);

        using class_entry_address_cb = UINT (*)(struct UX_HOST_CLASS_COMMAND_STRUCT *);

        virtual UINT class_register(UCHAR *class_name, class_entry_address_cb class_entry_address);

        // https://github.com/eclipse-threadx/rtos-docs/blob/main/rtos-docs/usbx/usbx-host-stack-4.md#ux_host_stack_class_instance_create
        virtual UINT class_instance_create(UX_HOST_CLASS *class_struct, VOID *class_instance) { return UX_ERROR; }

        // https://github.com/eclipse-threadx/rtos-docs/blob/main/rtos-docs/usbx/usbx-host-stack-4.md#ux_host_stack_class_instance_destroy
        virtual UINT class_instance_destroy(UX_HOST_CLASS *class_struct, VOID *class_instance) { return UX_ERROR; }

        // https://github.com/eclipse-threadx/rtos-docs/blob/main/rtos-docs/usbx/usbx-host-stack-4.md#ux_host_stack_class_instance_get
        virtual UINT class_instance_get(UX_HOST_CLASS *class_struct, UINT class_index, VOID **class_instance) {
            return UX_ERROR;
        }

        // https://github.com/eclipse-threadx/rtos-docs/blob/main/rtos-docs/usbx/usbx-host-stack-4.md#ux_host_stack_device_configuration_get
        virtual UINT device_configuration_get(UX_DEVICE *device, UINT configuration_index,
                                              UX_CONFIGURATION *configuration) { return UX_ERROR; }

        // https://github.com/eclipse-threadx/rtos-docs/blob/main/rtos-docs/usbx/usbx-host-stack-4.md#ux_host_stack_device_configuration_select
        virtual UINT device_configuration_select(UX_CONFIGURATION *configuration) { return UX_ERROR; }

        // https://github.com/eclipse-threadx/rtos-docs/blob/main/rtos-docs/usbx/usbx-host-stack-4.md#ux_host_stack_device_get
        virtual UINT device_get(UCHAR *device_name, UX_DEVICE **device_struct) { return UX_ERROR; }

        // https://github.com/eclipse-threadx/rtos-docs/blob/main/rtos-docs/usbx/usbx-host-stack-4.md#ux_host_stack_interface_endpoint_get
        virtual UINT
        interface_endpoint_get(UX_INTERFACE *interface, UINT endpoint_index, UX_ENDPOINT **endpoint_struct) {
            return UX_ERROR;
        }

        using hcd_function_cb = UINT (*)(struct UX_HCD_STRUCT *);
        /**
         * @brief Registers a Host Controller Driver (HCD) with the USB host stack.
         *
         * This method is used to register an HCD, allowing the USB host stack to interface
         * with a specific USB hardware controller. It accepts the name of the HCD, a callback function,
         * and two hardware-specific parameters required for initialization.
         *
         * @param hcd_name Pointer to a string representing the name of the HCD to register.
         * @param hcd_function Callback function for initializing the HCD.
         * @param hcd_param1 Hardware-specific parameter 1 for the HCD initialization.
         * @param hcd_param2 Hardware-specific parameter 2 for the HCD initialization.
         * @return Returns `UX_SUCCESS` on successful registration or an error code otherwise.
         */
        virtual UINT hcd_register(UCHAR *hcd_name, hcd_function_cb hcd_function, ULONG hcd_param1, ULONG hcd_param2);

        // https://github.com/eclipse-threadx/rtos-docs/blob/main/rtos-docs/usbx/usbx-host-stack-4.md#ux_host_stack_configuration_interface_get
        virtual UINT configuration_interface_get(UX_CONFIGURATION *configuration, UINT interface_index,
                                                 UX_INTERFACE **interface_struct) { return UX_ERROR; }

        // https://github.com/eclipse-threadx/rtos-docs/blob/main/rtos-docs/usbx/usbx-host-stack-4.md#ux_host_stack_interface_setting_select
        virtual UINT interface_setting_select(UX_INTERFACE *interface, UINT setting_index) { return UX_ERROR; }

        // https://github.com/eclipse-threadx/rtos-docs/blob/main/rtos-docs/usbx/usbx-host-stack-4.md#ux_host_stack_transfer_request_abort
        virtual UINT transfer_request_abort(UX_TRANSFER *transfer) { return UX_ERROR; }

        // https://github.com/eclipse-threadx/rtos-docs/blob/main/rtos-docs/usbx/usbx-host-stack-4.md#ux_host_stack_transfer_request
        virtual UINT transfer_request(UX_TRANSFER *transfer) { return UX_ERROR; }

        // https://github.com/eclipse-threadx/rtos-docs/blob/main/rtos-docs/usbx/usbx-host-stack-5.md#ux_host_class_hid_client_register
        using hid_client_handler_cb = UINT (*)(UX_HOST_CLASS_HID_CLIENT_COMMAND_STRUCT *);
        /**
         * @brief Registers a HID client with a specified name and handler.
         *
         * This method allows the registration of a Human Interface Device (HID) client
         * to enable custom processing for specific HID devices. The `hid_client_name`
         * corresponds to the name of the HID client to register, and `hid_client_handler`
         * is the callback handler responsible for handling events from the registered HID
         * client.
         *
         * @param hid_client_name The name of the HID client to register.
         * @param hid_client_handler The callback function for handling events associated with the HID client.
         * @return Returns `UX_SUCCESS` if the registration is successful, or an error code otherwise.
         */
        [[deprecated("This should actually reside in an own class")]]
        virtual UINT class_hid_client_register(UCHAR *hid_client_name, hid_client_handler_cb hid_client_handler);

        virtual bool isInitialized();

    private:
        bool initialized = false;
    };
}
