/*
 * SPDX-FileCopyrightText: 2025 Roland Rusch, easy-smart solution GmbH <roland.rusch@easy-smart.ch>
 * SPDX-License-Identifier: AGPL-3.0-only
 */

/**
 * This file holds the main setup() and loop() functions for C++ code.
 * If a RTOS is used, loop() is called in the main task and setup() is called before RTOS initialization.
 * @see App_ThreadX_Init() in Core/Src/app_threadx.c
 */

#include "main.hpp"
#include "globals.hpp"
#include "RunEvery.hpp"
#include "RunOnce.hpp"
#include "RunThreadEvery.hpp"
#include "RunThreadOnce.hpp"
#include "WaitOption.hpp"
#include "Command/RegisterCommands.hpp"
#include "EventFlags/EventFlags.hpp"

extern "C" {
#include "usb_otg.h"
#include "ux_hcd_stm32.h"
#include "ux_host_class_prolific.h"
#include "ux_host_class_gser.h"
#include "ux_host_class_cdc_acm.h"
#include "ux_host_class_hid.h"
#include "ux_host_class_hid_keyboard.h"
// #include "ux_host_class_storage.h"
}

/**
 * @brief Setup function.
 * This function is called once at the beginning of the program before ThreadX is initialized.
 * @see main() in Core/Src/main.c
 */
void setup() {
    Stm32ItmLogger::logger.setSeverity(Stm32ItmLogger::LoggerInterface::Severity::INFORMATIONAL)
            ->println("::setup()");

    dummyCpp = 0;
    dummyCandCpp = 0;

    ::AppCore::Command::RegisterCommands()();

    Serial3.begin();

    // print welcome message
    // Serial3.print(F("startup "));
    // Serial3.print(FIRMWARE_NAME);
    // Serial3.print(F(" v"));
    // Serial3.print(FIRMWARE_VERSION);
    // Serial3.print(F(" "));
    // Serial3.println(FIRMWARE_COPY);
    // Serial3.flush();
    delay(500);
    Serial3.print('\0');
    Serial3.flush();
}

#define NEW_RECEIVED_DATA    0x01
#define NEW_DATA_TO_SEND     0x02
#define REMOVED_CDC_INSTANCE 0x03
Stm32ThreadX::EventFlags ux_app_EventFlag;
UX_HOST_CLASS_CDC_ACM *cdc_acm{};
UX_HOST_CLASS_HID *hid_instance{};
UX_HOST_CLASS_HID_KEYBOARD *keyboard{};

/**
  * @brief  ux_host_event_callback
  *         This callback is invoked to notify application of instance changes.
  * @param  event: event code.
  * @param  current_class: Pointer to class.
  * @param  current_instance: Pointer to class instance.
  * @retval status
  */
extern "C" UINT ux_host_event_callback(ULONG event, UX_HOST_CLASS *current_class, VOID *current_instance) {
    UINT status = UX_SUCCESS;

    /* Get current Hid Client */
    auto *client = static_cast<UX_HOST_CLASS_HID_CLIENT *>(current_instance);

    switch (event) {
        case UX_DEVICE_INSERTION: {
            // Registered class created and activated a function instance
            Logger.printf("UX_DEVICE_INSERTION\r\n");

            // Get current CDC Class
            if (current_class->ux_host_class_entry_function == ux_host_class_cdc_acm_entry) {
                if (cdc_acm == UX_NULL) {
                    /* Get current CDC Instance */
                    cdc_acm = (UX_HOST_CLASS_CDC_ACM *) current_instance;

                    /* Check if this is CDC DATA instance */
                    if (cdc_acm->ux_host_class_cdc_acm_bulk_in_endpoint == UX_NULL) {
                        cdc_acm = nullptr;
                    } else {
                        Logger.printf("USB CDC Device Found\r\n");
                        Logger.printf("PID: %#x \r\n",
                                      (UINT) cdc_acm->ux_host_class_cdc_acm_device->ux_device_descriptor.idProduct);
                        Logger.printf("VID: %#x \r\n",
                                      (UINT) cdc_acm->ux_host_class_cdc_acm_device->ux_device_descriptor.idVendor);
                        Logger.printf("Data Interface initialized\r\n");
                    }
                }
            }

            // Get current Hid Class
            if (current_class->ux_host_class_entry_function == ux_host_class_hid_entry) {
                if (hid_instance == UX_NULL) {
                    /* Get current Hid Instance */
                    hid_instance = (UX_HOST_CLASS_HID *) current_instance;
                    Logger.printf("USB HID Device Found\r\n");
                    Logger.printf("PID: %#x \r\n",
                                  (UINT) hid_instance->ux_host_class_hid_device->ux_device_descriptor.idProduct);
                    Logger.printf("VID: %#x \r\n",
                                  (UINT) hid_instance->ux_host_class_hid_device->ux_device_descriptor.idVendor);
                }
            }


            /* USER CODE END UX_DEVICE_INSERTION */

            break;
        }

        case UX_DEVICE_REMOVAL: {
            // Function instance is deactivated
            Logger.printf("UX_DEVICE_REMOVAL\r\n");

            if ((VOID *) cdc_acm == current_instance) {
                /* Clear cdc instance */
                cdc_acm = nullptr;
                Logger.printf("USB CDC ACM Device Removal\r\n");

                /* Set NEW_RECEIVED_DATA flag */
                if (ux_app_EventFlag.set(REMOVED_CDC_INSTANCE, TX_OR) != TX_SUCCESS) {
                    Error_Handler();
                }
            }

            // Free HID Instance
            if ((VOID *) hid_instance == current_instance) {
                hid_instance = nullptr;
                Logger.printf("USB HID Device Removal\r\n");
            }

            break;
        }

        case UX_HID_CLIENT_INSERTION: {
            // HID class client (keyboard, mouse, remote control ...) is activated
            const auto instance = static_cast<UX_HOST_CLASS_HID_CLIENT *>(current_instance);
            LIBSMART_UNUSED(instance);

            Logger.printf("UX_HID_CLIENT_INSERTION\r\n");

            /* Check the HID_client if this is a HID keyboard device */
            if (client->ux_host_class_hid_client_handler == ux_host_class_hid_keyboard_entry) {
                /* Get current Hid Client */
                if (keyboard == nullptr) {
                    keyboard = static_cast<UX_HOST_CLASS_HID_KEYBOARD *>(client->
                        ux_host_class_hid_client_local_instance);

                    Logger.println("HID_Keyboard_Device Found");
                }
            }

            break;
        }

        case UX_HID_CLIENT_REMOVAL: {
            // HID class client is deactivated
            const auto instance = static_cast<UX_HOST_CLASS_HID_CLIENT *>(current_instance);
            LIBSMART_UNUSED(instance);

            Logger.printf("UX_HID_CLIENT_REMOVAL\r\n");

            /* Clear hid client local instance */
            if ((VOID *) keyboard == client->ux_host_class_hid_client_local_instance) {
                /* Clear hid keyboard instance */
                keyboard = nullptr;

                Logger.println("HID Client Keyboard Unplugged");
            }

            break;
        }

#if 0
        case UX_STORAGE_MEDIA_INSERTION: { // Storage media is ready (only available for No-FileX mode)
            const auto instance = static_cast<UX_HOST_CLASS_STORAGE_MEDIA *>(current_instance);
            LIBSMART_UNUSED(instance);
            Logger.printf("UX_STORAGE_MEDIA_INSERTION\r\n");
            break;
        }

        case UX_STORAGE_MEDIA_REMOVAL: { // Storage media is removed (only available for No-FileX mode)
            const auto instance = static_cast<UX_HOST_CLASS_STORAGE_MEDIA *>(current_instance);
            LIBSMART_UNUSED(instance);

            Logger.printf("UX_STORAGE_MEDIA_REMOVAL\r\n");
            break;
        }
#endif

        case UX_DEVICE_CONNECTION: {
            // Device is connected
            const auto instance = static_cast<UX_DEVICE *>(current_instance);
            LIBSMART_UNUSED(instance);

            Logger.printf("UX_DEVICE_CONNECTION\r\n");
#define COMPOSITE_DEVICE                0x000000EF
            if (_ux_system_host->ux_system_host_device_array->ux_device_descriptor.bDeviceClass == COMPOSITE_DEVICE) {
                // Composite interface type
                Logger.println("Composite Interface initialized");
            } else {
                // Simple interface type
                Logger.println("Simple Interface initialized");
            }

            Logger.printf("PID: %#x \r\n",
                          (UINT) _ux_system_host->ux_system_host_device_array->ux_device_descriptor.idProduct);
            Logger.printf("VID: %#x \r\n",
                          (UINT) _ux_system_host->ux_system_host_device_array->ux_device_descriptor.idVendor);
            Logger.printf("bDeviceClass: %#x \r\n",
                          (UINT) _ux_system_host->ux_system_host_device_array->ux_device_descriptor.bDeviceClass);

            if (keyboard != UX_NULL) {
                Logger.println("keyboard is ready...");
            }


            if (cdc_acm != UX_NULL) {
                Logger.println("CDC Data Interface initialized");
            }

            break;
        }

        case UX_DEVICE_DISCONNECTION: {
            // Device is disconnected
            const auto instance = static_cast<UX_DEVICE *>(current_instance);
            LIBSMART_UNUSED(instance);

            Logger.printf("UX_DEVICE_DISCONNECTION\r\n");
            break;
        }

        default:
            Logger.printf("Unknown event: %d\r\n", event);
            break;
    }

    return status;
}

/**
  * @brief ux_host_error_callback
  *         This callback is invoked to notify application of error changes.
  * @param  system_level: system level parameter.
  * @param  system_context: system context code.
  * @param  error_code: error event code.
  * @retval Status
  */
VOID ux_host_error_callback(UINT system_level, UINT system_context, UINT error_code) {
    switch (error_code) {
        case UX_DEVICE_ENUMERATION_FAILURE:
            Logger.printf("USB Device Enumeration Failure\r\n");
            break;

        case UX_NO_DEVICE_CONNECTED:
            Logger.printf("USB Device disconnected\r\n");
            break;

        default: {
            //TODO Why is this error constantly firing?
            if (error_code == 4 && system_context == UX_SYSTEM_CONTEXT_UTILITY && system_level ==
                UX_SYSTEM_LEVEL_THREAD) return;

            switch (system_level) {
                case UX_SYSTEM_LEVEL_INTERRUPT:
                    Logger.print("UX_SYSTEM_LEVEL_THREAD");
                    break;
                case UX_SYSTEM_LEVEL_THREAD:
                    Logger.print("UX_SYSTEM_LEVEL_THREAD");
                    break;
                default:
                    Logger.print("unknown SYSTEM_LEVEL");
            }

            Logger.print(" ");

            switch (system_context) {
                case UX_SYSTEM_CONTEXT_HCD:
                    Logger.print("UX_SYSTEM_CONTEXT_HCD");
                    break;

                case UX_SYSTEM_CONTEXT_DCD:
                    Logger.print("UX_SYSTEM_CONTEXT_DCD");
                    break;

                case UX_SYSTEM_CONTEXT_INIT:
                    Logger.print("UX_SYSTEM_CONTEXT_INIT");
                    break;
                case UX_SYSTEM_CONTEXT_ENUMERATOR:
                    Logger.print("UX_SYSTEM_CONTEXT_ENUMERATOR");
                    break;
                case UX_SYSTEM_CONTEXT_ROOT_HUB:
                    Logger.print("UX_SYSTEM_CONTEXT_ROOT_HUB");
                    break;
                case UX_SYSTEM_CONTEXT_HUB:
                    Logger.print("UX_SYSTEM_CONTEXT_HUB");
                    break;
                case UX_SYSTEM_CONTEXT_CLASS:
                    Logger.print("UX_SYSTEM_CONTEXT_CLASS");
                    break;
                case UX_SYSTEM_CONTEXT_UTILITY:
                    Logger.print("UX_SYSTEM_CONTEXT_UTILITY");
                    break;
                case UX_SYSTEM_CONTEXT_DEVICE_STACK:
                    Logger.print("UX_SYSTEM_CONTEXT_DEVICE_STACK");
                    break;
                case UX_SYSTEM_CONTEXT_HOST_STACK:
                    Logger.print("UX_SYSTEM_CONTEXT_HOST_STACK");
                    break;
                default: Logger.print("unknown SYSTEM_CONTEXT");
            }
            Logger.print(" ");
            Logger.printf("USB error_code=0x%02x (%d)\r\n", error_code, error_code);
            break;
        }
    }
}

/* UX Host CDC ACM Sending */
UCHAR UserTxBuffer[] = "USBX_STM32_Host_CDC_ACM \n";
ULONG tx_actual_length;
/* UX Host CDC ACM Reception */
#define APP_RX_DATA_SIZE             2048U
#define BLOCK_SIZE                   64U
UX_HOST_CLASS_CDC_ACM_RECEPTION cdc_acm_reception;
ULONG block_reception_count;
uint8_t block_reception_size[APP_RX_DATA_SIZE / BLOCK_SIZE];
uint16_t RxSzeIdx;
static UCHAR UserRxBuffer[APP_RX_DATA_SIZE];


/**
  * @brief  cdc_acm_reception_callback.
  *         This callback is invoked to notify reception transfer completion.
  * @param  cdc_acm: class instance.
  * @param  status: reception status.
  * @param  reception_buffer: reception buffer pointer.
  * @param  reception_size: block size reception.
  * @retval none
  */
VOID cdc_acm_reception_callback(struct UX_HOST_CLASS_CDC_ACM_STRUCT *cdc_acm,
                                UINT status, UCHAR *reception_buffer, ULONG reception_size) {
    /* Block reception count */
    block_reception_count++;

    /* Save block reception size */
    block_reception_size[RxSzeIdx] = reception_size;

    /* Move to the next block reception size */
    RxSzeIdx++;

    /* check if tail has reached end of user buffer */
    if (cdc_acm_reception.ux_host_class_cdc_acm_reception_data_tail + cdc_acm_reception.
        ux_host_class_cdc_acm_reception_block_size >=
        cdc_acm_reception.ux_host_class_cdc_acm_reception_data_buffer + cdc_acm_reception.
        ux_host_class_cdc_acm_reception_data_buffer_size) {
        /* Move back to the beginning  */
        cdc_acm_reception.ux_host_class_cdc_acm_reception_data_tail = cdc_acm_reception.
                ux_host_class_cdc_acm_reception_data_buffer;

        /* Reinitialize reception block size index */
        RxSzeIdx = 0U;
    } else {
        /* Program the tail to be after the current buffer */
        cdc_acm_reception.ux_host_class_cdc_acm_reception_data_tail += cdc_acm_reception.
                ux_host_class_cdc_acm_reception_block_size;
    }

    // Set NEW_RECEIVED_DATA flag
    if (ux_app_EventFlag.set(NEW_RECEIVED_DATA)) {
        Error_Handler();
    }
}


void loopOnce() {
    Stm32ItmLogger::logger.setSeverity(Stm32ItmLogger::LoggerInterface::Severity::INFORMATIONAL)
            ->println("::loopOnce()");

#ifdef TX_ENABLE_STACK_CHECKING
    tx_thread_stack_error_notify(Stack_Error_Handler);
#endif


    static uint8_t usbmem[40 * 1024];
    usbSystem.initialize(usbmem, sizeof(usbmem), nullptr, 0);
    usbSystem.error_callback_register(ux_host_error_callback);
    usbHost.initialize(ux_host_event_callback);
    usbHost.class_register(_ux_system_host_class_prolific_name, ux_host_class_prolific_entry);
    usbHost.class_register(_ux_system_host_class_gser_name, ux_host_class_gser_entry);
    usbHost.class_register(_ux_system_host_class_cdc_acm_name, ux_host_class_cdc_acm_entry);
    usbHost.class_register(_ux_system_host_class_hid_name, ux_host_class_hid_entry);
    usbHost.class_hid_client_register(_ux_system_host_class_hid_client_keyboard_name, ux_host_class_hid_keyboard_entry);

    ux_app_EventFlag.create();

    // Turn on VBUS for host mode
    HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_SET);

    MX_USB_OTG_FS_HCD_Init();
    usbHost.hcd_register(_ux_system_host_hcd_stm32_name, ux_hcd_stm32_initialize,
                         USB_OTG_HS_PERIPH_BASE, (ULONG) &hhcd_USB_OTG_FS);
    HAL_HCD_Start(&hhcd_USB_OTG_FS);

    static uint8_t usbHostHidKeyboardThreadStack[2 * 1024];
    static Stm32ThreadX::RunThreadEvery usbHostHidKeyboardThread(1, []() {
        // hid_keyboard_thread_entry

        ULONG keyboard_key;
        ULONG keyboard_state;

        for (;;) {
            // Start if the hid client is a keyboard and connected
            if ((keyboard != nullptr) &&
                (keyboard->ux_host_class_hid_keyboard_state == static_cast<ULONG>(UX_HOST_CLASS_INSTANCE_LIVE))) {
                // Get the keyboard key pressed
                volatile const auto ret = ux_host_class_hid_keyboard_key_get(keyboard, &keyboard_key, &keyboard_state);
                if (ret == UX_SUCCESS) {
                    // Print the key pressed
                    Logger.printf("%c", static_cast<char>(keyboard_key));
                    Serial3.print(static_cast<char>(keyboard_key));
                }
            }
            delay(10);
        }
    });
    usbHostHidKeyboardThread.createAndResumeThread(usbHostHidKeyboardThreadStack, sizeof(usbHostHidKeyboardThreadStack),
                                                   "usbHostHidKeyboardThread");


    static uint8_t usbHostCdcAcmTxThreadStack[2 * 1024];
    static Stm32ThreadX::RunThreadEvery usbHostCdcAcmTxThread(1, [](){
        UINT status;
        ULONG send_dataflag = 0;

        for (;;) {
            // Check the CDC class state
            if ((cdc_acm != nullptr) &&
                (cdc_acm->ux_host_class_cdc_acm_state == UX_HOST_CLASS_INSTANCE_LIVE)) {
                // Wait until the requested flag NEW_DATA_TO_SEND is received
                if (ux_app_EventFlag.get(
                        NEW_DATA_TO_SEND,
                        Stm32ThreadX::EventFlags::getOption_t::OR_CLEAR,
                        send_dataflag,
                        Stm32ThreadX::EventFlags::waitOption_t{
                            Stm32ThreadX::EventFlags::waitOption_t::WAIT_FOREVER
                        }) != TX_SUCCESS)
                    if (ux_app_EventFlag.get(NEW_DATA_TO_SEND, TX_OR_CLEAR,
                                           &send_dataflag, TX_WAIT_FOREVER) != TX_SUCCESS) {
                        Error_Handler();
                    }

                // Start sending data
                status = _ux_host_class_cdc_acm_write(cdc_acm, UserTxBuffer,
                                                      ux_utility_string_length_get(UserTxBuffer),
                                                      &tx_actual_length);
                if (status == UX_SUCCESS) {
                    Logger.println("Data sent successfully");
                } else {
                    Logger.println("Unable to send data");
                }
            } else {
                tx_thread_sleep(MS_TO_TICK(10));
            }
        }
    });
    usbHostCdcAcmTxThread.createAndResumeThread(usbHostCdcAcmTxThreadStack, sizeof(usbHostCdcAcmTxThreadStack),
                                                "usbHostCdcAcmTxThread");

    static uint8_t usbHostCdcAcmRxThreadStack[2 * 1024];
    static Stm32ThreadX::RunThreadEvery usbHostCdcAcmRxThread(1, []() {
        UINT status;
        ULONG receive_dataflag = 0;
        UCHAR *read_data_pointer = nullptr;
        ULONG read_block_count = 0;
        uint16_t read_data_block_count = 0;
        uint16_t count = 0;

        for (;;) {
            // Check the CDC class state
            if ((cdc_acm != nullptr) &&
                (cdc_acm->ux_host_class_cdc_acm_state == UX_HOST_CLASS_INSTANCE_LIVE)) {
                if (cdc_acm_reception.ux_host_class_cdc_acm_reception_state !=
                    UX_HOST_CLASS_CDC_ACM_RECEPTION_STATE_STARTED) {
                    /* Get a pointer to the USB user buffer reception */
                    read_data_pointer = UserRxBuffer;

                    /* Set the block size parameter reception */
                    cdc_acm_reception.ux_host_class_cdc_acm_reception_block_size = BLOCK_SIZE;

                    /* Set the buffer for reception */
                    cdc_acm_reception.ux_host_class_cdc_acm_reception_data_buffer = (UCHAR *) UserRxBuffer;

                    /* Set the size of the data reception buffer */
                    cdc_acm_reception.ux_host_class_cdc_acm_reception_data_buffer_size = APP_RX_DATA_SIZE;

                    /* Set the callback for each reception transfer completion */
                    cdc_acm_reception.ux_host_class_cdc_acm_reception_callback = cdc_acm_reception_callback;

                    /* Start reception */
                    status = ux_host_class_cdc_acm_reception_start(cdc_acm, &cdc_acm_reception);

                    /* Check status to starting data reception */
                    if (status == UX_SUCCESS) {
                        Logger.println("Ready to receive data");
                    } else {
                        Logger.println("Unable to start reception");
                        delay(10);
                    }
                } else if (cdc_acm_reception.ux_host_class_cdc_acm_reception_state ==
                           UX_HOST_CLASS_CDC_ACM_RECEPTION_STATE_STARTED) {
                    /* Wait until the requested flag NEW_RECEIVED_DATA is received */
                    if (ux_app_EventFlag.get(NEW_RECEIVED_DATA, TX_OR_CLEAR,
                                           &receive_dataflag, TX_WAIT_FOREVER) != TX_SUCCESS) {
                        Error_Handler();
                    }

                    while ((read_block_count < block_reception_count) && (cdc_acm != NULL)) {
                        /* Check if read_data_pointer reached end of user buffer */
                        if (read_data_pointer >= cdc_acm_reception.ux_host_class_cdc_acm_reception_data_buffer +
                            cdc_acm_reception.ux_host_class_cdc_acm_reception_data_buffer_size) {
                            read_data_pointer = cdc_acm_reception.ux_host_class_cdc_acm_reception_data_buffer;

                            /* Reinitialize block reception size index */
                            read_data_block_count = 0;
                        }

                        /* Display the received data */
                        for (count = 0; count < block_reception_size[read_data_block_count]; count++) {
                            Logger.printf("%c", *read_data_pointer);
                            read_data_pointer++;
                        }

                        /* Move to next block reception */
                        read_block_count++;

                        /* Move to the next block reception buffer */
                        read_data_pointer += (BLOCK_SIZE - count);

                        /* Move to the next block reception size */
                        read_data_block_count++;
                    }
                }
            } else {
                delay(10);
            }
        }
    });
    usbHostCdcAcmRxThread.createAndResumeThread(usbHostCdcAcmRxThreadStack, sizeof(usbHostCdcAcmRxThreadStack),
                                                "usbHostCdcAcmRxThread");


}

/**
 * @brief This function is the main loop that executes continuously.
 * The function is called inside the mainLoopThread().
 * @see mainLoopThread() in AZURE_RTOS/App/app_azure_rtos.c
 */
void loop() {
    Serial3.loop();

    dummyCpp++;
    dummyCandCpp++;


    static Stm32Common::RunEvery blinker;
    if (blinker.loop(300)) {
        HAL_GPIO_TogglePin(LED1_GRN_GPIO_Port, LED1_GRN_Pin);
    }
}


/**
 * @brief This function handles fatal errors.
 * @see Error_Handler() in Core/Src/main.c
 */
void errorHandler() {
    HAL_GPIO_WritePin(LED1_GRN_GPIO_Port, LED1_GRN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED2_BLU_GPIO_Port, LED2_BLU_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED3_RED_GPIO_Port, LED3_RED_Pin, GPIO_PIN_RESET);

    while (true) {
        for (uint32_t i = (SystemCoreClock / 10); i > 0; i--) { UNUSED(i); }
        HAL_GPIO_TogglePin(LED1_GRN_GPIO_Port, LED1_GRN_Pin);
        HAL_GPIO_TogglePin(LED2_BLU_GPIO_Port, LED2_BLU_Pin);
        HAL_GPIO_TogglePin(LED3_RED_GPIO_Port, LED3_RED_Pin);
    }
}


[[noreturn]] void Stack_Error_Handler(TX_THREAD *thread_ptr) {
    Logger.print("==> Stack_Error_Handler() called in thread ");
    Logger.println(thread_ptr->tx_thread_name);
    Logger.print("    Stack size: ");
    Logger.println(thread_ptr->tx_thread_stack_size);
    Error_Handler();
    __disable_irq();
    for (;;) { ; }
}
