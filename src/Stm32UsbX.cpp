/*
 * SPDX-FileCopyrightText: 2024 Roland Rusch, easy-smart solution GmbH <roland.rusch@easy-smart.ch>
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "Stm32UsbX.hpp"
#include "main.h"
#include "RTE_Components.h"
#include "ux_device_descriptors.h"
#include "Stm32UsbXCdcAcm.hpp"


#define USBX_SYSTEM_MEMORY_SIZE (11*1024)
#define USBX_CDC_ACM_THREAD_STACK_SIZE 1024



Stm32UsbX::Stm32UsbXCdcAcm *Stm32UsbX::cdcAcmWorker{};

UINT Stm32UsbX::setupThread(TX_BYTE_POOL *byte_pool) {
    Debugger_log(DBG, "Stm32UsbX::setupThread()");

    UINT ret = TX_SUCCESS;
    CHAR *pointer;
    UCHAR *device_framework_high_speed;
    ULONG device_framework_hs_length;
    UCHAR *device_framework_full_speed;
    ULONG device_framework_fs_length;
    UCHAR *string_framework;
    ULONG string_framework_length;
    UCHAR *language_id_framework;
    ULONG languge_id_framework_length;

#ifdef USBXDEVICE_ENABLED

    // Allocate USBX system memory from the UsbX Byte pool
    ret = tx_byte_allocate(byte_pool, (VOID **) &pointer, USBX_SYSTEM_MEMORY_SIZE, TX_NO_WAIT);
    if (ret != TX_SUCCESS) {
        Debugger_log(DBG, "%lu: tx_byte_allocate = 0x%02x", HAL_GetTick(), ret);
        Error_Handler();
    }

    // Initialize USBX system
    ret = ux_system_initialize(pointer, USBX_SYSTEM_MEMORY_SIZE, UX_NULL, 0);
    if (ret != UX_SUCCESS) {
        Debugger_log(DBG, "%lu: ux_system_initialize = %d", HAL_GetTick(), ret);
        Error_Handler();
    }

    // Get_Device_Framework_High_Speed and get the length
    device_framework_high_speed = USBD_Get_Device_Framework_Speed(USBD_HIGH_SPEED,
                                                                  &device_framework_hs_length);

    // Get_Device_Framework_Full_Speed and get the length
    device_framework_full_speed = USBD_Get_Device_Framework_Speed(USBD_FULL_SPEED,
                                                                  &device_framework_fs_length);

    // Get_String_Framework and get the length
    string_framework = USBD_Get_String_Framework(&string_framework_length);

    // Get_Language_Id_Framework and get the length
    language_id_framework = USBD_Get_Language_Id_Framework(&languge_id_framework_length);

    // Install the device portion of USBX
    ret = ux_device_stack_initialize(device_framework_high_speed, device_framework_hs_length,
                                     device_framework_full_speed, device_framework_fs_length,
                                     string_framework, string_framework_length,
                                     language_id_framework, languge_id_framework_length,
                                     nullptr);
    if (ret != UX_SUCCESS) {
        Debugger_log(DBG, "%lu: ux_device_stack_initialize = %d", HAL_GetTick(), ret);
        Error_Handler();
    }


    // USBX_APP_Device_Init();


#ifdef UX_DEVICE_CDC_ACM
    // Allocate the stack for App_Usb_Main_Thread_Entry
    ret = tx_byte_allocate(byte_pool, (VOID **) &pointer, USBX_CDC_ACM_THREAD_STACK_SIZE, TX_NO_WAIT);
    if (ret != UX_SUCCESS) Error_Handler();

    cdcAcmWorker = new Stm32UsbXCdcAcm(pointer, USBX_CDC_ACM_THREAD_STACK_SIZE, Stm32ThreadxThread::thread::priority(),
                                       "USBX CDC ACM worker");
    cdcAcmWorker->createThread();
    cdcAcmWorker->resume();
#endif


#endif
    return ret;
}





