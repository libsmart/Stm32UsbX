/*
 * SPDX-FileCopyrightText: 2024 Roland Rusch, easy-smart solution GmbH <roland.rusch@easy-smart.ch>
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "Stm32UsbXDeviceInit.h"
#include "main.h"
#include "tx_api.h"
#include "usb_otg.h"
#include "ux_api.h"
#include "ux_dcd_stm32.h"

/**
  * @brief USBX_APP_Device_Init
  *        Initialization of USB device.
  * Init USB device Library, add supported class and start the library
  * @retval None
  */
void USBX_APP_Device_Init()
{
    UINT ret = UX_SUCCESS;
    Debugger_log(DBG, "%lu: Stm32UsbX::USBX_APP_Device_Init()", HAL_GetTick());

    // Rendering hardware reset harmless (no need to replug USB cable)
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // GPIO Ports Clock Enable
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Configure GPIO pin Output Level
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

    // Configure GPIO pin : PA12, a.k.a. USB_DP
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Sleep for 100 ms
    tx_thread_sleep(UX_MS_TO_TICK(500));
    // Hardware reset rendered harmless!



    // USB_OTG init function
    Debugger_log(DBG, "%lu: MX_USB_OTG_FS_PCD_Init()", HAL_GetTick());
    MX_USB_OTG_FS_PCD_Init();

    // Set FIFOs
    Debugger_log(DBG, "%lu: HAL_PCDEx_SetRxFiFo()", HAL_GetTick());
    HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_FS, 0x100);
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 0, 0x10); //USBD_MAX_EP0_SIZE/4
    //    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 1, 0x10);
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 2, 0x10); // USBD_CDCACM_EPIN_HS_MPS/4
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 3, 0x20); // USBD_CDCACM_EPINCMD_HS_MPS/4

    // Initialize and link controller HAL driver
    ret = ux_dcd_stm32_initialize((ULONG)USB_OTG_FS, (ULONG)&hpcd_USB_OTG_FS);
    Debugger_log(DBG, "%lu: ux_dcd_stm32_initialize = 0x%02x", HAL_GetTick(), ret);
    if (ret != UX_SUCCESS) Error_Handler();

    // Start USB device
    HAL_PCD_Start(&hpcd_USB_OTG_FS);
}
