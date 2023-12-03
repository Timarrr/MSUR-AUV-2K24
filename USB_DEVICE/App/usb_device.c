/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb_device.c
  * @version        : v1.0_Cube
  * @brief          : This file implements the USB Device
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int flag = 0;
int tick = 0;
int bTick = 0;
/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USB Device Core handle declaration. */
USBD_HandleTypeDef hUsbDeviceFS;

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */
uint16_t CDC_OTG_Send(uint8_t *Buf, uint16_t Len){
	  uint8_t result = USBD_OK;
	  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
	  if (USB_OTG_FS_State != 0){
		  GPIOB->BSRR = (1<<16);
		  return USBD_OK;
	  }

	  if (hcdc->TxState != 0 && hcdc->TxState != USBD_BUSY){
		  GPIOB->BSRR = (1<<16);
	      return hcdc->TxState;
	  }

	  while(hcdc->TxState == USBD_BUSY){
		  GPIOB->BSRR = ((1<<0) << (16*flag));
		  if(tick + 25 < HAL_GetTick()){
			  flag = flag?0:1;
			  tick = HAL_GetTick();
		  }
		  if (USB_OTG_FS_State != 0){
			  return USBD_OK;
		  }
	  }
	  GPIOB->BSRR = (1<<16);

	  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
	  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
	  return result;
}

uint16_t CDC_OTG_Recv(uint8_t *Buf){

	  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, Buf);
	  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
	  return (USBD_OK);
}

/* USER CODE END 1 */

/**
  * Init USB device Library, add supported class and start the library
  * @retval None
  */
void MX_USB_DEVICE_Init(void)
{
  /* USER CODE BEGIN USB_DEVICE_Init_PreTreatment */

  /* USER CODE END USB_DEVICE_Init_PreTreatment */

  /* Init Device Library, add supported class and start the library. */
  if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN USB_DEVICE_Init_PostTreatment */
  HAL_PWREx_EnableUSBVoltageDetector();

  /* USER CODE END USB_DEVICE_Init_PostTreatment */
}

/**
  * @}
  */

/**
  * @}
  */

