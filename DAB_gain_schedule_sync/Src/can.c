/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
CAN_FilterTypeDef sFilterConfig; 
CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               TxData[4];
uint8_t               RxData[4];
uint32_t TxMailbox; 
short   dCmdRecv   = 0; 
short   phiCmdRecv = 0; 
uint8_t CanMaster  = 1; // 0: Slave, 1: Master, 2: Pseudo Slave
/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  hcan.Instance = CAN;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN)
  {
  /* USER CODE BEGIN CAN_MspInit 0 */

  /* USER CODE END CAN_MspInit 0 */
    /* CAN clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration    
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN interrupt Init */
    HAL_NVIC_SetPriority(USB_LP_CAN_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN_RX0_IRQn);
  /* USER CODE BEGIN CAN_MspInit 1 */

  /* USER CODE END CAN_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN)
  {
  /* USER CODE BEGIN CAN_MspDeInit 0 */

  /* USER CODE END CAN_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();
  
    /**CAN GPIO Configuration    
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN_RX0_IRQn);
  /* USER CODE BEGIN CAN_MspDeInit 1 */

  /* USER CODE END CAN_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Get RX message */
  // SET_BIT(hcan->Instance->RF0R, CAN_RF0R_RFOM0);
  HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_0);   
  // HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_SET);
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }
  else
  {
     dCmdRecv = RxData[0]|(RxData[1]<<8);
     phiCmdRecv = RxData[2]|(RxData[3]<<8);
  }
}

void CAN_Filter_Init()
{
    
  sFilterConfig.FilterBank=0;   
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; 
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; 
  sFilterConfig.FilterIdHigh = 0x0000; 
  sFilterConfig.FilterIdLow = 0x0000; 
  sFilterConfig.FilterMaskIdHigh = 0x0000; 
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation=ENABLE; 

  
   if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }
  
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }
  
  if (CanMaster != 1)
  {
    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
      /* Notification Error */
      Error_Handler();
    }
  }
  
  TxData[0] = 2;
  TxData[1] = 5;  
  TxHeader.StdId = 0x321;
  TxHeader.ExtId = 0x01;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 4; // datalength
  TxHeader.TransmitGlobalTime = DISABLE;
 
}


/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
