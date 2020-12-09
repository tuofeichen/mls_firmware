/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "printf.h"
#include "adc.h"
__IO char debug_rx_buffer[DEBUG_RX_BUF_SIZE];
__IO char debug_tx_buffer[DEBUG_TX_BUF_SIZE];
__IO uint16_t debug_tx_to_send = 0;
__IO char *debug_tx_buf_start;
uint8_t sineTableCnt = 0; 
uint8_t sineCntPrev = 201; 

void print_debug(char *msg) {
  // check if TX is free
  uint8_t state = HAL_UART_GetState(&huart1);
  if (state == HAL_UART_STATE_BUSY_TX || state == HAL_UART_STATE_BUSY_TX_RX) {
    // append to TX buffer
    strncat((char *)debug_tx_buffer, msg, DEBUG_TX_BUF_SIZE - strlen((char *)debug_tx_buffer));
    // update to_send length
    debug_tx_to_send = debug_tx_to_send + strlen(msg); 
  } else {
    strncpy((char *)debug_tx_buffer, msg, DEBUG_TX_BUF_SIZE);
    debug_tx_to_send = 0;
    uint16_t ntransfer = strlen((char *)debug_tx_buffer);
    
    // start TX event
    // set to_send to 0 and update start pointer
    debug_tx_buf_start = debug_tx_buffer + ntransfer;
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)debug_tx_buffer, ntransfer);
    
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  char err_msg [20];
  if (huart == &huart2)
  {
    snprintf(err_msg,20,"uart1 error %d\r\n", huart->ErrorCode);
    print_debug(err_msg);
    // HAL_UART_Receive_DMA(&huart2, &sineTableCnt, 1);
  }
}
  

  void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  // check if Debug UART
  if (huart == &huart1) {
    // check if tx buffer is not empty
    if (debug_tx_to_send > 0) {
      // truncate string if necessary
      if (debug_tx_buf_start + debug_tx_to_send > debug_tx_buffer + DEBUG_TX_BUF_SIZE) {
        // set to_send to 0 and update start pointer
        debug_tx_buf_start = debug_tx_buffer + strlen((char *)debug_tx_buffer);
        uint16_t ntransfer = (uint16_t)(DEBUG_TX_BUF_SIZE - (debug_tx_buf_start-debug_tx_buffer));
        debug_tx_to_send = 0;
        HAL_UART_Transmit_DMA(huart, (uint8_t *)debug_tx_buf_start, ntransfer);
      } else {
        debug_tx_buf_start = debug_tx_buffer + strlen((char *)debug_tx_buffer) - debug_tx_to_send;
        uint16_t ntransfer = debug_tx_to_send;
        debug_tx_to_send = 0;
        HAL_UART_Transmit_DMA(huart, (uint8_t *)debug_tx_buf_start, ntransfer);
      }
    } else {
      // reset buffer start
      debug_tx_buf_start = debug_tx_buffer;      
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST); // Clear the buffer to prevent overrun
  // save received data in buffer
  //store_text((char *)debug_rx_buffer);
  // echo response
  // char echo_char = *debug_rx_buffer;
  
  if (adc1_buffer[2]<20) // low noise trust measurement
  {
    sineCntPrev = sineTableCnt; 
  }
  else 
  {
    uint8_t sineBlind = sineCntPrev+1; 
    if (sineTableCnt>199)
      sineTableCnt = 0; 
    
//    if (sineBlind>199)
//      sineBlind = 0;
//    
//    if ((sineTableCnt>199)||((sineTableCnt-sineCntPrev)>5))
//    {
//      HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_0);
//      sineTableCnt = sineBlind;
//    }
//    else if ((sineTableCnt<sineCntPrev)&&((sineTableCnt+200-sineCntPrev)>5))
//    {
//        // HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_0);
//        sineTableCnt = sineBlind; 
//    }
  }
  // sineCntPrev = sineTableCnt; 
  
 
  
  currCmd = sineTable[sineTableCnt]; 
  K11 = K11Table[sineTableCnt];         
  K22 = K22Table[sineTableCnt];
  
  //HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_0);
  //echo[0] = *debug_rx_buffer; 
 //echo[1] = *(debug_rx_buffer+1); 
 // echo[2] = *(debug_rx_buffer+2); 
//  HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_0);
//  snprintf(echo,10,"%d\r\n",echo[0]); 
//  print_debug(echo);    
//  echo[3] = '\r';
//  echo[4] = '\n';
//  print_debug(echo);
  
  /*
  char echo_char = *curr_rx_ptr;
  print_debug(&echo_char);
  curr_rx_ptr++; 
  // wrap curr_rx_ptr if necessary
  if (curr_rx_ptr >= debug_rx_buffer + DEBUG_RX_BUF_SIZE)
    curr_rx_ptr = debug_rx_buffer;
  */
}

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_tx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART1 GPIO Configuration    
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA1_Channel2;
    hdma_usart1_tx.Init.Request = DMA_REQUEST_USART1_TX;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA15 (JTDI)     ------> USART2_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA15 (JTDI)     ------> USART2_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_15);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
