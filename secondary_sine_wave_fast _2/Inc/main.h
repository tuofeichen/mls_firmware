/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define SCRA_Pin GPIO_PIN_9
#define SCRA_GPIO_Port GPIOE
#define SCRB_Pin GPIO_PIN_11
#define SCRB_GPIO_Port GPIOE
#define SCRC_Pin GPIO_PIN_13
#define SCRC_GPIO_Port GPIOE
#define SCRD_Pin GPIO_PIN_14
#define SCRD_GPIO_Port GPIOE
#define GPIO3_Pin GPIO_PIN_10
#define GPIO3_GPIO_Port GPIOB
#define GPIO4_Pin GPIO_PIN_11
#define GPIO4_GPIO_Port GPIOB
#define USER2_Pin GPIO_PIN_14
#define USER2_GPIO_Port GPIOD
#define USER1_Pin GPIO_PIN_15
#define USER1_GPIO_Port GPIOD
#define TINT2_Pin GPIO_PIN_11
#define TINT2_GPIO_Port GPIOC
#define TCRIT2_Pin GPIO_PIN_12
#define TCRIT2_GPIO_Port GPIOC
#define TINT1_Pin GPIO_PIN_0
#define TINT1_GPIO_Port GPIOD
#define TCRIT1_Pin GPIO_PIN_1
#define TCRIT1_GPIO_Port GPIOD
#define GPIO1_Pin GPIO_PIN_3
#define GPIO1_GPIO_Port GPIOD
#define GPIO2_Pin GPIO_PIN_4
#define GPIO2_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

// Size of RX circular buffers
#define DEBUG_RX_BUF_SIZE 100
#define DEBUG_TX_BUF_SIZE 100
#define DEBUG_MAX_MSG_SIZE 125

// How often to print monitoring statistics
#define MONITOR_COUNT_MAX 100
/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
