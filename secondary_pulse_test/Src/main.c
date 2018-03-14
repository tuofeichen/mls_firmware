/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

SMBUS_HandleTypeDef hsmbus1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
// Debug UART variables
__IO char debug_rx_buffer[DEBUG_RX_BUF_SIZE];
__IO char debug_tx_buffer[DEBUG_TX_BUF_SIZE];
__IO uint16_t debug_tx_to_send = 0;
__IO char *debug_tx_buf_start;

__IO char msg_buffer[DEBUG_MAX_MSG_SIZE];

__IO char *start_ptr;
__IO char *end_ptr;
__IO char *curr_rx_ptr;

// ADC buffers
__IO uint32_t adc1_buffer[3];
__IO uint32_t adc2_buffer[3];

// ADC aliases
__IO uint32_t *adc_swv = &adc1_buffer[0];
__IO uint32_t *adc_dcv = &adc1_buffer[1];
__IO uint32_t *adc_outv = &adc1_buffer[2];

__IO uint32_t *adc_swi = &adc2_buffer[0];
__IO uint32_t *adc_dci = &adc2_buffer[1];
__IO uint32_t *adc_outi = &adc2_buffer[2];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_SMBUS_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
// set USER1 LED
void set_user1_led(bool state) {
  if (state)
    HAL_GPIO_WritePin(USER1_GPIO_Port, USER1_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(USER1_GPIO_Port, USER1_Pin, GPIO_PIN_RESET);
}

// set USER2 LED
void set_user2_led(bool state) {
  if (state)
    HAL_GPIO_WritePin(USER2_GPIO_Port, USER2_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(USER2_GPIO_Port, USER2_Pin, GPIO_PIN_RESET);
}

// pulse left leg of unfolding bridge (SCR 1 and SCR4)
void pulse_left_leg() {
  HAL_GPIO_WritePin(SCRA_GPIO_Port, SCRA_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SCRD_GPIO_Port, SCRD_Pin, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(SCRA_GPIO_Port, SCRA_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SCRD_GPIO_Port, SCRD_Pin, GPIO_PIN_RESET);
}

// pulse right leg of unfolding bridge (SCR 2 and SCR3)
void pulse_right_leg() {
  HAL_GPIO_WritePin(SCRB_GPIO_Port, SCRB_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SCRC_GPIO_Port, SCRC_Pin, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(SCRB_GPIO_Port, SCRB_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SCRC_GPIO_Port, SCRC_Pin, GPIO_PIN_RESET);
}

/* --------------- Serial Interface -----------------------*/
void print_debug(char *msg) {
  // check if TX is free
  uint8_t state = HAL_UART_GetState(&huart2);
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
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)debug_tx_buffer, ntransfer);
    
  }
}

// method to check if there is available text
bool serial_available() {
  if (end_ptr != start_ptr)
    return 1;
  
  return 0;
}

// method to check if there is a command lined up
bool cmd_available() {
  // search from start of the end buffer and look for \r or \n to signal the end
  // of a message. returns 1 if there is a message. 0 otherwise
  for (char *ptr = (char *)debug_rx_buffer; ptr < debug_rx_buffer + DEBUG_RX_BUF_SIZE; ptr++) {
    if (*ptr == '\r' || *ptr == '\n') {
      return 1;
    }
  }
  
  // no command found
  return 0;
}

// method to read the most recent command. you must check if there is an available
// command before calling the method. if there is no command the behavior is undefined.
int get_cmd(char *parsed_msg) {
  for(int k=0; k < DEBUG_RX_BUF_SIZE; k++) {
    char *ptr = (char *)start_ptr + k;
    // check if wrapped around circular buffer
    if (k + start_ptr >= debug_rx_buffer + DEBUG_RX_BUF_SIZE) {
      ptr = k - DEBUG_RX_BUF_SIZE + (char *)start_ptr;
    }
    
    char temp = *ptr;
    
    // check if \r or \n signaling end of msg
    if (temp == '\r' || temp == '\n') {
      // set new start pointer
      if (temp == '\r') {
        // check if \r\n sent as terminating string
        char *next_ptr = ptr + 1;
        // wrap if necessary
        if (next_ptr >= debug_rx_buffer + DEBUG_RX_BUF_SIZE) {
          next_ptr = (char *)debug_rx_buffer;
        }
        
        // check if next_ptr is \n
        if (*next_ptr == '\n') {
          // increment start_ptr to follow next_ptr
          start_ptr = next_ptr + 1;
          // replace \n with \0
          *next_ptr = '\0';
        } else {
          start_ptr = ptr + 1;
        }
      } else {
        start_ptr = ptr + 1;
      }
      
      // replace ptr with \0
      *ptr = '\0';
      
      // make sure start pointer doesn't overflow
      if (start_ptr >= debug_rx_buffer + DEBUG_RX_BUF_SIZE)
        start_ptr = debug_rx_buffer;
      
      // terminate parsed command string with \0
      parsed_msg[k] = '\0';
      
      // return length of parsed message
      return k;
    } else {
      // save character
      parsed_msg[k] = temp;
      // replace old character with \0
      *ptr = '\0';
    }
  }
  
  return 0;
}

// method to echo recieved data back. this method reads through the rx buffer and
// prints any unprinted characters until it hits a \0. make sure to call this
// before parsing the command or it may miss text
void echo_text() {
  // make sure there is text to print
  if (*curr_rx_ptr == '\0')
    return;
  
  int len = strlen((const char *)curr_rx_ptr);
  print_debug((char *)curr_rx_ptr);
  curr_rx_ptr += len;
}

// handle end of TX transaction and see if there is still remaining data to be sent
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  // check if Debug UART
  if (huart == &huart2) {
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

// handle end of recieve transaction and echo result
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST); // Clear the buffer to prevent overrun
  // save received data in buffer
  //store_text((char *)debug_rx_buffer);
  // echo response
  /*
  char echo_char = *curr_rx_ptr;
  print_debug(&echo_char);
  curr_rx_ptr++; 
  // wrap curr_rx_ptr if necessary
  if (curr_rx_ptr >= debug_rx_buffer + DEBUG_RX_BUF_SIZE)
    curr_rx_ptr = debug_rx_buffer;
  */
}

/* ------------------------ Serial Interface End ------------------*/

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  // initialize pointers
  start_ptr = debug_rx_buffer;
  end_ptr = debug_rx_buffer;
  curr_rx_ptr = start_ptr;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C1_SMBUS_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  // Set LEDs off
  set_user1_led(false);
  set_user2_led(false);
    
  // Print boot message
  print_debug("Secondary Board v1.0\n\r");
  
  // Start receiver
  HAL_UART_Receive_DMA(&huart2, (uint8_t *)debug_rx_buffer, DEBUG_RX_BUF_SIZE);
  
  // Start ADC2
  //if (HAL_ADC_Start(&hadc2) != HAL_OK)
  //  return 0;
  
  // Start ADC1
  //if( HAL_ADC_Start(&hadc1) != HAL_OK)
  //  return 0;
  // start ADC1 DMA and set buffer address
  //int status = HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)adc1_buffer, 3);
  
  int status = HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buffer, 3);
  if (status != HAL_OK)
    return 0;
  
  status = HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buffer, 3);
  if (status != HAL_OK)
    return 0;
  
  HAL_TIM_Base_Start(&htim3);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int monitor_count = 0;
  char monitor_buf[25];
  //pulse_right_leg();
  
  while (1)
  {
    // check if there is a new command waiting. if so parse it
    pulse_left_leg();
   
    echo_text();
    if (cmd_available()) {
      int len = get_cmd((char *)msg_buffer);
      // check command
      if (strcmp((char *)msg_buffer, "1") == 0) {
        print_debug("Turning on left leg\r\n");
        pulse_left_leg();
      } else if (strcmp((char *)msg_buffer, "2") == 0) {
        print_debug("Turning on right leg\r\n");
        pulse_right_leg();
      }
    }
    
    // check if we should print out periodic health information
    if (monitor_count >= MONITOR_COUNT_MAX*2) {
      sprintf(monitor_buf, "Output Voltage: %d\r\n", *adc_outv);
      
      print_debug(monitor_buf);
      sprintf(monitor_buf, "Output Current: %d\r\n", *adc_outi);
      print_debug(monitor_buf);
      
      monitor_count = 0;
    } else {
      monitor_count++;
    }
    
    HAL_Delay(5);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the ADC multi-mode 
    */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* ADC2 init function */
static void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 3;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2C1 init function */
static void MX_I2C1_SMBUS_Init(void)
{

  hsmbus1.Instance = I2C1;
  hsmbus1.Init.Timing = 0x2000090E;
  hsmbus1.Init.AnalogFilter = SMBUS_ANALOGFILTER_ENABLE;
  hsmbus1.Init.OwnAddress1 = 2;
  hsmbus1.Init.AddressingMode = SMBUS_ADDRESSINGMODE_7BIT;
  hsmbus1.Init.DualAddressMode = SMBUS_DUALADDRESS_DISABLE;
  hsmbus1.Init.OwnAddress2 = 0;
  hsmbus1.Init.OwnAddress2Masks = SMBUS_OA2_NOMASK;
  hsmbus1.Init.GeneralCallMode = SMBUS_GENERALCALL_DISABLE;
  hsmbus1.Init.NoStretchMode = SMBUS_NOSTRETCH_DISABLE;
  hsmbus1.Init.PacketErrorCheckMode = SMBUS_PEC_DISABLE;
  hsmbus1.Init.PeripheralMode = SMBUS_PERIPHERAL_MODE_SMBUS_SLAVE;
  hsmbus1.Init.SMBusTimeout = 0x00008061;
  if (HAL_SMBUS_Init(&hsmbus1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 7199;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
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
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SCRA_Pin|SCRB_Pin|SCRC_Pin|SCRD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USER2_Pin|USER1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SCRA_Pin SCRB_Pin SCRC_Pin SCRD_Pin */
  GPIO_InitStruct.Pin = SCRA_Pin|SCRB_Pin|SCRC_Pin|SCRD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USER2_Pin USER1_Pin */
  GPIO_InitStruct.Pin = USER2_Pin|USER1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : TINT2_Pin TCRIT2_Pin */
  GPIO_InitStruct.Pin = TINT2_Pin|TCRIT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : TINT1_Pin TCRIT1_Pin */
  GPIO_InitStruct.Pin = TINT1_Pin|TCRIT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
