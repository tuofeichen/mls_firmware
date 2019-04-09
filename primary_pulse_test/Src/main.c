/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

void stop_pwm(); 

// Optic UART RX Buffer
__IO char optic_rx_buffer[OPTIC_RX_BUF_SIZE];
__IO char *optic_start_ptr;
__IO char *optic_end_ptr;
__IO char *optic_curr_rx_ptr;

// USB Uart TX Buffer

__IO char debug_tx_buffer[DEBUG_TX_BUF_SIZE];
__IO char debug_rx_buffer[DEBUG_RX_BUF_SIZE];
__IO uint16_t debug_tx_to_send = 0;
__IO char *debug_tx_buf_start;
__IO char msg_buffer[DEBUG_MAX_MSG_SIZE];



// Pulse settings
__IO float duty = .5;

__IO uint16_t pulse_length = 1;

int init = 0 ;
int td = 5;  // dead time for switch transitions
int tcomm = 45;  // commutation period for leakage inductance
int period = PWM_PERIOD;
int new_pwm = 0;
uint32_t duty_ul;
uint32_t duty_ll;
uint32_t duty_ur;
uint32_t duty_lr;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART3_UART_Init(void);                                    
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

/* --------------- Serial Interface -----------------------*/
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

// method to check if there is available text
bool serial_available() {
  if (optic_end_ptr != optic_start_ptr)
    return 1;
  
  return 0;
}

// method to check if there is a command lined up
bool cmd_available() {
  // search from start of the end buffer and look for \r or \n to signal the end
  // of a message. returns 1 if there is a message. 0 otherwise
  for (char *ptr = (char *)optic_rx_buffer; ptr < optic_rx_buffer + OPTIC_RX_BUF_SIZE; ptr++) {
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
  for(int k=0; k < OPTIC_RX_BUF_SIZE; k++) {
    char *ptr = (char *)optic_start_ptr + k;
    // check if wrapped around circular buffer
    if (k + optic_start_ptr >= optic_rx_buffer + OPTIC_RX_BUF_SIZE) {
      ptr = k - OPTIC_RX_BUF_SIZE + (char *)optic_start_ptr;
    }
    
    char temp = *ptr;
    
    // check if \r or \n signaling end of msg
    if (temp == '\r' || temp == '\n') {
      // set new start pointer
      if (temp == '\r') {
        // check if \r\n sent as terminating string
        char *next_ptr = ptr + 1;
        // wrap if necessary
        if (next_ptr >= optic_rx_buffer + OPTIC_RX_BUF_SIZE) {
          next_ptr = (char *)optic_rx_buffer;
        }
        
        // check if next_ptr is \n
        if (*next_ptr == '\n') {
          // increment optic_start_ptr to follow next_ptr
          optic_start_ptr = next_ptr + 1;
          // replace \n with \0
          *next_ptr = '\0';
        } else {
          optic_start_ptr = ptr + 1;
        }
      } else {
        optic_start_ptr = ptr + 1;
      }
      
      // replace ptr with \0
      *ptr = '\0';
      
      // make sure start pointer doesn't overflow
      if (optic_start_ptr >= optic_rx_buffer + OPTIC_RX_BUF_SIZE)
        optic_start_ptr = optic_rx_buffer;
      
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
  if (*optic_curr_rx_ptr == '\0')
    return;
  
  int len = strlen((const char *)optic_curr_rx_ptr);
  print_debug((char *)optic_curr_rx_ptr);
  optic_curr_rx_ptr += len;
}

// handle end of TX transaction and see if there is still remaining data to be sent
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

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  
  if (huart==&huart3)
  {
//  char err_msg [20];
//  
////  sprintf(err_msg,"uart error %d\r\n",huart->ErrorCode);
////  print_debug(err_msg);

  // reset hal dma   
  HAL_UART_Receive_DMA(&huart3, (uint8_t *)optic_rx_buffer, OPTIC_RX_BUF_SIZE);
  if (init==1)
    stop_pwm(); 
  
  }
}

// handle end of recieve transaction and echo result
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST); // Clear the buffer to prevent overrun
  // save received data in buffer
  //store_text((char *)optic_rx_buffer);
  // echo response
  /*
  char echo_char = *optic_curr_rx_ptr;
  print_debug(&echo_char);
  optic_curr_rx_ptr++; 
  // wrap optic_curr_rx_ptr if necessary
  if (optic_curr_rx_ptr >= optic_rx_buffer + OPTIC_RX_BUF_SIZE)
    optic_curr_rx_ptr = optic_rx_buffer;
  */
}


/* ------------------------ Serial Interface End ------------------*/

void start_pwm() {
  /* Start channel 1 */
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
  /* Start channel 2 */
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
  /* Start channel 3 */
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK)
  {
    /* PWM generation Error */
    Error_Handler();
  }
  /* Start channel 4 */
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) != HAL_OK)
  {
    /* PWM generation Error */
    Error_Handler();
  }
  HAL_TIM_Base_Start_IT(&htim2);
}


void stop_pwm() {
  /* Stop channel 1 */
  if (HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
  /* Stop channel 2 */
  if (HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
  /* Stop channel 3 */
  if (HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3) != HAL_OK)
  {
    /* PWM generation Error */
    Error_Handler();
  }
  /* Stop channel 4 */
  if (HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4) != HAL_OK)
  {
    /* PWM generation Error */
    Error_Handler();
  }
}


void init_pwm(float pwm) {
  // Configure and start PWM
  int ttot = period - 4 * td;
  int ton = (uint32_t)(ttot / 2.0 * duty + tcomm);
  int toff = (uint32_t)((ttot - 2*ton)/2.0);
  
  duty_ul = (uint32_t)(ton/2.0) + td;
  duty_ll = (uint32_t)(ton/2.0);
  duty_ur = (uint32_t)(ton/2.0 + toff) + td; 
  duty_lr = (uint32_t)(ton/2.0 + toff) + 2*td;
  
  // Timer setup
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim2.Init.Period = period/2;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  
  // configure channel 1 (LR)
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = duty_lr;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  
  // configure channel 2 (UL)
  sConfigOC.Pulse = duty_ul;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  // configure channel 3 (LL)
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.Pulse = duty_ll;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  // configure channel 4 (UR)
  sConfigOC.Pulse = duty_ur;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  
}

void set_pwm(float pwm) {
  
  if (pwm >= 0.95)
    pwm = 0.95;
  else if (pwm<=0.0)
    pwm = 0.0;
  
  duty = pwm;
  int ton = 0;
  // Configure and start PWM
  int ttot = period - 4 * td;
  if (pwm > 0.0)
    ton = (uint32_t)(ttot / 2.0 * duty + tcomm);
  
  int toff = (uint32_t)((ttot - 2*ton)/2.0);
  
 
  // wait for last pwm value to be updated
  while (new_pwm);
  
  duty_ul = (uint32_t)(ton/2.0) + td;
  duty_ll = (uint32_t)(ton/2.0);
  duty_ur = (uint32_t)(ton/2.0 + toff) + td; 
  duty_lr = (uint32_t)(ton/2.0 + toff) + 2*td;
  
  // signal new pwm values are present
  new_pwm = 1;
}

// Interrupt handler to change pwm values on timer reset and handle ending pulse
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  // change PWM values
  if (htim->Instance == htim2.Instance) {
    if (new_pwm) {
      htim->Instance->CCR1 = duty_lr;
      htim->Instance->CCR2 = duty_ul;
      htim->Instance->CCR3 = duty_ll;
      htim->Instance->CCR4 = duty_ur;
      // clear flag
      new_pwm = 0;
    }
  }
}

// need to write enable for outputs based on another pulse length timer

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
 int main(void)
{
  /* USER CODE BEGIN 1 */
  // initialize pointers
  optic_start_ptr = optic_rx_buffer;
  optic_end_ptr = optic_rx_buffer;
  optic_curr_rx_ptr = optic_start_ptr;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  // Set LEDs off
  set_user1_led(false);
  set_user2_led(false);
  
  // initialize pwm to 50% duty cycle (doesn't start pwm)
  init_pwm(.5);
  start_pwm();
  
  // Print boot message
  print_debug("Primary Board v1.0\n\r");
   // Start Optical Receiver
  HAL_UART_Receive_DMA(&huart3, (uint8_t *)optic_rx_buffer, OPTIC_RX_BUF_SIZE);
  HAL_UART_Receive_DMA(&huart1, (uint8_t *)debug_rx_buffer, DEBUG_RX_BUF_SIZE);
 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int cmd_cnt = 0;
  while (1)
  {
    // check if there is a new command waiting. if so parse it
   // echo_text();
    
    if (cmd_available()) {
      int len = get_cmd((char *)msg_buffer);
      
      // check command
      if (len != 0) { // if just a blank entering command -> start pulse
        init = 1; 
        int  duty_int = atoi((char *)msg_buffer);
        char duty_debug_msg [20];

        duty = 1.0*duty_int/900.0;
        
        if (((cmd_cnt++)%100000) == 0)
        {
           sprintf(duty_debug_msg,"Set duty %5.3f\r\n",duty);
           print_debug(duty_debug_msg);
        }
       // set_pwm(duty); 
      }   
      
      // start pulse
      // start pwm
      // start_pwm();
      // HAL_Delay(pulse_length);
      // print_debug("Shit got turned on/off\r\n");
      // stop_pwm();
    }

//    print_debug("Test UART\r\n"); 
   // HAL_Delay(5);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART3;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim2.Init.Period = PWM_PERIOD/2;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 7200;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 15200;
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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 1000000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, USER2_Pin|USER1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : USER2_Pin USER1_Pin */
  GPIO_InitStruct.Pin = USER2_Pin|USER1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
