/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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


__IO char optic_rx_buffer[DEBUG_RX_BUF_SIZE];
__IO char optic_tx_buffer[DEBUG_TX_BUF_SIZE];
__IO uint16_t optic_tx_to_send = 0;
__IO char *optic_tx_buf_start;

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

float sine_array[] = {0.00000, 0.01892, 0.03784, 0.05675 ,0.07563 ,0.09449,0.11331
,0.13209
,0.15082
,0.16951
,0.18813
,0.20668
,0.22516
,0.24355
,0.26186
,0.28008
,0.29820
,0.31621
,0.33410
,0.35188
,0.36953
,0.38705
,0.40443
,0.42167
,0.43875
,0.45568
,0.47244
,0.48903
,0.50545
,0.52169
,0.53774
,0.55360
,0.56926
,0.58472
,0.59997
,0.61500
,0.62981
,0.64440
,0.65875
,0.67287
,0.68675
,0.70038
,0.71377
,0.72689
,0.73976
,0.75236
,0.76469
,0.77675
,0.78853
,0.80003
,0.81124
,0.82216
,0.83278
,0.84311
,0.85313
,0.86285
,0.87227
,0.88136
,0.89015
,0.89861
,0.90675
,0.91457
,0.92206
,0.92922
,0.93605
,0.94254
,0.94869
,0.95450
,0.95998
,0.96511
,0.96989
,0.97432
,0.97841
,0.98215
,0.98553
,0.98856
,0.99124
,0.99356
,0.99553
,0.99714
,0.99839
,0.99928};
float R = 1.163; // These values were made using the matlab code in the system files. 
float P = .0001996;
// look up table related value 
bool forward = 1;
int loop_count = 0;
uint32_t odd = 0;
  
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_SMBUS_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);

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
void set_left_leg() {
  HAL_GPIO_WritePin(SCRA_GPIO_Port, SCRA_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SCRD_GPIO_Port, SCRD_Pin, GPIO_PIN_SET);
}

void reset_left_leg(){
  HAL_GPIO_WritePin(SCRA_GPIO_Port, SCRA_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SCRD_GPIO_Port, SCRD_Pin, GPIO_PIN_RESET);
}

// pulse right leg of unfolding bridge (SCR 2 and SCR3)
void set_right_leg() {
  HAL_GPIO_WritePin(SCRB_GPIO_Port, SCRB_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SCRC1_GPIO_Port, SCRC1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SCRC2_GPIO_Port, SCRC2_Pin, GPIO_PIN_SET);
}

void reset_right_leg(){
  HAL_GPIO_WritePin(SCRB_GPIO_Port, SCRB_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SCRC1_GPIO_Port, SCRC1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SCRC2_GPIO_Port, SCRC2_Pin, GPIO_PIN_RESET);
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

// transmit through optical 
void print_optic(char*msg){
  // check if TX is free
  uint8_t state = HAL_UART_GetState(&huart1);
  if (state == HAL_UART_STATE_BUSY_TX || state == HAL_UART_STATE_BUSY_TX_RX) {
    // append to TX buffer
    strncat((char *)optic_tx_buffer, msg, DEBUG_TX_BUF_SIZE - strlen((char *)optic_tx_buffer));
    // update to_send length
    optic_tx_to_send = optic_tx_to_send + strlen(msg); 
  } else {
    strncpy((char *)optic_tx_buffer, msg, DEBUG_TX_BUF_SIZE);
    optic_tx_to_send = 0;
    uint16_t ntransfer = strlen((char *)optic_tx_buffer);
    
    // start TX event
    // set to_send to 0 and update start pointer
    optic_tx_buf_start = optic_tx_buffer + ntransfer;
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)optic_tx_buffer, ntransfer);
    
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
  else if (*curr_rx_ptr=='l')
  {
    print_debug("pulse left leg\r\n");
    //pulse_left_leg();
  }
  else if(*curr_rx_ptr=='r')
  {
    print_debug("pulse right leg\r\n");
    //pulse_right_leg();
  }
  
  
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
  else
  {
    if (optic_tx_to_send > 0) {
      // truncate string if necessary
      if (optic_tx_buf_start + optic_tx_to_send > optic_tx_buffer + DEBUG_TX_BUF_SIZE) {
        // set to_send to 0 and update start pointer
        optic_tx_buf_start = optic_tx_buffer + strlen((char *)optic_tx_buffer);
        uint16_t ntransfer = (uint16_t)(DEBUG_TX_BUF_SIZE - (optic_tx_buf_start-optic_tx_buffer));
        optic_tx_to_send = 0;
        HAL_UART_Transmit_DMA(huart, (uint8_t *)optic_tx_buf_start, ntransfer);
      } else {
        optic_tx_buf_start = optic_tx_buffer + strlen((char *)optic_tx_buffer) - optic_tx_to_send;
        uint16_t ntransfer = optic_tx_to_send;
        optic_tx_to_send = 0;
        HAL_UART_Transmit_DMA(huart, (uint8_t *)optic_tx_buf_start, ntransfer);
      }
    } else {
      // reset buffer start
      optic_tx_buf_start = optic_tx_buffer;      
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

 void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
 {

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  // Set LEDs on
  HAL_TIM_Base_Start_IT(&htim3); // enable timer 3 interrupt  
  set_user1_led(true);
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
  int dcv = *adc_dcv; 
  float v_real = 0; 
  float err = 0;
  float err_old = 0;
  float der = 0;
  float d = 0;
  float d_des = 0;
  float tc  = 1;
  float p = .000196/34.0; // change 34 to 1.7*V_in (this is for 20V in).
  float r = 1.163/34.0;
  float div_per_v = 6.5;
  float v_des = 0;
  reset_left_leg(); // make sure gate is off
  reset_right_leg();
  
  while (1)
  {
    // check if there is a new command waiting. if so parse it

    //control law code. 
//    dcv = *adc_outv; 
//    v_des = 34.0*sine_array[loop_count];
//    d_des = v_des/34.0;
//    v_real = dcv/div_per_v;
//    err = v_real - v_des; 
//    der  = err_old - err;
//    err_old = err; 
//    d = err*p + der*r + d_des; 
     

      
//      if (tosend < 10) { 
//        tosend = 10; 
//        //TODO: ADD PFM. 
//      }
//      
//      if (tosend > 990) { 
//        tosend = 990;
//      }

//    if (monitor_count >= MONITOR_COUNT_MAX*50) {
//      char debug_msg[30];
//      sprintf(debug_msg, "Vout: %d Iout: %d\r\n", *adc_outv, *adc_outi);
//      print_debug(debug_msg);
//      monitor_count = 0;
//    } else {
//      monitor_count++;
//    }
    
      
     //print_debug(monitor_buf);
    
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

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3599;//7199;
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
  huart1.Init.BaudRate = 1200000;
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
  huart2.Init.BaudRate = 15200;
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
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SCRB_Pin|SCRD_Pin|SCRC2_Pin|SCRC1_Pin 
                          |SCRA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USER2_Pin|USER1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO4_Pin|GPIO3_Pin|GPIO2_Pin|GPIO2A12_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SCRB_Pin SCRD_Pin SCRC2_Pin SCRC1_Pin 
                           SCRA_Pin */
  GPIO_InitStruct.Pin = SCRB_Pin|SCRD_Pin|SCRC2_Pin|SCRC1_Pin 
                          |SCRA_Pin;
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

  /*Configure GPIO pins : GPIO4_Pin GPIO3_Pin GPIO2_Pin GPIO2A12_Pin */
  GPIO_InitStruct.Pin = GPIO4_Pin|GPIO3_Pin|GPIO2_Pin|GPIO2A12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) { 

  if (htim->Instance == htim3.Instance) {
     float d = 900*sine_array[loop_count]; // TODO: change to control law. 
     int tosend = (int) d;  
     char monitor_buf[25];
  
                if (loop_count == 0 && forward ==1)
                {           
                      if (1)//(odd%2)
                        set_right_leg();
                      else
                        set_left_leg();
                }
              
                    
                if (loop_count == 20 && forward ==1)
                {           
                      if (1)//((odd++)%2)
                        reset_right_leg();
                      else
                        reset_left_leg();
                }
              
      
      if (loop_count < 30 && forward == 0)
        tosend = 0; // make sure switching at zero
//        
      if (loop_count > 80) {       
         loop_count = 80;  
         forward = 0;
      } else if (loop_count < 0) {
         loop_count = 0;
         forward = 1;
      } else if (forward) { 
        loop_count++;
      } else { 
        loop_count--;
      }
    

    // If speed is necessary, speed up the \r\n process. 
    sprintf(monitor_buf, "%d\n", tosend);
    print_optic(monitor_buf);
  }
}
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
