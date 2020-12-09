/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l4xx_it.h"
#include "printf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef enum reset_cause_e
{
    RESET_CAUSE_UNKNOWN = 0,
    RESET_CAUSE_LOW_POWER_RESET,
    RESET_CAUSE_WINDOW_WATCHDOG_RESET,
    RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET,
    RESET_CAUSE_SOFTWARE_RESET,
    RESET_CAUSE_POWER_ON_POWER_DOWN_RESET,
    RESET_CAUSE_EXTERNAL_RESET_PIN_RESET,
    RESET_CAUSE_BROWNOUT_RESET,
} reset_cause_t;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

uint32_t DWT_Init(void)
{
    /* Disable TRC */
    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
    /* Enable TRC */
    CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;
    /* Disable clock cycle counter */
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
    /* Enable  clock cycle counter */
    DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk; //0x00000001;
    /* Reset the clock cycle counter value */
    DWT->CYCCNT = 0;
    /* 3 NO OPERATION instructions */
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");
    /* Check if clock cycle counter has started */
    if(DWT->CYCCNT)
    {
       return 0; /*clock cycle counter started*/
    }
    else
    {
      return 1; /*clock cycle counter not started*/
    }
}

reset_cause_t reset_cause_get(void)
{
    reset_cause_t reset_cause;

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST))
    {
        reset_cause = RESET_CAUSE_LOW_POWER_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST))
    {
        reset_cause = RESET_CAUSE_WINDOW_WATCHDOG_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
    {
        reset_cause = RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))
    {
        reset_cause = RESET_CAUSE_SOFTWARE_RESET; // This reset is induced by calling the ARM CMSIS `NVIC_SystemReset()` function!
    }
//    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))
//    {
//        reset_cause = RESET_CAUSE_POWER_ON_POWER_DOWN_RESET;
//    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))
    {
        reset_cause = RESET_CAUSE_EXTERNAL_RESET_PIN_RESET;
    }
    // Needs to come *after* checking the `RCC_FLAG_PORRST` flag in order to ensure first that the reset cause is 
    // NOT a POR/PDR reset. See note below. 
    else
    {
        reset_cause = RESET_CAUSE_UNKNOWN;
    }

    // Clear all the reset flags or else they will remain set during future resets until system power is fully removed.
    __HAL_RCC_CLEAR_RESET_FLAGS();

    return reset_cause; 
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  short resetFlag = reset_cause_get(); 
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
    
  // auto calibrating ADC 
  HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED); 
  HAL_ADCEx_Calibration_Start(&hadc1,ADC_DIFFERENTIAL_ENDED); 

  
  CAN_Filter_Init();
  char monitor_buf[55];
  snprintf(monitor_buf,55,"System Start up! Last Reset Flag is %d, DWT Status %d\r\n\n",resetFlag,DWT_Init()); 
  print_debug(monitor_buf); 
  start_pwm(); 
  change_phase_shift(200); 
  change_duty_cycle(800);  

  // Setup UART receiver DMA
  //HAL_UART_Receive_DMA(&huart1, &sineTableCnt, 1);
  //HAL_UART_Receive_IT(&huart2, &sineTableCnt,1); 
                     
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // start the ADC
  int status = HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buffer, 3);
//  HAL_OPAMP_Start(&hopamp1); 
  if (status != HAL_OK)
      print_debug("Failed to Initialize ADC\r\n");
  
  HAL_TIM_Base_Start_IT(&htim3); // enable timer 3 interrupt 
    
  // Control Gain
  K11 = 0; 
  short K12 = 22000; 
  short K13 = 0; 
  short K14 = 2; 
  
  K22 = 0; 
  short K21 = 5000; 
  short K23 = 2; 
  short K24 = 0; 
  
  
  // Current Control Variables
  short currOutput = 0; 
  short currFiltered = 0; 
  //short currCmd  = 100;
  short currError = 0; 
  
 // float currSI = 0.0; 
 // float currSiFiltered = 0.0;
  float currErrorInt = 0.0f; 
  float currErrorIntSat = 0.03f; 
  short Dcmd = 0;
  short DcmdNew = Dcmd; 
  
  
  // Voltage Control Variables 
  float voltErrorInt = 0.0f;
  float voltErrorIntSat = 0.03f; 
  short voltOutput = 0; 
  short voltInput  = 0; 
  short voltError = 0; 
  short phiCmd = 0; 
  
  // CAN related
  // CanMaster = 1; 
  short dutyCmdCurrInt = 0;
  short phiCmdCurrInt = 0;
  TxData[0] = dutyCmdCurrInt & 0xff;  // Low 8 bits
  TxData[1] = dutyCmdCurrInt >> 8;    // High 8 bits
  TxData[2] = phiCmdCurrInt  & 0xff;  // Low 8 bits
  TxData[3] = phiCmdCurrInt  >> 8;    // High 8 bits
 
  // synchronization
  // uint8_t sync = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11);
  
  int tick = 0; 
  long int  monitorCnt = 0; 
  
  char monitorBuf[100];
  // start the timer
  // HAL_TIM_Base_Start_IT(&htim3); // enable timer 3 interrupt 
    
  while (1)
  {
    if (voltUpdate == 0) // wait for new ADC value to come in
        continue; 
    
    tick = DWT->CYCCNT; // Check Timelapse
    DWT->CYCCNT = 0;
    voltUpdate  = 0; // clear update flag
    
    voltInput    = 0.8f*voltInput+0.2f*(*adc_val3);
    voltCmd      = voltInput/2.67f;// 200; // Fixed Voltage Tracking
    voltOutput   = 0.8f*voltOutput+0.2f*(*adc_val2 - 2048); 
    voltError    = voltCmd - voltOutput; 
    voltErrorInt = voltErrorInt + voltError*tick/(72000000.0f); 
    
//    if (voltOutput>1000)
//    {
//      snprintf(monitorBuf,50,"HV at Cout!\r\n"); 
//      print_debug(monitorBuf);    
//      change_phase_shift(0); 
//      change_duty_cycle(0); 
//      continue; 
//    } 
     
    // current control 
    currOutput = *adc_val1 - 2048; // Note possible OFFSET
    currFiltered = 0.8f*currFiltered + 0.2f*currOutput; 
    currError = currCmd - currFiltered;
    currErrorInt = currErrorInt + currError*tick/(72000000.0f);
    
    if (CanMaster>0)
    {
      dutyCmdCurrInt =  K12*currErrorInt; 
      phiCmdCurrInt  =  K22*currErrorInt; 
      TxData[0] = dutyCmdCurrInt & 0xff;  // Low 8 bits
      TxData[1] = dutyCmdCurrInt >> 8;    // High 8 bits
      TxData[2] = phiCmdCurrInt  & 0xff;  // Low 8 bits
      TxData[3] = phiCmdCurrInt >> 8;    // High 8 bits
       
      if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
      {
        /* Transmission request Error */
        Error_Handler();
      }
    }
     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);  
     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,GPIO_PIN_RESET);  
    //Integrator Saturations for current
    if (currErrorInt>currErrorIntSat)
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET);  
      // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);  
      currErrorInt = currErrorIntSat;
    }
    else if (currErrorInt<-currErrorIntSat)
    {
       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET);  
      // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);  
      currErrorInt = -currErrorIntSat;
    }
    
    // Integrator Saturations for voltage
    if (voltErrorInt>voltErrorIntSat)
    {   
       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET);  
      //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);  
      voltErrorInt = voltErrorIntSat;
    }
    else if (voltErrorInt<-voltErrorIntSat)
    {
       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET);  
       //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);  
      voltErrorInt = -voltErrorIntSat;  
    }
   
    //    DcmdNew = K11*voltErrorInt + K12*currErrorInt +K13*voltError+K14*currError;
    //    phiCmd = K21*voltErrorInt + K22*currErrorInt + K23*voltError + K24*currError; 
   
    if (CanMaster == 1)
    {
       DcmdNew =  K11*voltErrorInt + dutyCmdCurrInt +K13*voltError + K14*currError;
       phiCmd  =  K21*voltErrorInt + phiCmdCurrInt + K23*voltError + K24*currError; 
    }
    else
    {
       DcmdNew =  K11*voltErrorInt + dCmdRecv +K13*voltError + K14*currError;
       // DcmdNew = DtTable[sineTableCnt]; 
       phiCmd  = K21*voltErrorInt + phiCmdRecv +  K23*voltError + K24*currError;//
    }
    if (DcmdNew>999)
    {
//      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,GPIO_PIN_SET);  
      DcmdNew = 999;
    }
    else if (DcmdNew<-999)
    {
//      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,GPIO_PIN_SET);  
      DcmdNew = -999;
    }  
    
    if (phiCmd>230)
    {
      //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,GPIO_PIN_SET);  
      phiCmd=230;
    }
    else if (phiCmd<-230)
    {
      //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,GPIO_PIN_SET);  
      phiCmd = -230;
    }
    
    if (abs(phiCmd-Dcmd)>50) // Jitter Indicator 
    {
       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,GPIO_PIN_SET);   
    }
    
//    phiCmdTick = 0.2f*phiCmdTick + 0.8f*phiCmd;
    
    change_phase_shift(phiCmd);     
    change_duty_cycle(DcmdNew);    
    Dcmd = phiCmd; 
    
      
    if (monitorCnt==200000) 
    {
        // Vin  to  ADC: 20.28  <-> 630
        // Vout to  ADC: 18 <-> 209
        // Iout to  ADC: 1.27 <-> 100
       // snprintf(monitorBuf,60,"UART Heartbeat\r\n");
       snprintf(monitorBuf,60,"D:%d, phi:%d, Vin:%d I:%d, Vout:%d \r\n",DcmdNew,phiCmd,voltInput,currFiltered,voltOutput); 
       //snprintf(monitorBuf,10,"I,%d\r\n",currFiltered); 
       print_debug(monitorBuf);    
       monitorCnt = 1;    
    } 
    else
    {
     monitorCnt++; 
    }
    
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSE;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
