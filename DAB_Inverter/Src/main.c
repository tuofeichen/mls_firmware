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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f3xx_it.h"
#include "stm32f3xx_hal_rcc.h"
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
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))
    {
        reset_cause = RESET_CAUSE_POWER_ON_POWER_DOWN_RESET;
    }
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
  MX_TIM1_Init();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
    
  char monitor_buf[55];
  snprintf(monitor_buf,55,"System Start up! Last Reset Flag is %d, DWT Status %d\r\n\n",resetFlag,DWT_Init()); 
  print_debug(monitor_buf); 
  start_pwm(); 
  change_phase_shift(145); 
  change_duty_cycle(500); 
  // turn on unfolding bridge
  // HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
  // HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);  
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
  // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);  
   
  // auto calibrating ADC 
  HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED); 
  HAL_ADCEx_Calibration_Start(&hadc1,ADC_DIFFERENTIAL_ENDED); 
      
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // start the ADC
  int status = HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buffer, 3);
  if (status != HAL_OK)
      print_debug("Failed to Initialize ADC\r\n");
  
  
  // Current Control Variables
  short currOutput = 0; 
  short currFiltered = 0; 
  //short currCmd  = 100;
  short currError = 0; 
  
  float currSI = 0.0; 
  float currSiFiltered = 0.0;
  float currErrorInt = 0.0f; 
  float currErrorIntSat = 5.0f; 
  
  short KpCurrent = 3;
  short KiCurrent = 3000; 
  short Kpv = -2; 
  short Dcmd = 0;
  short DcmdNew = Dcmd; 
  
  
  // Voltage Control Variables 
  float voltErrorInt = 0.0f;
  float voltErrorIntSat = 15.0f; 
  short Kp = 4 ;
  short Ki = 3400;
  short phiCmdTick = 0; 
  //short phiNom = 150; 
  short voltOutput = 0; 
  short voltInput  = 0; 
  short voltError = 0; 
  short dphiCmdTick = 0; 
  
  short maxDphi = 10; 
  int saturateCnt = 0; 
  int tick = 0; 
  long int  monitorCnt = 0; 
  
  char monitorBuf[100];
  // start the timer
  HAL_TIM_Base_Start_IT(&htim3); // enable timer 3 interrupt 

  while (1)
  {
   
    if (voltUpdate == 0) // wait for new ADC value to come in
        continue; 
     
    tick = DWT->CYCCNT; // Check Timelapse
    DWT->CYCCNT = 0;
    voltUpdate  = 0; // clear update flag
    
    
   
    voltInput  = 0.9f*voltInput+0.1f*(*adc_val3);
    voltCmd = voltInput/2.5f;// 200; // Fixed Voltage Tracking
    voltOutput = *adc_val1 - 2048; 
    voltError  = voltCmd - voltOutput; 
    voltErrorInt = voltErrorInt + voltError*tick/(72000000.0f); 
    
    if (voltOutput>1000)
    {
      snprintf(monitorBuf,50,"HV at Cout!\r\n"); 
      print_debug(monitorBuf);    
      change_phase_shift(0); 
      continue; 
    } 
     
    // current 
    currCmd = 100; 
    currOutput = *adc_val2 - 2048;
    currFiltered = 0.95f*currFiltered + 0.05f*currOutput; 
    currError = currCmd - currFiltered;
    currErrorInt = currErrorInt + currError*tick/(72000000.0f);
    
    // Anti-windup for current
    if (currErrorInt>currErrorIntSat)
    {
      currErrorInt = currErrorIntSat;
    }
    else if (currErrorInt<-currErrorIntSat)
    {
      currErrorInt = -currErrorIntSat;
    }
    
    // DcmdNew = (KpCurrent*currError + KiCurrent*currErrorInt)+Kpv*voltError; 
    DcmdNew = -48*voltErrorInt - 106*currErrorInt - 3*voltError+2*currError; 
    

    if (DcmdNew>590)
    {
      DcmdNew = 590;
    }
    else if (DcmdNew<20)
    {
      DcmdNew = 20;
    }  
    
    Dcmd = 0.9f*Dcmd+0.1f*DcmdNew; 
    
    change_duty_cycle(Dcmd);         
        
//    currSI = (currOutput-2048.0)/4096.0*50.0;
//    currSiFiltered = 0.9*currSiFiltered + 0.1*currSI; 
    
    // clamp integral error (anti windup)
    if (voltErrorInt>voltErrorIntSat)
    {   
      voltErrorInt = voltErrorIntSat;
    }
    else if (voltErrorInt<-voltErrorIntSat)
    {
      voltErrorInt = -voltErrorIntSat;  
    }
   
    //dphiCmdTick = (Kp*voltError + Ki*voltErrorInt) -  phiCmdTick ;   
    
    //phiCmdTick = 0.9f*phiCmdTick + 0.1f*(Kp*voltError + Ki*voltErrorInt); 
     phiCmdTick = 0.9f*phiCmdTick + 0.1f*(-2*voltErrorInt +38*currErrorInt);
    
    if (phiCmdTick>150)
    {
      //
      phiCmdTick=150;
    }
    else if (phiCmdTick<20)
    {
      phiCmdTick = 20 ;
    }
    //else
    //{
    //  HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_0);
    //}
//    else if (phiCmdTick<-10)
//    {
//      HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_SET);
//    }
//    else
//    {
//      HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_RESET);
//    }
    
     change_phase_shift(phiCmdTick); 
     // change_phase_shift(phiNom); 
     //phiNom = -phiNom; 
                         
//     if (abs(Dcmd-DcmdNew)> 30)
//     {
//         snprintf(monitorBuf,50,"Djump!!\r\n"); 
//         print_debug(monitorBuf);   
//     }
    
     if ((monitorCnt%300000)==0) 
     {
        // Vin  to  ADC: 19.7  <-> 620
        // Vout to  ADC: 11.27 <-> 170
        // Iout to  ADC: 1.12 <-> 83
//       snprintf(monitorBuf,50,"phi: %d, err_int %5.4f,tick: %d \r\n\n",phiCmdTick,voltErrorInt,tick);  
       snprintf(monitorBuf,100,"D: %d, phi: %d, Curr: %d, Vout %d, Vin: %d, VerrInt: %4.2f, IerrInt: %4.2f\r\n",Dcmd,phiCmdTick,currError, voltError,voltInput,voltErrorInt,currErrorInt); 
       print_debug(monitorBuf);    
       monitorCnt = 1;    
     } 
      monitorCnt++; 
      HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_0);
    
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
                              |RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_PLLCLK;
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
