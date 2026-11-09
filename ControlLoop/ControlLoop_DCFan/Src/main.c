/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "fan_controller.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum { TEST_HOLD, TEST_COMPARE, TEST_CHARACTERIZE } FanTestPlan;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* USER SETTINGS: target, mode and experiment can also be changed in the debugger.
   TEST_COMPARE runs all four modes once; TEST_HOLD uses controllerMode. */
volatile float targetRpm = 3500.0f;
volatile FanControlMode controllerMode = FAN_MODE_PID;
volatile FanTestPlan testPlan = TEST_COMPARE;

/* Starting gains, not hardware-tuned. Duty is percent, time is seconds.
   Existing log, after 3 s settling: ~4094 RPM at 100%; Kplant ~40.94 RPM/percent.
   Kp = 0.5/Kplant; Ki = Kp/2 seconds; Kd = Kp*0.05 seconds. */
static const float Kp = 0.0122f;
static const float Ki = 0.0061f;
static const float Kd = 0.00061f;
static const float derivativeFilterSeconds = 0.20f;
static const bool useFeedforward = true;
static const uint32_t controlPeriodMs = 100U;

/* Provisional endpoint estimate. Replace with measured sweep points. */
static const FanMapPoint openLoopMap[] = {{0.0f, 0.0f}, {4093.65f, 100.0f}};
static const float stepFractions[] = {0.0f, 1.0f, 0.7f, 1.0f, 0.0f};
static const uint32_t stepDurationsMs[] = {5000U, 10000U, 10000U, 10000U, 5000U};
static const float calibrationDuties[] = {0.0f, 20.0f, 35.0f, 50.0f,
                                         65.0f, 80.0f, 100.0f, 0.0f};
static const uint32_t calibrationDwellMs = 8000U;

/* The fan produces two tach pulses per revolution. */
static volatile uint32_t tachPulseCount = 0U;
static volatile uint32_t lastTachPulseMs = 0U;
static FanController fanController;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static float ReadFanRpm(uint32_t *sampleTime)
{
  static uint32_t previousCount = 0U;
  static uint32_t previousSampleMs = 0U;
  static uint32_t previousEdgeMs = 0U;
  static bool previousEdgeValid = false;

  uint32_t interruptMask = __get_PRIMASK();
  __disable_irq();
  uint32_t now = HAL_GetTick();
  uint32_t count = tachPulseCount;
  uint32_t edgeMs = lastTachPulseMs;
  __set_PRIMASK(interruptMask);

  uint32_t pulses = count - previousCount;
  uint32_t elapsed = now - previousSampleMs;
  /* Keep pulse counting. Align the time span to the last edge of each
     window to reduce the old +/-one-pulse (~300 RPM) quantization. */
  if (previousEdgeValid && (pulses > 0U) && ((edgeMs - previousEdgeMs) > 0U))
  {
    elapsed = edgeMs - previousEdgeMs;
  }
  float rpm = elapsed > 0U ? (float)pulses * 60000.0f / (2.0f * (float)elapsed) : 0.0f;
  previousCount = count;
  previousSampleMs = now;
  previousEdgeMs = edgeMs;
  previousEdgeValid = pulses > 0U;
  *sampleTime = now;
  return rpm;
}

static uint32_t ComparisonDurationMs(void)
{
  uint32_t duration = 0U;
  for (size_t i = 0U; i < sizeof(stepDurationsMs) / sizeof(stepDurationsMs[0]); i++)
  {
    duration += stepDurationsMs[i];
  }
  return duration;
}

static float ComparisonTarget(uint32_t elapsed, float target)
{
  for (size_t i = 0U; i < sizeof(stepFractions) / sizeof(stepFractions[0]); i++)
  {
    if (elapsed < stepDurationsMs[i])
    {
      return target * stepFractions[i];
    }
    elapsed -= stepDurationsMs[i];
  }
  return 0.0f;
}

static float ApplyFanDuty(float duty)
{
  if (duty < 0.0f) { duty = 0.0f; }
  if (duty > 100.0f) { duty = 100.0f; }
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim2) + 1U;
  uint32_t compare = (uint32_t)((float)period * duty / 100.0f + 0.5f);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, compare);
  return (float)compare * 100.0f / (float)period;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint32_t nextControlTime;
  uint32_t experimentStartTime;
  FanTestPlan activePlan;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  const FanControllerConfig controllerConfig = {
    .sample_seconds = (float)controlPeriodMs / 1000.0f,
    .kp = Kp, .ki = Ki, .kd = Kd,
    .derivative_tau = derivativeFilterSeconds,
    .duty_min = 0.0f, .duty_max = 100.0f,
    .use_feedforward = useFeedforward,
    .map = openLoopMap,
    .map_count = sizeof(openLoopMap) / sizeof(openLoopMap[0])
  };
  if (!FanController_Init(&fanController, &controllerConfig))
  {
    Error_Handler();
  }
  /* Preserve the verified PWM channel and input-capture interrupt. */
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0U);
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  experimentStartTime = HAL_GetTick();
  nextControlTime = experimentStartTime + controlPeriodMs;
  activePlan = testPlan;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    if ((int32_t)(now - nextControlTime) >= 0)
    {
      /* Deadline advances by Ts, not by logging time. Never run catch-up
         updates on the same measurement after a missed control period. */
      /* Allow millisecond tick jitter, but never follow a very late sample
         with another control update only a few milliseconds later. */
      bool missedPeriod = (now - nextControlTime) > 5U;
      if (missedPeriod)
      {
        FanController_Reset(&fanController);
        nextControlTime = now;
      }
      nextControlTime += controlPeriodMs;
      float rpm = ReadFanRpm(&now);
      if (missedPeriod)
      {
        /* Refresh the measurement baseline, hold the last PWM output, and
           wait for a fresh Ts window instead of integrating stale data. */
        continue;
      }

      if (activePlan != testPlan)
      {
        activePlan = testPlan;
        experimentStartTime = now;
        FanController_Reset(&fanController);
      }
      uint32_t elapsed = now - experimentStartTime;
      FanControlMode mode = controllerMode;
      float requestedRpm = targetRpm;
      float duty = 0.0f;
      bool done = false;
      const char *modeName;

      if (activePlan == TEST_COMPARE)
      {
        uint32_t duration = ComparisonDurationMs();
        uint32_t modeIndex = elapsed / duration;
        done = modeIndex >= FAN_MODE_COUNT;
        mode = done ? FAN_MODE_PID : (FanControlMode)modeIndex;
        requestedRpm = done ? 0.0f : ComparisonTarget(elapsed % duration, targetRpm);
      }
      else if (activePlan == TEST_CHARACTERIZE)
      {
        uint32_t step = elapsed / calibrationDwellMs;
        done = step >= sizeof(calibrationDuties) / sizeof(calibrationDuties[0]);
        duty = done ? 0.0f : calibrationDuties[step];
        requestedRpm = 0.0f;
        mode = FAN_MODE_OPEN_LOOP;
      }
      else if (activePlan != TEST_HOLD)
      {
        done = true;
        requestedRpm = 0.0f;
      }

      if (!FanController_SetMode(&fanController, mode))
      {
        done = true;
        requestedRpm = 0.0f;
      }
      if (done)
      {
        FanController_Reset(&fanController);
        duty = 0.0f;
        modeName = "DONE";
      }
      else if (activePlan == TEST_CHARACTERIZE)
      {
        modeName = "CALIBRATION";
      }
      else
      {
        duty = FanController_Update(&fanController, requestedRpm, rpm);
        modeName = FanController_ModeName(mode);
      }
      duty = ApplyFanDuty(duty);

      /* CSV: time_ms,target_rpm,actual_rpm,pwm_duty,control_mode.
         Integer formatting avoids enabling printf's floating-point library. */
      uint32_t loggedTarget = requestedRpm > 0.0f ? (uint32_t)(requestedRpm + 0.5f) : 0U;
      uint32_t dutyTenths = (uint32_t)(duty * 10.0f + 0.5f);
      char message[96];
      int length = snprintf(message, sizeof(message), "%lu,%lu,%lu,%lu.%lu,%s\r\n",
                            (unsigned long)elapsed, (unsigned long)loggedTarget,
                            (unsigned long)(rpm + 0.5f),
                            (unsigned long)(dutyTenths / 10U),
                            (unsigned long)(dutyTenths % 10U), modeName);
      if ((length > 0) && ((size_t)length < sizeof(message)))
      {
        HAL_UART_Transmit(&huart2, (const uint8_t *)message, (uint16_t)length, 20U);
      }
    }
    HAL_Delay(1U);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 639;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if ((htim->Instance == TIM1) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
  {
    tachPulseCount++;
    lastTachPulseMs = HAL_GetTick();
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
