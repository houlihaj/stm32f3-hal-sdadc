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
#include "lwrb/lwrb.h"  /* lwrb_t */
#include "printf.h"     /* snprintf_ */
#include <stdint.h>
#include <string.h>     /* size_t, strSNPRINTF_BUFFER_SIZE */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// C99: Define an enumerated type with typedef
typedef enum {
    SDADC1_CH5,
    SDADC3_CH8
} sdadc_inj_channels_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LWRB_SIZE               128
#define SDADC1_CH5_VALUE        5
#define SDADC3_CH8_VALUE        8
#define SNPRINTF_BUFFER_SIZE    64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SDADC_HandleTypeDef hsdadc1;  /* hsdadc1 is a handle to manage the SDADC1 */
SDADC_HandleTypeDef hsdadc3;  /* hsdadc3 is a handle to manage the SDADC3 */

TIM_HandleTypeDef htim15;  /* htim15 is a handle to manage the TIMER */

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/**
 * \brief
 */
char snprintf_buf[SNPRINTF_BUFFER_SIZE];

/**
 * \brief  Ring buffer instance for TX data
 */
lwrb_t sdadc1_ch5_rb;  // sdadc1_ch5
lwrb_t sdadc3_ch8_rb;  // sdadc3_ch8

/**
 * \brief  Ring buffer data array for TX DMA
 */
uint8_t sdadc1_ch5_rb_data[LWRB_SIZE];
uint8_t sdadc3_ch8_rb_data[LWRB_SIZE];

/**
* \brief  InjectedConvData contains the result of the conversion,
*         updated in HAL_SDADC_InjectedConvCpltCallback
*/
__IO int16_t InjectedConvData = 0;

/**
* \brief  InjChannel contains the converted channel after each
*         conversion. This is not used in this example.
*/
__IO uint32_t InjChannel = 0;

/**
  * @brief Text strings printed on PC Com port for user information
  */
char aTextInfoStart[] = "\r\nUSART Example : Enter characters to fill reception buffers.\r\n";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SDADC1_Init(void);
static void MX_SDADC3_Init(void);
static void MX_TIM15_Init(void);
/* USER CODE BEGIN PFP */
void my_buff_evt_fn(lwrb_t* buff, lwrb_evt_type_t type, size_t len);
void user_uart_println(char* msg);
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

    lwrb_init(&sdadc1_ch5_rb, sdadc1_ch5_rb_data, sizeof(sdadc1_ch5_rb_data));
    lwrb_set_evt_fn(&sdadc1_ch5_rb, my_buff_evt_fn);

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SDADC1_Init();
  MX_SDADC3_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */

  /* Print user info on PC com port */
  user_uart_println(aTextInfoStart);

    /* Start the TIMER's Channel */
    if (HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }

    /* Start Calibration in polling mode */
    if (HAL_SDADC_CalibrationStart(&hsdadc1, SDADC_CALIBRATION_SEQ_1) != HAL_OK)
    {
        Error_Handler();
    }

    /* Pool for the end of calibration */
    if (HAL_SDADC_PollForCalibEvent(&hsdadc1, HAL_MAX_DELAY) != HAL_OK)
    {
        Error_Handler();
    }

    /* Start injected conversion in interrupt mode */
    if (HAL_SDADC_InjectedStart_IT(&hsdadc1) != HAL_OK)
    {
        Error_Handler();
    }

    /* Start Calibration in polling mode */
    if (HAL_SDADC_CalibrationStart(&hsdadc3, SDADC_CALIBRATION_SEQ_1) != HAL_OK)
    {
        Error_Handler();
    }

    /* Pool for the end of calibration */
    if (HAL_SDADC_PollForCalibEvent(&hsdadc3, HAL_MAX_DELAY) != HAL_OK)
    {
        Error_Handler();
    }

    /* Start injected conversion in interrupt mode */
    if (HAL_SDADC_InjectedStart_IT(&hsdadc3) != HAL_OK)
    {
        Error_Handler();
    }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_SDADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.SdadcClockSelection = RCC_SDADCSYSCLK_DIV20;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_PWREx_EnableSDADC(PWR_SDADC_ANALOG1);
  HAL_PWREx_EnableSDADC(PWR_SDADC_ANALOG3);
}

/**
  * @brief SDADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDADC1_Init(void)
{

  /* USER CODE BEGIN SDADC1_Init 0 */

  /* USER CODE END SDADC1_Init 0 */

  SDADC_ConfParamTypeDef ConfParamStruct = {0};

  /* USER CODE BEGIN SDADC1_Init 1 */

  /* USER CODE END SDADC1_Init 1 */

  /** Configure the SDADC low power mode, fast conversion mode,
  slow clock mode and SDADC1 reference voltage
  */
  hsdadc1.Instance = SDADC1;
  hsdadc1.Init.IdleLowPowerMode = SDADC_LOWPOWER_NONE;
  hsdadc1.Init.FastConversionMode = SDADC_FAST_CONV_DISABLE;
  hsdadc1.Init.SlowClockMode = SDADC_SLOW_CLOCK_DISABLE;
  hsdadc1.Init.ReferenceVoltage = SDADC_VREF_EXT;
  hsdadc1.InjectedTrigger = SDADC_EXTERNAL_TRIGGER;
  hsdadc1.ExtTriggerEdge = SDADC_EXT_TRIG_RISING_EDGE;
  if (HAL_SDADC_Init(&hsdadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the Injected Mode
  */
  if (HAL_SDADC_SelectInjectedDelay(&hsdadc1, SDADC_INJECTED_DELAY_NONE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_SDADC_SelectInjectedExtTrigger(&hsdadc1, SDADC_EXT_TRIG_TIM15_CC2, SDADC_EXT_TRIG_RISING_EDGE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_SDADC_SelectInjectedTrigger(&hsdadc1, SDADC_EXTERNAL_TRIGGER) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_SDADC_InjectedConfigChannel(&hsdadc1, SDADC_CHANNEL_5, SDADC_CONTINUOUS_CONV_OFF) != HAL_OK)
  {
    Error_Handler();
  }

  /** Set parameters for SDADC configuration 0 Register
  */
  ConfParamStruct.InputMode = SDADC_INPUT_MODE_SE_ZERO_REFERENCE;
  ConfParamStruct.Gain = SDADC_GAIN_1;
  ConfParamStruct.CommonMode = SDADC_COMMON_MODE_VSSA;
  ConfParamStruct.Offset = 0;
  if (HAL_SDADC_PrepareChannelConfig(&hsdadc1, SDADC_CONF_INDEX_0, &ConfParamStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the Injected Channel
  */
  if (HAL_SDADC_AssociateChannelConfig(&hsdadc1, SDADC_CHANNEL_5, SDADC_CONF_INDEX_0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDADC1_Init 2 */

  /* USER CODE END SDADC1_Init 2 */

}

/**
  * @brief SDADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDADC3_Init(void)
{

  /* USER CODE BEGIN SDADC3_Init 0 */

  /* USER CODE END SDADC3_Init 0 */

  SDADC_ConfParamTypeDef ConfParamStruct = {0};

  /* USER CODE BEGIN SDADC3_Init 1 */

  /* USER CODE END SDADC3_Init 1 */

  /** Configure the SDADC low power mode, fast conversion mode,
  slow clock mode and SDADC1 reference voltage
  */
  hsdadc3.Instance = SDADC3;
  hsdadc3.Init.IdleLowPowerMode = SDADC_LOWPOWER_NONE;
  hsdadc3.Init.FastConversionMode = SDADC_FAST_CONV_DISABLE;
  hsdadc3.Init.SlowClockMode = SDADC_SLOW_CLOCK_DISABLE;
  hsdadc3.Init.ReferenceVoltage = SDADC_VREF_EXT;
  hsdadc3.InjectedTrigger = SDADC_SYNCHRONOUS_TRIGGER;
  if (HAL_SDADC_Init(&hsdadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the Injected Mode
  */
  if (HAL_SDADC_SelectInjectedDelay(&hsdadc3, SDADC_INJECTED_DELAY_NONE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_SDADC_SelectInjectedTrigger(&hsdadc3, SDADC_SYNCHRONOUS_TRIGGER) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_SDADC_InjectedConfigChannel(&hsdadc3, SDADC_CHANNEL_8, SDADC_CONTINUOUS_CONV_OFF) != HAL_OK)
  {
    Error_Handler();
  }

  /** Set parameters for SDADC configuration 0 Register
  */
  ConfParamStruct.InputMode = SDADC_INPUT_MODE_SE_ZERO_REFERENCE;
  ConfParamStruct.Gain = SDADC_GAIN_1;
  ConfParamStruct.CommonMode = SDADC_COMMON_MODE_VSSA;
  ConfParamStruct.Offset = 0;
  if (HAL_SDADC_PrepareChannelConfig(&hsdadc3, SDADC_CONF_INDEX_0, &ConfParamStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the Injected Channel
  */
  if (HAL_SDADC_AssociateChannelConfig(&hsdadc3, SDADC_CHANNEL_8, SDADC_CONF_INDEX_0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDADC3_Init 2 */

  /* USER CODE END SDADC3_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 12000;  // 12;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 4000;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 2000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
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
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * \brief  Buffer event function
 */
void my_buff_evt_fn(lwrb_t* buff, lwrb_evt_type_t type, size_t len)
{
    switch (type) {
        case LWRB_EVT_RESET:
            // printf("[EVT] Buffer reset event!\r\n");
            user_uart_println("[EVT] Buffer reset event!\r\n");
            break;
        case LWRB_EVT_READ:
            // printf("[EVT] Buffer read event: %d byte(s)!\r\n", (int)len);
            snprintf_(
                snprintf_buf, SNPRINTF_BUFFER_SIZE,
                "[EVT] Buffer read event: %d byte(s)!\r\n", (int)len
            );
            user_uart_println(snprintf_buf);
            break;
        case LWRB_EVT_WRITE:
            // printf("[EVT] Buffer write event: %d byte(s)!\r\n", (int)len);
            snprintf_(
                snprintf_buf, SNPRINTF_BUFFER_SIZE,
                "[EVT] Buffer write event: %d byte(s)!\r\n", (int)len
            );
            user_uart_println(snprintf_buf);
            break;
        default: break;
    }
}

/**
  * @brief  Send text information message on UART Tx line (to PC Com port).
  * @note
  * @param[in] msg  String to be sent to user display
  * @retval None
  */
void user_uart_println(char* msg)
{
    size_t len = strlen(msg);  /* bytes before '\0' */
    if (HAL_OK != HAL_UART_Transmit(&huart1, (uint8_t*)msg, (uint16_t)len, 100))
    {
        Error_Handler();
    }
}

/* --------------------- Callback ------------------------- */

/**
  * @brief  Injected conversion complete callback.
  * @note
  *
  * https://github.com/KiteBuilder/UvhPu/blob/5b00b3bfccb5a2864dbbb0c1755949325a77b525/user/app.cpp#L221
  * https://github.com/openpowerquality/opq/blob/d3d9f051b5096809b1e06d8f483fd4ff9dc861f2/box/Firmware/Src/opq.c
  * https://github.com/sk-co/rd-rs485/blob/266703887e98e99067b0443ddf21a2f88a80566c/src/boards/rd10/board-adc.cpp#L488
  *
  * @param[in]  hsdadc : SDADC handle.
  * @retval None
  */
void HAL_SDADC_InjectedConvCpltCallback(SDADC_HandleTypeDef *hsdadc)
{
    /* Get conversion value */
    InjectedConvData = HAL_SDADC_InjectedGetValue(hsdadc, (uint32_t *) &InjChannel);

    if (hsdadc == &hsdadc1)
    {
        if (InjChannel == SDADC1_CH5_VALUE) {
            snprintf_(
                snprintf_buf, SNPRINTF_BUFFER_SIZE,
                "hsdadc1; ch: %ld; data: %d\r\n", InjChannel, InjectedConvData
            );
        } else {
            snprintf_(
                snprintf_buf, SNPRINTF_BUFFER_SIZE,
                "hsdadc1; ch: %ld; data: %d\r\n", InjChannel, InjectedConvData
            );
        }
        lwrb_write(&sdadc1_ch5_rb, "1", 1);
    }
    else if (hsdadc == &hsdadc3)
    {
        if (InjChannel == SDADC3_CH8_VALUE) {
            snprintf_(
                snprintf_buf, SNPRINTF_BUFFER_SIZE,
                "hsdadc3; ch: %ld; data: %d\r\n", InjChannel, InjectedConvData
            );
        }
        else {
            snprintf_(
                snprintf_buf, SNPRINTF_BUFFER_SIZE,
                "hsdadc3; ch: %ld; data: %d\r\n", InjChannel, InjectedConvData
            );
        }
        // lwrb_write(&sdadc3_ch8_rb, "3", 1);
    }

    user_uart_println(snprintf_buf);
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
