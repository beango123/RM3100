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
#include "rm3100.h"
#include <stdio.h>

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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static RM3100_t rm3100[2] = {
  {RM3100_CS1_GPIO_Port, RM3100_CS1_Pin,
   RM3100_DRDY1_GPIO_Port, RM3100_DRDY1_Pin, 200U},
  {RM3100_CS2_GPIO_Port, RM3100_CS2_Pin,
   RM3100_DRDY2_GPIO_Port, RM3100_DRDY2_Pin, 200U}
};
static uint32_t sample_seq = 0U;
static uint32_t boot_reset_csr = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void DWT_MicrosInit(void);
static uint32_t Micros(void);
static void SendSample(uint32_t seq, uint32_t t1, uint32_t t2,
                       uint8_t ready_mask,
                       const RM3100_Vector_t *v1,
                       const RM3100_Vector_t *v2);
static void SendDebugInit(uint8_t sensor, HAL_StatusTypeDef status);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* 调试信息使用 DBG 前缀，不会被电脑端 RM, 数据帧解析。 */
static void SendDebugInit(uint8_t sensor, HAL_StatusTypeDef status)
{
  const RM3100_t *s = &rm3100[sensor - 1U];
  char text[128];
  int length = snprintf(text, sizeof(text),
                        "DBG,INIT%u,HAL=%u,SPIERR=0x%08lX,CS=%u,DRDY=%u\r\n",
                        (unsigned)sensor, (unsigned)status,
                        (unsigned long)hspi1.ErrorCode,
                        (unsigned)HAL_GPIO_ReadPin(s->cs_port, s->cs_pin),
                        (unsigned)HAL_GPIO_ReadPin(s->drdy_port, s->drdy_pin));
  if (length > 0 && (size_t)length < sizeof(text))
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)text,
                            (uint16_t)length, 100U);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* 保存复位来源后再清标志，便于区分上电、NRST 和软件/看门狗复位。 */
  boot_reset_csr = RCC->CSR;
  __HAL_RCC_CLEAR_RESET_FLAGS();

  /* USER CODE BEGIN 1 */

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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  DWT_MicrosInit();
  HAL_Delay(10U);
  {
    HAL_StatusTypeDef init2 = RM3100_Init(&hspi1, &rm3100[1], 200U);

    SendDebugInit(2U, init2);

    if (init2 != HAL_OK)
    {
      static const uint8_t err[] = "ERR,RM3100_INIT2\r\n";
      (void)HAL_UART_Transmit(&huart1, (uint8_t *)err, sizeof(err) - 1U, 100U);
      Error_Handler();
    }
  }
  {
    static const uint8_t banner[] = "RM3100,1,raw24,us\r\n";
    char boot_info[32];
    int boot_len = snprintf(boot_info, sizeof(boot_info),
                            "BOOT,CSR=0x%08lX\r\n",
                            (unsigned long)boot_reset_csr);
    if (boot_len > 0 && (size_t)boot_len < sizeof(boot_info))
      (void)HAL_UART_Transmit(&huart1, (uint8_t *)boot_info,
                              (uint16_t)boot_len, 100U);
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)banner,
                            sizeof(banner) - 1U, 100U);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    RM3100_Vector_t v1 = {0, 0, 0};
    RM3100_Vector_t v2 = {0, 0, 0};
    uint8_t ready_mask = 0U;
    uint32_t trigger2_us;

    trigger2_us = Micros();
    if (RM3100_StartSingle(&hspi1, &rm3100[1]) != HAL_OK) Error_Handler();

    if (RM3100_WaitReady(&rm3100[1], 100U) == HAL_OK &&
        RM3100_ReadXYZ(&hspi1, &rm3100[1], &v2) == HAL_OK)
      ready_mask |= 0x02U;

    SendSample(sample_seq++, 0U, trigger2_us,
               ready_mask, &v1, &v2);
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RM3100_CS1_Pin|RM3100_CS2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : RM3100_CS1_Pin RM3100_CS2_Pin */
  GPIO_InitStruct.Pin = RM3100_CS1_Pin|RM3100_CS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RM3100_DRDY1_Pin RM3100_DRDY2_Pin */
  GPIO_InitStruct.Pin = RM3100_DRDY1_Pin|RM3100_DRDY2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void DWT_MicrosInit(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t Micros(void)
{
  return DWT->CYCCNT / (HAL_RCC_GetHCLKFreq() / 1000000U);
}

static void SendSample(uint32_t seq, uint32_t t1, uint32_t t2,
                       uint8_t ready_mask,
                       const RM3100_Vector_t *v1,
                       const RM3100_Vector_t *v2)
{
  char payload[128];
  char frame[140];
  int length;
  uint8_t checksum = 0U;

  length = snprintf(payload, sizeof(payload), "RM,%lu,%lu,%lu,%u,"
                    "%ld,%ld,%ld,%ld,%ld,%ld",
                    (unsigned long)seq, (unsigned long)t1, (unsigned long)t2,
                    (unsigned)ready_mask,
                    (ready_mask & 0x01U) ? (long)v1->x : 0L,
                    (ready_mask & 0x01U) ? (long)v1->y : 0L,
                    (ready_mask & 0x01U) ? (long)v1->z : 0L,
                    (ready_mask & 0x02U) ? (long)v2->x : 0L,
                    (ready_mask & 0x02U) ? (long)v2->y : 0L,
                    (ready_mask & 0x02U) ? (long)v2->z : 0L);
  if (length <= 0 || (size_t)length >= sizeof(payload)) return;
  for (int i = 0; i < length; ++i) checksum ^= (uint8_t)payload[i];
  int frame_length = snprintf(frame, sizeof(frame), "%s*%02X\r\n",
                              payload, checksum);
  if (frame_length > 0 && (size_t)frame_length < sizeof(frame))
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)frame,
                            (uint16_t)frame_length, 100U);
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
