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
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "hrtim.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "OLED.h"
#include "Control.h"
#include "Sample.h"
#include "AS5048_SPI.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern CurrentSample_t current_sample;  //三相电流采样值，单位[A]，在ADC2注入转换完成回调中更新
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
AS5048A_HandleTypeDef as5048a;   //磁编码器句柄，TIM1 PWM输入捕获（PC0）
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//浮点数转定点字符串，例如 Fx(buf, -1.2345f, 3) -> "-1.235"
//不依赖printf的%f，避免OLED上浮点显示被截断
static char *Fx(char *buf, float v, uint8_t dec)
{
  static const int32_t p10[4] = {1, 10, 100, 1000};
  int32_t s;
  char sign = (v < 0.0f) ? '-' : '+';

  if (dec > 3U) dec = 3U;
  s = (int32_t)(fabsf(v) * (float)p10[dec] + 0.5f);
  if (dec == 0U)
  {
    sprintf(buf, "%c%ld", sign, (long)s);
  }
  else
  {
    sprintf(buf, "%c%ld.%0*ld", sign, (long)(s / p10[dec]), (int)dec, (long)(s % p10[dec]));
  }
  return buf;
}
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_HRTIM1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_FDCAN2_Init();
  MX_I2C1_Init();
  MX_UART4_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  AS5048A_Init(&as5048a, &htim1);
  if (AS5048A_Start(&as5048a) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  OLED_Init();
  Driver_Init();
  Driver_Enable();   //对齐->校验->电压模式FOC，在ADC2注入转换完成回调中执行（Control.c）

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //FOC调试显示（6x8字体，8行）
    {
      char a[16], b[16];
      OLED_Clear();
      OLED_Printf(0,  0, OLED_6X8, "S:%d F:%d dir:%d", (int)foc_dbg.state, (int)foc_dbg.fault, (int)foc_dbg.dir);
      OLED_Printf(0,  8, OLED_6X8, "pole:%s n:%lu", Fx(a, foc_dbg.pole_ratio, 2), (unsigned long)(foc_dbg.isr_cnt / 20000U));
      OLED_Printf(0, 16, OLED_6X8, "m0r:%s m0:%s", Fx(a, foc_dbg.theta_m0_raw, 3), Fx(b, foc_dbg.theta_m0, 3));
      OLED_Printf(0, 24, OLED_6X8, "fw:%s rv:%s", Fx(a, foc_dbg.diff_fwd, 3), Fx(b, foc_dbg.diff_rev, 3));
      OLED_Printf(0, 32, OLED_6X8, "avg:%s sp:%s", Fx(a, foc_dbg.diff_mean, 3), Fx(b, foc_dbg.diff_span, 3));
      OLED_Printf(0, 40, OLED_6X8, "id:%s iq:%s", Fx(a, foc_dbg.id, 3), Fx(b, foc_dbg.iq, 3));
      OLED_Printf(0, 48, OLED_6X8, "rpm:%s uq:%s", Fx(a, foc_dbg.rpm, 1), Fx(b, foc_uq, 1));
      OLED_Printf(0, 56, OLED_6X8, "th:%s e:%lu", Fx(a, foc_dbg.theta_m, 3), (unsigned long)as5048a.error_count);
    }
    OLED_Update();
    HAL_Delay(100);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//TIM1输入捕获回调：每个PWM上升沿进入一次，解码上一帧角度
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  AS5048A_CaptureCallback(&as5048a, htim);
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
