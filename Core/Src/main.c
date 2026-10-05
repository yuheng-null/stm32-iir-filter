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
#include "dac.h"
#include "dma.h"
#include "gpio.h"
#include "tim.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "filter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ---- 系统级常数：全部工作在12bit ADC/DAC码值域（满量程3.3V，每码约0.806mV）---- */
#define IN_BIAS_CODE 993.0f  /* 输入直流偏置 0.8V -> 993码 */
#define FILTER_DC_GAIN 2.0f  /* 低通滤波器直流增益 */
#define OUT_BIAS_CODE 1986U  /* 上电预置输出 1.6V -> 1986码 = IN_BIAS_CODE x FILTER_DC_GAIN */
#define ADC_CODE_MAX 4095.0f /* 12bit码值上限，用于滤波输出限幅 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* DMA 单样本缓冲：每个采样点由 DMA1_CH1 循环写入、在中断中读取，
   必须 volatile，防止编译器在主循环/中断之间把它优化进寄存器；
   static 将可见性限制在本文件（仅 main() 与滤波回调访问） */
static volatile uint16_t adc_raw;

/* 二阶低通，Fs=50kHz 锁定系数（a1 自带负号，禁止再变号） */
static biquad_t lpf = {.b0 = 0.0152672f,
                       .b1 = 0.0305344f,
                       .b2 = 0.0152672f,
                       .a1 = -1.5114504f,
                       .a2 = 0.5419847f,
                       .s1 = 0.0f,
                       .s2 = 0.0f};

/* 输出增益微调：1.0=不修正；若要补偿输出重建RC在2kHz的~0.78%衰减可略调大 */
#define GAIN_TRIM 1.0f
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  /* 1) 启动 DAC 通道1，先把输出预置到 1.6V(OUT_BIAS_CODE)，避免上电瞬间从0跳变 */
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, OUT_BIAS_CODE);

  /* 2) 滤波器直接置为 0.8V输入->1.6V输出的直流稳态，消除建立瞬态 */
  biquad_preset_dc(&lpf, IN_BIAS_CODE, FILTER_DC_GAIN);

  /* 3) 启动 ADC+DMA：DMA 循环搬运单个半字到 adc_raw，每次转换完成触发TC中断 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adc_raw, 1);

  /* 4) 启动 TIM3：更新事件(TRGO)以 72e6/(0+1)/(1439+1)=50kHz 触发 ADC 转换。
        CubeMX
     只做初始化、不启动计数器，这一行必须由用户代码补上，否则无采样节拍 */
  HAL_TIM_Base_Start(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 存活指示：每500ms翻转一次PC13，亮灭各0.5s，周期1s。
       若它停止闪烁，说明主循环被中断长期占用（ISR超时/采样率过高） */
    HAL_GPIO_TogglePin(LED_ALIVE_GPIO_Port, LED_ALIVE_Pin);
    HAL_Delay(500);
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @brief  规则组转换完成回调（DMA 搬完一个样本即被 HAL 调用，
 *         运行在 DMA1_Channel1 全局中断上下文，50kHz 进一次）
 * 调用链：DMA1_Channel1_IRQHandler -> HAL_DMA_IRQHandler ->
 *         ADC_DMAConvCplt -> HAL_ADC_ConvCpltCallback（本函数），无需手写ISR
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  /* 只响应 ADC1（本工程也只有 ADC1，判断是防御性写法） */
  if (hadc->Instance != ADC1) {
    return;
  }

  /* PB0 置高：示波器量此高电平宽度即单次ISR运算耗时 */
  HAL_GPIO_WritePin(DBG_ISR_GPIO_Port, DBG_ISR_Pin, GPIO_PIN_SET);

  /* 码值域直接滤波，再乘增益微调 */
  float y = biquad_update(&lpf, (float)adc_raw) * GAIN_TRIM;

  /* 限幅到12bit码值域[0,ADC_CODE_MAX]，保证不削顶、不出现非法码 */
  if (y > ADC_CODE_MAX) {
    y = ADC_CODE_MAX;
  } else if (y < 0.0f) {
    y = 0.0f;
  }

  /* 四舍五入(+0.5后截断)为整数码，写DAC通道1的12bit右对齐保持寄存器DHR12R1 */
  uint16_t dac_code = (uint16_t)(y + 0.5f);
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_code);

  /* PB0 清低，结束观测脉宽 */
  HAL_GPIO_WritePin(DBG_ISR_GPIO_Port, DBG_ISR_Pin, GPIO_PIN_RESET);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
