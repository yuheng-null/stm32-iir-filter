/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    filter.h
  * @brief   二阶 IIR 双二阶节（转置 II 型 / Direct Form II Transposed）
  *          在 ADC 码值域(0~4095)直接运算，不换算电压。
  ******************************************************************************
  */
/* USER CODE END Header */

/* 头文件保护 ---------------------------------------------------------------*/
#ifndef __FILTER_H
#define __FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 二阶节结构体 --------------------------------------------------------------*/
typedef struct
{
  /* 分子系数（前向支路） */
  float b0;
  float b1;
  float b2;
  /* 分母系数（反馈支路）。注意：a1 自带负号（锁定值 a1=-1.5114504），
     差分方程中写作 -a1*y，使用时禁止再对 a1 取一次反。 */
  float a1;
  float a2;
  /* 转置 II 型内部状态（对应两级延迟单元） */
  float s1;
  float s2;
} biquad_t;

/**
  * @brief  输入一个样本，推进一阶差分并返回滤波输出（码值域浮点）
  *         差分方程（锁定，顺序不可调换）：
  *           y  = b0*x + s1
  *           s1 = b1*x - a1*y + s2
  *           s2 = b2*x - a2*y
  * @param  f: 二阶节指针
  * @param  x: 输入样本（ADC 原始码值）
  * @retval 滤波输出（码值域浮点）
  */
float biquad_update(biquad_t *f, float x);

/**
  * @brief  直接把内部状态置为“直流输入 x_dc、直流增益 dc_gain”对应的稳态，
  *         用于上电时消除对 0.8V 偏置的建立瞬态。
  *         稳态关系：y = dc_gain*x_dc
  *                   s1 = y - b0*x
  *                   s2 = b2*x - a2*y
  * @param  f:       二阶节指针
  * @param  x_dc:    直流输入码值（本项目 993，对应 0.8V）
  * @param  dc_gain: 直流增益（本项目 2.0）
  */
void biquad_preset_dc(biquad_t *f, float x_dc, float dc_gain);

#ifdef __cplusplus
}
#endif

#endif /* __FILTER_H */
