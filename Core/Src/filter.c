/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    filter.c
  * @brief   二阶 IIR 转置 II 型实现（码值域运算）
  ******************************************************************************
  */
/* USER CODE END Header */

#include "filter.h"

/* 单样本更新：严格按锁定差分方程，a1 自带负号，不再变号 ---------------------*/
float biquad_update(biquad_t *f, float x)
{
  /* 当前输出 = 前向 b0 支路 + 第一级延迟状态 */
  float y = f->b0 * x + f->s1;

  /* 更新两级延迟状态（必须先算 s1 再算 s2，s1 的新值不影响 s2，二者都依赖本次 y） */
  f->s1 = f->b1 * x - f->a1 * y + f->s2;
  f->s2 = f->b2 * x - f->a2 * y;

  return y;
}

/* 直接置直流稳态，消除上电瞬态 ---------------------------------------------*/
void biquad_preset_dc(biquad_t *f, float x_dc, float dc_gain)
{
  float y = dc_gain * x_dc;   /* 直流稳态输出码值 */

  f->s1 = y - f->b0 * x_dc;
  f->s2 = f->b2 * x_dc - f->a2 * y;
}
