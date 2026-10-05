/*
 * 烧录前离线验证（不依赖芯片，纯主机 C）
 * 直接编译板上同一份 Core/Src/filter.c，保证验证逻辑与固件同源：
 *   gcc -std=c11 -I Core/Inc verify/test_filter.c Core/Src/filter.c -lm -o verify/test_filter
 *   verify/test_filter.exe        (Windows)
 *   ./verify/test_filter          (Linux/macOS)
 *
 * 输入：码值域正弦 x = 993 + 930.7*sin(2*pi*f*n/50000)（即0.8V偏置、1.5Vpp）
 * 输出：均值电压、Vpp、码范围、直流增益、基波相位，并自动对照锚点判定 PASS/FAIL
 */
#include <stdio.h>
#include <math.h>
#include "filter.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FS        50000.0
#define RUN_SEC   2.0                 /* 总仿真时长 */
#define WIN_SEC   0.5                 /* 取最后0.5s作为稳态统计窗 */
#define VPERCODE  (3.3 / 4096.0)      /* 每码电压 */
#define DC_CODE   993.0f              /* 0.8V 偏置码 */
#define AMP_CODE  930.7               /* 0.75V 幅度码(1.5Vpp) */

/* 单次交流扫描：返回稳态统计 */
static void sweep_ac(double fin, double *mean_code, double *vpp,
                     double *ymin, double *ymax, double *phase_deg)
{
  /* 50k 锁定系数，a1 自带负号 */
  biquad_t f = {0.0152672f, 0.0305344f, 0.0152672f,
                -1.5114504f, 0.5419847f, 0.0f, 0.0f};
  /* 与固件一致：先置0.8V直流稳态，消除建立瞬态 */
  biquad_preset_dc(&f, DC_CODE, 2.0f);

  int n_total = (int)(FS * RUN_SEC);
  int n_win   = (int)(FS * WIN_SEC);
  int n_start = n_total - n_win;

  double mn = 1e9, mx = -1e9, sum = 0.0, cs = 0.0, cc = 0.0;
  for (int n = 0; n < n_total; n++)
  {
    double theta = 2.0 * M_PI * fin * n / FS;
    float x = (float)(DC_CODE + AMP_CODE * sin(theta));
    float y = biquad_update(&f, x);
    if (n >= n_start)
    {
      if (y < mn) mn = y;
      if (y > mx) mx = y;
      sum += y;
      cs += (double)y * sin(theta);
      cc += (double)y * cos(theta);
    }
  }
  *mean_code = sum / n_win;
  *ymin = mn;
  *ymax = mx;
  *vpp = (mx - mn) * VPERCODE;
  /* 去掉直流分量再做 I/Q 相关，求基波相位（相对输入 sin） */
  double cs_ac = 0.0, cc_ac = 0.0;
  /* 重新跑一遍做去均值相关（上面 cs/cc 含直流，这里扣除直流项） */
  biquad_t g = {0.0152672f, 0.0305344f, 0.0152672f,
                -1.5114504f, 0.5419847f, 0.0f, 0.0f};
  biquad_preset_dc(&g, DC_CODE, 2.0f);
  for (int n = 0; n < n_total; n++)
  {
    double theta = 2.0 * M_PI * fin * n / FS;
    float x = (float)(DC_CODE + AMP_CODE * sin(theta));
    float y = biquad_update(&g, x);
    if (n >= n_start)
    {
      cs_ac += ((double)y - *mean_code) * sin(theta);
      cc_ac += ((double)y - *mean_code) * cos(theta);
    }
  }
  *phase_deg = atan2(cc_ac / n_win, cs_ac / n_win) * 180.0 / M_PI;
}

/* 直流增益测试：恒定993输入，稳态输出/输入 */
static double dc_gain_test(void)
{
  biquad_t f = {0.0152672f, 0.0305344f, 0.0152672f,
                -1.5114504f, 0.5419847f, 0.0f, 0.0f};
  biquad_preset_dc(&f, DC_CODE, 2.0f);
  float y = 0.0f;
  for (int n = 0; n < 100000; n++) y = biquad_update(&f, DC_CODE);
  return (double)y / DC_CODE;
}

int main(void)
{
  int pass = 1;
  printf("==== IIR offline check, Fs=%.0f Hz, code-domain (0..4095) ====\n\n", FS);

  double g = dc_gain_test();
  printf("[DC ] gain = %.6f  (target 2.0000)\n", g);
  if (fabs(g - 2.0) > 1e-4) { printf("     FAIL\n"); pass = 0; } else printf("     PASS\n");

  struct { double f, vpp_lo, vpp_hi, rmin_lo, rmin_hi, rmax_lo, rmax_hi, ph_lo, ph_hi; const char *tag; }
  cases[2] = {
    /* 锚点：1k Vpp 1.49~1.53V；码范围约1048~2923；相位-72.2 */
    {1000.0, 1.49, 1.53, 1020, 1080, 2890, 2960, -75, -69, "1kHz"},
    /* 锚点：2k Vpp 0.76~0.80V；码范围约1502~2470；相位-98.7 */
    {2000.0, 0.76, 0.80, 1470, 1530, 2440, 2500, -102, -95, "2kHz"},
  };

  for (int i = 0; i < 2; i++)
  {
    double mean, vpp, ymin, ymax, ph;
    sweep_ac(cases[i].f, &mean, &vpp, &ymin, &ymax, &ph);
    int ok = 1;
    printf("[%s] mean=%.4f V  Vpp=%.4f V  range=[%.0f .. %.0f] code  phase=%.2f deg\n",
           cases[i].tag, mean * VPERCODE, vpp, ymin, ymax, ph);
    if (fabs(mean * VPERCODE - 1.60) > 0.01) { printf("     mean FAIL (want 1.60V)\n"); ok = 0; }
    if (!(vpp >= cases[i].vpp_lo && vpp <= cases[i].vpp_hi)) { printf("     Vpp FAIL (want %.2f~%.2f)\n", cases[i].vpp_lo, cases[i].vpp_hi); ok = 0; }
    if (!(ymin >= cases[i].rmin_lo && ymin <= cases[i].rmin_hi)) { printf("     min code FAIL\n"); ok = 0; }
    if (!(ymax >= cases[i].rmax_lo && ymax <= cases[i].rmax_hi)) { printf("     max code FAIL\n"); ok = 0; }
    if (!(ph >= cases[i].ph_lo && ph <= cases[i].ph_hi)) { printf("     phase FAIL (want around anchor)\n"); ok = 0; }
    if (ymin <= 0 || ymax >= 4095) { printf("     CLIP FAIL\n"); ok = 0; }
    printf("     %s\n\n", ok ? "PASS" : "FAIL");
    if (!ok) pass = 0;
  }

  printf("==== OVERALL: %s ====\n", pass ? "PASS (allow flashing)" : "FAIL (do NOT flash)");
  return pass ? 0 : 1;
}
