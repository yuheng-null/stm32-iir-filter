#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
滤波器系数设计脚本：模拟原型 H(s) -> 离散 Biquad 系数（双线性变换）

设计目标（二阶低通模拟原型）：
                 2                       G*w0^2
    H(s) = ------------------- = ----------------------
           1e-8*s^2 + 3e-4*s + 1    s^2 + (w0/Q)*s + w0^2

    G  = 2            直流增益（+6.02 dB）
    w0 = 1e4 rad/s    特征角频率，f0 = w0/(2*pi) ≈ 1591.5 Hz
    Q  = 1/3          过阻尼（Q < 0.707），两个实极点，无谐振峰

对应的模拟参考实现：Sallen-Key 二阶低通 + 比例放大级联（硬件电路或电路仿真）。
本项目在 STM32 上以 50 kHz 采样实时复现该传递函数（数字域模拟电路仿真器）。

离散化方法：双线性变换 s = c*(1-z^-1)/(1+z^-1)，c = 2*Fs（不预畸变）
  1) 因式分解：过阻尼二阶节 = 两个一阶低通级联，H(s) = G * Π wc_i/(s + wc_i)
  2) 逐节映射：p_i = (c-wc_i)/(c+wc_i)，K_i = wc_i/(c+wc_i) = (1-p_i)/2
     H_i(z) = K_i*(1+z^-1)/(1-p_i*z^-1)，每节直流增益恒为 1
  3) 级联乘 G，合并为单个 Biquad（固件按转置直接 II 型实现）：
     b = G*K1*K2*[1, 2, 1]，a = [1, -(p1+p2), p1*p2]

脚本输出：
  - 模拟原型参数（f0、Q、极点）
  - 设计得到的 Biquad 系数，与固件锁定值（Core/Src/main.c）逐位对照 PASS/FAIL
  - 1k/2kHz 处 模拟 H(jW) 与 离散 H(e^jw) 的幅相误差（验收指标：Vpp 误差 ≤ 10%）
  - 可直接粘贴进固件的 C 初始化片段

运行：python tools/design_filter.py
"""
import cmath
import math
import sys

# Windows 控制台默认编码可能不是 UTF-8，显式重配防止中文输出崩溃
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

# ---- 模拟原型：H(s) = NUM_G / (A2*s^2 + A1*s + A0) ----
NUM_G = 2.0
A2, A1, A0 = 1e-8, 3e-4, 1.0

# ---- 离散化参数 ----
FS = 50000.0          # 采样率，与固件 TIM3 配置一致（72 MHz / 1440）

# ---- 固件锁定系数（Core/Src/main.c 中 lpf 初始化，a1 自带负号）----
FIRMWARE = {"b0": 0.0152672, "b1": 0.0305344, "b2": 0.0152672,
            "a1": -1.5114504, "a2": 0.5419847}
TOL = 2e-7            # 固件系数按 7 位小数舍入，容差取 2e-7

# ---- 验收指标（与模拟参考对照）----
VPP_TOL = 0.10        # 输出 Vpp 相对误差 ≤ 10%
VIN_PP = 1.5          # 测试激励：1.5 Vpp、0.8 V 直流偏置正弦
TEST_FREQS = (1000.0, 2000.0)


def analog_poles():
    """求 A2*s^2 + A1*s + A0 的根；过阻尼时为一对负实数（rad/s）"""
    disc = A1 * A1 - 4.0 * A2 * A0
    if disc <= 0.0:
        raise SystemExit("本脚本仅支持过阻尼原型（两个实极点）；"
                         "Q >= 0.707 时请改用复极点双二阶直接设计。")
    sq = math.sqrt(disc)
    return (-A1 + sq) / (2.0 * A2), (-A1 - sq) / (2.0 * A2)


def design():
    """模拟原型 -> Biquad 系数（双线性变换，不预畸变）"""
    s1, s2 = analog_poles()
    wc1, wc2 = -s1, -s2           # 两个一阶节的转角频率（正值 rad/s）
    c = 2.0 * FS                  # 双线性变换常数
    p1 = (c - wc1) / (c + wc1)    # z 域极点
    p2 = (c - wc2) / (c + wc2)
    K1 = wc1 / (c + wc1)          # 每节前向增益，= (1-p)/2，保证单节直流增益为 1
    K2 = wc2 / (c + wc2)
    b0 = NUM_G * K1 * K2
    return {
        "s1": s1, "s2": s2, "wc1": wc1, "wc2": wc2,
        "p1": p1, "p2": p2,
        "b0": b0, "b1": 2.0 * b0, "b2": b0,
        "a1": -(p1 + p2), "a2": p1 * p2,
    }


def H_analog(f):
    """模拟原型频响 H(j*2*pi*f)"""
    s = 2j * math.pi * f
    return NUM_G / (A2 * s * s + A1 * s + A0)


def H_digital(f, d):
    """离散 Biquad 频响 H(e^{j*2*pi*f/Fs})"""
    z = cmath.exp(2j * math.pi * f / FS)
    num = d["b0"] * z * z + d["b1"] * z + d["b2"]
    den = z * z + d["a1"] * z + d["a2"]
    return num / den


def main():
    ok = True
    w0 = math.sqrt(A0 / A2)
    Q = math.sqrt(A0 / A2) / (A1 / A2)
    print("==== design_filter: 模拟原型 -> Biquad（双线性变换，Fs=%.0f Hz）====" % FS)
    print("H(s) = %g / (%g*s^2 + %g*s + %g)" % (NUM_G, A2, A1, A0))
    print("直流增益 G = %g (+%.2f dB), f0 = %.1f Hz, Q = %.4f (过阻尼)\n"
          % (NUM_G, 20 * math.log10(NUM_G), w0 / (2 * math.pi), Q))

    # ---- 模拟极点 -> z 域极点 ----
    d = design()
    print("[模拟极点] s1 = %.2f rad/s (%.1f Hz),  s2 = %.2f rad/s (%.1f Hz)"
          % (d["s1"], d["wc1"] / (2 * math.pi), d["s2"], d["wc2"] / (2 * math.pi)))
    print("[z域极点] p1 = %.7f,  p2 = %.7f   (|p|<1 稳定: %s)\n"
          % (d["p1"], d["p2"], abs(d["p1"]) < 1 and abs(d["p2"]) < 1))

    # ---- 与固件锁定系数对照 ----
    print("[系数对照 vs Core/Src/main.c]")
    for k in ("b0", "b1", "b2", "a1", "a2"):
        diff = abs(d[k] - FIRMWARE[k])
        good = diff <= TOL
        print("  %-2s 设计=%+.7f  固件=%+.7f  |diff|=%.1e -> %s"
              % (k, d[k], FIRMWARE[k], diff, "PASS" if good else "FAIL"))
        if not good:
            ok = False
    g_dc = (d["b0"] + d["b1"] + d["b2"]) / (1.0 + d["a1"] + d["a2"])
    print("  直流增益 = %.6f (target %g)\n" % (g_dc, NUM_G))

    # ---- 模拟 vs 离散 一致性（验收指标）----
    print("[模拟参考 vs 数字实现]（激励 %.1f Vpp 正弦，验收：Vpp 误差 ≤ %.0f%%）"
          % (VIN_PP, VPP_TOL * 100))
    print("%8s | %8s %9s | %8s %9s | %8s %9s" %
          ("f (Hz)", "|H(jW)|", "phase", "|H(e^jw)|", "phase", "dVpp", "dphase"))
    for f in TEST_FREQS:
        ha, hd = H_analog(f), H_digital(f, d)
        vpp_a, vpp_d = VIN_PP * abs(ha), VIN_PP * abs(hd)
        err = abs(vpp_d - vpp_a) / vpp_a
        dph = math.degrees(cmath.phase(hd)) - math.degrees(cmath.phase(ha))
        good = err <= VPP_TOL
        print("%8.0f | %8.4f %8.2f° | %8.4f %8.2f° | %7.2f%% %8.2f°  -> %s"
              % (f, abs(ha), math.degrees(cmath.phase(ha)),
                 abs(hd), math.degrees(cmath.phase(hd)),
                 err * 100, dph, "PASS" if good else "FAIL"))
        if not good:
            ok = False
    print("  （误差来源：双线性频率翘曲；f << Fs/2 时可忽略，未做预畸变补偿）\n")

    # ---- 生成 C 初始化片段 ----
    print("[C 初始化片段]（如需修改规格，将下列数值同步到 Core/Src/main.c）")
    print("  static biquad_t lpf = {.b0 = %.7ff," % d["b0"])
    print("                         .b1 = %.7ff," % d["b1"])
    print("                         .b2 = %.7ff," % d["b2"])
    print("                         .a1 = %.7ff," % d["a1"])
    print("                         .a2 = %.7ff," % d["a2"])
    print("                         .s1 = 0.0f,")
    print("                         .s2 = 0.0f};")

    print("\n==== OVERALL: %s ====" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
