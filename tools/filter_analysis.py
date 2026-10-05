#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
滤波器系数分析 / 复算工具（纯标准库，无第三方依赖）

从固件锁定的系数出发，复算零极点分布、直流增益、幅相频响应与 -3 dB 截止频率，
并对照 verify/test_filter.c 中的锚点自动判定 PASS/FAIL，
保证“分析工具 / 离线测试 / 板上固件”三方数值一致。

系数唯一来源是 Core/Src/main.c 中 lpf 的初始化值；
若修改固件系数，请同步更新下方 B0..A2。

运行：python tools/filter_analysis.py
"""
import cmath
import math
import sys

# Windows 控制台默认编码可能不是 UTF-8（如 cp936/cp1252），显式重配防止中文输出崩溃
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

FS = 50000.0

# ---- 与 Core/Src/main.c 一致的锁定系数（a1 自带负号）----
B0, B1, B2 = 0.0152672, 0.0305344, 0.0152672
A1, A2 = -1.5114504, 0.5419847


def H(f):
    """频率响应 H(z) = B(z)/A(z)，z = e^{j*2*pi*f/Fs}"""
    z = cmath.exp(2j * math.pi * f / FS)
    return (B0 * z * z + B1 * z + B2) / (z * z + A1 * z + A2)


def db(x):
    return 20.0 * math.log10(x)


def f3db(gain_dc):
    """二分搜索 |H| 降到 gain_dc/sqrt(2) 的频率（-3 dB 截止点）"""
    target = gain_dc / math.sqrt(2.0)
    lo, hi = 1.0, FS / 2.0
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if abs(H(mid)) > target:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def main():
    ok = True
    print("==== filter_analysis: Fs=%.0f Hz ====" % FS)
    print("b = (%.7f, %.7f, %.7f)" % (B0, B1, B2))
    print("a = (1, %.7f, %.7f)\n" % (A1, A2))

    # ---- 零极点 ----
    disc = complex(A1 * A1 - 4.0 * A2)
    sq = cmath.sqrt(disc)
    p1 = (-A1 + sq) / 2.0
    p2 = (-A1 - sq) / 2.0
    stable = abs(p1) < 1.0 and abs(p2) < 1.0
    print("[Poles ] p1=%.6f, p2=%.6f   |p|<1 稳定: %s" % (p1.real, p2.real, stable))
    if not stable:
        print("       FAIL: 存在单位圆外极点")
        ok = False
    print("[Zeros ] z=-1 双重零点（分子为 b0*(1+z^-1)^2，奈奎斯特处深衰减）")
    # 实极点 = 两个一阶低通级联；按设计方法（双线性、不预畸变）折算回模拟原型转角，
    # 与 tools/design_filter.py 的模拟极点一一对应
    for p in sorted((p1.real, p2.real), reverse=True):
        wc = 2.0 * FS * (1.0 - p) / (1.0 + p)
        print("         极点 %.6f -> 模拟原型转角 %.1f Hz" % (p, wc / (2.0 * math.pi)))

    # ---- 直流增益 ----
    g_dc = (B0 + B1 + B2) / (1.0 + A1 + A2)
    print("\n[DC    ] gain = %.6f  (target 2.0000)" % g_dc)
    if abs(g_dc - 2.0) > 1e-4:
        print("         FAIL")
        ok = False
    else:
        print("         PASS")

    # ---- -3 dB 截止频率 ----
    fc3 = f3db(g_dc)
    print("\n[-3dB  ] cutoff = %.1f Hz  (|H| = %.4f)" % (fc3, g_dc / math.sqrt(2.0)))

    # ---- 幅相频响应表 ----
    print("\n[Freq response]")
    print("%9s %9s %9s %13s" % ("f (Hz)", "|H|", "dB", "phase (deg)"))
    for f in (100, 300, 600, 1000, 2000, 5000, 10000, 20000):
        h = H(f)
        print("%9d %9.4f %9.2f %13.2f" % (f, abs(h), db(abs(h)), math.degrees(cmath.phase(h))))

    # ---- 锚点交叉验证（与 verify/test_filter.c 的 cases[] 保持一致）----
    print("\n[Anchor cross-check vs verify/test_filter.c]")
    anchors = [
        # (f, Vpp_lo, Vpp_hi, phase_lo, phase_hi, tag)，输入 1.5 Vpp 正弦
        (1000.0, 1.49, 1.53, -75.0, -69.0, "1kHz"),
        (2000.0, 0.76, 0.80, -102.0, -95.0, "2kHz"),
    ]
    vin_pp = 1.5
    for f, lo, hi, plo, phi, tag in anchors:
        h = H(f)
        vpp = vin_pp * abs(h)
        ph = math.degrees(cmath.phase(h))
        good = (lo <= vpp <= hi) and (plo <= ph <= phi)
        print("  %-5s Vpp=%.4f V (锚点 %.2f~%.2f)  phase=%.2f deg (锚点 %.0f~%.0f)  -> %s"
              % (tag, vpp, lo, hi, ph, plo, phi, "PASS" if good else "FAIL"))
        if not good:
            ok = False

    print("\n==== OVERALL: %s ====" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
