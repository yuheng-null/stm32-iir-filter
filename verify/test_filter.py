#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
烧录前离线验证（等价 Python 版，仅用标准库，与 Core/Src/filter.c 同一份数学）
运行：python verify/test_filter.py
输入码值域正弦 x = 993 + 930.7*sin(2*pi*f*n/50000)，输出均值电压/Vpp/码范围/直流增益/相位
"""
import math

FS = 50000.0
RUN_SEC = 2.0
WIN_SEC = 0.5
VPERCODE = 3.3 / 4096.0
DC_CODE = 993.0
AMP = 930.7

# 50k 锁定系数（a1 自带负号）
B0, B1, B2 = 0.0152672, 0.0305344, 0.0152672
A1, A2 = -1.5114504, 0.5419847


class Biquad:
    """转置 II 型，差分方程与固件逐行对应"""
    def __init__(self):
        self.s1 = 0.0
        self.s2 = 0.0

    def update(self, x):
        y = B0 * x + self.s1
        self.s1 = B1 * x - A1 * y + self.s2
        self.s2 = B2 * x - A2 * y
        return y

    def preset_dc(self, x_dc, dc_gain):
        y = dc_gain * x_dc
        self.s1 = y - B0 * x_dc
        self.s2 = B2 * x_dc - A2 * y


def dc_gain_test():
    f = Biquad()
    f.preset_dc(DC_CODE, 2.0)
    y = 0.0
    for _ in range(100000):
        y = f.update(DC_CODE)
    return y / DC_CODE


def sweep_ac(fin):
    f = Biquad()
    f.preset_dc(DC_CODE, 2.0)
    n_total = int(FS * RUN_SEC)
    n_start = n_total - int(FS * WIN_SEC)
    ys, cs, cc = [], 0.0, 0.0
    for n in range(n_total):
        th = 2 * math.pi * fin * n / FS
        y = f.update(DC_CODE + AMP * math.sin(th))
        if n >= n_start:
            ys.append(y)
    mean = sum(ys) / len(ys)
    # 去均值 I/Q 相关求基波相位
    k = 0
    for n in range(n_start, n_total):
        th = 2 * math.pi * fin * n / FS
        d = ys[k] - mean
        cs += d * math.sin(th)
        cc += d * math.cos(th)
        k += 1
    ph = math.degrees(math.atan2(cc / len(ys), cs / len(ys)))
    return mean, min(ys), max(ys), ph


def main():
    ok_all = True
    print("==== IIR offline check (Python), Fs=50000 Hz ====")
    g = dc_gain_test()
    ok = abs(g - 2.0) < 1e-4
    ok_all &= ok
    print(f"[DC ] gain={g:.6f} target 2.0000 -> {'PASS' if ok else 'FAIL'}")

    # (频率, Vpp下, Vpp上, min码下/上, max码下/上, 相位下/上)
    cases = [
        ("1kHz", 1000, 1.49, 1.53, 1020, 1080, 2890, 2960, -75, -69),
        ("2kHz", 2000, 0.76, 0.80, 1470, 1530, 2440, 2500, -102, -95),
    ]
    for tag, fin, vl, vh, il, ih, xl, xh, pl, ph_hi in cases:
        mean, ymin, ymax, phase = sweep_ac(fin)
        vpp = (ymax - ymin) * VPERCODE
        checks = [
            abs(mean * VPERCODE - 1.60) < 0.01,
            vl <= vpp <= vh,
            il <= ymin <= ih,
            xl <= ymax <= xh,
            pl <= phase <= ph_hi,
            ymin > 0 and ymax < 4095,
        ]
        ok = all(checks)
        ok_all &= ok
        print(f"[{tag}] mean={mean*VPERCODE:.4f} V  Vpp={vpp:.4f} V  "
              f"range=[{ymin:.0f}..{ymax:.0f}] code  phase={phase:.2f} deg "
              f"-> {'PASS' if ok else 'FAIL'}")
    print("==== OVERALL:", "PASS (allow flashing)" if ok_all else "FAIL (do NOT flash)", "====")


if __name__ == "__main__":
    main()
