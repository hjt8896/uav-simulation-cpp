//
// Created by hujiet on 2026/6/18.
//

#ifndef AIRCRAFT_SIM_NOTCHFILTER_H
#define AIRCRAFT_SIM_NOTCHFILTER_H


#include <cmath>

class NotchFilter {
private:
    // 滤波器系数
    float b0, b1, b2, a1, a2;
    // 历史状态缓存
    float x1, x2, y1, y2;

public:
    NotchFilter() : x1(0), x2(0), y1(0), y2(0) {}

    // 初始化/更新滤波器系数
    // f_center: 要干掉的电机噪声频率 (比如 200 Hz)
    // f_samp: IMU 采样率 (咱们是 1000 Hz)
    // Q: 品质因数，控制陷波的“宽度”。Q越大，坑越窄越深；Q越小，连带滤掉的正常信号越多。通常取 1.0~2.0
    void init(float f_center, float f_samp, float Q) {
        float omega = 2.0f * M_PI * f_center / f_samp;
        float alpha = std::sin(omega) / (2.0f * Q);
        float cos_w = std::cos(omega);

        float a0 = 1.0f + alpha;

        // 计算标准差分方程系数
        b0 = 1.0f / a0;
        b1 = -2.0f * cos_w / a0;
        b2 = 1.0f / a0;
        a1 = -2.0f * cos_w / a0;
        a2 = (1.0f - alpha) / a0;
    }

    // 核心执行函数，放在 1000Hz 循环里
    float apply(float sample) {
        // 核心差分方程
        float out = b0 * sample + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;

        // 更新历史状态 (将数据往过去推移一步)
        x2 = x1;
        x1 = sample;
        y2 = y1;
        y1 = out;

        return out;
    }
};


#endif //AIRCRAFT_SIM_NOTCHFILTER_H