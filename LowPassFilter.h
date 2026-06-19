#ifndef AIRCRAFT_SIM_LOWPASSFILTER_H
#define AIRCRAFT_SIM_LOWPASSFILTER_H

#include <cmath>

class LowPassFilter {
private:
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;

public:
    LowPassFilter() : x1(0), x2(0), y1(0), y2(0) {}

    // 初始化二阶巴特沃斯低通滤波器
    void init(float cutoff_freq, float sample_rate) {
        float omega = 2.0f * M_PI * cutoff_freq / sample_rate;
        float sn = std::sin(omega);
        float cs = std::cos(omega);

        // 巴特沃斯 Q 值固定为 0.707 (即 1/sqrt(2))，保证通带最平坦
        float alpha = sn / (2.0f * 0.70710678f);

        float a0 = 1.0f + alpha;

        b0 = (1.0f - cs) / 2.0f / a0;
        b1 = (1.0f - cs) / a0;
        b2 = (1.0f - cs) / 2.0f / a0;
        a1 = -2.0f * cs / a0;
        a2 = (1.0f - alpha) / a0;
    }

    float apply(float sample) {
        float out = b0 * sample + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        x2 = x1; x1 = sample; y2 = y1; y1 = out;
        return out;
    }
};

#endif // AIRCRAFT_SIM_LOWPASSFILTER_H