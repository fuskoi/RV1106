/*
 * gcc_phat.c
 *
 * 简化版 GCC-PHAT / 归一化互相关实现
 *
 * 为了在 RV1106 上快速验证声源定位，这里采用“时域归一化互相关”，
 * 没有使用完整的 FFT 版 GCC-PHAT，优点是实现简单，缺点是对混响和噪声鲁棒性略差。
 *
 * 如果后续需要更高精度，可以将此实现替换为频域 GCC-PHAT。
 */

#include "gcc_phat.h"

#include <math.h>
#include <float.h>

/* 计算单路信号能量 */
static float signal_energy(const float *x, size_t n)
{
    float e = 0.0f;
    for (size_t i = 0; i < n; ++i) {
        e += x[i] * x[i];
    }
    return e;
}

/*
 * 归一化互相关：
 *   R_xy(k) = sum_i x[i] * y[i+k] / sqrt(E_x * E_y)
 * 其中 k 为样本级的延迟。
 */
float gcc_phat_tdoa(const float *signal1,
                    const float *signal2,
                    size_t       n,
                    float        sample_rate,
                    float        max_tau)
{
    if (!signal1 || !signal2 || n < 8 || sample_rate <= 0.0f) {
        return 0.0f;
    }

    /* 能量归一化，避免幅度影响 */
    float e1 = signal_energy(signal1, n);
    float e2 = signal_energy(signal2, n);
    if (e1 <= FLT_EPSILON || e2 <= FLT_EPSILON) {
        return 0.0f;
    }
    float norm = sqrtf(e1 * e2);

    /* 最大延迟样本数（双向）*/
    int max_lag = (int)(max_tau * sample_rate);
    if (max_lag <= 0) {
        max_lag = (int)(0.001f * sample_rate); /* 至少 1ms 搜索窗 */
    }
    if (max_lag > (int)(n / 2)) {
        max_lag = (int)(n / 2);
    }

    float best_corr = -1e30f;
    int   best_lag  = 0;

    for (int lag = -max_lag; lag <= max_lag; ++lag) {
        float corr = 0.0f;

        /* 对齐两个序列 */
        size_t start1 = 0;
        size_t start2 = 0;
        size_t len    = 0;

        if (lag >= 0) {
            start1 = 0;
            start2 = (size_t)lag;
            if (start2 >= n) {
                continue;
            }
            len = n - start2;
        } else {
            start1 = (size_t)(-lag);
            start2 = 0;
            if (start1 >= n) {
                continue;
            }
            len = n - start1;
        }

        for (size_t i = 0; i < len; ++i) {
            corr += signal1[start1 + i] * signal2[start2 + i];
        }

        corr /= norm;

        if (corr > best_corr) {
            best_corr = corr;
            best_lag  = lag;
        }
    }

    /* 样本延迟 -> 时间延迟（秒）*/
    return (float)best_lag / sample_rate;
}

/*
 * 根据 X/Y 方向上的时延，估计平面 DOA。
 *
 * 模型假设：
 *   - Mic0 与 Mic2 构成 X 轴基线，间距约为 2R；
 *   - Mic1 与 Mic3 构成 Y 轴基线，间距约为 2R；
 *
 * 在远场近似下：
 *   tau_x ≈ (2R * cos(theta)) / c
 *   tau_y ≈ (2R * sin(theta)) / c
 *
 * => cos(theta) = c * tau_x / (2R)
 *    sin(theta) = c * tau_y / (2R)
 */
float estimate_azimuth_deg(float tau_x,
                           float tau_y,
                           float radius,
                           float c)
{
    if (radius <= 0.0f || c <= 0.0f) {
        return 0.0f;
    }

    float denom = 2.0f * radius / c; /* tau = 2R/c * cos/sin(theta) */

    float cx = tau_x / denom; /* ~ cos(theta) */
    float sy = tau_y / denom; /* ~ sin(theta) */

    /* 限幅，避免数值误差 */
    if (cx > 1.0f)  cx = 1.0f;
    if (cx < -1.0f) cx = -1.0f;
    if (sy > 1.0f)  sy = 1.0f;
    if (sy < -1.0f) sy = -1.0f;

    /* 使用 atan2(sin, cos) 计算角度 */
    float theta = atan2f(sy, cx); /* 弧度，范围 [-pi, pi] */

    if (theta < 0.0f) {
        theta += 2.0f * (float)M_PI;
    }

    return theta * 180.0f / (float)M_PI;
}


