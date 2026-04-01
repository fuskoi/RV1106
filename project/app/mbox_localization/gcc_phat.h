/*
 * gcc_phat.h
 *
 * 简化版 GCC-PHAT / 互相关时延估计算法，用于 4 麦克风声源方位角估计
 *
 * 假设麦克风为环形 4 阵列，位置近似为：
 *   Mic0: ( R, 0 )
 *   Mic1: ( 0, R )
 *   Mic2: ( -R, 0 )
 *   Mic3: ( 0, -R )
 *
 * 方位角 0° 指向 +X 方向（Mic0 正前方），逆时针增加。
 */

#ifndef MBOX_GCC_PHAT_H
#define MBOX_GCC_PHAT_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 计算两路信号的时延（秒）
 * signal1, signal2: 输入信号（float，已归一化到 [-1,1]）
 * n:                采样点数
 * sample_rate:      采样率（Hz）
 * max_tau:          最大允许时延（秒），用于限制搜索范围（例如 麦距/c 的 2 倍）
 *
 * 返回：时延（秒），正值表示 signal2 比 signal1 滞后
 */
float gcc_phat_tdoa(const float *signal1,
                    const float *signal2,
                    size_t       n,
                    float        sample_rate,
                    float        max_tau);

/* 根据两组互相垂直的麦克风对时延，估计平面方位角
 *
 * tau_x: Mic0 与 Mic2 之间的时延（秒） -> X 轴方向
 * tau_y: Mic1 与 Mic3 之间的时延（秒） -> Y 轴方向
 * radius: 阵列半径（米）
 * c:      声速（米/秒），例如 343.0f
 *
 * 返回：方位角（度），范围 [0, 360)
 */
float estimate_azimuth_deg(float tau_x,
                           float tau_y,
                           float radius,
                           float c);

#ifdef __cplusplus
}
#endif

#endif /* MBOX_GCC_PHAT_H */


