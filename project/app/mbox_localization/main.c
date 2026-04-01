/*
 * main.c - RV1106 MBox 声源定位 Demo
 *
 * 功能：
 *   - 通过 ALSA 从 ES7210 四麦克风阵列采集音频
 *   - 使用简化版 GCC-PHAT 计算两个正交基线的时延 (Mic0-2, Mic1-3)
 *   - 估计平面方位角（0°~360°），并周期性打印
 *
 * 依赖：
 *   - ALSA 库 (libasound)
 *   - 已在设备树中启用 es7210_sound（simple-audio-card）
 *
 * 编译：
 *   - 通过 project/app/mbox_localization/Makefile 由 SDK 统一构建
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>

#include <alsa/asoundlib.h>

#include "gcc_phat.h"

#define NUM_MICS        4
#define DEFAULT_DEVICE  "hw:0,0"
#define DEFAULT_RATE    48000
#define DEFAULT_FRAMES  1024

/* 阵列半径（米），根据你的实际 PCB 麦克风布局调整。
 * 比如四个麦克风在直径约 6cm 圆上，则 R ≈ 0.03
 */
#define ARRAY_RADIUS_M  0.03f
#define SOUND_SPEED     343.0f

static volatile int g_running = 1;

static void handle_signal(int sig)
{
    (void)sig;
    g_running = 0;
}

int main(int argc, char *argv[])
{
    const char   *device      = DEFAULT_DEVICE;
    unsigned int  sample_rate = DEFAULT_RATE;
    snd_pcm_uframes_t frames  = DEFAULT_FRAMES;

    if (argc > 1) {
        device = argv[1];
    }
    if (argc > 2) {
        sample_rate = (unsigned int)atoi(argv[2]);
    }
    if (argc > 3) {
        frames = (snd_pcm_uframes_t)atoi(argv[3]);
    }

    printf("=== RV1106 MBox 声源定位 Demo ===\n");
    printf("ALSA 设备  : %s\n", device);
    printf("采样率     : %u Hz\n", sample_rate);
    printf("每帧采样点 : %lu\n", (unsigned long)frames);
    printf("阵列半径   : %.3f m\n", ARRAY_RADIUS_M);
    printf("按 Ctrl+C 退出\n\n");

    /* 注册信号处理 */
    signal(SIGINT,  handle_signal);
    signal(SIGTERM, handle_signal);

    /* 打开 PCM 采集设备 */
    snd_pcm_t *handle = NULL;
    int err = snd_pcm_open(&handle, device,
                           SND_PCM_STREAM_CAPTURE, 0);
    if (err < 0) {
        fprintf(stderr, "无法打开音频设备 %s: %s\n",
                device, snd_strerror(err));
        return 1;
    }

    /* 配置硬件参数 */
    snd_pcm_hw_params_t *hw_params = NULL;
    snd_pcm_hw_params_alloca(&hw_params);
    snd_pcm_hw_params_any(handle, hw_params);
    snd_pcm_hw_params_set_access(handle, hw_params,
                                 SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(handle, hw_params,
                                 SND_PCM_FORMAT_S16_LE);
    snd_pcm_hw_params_set_channels(handle, hw_params, NUM_MICS);
    snd_pcm_hw_params_set_rate_near(handle, hw_params,
                                    &sample_rate, 0);
    snd_pcm_hw_params_set_period_size_near(handle, hw_params,
                                           &frames, 0);

    err = snd_pcm_hw_params(handle, hw_params);
    if (err < 0) {
        fprintf(stderr, "无法设置硬件参数: %s\n",
                snd_strerror(err));
        snd_pcm_close(handle);
        return 1;
    }

    printf("实际采样率 : %u Hz\n", sample_rate);
    printf("实际帧大小 : %lu 帧\n\n", (unsigned long)frames);

    /* 分配缓冲区：交错格式 + 4 个单独通道 */
    size_t frame_samples = (size_t)frames;
    int16_t *interleaved =
        (int16_t *)malloc(frame_samples * NUM_MICS * sizeof(int16_t));
    float *ch[NUM_MICS] = {0};

    if (!interleaved) {
        fprintf(stderr, "内存分配失败\n");
        snd_pcm_close(handle);
        return 1;
    }

    for (int i = 0; i < NUM_MICS; ++i) {
        ch[i] = (float *)malloc(frame_samples * sizeof(float));
        if (!ch[i]) {
            fprintf(stderr, "内存分配失败 (ch[%d])\n", i);
            for (int j = 0; j < i; ++j) {
                free(ch[j]);
            }
            free(interleaved);
            snd_pcm_close(handle);
            return 1;
        }
    }

    /* 启动采集 */
    err = snd_pcm_prepare(handle);
    if (err < 0) {
        fprintf(stderr, "PCM 设备准备失败: %s\n",
                snd_strerror(err));
        goto cleanup;
    }

    unsigned long frame_count = 0;

    while (g_running) {
        /* 从 ALSA 读取一帧数据（交错格式） */
        snd_pcm_sframes_t r =
            snd_pcm_readi(handle, interleaved, frames);

        if (r == -EPIPE) {
            /* overrun，重新准备设备 */
            snd_pcm_prepare(handle);
            continue;
        } else if (r < 0) {
            fprintf(stderr, "读取音频失败: %s\n",
                    snd_strerror(r));
            break;
        } else if (r == 0) {
            /* 没有数据，稍等一下 */
            usleep(5000);
            continue;
        }

        size_t valid_frames = (size_t)r;

        /* 分离四通道，并归一化为 [-1,1] */
        for (size_t i = 0; i < valid_frames; ++i) {
            for (int c = 0; c < NUM_MICS; ++c) {
                int16_t s = interleaved[i * NUM_MICS + c];
                ch[c][i]  = (float)s / 32768.0f;
            }
        }

        /* 计算两条基线上的时延：
         *   X 轴: Mic0 vs Mic2
         *   Y 轴: Mic1 vs Mic3
         */
        float max_tau = (ARRAY_RADIUS_M * 2.0f) / SOUND_SPEED * 1.5f;

        float tau_x = gcc_phat_tdoa(ch[0], ch[2],
                                    valid_frames,
                                    (float)sample_rate,
                                    max_tau);
        float tau_y = gcc_phat_tdoa(ch[1], ch[3],
                                    valid_frames,
                                    (float)sample_rate,
                                    max_tau);

        float az_deg = estimate_azimuth_deg(tau_x, tau_y,
                                            ARRAY_RADIUS_M,
                                            SOUND_SPEED);

        frame_count++;
        if (frame_count % 10 == 0) {
            printf("方位角: %7.2f 度  |  tau_x = %+8.5f ms  tau_y = %+8.5f ms\n",
                   az_deg,
                   tau_x * 1000.0f,
                   tau_y * 1000.0f);
            fflush(stdout);
        }
    }

    printf("\n收到退出信号，停止采集...\n");

cleanup:
    if (handle) {
        snd_pcm_drop(handle);
        snd_pcm_close(handle);
    }
    if (interleaved) {
        free(interleaved);
    }
    for (int i = 0; i < NUM_MICS; ++i) {
        if (ch[i]) {
            free(ch[i]);
        }
    }

    return 0;
}


