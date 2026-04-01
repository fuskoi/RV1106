#!/bin/sh
# ES8311 音频编解码器配置脚本
# 用于设置播放和录音的最佳参数

echo "配置 ES8311 音频编解码器..."

# ========== 播放相关设置 ==========
# DAC 音量设置为 0dB (191 = 0xbf)
amixer set 'DAC VOLUME' 120

# 确保 DAC 没有静音
amixer set 'DAC SDP MUTE' off
amixer set 'DAC DEM MUTE' off

# DAC 数据源选择（通常使用左声道数据）
amixer set 'DAC SDP SRC MUX' 'SELECT SDP LEFT DATA'

# ========== 录音相关设置 ==========
# ADC 音量设置为 0dB (191 = 0xbf)
amixer set 'ADC VOLUME' 191

# 确保 ADC 没有静音
amixer set 'ADC SDP MUTE' off

# 麦克风 PGA 增益（0-10，每步3dB，0=0dB, 10=30dB）
# 根据实际麦克风灵敏度调整，建议从 0 开始测试
amixer set 'MIC PGA GAIN' 0

# ADC 过采样率（16 是合理的默认值）
amixer set 'ADC OSR' 16

# ADC 同步（保持开启）
amixer set 'ADC SYNC' on

# DMIC MUX 设置为禁用（使用模拟麦克风 AMIC）
amixer set 'DMIC MUX' 'DMIC DISABLE'

# SDP OUT MUX 选择从 ADC 输出（录音路径）
amixer set 'SDP OUT MUX' 'FROM ADC OUT'

echo "配置完成！"
echo ""
echo "当前关键设置："
echo "播放设置："
amixer get 'DAC VOLUME'
amixer get 'DAC SDP MUTE'
echo ""
echo "录音设置："
amixer get 'ADC VOLUME'
amixer get 'MIC PGA GAIN'
amixer get 'ADC SDP MUTE'
amixer get 'SDP OUT MUX'

