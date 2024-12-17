#!/bin/bash

# 检查是否提供了时间戳参数
if [ $# -eq 0 ]; then
    echo "请提供实验时间戳"
    echo "用法: bash analyze_experiments.sh <timestamp>"
    echo "示例: bash analyze_experiments.sh 20240315_123456"
    exit 1
fi

timestamp=$1

# 激活正确的conda环境
conda deactivate
conda activate limsim-o

# 切换到正确的工作目录
cd /data/limsim-o/LimSimLLM

# 运行分析脚本
echo "开始分析实验结果..."
echo "时间戳: $timestamp"
python analyze_results.py --timestamp $timestamp

echo "分析完成！结果已保存到 experiment_summary_${timestamp}.csv" 