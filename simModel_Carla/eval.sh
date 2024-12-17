#!/bin/bash  

# 获取数据库参数
DATABASE=$1

export CARLA_ROOT=/data/home_backup/DriveVLM/carla-0.9.15
export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg":"${SUMO_HOME}/tools/":${PYTHONPATH}

# 检查是否提供了数据库参数
if [ -z "$DATABASE" ]; then
    echo "错误：请提供数据库文件路径"
    echo "用法：$0 <数据库文件路径>"
    exit 1
fi

# 获取数据库文件的目录路径
DB_DIR=$(dirname "$DATABASE")
# 生成评估结果文件路径（与数据库文件在同一目录下）
EVAL_OUTPUT="${DB_DIR}/eval_result.log"

python /data/limsim-o/LimSimLLM/simModel_Carla/exampleReplay.py --database "$DATABASE"