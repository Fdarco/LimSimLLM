#!/bin/bash  
 
carla_port=$1 
Traffic_port=$2
SEED=$3
DATABASE=$4
CONFIG_PATH=$5
EVAL_ONLY=${6:-false}

export PORT=${carla_port}
export TM_PORT=${Traffic_port}
export SEED=${SEED}
export DATABASE=${DATABASE}
export CONFIG_PATH=${CONFIG_PATH}

export CARLA_ROOT=/data/home_backup/DriveVLM/carla-0.9.15
export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg":"${SUMO_HOME}/tools/":${PYTHONPATH}

# python /data/limsim-o/LimSimLLM/simModel_Carla/RoadInfoGet.py
if [ "$EVAL_ONLY" != true ]; then
    python /data/limsim-o/LimSimLLM/simModel_Carla/ExamplePDM.py \
    --port=${PORT} \
    --trafficManagerPort=${TM_PORT} \
    --random_seed=${SEED} \
    --database=${DATABASE} \
    --config_path=${CONFIG_PATH}

    echo "PDM Experiment finished"
fi

# 实验完成后立即运行评估
bash ./simModel_Carla/eval.sh ${4}  # ${4}是数据库名称参数
echo "Evaluation finished"
# 清理CARLA进程
pkill -9 -f "CarlaUE4-Linux.*--world-port=${1}"  # ${1}是端口参数
