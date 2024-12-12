#!/bin/bash

carla_port=$1
Traffic_port=$2

export PORT=${carla_port}
export TM_PORT=${Traffic_port}

export CARLA_ROOT=/data/home_backup/DriveVLM/carla-0.9.15
export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg":"${SUMO_HOME}/tools/":${PYTHONPATH}

python /data/limsim-o/LimSimLLM/simModel_Carla/RoadInfoGet.py \
--port=${PORT} \
--trafficManagerPort=${TM_PORT} 