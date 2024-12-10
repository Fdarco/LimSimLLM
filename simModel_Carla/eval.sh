#!/bin/bash  
 
export CARLA_ROOT=/data/home_backup/DriveVLM/carla-0.9.15
# export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${SCENCIARIO_RUNNER_ROOT}":"${LEADERBOARD_ROOT}":"${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg":"${SUMO_HOME}/tools/":${PYTHONPATH}
export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg":"${SUMO_HOME}/tools/":${PYTHONPATH}

python /data/limsim-o/LimSimLLM/simModel_Carla/exampleReplay.py 