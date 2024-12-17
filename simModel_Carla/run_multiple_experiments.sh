#!/bin/bash

# 实验次数
NUM_EXPERIMENTS=1

# 模型列表
MODELS=("interfuser" "PDM")

# 场景列表
SCENARIOS=("ramp" "intersection" "roundabout" "straight" "curve")

# 配置基础端口
BASE_PORT_INTERFUSER=3000
BASE_PORT_PDM=10000

# 为整个实验批次创建一个统一的时间戳
batch_timestamp=$(date +%Y%m%d_%H%M%S)

# 创建新的tmux会话（如果不存在）
if ! tmux has-session -t limsim 2>/dev/null; then
    tmux new-session -d -s limsim
fi

for exp_num in $(seq 1 $NUM_EXPERIMENTS); do
    echo "Starting experiment run $exp_num"
    
    for model in "${MODELS[@]}"; do
        for ((i=0; i<${#SCENARIOS[@]}; i++)); do
            scenario=${SCENARIOS[$i]}
            
            # 设置端口
            if [ "$model" == "interfuser" ]; then
                port=$((BASE_PORT_INTERFUSER + i*200 + (exp_num-1) * ${#SCENARIOS[@]}))
                tm_port=$((1112 + i*20000 + (exp_num-1) * ${#SCENARIOS[@]}))
                gpu=0
            else
                port=$((BASE_PORT_PDM + i*200 + (exp_num-1) * ${#SCENARIOS[@]}))
                tm_port=$((2112 + i*200 + (exp_num-1) * ${#SCENARIOS[@]}))
                gpu=2
            fi
            
            # 创建唯一的数据库名称
            db_name="${model}_${scenario}_${batch_timestamp}_run${exp_num}"
            
            # 在limsim会话中启动CARLA服务器
            tmux new-window -t limsim -n "carla_${scenario}_${model}_${exp_num}"
            tmux send-keys -t limsim:"carla_${scenario}_${model}_${exp_num}" "cd /data/home_backup/DriveVLM/carla-0.9.15; ./CarlaUE4.sh -RenderOffScreen -nosound -quality-level=low --world-port=${port} -graphicsadapter=${gpu}" C-m
            
            # 等待CARLA启动
            sleep 20
            
            # 在limsim会话中运行模型
            tmux new-window -t limsim -n "${model}_${scenario}_${exp_num}"
            tmux send-keys -t limsim:"${model}_${scenario}_${exp_num}" "conda deactivate; conda activate limsim-o" C-m
            tmux send-keys -t limsim:"${model}_${scenario}_${exp_num}" "cd /data/limsim-o/LimSimLLM; bash ./simModel_Carla/Example${model^}.sh ${port} ${tm_port} 1121$(printf '%03d' $((exp_num * 100 + i))) ${db_name} ./simModel_Carla/exp_config/${scenario}_config.yaml" C-m
            
        done
    done
done

# echo "所有实验已完成！"
# echo "时间戳: ${batch_timestamp}"
# echo "请使用以下命令分析结果："
# echo "bash ./simModel_Carla/analyze_experiments.sh ${batch_timestamp}"

echo "所有实验已在limsim会话中启动！"
echo "使用 'tmux attach-session -t limsim' 查看实验进度"
echo "时间戳: ${batch_timestamp}"