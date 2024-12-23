#!/bin/bash
pkill -f "python.*Example"
# 实验次数
NUM_EXPERIMENTS=3

# 模型列表
MODELS=( 
    # "interfuser"
    # "PDM" 
    "VLMagentCloseLoop"
)

# 场景列表
SCENARIOS=(
    "ramp"           # 匝道场景
    # "intersection"   # 十字路口场景
    # "roundabout"    # 环岛场景
    # "straight"      # 直道场景
    # "curve"         # 弯道场景
)

# 配置基础端口
BASE_PORT_INTERFUSER=3000
BASE_PORT_PDM=10000
BASE_PORT_VLMAGENT=8000

# 为整个实验批次创建一个统一的时间戳
batch_timestamp=$(date +%Y%m%d_%H%M%S)

# 创建新的tmux会话（如果不存在）
if ! tmux has-session -t limsim 2>/dev/null; then
    tmux new-session -d -s limsim
fi

for exp_num in $(seq 1 $NUM_EXPERIMENTS); do
    echo "Starting experiment run $exp_num"
    
    # 基于实验编号分配GPU（假设有3个GPU：0,1,2）
    gpu=$(( (exp_num - 1) % 4 ))
    
    for model in "${MODELS[@]}"; do
        for ((i=0; i<${#SCENARIOS[@]}; i++)); do
            scenario=${SCENARIOS[$i]}
            
            # 设置端口
            if [ "$model" == "interfuser" ]; then
                port=$((BASE_PORT_INTERFUSER + i*200 + (exp_num-1) * ${#SCENARIOS[@]}))
                tm_port=$((4000 + i*20000 + (exp_num-1) * ${#SCENARIOS[@]}))
            elif [ "$model" == "PDM" ]; then
                port=$((BASE_PORT_PDM + i*200 + (exp_num-1) * ${#SCENARIOS[@]}))
                tm_port=$((2112 + i*200 + (exp_num-1) * ${#SCENARIOS[@]}))
            elif [ "$model" == "VLMagentCloseLoop" ]; then
                port=$((BASE_PORT_VLMAGENT + i*2000 + (exp_num-1) * ${#SCENARIOS[@]}))
                tm_port=$((9000 + i*20000 + (exp_num-1) * ${#SCENARIOS[@]}))
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