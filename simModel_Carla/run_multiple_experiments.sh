#!/bin/bash

# 清理已有进程
pkill -f "python.*Example"

# 实验轮次
NUM_EXPERIMENTS=1  # 添加实验轮次参数

# 模型和场景配置
MODELS=( 
    # "interfuser"
    "PDM" 
    # "VLMagentCloseLoop"
)

SCENARIOS=(
    # "ramp"           
    # "intersection"   
    # "roundabout"     
    # "straight"       
    "curve"          
    # "long_term"      
)

# 为每个GPU配置基础端口
declare -A PORT_MAP=(
    ["0"]="2000"
    ["1"]="3000"
    ["2"]="6000"
    ["3"]="5000"
)

batch_timestamp=$(date +%Y%m%d_%H%M%S)

# 创建tmux会话
if ! tmux has-session -t limsim 2>/dev/null; then
    tmux new-session -d -s limsim
fi

# 计算总实验数和每个GPU应运行的实验数
total_experiments=$((${#MODELS[@]} * ${#SCENARIOS[@]} * NUM_EXPERIMENTS))
experiments_per_gpu=$((total_experiments / 4 + (total_experiments % 4 > 0 ? 1 : 0)))

echo "=== 实验配置 ==="
echo "总实验数: $total_experiments"
echo "每个GPU实验数: $experiments_per_gpu"
echo "实验轮次: $NUM_EXPERIMENTS"
echo -n "运行模型: "
printf "%s " "${MODELS[@]}"
echo
echo -n "测试场景: "
printf "%s " "${SCENARIOS[@]}"
echo
echo "开始时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo "================="

# 定义在单个GPU上串行运行实验的函数
run_experiments_on_gpu() {
    local gpu=$1
    local start_index=$((gpu * experiments_per_gpu))
    local end_index=$((start_index + experiments_per_gpu - 1))
    
    # 修改边界检查逻辑
    if [ $start_index -ge $total_experiments ]; then
        return
    fi
    
    # 确保end_index不超过总实验数减1
    if [ $end_index -ge $total_experiments ]; then
        end_index=$((total_experiments - 1))
    fi
    
    local base_port=${PORT_MAP[$gpu]}
    local port_offset=0
    
    # 直接从start_index开始运行分配的实验
    for ((current_exp=start_index; current_exp<=end_index && current_exp<total_experiments; current_exp++)); do
        # 计算当前实验对应的模型、场景和轮次
        local exp_round=$((current_exp / (${#SCENARIOS[@]} * ${#MODELS[@]} ) + 1))
        local scenario_index=$(( (current_exp % (${#SCENARIOS[@]} * ${#MODELS[@]})) % ${#SCENARIOS[@]} ))
        local model_index=$(( (current_exp % (${#SCENARIOS[@]} * ${#MODELS[@]})) / ${#SCENARIOS[@]} ))
        
        local model=${MODELS[$model_index]}
        local scenario=${SCENARIOS[$scenario_index]}
        
        echo "GPU ${gpu}: 开始运行实验 ${current_exp}/${total_experiments} - ${model} 在 ${scenario} 场景 (轮次 ${exp_round})"
        
        port=$((base_port + port_offset))
        tm_port=$((port + 10))
        port_offset=$((port_offset + 100))
        
        db_name="${model}_${scenario}_${batch_timestamp}_gpu${gpu}_exp${exp_round}"
        
        # 修改 tmux 窗口名称，添加轮次编号
        window_name_carla="carla_${gpu}_${scenario}_${model}_r${exp_round}"
        window_name_exp="${gpu}_${model}_${scenario}_r${exp_round}"
        
        # 计算随机数种子：使用场景索引和轮次来确定
        # 这样确保同一场景同一轮次下的不同模型使用相同的种子
        # random_seed=$((1121000 + scenario_index * 100 + exp_round))
        random_seed=$((1230406 + scenario_index * 100 + exp_round))

        
        # 记录实验开始时间
        start_time=$(date +%s)
        
        # 启动CARLA服务器
        tmux new-window -t limsim -n "${window_name_carla}"
        tmux send-keys -t limsim:"${window_name_carla}" \
            "cd /data/home_backup/DriveVLM/carla-0.9.15; ./CarlaUE4.sh -RenderOffScreen -nosound -quality-level=low --world-port=${port} -graphicsadapter=${gpu}" C-m
        
        sleep 20
        
        # 运行模型
        tmux new-window -t limsim -n "${window_name_exp}"
        tmux send-keys -t limsim:"${window_name_exp}" "conda deactivate; conda activate limsim-o" C-m
        tmux send-keys -t limsim:"${window_name_exp}" \
            "cd /data/limsim-o/LimSimLLM; bash ./simModel_Carla/Example${model^}.sh ${port} ${tm_port} ${random_seed} ${db_name} ./simModel_Carla/exp_config/${scenario}_config.yaml" C-m
        
        # 等待当前实验完成
        while true; do
            if ! pgrep -f "CarlaUE4-Linux.*--world-port=${port}" > /dev/null; then
                end_time=$(date +%s)
                duration=$((end_time - start_time))
                hours=$((duration / 3600))
                minutes=$(( (duration % 3600) / 60 ))
                seconds=$((duration % 60))
                
                echo "GPU ${gpu}: 完成实验 ${current_exp}/${total_experiments}"
                echo "├─ 模型: ${model}"
                echo "├─ 场景: ${scenario}"
                echo "├─ 轮次: ${exp_round}"
                echo "└─ 耗时: ${hours}小时 ${minutes}分钟 ${seconds}秒"
                break
            fi
            sleep 10
        done
        
        sleep 5
    done
}

# 为每个GPU启动并行进程
for gpu in {0..3}; do
    run_experiments_on_gpu $gpu &
done

# 等待所有GPU的实验完成
wait

echo "=== 实验完成 ==="
echo "结束时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo "时间戳: ${batch_timestamp}"
echo "请使用以下命令分析结果："
echo "bash ./simModel_Carla/analyze_experiments.sh ${batch_timestamp}"
echo "================="