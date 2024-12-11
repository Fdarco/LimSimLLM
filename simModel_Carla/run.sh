#!/bin/bash  
  
# 创建一个新的tmux会话  
tmux new-session -d -s limsim  

# Interfuser模型测试
# 匝道场景
tmux new-window -t limsim:1 -n 'carla_ramp_interfuser'
tmux send-keys -t limsim:1 'cd /data/home_backup/DriveVLM/carla-0.9.15;./CarlaUE4.sh -RenderOffScreen -nosound -quality-level=low --world-port=3000 -graphicsadapter=0' C-m  
sleep 8
tmux new-window -t limsim:2 -n 'interfuser_ramp' 
tmux send-keys -t limsim:2 'conda deactivate; conda activate limsim-o' C-m   
tmux send-keys -t limsim:2 'cd /data/limsim-o/LimSimLLM;bash ./simModel_Carla/ExampleInterfuser.sh 3000 1112 1121102 interfuser_ramp ./simModel_Carla/exp_config/ramp_config.yaml' C-m  

# 交叉口场景
tmux new-window -t limsim:3 -n 'carla_intersection_interfuser'
tmux send-keys -t limsim:3 'cd /data/home_backup/DriveVLM/carla-0.9.15;./CarlaUE4.sh -RenderOffScreen -nosound -quality-level=low --world-port=3001 -graphicsadapter=0' C-m  
sleep 8
tmux new-window -t limsim:4 -n 'interfuser_intersection' 
tmux send-keys -t limsim:4 'conda deactivate; conda activate limsim-o' C-m   
tmux send-keys -t limsim:4 'cd /data/limsim-o/LimSimLLM;bash ./simModel_Carla/ExampleInterfuser.sh 3001 1113 1121103 interfuser_intersection ./simModel_Carla/exp_config/intersection_config.yaml' C-m  

# 环岛场景
tmux new-window -t limsim:5 -n 'carla_roundabout_interfuser'
tmux send-keys -t limsim:5 'cd /data/home_backup/DriveVLM/carla-0.9.15;./CarlaUE4.sh -RenderOffScreen -nosound -quality-level=low --world-port=3002 -graphicsadapter=0' C-m  
sleep 8
tmux new-window -t limsim:6 -n 'interfuser_roundabout' 
tmux send-keys -t limsim:6 'conda deactivate; conda activate limsim-o' C-m   
tmux send-keys -t limsim:6 'cd /data/limsim-o/LimSimLLM;bash ./simModel_Carla/ExampleInterfuser.sh 3002 1114 1121104 interfuser_roundabout ./simModel_Carla/exp_config/roundabout_config.yaml' C-m  

# 直线场景
tmux new-window -t limsim:7 -n 'carla_straight_interfuser'
tmux send-keys -t limsim:7 'cd /data/home_backup/DriveVLM/carla-0.9.15;./CarlaUE4.sh -RenderOffScreen -nosound -quality-level=low --world-port=3003 -graphicsadapter=0' C-m  
sleep 8
tmux new-window -t limsim:8 -n 'interfuser_straight' 
tmux send-keys -t limsim:8 'conda deactivate; conda activate limsim-o' C-m   
tmux send-keys -t limsim:8 'cd /data/limsim-o/LimSimLLM;bash ./simModel_Carla/ExampleInterfuser.sh 3003 1115 1121105 interfuser_straight ./simModel_Carla/exp_config/straight_config.yaml' C-m  

# 弯道场景
tmux new-window -t limsim:9 -n 'carla_curve_interfuser'
tmux send-keys -t limsim:9 'cd /data/home_backup/DriveVLM/carla-0.9.15;./CarlaUE4.sh -RenderOffScreen -nosound -quality-level=low --world-port=3004 -graphicsadapter=0' C-m  
sleep 8
tmux new-window -t limsim:10 -n 'interfuser_curve' 
tmux send-keys -t limsim:10 'conda deactivate; conda activate limsim-o' C-m   
tmux send-keys -t limsim:10 'cd /data/limsim-o/LimSimLLM;bash ./simModel_Carla/ExampleInterfuser.sh 3004 1116 1121106 interfuser_curve ./simModel_Carla/exp_config/curve_config.yaml' C-m  

# PDM模型测试
# 匝道场景
tmux new-window -t limsim:11 -n 'carla_ramp_pdm'
tmux send-keys -t limsim:11 'cd /data/home_backup/DriveVLM/carla-0.9.15;./CarlaUE4.sh -RenderOffScreen -nosound -quality-level=low --world-port=4000 -graphicsadapter=1' C-m  
sleep 8
tmux new-window -t limsim:12 -n 'pdm_ramp' 
tmux send-keys -t limsim:12 'conda deactivate; conda activate limsim-o' C-m   
tmux send-keys -t limsim:12 'cd /data/limsim-o/LimSimLLM;bash ./simModel_Carla/ExamplePDM.sh 4000 2112 1121102 pdm_ramp ./simModel_Carla/exp_config/ramp_config.yaml' C-m  

# 交叉口场景
tmux new-window -t limsim:13 -n 'carla_intersection_pdm'
tmux send-keys -t limsim:13 'cd /data/home_backup/DriveVLM/carla-0.9.15;./CarlaUE4.sh -RenderOffScreen -nosound -quality-level=low --world-port=4001 -graphicsadapter=1' C-m  
sleep 8
tmux new-window -t limsim:14 -n 'pdm_intersection' 
tmux send-keys -t limsim:14 'conda deactivate; conda activate limsim-o' C-m   
tmux send-keys -t limsim:14 'cd /data/limsim-o/LimSimLLM;bash ./simModel_Carla/ExamplePDM.sh 4001 2113 1121103 pdm_intersection ./simModel_Carla/exp_config/intersection_config.yaml' C-m  

# 环岛场景
tmux new-window -t limsim:15 -n 'carla_roundabout_pdm'
tmux send-keys -t limsim:15 'cd /data/home_backup/DriveVLM/carla-0.9.15;./CarlaUE4.sh -RenderOffScreen -nosound -quality-level=low --world-port=4002 -graphicsadapter=1' C-m  
sleep 8
tmux new-window -t limsim:16 -n 'pdm_roundabout' 
tmux send-keys -t limsim:16 'conda deactivate; conda activate limsim-o' C-m   
tmux send-keys -t limsim:16 'cd /data/limsim-o/LimSimLLM;bash ./simModel_Carla/ExamplePDM.sh 4002 2114 1121104 pdm_roundabout ./simModel_Carla/exp_config/roundabout_config.yaml' C-m  

# 直线场景
tmux new-window -t limsim:17 -n 'carla_straight_pdm'
tmux send-keys -t limsim:17 'cd /data/home_backup/DriveVLM/carla-0.9.15;./CarlaUE4.sh -RenderOffScreen -nosound -quality-level=low --world-port=4003 -graphicsadapter=1' C-m  
sleep 8
tmux new-window -t limsim:18 -n 'pdm_straight' 
tmux send-keys -t limsim:18 'conda deactivate; conda activate limsim-o' C-m   
tmux send-keys -t limsim:18 'cd /data/limsim-o/LimSimLLM;bash ./simModel_Carla/ExamplePDM.sh 4003 2115 1121105 pdm_straight ./simModel_Carla/exp_config/straight_config.yaml' C-m  

# 弯道场景
tmux new-window -t limsim:19 -n 'carla_curve_pdm'
tmux send-keys -t limsim:19 'cd /data/home_backup/DriveVLM/carla-0.9.15;./CarlaUE4.sh -RenderOffScreen -nosound -quality-level=low --world-port=4004 -graphicsadapter=1' C-m  
sleep 8
tmux new-window -t limsim:20 -n 'pdm_curve' 
tmux send-keys -t limsim:20 'conda deactivate; conda activate limsim-o' C-m   
tmux send-keys -t limsim:20 'cd /data/limsim-o/LimSimLLM;bash ./simModel_Carla/ExamplePDM.sh 4004 2116 1121106 pdm_curve ./simModel_Carla/exp_config/curve_config.yaml' C-m  