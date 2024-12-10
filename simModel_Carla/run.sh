#!/bin/bash  
  
# 创建一个新的tmux会话  
tmux new-session -d -s limsim  


#E2E
tmux new-window -t limsim:1 -n 'carla E2E'
tmux send-keys -t limsim:1 'cd /data/home_backup/DriveVLM/carla-0.9.15;./CarlaUE4.sh -RenderOffScreen -nosound -quality-level=low --world-port=3000 -graphicsadapter=0' C-m  

# -- 
sleep 8
tmux new-window -t limsim:2 -n 'E2E limsim rollout' 
tmux send-keys -t limsim:2 'conda deactivate' C-m   
tmux send-keys -t limsim:2 'conda activate limsim-o' C-m   
tmux send-keys -t limsim:2 'cd /data/limsim-o/LimSimLLM;bash ./simModel_Carla/ExampleInterfuser.sh 3000 1112 1121102 interfuser_1' C-m  


#PDM
tmux new-window -t limsim:3 -n 'carla PDM'
tmux send-keys -t limsim:3 'cd /data/home_backup/DriveVLM/carla-0.9.15;./CarlaUE4.sh -RenderOffScreen -nosound -quality-level=low --world-port=4000 -graphicsadapter=1' C-m 
# -- 
sleep 8
tmux new-window -t limsim:4 -n 'PDM limsim rollout' 
tmux send-keys -t limsim:4 'conda deactivate' C-m   
tmux send-keys -t limsim:4 'conda activate limsim-o' C-m   
tmux send-keys -t limsim:4 'cd /data/limsim-o/LimSimLLM;bash ./simModel_Carla/ExamplePDM.sh 4000 2112 1121102 pdm_1' C-m