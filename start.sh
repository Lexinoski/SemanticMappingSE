#!/bin/bash

# Iniciar o servidor tmux
tmux new-session -d -s SensoresLV_DE

# Iniciar o ROS Master em uma nova janela do tmux
tmux new-window -t SensoresLV_DE -n "ROS Master" "roscore"

# Esperar um pouco para o ROS Master inicializar completamente
sleep 2

# Iniciar os outros componentes do ROS em abas separadas
if [ "$1" != "local" ]; then

    tmux new-window -t SensoresLV_DE -n "Joy" "echo lactec123 | sudo -S chmod a+rw /dev/input/js0 && rosparam set joy_node/dev "/dev/input/js0" && rosrun joy joy_node"
    tmux new-window -t SensoresLV_DE -n "CoppeliaSim" "cd ./modules/CoppeliaSim_Edu_V4_0_0_Ubuntu18_04/ && ./coppeliaSim.sh ./MyScene/SE_LacBotv6_6.ttt"
    tmux new-window -t SensoresLV_DE -n "Float32toPC2" "cd ./modules/float32multiarray_to_pointcloud2-master/launch/ && roslaunch float32multiarray_to_pointcloud2 float32multiarray_to_pointcloud2.launch"
    tmux new-window -t SensoresLV_DE -n "Publish TF" "sleep 20 && cd ./modules && python2 tfs_pub_node.py"
    tmux new-window -t SensoresLV_DE -n "DepthEstimation" "cd ./modules && python2 depth_estimation_node.py"
    tmux new-window -t SensoresLV_DE -n "YoloV7 Detection" "cd ./modules/yolov7-main && python3 detect_v8.py --weights best_english.pt"
    tmux new-window -t SensoresLV_DE -n "DBScan to Markers" "cd ./modules && python2 dbscan_markers_node.py"
    #gnome-terminal -- bash -c "sleep 1; ./ROS_Record.sh; exec bash -i"

fi

if [ "$1" = "local" ]; then
    tmux new-window -t SensoresLV_DE -n "Intelbros Node" "cd ~/Projects/SensorLV/scripts/ && python3 ./modules/intelbros_node.py"
    tmux new-window -t SensoresLV_DE -n "YoloV7 Detection" "cd ~/Projects/SensorLV/scripts/modules/yolov7-main_customized && python3 detect_v7.py"
    tmux new-window -t SensoresLV_DE -n "TFS Publisher" "cd ~/Projects/SensorLV/scripts/ && python2 ./modules/tfs_pub_node.py"
fi

# Anexar ao servidor tmux para visualizar as janelas
tmux attach -t SensoresLV_DE

