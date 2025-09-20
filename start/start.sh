#!/bin/bash

# Set password
password="123"

# Function to check if a command was successful
# check_status() {
#     if [ $? -ne 0 ]; then
#         echo "Error: $1 failed"
#         exit 1
#     fi
# }

# Configure CAN interface
echo "Configuring CAN interface..."
echo $password | sudo -S ip link set can0 up type can bitrate 500000
# check_status "CAN interface configuration"

# Launch terminator with custom layout
terminator -l nav &
sleep 2  # Wait for terminator to initialize


# Set USB permissions and launch LiDAR camera nodes
echo "Setting USB permissions and launching sensors..."
terminator -e "echo '123' | sudo -S chmod 666 /dev/ttyUSB0 /dev/ttyUSB1 && cd /home/cjh/nav_ws && source devel/setup.bash && roslaunch zj_nav zj_lidarcamera.launch" --new-tab -T "LiDAR_Camera"
sleep 2  # Wait for sensors to initialize

# Launch localization and aruco
echo "Launching localization and aruco..."
terminator -e "cd /home/cjh/nav_ws && source devel/setup.bash && roslaunch zj_nav zj_localize.launch" --new-tab -T "Localization"
sleep 2

# Publish initial pose in the first terminal
echo "Publishing initial pose..."
terminator -e "cd /home/cjh/zhujiang2_ws && source devel/setup.bash && rostopic pub /start_lio std_msgs/Bool true --once" --new-tab -T "Initialization"
sleep 15

# Launch navigation
echo "Launching navigation..."
terminator -e "cd /home/cjh/nav_ws && source devel/setup.bash && roslaunch zj_nav zj_movebase.launch" --new-tab -T "Navigation"
sleep 5

# Configure CAN interface
echo "Configuring CAN interface..."
echo $password | sudo -S ip link set can0 up type can bitrate 500000
# check_status "CAN interface configuration"

# Configure CAN and launch Tracer
echo "Configuring CAN interface and launching Tracer base..."
terminator -e "cd /home/cjh/zhujiang2_ws && source devel/setup.bash && roslaunch master_pkg start.launch" --new-tab -T "Master"

echo "All systems initialized successfully!"