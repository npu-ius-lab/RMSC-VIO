#开启mavros以及相机
gnome-terminal --window -e 'bash -c "cd ~/catkin_ws;source devel/setup.bash;roslaunch realsense2_camera rs_multiple_devices.launch serial_no_camera1:=135122070992 serial_no_camera2:=135122077361; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch mavros px4.launch; exec bash"' \



