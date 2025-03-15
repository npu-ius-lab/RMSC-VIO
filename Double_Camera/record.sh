gnome-terminal --window -e 'bash -c "cd ~/sh/Double_Camera;rosbag record /camera1/infra1/image_rect_raw /camera1/infra2/image_rect_raw /camera2/infra1/image_rect_raw /camera2/infra2/image_rect_raw /mavros/imu/data_raw ; exec bash"' \

