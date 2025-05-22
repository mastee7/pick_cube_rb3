ros2 run camera_node image_publisher_saver --ros-args \
    -p camera_index:=1 \
    -p save_path:="/home/ubuntu/bin/ros2_ws" \
    -p capture_interval_sec:=5.5 \
    -p filename_prefix:="capture_front_"
