 ros2 run detection_node yolo_subscriber --ros-args \
    -p confidence_threshold:=0.6 \
    -p output_dir:="./detections" # Example absolute path
