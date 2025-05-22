    source /opt/ros/humble/setup.bash 
    colcon build --packages-select  camera_node
    source install/setup.bash
    # run camear node
    sh run_camera_node.sh 
    # run yolo detextion
    sh run_detection_node.sh
    # run logic node and communication node
    sh run_logic_comm.sh
