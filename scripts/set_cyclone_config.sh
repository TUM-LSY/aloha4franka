export ROS_DOMAIN_ID=100
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/ros/ros2_ws/src/aloha4franka/scripts/cyclone_config.xml
ros2 daemon stop && ros2 daemon start
