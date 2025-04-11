echo "source /opt/ros/$ROS_DISTRO/setup.zsh" >> ~/.zshrc
# echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

# --- If using UV ---
# source /opt/ros/$ROS_DISTRO/setup.bah
# source ~/ros2_ws/install/setup.bash
# cd ~/ros2_ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON 
# cd ~/ros2_ws/src/lsy_franka
# uv venv
# uv sync
# echo "export PYTHONPATH=~/ros2_ws/src/this_project/.venv/lib/python3.10/site-packages:$PYTHONPATH" >> ~/.bashrc
# echo "source ${UV_PROJECT_ENVIRONMENT}/bin/activate" >> ~/.bashrc
