# Getting Started

Ready to control your own robotic gripper? Let's get started.
First, grab the code:
```bash
git clone https://github.com/TUM-LSY/aloha4franka.git
cd aloha4franka
```

Connect your motors - They'll show up as `/dev/ttyACM0` or `/dev/ttyUSB0`.
Find your gripper's serial ID using [cyme](https://github.com/tuna-f1sh/cyme) (or `lsusb`).
Configure the udev rule - Update `scripts/99_gripper.rules` with your serial ID:

```bash
SUBSYSTEM=="tty", ATTRS{serial}=="FT9HDCKD", SYMLINK+="gripper_right", MODE="0666", ATTR{device/latency_timer}="1"
```

Build and launch:

```bash
ROS_DISTRO=humble docker compose build
ROS_DISTRO=humble NAMESPACE=right DEVICE=/dev/gripper_right docker compose up launch_aloha_gripper
```
to start the gripper.

Switch to Jazzy by changing `ROS_DISTRO`:

```bash
ROS_DISTRO=jazzy docker compose build
ROS_DISTRO=jazzy NAMESPACE=right DEVICE=/dev/gripper_right docker compose up launch_aloha_gripper
```

Ready to control! Use [CRISP after calibration](https://utiasdsl.github.io/crisp_controllers/misc/calibrate_gripper/) to start manipulating objects.

!!! Note "Using a different RMW"
    Multi-machine setup? For distributed systems using [crisp_py](https://github.com/utiasDSL/crisp_py), we recommend CycloneDDS or Zenoh middleware for better performance!
    ```bash
    ROS_DISTRO=humble RMW=cyclone NAMESPACE=right DEVICE=/dev/gripper_right docker compose up launch_aloha_gripper
    ```

## Cross-distro test matrix (Humble publisher, Jazzy consumer)

Use the matrix below to validate gripper topic discovery from a Humble publisher to a Jazzy consumer while keeping the same `NAMESPACE` and `DEVICE` mapping.

| Publisher command | Jazzy consumer command | Expected result |
|---|---|---|
| `ROS_DISTRO=humble NAMESPACE=right DEVICE=/dev/gripper_right docker compose up launch_aloha_gripper` | `docker run --rm --net=host --ipc=host --pid=host osrf/ros:jazzy-desktop bash -lc "source /opt/ros/jazzy/setup.bash && ros2 topic list | grep '^/right/gripper/'"` | Topics under `/right/gripper/` are discovered. |
| `ROS_DISTRO=humble RMW=cyclone NAMESPACE=right DEVICE=/dev/gripper_right docker compose up launch_aloha_gripper` | `docker run --rm --net=host --ipc=host --pid=host -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -v $PWD/scripts/cyclone_config.xml:/tmp/cyclone_config.xml:ro -e CYCLONEDDS_URI=file:///tmp/cyclone_config.xml osrf/ros:jazzy-desktop bash -lc "source /opt/ros/jazzy/setup.bash && ros2 topic list | grep '^/right/gripper/'"` | CycloneDDS discovery succeeds with matching `RMW_IMPLEMENTATION` and Cyclone config. |
| `ROS_DISTRO=humble NAMESPACE=left DEVICE=/dev/gripper_left docker compose up launch_aloha_gripper` | `docker run --rm --net=host --ipc=host --pid=host osrf/ros:jazzy-desktop bash -lc "source /opt/ros/jazzy/setup.bash && ros2 topic list | grep '^/left/gripper/'"` | Namespace remains isolated to `/left/gripper/`. |
