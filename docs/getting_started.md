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
docker compose build
NAMESPACE=right DEVICE=/dev/gripper_right docker compose up launch_aloha_gripper
```
to start the gripper.

Ready to control! Use [CRISP after calibration](https://utiasdsl.github.io/crisp_controllers/misc/calibrate_gripper/) to start manipulating objects.

!!! Note "Using a different RMW"
    Multi-machine setup? For distributed systems using [crisp_py](https://github.com/utiasDSL/crisp_py), we recommend CycloneDDS or Zenoh middleware for better performance!
    ```bash
    RMW=cyclone NAMESPACE=right DEVICE=/dev/gripper_right docker compose up launch_aloha_gripper
    ```
