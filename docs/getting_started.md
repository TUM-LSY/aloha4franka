# Getting Started

Ready to control your own robotic gripper? Let's get started.

## Create a symlink

Connect your motors, to the power and the USB to your computer - They'll show up as `/dev/ttyACM0` or `/dev/ttyUSB0`.
Find your gripper's serial ID using [cyme](https://github.com/tuna-f1sh/cyme) (or `lsusb`).
Configure the udev rule - Update `scripts/99_gripper.rules` with your serial ID:

```bash
SUBSYSTEM=="tty", ATTRS{serial}=="FT9HDCKD", SYMLINK+="name_for_your_gripper", MODE="0666", ATTR{device/latency_timer}="1"
```

!!! Warning
    If you skip this step, you will have to manually set the mode of the gripper every time you power it off with:
    ```bash
    sudo chmod 666 /dev/ttyUSB0
    ```
    Also, it might be that the device name varies every time you plug it in.
    We highly recommend not skipping this step.

## Use the gripper

You can use the gripper directly using the [dynamixel-wrapper](https://github.com/danielsanjosepro/dynamixel_wrapper), 
which is a simple wrapper around the dynamixel python sdk.

It als contains a script to create a calibration file for your gripper, to find the ends of the robot.
If you use the [ROS2 script](https://github.com/danielsanjosepro/dynamixel_wrapper) you can control the gripper from 
[CRISP_PY](https://github.com/utiasDSL/crisp_py).

Enjoy the gripper!
