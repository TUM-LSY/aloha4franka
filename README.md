![Aloha4franka](media/gripper_5.png)

# Aloha4Franka

<a href="https://github.com/danielsanjosepro/ros2_docker_template/actions/workflows/docker_build.yml"><img src="https://github.com/danielsanjosepro/ros2_docker_template/actions/workflows/docker_build.yml/badge.svg"/></a>

A cheap gripper - estimated 350€ - with real-time control capabilities and compatible with any cobot.
The gripper can be used with [CRISP](https://utiasdsl.github.io/crisp_controllers/) to record data as well as deploying learning-based policies.

This repository contains:
- An assembly guide as well as URDF files for an aloha gripper modification that allows it to be mounted on end-effector complying with ISO 9409-1, like FR3 of Franka Robotics, UR3 of Universal Robots...
- A simple control setup to test the gripper in ROS2o
- A handle with a trigger to allow for teleoperation and gripper control in a leader-follower system.


## Assembly guide

👷 TODO 👷

## Getting started

First clone this repository:
```bash
git clone https://github.com/TUM-LSY/aloha4franka.git
cd aloha4franka
```
Plug the motors to your computer, they should be listed as `/dev/ttyACM0` or `/dev/ttyUSB0`.
Check out using a tool like [cyme](https://github.com/tuna-f1sh/cyme) the serial-id for the gripper.
Modify `scripts/99_gripper.rules` with your gripper serial-id:

```bash
SUBSYSTEM=="tty", ATTRS{serial}=="FT9HDCKD", SYMLINK+="gripper_right", MODE="0666", ATTR{device/latency_timer}="1"
```

Now build and run

```bash
docker compose build
NAMESPACE=right DEVICE=/dev/gripper_right docker compose up launch_aloha_gripper
```
to start the gripper.

You can control the gripper using [CRISP after calibration](https://utiasdsl.github.io/crisp_controllers/misc/calibrate_gripper/).

> [!WARNING]  
> If you work in different machines (using [crisp_py](https://github.com/utiasDSL/crisp_py) or others) you might want to consider using CycloneDDS or Zenoh as you ROS middleware.
> Simply run the cyclone/zenoh version `RMW=cyclone NAMESPACE=right DEVICE=/dev/gripper_right docker compose up launch_aloha_gripper`

## Part list

| Quantity | Part | Link for Germany | Price |
| --- | --- | --- | --- |
| 2 | Linear rails | [RS Online](https://de.rs-online.com/web/p/linearfuhrungsschiene/1766658) ||
| 2 | Linear rail carriage | [RS Online](https://de.rs-online.com/web/p/linearfuhrungsblocke/1766655?searchId=f66cdd79-6102-401d-bbf1-d98e68436d04&gb=s) ||
| 6 | Ball Bearing | [Conrad](https://www.conrad.de/de/p/reely-kugellager-radial-edelstahl-innen-durchmesser-3-mm-aussen-durchmesser-6-mm-drehzahl-max-80000-u-min-1359926.html) ||
| 1 | Servomotor Dynamixel XC430-W150-T  | [MyBotShop](https://www.mybotshop.de/DYNAMIXEL-XC430-W150-T) ||
| 2 | M3 Locknut | [Conrad](https://www.conrad.de/de/p/toolcraft-812808-sicherungsmuttern-m3-din-985-stahl-verzinkt-100-st-812808.html?hk=SEM&WT.mc_id=google_pla&utm_source=google&utm_medium=cpc&utm_campaign=DE+-+PMAX+-+Brand+-+All+products&utm_id=21116787988&gad_source=1&gclid=Cj0KCQjw782_BhDjARIsABTv_JANTH5jU4y1pHcT1HjQwnI3sQFuUyB9l2tfVgSDyTW5K7AZeIDC-u4aAqDdEALw_wcB)||
| 1 | U2D2 Controller | [Reichelt](https://www.reichelt.de/de/de/shop/produkt/dynamixel_u2d2-263256?PROVID=2788&gad_source=1&gclid=CjwKCAjwktO_BhBrEiwAV70jXnXg4zTiIdI9f9MVoYfChmQ_a8Ncw8nTg040xoYzePDXbz_71C89ABoClOgQAvD_BwE)||
| 8 | Normal head M2 x 8mm for finger adaptor on linear rails |  |  |
| 2 | Normal head M2 x 10 mm for motor disk |  |  |
| 4 | Flathead M3 x 12 mm for linear + backplate connection |  |  |
| 4 | Flathead M2 x 10 mm for linear on rail |  |  |
| 6 | Flathead M2 x 20 mm for fingers |  |  |
| 4 | Flathead M2.5 x 12 mm for motor attachment |  |  |
| 2 | Normal head M3 x 20-22 mm for u-shape connection to finger adaptor |  |  |
| 2 | Flathead M3 x 10 mm for u-shape connection to rotor |  |  |

Tools for assembly
- M2 Tap [Conrad](https://www.conrad.de/de/p/eventus-by-exact-40701-gewindereparaturbohrer-m2-1-st-1224686.html)
- M3 Tap [Conrad](https://www.conrad.de/de/p/exact-05937-einschnittgewindebohrer-7teilig-metrisch-m3-m4-m5-m6-m8-m10-rechtsschneidend-din-3126-hss-1-set-813035.html)
- Adjustable tap wrench [Conrad](https://www.conrad.de/de/p/exact-04971-windeisen-m1-m8-din-1814-816718.html)

## Similar projects

Check out [Actuated UMI](https://github.com/actuated-umi/actuated-umi-gripper), which is a nice alternative gripper using an UMI like gripper.
We also plan to use rayfin fingers in a future iteration...
