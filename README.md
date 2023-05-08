<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `l3xz_watchdog`
==============================
[![Build Status](https://github.com/107-systems/l3xz_watchdog/actions/workflows/ros2.yml/badge.svg)](https://github.com/107-systems/l3xz_watchdog/actions/workflows/ros2.yml)
[![Spell Check status](https://github.com/107-systems/l3xz_watchdog/actions/workflows/spell-check.yml/badge.svg)](https://github.com/107-systems/l3xz_watchdog/actions/workflows/spell-check.yml)

This packages supervises the state of all of L3X-Z's ROS sub-systems.

<p align="center">
  <a href="https://github.com/107-systems/l3xz"><img src="https://raw.githubusercontent.com/107-systems/.github/main/logo/l3xz-logo-memento-mori-github.png" width="40%"></a>
</p>

#### How-to-build
```bash
cd $COLCON_WS/src
git clone https://github.com/107-systems/l3xz_watchdog
cd $COLCON_WS
source /opt/ros/humble/setup.bash
colcon build --packages-select l3xz_watchdog
```

#### How-to-run
```bash
cd $COLCON_WS
. install/setup.bash
ros2 launch l3xz_watchdog watchdog.py
```

#### Interface Documentation
##### Subscribed Topics
|       Default name        |                                     Type                                     |
|:-------------------------:|:----------------------------------------------------------------------------:|
| `/l3xz/${node}/heartbeat` | [`std_msgs/UInt64`](https://docs.ros2.org/foxy/api/std_msgs/msg/UInt64.html) |
|   `/l3xz/estop/actual`    |   [`std_msgs/Bool`](https://docs.ros2.org/foxy/api/std_msgs/msg/Bool.html)   |

##### Published Topics
|        Default name       |                                   Type                                   |
|:-------------------------:|:------------------------------------------------------------------------:|
| `/l3xz/light_mode/target` | [`std_msgs/Int8`](https://docs.ros2.org/foxy/api/std_msgs/msg/Int8.html) |

##### Parameters
|      Name      |         Default         | Description                                                               |
|:--------------:|:-----------------------:|---------------------------------------------------------------------------|
| `config_file` | `watchdog-config.json`  | Complete file path to the JSON file containing the nodes to be monitored. |
