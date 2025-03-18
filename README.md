# ROS2 example of SONY spresense multi-imu addon board

This repository contains `pwbimu_logger` wrapper code, just for testing, not for production.

Before run this, bake the example at [https://github.com/sonydevworld/spresense/tree/master/examples/cxd5602pwbimu](https://github.com/sonydevworld/spresense/tree/master/examples/cxd5602pwbimu).

## Run

Install `serial` python library beforehand, like:

```bash
sudo apt install python3-serial
```

Clone this repository into your ros2 workspace, like:

```bash
cd ~/colcon_ws/src
git clone https://github.com/vrano714/spr-imu-ros.git spr_imu
```

Build

```bash
cd ~/colcon_ws
colcon build --symlink-install --packages-select spr_imu
```

Run

(Tested only in ROS2 Galactic)

```bash
source ~/colcon_ws/install/setup.bash
ros2 launch spr_imu imu.launch.xml
```

When successfully running, imu timecode should be shown on your terminal and you can check its output in `/imu/raw` topic (by default, changable).

## Change settings

Configuring `launch/imu.launch.xml` as you like.

Compatible values are same as those of `pwbimu_logger`.
