![rolling](https://github.com/NOKOV-MOCAP/mocap_nokov/actions/workflows/rolling.yml/badge.svg)
![jazzy](https://github.com/NOKOV-MOCAP/mocap_nokov/actions/workflows/jazzy.yml/badge.svg)
![jazzy_arm64](https://github.com/NOKOV-MOCAP/mocap_nokov/actions/workflows/arm64_jazzy.yml/badge.svg)
![iron](https://github.com/NOKOV-MOCAP/mocap_nokov/actions/workflows/iron.yml/badge.svg)
![humble](https://github.com/NOKOV-MOCAP/mocap_nokov/actions/workflows/humble.yml/badge.svg)
![humble_arm32](https://github.com/NOKOV-MOCAP/mocap_nokov/actions/workflows/arm32_humble.yml/badge.svg)
![foxy](https://github.com/NOKOV-MOCAP/mocap_nokov/actions/workflows/foxy.yml/badge.svg)
![galactic](https://github.com/NOKOV-MOCAP/mocap_nokov/actions/workflows/galactic.yml/badge.svg)

# mocap_nokov
ROS2 nodes for working with the Nokov motion capture system

## Usage

```
sudo apt install -y \
     ros-$ROS_DISTRO-geometry-msgs \
     ros-$ROS_DISTRO-tf2 \
     ros-$ROS_DISTRO-tf2-ros \
     ros-$ROS_DISTRO-ament-cmake \
     ros-$ROS_DISTRO-ament-lint-auto \
     ros-$ROS_DISTRO-ament-lint-common \
     ros-$ROS_DISTRO-rclcpp
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/ros2_ws/src
git clone -b ros2 https://github.com/NOKOV-MOCAP/mocap_nokov.git
cd ~/ros2_ws
colcon build --packages-select mocap_nokov --symlink-install
source install/setup.bash
```

nokov@ubuntu:~/ros2_ws/src/mocap_nokov/config$ gedit mocap.yaml

```
mocap_node:
    ros__parameters:
        rigid_bodies:
            1:
                pose: "Robot_1/pose"
                pose2d: "Robot_1/ground_pose"
                child_frame_id: "Robot_1/base_link"
                parent_frame_id: "world"
            2:
                pose: "Robot_2/pose"
                pose2d: "Robot_2/ground_pose"
                child_frame_id: "Robot_2/base_link"
                parent_frame_id: "world"
        nokov_config:
                server_address: "10.1.1.198"
```

This is a ROS parameter configuration file for `mocap_nokov`. Here is a description of each parameter:

- `rigid_bodies` : Add rigid body to track.
- `1:` : Asset number of rigid body.
- `server_address` : indicates the IP address of the MOCAP server. In this example, the server's IP address is' 10.1.1.198 '.

### Launch Default Configuration from Command Line

Run the following command,

```bash
ros2 launch mocap_nokov mocap.launch.py
```

```Then with `ros2 topic list`, you should be able to see the following topics

```bash
/<tracker_name>/ground_pose
/<tracker_name>/pose
```
where `<tracker_name>` is usually the name of your tracked objects.