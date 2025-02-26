![melodic](https://github.com/NOKOV-MOCAP/mocap_nokov/actions/workflows/melodic.yml/badge.svg)
![noetic](https://github.com/NOKOV-MOCAP/mocap_nokov/actions/workflows/noetic.yml/badge.svg)

# mocap_nokov
ROS nodes for working with the Nokov motion capture system

## Usage

```
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/catkin_ws/src
git clone https://github.com/NOKOV-MOCAP/mocap_nokov.git
cd ~/catkin_ws
rosdep install --from-paths src -y --ignore-src
catkin_make
source /devel/setup.bash
```

nokov@ubuntu:~/catkin_ws/src/mocap_nokov/config$ gedit mocap.yaml

```
rigid_bodies:
    '1':
        pose: Robot_1/pose
        pose2d: Robot_1/ground_pose
        odom: Robot_1/Odom
        tf: tf
        child_frame_id: Robot_1/base_link
        parent_frame_id: world
    '2':
        pose: Robot_2/pose
        pose2d: Robot_2/ground_pose
        odom: Robot_2/Odom
        tf: tf
        child_frame_id: Robot_2/base_link
        parent_frame_id: world
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
roslaunch mocap_nokov mocap.launch
```

```Then with `rostopic list`, you should be able to see the following topics

```bash
/mocap_node/<tracker_name>/ground_pose
/mocap_node/<tracker_name>/pose
/mocap_node/<tracker_name>/Odom
```
where `<tracker_name>` is usually the name of your tracked objects.