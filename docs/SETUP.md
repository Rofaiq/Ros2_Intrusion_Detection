# Setup: Ubuntu 24.04 + ROS 2 Jazzy + TurtleBot3 simulation

These steps follow the official guides:
[ROS 2 Jazzy install](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
and [TurtleBot3 simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/).
If a command fails, check those pages first; they are updated more often than this file.

## 1. Check your machine

```bash
lsb_release -a   # needs Ubuntu 24.04 (noble) for Jazzy
free -h          # 8 GB RAM is enough, 16 GB is comfortable
```

## 2. Install ROS 2 Jazzy

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe

# ROS apt source (installs the signing key and repository entry)
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop ros-dev-tools
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo rosdep init && rosdep update
```

Check it works: run `ros2 run demo_nodes_cpp talker` in one terminal and
`ros2 run demo_nodes_py listener` in another.

## 3. Install Gazebo and TurtleBot3

```bash
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-turtlebot3 \
    ros-jazzy-turtlebot3-simulations ros-jazzy-turtlebot3-teleop \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

Check it works:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

## 4. Build this project

```bash
mkdir -p ~/ids_ws/src && cd ~/ids_ws/src
git clone https://github.com/Rofaiq/ros2-robot-intrusion-detection.git
cd ~/ids_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
echo "source ~/ids_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Then follow the Quick start in the [README](../README.md).

## Troubleshooting

* **Gazebo window is black or very slow:** try `export LIBGL_ALWAYS_SOFTWARE=1`
  before launching, or close other heavy applications.
* **Robot does not move with teleop:** make sure you remapped teleop to
  `cmd_vel_in` and that `/ids/attack_detected` is false
  (`ros2 topic echo /ids/attack_detected`). Reset with
  `ros2 service call /ids/reset std_srvs/srv/Trigger`.
* **Nodes on two machines see each other unexpectedly:** set a private
  `export ROS_DOMAIN_ID=42` (any number 0-101) in `~/.bashrc`.
