# Jetbot ROS2 Package

### Install ROS2 Eloquent
```bash
# setup sources
$ sudo apt update && sudo apt install curl gnupg2 lsb-release
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
$ sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

# install ROS 2 packages
$ sudo apt update && sudo apt install ros-eloquent-ros-base

# add ROS paths to environment
echo "source /opt/ros/eloquent/setup.bash" >> ~/.bashrc
```

### Other setup
Grant your user access to the i2c bus:

```bash
$ sudo usermod -aG i2c $USER
```

Reboot the system for the changes to take effect.

### Building jetbot_ros2
```bash
$ sudo apt install python3-argcomplete python3-colcon-common-extensions libboost-system-dev
$ mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
$ git clone https://github.com/rosterloh/jetbot_ros2

# build the package
$ cd ~/ros2_ws/
$ source /opt/ros/dashing/setup.bash
$ colcon build --symlink-install --parallel-workers 2
```