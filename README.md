# Решение задачи Pick&Place

Версия ROS - Noetic

## Установка зависимостей и пакетов

**Проверка и обноление пакетов**

`sudo apt update`

`sudo apt upgrade`

`rosdep update`

**Установка пакетов panda_robot**
[Здесь](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html)

`sudo apt dist-upgrade`

`sudo apt install ros-noetic-catkin python3-catkin-tools python3-osrf-pycommon`

`sudo apt install python3-wstool`

Предполагается что уже имеется рабочее пространство catkin_ws

`cd ~/catkin_ws/src`

`wstool init .`

`wstool merge -t . https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall`

`wstool remove  moveit_tutorials`

`wstool update -t .`

`git clone https://github.com/ros-planning/moveit_tutorials.git -b master`

`git clone https://github.com/ros-planning/panda_moveit_config.git -b noetic-devel`

 `git clone https://github.com/frankaemika/franka_ros/tree/noetic-devel`

`rosdep install -y --from-paths . --ignore-src --rosdistro noetic`

`sudo sh -c 'echo "deb http://packages.ros.org/ros-testing/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`

`sudo apt update`

Переходим в catkin_ws

`cd ..`

`catkin_make`

**команда выше может долго выполняться +- 5-10 мин**

`source ~/catkin_ws/devel/setup.bash`

OR

`echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc`

**Необходимо заменить папку franka_gazebo **

`cd`

`cd catkin/src/franka_ros/`


Сохранить

## Запуск

`roslaunch panda_moveit_config demo_gazebo.launch world:=$(rospack find panda_moveit_config)/world/stone.sdf`

**Запуск скрипта управления**

`cd`

`cd catkin_ws/src/control_script/scripts/`

**Запустить симуляцию pick&place**

`pyhton3 cycle_script.py`

**Вспомогательный скрипт для отоброжения положения звеньев в [rad]**

`python3 check_joint_position.py`

**Вспомогательный скрипт для проверки эндефектора**

`python3 check_gripper.py`

# FIN
## Вы шикарны




