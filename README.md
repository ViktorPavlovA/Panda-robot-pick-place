# Решение задачи Pick&Place

Версия ROS - Noetic

Выполнили: Павлов В., Никита Г., Балахонов С.

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

`sudo apt update`

Предполагается что уже имеется рабочее пространство catkin_ws/src
`cd catkin/src/`
`git clone https://github.com/ViktorPavlovA/Panda-robot-pick-place.git`

Переходим в catkin_ws

`cd ..`

`catkin_make`

** команда выше может долго выполняться +- 5-10 мин**

`source ~/catkin_ws/devel/setup.bash`

OR

`echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc`

## Запуск

`roslaunch panda_moveit_config demo_gazebo.launch world:=$(rospack find panda_moveit_config)/world/stone.sdf`

**Запуск скрипта управления**

`cd`

`cd catkin_ws/src/control_script/scripts/`

**Запустить симуляцию pick&place**

`python3 cycle_script.py`

**Вспомогательный скрипт для отоброжения положения звеньев в [rad]**

`python3 check_joint_position.py`

**Вспомогательный скрипт для проверки эндефектора**

`python3 check_gripper.py`

** Параметры куба **
![alt text]([https://ic.wampi.ru/2023/01/11/n5NWBNJCe8U.jpg])

# FIN
## Вы шикарны




