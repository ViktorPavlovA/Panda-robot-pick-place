# Решение задачи Pick&Place

Версия ROS - Noetic

Выполнили: Павлов В., Никита Г., Балахонов С.


![alt text](https://ie.wampi.ru/2023/01/12/DAkeqI0Ep50.jpg)

## Установка зависимостей и пакетов

**Проверка и обноление пакетов**

`sudo apt update`

`sudo apt upgrade`

`rosdep update`

**Установка пакетов panda_robot**

Перейдите по ссылке и выполните инструкции 

[Здесь](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html)

**Также в catkin\src**

необходимо склонировать репозиторий 

`git clone https://github.com/frankaemika/franka_ros.git`

## Замена файлов

Взять из репозитория папку `panda_moveit_config` и заменить ее на ту что есть.

Затем в пакете `franka_ros` аналогичным образом заменить `franka_gazebo`

Возможно понадобится сделать 

`catkin_make`

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

**Параметры куба**

![alt text](https://ic.wampi.ru/2023/01/11/n5NWBNJCe8U.jpg)

# FIN
## Вы шикарны

![alt text](https://ie.wampi.ru/2023/01/11/S9TT55N79Tk.jpg)


