# ur5_moveit_pick-place

## Описание
Проект для решения задачи pick&place на базе ur5 с использованием ros-noetic и moveit.

## Зависимости
Для работы необходимо установить пакеты: 
```
ros-noetic-moveit
ros-noetic-moveit-visual-tools
ros-noetic-gazebo-ros-pkgs
```

Необходимо установить [сборщик пакетов catkin](https://catkin-tools.readthedocs.io/en/latest/installing.html)

Также необходимо установить MoveIt [конфиги для UR5](https://github.com/ros-industrial/universal_robot.git).

## Сборка
Сборка осуществляется из папки проекта `ur5_pick_and_place/ur5_motion_control` следующийм образом:

```
$ catkin build
```

## Запуск
### Запуск окружения для работы в gazebo и планирования движения в moveit 
#### Запуск манипулятора
Этот пункт опционален и необходим только для каких-то базовых тестов, чтобы посмотреть на робота и поуправлять им.
```
# Запуск gazebo с моделью ur5e
roslaunch ur_gazebo ur5e_bringup.launch

# Запуск планера moveit 
roslaunch ur5e_moveit_config moveit_planning_execution.launch sim:=true

# Запуск rviz для управления роботом оттуда
roslaunch ur5e_moveit_config moveit_rviz.launch
```
#### Запуск схвата
Этот пункт опционален и необходим только для каких-то базовых тестов, чтобы посмотреть на схват и поуправлять им.
```
# Запуск схвата в среде gazebo
roslaunch robotiq_2f_85_gripper_gazebo robotiq_2f_85_bringup.launch

# Отправка сообщения в топик на закрытие схвата
rostopic pub /gripper_controller/gripper_cmd/goal control_msgs/GripperCommandActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  command:
    position: 1.0
    max_effort: 0.5" 

```

### Запуск проекта 
```
$ source devel/setup.bash
$ roslaunch ur5_moveit_config demo.launch
$ rosrun cobot_test pick_and_place_node
```
  
Работа pick&place начинается по инициализации "next" в RViz
