# ur5_moveit_pick-place

### Описание
Проект для решения задачи pick&place на базе ur5 с использованием ros-noetic и moveit.
Для работы необходимо установить пакеты: 
```
ros-noetic-moveit
ros-noetic-moveit-visual-tools

```

Необходимо установить [сборщик пакетов catkin](https://catkin-tools.readthedocs.io/en/latest/installing.html)

Также необходимо установить MoveIt [конфиги для UR5](https://github.com/ros-industrial/universal_robot.git).

### Сборка
Сборка осуществляется из папки проекта `ur5_pick_and_place/ur5_motion_control` следующийм образом:

```
$ catkin build
```

### Запуск
Запуск осуществялется следующим образом
```
$ source devel/setup.bash
$ roslaunch ur5_moveit_config demo.launch
$ rosrun cobot_test pick_and_place_node
```
  
Работа pick&place начинается по инициализации "next" в RViz
