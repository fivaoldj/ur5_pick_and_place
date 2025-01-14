import random
import rospy
import math
import numpy as np
from robot_control import RobotControlUR5
from gazebo_object_manager import GazeboObjectManager
from csv_logger import CSVLogger

import sys
import os

# Добавляем соседнюю папку в sys.path и подключаем оттуда 
# файлик, который обрабатывает глубинную карту
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../ggcnn'))
sys.path.append(parent_dir)

from image_process import GGCNNGraspDetector

def main():

    rospy.init_node("grasping_master", anonymous=True)
    rate = rospy.Rate(1)

    # Создадим класс для верхнеуровнего управления роботом
    robot = RobotControlUR5()
    # Оптимальная высота для захвата объекта (подобрана согласно параметрам мира)
    OPTIMAL_HEIGHT_OF_GRASP = 0.36

    # Создадим класс для добавления или удаления объекта на рабочую область
    # и удалим объект, если он уже существует
    model_name = "cube"
    object_manager = GazeboObjectManager(model_name=model_name, model_path= \
                                         "../ur5_gripper_control/urdf/cube_red.urdf")
    object_manager.delete_object()

    # Создадим класс для логирования результатов эксперимента в csv файл
    csv_logger = CSVLogger("./docs/experiment_1.csv")

    # Создадим класс для детектирования результатов работы нейросети
    ggcnn_grasp_detector = GGCNNGraspDetector()

    grasp_params = None

    iterations = 0
    while (True):
        try:
            iterations += 1
            rospy.loginfo(f"Итерация номер: [{iterations}]")

            # 1. Создать объект в рабочей области камеры
            object_manager.create_object(position=(0.7, 0.1, 0.5), \
                                        orientation=(random.uniform(0.0, 1.0), \
                                                        random.uniform(0.0, 1.0), \
                                                        random.uniform(0.0, 1.0), \
                                                        random.uniform(0.0, 1.0)))
            # 2. Ждём, пока объект упадёт на рабочую область и стабилизируется на ней
            rospy.sleep(4)
            # robot.go_to(x=0.2, y=0, z=0.7)
            
            # 3. Проверим, появились ли параметры от узла обработки изображения, если появились,
            # то выведем параметры и запишем структуру данных в переменную
            if ggcnn_grasp_detector.grasp_params:
                grasp_params = ggcnn_grasp_detector.grasp_params
                rospy.loginfo(f"Grasp Params: {grasp_params}")
            
            
            # Параметры камеры
            focal_length = 640.0  # Фокусное расстояние (в пикселях)
            cx = 960 / 2  # Центр изображения по оси X
            cy = 540 / 2   # Центр изображения по оси Y
            camera_height = 0.65 - 0.16  # Высота камеры (в метрах)

            # Параметры захвата из нейросети
            x_pixel = grasp_params["x"]  # Координата захвата по оси X (в пикселях)
            y_pixel = grasp_params["y"]  # Координата захвата по оси Y (в пикселях)

            # Шаг 1: Смещение пикселя от центра изображения
            dx = x_pixel - cx
            dy = y_pixel - cy

            # Шаг 2: Угол смещения от нормали камеры
            alpha = np.arctan(np.sqrt(dx**2 + dy**2) / focal_length)

            # Шаг 3: Вычисление глубины точки
            z_calculated = camera_height / np.cos(alpha)

            # Перевод из пикселей в метры в системе координат камеры
            x_camera = (x_pixel - cx) * z_calculated / focal_length
            y_camera = (y_pixel - cy) * z_calculated / focal_length
            z_camera = z_calculated  # Используем вычисленную глубину

            # Логгирование координат в системе камеры
            rospy.loginfo(f"Координаты в системе камеры: x={x_camera}, y={y_camera}, z={z_camera}")

            # Трансформация из системы координат камеры в систему робота
            T_camera_to_robot = np.array([
                [0, -1, 0, 0.7],  # Ось X камеры -> отрицательная ось Y робота
                [-1, 0, 0, 0],    # Ось Y камеры -> отрицательная ось X робота
                [0, 0, -1, camera_height],  # Ось Z камеры -> отрицательная ось Z робота
                [0, 0, 0, 1]      # Однородные координаты
            ])

            # Однородные координаты точки в системе камеры
            camera_coords = np.array([x_camera, y_camera, z_camera, 1])

            # Преобразование координат из системы камеры в систему робота
            robot_coords = T_camera_to_robot @ camera_coords
            x_robot, y_robot, z_robot = robot_coords[:3]  # Отбрасываем однородную компоненту

            # Логгирование координат в системе робота
            rospy.loginfo(f"Координаты в системе робота: x={x_robot}, y={y_robot}, z={z_robot}")



            robot.to_grasp(x=0.7-y_camera, y=0.0-x_camera,  z=OPTIMAL_HEIGHT_OF_GRASP, angle=grasp_params["theta"]*180/math.pi-90)
            rospy.sleep(0.5)
            
            robot.close_gripper()
            robot.to_home()
            object_state = object_manager.get_object_state()
            if (object_state["position"]["z"] >= 0.4):
                csv_logger.log(True, model_name)
                rospy.sleep(1)
            else:
                csv_logger.log(False, model_name)
            object_manager.delete_object()
            robot.open_gripper()
        except Exception as e:
            csv_logger.log(False, f"{model_name} {e}")
            object_manager.delete_object()

if __name__ == "__main__":
    main()