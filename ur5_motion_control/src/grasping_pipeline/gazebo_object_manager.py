#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetModelState
from geometry_msgs.msg import Pose

class GazeboObjectManager:
    def __init__(self, model_name="cube", model_path="path_to_model.sdf"):
        """
        Инициализация менеджера объектов.
        :param model_name: Имя объекта в Gazebo (по умолчанию 'cube').
        :param model_path: Путь к файлу модели (например, SDF или URDF).
        """
        self.model_name = model_name
        self.model_path = model_path
        self.spawn_service = "/gazebo/spawn_sdf_model"
        self.delete_service = "/gazebo/delete_model"
        self.get_state_service = "/gazebo/get_model_state"
        self.is_initialized = False

        # Инициализация ROS
        rospy.init_node("gazebo_object_manager", anonymous=True)

        # Проверка наличия сервисов Gazebo
        rospy.wait_for_service(self.spawn_service)
        rospy.wait_for_service(self.delete_service)
        rospy.wait_for_service(self.get_state_service)
        self.is_initialized = True

    def create_object(self, position=(0, 0, 0), orientation=(0, 0, 0, 1)):
        """
        Создает объект в рабочей области, если он еще не существует.
        :param position: Координаты (x, y, z) объекта.
        :param orientation: Ориентация (x, y, z, w) объекта в пространстве.
        :return: Статус операции.
        """
        if not self.is_initialized:
            raise RuntimeError("ROS services are not initialized.")

        # Проверка, существует ли объект
        if self.get_object_state():
            rospy.loginfo(f"Object '{self.model_name}' already exists in Gazebo.")
            return False

        try:
            with open(self.model_path, "r") as f:
                model_xml = f.read()
        except FileNotFoundError:
            rospy.logerr(f"Model file '{self.model_path}' not found.")
            return False

        # Создание объекта
        spawn_model = rospy.ServiceProxy(self.spawn_service, SpawnModel)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = orientation

        try:
            spawn_model(self.model_name, model_xml, "", pose, "world")
            rospy.loginfo(f"Object '{self.model_name}' successfully created.")
            return True
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to create object '{self.model_name}': {e}")
            return False

    def delete_object(self):
        """
        Удаляет объект из рабочей области.
        :return: Статус операции.
        """
        if not self.is_initialized:
            raise RuntimeError("ROS services are not initialized.")

        delete_model = rospy.ServiceProxy(self.delete_service, DeleteModel)

        try:
            delete_model(self.model_name)
            rospy.loginfo(f"Object '{self.model_name}' successfully deleted.")
            return True
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to delete object '{self.model_name}': {e}")
            return False

    def get_object_state(self):
        """
        Получает координаты объекта в пространстве Gazebo.
        :return: Словарь с позицией и ориентацией объекта или None, если объект не существует.
        """
        if not self.is_initialized:
            raise RuntimeError("ROS services are not initialized.")

        get_model_state = rospy.ServiceProxy(self.get_state_service, GetModelState)

        try:
            response = get_model_state(self.model_name, "world")
            if response.success:
                state = {
                    "position": {
                        "x": response.pose.position.x,
                        "y": response.pose.position.y,
                        "z": response.pose.position.z,
                    },
                    "orientation": {
                        "x": response.pose.orientation.x,
                        "y": response.pose.orientation.y,
                        "z": response.pose.orientation.z,
                        "w": response.pose.orientation.w,
                    },
                }
                rospy.loginfo(f"Object '{self.model_name}' state: {state}")
                return state
            else:
                rospy.logwarn(f"Object '{self.model_name}' does not exist.")
                return None
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to get state of object '{self.model_name}': {e}")
            return None
