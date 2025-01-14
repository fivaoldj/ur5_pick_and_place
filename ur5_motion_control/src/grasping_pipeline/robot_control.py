#!/usr/bin/env python3

import sys
import moveit_commander
import math
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from std_srvs.srv import Empty
from ur5_gripper_control.srv import FilterWorkspace, FilterWorkspaceRequest

from tf.transformations import quaternion_from_euler, quaternion_multiply

class RobotControlUR5:
    def __init__(self):
        # Initialize MoveIt commander и rosnode
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('pick_place', anonymous=False)
        
        # Octomap topics and services
        self.camera_topics = ['camera_1_depth', 'camera_2_depth']
        self.clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
        self.publish_octomap = rospy.ServiceProxy('/filter_workspace', FilterWorkspace)
        
        # Diagnostic publisher for waypoint poses
        self.pose_pub = rospy.Publisher('/checker',PoseStamped,latch=True,queue_size=5)
        
        # Initialise robot and move groups
        self.robot = moveit_commander.robot.RobotCommander()
        self.arm_group = moveit_commander.move_group.MoveGroupCommander("ur5_arm")
        self.gripper_group = moveit_commander.move_group.MoveGroupCommander("gripper")
        
        self.arm_group.set_max_velocity_scaling_factor(1)
        self.arm_group.set_max_acceleration_scaling_factor(1)
        self.arm_group.set_planning_time(10.0)

        self.gripper_group.set_max_velocity_scaling_factor(1)
        self.gripper_group.set_max_acceleration_scaling_factor(1)
        self.gripper_group.set_planning_time(10.0)

        self.pose_goal = Pose()
        self.pose_goal.orientation = self.arm_group.get_current_pose().pose.orientation

        # Start at home position
        self.update_octomap()
        self.home_state = self.arm_group.get_current_state().joint_state
        self.home_state.name = list(self.home_state.name)[:6]
        self.home_state.position = [self.arm_group.get_named_target_values('home')['shoulder_pan_joint'],
                                self.arm_group.get_named_target_values('home')['shoulder_lift_joint'],
                                self.arm_group.get_named_target_values('home')['elbow_joint'],
                                self.arm_group.get_named_target_values('home')['wrist_1_joint'],
                                self.arm_group.get_named_target_values('home')['wrist_2_joint'],
                                self.arm_group.get_named_target_values('home')['wrist_3_joint']]
        plan = self.arm_group.plan(self.home_state)
        success = self.arm_group.execute(plan[1], wait=True)
        self.arm_group.stop()
        while not success:  # FALLBACK FOR SAFETY
            self.arm_group.stop()
            plan = self.arm_group.plan(self.home_state)
            success = self.arm_group.execute(plan[1], wait=True)
            self.arm_group.stop()
        rospy.sleep(1)
        rospy.loginfo("RobotControl class initialize successfully")

    def update_octomap(self):
        """
        Update octomap in moveit planning scene.
        """
        
        # First clearing octomap
        self.clear_octomap.call()
        # Loop through available depth cameras and obtain pointclouds for octomap
        for camera in self.camera_topics:
            req = FilterWorkspaceRequest()
            req.pointcloud_topic.data = camera + '/depth/color/points/'
            req.image_topic.data = camera + '/color/image_raw/'
            self.publish_octomap.call(req)
        return
    
    def to_home(self):
        """
        Этот метод необходим для того, чтобы спозиционировать
        робота в домашнее положение.
        """
        self.update_octomap()
        plan = self.arm_group.plan(self.home_state)
        success = self.arm_group.execute(plan[1], wait=True)
        self.arm_group.stop()
        while not success:  # FALLBACK FOR SAFETY
            self.arm_group.stop()
            plan = self.arm_group.plan(self.home_state)
            success = self.arm_group.execute(plan[1], wait=True)
            self.arm_group.stop()
        rospy.sleep(1)
        rospy.loginfo("Robot is home position")


    def rotate_flangue(self, angle):
        joint_goal = self.arm_group.get_current_joint_values()
        joint_goal[5] = angle * math.pi / 180
        self.arm_group.go(joint_goal, wait=True)
        self.arm_group.stop()
        rospy.sleep(1)
        rospy.loginfo("Flangue is already rotate")


    def to_grasp(self, x=0.5, y=0, z=0.5, angle=0):
        self.to_home()  # Перейти в начальную позицию
        self.rotate_flangue(angle)  # Повернуть фланец на заданный угол

        # Обновить Octomap
        self.update_octomap()

        # Переместиться над целевой позицией
        self.pose_goal.position.x = x
        self.pose_goal.position.y = y
        self.pose_goal.position.z = z + 0.1  # Начать на 10 см выше цели

        # Установить ориентацию
        self.pose_goal.orientation = self.arm_group.get_current_pose().pose.orientation
        self.arm_group.set_pose_target(self.pose_goal)

        # Добавить допуски и увеличить время планирования
        self.arm_group.set_goal_tolerance(0.01)
        self.arm_group.set_planning_time(20.0)

        # Планировать и выполнять движение
        plan = self.arm_group.plan()
        if not plan[0]:
            rospy.logerr("Не удалось спланировать движение к целевой позиции")
            return
        success = self.arm_group.execute(plan[1], wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

        if success:
            rospy.loginfo("Робот переместился над целевой позицией")
        else:
            rospy.logerr("Ошибка выполнения движения")
            return

        # Опуститься до целевой координаты z
        self.pose_goal.position.z = z
        self.arm_group.set_pose_target(self.pose_goal)
        plan = self.arm_group.plan()
        if plan[0]:
            success = self.arm_group.execute(plan[1], wait=True)
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
            if success:
                rospy.loginfo("Робот переместился в целевую позицию захвата")
            else:
                rospy.logerr("Ошибка выполнения движения в позицию захвата")
        else:
            rospy.logerr("Не удалось спланировать движение в позицию захвата")


    def close_gripper(self):
        """
        Этот метод необходим для закрытия схвата.
        """
        # self.update_octomap()
        close_gripper = [self.gripper_group.get_named_target_values('closed')['robotiq_85_left_knuckle_joint']]
        # self.gripper_group.go(close_gripper, wait=True)
        self.gripper_group.set_start_state_to_current_state()  # Установить текущее состояние как начальное
        self.gripper_group.set_max_velocity_scaling_factor(0.5)  # Ограничить скорость движения
        self.gripper_group.go(close_gripper, wait=True)
        self.gripper_group.stop()
        rospy.sleep(1)
        rospy.loginfo("Gripper is close")

    def open_gripper(self):
        """
        Этот метод необходим для открытия схвата.
        """
        # self.update_octomap()
        self.gripper_group.set_planner_id("RRTConnectkConfigDefault")  # Выбор планировщика
        open_gripper = [self.gripper_group.get_named_target_values('open')['robotiq_85_left_knuckle_joint']]
        self.gripper_group.go(open_gripper, wait=True)
        self.gripper_group.stop()
        rospy.sleep(1)
        rospy.loginfo("Gripper is open")

    def go_to(self, x=0.5, y=0, z=0.5):
        # Обновить Octomap
        self.update_octomap()

        # Переместиться над целевой позицией
        self.pose_goal.position.x = x
        self.pose_goal.position.y = y
        self.pose_goal.position.z = z

        # Установить ориентацию
        self.pose_goal.orientation = self.arm_group.get_current_pose().pose.orientation
        self.arm_group.set_pose_target(self.pose_goal)

        # Добавить допуски и увеличить время планирования
        self.arm_group.set_goal_tolerance(0.005)
        self.arm_group.set_planning_time(20.0)

        # Планировать и выполнять движение
        plan = self.arm_group.plan()
        if not plan[0]:
            rospy.logerr("Не удалось спланировать движение к целевой позиции")
            return
        success = self.arm_group.execute(plan[1], wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

        if success:
            rospy.loginfo("Робот переместился над целевой позицией")
        else:
            rospy.logerr("Ошибка выполнения движения")
            return