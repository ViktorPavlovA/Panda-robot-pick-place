#!/usr/bin/env python3


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time

from franka_gripper.msg import GraspActionGoal


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def position_checker(value):

    while not rospy.is_shutdown():
            moveit_commander.roscpp_initialize(sys.argv)
            rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
            robot = moveit_commander.RobotCommander()
            scene = moveit_commander.PlanningSceneInterface()
            group_name = "panda_arm" 
            move_group = moveit_commander.MoveGroupCommander(group_name)
            display_trajectory_publisher = rospy.Publisher(
                "/move_group/display_planned_path",
             moveit_msgs.msg.DisplayTrajectory,
                queue_size=20,
                 ) 
            planning_frame = move_group.get_planning_frame()
            time.sleep(int(60/value))
            rospy.logwarn("Положение звеньев робота [rad]")
            print(robot.get_current_state())
 


if __name__ == "__main__":
    
    print("Введите количество повторений сообщения о положении звеньев в минуту, например 2.")

    value = input()

    position_checker(int(value))
