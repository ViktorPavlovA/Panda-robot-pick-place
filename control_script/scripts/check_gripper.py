#!/usr/bin/env python3

import franka_gripper.msg
import rospy
import sys
import actionlib

def grasp_client():
    #Создадим акшен клиент 
    client = actionlib.SimpleActionClient('/franka_gripper/grasp', \
        franka_gripper.msg.GraspAction)
    client.wait_for_server()
    
    gripper_move = franka_gripper.msg.GraspGoal()
    gripper_move.width = 0.025
    gripper_move.epsilon.inner = 0.008
    gripper_move.epsilon.outer = 0.008
    gripper_move.speed = 0.1
    gripper_move.force = 5
    client.send_goal(gripper_move)
    client.wait_for_result()
    return client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('grasp_client_py')
        result = grasp_client()
        print("Вроде все хорошо: ",result.success)
        print("Косяк, нужно проверить параметры ", result.error)
    except rospy.ROSInterruptException:
        print("Прерывание", file=sys.stderr)
