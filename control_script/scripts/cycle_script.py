#!/usr/bin/env python3


import sys
import time
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import franka_gripper.msg 

# Для action clients
import actionlib

try:
    from math import pi, tau, dist, fabs, cos
except:  
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list



def all_close(goal, actual, tolerance):
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class Cycle_action(object):

    def __init__(self):
        super(Cycle_action, self).__init__()

        ##Инициализация основных нод moveit_commander и rospy
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("Controll_panda_bot", anonymous=True)

        ##Инициализация что робот представляет собой кинематическую модель
        robot = moveit_commander.RobotCommander()

        ##Наверное нужно для обновления мира
        scene = moveit_commander.PlanningSceneInterface()

        ## Группа должна точно быть такая же как в moveit
        group_name = "panda_arm" 
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Группа 2 для захвата
        group_name2 = "panda_hand" 
        move_group2 = moveit_commander.MoveGroupCommander(group_name2)

        ##  Создание издателя который публикует траекторию         
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            franka_gripper.msg.GraspAction
        )
        planning_frame = move_group.get_planning_frame()
        rospy.logwarn("Reference frame для данного робота")
        print(planning_frame)

        eef_link = move_group.get_end_effector_link()
        rospy.logwarn("Название звена эндефектора")
        print(eef_link)
        print("="*60 )

        group_names = robot.get_group_names()
        rospy.logwarn("Звенья доступные для планирования")
        print(robot.get_group_names())
        print("="*60 )
        rospy.logwarn("Положение звеньев робота")
        print(robot.get_current_state())
        print("="*60 )
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.move_group2 = move_group2
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
    
    def go2pose1(self):
        ref_pose = [0.000000001, 0.24286789895779126, 0.0003225356836855653,\
                 -2.9554881936230384, 0.0009427103928310032, 3.200537108661095,\
                  0.784816909216266, 0.021412689304403924, 0.018512689304403924]
        move_group = self.move_group
        rospy.logwarn("Переход в положение 1")
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = ref_pose[0]
        joint_goal[1] = ref_pose[1]
        joint_goal[2] = ref_pose[2]
        joint_goal[3] =  ref_pose[3]
        joint_goal[4] = ref_pose[4]
        joint_goal[5] = ref_pose[5]
        joint_goal[6] = ref_pose[6]
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    def go2pose2(self):
        ref_pose = [1.566284675315564, 0.24286789895779126, 0.0003225356836855653,\
                 -2.9554881936230384, 0.0009427103928310032, 3.200537108661095,\
                  0.784816909216266, 0.021412689304403924, 0.018512689304403924]

        move_group = self.move_group
        rospy.logwarn("Переход в положение 2")
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = ref_pose[0]
        joint_goal[1] = ref_pose[1]
        joint_goal[2] = ref_pose[2]
        joint_goal[3] =  ref_pose[3]
        joint_goal[4] = ref_pose[4]
        joint_goal[5] = ref_pose[5]
        joint_goal[6] = ref_pose[6]
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    def go2pose1_1(self):
        ref_pose = [0.000000001, 0.24286789895779126, 0.0003225356836855653,\
                 -2.9554881936230384, 0.0009427103928310032, 3.200537108661095,\
                  0.784816909216266, 0.021412689304403924, 0.018512689304403924]

        move_group = self.move_group
        rospy.logwarn("Переход в положение 1")
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = ref_pose[0]
        joint_goal[1] = ref_pose[1]
        joint_goal[2] = ref_pose[2]
        joint_goal[3] =  ref_pose[3]
        joint_goal[4] = ref_pose[4]
        joint_goal[5] = ref_pose[5]
        joint_goal[6] = ref_pose[6]
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    def go2pose2_1(self):
        ref_pose = [0.000000001, 0.24286789895779126, 0.0003225356836855653,\
                 -2.9554881936230384, 0.0009427103928310032, 3.200537108661095,\
                  0.784816909216266, 0.021412689304403924, 0.018512689304403924]

        move_group = self.move_group
        rospy.logwarn("Переход в положение 1")
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = ref_pose[0]
        joint_goal[1] = ref_pose[1]
        joint_goal[2] = ref_pose[2]
        joint_goal[3] =  ref_pose[3]
        joint_goal[4] = ref_pose[4]
        joint_goal[5] = ref_pose[5]
        joint_goal[6] = ref_pose[6]
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def middle_pose1(self):
        ref_pose = [0.000001, -0.2783076820296584, -8.797852469744072e-05,\
             -3.0157774299770352, 4.810139364508359e-05, 2.7396815501972167,\
                 0.7853895189565767, 0.0010041369838028201, 0.0010041369838028201]
        move_group = self.move_group
        rospy.logwarn("Переход в положение 1_2")
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = ref_pose[0]
        joint_goal[1] = ref_pose[1]
        joint_goal[2] = ref_pose[2]
        joint_goal[3] =  ref_pose[3]
        joint_goal[4] = ref_pose[4]
        joint_goal[5] = ref_pose[5]
        joint_goal[6] = ref_pose[6]
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def middle_pose2(self):
        ref_pose = ref_pose = [1.566284675315564, -0.2783076820296584, -8.797852469744072e-05,\
             -3.0157774299770352, 4.810139364508359e-05, 2.7396815501972167,\
                 0.7853895189565767, 0.0010041369838028201, 0.0010041369838028201]
        move_group = self.move_group
        rospy.logwarn("Переход в положение 2_2")
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = ref_pose[0]
        joint_goal[1] = ref_pose[1]
        joint_goal[2] = ref_pose[2]
        joint_goal[3] =  ref_pose[3]
        joint_goal[4] = ref_pose[4]
        joint_goal[5] = ref_pose[5]
        joint_goal[6] = ref_pose[6]
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def open_gripper(self):
        rospy.logwarn("Открытие захвата")
        move_group2 = self.move_group2
        joint_goal = move_group2.get_current_joint_values()
        joint_goal[0] = 0.03
        move_group2.go(joint_goal, wait=True)
        move_group2.stop()
        current_joints = move_group2.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def close_gripper(self):
        rospy.logwarn("Закрытие захвата")
        #вариант кода 1

        # move_group2 = self.move_group2
        # joint_goal = move_group2.get_current_joint_values()
        # # joint_goal[0] = 0.01
        # joint_goal[0] = 0.003
        # move_group2.go(joint_goal, wait=True)
        # move_group2.stop()
        # current_joints = move_group2.get_current_joint_values()
        # return all_close(joint_goal, current_joints, 0.01)

        #вариант кода 2

        client = actionlib.SimpleActionClient('/franka_gripper/grasp', \
        franka_gripper.msg.GraspAction)
        client.wait_for_server()
        gripper_move = franka_gripper.msg.GraspGoal()
        gripper_move.width = 0.02
        gripper_move.epsilon.inner = 0.008
        gripper_move.epsilon.outer = 0.008
        gripper_move.speed = 0.1
        gripper_move.force = 5
        client.send_goal(gripper_move)
        client.wait_for_result()
        # result_action = client.get_result() 

    def display_trajectory(self, plan):
        rospy.logwarn("Отображение траектории")
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        move_group = self.move_group
        rospy.logwarn("Выполнение траектории")
        move_group.execute(plan, wait=True)

def main():
    time_wait =0
    try:
        
        robot_actions  = Cycle_action()
        input(
            "Для старта симуляции нажмите Enter"
        )
        while not rospy.is_shutdown():
            robot_actions.open_gripper()
            time.sleep(time_wait) 
            robot_actions.go2pose1()
            time.sleep(time_wait) 
            robot_actions.close_gripper()
            time.sleep(time_wait) 
            robot_actions.middle_pose1()
            time.sleep(time_wait)
            robot_actions.middle_pose2()
            time.sleep(time_wait)
            robot_actions.go2pose2()
            time.sleep(time_wait)
            robot_actions.open_gripper()
            time.sleep(time_wait)
            robot_actions.close_gripper()
            time.sleep(time_wait)
            robot_actions.middle_pose2()
            time.sleep(time_wait)
            robot_actions.middle_pose1()
            time.sleep(time_wait) 
            robot_actions.go2pose1()
            time.sleep(time_wait)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return



if __name__ == "__main__":
    main()
