#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Lista de waypoints: cada elemento é uma tupla ((x, y, z), (qx, qy, qz, qw))
# onde (x, y, z) define a posição e (qx, qy, qz, qw) define a orientação (quaternion).
waypoints = [
    ((2.1, 2.2, 0.0), (0.0, 0.0, 0.0, 0.99844723805)), 
    ((4.0, 4.3, 0.0), (0.0, 0.0, 0.0, 0.98480775301)), 
    ((2.0, 4.0, 0.0), (0.0, 0.0, 0.0, 0.70710678118)), 
    ((4.0, 2.0, 0.0), (0.0, 0.0, 0.0, 0.64278760968)), 
    ((3.5, 2.5, 0.0), (0.0, 0.0, 0.0, 0.76604444311))
]

def goal_pose(pose):
    """
    Converte a tupla 'pose' em um objeto MoveBaseGoal,
    definindo posição e orientação no frame 'map'.
    """
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'

    # Pose[0] é a posição (x, y, z)
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]

    # Pose[1] é a orientação (qx, qy, qz, qw)
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]

    return goal_pose

if __name__ == '__main__':
    rospy.init_node('patrol')

    # Cria um cliente de ação para o servidor 'move_base'
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Envia cada waypoint como um objetivo para o move_base e aguarda o resultado
    for pose in waypoints:
        goal = goal_pose(pose)
        client.send_goal(goal)
        client.wait_for_result()
