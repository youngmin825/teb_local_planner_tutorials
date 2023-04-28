#! /usr/bin/python
# -*- coding: utf-8 -*-

import rospy, actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64MultiArray
import actionlib



waypoints = [[13.7, 3.0, 0.0], [0.0, 0.0, 0.0, 0.1]]

if __name__ == '__main__':
    rospy.init_node('path_planing')
    
    client = actionlib.SimpleActionClient('/robot_0/move_base', MoveBaseAction) 
    client.wait_for_server()

    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'

    # 목표 지점 위치로 이동
    goal_pose.target_pose.pose.position.x = waypoints[0][0]
    goal_pose.target_pose.pose.position.y = waypoints[0][1]
    goal_pose.target_pose.pose.position.z = waypoints[0][2]

    # 목표 지점 방향으로 이동
    goal_pose.target_pose.pose.orientation.x = waypoints[1][0]
    goal_pose.target_pose.pose.orientation.y = waypoints[1][1]
    goal_pose.target_pose.pose.orientation.z = waypoints[1][2]
    goal_pose.target_pose.pose.orientation.w = waypoints[1][3]

    # 목표 지점으로 이동하라고 명령을 보낸다.
    client.send_goal(goal_pose)
    client.wait_for_result()
    


