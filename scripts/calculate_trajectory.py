#!/usr/bin/env python

import rospy, math
from geometry_msgs.msg import  PolygonStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

first_point = True
total_length = 0
pre_x = 0
pre_y = 0

def make_path(x, y):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = 0.0 
    pose.pose.orientation.y = 0.0 
    pose.pose.orientation.z = 0.0 
    pose.pose.orientation.w = 0.0 
    pose.header.seq = path.header.seq + 1
    path.header.frame_id = "map"
    path.header.stamp = rospy.Time.now()
    pose.header.stamp = path.header.stamp
    path.poses.append(pose)

    pub_path.publish(path)
  

def calc_callback(data):
  global first_point
  global total_length
  global pre_x
  global pre_y

  if first_point:
    pre_x = data.polygon.points[0].x
    pre_y = data.polygon.points[0].y
    first_point = False

  current_pos = data.polygon.points[0]

  dx = current_pos.x - pre_x
  dy = current_pos.y - pre_y

  length = math.sqrt(dx*dx + dy*dy)
  total_length += length

  pre_x = current_pos.x
  pre_y = current_pos.y

  make_path(pre_x, pre_y)

  print(total_length)
  

if __name__ == '__main__': 
  try:
    rospy.init_node('calculate_trajectory')

    path = Path()

    pub_path = rospy.Publisher('/trajectory_path', Path, queue_size=1)
    rospy.Subscriber("/robot_0/move_base/global_costmap/footprint", PolygonStamped, calc_callback, queue_size=1)
    
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass