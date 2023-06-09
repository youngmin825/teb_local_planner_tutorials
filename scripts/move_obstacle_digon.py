#!/usr/bin/python

import rospy, math, random
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32

# Parameters
vel_max = 1.0
vel_min = 0.1

# Define parameters, if parameter server provides no definitions
# Lower bound of the obstacle movement in y-direction
if rospy.has_param("pos_lb"):
  posy_lb = rospy.get_param("pos_lb")
else:
  posy_lb = -0.53

# Upper bound of the obstacle movement in y-direction
if rospy.has_param("pos_ub"):
  posy_ub = rospy.get_param("pos_ub")
else:
  # pos_ub = 5.5
  posy_ub = 2.98
# Define global Twist message which is modified in the callback_base_pose_ground_truth and published in the main loop
Twist_msg = Twist()

posx_lb = 1.25

posx_ub = 4.57



def flag_callback(msg):
   flag = msg.data
   move_object()

# This function manages turnarounds and new (possibly random) velocities of the object
def callback_base_pose_ground_truth(base_pose_ground_truth):
  # Define random velocity if no specific velocity is defined in the parameter server

  
  if rospy.has_param("vel_y"):
  	vel_y = rospy.get_param("vel_y")
  else:
  	vel_y = random.uniform(vel_min, vel_max)

  # Turn in y-direction
  if base_pose_ground_truth.pose.pose.position.y >= posy_ub and base_pose_ground_truth.pose.pose.position.x >= posx_ub:
    Twist_msg.linear.y = -vel_y
    Twist_msg.linear.x = -vel_y

  if base_pose_ground_truth.pose.pose.position.y <= posy_lb and base_pose_ground_truth.pose.pose.position.x <= posx_lb:
    Twist_msg.linear.y = vel_y
    Twist_msg.linear.x = vel_y

# This function initializes the mover node and publishes continously a Twist message
def move_object():
  r = rospy.Rate(10)
  pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
  sub = rospy.Subscriber('base_pose_ground_truth', Odometry, callback_base_pose_ground_truth)
  start_t = rospy.get_time()

  # initialize movement with random direction
  if rospy.has_param("vel_y"):
  	vel_y = rospy.get_param("vel_y")
  else:
  	vel_y = random.uniform(vel_min, vel_max)
  Twist_msg.linear.y = float(random.choice(['-1', '1'])) * vel_y

  # publish movement command continuously
  while not rospy.is_shutdown():
    pub.publish(Twist_msg)
    r.sleep()


if __name__ == '__main__': 
  try:
    rospy.init_node('move_obstacle_digon')
    rospy.Subscriber("/flag", Int32, flag_callback, queue_size=10)
    

    rospy.spin()
  except rospy.ROSInterruptException:
    pass