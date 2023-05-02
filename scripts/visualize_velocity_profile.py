#!/usr/bin/python

# This small script subscribes to the FeedbackMsg message of teb_local_planner
# and plots the current velocity.
# publish_feedback must be turned on such that the planner publishes this information.
# Author: christoph.roesmann@tu-dortmund.de

import rospy, math
from teb_local_planner.msg import FeedbackMsg, TrajectoryMsg, TrajectoryPointMsg
from geometry_msgs.msg import PolygonStamped, Point32
import numpy as np
import matplotlib.pyplot as plotter
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock

def feedback_callback(data):
  # global trajectory
  global vel
  global ang
  # if not data.trajectories: # empty
  #   trajectory = []
  #   return
  # trajectory = data.trajectories[data.selected_trajectory_idx].trajectory

  vel = data.linear.x
  ang = data.angular.z

def clock_callback(data):
  
  global time
  time = data.clock.secs

def plot_velocity_profile(fig, ax_v, ax_omega, t, v, omega):
  ax_v.cla()
  ax_v.grid()
  ax_v.set_ylabel('Trans. velocity [m/s]')
  ax_v.plot(t, v, '-bx')
  ax_omega.cla()
  ax_omega.grid()
  ax_omega.set_ylabel('Rot. velocity [rad/s]')
  ax_omega.set_xlabel('Time [s]')
  ax_omega.plot(t, omega, '-bx')
  fig.canvas.draw()

  
  
def velocity_plotter():
  global trajectory
  global vel
  global ang
  global time
  global t,v,omega
  rospy.init_node("visualize_velocity_profile", anonymous=True)
  
  topic_name = "/robot_0/cmd_vel"
  r = rospy.Rate(2) # define rate here
  # two subplots sharing the same t axis
  fig, (ax_v, ax_omega) = plotter.subplots(2, sharex=True)
  plotter.ion()
  plotter.show()
  while not rospy.is_shutdown():
    rospy.Subscriber(topic_name, Twist, feedback_callback, queue_size = 1) # define feedback topic here!
    rospy.Subscriber("/clock", Clock, clock_callback, queue_size = 1)
    rospy.loginfo("Visualizing velocity profile published on '%s'.",topic_name) 
    rospy.loginfo("Make sure to enable rosparam 'publish_feedback' in the teb_local_planner.")

    
  

  
  
    
    
    
    
    t.append(float(time))
    v.append(vel)
    omega.append(ang)
          
    plot_velocity_profile(fig, ax_v, ax_omega, np.asarray(t), np.asarray(v), np.asarray(omega))
        
    r.sleep()

if __name__ == '__main__': 
  try:

    t = []
    v = []
    omega = []
    trajectory = []
    vel = 0.0
    ang = 0.0
    time = 0
    velocity_plotter()
  except rospy.ROSInterruptException:
    pass

