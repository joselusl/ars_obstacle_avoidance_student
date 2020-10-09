#!/usr/bin/env python

import numpy as np
from numpy import *

import os




# ROS

import rospy

import rospkg

import std_msgs.msg
from std_msgs.msg import Header

import geometry_msgs.msg
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped

import nav_msgs.msg
from nav_msgs.msg import Path

import visualization_msgs.msg
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker


import tf_conversions

import tf2_ros




#
from ars_obstacle_avoidance import *


#
import ars_lib_helpers





class ArsObstacleAvoidanceRos:

  #######


  # Robot frame
  robot_frame = 'robot_base_link'

  # World frame
  world_frame = 'world'



  # Ctr command loop freq 
  # time step
  ctr_cmd_loop_freq = 10.0
  # Timer
  ctr_cmd_loop_timer = None



  # Robot traj subscriber
  robot_traj_sub = None

  # Robot pose subscriber
  robot_pose_sub = None
  # Robot velocity subscriber
  robot_vel_world_sub = None


  #
  obstacles_detected_sub = None


  # Collision-free traj pub
  coll_free_robot_traj_pub = None


  # Motion controller
  obstacle_avoidance = ArsObstacleAvoidance()
  


  #########

  def __init__(self):


    # Motion controller
    self.obstacle_avoidance = ArsObstacleAvoidance()


    # end
    return


  def init(self, node_name='ars_obstacle_avoidance_node'):
    #

    # Init ROS
    rospy.init_node(node_name, anonymous=True)

    
    # Package path
    pkg_path = rospkg.RosPack().get_path('ars_obstacle_avoidance')
    

    #### READING PARAMETERS ###
    
    # TODO

    ###

    
    # End
    return


  def open(self):

    # Subscribers

    #
    self.robot_traj_sub = rospy.Subscriber('robot_trajectory_ref', Path, self.robotTrajectoryCallback)

    # 
    self.robot_pose_sub = rospy.Subscriber('robot_pose', PoseStamped, self.robotPoseCallback)
    #
    self.robot_vel_world_sub = rospy.Subscriber('robot_velocity_world', TwistStamped, self.robotVelWorldCallback)

    #
    self.obstacles_detected_sub = rospy.Subscriber('obstacles_detected', MarkerArray, self.obstaclesDetectedCallback)
    


    # Publishers

    # Collision-free traj pub
    self.coll_free_robot_traj_pub = rospy.Publisher('robot_trajectory_coll_free_ref', Path, queue_size=1)




    # Timers
    #
    self.ctr_cmd_loop_timer = rospy.Timer(rospy.Duration(1.0/self.ctr_cmd_loop_freq), self.ctrCommandLoopTimerCallback)


    # End
    return


  def run(self):

    rospy.spin()

    return


  def robotTrajectoryCallback(self, robot_trajectory_msg):

    # 
    robot_trajectory = []

    #
    for robot_trajectory_pose_i_msg in robot_trajectory_msg.poses:

      robot_trajectory_pose_i = ars_lib_helpers.PoseSimp()

      robot_trajectory_pose_i.position[0] = robot_trajectory_pose_i_msg.pose.position.x
      robot_trajectory_pose_i.position[1] = robot_trajectory_pose_i_msg.pose.position.y
      robot_trajectory_pose_i.position[2] = robot_trajectory_pose_i_msg.pose.position.z

      quat_i = ars_lib_helpers.Quaternion.zerosQuat()
      quat_i[0] = robot_trajectory_pose_i_msg.pose.orientation.w
      quat_i[1] = robot_trajectory_pose_i_msg.pose.orientation.x
      quat_i[2] = robot_trajectory_pose_i_msg.pose.orientation.y
      quat_i[3] = robot_trajectory_pose_i_msg.pose.orientation.z

      quat_i = ars_lib_helpers.Quaternion.normalize(quat_i)

      robot_trajectory_pose_i.attitude_quat_simp = ars_lib_helpers.Quaternion.getSimplifiedQuatRobotAtti(quat_i)

      robot_trajectory.append(robot_trajectory_pose_i)

    #
    self.obstacle_avoidance.setRobotTrajectoryRef(robot_trajectory)


    # End
    return


  def robotPoseCallback(self, robot_pose_msg):

    # Position
    robot_posi = np.zeros((3,), dtype=float)
    robot_posi[0] = robot_pose_msg.pose.position.x
    robot_posi[1] = robot_pose_msg.pose.position.y
    robot_posi[2] = robot_pose_msg.pose.position.z

    # Attitude quat simp
    robot_atti_quat = ars_lib_helpers.Quaternion.zerosQuat()
    robot_atti_quat[0] = robot_pose_msg.pose.orientation.w
    robot_atti_quat[1] = robot_pose_msg.pose.orientation.x
    robot_atti_quat[2] = robot_pose_msg.pose.orientation.y
    robot_atti_quat[3] = robot_pose_msg.pose.orientation.z

    robot_atti_quat_simp = ars_lib_helpers.Quaternion.getSimplifiedQuatRobotAtti(robot_atti_quat)

    #
    self.obstacle_avoidance.setRobotPose(robot_posi, robot_atti_quat_simp)

    #
    return


  def robotVelWorldCallback(self, robot_vel_msg):

    # Linear
    lin_vel_world = np.zeros((3,), dtype=float)
    lin_vel_world[0] = robot_vel_msg.twist.linear.x
    lin_vel_world[1] = robot_vel_msg.twist.linear.y
    lin_vel_world[2] = robot_vel_msg.twist.linear.z

    # Angular
    ang_vel_world = np.zeros((1,), dtype=float)
    ang_vel_world[0] = robot_vel_msg.twist.angular.z

    #
    self.obstacle_avoidance.setRobotVelWorld(lin_vel_world, ang_vel_world)

    #
    return


  def obstaclesDetectedCallback(self, obstacles_detected_msg):

    # Save
    self.obstacle_avoidance.obstacles_detected_msg = obstacles_detected_msg

    #
    return

  def publishCollFreeTraj(self, time_stamp_current):

    if(self.obstacle_avoidance.flag_set_robot_traj_coll_free_ref == True):

      coll_free_robot_traj_msg = Path()

      coll_free_robot_traj_msg.header.stamp = time_stamp_current
      coll_free_robot_traj_msg.header.frame_id = self.world_frame

      coll_free_robot_traj_msg.poses = []

      for pose_i in self.obstacle_avoidance.robot_traj_coll_free_ref:

        pose_i_msg = PoseStamped()

        pose_i_msg.header.stamp = rospy.Time()
        pose_i_msg.header.frame_id = self.world_frame

        pose_i_msg.pose.position.x = pose_i.position[0]
        pose_i_msg.pose.position.y = pose_i.position[1]
        pose_i_msg.pose.position.z = pose_i.position[2]

        pose_i_msg.pose.orientation.w = pose_i.attitude_quat_simp[0]
        pose_i_msg.pose.orientation.x = 0.0
        pose_i_msg.pose.orientation.y = 0.0
        pose_i_msg.pose.orientation.z = pose_i.attitude_quat_simp[1]

        coll_free_robot_traj_msg.poses.append(pose_i_msg)

      #
      self.coll_free_robot_traj_pub.publish(coll_free_robot_traj_msg)

    #
    return

  def collisionFreePathCheck(self):

    # Get time
    time_stamp_current = rospy.Time.now()

    #
    self.obstacle_avoidance.pathPlannerLoop(time_stamp_current)

    #
    if(self.obstacle_avoidance.flag_new_robot_traj_coll_free_ref):

      #
      self.obstacle_avoidance.flag_new_robot_traj_coll_free_ref = False

      # Publish
      self.publishCollFreeTraj(time_stamp_current)

    return


  def ctrCommandLoopTimerCallback(self, timer_msg):

    
    #
    self.collisionFreePathCheck()

    
    # End
    return

  