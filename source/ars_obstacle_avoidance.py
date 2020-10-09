#!/usr/bin/env python

import numpy as np
from numpy import *

import os


# ROS

import rospy

import tf_conversions as tf


import nav_msgs.msg
from nav_msgs.msg import Path

import visualization_msgs.msg
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker



#
import ars_lib_helpers






class ArsObstacleAvoidance:

  #######

  # Robot size radius
  robot_size_radius = 0.0

  #
  safety_distance_nav = 0.0
  safety_distance_plan = 0.0

  # Tol
  tol_posi = 0.1
  tol_angle = 0.1

  # Flag hover when collision
  flag_hover_when_collision = False



  # Output: Collision-free traj
  flag_set_robot_traj_coll_free_ref = False
  flag_new_robot_traj_coll_free_ref = False
  robot_traj_coll_free_ref = None
  traj_coll_free_ref_keypoint = 0


  # Input: Pose & Velocity Feedback
  #
  flag_set_robot_pose = False
  robot_posi = None
  robot_atti_quat_simp = None
  #
  flag_set_robot_vel_world = False
  robot_velo_lin_world = None
  robot_velo_ang_world = None


  # Input: Trajectory reference
  flag_set_robot_traj_ref = False
  flag_new_robot_traj_ref = False
  robot_traj_ref = None
  traj_ref_keypoint = 0


  # Input: Obstacles detected
  obstacles_detected_msg = None

  




  #########

  def __init__(self):


    # Robot size radius
    self.robot_size_radius = 0.3

    #
    self.safety_distance_nav = 0.25
    self.safety_distance_plan = 0.75


    # Tol
    self.tol_posi = 0.1
    self.tol_angle = 0.1

    #
    self.flag_hover_when_collision = False



    # Input: Trajectory ref
    #
    self.flag_set_robot_traj_ref = False
    self.flag_new_robot_traj_ref = False
    self.robot_traj_ref = []
    self.traj_ref_keypoint = 0

    # Input: Feedback
    #
    self.flag_set_robot_pose = False
    self.robot_posi = np.zeros((3,), dtype=float)
    self.robot_atti_quat_simp = ars_lib_helpers.Quaternion.zerosQuatSimp()
    #
    self.flag_set_robot_vel_world = False
    self.robot_velo_lin_world = np.zeros((3,), dtype=float)
    self.robot_velo_ang_world = np.zeros((1,), dtype=float)


    # Input: References
    #
    self.flag_set_robot_pose_ref = False
    self.robot_posi_ref = np.zeros((3,), dtype=float)
    self.robot_atti_quat_simp_ref = ars_lib_helpers.Quaternion.zerosQuatSimp()
    #
    self.flag_set_robot_velo_world_ref = False
    self.robot_velo_lin_world_ref = np.zeros((3,), dtype=float)
    self.robot_velo_ang_world_ref = np.zeros((1,), dtype=float)
    #
    self.flag_set_robot_velo_cmd_ref = False
    self.robot_velo_lin_cmd_ref = np.zeros((3,), dtype=float)
    self.robot_velo_ang_cmd_ref = np.zeros((1,), dtype=float)


    # Input: Obstacles detected
    self.obstacles_detected_msg = MarkerArray()


    # Output: Collision-free traj
    self.flag_set_robot_traj_coll_free_ref = False
    self.flag_new_robot_traj_coll_free_ref = False
    self.robot_traj_coll_free_ref = []
    self.traj_coll_free_ref_keypoint = 0


    # End
    return


  def setRobotPose(self, robot_posi, robot_atti_quat_simp):
    
    self.flag_set_robot_pose = True

    self.robot_posi = robot_posi
    self.robot_atti_quat_simp = robot_atti_quat_simp

    return

  def setRobotVelWorld(self, lin_vel_world, ang_vel_world):

    self.flag_set_robot_vel_world = True

    self.robot_velo_lin_world = lin_vel_world
    self.robot_velo_ang_world = ang_vel_world

    return

  def setRobotTrajectoryRef(self, robot_traj_ref):

    self.flag_set_robot_traj_ref = True
    self.flag_new_robot_traj_ref = True

    self.traj_ref_keypoint = 0

    self.robot_traj_ref = robot_traj_ref

    return


  def setCollFreePathRef(self, path):

    self.flag_set_robot_traj_coll_free_ref = True
    self.flag_new_robot_traj_coll_free_ref = True
    self.robot_traj_coll_free_ref = path
    self.traj_coll_free_ref_keypoint = 0


    return



  def trackPathRef(self):


    position_over_path = np.zeros((3,), dtype=float)


    if(self.flag_set_robot_traj_ref):

      if(self.flag_set_robot_pose):

        #
        while(self.traj_ref_keypoint < len(self.robot_traj_ref)-1):
          
          # Difference to next waypoint

          delta_posi, delta_atti_quat_simp = ars_lib_helpers.PoseAlgebra.computePoseSimpDifference(self.robot_posi, self.robot_atti_quat_simp, 
            self.robot_traj_ref[self.traj_ref_keypoint].position, self.robot_traj_ref[self.traj_ref_keypoint].attitude_quat_simp)


          norm_pos = np.linalg.norm(delta_posi)

          error_att_z = ars_lib_helpers.PoseAlgebra.computeScalarDiffFromDiffQuatSimp(delta_atti_quat_simp)

      
          if( norm_pos<self.tol_posi and error_att_z<self.tol_angle ):
            if( self.traj_ref_keypoint < len(self.robot_traj_ref)-1 ):
              self.traj_ref_keypoint+=1
          else:
            break


    return


  def trackPathCollFreeRef(self):

    position_over_path = np.zeros((3,), dtype=float)


    if(self.flag_set_robot_traj_coll_free_ref):

      if(self.flag_set_robot_pose):

        #
        while(self.traj_coll_free_ref_keypoint < len(self.robot_traj_coll_free_ref)-1):
          
          # Difference to next waypoint

          delta_posi, delta_atti_quat_simp = ars_lib_helpers.PoseAlgebra.computePoseSimpDifference(self.robot_posi, self.robot_atti_quat_simp, 
            self.robot_traj_coll_free_ref[self.traj_coll_free_ref_keypoint].position, self.robot_traj_coll_free_ref[self.traj_coll_free_ref_keypoint].attitude_quat_simp)


          norm_pos = np.linalg.norm(delta_posi)

          error_att_z = ars_lib_helpers.PoseAlgebra.computeScalarDiffFromDiffQuatSimp(delta_atti_quat_simp)

      
          if( norm_pos<self.tol_posi and error_att_z<self.tol_angle ):
            if( self.traj_coll_free_ref_keypoint < len(self.robot_traj_coll_free_ref)-1 ):
              self.traj_coll_free_ref_keypoint+=1
          else:
            break


    return


  def isPointCollisionFree(self, waypoint, dist_safe_max_to_obst = 0.0):

    #
    flag_point_collision_free = True

    #
    for obst_i_msg in self.obstacles_detected_msg.markers:

      if(obst_i_msg.action == 0):

        if(obst_i_msg.type == 3):

          # Obstacle i
          obst_i_posi = np.zeros((3,), dtype=float)
          obst_i_posi[0] = obst_i_msg.pose.position.x
          obst_i_posi[1] = obst_i_msg.pose.position.y
          obst_i_posi[2] = obst_i_msg.pose.position.z

          obst_i_rad = obst_i_msg.scale.x/2.0

          #
          distance_point_obstacle_i = ars_lib_helpers.distancePointCircle(waypoint.position[0:2], obst_i_posi[0:2], obst_i_rad)

          #
          if(distance_point_obstacle_i < dist_safe_max_to_obst):

            flag_path_segment_collision_free = False

            break


    return flag_point_collision_free


  def isPathSegmentCollisionFreeObstacle(self, waypoint_1, waypoint_2, obst_i_posi, obst_i_rad, dist_safe_max_to_obst = 0.0):

    # Distance to detected objects

    flag_path_segment_collision_free = True


    #
    distance_path_obstacle_i = ars_lib_helpers.distanceSegmentCircle(waypoint_1.position[0:2], waypoint_2.position[0:2], obst_i_posi[0:2], obst_i_rad)

    #
    if(distance_path_obstacle_i < dist_safe_max_to_obst):

      flag_path_segment_collision_free = False

      
    return flag_path_segment_collision_free


  def isPathSegmentCollisionFree(self, waypoint_1, waypoint_2, dist_safe_max_to_obst = 0.0):

    # Distance to detected objects

    flag_path_segment_collision_free = True


    # Check

    for obst_i_msg in self.obstacles_detected_msg.markers:

      if(obst_i_msg.action == 0):

        if(obst_i_msg.type == 3):

          # Obstacle i
          obst_i_posi = np.zeros((3,), dtype=float)
          obst_i_posi[0] = obst_i_msg.pose.position.x
          obst_i_posi[1] = obst_i_msg.pose.position.y
          obst_i_posi[2] = obst_i_msg.pose.position.z

          obst_i_rad = obst_i_msg.scale.x/2.0

          #
          distance_path_obstacle_i = ars_lib_helpers.distanceSegmentCircle(waypoint_1.position[0:2], waypoint_2.position[0:2], obst_i_posi[0:2], obst_i_rad)

          #
          if(distance_path_obstacle_i < dist_safe_max_to_obst):

            flag_path_segment_collision_free = False

            break

    #
    return flag_path_segment_collision_free


  def isPathCollisionFree(self, path, dist_safe_max_to_obst = 0.0):

    # Distance to detected objects

    flag_path_collision_free = True
    segment_waypoint_init_collision = -1

    # Check if planned path is collision free
    for waypoint_idx in range(len(path)-1):

      waypoint_1 = path[waypoint_idx]
      waypoint_2 = path[waypoint_idx+1]

      flag_path_segment_collision_free = self.isPathSegmentCollisionFree(waypoint_1, waypoint_2, dist_safe_max_to_obst)

      if(flag_path_segment_collision_free == False):

        flag_path_collision_free = False
        segment_waypoint_init_collision = waypoint_idx

        break

    return flag_path_collision_free, segment_waypoint_init_collision



  def isRobotAndPathCollFree(self, path_in, dist_safe_max_to_obst = 0.0):

    # Path
    path = []

    # Path - init robot
    robot_pose = ars_lib_helpers.PoseSimp()

    robot_pose.position = self.robot_posi

    robot_pose.attitude_quat_simp = self.robot_atti_quat_simp

    path.append(robot_pose)

    # Path rest
    path.extend(path_in)


    # Check collision
    flag_path_collision_free, segment_waypoint_init_collision = self.isPathCollisionFree(path, dist_safe_max_to_obst)


    # End
    return flag_path_collision_free, segment_waypoint_init_collision-1



  def isRobotAndPathPlannedCollFree(self, dist_safe_max_to_obst = 0.0):

    #
    flag_path_collision_free, segment_waypoint_init_collision = self.isRobotAndPathCollFree(self.robot_traj_coll_free_ref[self.traj_coll_free_ref_keypoint:], dist_safe_max_to_obst)

    # End
    return flag_path_collision_free, segment_waypoint_init_collision



  def isRobotAndPathRefCollFree(self, dist_safe_max_to_obst = 0.0):

    #
    flag_path_collision_free, segment_waypoint_init_collision = self.isRobotAndPathCollFree(self.robot_traj_ref[self.traj_ref_keypoint:], dist_safe_max_to_obst)

    # End
    return flag_path_collision_free, segment_waypoint_init_collision



  def planCollisionFreeSegment(self, waypoint_1, waypoint_2, dist_safe_max_to_obst_plan = 0.0):

    #
    collision_free_path = []


    # Check that waypoints are collision-free

    flag_waypoint_1_coll_free = self.isPointCollisionFree(waypoint_1, dist_safe_max_to_obst_plan)
    flag_waypoint_2_coll_free = self.isPointCollisionFree(waypoint_2, dist_safe_max_to_obst_plan)

    if(flag_waypoint_1_coll_free == False or flag_waypoint_2_coll_free == False):
      print('Waypoints are not collision-free')
      return collision_free_path


    # Check obstacles that generate a collision

    obst_coll_list_idx = []

    for obst_coll_list_idx_i in range(len(self.obstacles_detected_msg.markers)):

      obst_i_msg = self.obstacles_detected_msg.markers[obst_coll_list_idx_i]

      if(obst_i_msg.action == 0):

        if(obst_i_msg.type == 3):

          # Obstacle i
          obst_i_posi = np.zeros((3,), dtype=float)
          obst_i_posi[0] = obst_i_msg.pose.position.x
          obst_i_posi[1] = obst_i_msg.pose.position.y
          obst_i_posi[2] = obst_i_msg.pose.position.z

          obst_i_rad = obst_i_msg.scale.x/2.0


          flag_path_segment_obst_i_collision_free = self.isPathSegmentCollisionFreeObstacle(waypoint_1, waypoint_2, obst_i_posi, obst_i_rad, dist_safe_max_to_obst_plan)

          if(flag_path_segment_obst_i_collision_free == False):
            obst_coll_list_idx.append(obst_coll_list_idx_i)


    # Only working with one obstacle at a time!!!

    if(len(obst_coll_list_idx)>1):
      print("UNABLE TO HANDLE MULTIPLE OBSTACLES!!")
      return collision_free_path


    #
    flag_planned_segment_coll_free = False
    while (flag_planned_segment_coll_free == False):

      # Plan
      collision_free_path = []

      # First waypoint
      collision_free_path.append(waypoint_1)


      # Intermediate waypoints

      # Obstacles that generate a collision
      for obst_coll_list_idx_i in obst_coll_list_idx:

        #
        obst_i_msg = self.obstacles_detected_msg.markers[obst_coll_list_idx_i]

        # Obstacle i
        obst_i_posi = np.zeros((3,), dtype=float)
        obst_i_posi[0] = obst_i_msg.pose.position.x
        obst_i_posi[1] = obst_i_msg.pose.position.y
        obst_i_posi[2] = obst_i_msg.pose.position.z

        obst_i_rad = obst_i_msg.scale.x/2.0        


        ######################


        # TODO BY STUDENT!!!!


        ######################


      # Last waypoint
      collision_free_path.append(waypoint_2)



      # Check collision
      flag_planned_segment_coll_free, segment_waypoint_init_collision = self.isPathCollisionFree(collision_free_path, dist_safe_max_to_obst_plan)

      if(flag_planned_segment_coll_free == False):
        print("UNABLE TO PLAN COLLISION-FREE SEGMENT!!")
        collision_free_path = []
        return collision_free_path



    # Return
    return collision_free_path


  def pathPlannerLoop(self, time_stamp_current):

    if(self.flag_set_robot_traj_ref):

      if(self.flag_set_robot_pose):

        # New path coll free reference
        if(self.flag_new_robot_traj_ref):
          self.flag_new_robot_traj_ref = False

          if(not self.robot_traj_ref):
            self.setCollFreePathRef([])
          else:
            self.setCollFreePathRef(self.robot_traj_ref[0:])


        #
        if(self.robot_traj_ref):

          # Track path ref
          self.trackPathRef()

          # Track path coll free ref
          self.trackPathCollFreeRef()


          # Distance to detected objects
          dist_safe_max_to_obst_nav = self.safety_distance_nav+self.robot_size_radius
          dist_safe_max_to_obst_plan = self.safety_distance_plan+self.robot_size_radius


          # Check if planned path is collision free
          flag_path_planned_collision_free, segment_waypoint_path_planned_init_collision = self.isRobotAndPathPlannedCollFree(dist_safe_max_to_obst_nav)
          
          # Plan collision-free path -> Obstacle avoidance
          if(not self.robot_traj_coll_free_ref or flag_path_planned_collision_free == False):

            # Check if reference path is collision-free and find out in which part there is a collision
            flag_ref_collision_free, segment_waypoint_path_ref_init_collision = self.isRobotAndPathRefCollFree(dist_safe_max_to_obst_nav)

            #
            if(flag_ref_collision_free == True):

              # Ref traj is collision-free

              # Set collision-free path
              self.setCollFreePathRef(self.robot_traj_ref[self.traj_ref_keypoint:])

            else:

              # Ref trajectory is not collision-free

              if(self.flag_hover_when_collision):

                # Just hover
                self.setCollFreePathRef([])

              else:

                # Need to plan a collision free traj different than ref trajectory

                #
                new_path_planned = []
                flag_new_path_planned_collision_free = False
                
                # Path - init robot
                robot_pose = ars_lib_helpers.PoseSimp()
                robot_pose.position = self.robot_posi
                robot_pose.attitude_quat_simp = self.robot_atti_quat_simp

                #
                flag_unable_to_plan_full_path = False
                segment_waypoint_path_ref_init_collision_i = segment_waypoint_path_ref_init_collision

                while(flag_new_path_planned_collision_free == False):

                  # Plan for all the segments
                  while( segment_waypoint_path_ref_init_collision_i < len(self.robot_traj_ref)-1 ):

                    if(segment_waypoint_path_ref_init_collision_i == -1):
                      waypoint_1 = robot_pose
                      waypoint_2 = self.robot_traj_ref[self.traj_ref_keypoint]
                    else:
                      waypoint_1 = self.robot_traj_ref[segment_waypoint_path_ref_init_collision_i]
                      waypoint_2 = self.robot_traj_ref[segment_waypoint_path_ref_init_collision_i+1]
                    

                    # Plan collision-free path segment
                    new_sub_path_planned = self.planCollisionFreeSegment(waypoint_1, waypoint_2, dist_safe_max_to_obst_plan)


                    # Check result of planning collision-free path segment
                    if(not new_sub_path_planned):
                      disp("UNABLE TO PLAN SEGMENT!")
                      flag_unable_to_plan_full_path = True
                      break


                    # Push segment to total path. Note: No last waypoint to avoid duplications!
                    new_path_planned.extend(new_sub_path_planned[0:-1])


                    # Update for next iteration
                    if(segment_waypoint_path_ref_init_collision_i == -1):
                      segment_waypoint_path_ref_init_collision_i = self.traj_ref_keypoint
                    else:
                      segment_waypoint_path_ref_init_collision_i+=1


                  # Check planning result
                  if(flag_unable_to_plan_full_path):
                    break

                  # Append last waypoint!
                  new_path_planned.append(self.robot_traj_ref[-1])

                    
                  # Check if planned path is collision-free
                  flag_new_path_planned_collision_free, segment_waypoint_init_collision = self.isPathCollisionFree(new_path_planned, dist_safe_max_to_obst_plan)


                # Set planned path
                self.setCollFreePathRef(new_path_planned)

    return
