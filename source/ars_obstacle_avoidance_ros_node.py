#!/usr/bin/env python

import numpy as np
from numpy import *

import os


import rospy


from ars_obstacle_avoidance_ros import *




def main():

  ars_obstacle_avoidance_ros = ArsObstacleAvoidanceRos()

  ars_obstacle_avoidance_ros.init()
  ars_obstacle_avoidance_ros.open()

  try:
    ars_obstacle_avoidance_ros.run()
  except rospy.ROSInterruptException:
    pass


  return 0



''' MAIN '''
if __name__ == '__main__':

  main()