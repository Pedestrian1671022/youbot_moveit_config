#!/usr/bin/env python

"""
    moveit_ik_demo.py - Version 0.1 2014-01-14
    
    Use inverse kinemtatics to move the end effector to a specified pose
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Int32

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class MoveItDemo:
    # spin() simply keeps python from exiting until this node is stopped

    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # rospy.init_node('moveit_ik')
        # rospy.Subscriber("chatter", Pose, callback)
        # Initialize the move group for the right arm
        arm = moveit_commander.MoveGroupCommander('arm')

        # Set the reference frame for pose targets
        reference_frame = 'base_link'

        # Set the right arm reference frame accordingly
        arm.set_pose_reference_frame(reference_frame)

        # Allow replanning to increase the odds of a solution
        arm.allow_replanning(True)

        # Allow some leeway in position (meters) and orientation (radians)
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.01)

        arm.set_random_target()
        traj = arm.plan()
        arm.execute(traj)
        rospy.sleep(3)

        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()

        # Exit MoveIt
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    try:
        MoveItDemo()
    except KeyboardInterrupt:
        raise
