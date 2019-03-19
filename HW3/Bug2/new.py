#!/usr/bin/env python

""" odom_out_and_back.py - Version 1.1 2013-12-20
    A basic demo of using the /odom topic to move a robot a given distance
    or rotate through a given angle.
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.
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

import sys
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import LaserScan
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi, isnan

class OutAndBack():
    def __init__(self):
        # Give the node a name
        rospy.init_node('out_and_back', anonymous=False)

        # Set rospy to execute a shutdown function when exiting       
        rospy.on_shutdown(self.shutdown)

        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)

        # Subscriber to obtain the distance to nearest obstacle
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # How fast will we update the robot's movement?
        rate = 10
        
        # Set the equivalent ROS rate variable
        r = rospy.Rate(rate)
        
        # Set the forward linear speed to 0.40 meters per second 
        linear_speed = 0.40
        
        # Set the travel distance in meters
        goal_distance = 10.0

        # Set the rotation speed in radians per second
        angular_speed = 0.5

        # Set the angular tolerance in degrees converted to radians
        angular_tolerance = radians(5.0)

        # Set the proximity tolerances
        self.proximity_tolerance = 0.30 # within this distance is considered hit obstacle or reached goal!
        threshold_dist = 0.80 # this is the threshold distance to obstacle before turning
        mline_dist = 0.20 # within this distance is considered on mline!

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        
        # Set the odom frame
        self.odom_frame = '/odom'
        
        # Find out if the robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")  
        
        # Initialize the position variable as a Point type
        position = Point()
            
        # Initialize the movement commands
        move_cmd_forward = Twist()
        move_cmd_left = Twist()
        move_cmd_right = Twist()
        
        # Set the movement command to forward/left/right motion
        move_cmd_forward.linear.x = linear_speed
        move_cmd_left.angular.z = angular_speed
        move_cmd_right.angular.z = -angular_speed
        
        # Get the starting position values     
        (position, rotation) = self.get_odom()
                    
        x_start = position.x
        y_start = position.y

	start_rotation = rotation

        # Goal position values
        self.x_goal = x_start + goal_distance
        self.y_goal = y_start

        # Hit point
        hit_point = None

        # Flags
        MLINE, FORWARD, TURNLEFT, TURNRIGHT = 0, 1, 2, 3

        move_cmds = {
            MLINE: move_cmd_forward,
            FORWARD: move_cmd_forward, 
            TURNLEFT: move_cmd_left,
            TURNRIGHT: move_cmd_right
        }

        # Initial direction
        direction = MLINE

        moved_after_hit = False

        while not rospy.is_shutdown():
            if self.goaltest(position): # Success!
                print("Robot reached goal! Exiting...")
                # Stop the robot
                self.cmd_vel.publish(Twist())
                break
            
            elif self.ahead_range < self.proximity_tolerance:
                print("Ouch! I hit something...")
                print("Exiting...")
                # Stop the robot
                self.cmd_vel.publish(Twist())
                break
            
            elif not direction == MLINE and abs(position.y - y_start) <= mline_dist and hit_point and self.dist_to_goal(position) - self.dist_to_goal(hit_point) < -0.3 and self.dist_to_point(position, hit_point) >= self.proximity_tolerance:
                # Continue along m-line!
                print(self.dist_to_goal(position), self.dist_to_goal(hit_point), self.dist_to_point(position, hit_point))
                print("Leave point found. Coordinates: (" + str(position.x) + ", " + str(position.y) + ")")
                print("Turned angle: " + str(rotation - start_rotation) + ". Turning back...")
                
                while abs(rotation - start_rotation) > angular_tolerance:
                    print(rotation, start_rotation, abs(rotation - start_rotation))
                    # Publish the Twist message and sleep 1 cycle
                    self.cmd_vel.publish(move_cmds[TURNLEFT])
                    r.sleep()
                    (position, rotation) = self.get_odom()
                    
                direction = MLINE
                moved_after_hit = False
            
            elif not direction == MLINE and abs(position.y - y_start) <= mline_dist and hit_point and self.dist_to_point(position, hit_point) < self.proximity_tolerance and moved_after_hit:
                print("Oh! No solution! Exiting...")
                # Stop the robot
                self.cmd_vel.publish(Twist())
                break
            
            elif self.ahead_range < threshold_dist: # Obstacle encountered, turn left
                print("Robot reached obstacle! Turning left...")
                print(self.ahead_range)

                if direction == MLINE: # this is a hit point!
                    print("Hit point found. Coordinates: (" + str(position.x) + ", " + str(position.y) + ")")
                    hit_point = position

                direction = TURNLEFT

                # Publish the Twist message and sleep 1 cycle
                self.cmd_vel.publish(move_cmds[direction])
                r.sleep()

            else:
                print(self.ahead_range)
                if direction == MLINE:
                    # following mline, continue moving forward
                    print('Keep following m-line...')
                elif direction == TURNLEFT:
                    # originally turning left, no more obstacles! move forward
                    direction = FORWARD
                    print('Moving forward...')
                    moved_after_hit = True
                elif direction == FORWARD:
                    direction = TURNRIGHT
                    print('Turning right...')
                elif direction == TURNRIGHT:
                    direction = FORWARD
                    print('Moving forward...')

                # Publish the Twist message and sleep 1 cycle
                self.cmd_vel.publish(move_cmds[direction])
                r.sleep()
    
            # Get the current position
            (position, rotation) = self.get_odom()

    def dist_to_point(self, position1, position2):
        return ((position1.x - position2.x) ** 2 + (position1.y - position2.y) ** 2) ** .5
    
    def dist_to_goal(self, position):
        return ((position.x - self.x_goal) ** 2 + (position.y - self.y_goal) ** 2) ** .5
    
    def goaltest(self, position):
        return self.dist_to_goal(position) <= self.proximity_tolerance

    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))
    
    def scan_callback(self, msg):
	laser_min = msg.range_max
	for i in msg.ranges:
	    if not isnan(i):
		laser_min = min(laser_min, i)
	self.ahead_range = laser_min
        
    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        OutAndBack()
    except:
        rospy.loginfo("Out-and-Back node terminated.")