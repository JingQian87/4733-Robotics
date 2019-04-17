#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi

#TB determined, from Q3
path_file = "../data/path.txt" 
path = np.genfromtxt(fname = path_file)

class FollowPath():
    def __init__(self):
        # Give the node a name
        rospy.init_node('move_robot', anonymous=False)

        # Set rospy to execute a shutdown function when exiting       
        rospy.on_shutdown(self.shutdown)

        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        #self.cmd_vel = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)
        
        # How fast will we update the robot's movement?
        rate = 20
        
        # Set the equivalent ROS rate variable
        r = rospy.Rate(rate)
        
        # Set the forward linear speed to 0.15 meters per second 
        linear_speed = 0.15

        # Set the rotation speed in radians per second
        angular_speed = 0.5
        
        # Set the angular tolerance in degrees converted to radians
        angular_tolerance = radians(1.0)
        
        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        #rospy.sleep(2)
        
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
            

        # Following points in path
        for i in range(1, len(path)):
            # Set parameters for current target
            # Initialize the movement command
            move_cmd = Twist()
            
            # Get the starting position values     
            (position, rotation) = self.get_odom()
                        
            x_start = position.x
            y_start = position.y

            # Scale the points in path.txt by 0.01
            goal = Point(path[i][0]*0.01, path[i][1]*0.01, 0)
            goal_distance = sqrt((goal.x-x_start)**2 + (goal.y-y_start)**2)
            goal_angle = np.arctan2(goal.y-y_start, goal.x-x_start)
            goal_angle -= rotation
            if goal_angle < 0:
                goal_angle += 2*pi
            
            # Rotate before moving
            print("Moving towards ("+str(goal.x)+", "+str(goal.y)+")!")                        
            # Set the movement command to a rotation
            move_cmd.angular.z = angular_speed
            
            # Track the last angle measured
            last_angle = rotation
            
            # Track how far we have turned
            turn_angle = 0
            
            while abs(turn_angle + angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
                # Publish the Twist message and sleep 1 cycle         
                self.cmd_vel.publish(move_cmd)
                r.sleep()
                
                # Get the current rotation
                (position, rotation) = self.get_odom()
                
                # Compute the amount of rotation since the last loop
                delta_angle = normalize_angle(rotation - last_angle)
                
                # Add to the running total
                turn_angle += delta_angle
                last_angle = rotation

            # Stop the robot before moving
            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1)

            # Set the movement command to forward motion
            move_cmd.linear.x = linear_speed

            # Keep track of the distance traveled
            distance = 0
            
            # Enter the loop to move along a side
            while distance < goal_distance and not rospy.is_shutdown():
                print("Moving towards ("+str(goal.x)+", "+str(goal.y)+")!")

                # Publish the Twist message and sleep 1 cycle         
                self.cmd_vel.publish(move_cmd)          
                r.sleep()
        
                # Get the current position
                (position, rotation) = self.get_odom()

                
                # Compute the Euclidean distance from the start
                distance = sqrt(pow((position.x - x_start), 2) + 
                                pow((position.y - y_start), 2))
                print("Current position: (" + str(position.x) + ", " + str(position.y) + "), " + str(distance) + " away from position ("+str(goal.x)+", "+str(goal.y)+")!")
            
            
            print("Robot reaches ("+str(goal.x)+", "+str(goal.y)+")!")    
            # Stop the robot before the next edge
            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1)
        
        print("Robot reaches final goal!")    
        # Stop the robot for good
        self.cmd_vel.publish(Twist())
        
    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))
        
    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        FollowPath()
    except:
        rospy.loginfo("Follow path node terminated.")

