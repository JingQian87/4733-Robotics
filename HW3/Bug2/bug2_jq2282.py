#!/usr/bin/env python

""" Videos for different worlds could be seen in following shared link:
https://drive.google.com/drive/folders/1EiRKY0wN-7oyT5FU-k_H--bAhaQ0sJ8f?usp=sharing
      
"""

import sys
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import LaserScan
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi, isnan

destination = Point(10, 0, 0)

class Bug2():
    def __init__(self):
        # Give the node a name
        rospy.init_node('Bug2', anonymous=False)

        # Set rospy to execute a shutdown function when exiting       
        rospy.on_shutdown(self.shutdown)

        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)

        # Subscriber to obtain the distance to nearest obstacle
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # How fast will we update the robot's movement?
        rate = 5
        
        # Set the equivalent ROS rate variable
        r = rospy.Rate(rate)
        
        # Set the forward linear speed to 0.25 meters per second 
        linear_speed = 0.40
        
        # Set the travel distance in meters
        goal_distance = 10.0

        # Set the rotation speed in radians per second
        angular_speed = 0.5
        
        # Set the angular tolerance in degrees converted to radians
        angular_tolerance = radians(1.0)
        
        # Set the rotation angle to Pi radians (180 degrees)
        # goal_angle = pi

        # Set the proximity tolerance
        self.proximity_tolerance = 0.30

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

        # Hit point
        hit_point = None

        # Threshold distance to obstacle before turning
        threshold_dist = 0.80

        mline_dist = 0.2

        # Flags
        m_line, FORWARD, TURNLEFT, TURNRIGHT = 0, 1, 2, 3

        move_cmds = {
            m_line: move_cmd_forward,
            FORWARD: move_cmd_forward, 
            TURNLEFT: move_cmd_left,
            TURNRIGHT: move_cmd_right
        }

        # Initial direction
        direction = m_line

	    # Previous direction
        prevdirection = 0

        moves = 0
        
        while not rospy.is_shutdown():
            if self.goaltest(position): # Success!
                print("Goal reached!")
                # Stop the robot
                self.cmd_vel.publish(Twist())
                break
            elif self.ahead_range < self.proximity_tolerance:
                print("No, I cannot reach the goal!")
                # Stop the robot
                self.cmd_vel.publish(Twist())
                break
            elif not direction == m_line and abs(position.y) <= 0.2 and hit_point and self.getDistance(position, destination) - self.getDistance(hit_point, destination) < -0.3 and self.dist_to_point(position, hit_point.x, hit_point.y) >= self.proximity_tolerance:
                # Continue along m-line!
                print(self.getDistance(position, destination), self.getDistance(hit_point, destination), self.dist_to_point(position, hit_point.x, hit_point.y))
                print("Leave point found. Coordinates: (" + str(position.x) + ", " + str(position.y) + ")")
                
                while abs(rotation - start_rotation) > angular_tolerance:
                    print(rotation, start_rotation, abs(rotation - start_rotation))
                    # Publish the Twist message and sleep 1 cycle
                    self.cmd_vel.publish(move_cmds[TURNLEFT])
                    r.sleep()
                    (position, rotation) = self.get_odom()
                
                direction = m_line  
                moves = 0
            
            elif not direction == m_line and abs(position.y - y_start) <= mline_dist and hit_point and self.dist_to_point(position, hit_point.x, hit_point.y) < self.proximity_tolerance and moves >= 25:
                print("No, I cannot reach the goal!")
                # Stop the robot
                self.cmd_vel.publish(Twist())
                break  

            elif self.ahead_range < threshold_dist: # Obstacle encountered, turn left
                print("Robot hit obstacle! Turning left...")

                if direction == m_line: # this is a hit point!
                    print("Hit point found. Coordinates: (" + str(position.x) + ", " + str(position.y) + ")")
                    hit_point = position

                prevdirection = direction
                direction = TURNLEFT

                # Publish the Twist message and sleep 1 cycle
                self.cmd_vel.publish(move_cmds[direction])
                r.sleep()

            else:
                if direction == m_line:
                    # following mline, continue moving forward
                    print('Keep following m-line...')
                    pass
                elif direction == TURNLEFT:
                    # originally turning left, no more obstacles! move forward
                    prevdirection = direction
                    direction = FORWARD
                    print('Moving forward...')
                    moves += 1
                elif direction == FORWARD:
            	    prevdirection = direction
            	    direction = TURNRIGHT
            	    print('Turning right...')
                elif direction == TURNRIGHT:
            	    prevdirection = direction
            	    direction = FORWARD
            	    print('Moving forward...')

                # Publish the Twist message and sleep 1 cycle
                self.cmd_vel.publish(move_cmds[direction])
                r.sleep()

            # Get the current position
            (position, rotation) = self.get_odom()
            print(str(self.getDistance(position, destination)) + ' away from destination!')


    def dist_to_point(self, position, x, y):
        return ((position.x - x) ** 2 + (position.y - y) ** 2) ** .5
    
    def getDistance(self, position, destination):
        result =  sqrt((position.x- destination.x)**2+ ( position.y-destination.y)**2)
        return result
    
    def goaltest(self, position):
        return self.getDistance(position, destination) <= self.proximity_tolerance

    def scan_callback(self, msg):
        min_dis = sys.float_info.max
        for i in msg.ranges:
            if not isnan(i):
                min_dis = min(min_dis, i)
        self.ahead_range = min_dis

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
        Bug2()
    except Exception as e:
        print(e)
        rospy.loginfo("Bug2 node terminated.")