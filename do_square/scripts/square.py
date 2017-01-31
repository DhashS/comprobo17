#!/usr/bin/env python

from __future__ import print_function, division

import numpy as np
from numpy.linalg import norm

import tf
import rospy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry


class Neato:
    def __init__(self):
        rospy.init_node('square')
        
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.distance = rospy.get_param("distance", 0.1)
        self.speed = rospy.get_param("speed", 1)
        
        self.last_keypoint = self.curr_position(rospy.wait_for_message('/odom', Odometry))
        self.curr_pos = self.last_keypoint
        self.to_angle = None
        rospy.Subscriber('/odom', Odometry, self.curr_position)
        
        self.phase = "Forward"

        self.rate = rospy.Rate(20)
     
    def curr_position(self, msg):
        pos = [getattr(msg.pose.pose.position, coord) for coord in 'xyz']
        angle = [getattr(msg.pose.pose.orientation, coord) for coord in 'xyzw'] #TODO
        self.curr_pos = np.array([pos + [0], angle])
        return self.curr_pos
   
    def logic(self):
        distance, _= abs(self.curr_pos - self.last_keypoint) 
        angle = tf.transformations.euler_from_quaternion(self.last_keypoint[1])[2]
        angle = angle + np.pi
        now_angle = tf.transformations.euler_from_quaternion(self.curr_pos[1])[2]
        now_angle = now_angle + np.pi
        print(distance)
        if angle + np.pi/2 > 2*np.pi:
            if self.to_angle is None:
                self.to_angle = np.pi/2 - (2*np.pi - now_angle)
            angle = self.to_angle

            if now_angle > np.pi*3/2:
                now_angle = 0
        else:
            angle = angle + np.pi/2
            


        #pretty simple 2-state within 2-state FSM
        if self.phase == "Forward":
            if norm(distance) < self.distance:
                self.pub.publish(self.go_fwd(self.speed))
                rospy.loginfo("{} at {}".format(self.phase, self.speed))
                return
            if norm(distance) >= self.distance:
                self.pub.publish(self.stop())
                rospy.loginfo("Reached polygon edge, beginning to Rotate")
                self.phase = "Rotate"
                self.last_keypoint = self.curr_pos
                return
        if self.phase == "Rotate":
            print(now_angle, angle)
            if now_angle - angle < 0:
                self.pub.publish(self.turn_right(-0.5))
                return
            else:
                self.pub.publish(self.stop())
                self.to_angle = None
                self.last_keypoint = self.curr_pos
                self.phase = "Forward"
                return

    def run(self):
        while not rospy.is_shutdown():
            self.logic()
            self.rate.sleep()

    @staticmethod
    def stop():
        return Twist(linear=Vector3(0, 0, 0),
                     angular=Vector3(0, 0, 0))
    @staticmethod
    def turn_left(v):
        return Twist(linear=Vector3(0, 0, 0),
                     angular=Vector3(0, 0, v)) #TODO
    
    @staticmethod 
    def turn_right(v):
        return Twist(linear=Vector3(0, 0, 0),
                     angular=Vector3(0, 0, -v)) #TODO
    
    @staticmethod 
    def go_fwd(v):
        return Twist(linear=Vector3(v, 0, 0), #TODO
                     angular=Vector3(0, 0, 0))


square = Neato()
square.run()
