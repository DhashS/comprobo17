#!/usr/bin/env python

import numpy as np
import tf
import rospy
from sensor_msgs.msg import LaserScan, PointCloud, ChannelFloat32
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, Vector3, Point32
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
#TODO imports


from sklearn import linear_model
from numpy.linalg import norm

from sympy import solve, Eq
from sympy.abc import x

class ransac:
    def __init__(self):
        rospy.init_node('ransac')
        self.listener = tf.TransformListener()
        rospy.Subscriber('/scan', LaserScan, self.logic)
        rospy.Subscriber('/odom', Odometry, self.position) 
        self.keypoint = self.position(rospy.wait_for_message('/odom', Odometry))
        self.curr_position = self.keypoint
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        #self.k = rospy.get_param('prop_k', 1)
        #for prop_control
        self.wall_dist = rospy.get_param("wall_distance", 0.1)
        self.tol = rospy.get_param('tol', 0.1)
        self.ang_tol = rospy.get_param('ang_tol', 10)
        self.debug_line = Marker
        self.debug_pub = rospy.Publisher('/wall_line', Marker, queue_size=10)
        #self.current_turn_rate = 0
        #self.wall_set = {} #dict to store (if wall) angle -> range mappings
        #self.non_wall_set = {} #dict to store rejects angle -> range mappings

    def logic(self, msg):
        m, b = self.get_wall_line(msg) #RANSAC with linear model results in y=mx+b
        m_norm, b_norm = self.get_normal_line(m, b) #solve it's normal eq 
        dist = self.distance(m, b, m_norm, b_norm) #get distance
        print(dist, " from wall")
        _, angle = self.curr_position
        angle = tf.transformations.euler_from_quaternion(angle)
        if dist > self.wall_dist:
            self.pub.publish(self.go_forward())
            return
        err = abs(dist-self.wall_dist)

        xs = np.linspace(0, 10)
        ys = xs*m + b
        self.debug_pub.publish(self.debug_line(header=Header(frame_id="/odom"), points=[Point32(x, y, 0) for x, y in zip(xs, ys)], type=Marker.LINE_STRIP))
        
        if err > self.tol: #easy corrective w/ tol
            if err > 0:
                # above line
                self.wall_turn(msg, 90)
                return
            else:
                # below line
                self.wall_turn(msg, 360 - 90)
                return
        else:
            self.pub.publish(self.go_forward())
            return


    def wall_turn(self, msg, hdg=90):
        closest = np.argmin(msg.ranges)
        delta = closest - hdg
        if delta > 0:
            self.pub.publish(self.turn_left())
            
        else:
            self.pub.publish(self.turn_right())


    @staticmethod
    def lidar_polar_cartesian(lidar):
        angles = np.array(range(len(lidar.ranges))) * 180/np.pi
        xs = np.cos(angles) * lidar.ranges
        ys = np.sin(angles) * lidar.ranges
        zs = [0] * len(lidar.ranges)
        points = [Point32(x, y, z) for x,y,z in zip(xs,ys,zs) if norm([x, y, z]) > lidar.range_min]
        return PointCloud(header=Header(frame_id="/base_laser_link",
                                        stamp=rospy.Time.now()),
                         points=points,
                         channels=ChannelFloat32(name="distance",
                                                 values=[norm([getattr(x, y) for y in 'xyz']) for x in points]))


    def get_wall_line(self, msg):
        scan = self.lidar_polar_cartesian(msg)
        #base_link transform from base_laser_link to all points in rospy.ranges
        scan = self.listener.transformPointCloud('/base_link', scan)
        #build up xy map for fit
        X, y = np.array([(p.x, p.y) for p in scan.points]).T
        X = X.reshape(-1, 1)
        wall = linear_model.RANSACRegressor(linear_model.LinearRegression())
        wall.fit(X, y)
        line = linear_model.LinearRegression().fit(X[wall.inlier_mask_], y[wall.inlier_mask_])
        m, b = line.coef_, line.intercept_
        return (m, b)

    @staticmethod
    def get_normal_line(m, b):
        return (1/m, 0)
    
    @staticmethod    
    def distance(m1, b1, m2, b2):
        x_int = solve(Eq(m1[0]*x + b1, m2[0]*x + b2), x, solution_dict=True)[0]
        y_int = m1[0]*x_int + b2
        x_int = float(x_int)
        y_int = float(y_int)
        return norm([x_int, y_int])
    
    def position(self, msg):
        pos = [getattr(msg.pose.pose.position, x) for x in 'xyz']
        angle = [getattr(msg.pose.pose.orientation, x) for x in 'xyzw']
        self.curr_position = np.array([pos, angle])
        return self.curr_position

    def run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            rate.sleep()
    @staticmethod
    def turn_left():
        return Twist(Vector3(0.2, 0, 0), Vector3(0, 0, -0.4))
    @staticmethod
    def turn_right():
        return Twist(Vector3(0.2, 0, 0), Vector3(0, 0, 0.4))
    @staticmethod
    def go_forward():
        print("going fwd")
        return Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

r = ransac()
r.run()
