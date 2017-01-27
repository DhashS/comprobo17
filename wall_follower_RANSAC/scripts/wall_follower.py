import numpy as np
import tf
import rospy
from sensor_msgs.msg import LaserScan

from sklearn import linear_model
from numpy.linalg import norm

from sympy import solve, Eq
from sympy.abc import x

class ransac():
    def __init__(self):
        rospy.init_node('ransac')
        self.listener = tf.TransformListener()
        rospy.Subscriber('/scan', LaserScan, logic)
        self.k = rospy.get_param('prop_k', 1)
        self.wall_dist = rospy.get_param("wall_distance", 1)
        self.current_turn_rate = 0

    def logic(self, msg):
        m, b = get_wall_line(msg)
        m_norm, b_norm = get_normal_line(m, b)
        dist = distance(m, b, m_norm, b_norm)
        while dist > 2 * self.wall_dist:
            if np.argmin(msg.ranges) 
            #go in the direction of the normal vector
        while dist > self.wall_dist:
            #rotate to the wall_dist line segment's angle by setting proportional control
            #drive in the direction of the curve
            #one statement (a z rotation?)
        while dist < self.wall_dist:
            #do the inverse



    @staticmethod
    def lidar_polar_cartesian(lidar):
        angles = range(len(lidar.ranges)) * 180/np.pi
        xs = np.cos(angles) * lidar.ranges
        ys = np.sin(angles) * lidar.ranges
        zs = [0] * lidar.ranges
        return zip(xs,ys,zs)

    def get_wall_line(self, msg):
        scan = lidar_polar_cartesian(msg)
        (trans,rot) = self.listener.lookupTransform('/base_laser_link','/base_link',rospy.Time(0))
        #base_link
        X, y = np.array([(x, y) for x, y, _ in scan]).T
        X = np.atleast_2d(X)
        wall = linear_model.RANSACRegressor(linear_model.LinearRegressor())
        wall.fit(X, y)
        line = wall.estimator
        m, b = line.coef_, line.intercept_
        return (m, b)

    @staticmethod
    def get_normal_line(m, b):
        return (m_norm, b_norm)
    
    @staticmethod    
    def distance(m1, b1, m2, b2):
        x_int = solve(Eq(m1*x + b1, m2*x + b2), x, solution_dict=True)[x]
        y_int = m1*x_int + b2
        distance = norm([x_int, y])


    def run(self):
