from numpy import array
from numpy.linalg import norm

import tf
import rospy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry


class Neato:
    def __init__(self):
        rospy.init_node('square')
        
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(5)
        self.speed = rospy.get_param("speed", 1)
        self.last_keypoint = self.log_position(rospy.wait_for_message('/odom', Odometry))
        
    
    def logic(self):
        if phase == "Forward":
            if self.log_position[0]


    def turn_left(v):
        return Twist(Vector3(0, 0, 0), Vector3(0, 0, v)) #TODO

    def turn_right(v):
        return Twist(Vector3(0, 0, 0), Vector3(0, 0, -v)) #TODO

    def go_fwd(v):
        return Twist(Vector3(v, 0, 0), Vector3(0, 0, 0))

    def log_position(msg):
        pos = [getattr(msg.position, coord) for coord in 'xyz']
        angle = [getattr(msg.position, coord) for coord in 'xyzw'] #TODO
        return array([pos, angle])

    def drive_square(size):
        while not rospy.is_shutdown():
            phase = self.phase
            


