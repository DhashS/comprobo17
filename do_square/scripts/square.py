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
        self.distance = rospy.get_param("distance", 1)
        self.speed = rospy.get_param("speed", 1)
        
        self.last_keypoint = self.curr_position(rospy.wait_for_message('/odom', Odometry))
        self.curr_pos = self.last_keypoint
        
               
        self.rate = rospy.Rate(5)
     
    def curr_position(msg):
        pos = [getattr(msg.position, coord) for coord in 'xyz']
        angle = [getattr(msg.position, coord) for coord in 'xyzw'] #TODO
        self.curr_pos = array([pos, angle])
        return self.curr_pos
   
    def logic(self):
        distance, angle = self.curr_pos - self.last_keypoint 
        if self.phase == "Forward":
            if norm(distance) < self.distance:
                self.pub.publish(go_fwd(self.speed))
                rospy.loginfo("{} at {}".format(self.phase, self.speed))
            if norm(distance) >= self.distance:
                self.pub.publish(stop())
                rospy.loginfo("Reached polygon edge, beginning to Rotate")
                self.phase = "Rotate"
                self.last_keypoint = self.curr_pos
        if self.phase == "Rotate":
            #TODO
            if angle < (np.pi/2):
                self.pub.publish(turn_left(self.speed))
                rospy.loginfo("{} at {}".format(self.phase, self.speed))
            #TODO
            if angle >= (np.pi/2):
                self.pub.publish(stop())
                self.rospy.loginfo("Done rotating, beginning to move forward")
                self.last_keypoint = self.curr_pos
                self.phase = "Forward"

    def run(self):
        while not rospy.is_shutdown():
            logic()
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
