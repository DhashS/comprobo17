import rospy 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3


class Naive_wall_follow:
    def __init__(self):
        rospy.init_node("naive-follower")

        self.move = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.tol = rospy.get_param('tol', 0.05)
        
        rospy.Subscriber('/scan', LaserScan, self.differential)
        self.last_range = None
    def differential(self, msg):
        d1, d2 = msg.ranges[10], msg.ranges[100]
        if d1 == 0.0 or d2 == 0.0:
            self.move.publish(self.fwd())
        if self.last_range is None:
            self.last_range = (d1, d2)
        delta1, delta2 = self.last_range[0]-d1, self.last_range[1]-d2
        print(delta1, delta2)
        if delta1 < self.tol or delta2 > self.tol:
            self.move.publish(self.left())
            return
        elif delta1 > self.tol or delta2 < self.tol:
            self.move.publish(self.right())
            return
        else:
            self.move.publish(self.fwd())
            return

    def run(self):
        rate = rospy.Rate(2)
        try:
            while not rospy.is_shutdown(): 
                rate.sleep()
        except:
            self.move.publish(self.stop())

        
    @staticmethod
    def fwd():
        return Twist(linear=Vector3(0.3, 0, 0),
                     angular=Vector3(0, 0, 0))
    @staticmethod
    def left():
        return Twist(linear=Vector3(0.1, 0, 0),
                     angular=Vector3(0, 0, 0.1))

    @staticmethod
    def right():
        return Twist(linear=Vector3(0.1, 0, 0),
                     angular=Vector3(0, 0, -0.1))
    @staticmethod
    def stop():
        return Twist(linear=Vector3(0, 0, 0),
                     angular=Vector3(0, 0, 0))
r = Naive_wall_follow()
r.run()
