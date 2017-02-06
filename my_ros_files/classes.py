from __future__ import print_function, division

import subprocess
import sys

import numpy as np

import rospy
import tf

class ROS:
    def __init__(self, setup, ip, rate):
        #Popen build format
        ros_env = " && ".join(setup) + ip
        ros_env = ros_env.split(" ")
        self.rosprocess = subprocess.Popen(ros_env, 
                                           shell=True,
                                           stdout=subprocess.PIPE,
                                           stderr=subprocess.PIPE,
                                           universal_newlines=True)
        self.rate = rate
        rospy.init_node("dhash")
        self.tf = tf.TransformListener()
    
    def register_subscriber(self, topic, T, cb):
        rospy.Subscriber(topic, T, cb)

    def logic():
        pass

    def shutdown_robot():
        pass

    def run(self):
        rate = rospy.rate(self.rate)
        while not rospy.is_shutdown():
            try:
                self.logic()
                #handle I/O redirection
                print(self.rosprocess.stdout.read(), file=sys.stdout)
                print(self.rosprocess.stderr.read(), file=sys.stderr)
                rate.sleep()
            except rospy.ROSException as e:
                print("ROS exception hit: ", e)
            except KeyboardInterrupt:
                print("Stop execution noticed...")

            finally:
                self.shutdown_robot()
                self.rosprocess.terminate()



class Neato(ROS):
    def __init__(self, ip, rate=5):
        from geometry_msgs.msg import Twist, Vector3
        from sensor_msgs.msg import LaserScan
        from nav_msgs.msg import Odometry


        super(Neato, self).__init__(std_setup(), ip, rate)
        
        self.last_pos = self.position(rospy.wait_for_message('/odom', Odometry))
        self.last_laser_cloud = self.laser_to_cloud(rospy.wait_for_message('/scan', LaserScan))
        
        self.historical_pos = [self.last_pos]
        self.historical_clouds = [(self.last_pos, self.last_laser_cloud)]

        self.register_subscriber('/odom', Odometry, self.log_position)
        self.register_subscriber('/scan', LaserScan, self.log_laser)
        
        self.move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    def shutdown_robot(self):
        self.move_pub.publish(self.stop())

    def movement(goal):
        #compute a path that minimizes energy from current position to a point
        #all in /odom frame
        points = self.laser_to_cloud(self.last_laser_cloud)
        #TODO
        #object avoidance, bezier curves, RBF svm, hoo boy.
        pass #for now
    
    @staticmethod
    def laser_to_cloud(msg):
        scan = msg.ranges[:-1] #the last value is a repeated first value
        angles = np.array(range(len(scan))) * 180/np.pi 
        xs = np.cos(angles) * scan
        ys = np.sin(angles) * scan
        zs = [0] * len(scan)
        points = [Point32(x,y,x) for x,y,z in zip(xs,ys,zs) if not np.norm([x,y,z]) == 0.0] #drop all zero-distance readings
        cloud = PointCloud(header=Header(frame_id="/base_laser_link",
                                         stamp=msg.header.stamp),
                           points=points,
                           channels=ChannelFloat32(name="distance",
                                                   values=[d for d in scan if not d == 0.0]))
        return cloud

    def log_laser(msg):
        cloud = self.laser_to_cloud(msg)
        self.historical_clouds.append(self.last_laser_cloud)
        self.last_laser_cloud = (self.last_pos, cloud)

    @staticmethod
    def position(msg):
        pos = [getattr(msg.pose.pose.position, x) for x in 'xyz']
        angle = [getattr(msg.pose.pose.orientation, x) for x in 'xyzw']
        return np.array([pos + [0], angle]) #to do array ops, needs to be square
    
    def log_position(self, msg):
        self.historical_pos.append(self.last_pos)
        self.last_pos = self.position(msg)
    
    @staticmethod
    def move_forward(vel):
        return self.compound_move(lx=vel)

    @staticmethod
    def stop():
        return self.compound_move(lx=0,ly=0,lz=0,
                                  ax=0,ay=0,az=0)

    @staticmethod
    def rotate_clockwise(vel):
        return self.compound_move(lz=vel)

    @staticmethod
    def rotate_anticlockwise(vel):
        return self.compound_move(lz=-vel)

    @staticmethod
    def compound_move(lx=0, ly=0, lz=0, ax=0, ay=0, az=0):
        return Twist(linear=Vector3(x=lx,
                                    y=ly,
                                    z=lz),
                     angular=Vector3(x=ax,
                                     y=ay,
                                     z=az))


def std_setup():
    src_scripts = ["source /home/dhash/School/CompRobo S17/ws/devel/setup.sh",
                   "source /home/dhash/School/CompRobo S17/ws/src/venv/bin/activate"]
    roslaunch = ["roslaunch neato_node my_bringup.launch host:="]
    return src_scripts + roslaunch
    
