from __future__ import print_function, division

import subprocess
import sys

import rospy
import tf

class ROS():
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
    
    def logic():
        pass

    def run(self):
        rate = rospy.rate(self.rate)
        while not rospy.is_shutdown():
            logic()
            #handle I/O redirection
            print(self.rosprocess.stdout, file=sys.stdout)
            print(self.rosprocess.stderr, file=sys.stderr)
            rate.sleep()


class Neato(ROS):
    def __init__(self):
        super(Neato, self).__init__(std_setup(), "192.168.16.52", 5)

    @staticmethod
    def movement(linear, ang):

    def move_forward()    
    

def std_setup():
    src_scripts = ["source /home/dhash/School/CompRobo S17/ws/devel/setup.sh",
                   "source /home/dhash/School/CompRobo S17/ws/src/venv/bin/activate"]
    roslaunch = ["roslaunch neato_node my_bringup.launch host:="]
    return src_scripts + roslaunch
    
