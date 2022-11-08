#!/usr/bin/env python3
from cmath import inf
from re import L
import time
import rospy
from geometry_msgs.msg import Twist , Pose
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np
import math
import sys
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Araba:
    def __init__(self):
        rospy.init_node("robot")
        self.pub = rospy.Publisher("cmd_vel",Twist,queue_size=1)
        self.hiz = Twist()
    
    def ileri_git_s(self,s):
        self.hiz.linear.x = 0.25
        t1 = time.time()
        t2 = time.time()
        while t2-t1<s:
            self.pub.publish(self.hiz)
            t2 = time.time()

    def ileri_git(self):
        self.hiz.linear.x = 0.25
        self.pub.publish(self.hiz)
    
    def ileri_sol_git(self):
        
        self.hiz.linear.x = 0.25
        self.hiz.angular.z = 0.07
        self.pub.publish(self.hiz)
            
    def ileri_sag_git(self):
        
        self.hiz.linear.x = 0.25
        self.hiz.angular.z = -0.05
        self.pub.publish(self.hiz)
        
    def sola_don(self):
        self.hiz.angular.z = -0.17
        self.pub.publish(self.hiz)
           
    
    def saga_don(self):
        self.hiz.angular.z = -0.17
        self.pub.publish(self.hiz)


    def dur(self):
        self.hiz.angular.z = 0.0
        self.hiz.linear.x = 0.0
        self.pub.publish(self.hiz)
 




def odom(msg):
    global x , y , yaw
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    
def laser(msg):
    global laser_on
    sol_on = list(msg.ranges[0:9])
    sag_on = list(msg.ranges[350:359])
    laser_on = sol_on + sag_on


def inf_to_binary():
    #return len([x for x in laser_on if x != inf])
    return np.average([x for x in laser_on if x != inf])



def sola_kac():
    araba = Araba()
    while not rospy.is_shutdown():
        try:
            print(inf_to_binary())
            if inf_to_binary() < 0.4:
                araba.dur()
            elif inf_to_binary() < 3:
                araba.ileri_sol_git()
                flag = False
            elif flag:
                araba.ileri_git_s(4)
                break
            elif str(inf_to_binary()) == "nan":
                flag = True
        except:pass

def konumu_duzelt(deg):
    araba = Araba()
    while not rospy.is_shutdown(): #Konum duzeltme
        print(yaw*60)
        if not deg-1 < yaw*60 < deg+1:
            araba.saga_don()
        else:
            break


def sag_merkeze_don():
    araba = Araba()
    while not y<0:
        print(x,y)
        araba.ileri_sag_git()
    
def sol_merkeze_don():
    araba = Araba()
    while not y>0:
        print(x,y)
        araba.ileri_sag_git()

def main():
    rospy.Subscriber('/odom', Odometry, odom)
    rospy.Subscriber('/scan', LaserScan, laser)
    araba = Araba()
    print("debug 1")
    sola_kac()
    
    araba.dur()
    time.sleep(1)
    konumu_duzelt(0)
    
    araba.ileri_git_s(3)

    sag_merkeze_don()
    time.sleep(1)
    konumu_duzelt(-170)
    sola_kac()
    araba.dur()
    time.sleep(1)
    konumu_duzelt(-172)
    araba.ileri_git_s(8)
    araba.dur()   
    sol_merkeze_don()
    araba.dur()
        


if __name__ == "__main__":
    main()