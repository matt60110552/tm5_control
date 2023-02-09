#!/usr/bin/env python

from ast import Pass
import rospy
import subprocess as sp
import os,sys
from sophia_test.srv import do_unseen,do_unseenResponse
from geometry_msgs.msg import PoseStamped,Twist,Transform
from std_msgs.msg import Bool
from tm_msgs.msg import FeedbackState
import tf 
class STAGE:
    def __init__(self):
        
        # initialize a node
        rospy.init_node("stage_switch")
        self.robot_target_pub = rospy.Publisher("/robot_target", Transform, queue_size=1)
        rospy.wait_for_service('do_unseen')
        self.do_unseen_srv = rospy.ServiceProxy('do_unseen', do_unseen)
        
    def do_sub(self, cmd):
        print("fuck", cmd)
        sp.call(cmd, shell=True)
    
if __name__=='__main__':
    stage = STAGE()
    stage.do_sub('ls')
