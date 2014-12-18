#!/usr/bin/env python

import roslib; 
roslib.load_manifest('dmp')
import rospy 
import numpy as np
from dmp.srv import *
from dmp.msg import *
from dance_bot.msg import Path, ArmPos, PlanRequest
import sys
import json
import time
    
def makePath(path, publisher):
	print str(p[0]) + ' ' + str(p[1]) + ' ' + str(p[2])
	publisher.Publsih

    publisher.publish([ArmPos(x=p[0], y=p[1], z=p[2]) for p in path])

def execute(self):
    rospy.spin()

if __name__ == "__main__":
	rospy.init_node('testing', anonymous=True)
	plan_publisher = rospy.Publisher("plan", Path, queue_size=10)

	makePath([[-3000,0,5500],[0,4000,5500],[3000,0,5500]], plan_publisher)
	
	time.sleep(1)
	execute()
	makePath()