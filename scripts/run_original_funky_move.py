#!/usr/bin/env python
import roslib; 
roslib.load_manifest('dmp')
import rospy 
import numpy as np
from dmp.srv import *
from dmp.msg import *
from dance_bot.msg import Path, ArmPos, PlanRequest
from learn_a_funky_move import LoadedPath
import sys
import json


class OriginalPlanner(object):

    def __init__(self, original_folder):
        rospy.init_node('original_funky_moves')
        self.plan_publisher = rospy.Publisher("plan", Path, queue_size=10)
        self.request_subscriber = rospy.Subscriber("plan_request", PlanRequest, self.make_plan)
        self.original_folder = original_folder

    def make_plan(self, request):
        # load points
        print "loading path"
        path_name = request.dmp_name
        paths = LoadedPath(self.original_folder+"/"+path_name+".csv") #not other OS compatible, but neither is ROS, so whatevs
        
        #convert to path
        path = self.makePath(paths.hand_path)

        # publish plan
        print "publishing"
        self.plan_publisher.publish(path)

    def makePath(self,plan):
        path = [ArmPos(x=p[0], y=p[1], z=p[2]) for p in plan]
        return Path(path=path)

    def execute(self):
        rospy.spin()

if __name__=="__main__":
    if len(sys.argv) == 2:
        original_folder = sys.argv[1]
    else:
        print "usage is rosrun plan_a_funky_move.py original_folder_path"
        print "using /home/rboy/catkin_ws/src/dance_bot as default folder"
        original_folder = "/home/rboy/catkin_ws/src/dance_bot"
    planner = OriginalPlanner(original_folder)
    planner.execute()

    
