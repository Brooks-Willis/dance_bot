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

def makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                seg_length, tau, dt, integrate_iter):
    '''Generate a plan from DMP'''
    print "Starting DMP planning..."
    rospy.wait_for_service('get_dmp_plan')
    try:
        gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
        resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh, 
                   seg_length, tau, dt, integrate_iter)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    print "DMP planning done"   
            
    return resp


def makeSetActiveRequest(dmp_list):
    '''Set a DMP as active for planning'''
    print "Starting to set active DMP..."
    rospy.wait_for_service('set_active_dmp')
    try:
        sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
        sad(dmp_list)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


class LoadedDMP(object):

    def __init__(self,filepath):
        with open(filepath) as dmp_file:
            json_data = dmp_file.read()
        data = json.loads(json_data)
        self.tau = data['tau']
        self.dmp_list = [DMPData(k_gain=dmp['k_gain'],
                                 d_gain=dmp['d_gain'],
                                 weights=dmp['weights'],
                                 f_domain=dmp['f_domain'],
                                 f_targets=dmp['f_targets']) for dmp in data['dmp_list']]


class DMPPlanner(object):

    def __init__(self, dmp_folder):
        rospy.init_node('planning_funky_move')
        self.plan_publisher = rospy.Publisher("plan", Path, queue_size=10)
        self.request_subscriber = rospy.Subscriber("plan_request", PlanRequest, self.make_plan)
        self.dmp_folder = dmp_folder

    def make_plan(self, request):
        # load DMP
        print "loading dmp"
        dmp_name = request.dmp_name
        try:
            dmps = LoadedDMP(self.dmp_folder+"/"+dmp_name+".json") #not other OS compatible, but neither is ROS, so whatevs
        except:
            return
        
        # SetActiveDMP
        print "setting active dmp"
        makeSetActiveRequest(dmps.dmp_list)

        # makePlanRequest
        print "planning"
        x_0 = request.start_pos
        x_dot_0 = request.start_vel
        goal = request.goal
        t_0 = 0
        seg_length = -1
        goal_thresh = [1,1,1]
        tau = dmps.tau
        integrate_iter = 5
        dt = .3
        plan = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                           seg_length, tau, dt, integrate_iter)

        # transform to path
        path = self.makePath(plan)

        # publish plan
        print "publishing"

        self.plan_publisher.publish(path)

    def makePath(self,plan):
        for point in plan.plan.points:
            point.positions = [int(val) for val in point.positions]

        out = [pnt.positions for pnt in plan.plan.points]
        path = [ArmPos(x=out[i][0], y=out[i][1], z=out[i][2]) for i in range(len(out)) if i%6 ==0]
        return Path(path=path[:13])

    def execute(self):
        rospy.spin()

if __name__=="__main__":
    if len(sys.argv) >= 2 and sys.argv[1][:2] != "__":
        dmp_folder = sys.argv[1]
    else:
        print "usage is rosrun plan_a_funky_move.py dmp_folder_path"
        print "using /home/rboy/catkin_ws/src/dance_bot as default folder"
        dmp_folder = "/home/rboy/catkin_ws/src/dance_bot"
    planner = DMPPlanner(dmp_folder)
    planner.execute()

    
