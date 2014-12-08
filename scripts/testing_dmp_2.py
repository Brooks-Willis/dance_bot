#!/usr/bin/env python
import roslib; 
roslib.load_manifest('dmp')
import rospy 
import numpy as np
from dmp.srv import *
from dmp.msg import *
import st

#Learn a DMP from demonstration data
def makeLFDRequest(dims, traj, dt, K_gain, 
                   D_gain, num_bases):
    demotraj = DMPTraj()
        
    for i in range(len(traj)):
        pt = DMPPoint();
        pt.positions = traj[i]
        demotraj.points.append(pt)
        demotraj.times.append(dt*i)
            
    k_gains = [K_gain]*dims
    d_gains = [D_gain]*dims
        
    print "Starting LfD..."
    rospy.wait_for_service('learn_dmp_from_demo')
    try:
        lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
        resp = lfd(demotraj, k_gains, d_gains, num_bases)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    print "LfD done"    
            
    return resp;


#Set a DMP as active for planning
def makeSetActiveRequest(dmp_list):
    try:
        sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
        sad(dmp_list)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


#Generate a plan from a DMP
def makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                    seg_length, tau, dt, integrate_iter):
    print "Starting DMP planning..."
    rospy.wait_for_service('get_dmp_plan')
    try:
        gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
        resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh, 
                   seg_length, tau, dt, integrate_iter)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    print "DMP planning done"   
            
    return resp;

def run_arm(plan):
    arm = st.StArm()
    arm.start()
    arm.calibrate()
    arm.cartesian()
    arm.home()

    for coord in plan:
        arm.move_to(coord[0],coord[1],coord[2])

def make_line(n_points):
    traj = [[0,0,7500]]
    for i in range(1,n_points+1):
        scale = i/float(n_points)
        traj.append([0,7500*scale,7500-(7500*scale)])
    return traj

def make_triangle(n_points):
    traj = []
    for i in range(int(n_points/2),n_points+1):
        scale = i/float(n_points)
        traj.append([-3000*scale, -3000+(3000*scale), 5500])
    for i in range(1,n_points+1):
        scale = i/float(n_points)
        traj.append([-3000+(3000*scale), 3000*scale, 5500])
    for i in range(1,n_points+1):
        scale = i/float(n_points)
        traj.append([3000*scale, 3000-(3000*scale), 5500])
    for i in range(1,n_points+1):
        scale = i/float(n_points)
        traj.append([3000*scale, 3000-(3000*scale), 5500])
    for i in range(1,int(n_points/2)):
        scale = i/float(n_points)
        traj.append([3000-(3000*scale), 3000*scale, 5500])
    return traj


if __name__ == '__main__':
    rospy.init_node('dmp_tutorial_node')

    #Create a DMP from a 2-D trajectory
    dims = 3                
    dt = 0.2               
    K = 100                 
    D = 2.0 * np.sqrt(K)      
    num_bases = 4          

    n_points = 10
    traj = make_triangle(n_points)
    

    resp = makeLFDRequest(dims, traj, dt, K, D, num_bases)

    #Set it as the active DMP
    makeSetActiveRequest(resp.dmp_list)

    #Now, generate a plan
    x_0 = [0.0, 0.0, 7500.0]          #Plan starting at a different point than demo 
    x_dot_0 = [0.0,0.0,0.0]   
    t_0 = 0                
    goal = [0.0, 7500.0, 0.0]         #Plan to a different goal than demo
    goal_thresh = [0.2,0.2,0.2]
    seg_length = -1          #Plan until convergence to goal
    tau = resp.tau       #Desired plan should take twice as long as dsemo
    dt = 1.0
    integrate_iter = 5       #dt is rather large, so this is > 1  
    plan = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                           seg_length, tau, dt, integrate_iter)

    for point in plan.plan.points:
        point.positions = [int(val) for val in point.positions]

    out = [pnt.positions for pnt in plan.plan.points]
    print out
    run_arm(out)
