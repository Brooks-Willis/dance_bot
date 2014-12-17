#!/usr/bin/env python
import roslib; 
roslib.load_manifest('dmp')
import rospy 
import numpy as np
from dmp.srv import *
from dmp.msg import *
import sys
import pandas
import json

class LoadedPath(object):

    def __init__(self, filepath):
        data = pandas.read_csv(filepath)
        self.hand_path = zip(data['hand_x'],data['hand_y'],data['hand_z'])
        self.elbow_path = zip(data['elbow_x'],data['elbow_y'],data['elbow_z'])
        self.times = list(data['times'].values)

#Learn a DMP from demonstration data
def makeLFDRequest(dims, traj, times, K_gain, 
                   D_gain, num_bases):
    demotraj = DMPTraj()
        
    for i in range(len(traj)):
        pt = DMPPoint();
        pt.positions = traj[i]
        demotraj.points.append(pt)
        demotraj.times.append(times[i])
            
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

def save_dmps(dmp_list, filepath):
    output_dmps = [{'k_gain': dmp.k_gain,
               'd_gain': dmp.d_gain,
               'weights': dmp.weights,
               'f_domain': dmp.f_domain,
               'f_targets': dmp.f_targets} for dmp in dmp_list.dmp_list]
    output = {'dmp_list': output_dmps, 'tau': dmp_list.tau}
    output_string = json.dumps(output)
    with open(filepath,"w") as dmp_file:
        dmp_file.write(output_string)


if __name__=="__main__":
    # get example path
    if len(sys.argv) == 2:
        filepath = sys.argv[1]
    else:
        print "Usage is rosrun dance_bot learn_dmp.py [filepath.csv]"
        sys.exit(0)
    path = LoadedPath(filepath)
    dims = 3
    K = 100
    D = 2.0 * np.sqrt(K)
    num_bases = 50

    # LearnDMPFromDemo
    resp = makeLFDRequest(dims, path.hand_path, path.times, K, D, num_bases)

    # Store DMP
    save_dmps(resp,filepath[:-4]+".json")
