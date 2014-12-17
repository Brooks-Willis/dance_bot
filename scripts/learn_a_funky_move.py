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
    if len(sys.argv) >= 2 and sys.argv[1][:2] != "__":
        folder_path = sys.argv[1]
    else:
        print sys.argv
        print "Usage is rosrun dance_bot learn_a_funky_move.py folder_path"
        sys.exit(0)

    while not(rospy.is_shutdown()):
        try:
            params = input("enter: (name without file extension, K, num_bases, optionally output name)\n")
        except:
            e = sys.exc_info()
            print e
            continue
        if len(params) == 3:
            name, K, num_bases = params
            output_name = name
        elif len(params) == 4:
            name, K, num_bases, output_name = params
        else:
            print "invalid input length, sorry"
            continue

        if type(name) != str:
            print "name must be a string"
            continue
        if type(K) != int:
            print "K must be an integer"
            continue
        if type(num_bases) != int or num_bases < 1 or num_bases > 100:
            print "num_bases must be an integer between 1 and 100"
            continue

        inputpath = folder_path+"/"+name+".csv"
        try:
            path = LoadedPath(inputpath)
            print "read data from file", inputpath
        except:
            print "error loading file", inputpath
            e = sys.exc_info()[0]
            print e
            continue

        dims = 3
        D = 2.0 * np.sqrt(K)

        # LearnDMPFromDemo
        resp = makeLFDRequest(dims, path.hand_path, path.times, K, D, num_bases)

        # Store DMP
        output_file = folder_path+"/"+output_name+".json"
        try:
            save_dmps(resp,output_file)
            print "saved DMP to file", output_file
        except:
            print "error saving data to file", output_file
            e = sys.exc_info()[0]
            print e
