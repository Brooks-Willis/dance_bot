#!/usr/bin/env python
from dance_bot.msg import Path
import rospy
import math
import st
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class ArmCommands:
    def __init__(self):
        self.dmp_plan = rospy.Subscriber("plan", Path, self.build_plot)
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

    def build_plot(self,plan): 
        self.plan = self.convert_plan_type(plan)
        fixed_output = self.plan_check()
        print "Checked output:", fixed_output
        [xs, ys, zs]=[list(b) for b in zip(*a)]
        Axes3D.scatter(xs, ys, zs)
        self.plan = []

    def convert_plan_type(self,plan):
        return [[p.x, p.y, p.z] for p in plan.path]

    def norm(self, coord):
        norm_val = math.sqrt(coord[0]**2+coord[1]**2+coord[2]**2)
        #print "Norm:", norm_val
        return norm_val

    def plan_check(self):
        new_plan = []
        for coord in self.plan:
            if coord[2] < 0:
                print "Z target less than zero (%d), recasting to zero" %coord[2]
                coord[2] = 0
            
            if self.norm(coord) < 3000:
                error = 3000/norm(coord)
                for i in range(len(coord)):
                    coord[i]=int(math.ceil(coord[i]*error))
                print "Too close - new coord:", coord
            
            if self.norm(coord) > 7500:
                error = 7500/norm(coord)
                for i in range(len(coord)):
                    coord[i]=math.trunc(coord[i]*error)
                print "Too far - new coord:", coord
            new_plan.append(coord)
        return new_plan    

if __name__ == "__main__":
    rospy.init_node('robot_arm', anonymous=True)
    object_tracker = ArmCommands()
    rospy.spin()
