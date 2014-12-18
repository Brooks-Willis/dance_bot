#!/usr/bin/env python
from dance_bot.msg import Path
import rospy
import math
import st
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import cv2

class ArmCommands:
    def __init__(self):
        self.dmp_plan = rospy.Subscriber("plan", Path, self.build_plot)
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.plan = False
        self.distance = 100
        cv2.namedWindow('UI')
        cv2.createTrackbar('distance', 'UI', self.distance, 400, self.set_target_distance)

    def set_target_distance(self,new_distance):
        """ call back function for the OpenCv Slider to set the target distance """
        self.distance = new_distance

    def build_plot(self,plan):
        self.plan = self.convert_plan_type(plan)
        self.safe_output = self.plan_check()
        self.fixed_output = self.distance_check(self.distance)
        print "Checked output:", self.fixed_output

    def convert_plan_type(self,plan):
        return [[p.x, p.y, p.z] for p in plan.path]

    def norm(self, coord):
        norm_val = math.sqrt(coord[0]**2+coord[1]**2+coord[2]**2)
        #print "Norm:", norm_val
        return norm_val

    def safe_dist(self,new_point, prior_point, distance):
        dx = abs(new_point[0]-prior_point[0])
        dy = abs(new_point[1]-prior_point[1])
        dz = abs(new_point[2]-prior_point[2])
        variance = [dx, dy, dz]
        print "variance:", variance
        if np.linalg.norm(variance) > distance:
            return True
        else:
            return False

    def distance_check(self,distance):
        spaced_points = [self.safe_output[0]]
        for point in self.safe_output[1:]:
            if self.safe_dist(point, spaced_points[-1], distance):
                spaced_points.append(point)
        return spaced_points

    def plan_check(self):
        new_plan = []
        for coord in self.plan:
            if coord[2] < 0:
                print "Z target less than zero (%d), recasting to zero" %coord[2]
                coord[2] = 0
            
            if self.norm(coord) < 3000:
                error = 3000/self.norm(coord)
                for i in range(len(coord)):
                    coord[i]=int(math.ceil(coord[i]*error))
                print "Too close - new coord:", coord
            
            if self.norm(coord) > 7500:
                error = 7500/self.norm(coord)
                for i in range(len(coord)):
                    coord[i]=math.trunc(coord[i]*error)
                print "Too far - new coord:", coord
            new_plan.append(coord)
        return new_plan    

    def execute(self):
        r = rospy.Rate(1)
        while not(rospy.is_shutdown()):
            if self.plan:
                [xs, ys, zs]=[list(b) for b in zip(*self.fixed_output)]
                self.plan = []
                length = len(xs)
                self.ax.scatter(xs[:length], ys[:length], zs[:length], c=np.arange(length))
                plt.show()
                self.fig = plt.figure()
                self.ax = self.fig.add_subplot(111, projection='3d')
            cv2.waitKey(10)
            r.sleep()

if __name__ == "__main__":
    rospy.init_node('robot_arm', anonymous=True)
    object_tracker = ArmCommands()
    object_tracker.execute()
