#!/usr/bin/env python

from dance_bot.msg import Path
import rospy
import math
import st
import numpy as np
import cv2

class ArmCommands(object):
    def __init__(self):
        self.dmp_plan = rospy.Subscriber("plan", Path, self.run_arm)
        self.plan = []
        self.distance = 0.0
        try:
            self.arm = st.StArm()
            self.arm.start()
            self.arm.calibrate()
            self.arm.home()
        except:
            print "Arm not connected"
        self.distance=100
        cv2.namedWindow('UI')
        cv2.createTrackbar('distance', 'UI', self.distance, 400, self.set_target_distance)

    def set_target_distance(self,new_distance):
        """ call back function for the OpenCv Slider to set the target distance """
        self.distance = new_distance

    def run_arm(self,plan):
        self.plan = self.convert_plan_type(plan)
        self.safe_output = self.plan_check()
        fixed_output = self.distance_check(self.distance)
        print "Checked output:", fixed_output
        for coord in fixed_output:
            try:
                self.arm.continuous()
                self.arm.create_route("TEST1",fixed_output)
                # self.arm.create_route("TEST1",[[-2992, 0, 5500], [-2000,1800,5500]])
                self.arm.run_route("TEST1")
            except:
                pass
        self.plan = []

    def convert_plan_type(self,plan):
        return [[p.x, p.y, p.z] for p in plan.path]

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
            
            if np.linalg.norm(coord) < 3000:
                error = 3000/np.linalg.norm(coord)
                for i in range(len(coord)):
                    coord[i]=int(math.ceil(coord[i]*error))
                print "Too close - new coord:", coord
            
            if np.linalg.norm(coord) > 7500:
                error = 7500/np.linalg.norm(coord)
                for i in range(len(coord)):
                    coord[i]=math.trunc(coord[i]*error)
                print "Too far - new coord:", coord
            new_plan.append(coord)
        return new_plan    

if __name__ == "__main__":
    rospy.init_node('robot_arm', anonymous=True)
    object_tracker = ArmCommands()
    r = rospy.Rate(10)
    while not(rospy.is_shutdown()):
        #if distance_to_wall != -1:
        cv2.waitKey(10)
        r.sleep()
