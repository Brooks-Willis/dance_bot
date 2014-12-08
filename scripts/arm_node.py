#!/usr/bin/env python

from dance_bot.msg import Plan

class ArmCommands:
    def __init__(self):
        self.dmp_plan = rospy.Subscriber("plan", Plan, self.run_arm)
        self.plan = []
        
    def run_arm(self,plan): 
        self.plan = plan
        arm = st.StArm()
        arm.start()
        arm.calibrate()
        arm.cartesian()
        arm.home()
        fixed_output = self.plan_check(plan)
        print "Checked output:", fixed_output
        return None #Used to keep arm from moving during testing
        for coord in fixed_plan:
            arm.move_to(coord[0],coord[1],coord[2])
        self.plan = []

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