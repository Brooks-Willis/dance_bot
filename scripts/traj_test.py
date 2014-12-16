import st
import math

def norm(coord):
    norm_val = math.sqrt(coord[0]**2+coord[1]**2+coord[2]**2)
    print "Norm:", norm_val
    return norm_val

def plan_check(plan):
    new_plan = []
    for coord in plan:
        if coord[2] < 0:
            print "Z target less than zero (%d), recasting to zero" %coord[2]
            coord[2] = 0
        
        if norm(coord) < 3000:
            error = 3000/norm(coord)
            for i in range(len(coord)):
                coord[i]=int(math.ceil(coord[i]*error))
            print "Too close - new coord:", coord
        
        if norm(coord) > 7500:
            error = 7500/norm(coord)
            for i in range(len(coord)):
                coord[i]=math.trunc(coord[i]*error)
            print "Too far - new coord:", coord
        new_plan.append(coord)
    return new_plan

if __name__ == '__main__':

    plan = [[1,1,1],[1,1,-1],[7500,7500,7500]]
    """[[0, 0, 7500],[0, 750, 6750],[0, 1500, 6000],[0, 2250, 5250],
            [0, 3000, 4500],[0, 3750, 3750],[0, 4500, 3000],[0, 5250, 2250],
            [0, 6000, 1500],[0, 6750, 750],[0,7500,0]]"""

    print plan_check(plan)


"""Total distance less than 3000 and z less than 0 are dangerous
Total distance greater than 7500 won't run"""
