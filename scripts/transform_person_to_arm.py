def transform_coords(person_coord, p_rad):
    p_x, p_y, p_z = person_coord
    robot_coord = [coord * 7500.0 / p_rad for coord in [p_y, p_z, p_x]]
    return robot_coord

def transform_path(person_path, p_rad):
    return [transform_coords(coord, p_rad) for coord in person_path]

if __name__ == "__main__":
    p_rad = 1.0

    p_coords = [[0,0,0],
                [1,0,0],
                [0,1,0],
                [0,0,1],
                [0,-1,0],
                [.5,.5,.5]]

    for coord in p_coords:
        print "Person: ", coord
        print "Robot Arm: ", transform_coords(coord,p_rad)
        print "\n"

    print "whole path:", transform_path(p_coords,p_rad)