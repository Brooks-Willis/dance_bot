PURGE = 'PURGE'
ROBOFORTH = 'ROBOFORTH'
DECIMAL = 'DECIMAL'
START = 'START'
JOINT = 'JOINT'
CALIBRATE = 'CALIBRATE'
HOME = 'HOME'
WHERE = 'WHERE'
CARTESIAN = 'CARTESIAN'
SPEED = 'SPEED'
ACCEL = 'ACCEL'
MOVETO = 'MOVETO'
HAND = 'HAND'
WRIST = 'WRIST'
ENERGIZE = 'ENERGIZE'
DE_ENERGIZE = 'DE-ENERGIZE'
QUERY = ' ?'
IMPERATIVE = ' !'
TELL = 'TELL'
MOVE = 'MOVE'
ALIGN = 'ALIGN'
NONALIGN = 'NONALIGN'
CONTINUOUS = 'CONTINUOUS'
RUN = 'RUN'
STARTOVER = 'STARTOVER'
RESERVE = 'RESERVE'
ROUTE = 'ROUTE'
LEARN = 'LEARN'
START_HERE = 'START_HERE'
SMOOTH = 'SMOOTH'
ADJUST = 'ADJUST'
NEW = 'NEW'
INSERT = 'INSERT'

OK = 'OK'


def create_route(route_name, commands):
        # commands should be a list [[x,y,z],[x,y,z],...]
        cmd = '  ' + CARTESIAN + ' ' + NEW + ' ' + ROUTE + ' ' + route_name

        print "Creating route " + route_name
        cmd += str(len(commands)) + ' ' + RESERVE + ' ' + route_name
        cmd += route_name + ' ' + LEARN + ' ' + DECIMAL + ' CF'
        print cmd

        for x in range(1,len(commands)):
            cmd = route_name + ' 1 INSERT DECIMAL CF'
            print cmd

        index = 0
        for point in commands:
            index += 1
            point = str(point[0]) + ' ' + str(point[1]) + ' ' + str(point[2])

            cmd = DECIMAL + " 0 0 900 " + point +  ' ' + route_name + ' ' + str(index) + ' LINE DLD' 
            print cmd


print create_route('hello', [[1,2,3],[4,5,6],[7,8,9]])


