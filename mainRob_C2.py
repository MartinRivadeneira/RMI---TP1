
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
import math

CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.lap_time = 0
        self.readSensors()

        global GPS_x_initial
        global GPS_y_initial
        global MAP
        global MAP_x_current
        global MAP_y_current

        GPS_x_initial = self.measures.x
        GPS_y_initial = self.measures.y

        MAP = [[0 for x in range(54)] for y in range(26)]
        print(MAP)

        MAP_x_current = []
        MAP_y_current = []


    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

        while True:
            self.readSensors()

            self.measures.gpsReady = True
            self.measures.gpsDirReady = True

            if self.measures.endLed:
                print(self.rob_name + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.ground==0:
                    self.setVisitingLed(True);
                self.wander()
            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                self.wander()

    def wander(self):

        # ------------------------------------------------------
        # VARIABLES
        # ------------------------------------------------------
        global GPS_x_initial
        global GPS_y_initial
        global MAP
        global MAP_x_current
        global MAP_y_current

        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3

        self.readSensors()

        sen_center = self.measures.irSensor[center_id]
        sen_left = self.measures.irSensor[left_id]
        sen_right = self.measures.irSensor[right_id]
        sen_back = self.measures.irSensor[back_id]

        compass = self.measures.compass

        GPS_x = self.measures.x - GPS_x_initial
        GPS_y = self.measures.y - GPS_y_initial
        GPS_dir = self.measures.dir

        # Movements
        GPS_x_current_vet = []
        for i in range(-26, 28, 2):
            GPS_x_current_vet.append(i)
        GPS_x_current = GPS_x_current_vet[closest(GPS_x_current_vet, GPS_x)]

        GPS_y_current_vet = []
        for j in range(-12, 14, 2):
            GPS_y_current_vet.append(j)
        GPS_y_current = GPS_y_current_vet[closest(GPS_y_current_vet, GPS_y)]

        compass_current_vet = [0, 90, -180, -90]
        compass_current = compass_current_vet[closest(compass_current_vet, compass)]

        # Initial position in PAM
        MAP_x_initial = 27
        MAP_y_initial = 13

        MAP_x_current = MAP_x_initial + GPS_x_current
        MAP_y_current = MAP_y_initial - GPS_y_current

        MAP[MAP_y_initial][MAP_x_initial] = 7

        # if sen_back <= 1.8:
        #     MAP[MAP_y_initial][MAP_x_initial - 2] = 5
        #     MAP[MAP_y_initial][MAP_x_initial - 1] = 1
        # else:
        #     MAP[MAP_y_initial][MAP_x_initial - 1] = 2

        if MAP_x_current == []:
            MAP_x_current = MAP_x_initial
        if MAP_y_current == []:
            MAP_y_current = MAP_y_initial

        min_side = min(sen_left, sen_right)


        # ------------------------------------------------------
        # ZONES
        # ------------------------------------------------------

        print('NEW MOVEMENT!')

        Z = [False, False, False, False]

        if abs(compass) <= 45:
            Z[0] = True
            Z[1] = False
            Z[2] = False
            Z[3] = False

        if compass > 45 and compass <= 135:
            Z[0] = False
            Z[1] = True
            Z[2] = False
            Z[3] = False

        if abs(compass) >= 135:
            Z[0] = False
            Z[1] = False
            Z[2] = True
            Z[3] = False

        if compass >= -135 and compass <= -45:
            Z[0] = False
            Z[1] = False
            Z[2] = False
            Z[3] = True

        threshold = 1.2

        MAP[MAP_y_current][MAP_x_current] = 7

        if Z[0] == True:

            print('Im mapping in Zone 0')

            if sen_center >= threshold:
                MAP[MAP_y_current][MAP_x_current+1] = 2
            else:
                MAP[MAP_y_current][MAP_x_current + 1] = 1
                if MAP[MAP_y_current][MAP_x_current + 2] != 7:
                    MAP[MAP_y_current][MAP_x_current + 2] = 5

            if sen_left >= threshold:
                MAP[MAP_y_current - 1][MAP_x_current] = 3
            else:
                MAP[MAP_y_current - 1][MAP_x_current] = 1
                if MAP[MAP_y_current - 2][MAP_x_current] != 7:
                    MAP[MAP_y_current - 2][MAP_x_current] = 5

            if sen_right >= threshold:
                MAP[MAP_y_current + 1][MAP_x_current] = 3
            else:
                MAP[MAP_y_current + 1] [MAP_x_current] = 1
                if MAP[MAP_y_current + 2][MAP_x_current] != 7:
                    MAP[MAP_y_current + 2][MAP_x_current] = 5

        if Z[1] == True:

            print('Im mapping in Zone 1')

            if sen_center >= threshold:
                MAP[MAP_y_current - 1][MAP_x_current] = 3
            else:
                MAP[MAP_y_current - 1][MAP_x_current] = 1
                if MAP[MAP_y_current - 2][MAP_x_current] != 7:
                    MAP[MAP_y_current - 2][MAP_x_current] = 5

            if sen_left >= threshold:
                MAP[MAP_y_current][MAP_x_current - 1] = 2
            else:
                MAP[MAP_y_current][MAP_x_current - 1] = 1
                if MAP[MAP_y_current][MAP_x_current - 2] != 7:
                    MAP[MAP_y_current][MAP_x_current - -2] = 5

            if sen_right >= threshold:
                MAP[MAP_y_current][MAP_x_current + 1] = 2
            else:
                MAP[MAP_y_current][MAP_x_current + 1] = 1
                if MAP[MAP_y_current][MAP_x_current + 2] != 7:
                    MAP[MAP_y_current][MAP_x_current + 2] = 5

        if Z[2] == True:

            print('Im mapping in Zone 2')

            if sen_center >= threshold:
                MAP[MAP_y_current][MAP_x_current - 1] = 2
            else:
                MAP[MAP_y_current][MAP_x_current - 1] = 1
                if MAP[MAP_y_current][MAP_x_current - 2] != 7:
                    MAP[MAP_y_current][MAP_x_current - 2] = 5

            if sen_left >= threshold:
                MAP[MAP_y_current + 1][MAP_x_current] = 3
            else:
                MAP[MAP_y_current + 1][MAP_x_current] = 1
                if MAP[MAP_y_current + 2][MAP_x_current] != 7:
                    MAP[MAP_y_current + 2][MAP_x_current] = 5
            if sen_right >= threshold:
                MAP[MAP_y_current - 1][MAP_x_current] = 3
            else:
                MAP[MAP_y_current - 1][MAP_x_current] = 1
                if MAP[MAP_y_current - 2][MAP_x_current] != 7:
                    MAP[MAP_y_current - 2][MAP_x_current] = 5

        if Z[3] == True:

            print('Im mapping in Zone 3')

            if sen_center >= threshold:
                MAP[MAP_y_current + 1][MAP_x_current] = 3
            else:
                MAP[MAP_y_current + 1][MAP_x_current] = 1
                if MAP[MAP_y_current + 2][MAP_x_current] != 7:
                    MAP[MAP_y_current + 2][MAP_x_current] = 5

            if sen_left >= threshold:
                MAP[MAP_y_current][MAP_x_current + 1] = 2
            else:
                MAP[MAP_y_current][MAP_x_current + 1] = 1
                if MAP[MAP_y_current][MAP_x_current + 2] != 7:
                    MAP[MAP_y_current][MAP_x_current + 2] = 5

            if sen_right >= threshold:
                MAP[MAP_y_current][MAP_x_current - 1] = 2
            else:
                MAP[MAP_y_current][MAP_x_current - 1] = 1
                if MAP[MAP_y_current][MAP_x_current - 2] != 7:
                    MAP[MAP_y_current][MAP_x_current - 2] = 5

        print(MAP)

        # ------------------------------------------------------
        # MOVEMENT
        # ------------------------------------------------------

        # Errors
        error_rot = 20
        error_x = 10
        error_y = 10

        sen_center_limit = 1.8


        # Go ahead
        if sen_center <= sen_center_limit:

            if Z[0] == True:

                while error_x > 0.1:

                    if sen_center >= sen_center_limit:
                        break

                    self.readSensors()
                    GPS_x = self.measures.x - GPS_x_initial
                    GPS_y = self.measures.y - GPS_y_initial
                    compass = self.measures.compass
                    sen_center = self.measures.irSensor[center_id]

                    error_x = (GPS_x_current + 2) - GPS_x
                    error_y = (GPS_y_current) - GPS_y

                    kx = 0.075
                    ky = 0.025

                    lin = error_x * kx
                    rot = error_y * ky

                    R = lin + rot
                    L = lin - rot
                    self.driveMotors(L, R)

                    error_lin = sqrt(error_x * error_x + error_y * error_y)

                    print('Moving in X\n')
                    print('Sensor Center = ' + str(sen_center))
                    print('Compass = ' + str(compass))
                    print('GPS = [' + str(GPS_x) + ', ' + str(GPS_y) + ', ' + str(GPS_dir) +']')
                    print('Map localization = [' + str(MAP_x_current) + ', ' + str(MAP_y_current) + ']')
                    print('error = ' + str(error_lin))
                    print('L = ' + str(L) + ', R = ' + str(R))

            if Z[1] == True:

                while error_y > 0.1:

                    if sen_center >= sen_center_limit:
                        break

                    self.readSensors()
                    GPS_x = self.measures.x - GPS_x_initial
                    GPS_y = self.measures.y - GPS_y_initial
                    compass = self.measures.compass
                    sen_center = self.measures.irSensor[center_id]

                    error_x = (GPS_x_current) - GPS_x
                    error_y = (GPS_y_current + 2) - GPS_y

                    kx = 0.025
                    ky = 0.075

                    rot = error_x * kx
                    lin = error_y * ky

                    R = lin - rot
                    L = lin + rot
                    self.driveMotors(L, R)

                    error_lin = sqrt(error_x * error_x + error_y * error_y)

                    print('Moving in Y\n')
                    print('Sensor Center = ' + str(sen_center))
                    print('Compass = ' + str(compass))
                    print('GPS = [' + str(GPS_x) + ', ' + str(GPS_y) + ', ' + str(GPS_dir) + ']')
                    print('Map localization = [' + str(MAP_x_current) + ', ' + str(MAP_y_current) + ']')
                    print('error = ' + str(error_lin))
                    print('L = ' + str(L) + ', R = ' + str(R))

            if Z[2] == True:

                while error_x > 0.1:

                    if sen_center >= sen_center_limit:
                        break

                    self.readSensors()
                    GPS_x = self.measures.x - GPS_x_initial
                    GPS_y = self.measures.y - GPS_y_initial
                    compass = self.measures.compass
                    sen_center = self.measures.irSensor[center_id]

                    error_x = GPS_x - (GPS_x_current - 2)
                    error_y = (GPS_y_current) - GPS_y

                    kx = 0.075
                    ky = 0.025

                    lin = error_x * kx
                    rot = error_y * ky

                    R = lin + rot
                    L = lin - rot
                    self.driveMotors(L, R)

                    error_lin = sqrt(error_x * error_x + error_y * error_y)

                    print('Moving in -X\n')
                    print('Sensor Center = ' + str(sen_center))
                    print('Compass = ' + str(compass))
                    print('GPS = [' + str(GPS_x) + ', ' + str(GPS_y) + ', ' + str(GPS_dir) + ']')
                    print('Map localization = [' + str(MAP_x_current) + ', ' + str(MAP_y_current) + ']')
                    print('error = ' + str(error_lin))
                    print('L = ' + str(L) + ', R = ' + str(R))

            if Z[3] == True:

                while error_y > 0.1:

                    if sen_center >= sen_center_limit:
                        break

                    self.readSensors()
                    GPS_x = self.measures.x - GPS_x_initial
                    GPS_y = self.measures.y - GPS_y_initial
                    compass = self.measures.compass
                    sen_center = self.measures.irSensor[center_id]

                    error_x = GPS_x - (GPS_x_current)
                    error_y = GPS_y - (GPS_y_current - 2)

                    kx = 0.025
                    ky = 0.075

                    rot = error_x * kx
                    lin = error_y * ky

                    R = lin - rot
                    L = lin + rot
                    self.driveMotors(L, R)

                    error_lin = sqrt(error_x*error_x + error_y*error_y)

                    print('Moving in -Y\n')
                    print('Sensor Center = ' + str(sen_center))
                    print('Compass = ' + str(compass))
                    print('GPS = [' + str(GPS_x) + ', ' + str(GPS_y) + ', ' + str(GPS_dir) + ']')
                    print('Map localization = [' + str(MAP_x_current) + ', ' + str(MAP_y_current) + ']')
                    print('error = ' + str(error_lin))
                    print('L = ' + str(L) + ', R = ' + str(R))

        # Rotate
        else:

            # Rotate Left
            if min_side == sen_left:

                while abs(error_rot) >= 1:

                    self.readSensors()
                    compass = self.measures.compass

                    setpoint = compass_current + 90

                    error_ang = (setpoint) - compass

                    if error_ang > 100:
                        error_ang = error_ang - 360

                    k_ang = 0.005

                    lin = 0
                    rot = error_ang * k_ang

                    R = lin + rot
                    L = lin - rot
                    self.driveMotors(L, R)

                    error_rot = error_ang

                    print('Compass = ' + str(compass))
                    print('L = ' + str(L) + ', R = ' + str(R))
                    print('error = ' + str(error_ang))

            # Rotate Right
            else:

                while abs(error_rot) >= 1:
                    self.readSensors()
                    compass = self.measures.compass

                    setpoint = compass_current - 90

                    error_ang = setpoint - compass

                    if error_ang < -100:
                        error_ang = error_ang + 360

                    k_ang = 0.005

                    lin = 0
                    rot = error_ang * k_ang

                    R = lin + rot
                    L = lin - rot
                    self.driveMotors(L, R)

                    error_rot = error_ang

                    print('Compass = ' + str(compass))
                    print('L = ' + str(L) + ', R = ' + str(R))
                    print('error = ' + str(error_ang))


        # Turn off
        R = 0
        L = 0
        self.driveMotors(L, R)


class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

def closest(list, Number):
    aux = []
    for valor in list:
        aux.append(abs(Number - valor))

    return aux.index(min(aux))

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,180.0],host)

    # Variables
    lap = []
    lap_time_vet = [0]
    sen_ground_vet = []

    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
