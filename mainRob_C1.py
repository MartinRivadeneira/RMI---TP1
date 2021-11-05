
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

        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3

        sen_center = self.measures.irSensor[center_id]
        sen_left = self.measures.irSensor[left_id]
        sen_right = self.measures.irSensor[right_id]
        sen_back = self.measures.irSensor[back_id]
        sen_ground = self.measures.ground
        moving_time = self.measures.time

        min_side = min(sen_left, sen_right)
        lin = 0
        rot = 0

        sen_ground_vet.append(sen_ground)

        if 2 in sen_ground_vet:
            third_base = sen_ground_vet.index(2)
        else:
            third_base = 0
        if 1 in sen_ground_vet:
            second_base = sen_ground_vet.index(1)
        else:
            second_base = 0
        if 0 in sen_ground_vet:
            first_base = max([i for i, x in enumerate(sen_ground_vet) if x == 0])
        else:
            first_base = 0

        if third_base > second_base and first_base > third_base:
            lap_time_vet.append(moving_time-sum(lap_time_vet))
            lap.append(1)
            sen_ground_vet.clear()

        try:
            lap_average = round(sum(lap_time_vet)/(len(lap_time_vet)-1), 2)
        except:
            lap_average = 0

        print('\n\nNew movement:\n')
        print('Center = ' + str(sen_center))
        print('Left = ' + str(sen_left))
        print('Right = ' + str(sen_right))
        print('Back = ' + str(sen_back))
        print('Ground = ' + str(sen_ground))

        print('LAP = ' + str(len(lap)))
        # print('LAP Time = ' + str(self.lap_time))
        print('LAP Time = ' + str(lap_time_vet[-1]))
        print('LAP Time Average = ' + str(lap_average))

        if sen_center <= 1.1:
            print('Move Forward')
            lin = 0.15

        else:

            if min_side == sen_right:

                print('Rotate min right')
                rot = 0.15
            else:

                print('Rotate min left')
                rot = -0.15

        self.driveMotors(lin + rot, lin - rot)

        limit = 7.5

        if sen_left >= limit:
            print('Rotate right!')
            self.driveMotors(0.15, -0.15)

        if sen_right >= limit:
            print('Rotate left!')
            self.driveMotors(-0.15, 0.15)


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

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host)

    # Variables
    lap = []
    lap_time_vet = [0]
    sen_ground_vet = []

    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
