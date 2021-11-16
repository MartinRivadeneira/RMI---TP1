import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
import math
import numpy as np
np.set_printoptions(threshold=np.inf)

CELLROWS = 7
CELLCOLS = 14


class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.lap_time = 0
        self.readSensors()

        global GPS_x_initial
        global GPS_y_initial
        global GPS_x_current
        global GPS_y_current
        global MAP
        global MAP_x_current
        global MAP_y_current
        global MAP_x_initial
        global MAP_y_initial

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
                if self.measures.visitingLed == True:
                    state = 'wait'
                if self.measures.ground == 0:
                    self.setVisitingLed(True);
                self.wander()
            elif state == 'wait':
                self.setReturningLed(True)
                if self.measures.visitingLed == True:
                    self.setVisitingLed(False)
                if self.measures.returningLed == True:
                    state = 'return'
                self.driveMotors(0.0, 0.0)
            elif state == 'return':
                if self.measures.visitingLed == True:
                    self.setVisitingLed(False)
                if self.measures.returningLed == True:
                    self.setReturningLed(False)
                self.wander()

    def DefineZone(self):

        self.readSensors()
        compass = self.measures.compass

        Z = [False, False, False, False]

        # Define a zone depending on compass orientation
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

        return Z

    def Mapping(self, Z, MAP_y_current, MAP_x_current):

        center_id = 0
        left_id = 1
        right_id = 2

        self.readSensors()

        sen_center = self.measures.irSensor[center_id]
        sen_left = self.measures.irSensor[left_id]
        sen_right = self.measures.irSensor[right_id]

        # Update contour cells depending on sensors readings and actual zone.
        # 0 = Unknown
        # 1 = Empty
        # 2 = Vertical wall
        # 3 = Horizontal wall
        # 4 = Initial position
        # 5 = Non explored, but possible to explore cell
        # 7 = Explored cell
        # 9 = Actual position

        threshold = 1.15

        if Z[0] == True:

            print('Im mapping in Zone 0')
            print('Sensor Center = ' + str(sen_center))
            print('Sensor Right = ' + str(sen_right))
            print('Sensor Left = ' + str(sen_left))

            if sen_center >= threshold:
                MAP[MAP_y_current][MAP_x_current + 1] = 2
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
                MAP[MAP_y_current + 1][MAP_x_current] = 1
                if MAP[MAP_y_current + 2][MAP_x_current] != 7:
                    MAP[MAP_y_current + 2][MAP_x_current] = 5

            # Extract contour values for future decisions
            F = MAP[MAP_y_current][MAP_x_current + 1]
            F2 = MAP[MAP_y_current][MAP_x_current + 2]
            R = MAP[MAP_y_current + 1][MAP_x_current]
            R2 = MAP[MAP_y_current + 2][MAP_x_current]
            L = MAP[MAP_y_current - 1][MAP_x_current]
            L2 = MAP[MAP_y_current - 2][MAP_x_current]

        if Z[1] == True:

            print('Im mapping in Zone 1')
            print('Sensor Center = ' + str(sen_center))
            print('Sensor Right = ' + str(sen_right))
            print('Sensor Left = ' + str(sen_left))

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
                    MAP[MAP_y_current][MAP_x_current - 2] = 5

            if sen_right >= threshold:
                MAP[MAP_y_current][MAP_x_current + 1] = 2
            else:
                MAP[MAP_y_current][MAP_x_current + 1] = 1
                if MAP[MAP_y_current][MAP_x_current + 2] != 7:
                    MAP[MAP_y_current][MAP_x_current + 2] = 5

            F = MAP[MAP_y_current - 1][MAP_x_current]
            F2 = MAP[MAP_y_current - 2][MAP_x_current]
            R = MAP[MAP_y_current][MAP_x_current + 1]
            R2 = MAP[MAP_y_current][MAP_x_current + 2]
            L = MAP[MAP_y_current][MAP_x_current - 1]
            L2 = MAP[MAP_y_current][MAP_x_current - 2]

        if Z[2] == True:

            print('Im mapping in Zone 2')
            print('Sensor Center = ' + str(sen_center))
            print('Sensor Right = ' + str(sen_right))
            print('Sensor Left = ' + str(sen_left))

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

            F = MAP[MAP_y_current][MAP_x_current - 1]
            F2 = MAP[MAP_y_current][MAP_x_current - 2]
            R = MAP[MAP_y_current - 1][MAP_x_current]
            R2 = MAP[MAP_y_current - 2][MAP_x_current]
            L = MAP[MAP_y_current + 1][MAP_x_current]
            L2 = MAP[MAP_y_current + 2][MAP_x_current]

        if Z[3] == True:

            print('Im mapping in Zone 3')
            print('Sensor Center = ' + str(sen_center))
            print('Sensor Right = ' + str(sen_right))
            print('Sensor Left = ' + str(sen_left))

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

            F = MAP[MAP_y_current + 1][MAP_x_current]
            F2 = MAP[MAP_y_current + 2][MAP_x_current]
            R = MAP[MAP_y_current][MAP_x_current - 1]
            R2 = MAP[MAP_y_current][MAP_x_current - 2]
            L = MAP[MAP_y_current][MAP_x_current + 1]
            L2 = MAP[MAP_y_current][MAP_x_current + 2]

        return F, F2, R, R2, L, L2

    def Go_Ahead(self, Z):

        center_id = 0

        self.readSensors()

        sen_center = self.measures.irSensor[center_id]

        GPS_x = self.measures.x - GPS_x_initial
        GPS_y = self.measures.y - GPS_y_initial

        # Define the linear setpoint (exact coords for the next cell given by pair numbers)
        GPS_x_current_vet = []
        for i in range(-26, 28, 2):
            GPS_x_current_vet.append(i)
        GPS_x_current = GPS_x_current_vet[closest(GPS_x_current_vet, GPS_x)]

        GPS_y_current_vet = []
        for j in range(-12, 14, 2):
            GPS_y_current_vet.append(j)
        GPS_y_current = GPS_y_current_vet[closest(GPS_y_current_vet, GPS_y)]

        error_x = 10
        error_y = 10

        # Actuate left and right motors depending on the zone and the error - proportional controlled. Linear velocity is constant while rotational velocity is variant.
        sen_center_limit = 1.8

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

                lin = 0.1

                ky = 0.02
                rot = error_y * ky

                R = lin + rot
                L = lin - rot
                self.driveMotors(L, R)

                error_lin = sqrt(error_x * error_x + error_y * error_y)

                print('\n\nMoving in X:\n')
                print('Sensor Center = ' + str(sen_center))
                print('Compass = ' + str(compass))
                print('GPS = [' + str(GPS_x) + ', ' + str(GPS_y) + ']')
                print('GPS Current = [' + str(GPS_x_current) + ', ' + str(GPS_y_current) + ']')
                print('Map localization = [' + str(MAP_x_current - MAP_x_initial) + ', ' + str(MAP_y_current - MAP_y_initial) + ']')
                print('error in x = ' + str(error_x))
                print('error in y = ' + str(error_y))
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

                lin = 0.1

                kx = 0.02
                rot = error_x * kx

                R = lin - rot
                L = lin + rot
                self.driveMotors(L, R)

                error_lin = sqrt(error_x * error_x + error_y * error_y)

                print('\n\nMoving in Y:\n')
                print('Sensor Center = ' + str(sen_center))
                print('Compass = ' + str(compass))
                print('GPS = [' + str(GPS_x) + ', ' + str(GPS_y) + ']')
                print('GPS Current = [' + str(GPS_x_current) + ', ' + str(GPS_y_current) + ']')
                print('Map localization = [' + str(MAP_x_current - MAP_x_initial) + ', ' + str(MAP_y_current - MAP_y_initial) + ']')
                print('error in x = ' + str(error_x))
                print('error in y = ' + str(error_y))
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
                error_y = GPS_y - (GPS_y_current)

                lin = 0.1

                ky = 0.02
                rot = error_y * ky

                R = lin + rot
                L = lin - rot
                self.driveMotors(L, R)

                error_lin = sqrt(error_x * error_x + error_y * error_y)

                print('\n\nMoving in -X:\n')
                print('Sensor Center = ' + str(sen_center))
                print('Compass = ' + str(compass))
                print('GPS = [' + str(GPS_x) + ', ' + str(GPS_y) + ']')
                print('GPS Current = [' + str(GPS_x_current) + ', ' + str(GPS_y_current) + ']')
                print('Map localization = [' + str(MAP_x_current - MAP_x_initial) + ', ' + str(MAP_y_current - MAP_y_initial) + ']')
                print('error in x = ' + str(error_x))
                print('error in y = ' + str(error_y))
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

                lin = 0.1
                kx = 0.02
                rot = error_x * kx

                R = lin - rot
                L = lin + rot
                self.driveMotors(L, R)

                error_lin = sqrt(error_x * error_x + error_y * error_y)

                print('\n\nMoving in -Y:\n')
                print('Sensor Center = ' + str(sen_center))
                print('Compass = ' + str(compass))
                print('GPS = [' + str(GPS_x) + ', ' + str(GPS_y) + ']')
                print('GPS Current = [' + str(GPS_x_current) + ', ' + str(GPS_y_current) + ']')
                print('Map localization = [' + str(MAP_x_current - MAP_x_initial) + ', ' + str(MAP_y_current - MAP_y_initial) + ']')
                print('error in x = ' + str(error_x))
                print('error in y = ' + str(error_y))
                print('error = ' + str(error_lin))
                print('L = ' + str(L) + ', R = ' + str(R))

    def Rotate_Left(self):

        compass = self.measures.compass

        # Define the rotational setpoint (exact angle for the next movement)
        compass_current_vet = [0, 90, -180, -90, 180]
        compass_current = compass_current_vet[closest(compass_current_vet, compass)]
        if compass_current == 180:
            compass_current = -180

        error_rot = 20

        while abs(error_rot) >= 1:

            self.readSensors()
            compass = self.measures.compass

            setpoint = compass_current + 90

            error_ang = (setpoint) - compass

            if error_ang > 120:
                error_ang = error_ang - 360

            k_ang = 0.005

            lin = 0
            rot = error_ang * k_ang

            R = lin + rot
            L = lin - rot
            self.driveMotors(L, R)

            error_rot = error_ang

            print('\n\nRotating Left!\n')
            print('Compass = ' + str(compass))
            print('L = ' + str(L) + ', R = ' + str(R))
            print('error = ' + str(error_ang))

    def Rotate_Right(self):

        compass = self.measures.compass

        compass_current_vet = [0, 90, -180, -90, 180]
        compass_current = compass_current_vet[closest(compass_current_vet, compass)]
        if compass_current == 180:
            compass_current = -180

        error_rot = 20

        while abs(error_rot) >= 1:
            self.readSensors()
            compass = self.measures.compass

            setpoint = compass_current - 90

            error_ang = setpoint - compass

            if error_ang < -120:
                error_ang = error_ang + 360

            k_ang = 0.005

            lin = 0
            rot = error_ang * k_ang

            R = lin + rot
            L = lin - rot
            self.driveMotors(L, R)

            error_rot = error_ang

            print('\n\nRotating Right!\n')
            print('Compass = ' + str(compass))
            print('L = ' + str(L) + ', R = ' + str(R))
            print('error = ' + str(error_ang))

    def CalculatePath(self, MAP_np, list_non_visited_i, list_non_visited_j, actual_position_i, actual_position_j):

        try:

            # Determinate euclidean distance to the closest non visited cell
            dist = []

            for k in range(0, len(list_non_visited_j)):
                dist.append(math.sqrt(
                    (list_non_visited_j[k] - actual_position_j[0]) ** 2 + (list_non_visited_i[k] - actual_position_i[0]) ** 2))
            print(dist)

            min_dist = min(dist)
            print(min_dist)

            idx = dist.index(min_dist)
            print(idx)

            MAP_for_path = np.array(MAP) * 0
            MAP_for_path[actual_position_j, actual_position_i] = 1
            iteration = 1

            print(MAP_np)

            # Dijkstra's Algorithm
            while MAP_for_path[list_non_visited_j[idx], list_non_visited_i[idx]] == 0:

                list_possivel_go_j, list_possivel_go_i = np.where(MAP_for_path == np.amax(MAP_for_path))
                print('list_possivel_go_j' + str(list_possivel_go_j))
                print('list_possivel_go_i' + str(list_possivel_go_i))
                print('Initial cell value = ' + str(MAP_for_path[actual_position_j, actual_position_i]))
                print('Final cell coords = ' + str(list_non_visited_j[idx]) + ', ' + str(list_non_visited_i[idx]))


                for z in range(0, len(list_possivel_go_j)):

                    if MAP_np[list_possivel_go_j[z], list_possivel_go_i[z]] == 1 or MAP_np[list_possivel_go_j[z], list_possivel_go_i[z]] == 7 or MAP_np[list_possivel_go_j[z], list_possivel_go_i[z]] == 9:
                        if MAP_for_path[list_possivel_go_j[z] + 1, list_possivel_go_i[z]] == 0 and (MAP_np[list_possivel_go_j[z] + 1, list_possivel_go_i[z]] == 7 or MAP_np[list_possivel_go_j[z] + 1, list_possivel_go_i[z]] == 1 or MAP_np[list_possivel_go_j[z] + 1, list_possivel_go_i[z]] == 5):
                            MAP_for_path[list_possivel_go_j[z] + 1, list_possivel_go_i[z]] = MAP_for_path[list_possivel_go_j[z], list_possivel_go_i[z]] + 1
                        if MAP_for_path[list_possivel_go_j[z] - 1, list_possivel_go_i[z]] == 0 and (MAP_np[list_possivel_go_j[z] - 1, list_possivel_go_i[z]] == 7 or MAP_np[list_possivel_go_j[z] - 1, list_possivel_go_i[z]] == 1 or MAP_np[list_possivel_go_j[z] - 1, list_possivel_go_i[z]] == 5):
                            MAP_for_path[list_possivel_go_j[z] - 1, list_possivel_go_i[z]] = MAP_for_path[list_possivel_go_j[z], list_possivel_go_i[z]] + 1
                        if MAP_for_path[list_possivel_go_j[z], list_possivel_go_i[z] + 1] == 0 and (MAP_np[list_possivel_go_j[z], list_possivel_go_i[z] + 1] == 7 or MAP_np[list_possivel_go_j[z], list_possivel_go_i[z] + 1] == 1 or MAP_np[list_possivel_go_j[z], list_possivel_go_i[z] + 1] == 5):
                            MAP_for_path[list_possivel_go_j[z], list_possivel_go_i[z] + 1] = MAP_for_path[list_possivel_go_j[z], list_possivel_go_i[z]] + 1
                        if MAP_for_path[list_possivel_go_j[z], list_possivel_go_i[z] - 1] == 0 and (MAP_np[list_possivel_go_j[z], list_possivel_go_i[z] - 1] == 7 or MAP_np[list_possivel_go_j[z], list_possivel_go_i[z] - 1] == 1 or MAP_np[list_possivel_go_j[z], list_possivel_go_i[z] - 1] == 5):
                            MAP_for_path[list_possivel_go_j[z], list_possivel_go_i[z] - 1] = MAP_for_path[list_possivel_go_j[z], list_possivel_go_i[z]] + 1

                iteration += 1
                if iteration >= 100:
                    break

            print('Final cell value = ' + str(MAP_for_path[list_non_visited_j[idx], list_non_visited_i[idx]]))
            print(MAP_for_path)

            # Create list of movements to get the agent to the wished cell
            list_movents = []
            idx_max = np.amax(MAP_for_path)
            j_idx_max = list_non_visited_j[idx]
            i_idx_max = list_non_visited_i[idx]

            # Analise every 3x3 cells of every cell of the path to define movement and orientation
            for x in range(0, idx_max-1):
                sides_array = np.array([[0, MAP_for_path[j_idx_max- 1, i_idx_max], 0],
                                        [MAP_for_path[j_idx_max, i_idx_max - 1], MAP_for_path[j_idx_max, i_idx_max], MAP_for_path[j_idx_max, i_idx_max + 1]],
                                        [0, MAP_for_path[j_idx_max + 1, i_idx_max], 0]])

                print(sides_array)

                j_idx, i_idx = np.where(sides_array == idx_max-1)
                ji = (j_idx[0], i_idx[0])
                print('Coord = [' + str(j_idx) + ', ' + str(i_idx) + ']')

                if ji == (0,1):
                    list_movents.append('DOWN')
                    j_idx_max = j_idx_max - 1
                elif ji == (1,0):
                    list_movents.append('RIGHT')
                    i_idx_max = i_idx_max - 1
                elif ji == (1, 2):
                    list_movents.append('LEFT')
                    i_idx_max = i_idx_max + 1
                elif ji == (2, 1):
                    list_movents.append('UP')
                    j_idx_max = j_idx_max + 1

                idx_max = idx_max - 1

            list_movents = list_movents[1::2]
            list_movents = list_movents[::-1]
            print('The agent must follow the next movements: ' + str(list_movents))

            return list_movents

        except:

            print('There are no unknown cells! All the maze has been recognized! (:')
            exit()

    def FollowPath(self, list_movents):

        Z = self.DefineZone()

        # Analize and actuate the agent depending on the zone and the list of movements

        # First rotation to start the new movement
        if list_movents[0] == 'RIGHT':
            while Z[0] == False:
                self.Rotate_Left()
                Z = self.DefineZone()
        elif list_movents[0] == 'UP':
            while Z[1] == False:
                self.Rotate_Left()
                Z = self.DefineZone()
        elif list_movents[0] == 'LEFT':
            while Z[2] == False:
                self.Rotate_Left()
                Z = self.DefineZone()
        elif list_movents[0] == 'DOWN':
            while Z[3] == False:
                self.Rotate_Left()
                Z = self.DefineZone()

        # Go ahead and then continue the defined path
        self.Go_Ahead(Z)

        for x in range(1, len(list_movents)):
            if list_movents[x] == list_movents[x-1]:
                self.Go_Ahead(Z)
            elif list_movents[x-1] == 'RIGHT' and list_movents[x] == 'DOWN':
                self.Rotate_Right()
                Z = self.DefineZone()
                self.Go_Ahead(Z)
            elif list_movents[x-1] == 'RIGHT' and list_movents[x] == 'UP':
                self.Rotate_Left()
                Z = self.DefineZone()
                self.Go_Ahead(Z)
            elif list_movents[x-1] == 'UP' and list_movents[x] == 'RIGHT':
                self.Rotate_Right()
                Z = self.DefineZone()
                self.Go_Ahead(Z)
            elif list_movents[x-1] == 'UP' and list_movents[x] == 'LEFT':
                self.Rotate_Left()
                Z = self.DefineZone()
                self.Go_Ahead(Z)
            elif list_movents[x-1] == 'LEFT' and list_movents[x] == 'UP':
                self.Rotate_Right()
                Z = self.DefineZone()
                self.Go_Ahead(Z)
            elif list_movents[x-1] == 'LEFT' and list_movents[x] == 'DOWN':
                self.Rotate_Left()
                Z = self.DefineZone()
                self.Go_Ahead(Z)
            elif list_movents[x-1] == 'DOWN' and list_movents[x] == 'LEFT':
                self.Rotate_Right()
                Z = self.DefineZone()
                self.Go_Ahead(Z)
            elif list_movents[x-1] == 'DOWN' and list_movents[x] == 'RIGHT':
                self.Rotate_Left()
                Z = self.DefineZone()
                self.Go_Ahead(Z)

    def WriteMap(self, MAP, MAP_y_initial, MAP_x_initial):


        # Convert MAP to string
        MAP[MAP_y_initial][MAP_x_initial] = 4
        MAP_str = str(MAP)

        # String of numbers only
        MAP_str2 = []
        for x in range(0, len(MAP_str)):
            if MAP_str[x].isnumeric():
                MAP_str2 += MAP_str[x]

        # Convert code numbers to actual representation
        MAP_str3 = []
        for c in range(len(MAP_str2)):
            if MAP_str2[c] == '2':
                MAP_str3 += '|'
            elif MAP_str2[c] == '3':
                MAP_str3 += '-'
            elif MAP_str2[c] == '7' or MAP_str2[c] == '9' or MAP_str2[c] == '1' or MAP_str2[c] == '5':
                MAP_str3 += 'X'
            elif MAP_str2[c] == '4':
                MAP_str3 += 'I'
            else:
                MAP_str3 += ' '

        # Concatenate strings and separe in lines
        MAP_str4 = ''.join(map(str, MAP_str3))
        MAP_str5 = MAP_str4[0:54]
        for a in range(1, 26):
            MAP_str5 = MAP_str5 + '\n' + MAP_str4[a * 54:(a + 1) * 54]

        # Write txt file
        with open('readmymap.txt', 'w') as f:
            f.write(MAP_str5)

    def wander(self):

        # ------------------------------------------------------
        # Start VARIABLES
        # ------------------------------------------------------
        global GPS_x_initial
        global GPS_y_initial
        global GPS_x_current
        global GPS_y_current
        global MAP
        global MAP_x_current
        global MAP_y_current
        global MAP_x_initial
        global MAP_y_initial

        self.readSensors()

        compass = self.measures.compass

        GPS_x = self.measures.x - GPS_x_initial
        GPS_y = self.measures.y - GPS_y_initial

        GPS_x_current_vet = []
        for i in range(-26, 28, 2):
            GPS_x_current_vet.append(i)
        GPS_x_current = GPS_x_current_vet[closest(GPS_x_current_vet, GPS_x)]

        GPS_y_current_vet = []
        for j in range(-12, 14, 2):
            GPS_y_current_vet.append(j)
        GPS_y_current = GPS_y_current_vet[closest(GPS_y_current_vet, GPS_y)]

        # Initial position in MAP
        MAP_x_initial = 27
        MAP_y_initial = 13

        MAP_x_current = MAP_x_initial + GPS_x_current
        MAP_y_current = MAP_y_initial - GPS_y_current

        if MAP_x_current == []:
            MAP_x_current = MAP_x_initial
        if MAP_y_current == []:
            MAP_y_current = MAP_y_initial

        print('\n\n\nNEW MOVEMENT!\n')

        # Define the working zone
        Z = self.DefineZone()

        # ------------------------------------------------------
        # Update MAP
        # ------------------------------------------------------
        MAP[MAP_y_initial][MAP_x_initial] = 7
        F, F2, R, R2, L, L2 = self.Mapping(Z, MAP_y_current, MAP_x_current)

        # ------------------------------------------------------
        # Analise MAP
        # ------------------------------------------------------
        MAP[MAP_y_current][MAP_x_current] = 9
        MAP_np = np.array(MAP)
        list_non_visited_j, list_non_visited_i = np.where(MAP_np == 5)
        actual_position_j, actual_position_i = np.where(MAP_np == 9)

        print('List coords not visited (i) = ' + str(list_non_visited_i))
        print('List coords not visited (j) = ' + str(list_non_visited_j))
        print('Actual Position (i) = ' + str(actual_position_i))
        print('Actual Position (j) = ' + str(actual_position_j))
        print(MAP)
        MAP[MAP_y_current][MAP_x_current] = 7


        # ------------------------------------------------------
        # Decision of movement
        # ------------------------------------------------------

        # Rotate First Right

        # Rotate Right
        if R2 == 5 and R == 1 and F2 != 5:

            self.Rotate_Right()

        # Rotate Left
        elif L2 == 5 and L == 1 and F2 != 5:

            self.Rotate_Left()

        # Rotate First Left
        # Rotate Left
        # if L2 == 5 and L == 1 and F2 != 5:
        #
        #     self.Rotate_Left()
        #
        # # Rotate Right
        # elif R2 == 5 and R == 1 and F2 != 5:
        #
        #     self.Rotate_Right()

        # Go ahead
        elif F == 1:

            self.Go_Ahead(Z)

        else:

            list_movents = self.CalculatePath(MAP_np, list_non_visited_i, list_non_visited_j, actual_position_i, actual_position_j)
            self.FollowPath(list_movents)


        # Turn off
        R = 0
        L = 0
        self.driveMotors(L, R)

        # Write MAP on txt
        self.WriteMap(MAP, MAP_y_initial, MAP_x_initial)

class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()

        self.labMap = [[' '] * (CELLCOLS * 2 - 1) for i in range(CELLROWS * 2 - 1)]
        i = 1
        for child in root.iter('Row'):
            line = child.attrib['Pattern']
            row = int(child.attrib['Pos'])
            if row % 2 == 0:  # this line defines vertical lines
                for c in range(len(line)):
                    if (c + 1) % 3 == 0:
                        if line[c] == '|':
                            self.labMap[row][(c + 1) // 3 * 2 - 1] = '|'
                        else:
                            None
            else:  # this line defines horizontal lines
                for c in range(len(line)):
                    if c % 3 == 0:
                        if line[c] == '-':
                            self.labMap[row][c // 3 * 2] = '-'
                        else:
                            None

            i = i + 1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None

for i in range(1, len(sys.argv), 2):
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
    rob = MyRob(rob_name, pos, [0.0, 90.0, -90.0, 180.0], host)

    # Variables
    lap = []
    lap_time_vet = [0]
    sen_ground_vet = []

    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()

    rob.run()
