
import sys
from croblink import *
from math import *
from robtools import *
from astar import astar
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14

CENTER_ID = 0
LEFT_ID = 1
RIGHT_ID = 2
BACK_ID = 3

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.readSensors()
        # Start variables related to maze
        self.map_width = CELLCOLS*4-1
        self.map_height = CELLROWS*4-1
        self.map_starting_spot = Point(floor(self.map_height/2), floor(self.map_width/2))
        
        if self.measures.gpsReady:
            print(self.measures.x, self.measures.y)
            self.gps_starting_spot = Point(self.measures.x, self.measures.y)    
            self.gps2cell_offset = (self.measures.x - self.map_starting_spot.y, self.measures.y + self.map_starting_spot.x)
        
        if challenge == 1:
            self.laps = 0
            self.visited_beacons = 0
            self.on_beacon = False
        elif challenge == 2:    
            self.map = [[' '] * self.map_width for i in range(self.map_height) ]
            self.visited_nodes = []
            self.nodes_to_visit = []
            # Mark known cell
            self.map[self.map_starting_spot.x][self.map_starting_spot.y] = 'X'
            self.visited_nodes.append(Point(0, 0))
            self.robot_state = RobotStates.MAPPING
    
    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap
    
    def add_to_visited_cells(self, cell):
        if cell not in self.visited_nodes:
            self.visited_nodes.append(cell)
    
    def add_to_cells_to_visit(self, cell):
        if cell not in self.visited_nodes and cell not in self.nodes_to_visit:
            self.nodes_to_visit.append(cell)
    
    def clean_cells_to_visit(self):
        aux = [i for i in self.nodes_to_visit if i not in self.visited_nodes]
        self.nodes_to_visit = aux
    
    # Takes GPS coordinates and returns x,y indexes for map
    def gps2mapcell(self, robot_location):
        return Point(round(robot_location.x - self.gps2cell_offset[0]), round(self.gps2cell_offset[1] - robot_location.y))

    # Takes GPS coordinates and returns the current cell
    def gps2robotcell(self, robot_location):
        return Point(robot_location.x - self.gps_starting_spot.x, robot_location.y - self.gps_starting_spot.y)

    # Takes map cell and converts it to robot cell    
    def mapcell2robotcell(self, cell):
        return Point(cell.x - self.map_starting_spot.y , self.map_starting_spot.x - cell.y)

    # Takes robot cell and conerts it to map cell
    def robotcell2mapcell(self, cell):
        return Point(cell.x + self.map_starting_spot.y,  self.map_starting_spot.x - cell.y)

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def readAndOrganizeSensors(self):
        self.readSensors()
        ir_sensors = None
        ground = None
        robot_location = None
        # Organize sensor data
        if self.measures.irSensorReady:
            ir_sensors = IRSensorData(self.measures.irSensor[CENTER_ID],
                                      self.measures.irSensor[LEFT_ID],
                                      self.measures.irSensor[RIGHT_ID],
                                      self.measures.irSensor[BACK_ID])
        if self.measures.beaconReady:
            pass
        if self.measures.groundReady:
            ground = SetGroundSensorData(self.measures.ground)
        if self.measures.gpsReady and self.measures.compassReady:
            robot_location = Point(self.measures.x, self.measures.y)
        
        return ir_sensors, ground, robot_location


    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

        while True:
            ir_sensors, ground, robot_location = self.readAndOrganizeSensors()
            # Track info on laps in challenge 1
            if challenge == 1:
                if ground.status == GroundStatus.beacon1 or ground.status == GroundStatus.beacon2:
                    if not self.on_beacon:
                        self.visited_beacons += 1
                        self.on_beacon = True
                        print("Visiting Beacon!")
                else:
                    self.on_beacon = False

                if self.visited_beacons == 2 and ground.status== GroundStatus.home:
                    self.laps += 1
                    print(f"Completed {self.laps} Lap! {10-self.laps} to go")
                    self.visited_beacons = 0

                if self.laps == 10 or self.measures.time == 4999:
                    print(f"Final Lap Count: {self.laps}!")
                    self.finish()
            elif challenge == 2:
                pass

            if self.measures.endLed:
                print(self.robName + " exiting")
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
                if challenge == 1:
                    self.c1_move(ir_sensors)
                else:
                    self.c2_brain(ir_sensors, robot_location)
    
    def rotate_until(self, angle):
        print("Initial:",angle, self.measures.compass, angle-self.measures.compass)

        if self.measures.compass >= 0:
            orientation_deviation = radians(angle - self.measures.compass)
        elif angle == Orientation.S.value:
            orientation_deviation =  radians(abs(self.measures.compass) - angle)
        else: 
            orientation_deviation = radians(angle - self.measures.compass)

        while True:
            deviation = orientation_deviation * 1
            if deviation == 0:
                break

            self.driveMotors(0.1-deviation, 0.1+deviation)
            self.readSensors()

            if self.measures.compass >= 0:
                orientation_deviation = radians(angle - self.measures.compass)
            elif angle == Orientation.S.value:
                orientation_deviation =  radians(abs(self.measures.compass) - angle)
            else: 
                orientation_deviation = radians(angle - self.measures.compass)

        self.driveMotors(0, 0)

        print("Final:",angle, self.measures.compass, angle-self.measures.compass)

    
    def rotate(self, angle):
        starting_angle = self.measures.compass

        end_angle = starting_angle + angle

        if end_angle < -180:
            end_angle = 180 - (abs(end_angle)-180)
        elif end_angle > 180:
            end_angle = -180 + (abs(end_angle)-180)

        self.rotate_until(end_angle)
    
    def calculate_deviation(self, robot_location, next_position):
        current_cardinal = degree_to_cardinal(self.measures.compass)
        expected_orientation = Orientation[current_cardinal]
        curr_cell = self.gps2robotcell(robot_location)
        print("curr", curr_cell)
        print("next", next_position)

        if self.measures.compass >= 0:
            orientation_deviation = radians(expected_orientation.value - self.measures.compass)
        elif expected_orientation == Orientation.S:
            orientation_deviation =  radians(abs(self.measures.compass) - expected_orientation.value)
        else: 
            orientation_deviation = radians(expected_orientation.value - self.measures.compass)

        if expected_orientation == Orientation.N:
            linear_deviation = next_position.y - curr_cell.y
        elif expected_orientation == Orientation.W:
            linear_deviation = curr_cell.x - next_position.x
        elif expected_orientation == Orientation.S:
            linear_deviation = curr_cell.y - next_position.y
        elif expected_orientation == Orientation.E:
            linear_deviation = next_position.x - curr_cell.x 

        return orientation_deviation + linear_deviation 

    def c2_brain(self, ir_sensors, robot_location):
        if self.robot_state == RobotStates.MAPPING:
            self.driveMotors(0,0)  
            # Mark walls on map, X on curr floor 
            self.mark_walls()
            # Prints to file, maybe only run this in the end due to performance
            self.print_map_to_file()
            
            ## IF MAP IS FINISHED STOP, ELSE MOVE
            self.robot_state =  RobotStates.MOVING
        elif self.robot_state == RobotStates.MOVING:
            # Find candidates to move to, get path to it, execute said movements
            #self.c2_smart_move(ir_sensors, robot_location)
            self.c2_move(ir_sensors, robot_location)
            self.robot_state = RobotStates.MAPPING
            self.clean_cells_to_visit()
            print("VISITED", self.visited_nodes)
            print("TO VISIT", self.nodes_to_visit)
            pass 

    def check_if_reachable(self, curr_cell, dest_cell):

        if abs(dest_cell.x - curr_cell.x) > 2 or dest_cell.y != curr_cell.y:
            return False
        elif abs(dest_cell.y - curr_cell.y) > 2 or dest_cell.x != curr_cell.x:
            return False
        else: 
            return True   

    def get_direction_to_cell(self, curr_cell, dest_cell):
        if dest_cell.x > curr_cell.x:
            return Orientation.N
        elif dest_cell.x < curr_cell.x:
            return Orientation.S
        elif dest_cell.y > curr_cell.y:
            return Orientation.W
        else:
            return Orientation.E 
    
    def c2_move(self, ir_sensors, robot_location):
        curr_cell = self.gps2robotcell(robot_location)
        print("cells to visit", self.nodes_to_visit, "\n\n\n")
        dest_cell = self.nodes_to_visit.pop(0)
        dest_cell = Point(round_up_to_even(dest_cell.x), round_up_to_even(dest_cell.y))

        if self.check_if_reachable(curr_cell, dest_cell):
            self.rotate_until(self.get_direction_to_cell(curr_cell, dest_cell).value)
            _, _, robot_location = self.readAndOrganizeSensors()
            dest_cell = Point(round_up_to_even(dest_cell.x), round_up_to_even(dest_cell.y))
            self.move_forward(robot_location, dest_cell)
        else:

            dest_map_cell = self.robotcell2mapcell(dest_cell)
            dest_map_cell = Point(dest_map_cell.y, dest_map_cell.x)

            curr_map_cell = Point(round_up_to_even(curr_cell.x), round_up_to_even(curr_cell.y))
            curr_map_cell = self.robotcell2mapcell(curr_map_cell)
            curr_map_cell = Point(curr_map_cell.y, curr_map_cell.x)
            print("curr",curr_map_cell)
            print("dest",dest_map_cell)
            print("curr rcell", curr_cell)
            print("dest rcell", dest_cell)
            path = astar(self.map, curr_map_cell, dest_map_cell)
            if not path:
                print(self.map)
            move_list = path_to_moves(path)
            print("Path", path)
            print("Move to path", path_to_moves(path))
            while move_list:
                _ = move_list.pop(0)
                dest_cell, orientation = move_list.pop(0)
                dest_cell = self.mapcell2robotcell(dest_cell)
                
                self.rotate_until(orientation.value)
                _, _, robot_location = self.readAndOrganizeSensors()
                dest_cell = Point(round_up_to_even(dest_cell.x), round_up_to_even(dest_cell.y))
                self.move_forward(robot_location, dest_cell)

    def c2_smart_move(self, ir_sensors, robot_location):
        curr_orientation = Orientation[degree_to_cardinal(self.measures.compass)]
        curr_cell = self.gps2robotcell(robot_location)
        threshold = 0.8
        dest_cell = Point(curr_cell.x, curr_cell.y)
        if ir_sensors.left < threshold:
            print("Go Left")
            if curr_orientation == Orientation.N:
                dest_cell = Point(curr_cell.x, curr_cell.y+2.0)
                self.rotate_until(Orientation.W.value)
            elif curr_orientation == Orientation.W:
                dest_cell = Point(curr_cell.x-2.0, curr_cell.y)
                self.rotate_until(Orientation.S.value)
            elif curr_orientation == Orientation.S:
                dest_cell = Point(curr_cell.x, curr_cell.y-2.0)
                self.rotate_until(Orientation.E.value)
            elif curr_orientation == Orientation.E:
                dest_cell = Point(curr_cell.x+2.0, curr_cell.y)
                self.rotate_until(Orientation.N.value)
        elif ir_sensors.right < threshold:
            print("Go Right")
            if curr_orientation == Orientation.N:
                dest_cell = Point(curr_cell.x, curr_cell.y-2.0)
                self.rotate_until(Orientation.E.value)
            elif curr_orientation == Orientation.W:
                dest_cell = Point(curr_cell.x+2.0, curr_cell.y)
                self.rotate_until(Orientation.N.value)
            elif curr_orientation == Orientation.S:
                dest_cell = Point(curr_cell.x, curr_cell.y+2.0)
                self.rotate_until(Orientation.W.value)
            elif curr_orientation == Orientation.E:
                dest_cell = Point(curr_cell.x-2.0, curr_cell.y)
                self.rotate_until(Orientation.S.value)
        elif ir_sensors.center < threshold:
            print("Move Forwards")
            if curr_orientation == Orientation.N:
                dest_cell = Point(curr_cell.x+2.0, curr_cell.y)
            elif curr_orientation == Orientation.W:
                dest_cell = Point(curr_cell.x, curr_cell.y+2.0)
            elif curr_orientation == Orientation.S:
                dest_cell = Point(curr_cell.x-2.0, curr_cell.y)
            elif curr_orientation == Orientation.E:
                dest_cell = Point(curr_cell.x, curr_cell.y-2.0)
        elif ir_sensors.back < threshold:
            print("Go back")
            if curr_orientation == Orientation.N:
                dest_cell = Point(curr_cell.x-2.0, curr_cell.y)
                self.rotate_until(Orientation.S.value)
            elif curr_orientation == Orientation.W:
                dest_cell = Point(curr_cell.x, curr_cell.y-2.0)
                self.rotate_until(Orientation.E.value)
            elif curr_orientation == Orientation.S:
                dest_cell = Point(curr_cell.x+2.0, curr_cell.y)
                self.rotate_until(Orientation.N.value)
            elif curr_orientation == Orientation.E:
                dest_cell = Point(curr_cell.x, curr_cell.y+2.0)
                self.rotate_until(Orientation.W.value)
        
        _, _, robot_location = self.readAndOrganizeSensors()
        curr_orientation = Orientation[degree_to_cardinal(self.measures.compass)]
        # TODO: LOOK INTO ROUNDING PROBLEMS
        dest_cell = Point(round_up_to_even(dest_cell.x), round_up_to_even(dest_cell.y))

        self.move_forward(robot_location, dest_cell)        

    
    def move_forward(self, robot_location, dest_cell):
        while True:
            deviation = self.calculate_deviation(robot_location, dest_cell)
            curr_orientation = Orientation[degree_to_cardinal(self.measures.compass)]
            
            self.driveMotors(0.1-deviation, 0.1+deviation)
            _, _, robot_location = self.readAndOrganizeSensors()
            curr_cell = self.gps2robotcell(robot_location)
            curr_map_cell = self.gps2mapcell(robot_location)
            self.map[curr_map_cell.y][curr_map_cell.x] = 'X'
            self.add_to_visited_cells(Point(round(curr_cell.x), round(curr_cell.y)))

            if curr_orientation == Orientation.N:
                if curr_cell.x >= dest_cell.x:
                    break
            elif curr_orientation == Orientation.W:
                if curr_cell.y >= dest_cell.y:
                    break
            elif curr_orientation == Orientation.S:
                if curr_cell.x <= dest_cell.x:
                    break
            else:
                if curr_cell.y <= dest_cell.y:
                    break
        self.driveMotors(0.0,0.0)

            
    def c1_move(self, ir_sensors):
        #print(sensors.center, sensors.left, sensors.right, sensors.back)
        if ir_sensors.center > 1.2:
            if ir_sensors.left < ir_sensors.right:
                print('Rotate left')
                self.driveMotors(-0.15,+0.15)
            else:
                print('Rotate right')
                self.driveMotors(+0.15, -0.15)
        elif ir_sensors.right > 6:
            print('Slight Rotate left')
            self.driveMotors(-0.15,+0.1)
        elif ir_sensors.left > 6:
            print('Slight rotate right') 
            self.driveMotors(+0.1, -0.15)
        else:
            print('Go')
            self.driveMotors(0.15,0.15)

    def mark_walls(self):
        ir_sensors, ground, robot_location = self.readAndOrganizeSensors()
        direction = degree_to_cardinal(self.measures.compass)
        curr_cell = self.gps2mapcell(robot_location)
        print(ir_sensors)
        threshold = 2.0
        
        if direction == 'N':
            if ir_sensors.left >= threshold:
                self.map[curr_cell.y-1][curr_cell.x] = '-'
            else:
                self.map[curr_cell.y-1][curr_cell.x] = 'X'
                self.map[curr_cell.y-2][curr_cell.x] = 'X'
                new_cell_to_visit = Point(curr_cell.x, curr_cell.y-1)
                self.add_to_cells_to_visit(self.mapcell2robotcell(new_cell_to_visit))
            if ir_sensors.right >= threshold:
                self.map[curr_cell.y+1][curr_cell.x] = '-'
            else:
                self.map[curr_cell.y+1][curr_cell.x] = 'X'
                self.map[curr_cell.y+2][curr_cell.x] = 'X'
                new_cell_to_visit = Point(curr_cell.x, curr_cell.y+1)
                self.add_to_cells_to_visit(self.mapcell2robotcell(new_cell_to_visit))
            if ir_sensors.center >= threshold:
                self.map[curr_cell.y][curr_cell.x+1] = '|'
            else:
                self.map[curr_cell.y][curr_cell.x+1] = 'X'
                self.map[curr_cell.y][curr_cell.x+2] = 'X'                
                new_cell_to_visit = Point(curr_cell.x+1, curr_cell.y)
                self.add_to_cells_to_visit(self.mapcell2robotcell(new_cell_to_visit))
            if ir_sensors.back >= threshold:
                self.map[curr_cell.y][curr_cell.x-1] = '|'
            else:
                self.map[curr_cell.y][curr_cell.x-1] = 'X'
                self.map[curr_cell.y][curr_cell.x-2] = 'X'
                new_cell_to_visit = Point(curr_cell.x-1, curr_cell.y)
                self.add_to_cells_to_visit(self.mapcell2robotcell(new_cell_to_visit))
        elif direction == "W":
            if ir_sensors.left >= threshold:
                self.map[curr_cell.y][curr_cell.x-1] = '|'
            else:
                self.map[curr_cell.y][curr_cell.x-1] = 'X'
                self.map[curr_cell.y][curr_cell.x-2] = 'X'
                new_cell_to_visit = Point(curr_cell.x-1, curr_cell.y)
                self.add_to_cells_to_visit(self.mapcell2robotcell(new_cell_to_visit))
            if ir_sensors.right >= threshold:
                self.map[curr_cell.y][curr_cell.x+1] = '|'
            else:
                self.map[curr_cell.y][curr_cell.x+1] = 'X'
                self.map[curr_cell.y][curr_cell.x+2] = 'X'
                new_cell_to_visit = Point(curr_cell.x+1, curr_cell.y)
                self.add_to_cells_to_visit(self.mapcell2robotcell(new_cell_to_visit))
            if ir_sensors.center >= threshold:
                self.map[curr_cell.y-1][curr_cell.x] = '-'
            else:
                self.map[curr_cell.y-1][curr_cell.x] = 'X'
                self.map[curr_cell.y-2][curr_cell.x] = 'X'
                new_cell_to_visit = Point(curr_cell.x, curr_cell.y-1)
                self.add_to_cells_to_visit(self.mapcell2robotcell(new_cell_to_visit))
            if ir_sensors.back >= threshold:
                self.map[curr_cell.y+1][curr_cell.x] = '-'
            else:
                self.map[curr_cell.y+1][curr_cell.x] = 'X'
                self.map[curr_cell.y+2][curr_cell.x] = 'X'
                new_cell_to_visit = Point(curr_cell.x, curr_cell.y+1)
                self.add_to_cells_to_visit(self.mapcell2robotcell(new_cell_to_visit))
        elif direction == "E":
            if ir_sensors.left >= threshold:
                self.map[curr_cell.y][curr_cell.x+1] = '|'
            else:
                self.map[curr_cell.y][curr_cell.x+1] = 'X'
                self.map[curr_cell.y][curr_cell.x+2] = 'X'
                new_cell_to_visit = Point(curr_cell.x+1, curr_cell.y)
                self.add_to_cells_to_visit(self.mapcell2robotcell(new_cell_to_visit))
            if ir_sensors.right >= threshold:
                self.map[curr_cell.y][curr_cell.x-1] = '|'
            else:
                self.map[curr_cell.y][curr_cell.x-1] = 'X'
                self.map[curr_cell.y][curr_cell.x-2] = 'X'
                new_cell_to_visit = Point(curr_cell.x-1, curr_cell.y)
                self.add_to_cells_to_visit(self.mapcell2robotcell(new_cell_to_visit))
            if ir_sensors.center >= threshold:
                self.map[curr_cell.y+1][curr_cell.x] = '-'
            else:
                self.map[curr_cell.y+1][curr_cell.x] = 'X'
                self.map[curr_cell.y+2][curr_cell.x] = 'X'
                new_cell_to_visit = Point(curr_cell.x, curr_cell.y+1)
                self.add_to_cells_to_visit(self.mapcell2robotcell(new_cell_to_visit))
            if ir_sensors.back >= threshold:
                self.map[curr_cell.y-1][curr_cell.x] = '-'
            else:
                self.map[curr_cell.y-1][curr_cell.x] = 'X'
                self.map[curr_cell.y-2][curr_cell.x] = 'X'
                new_cell_to_visit = Point(curr_cell.x, curr_cell.y-1)
                self.add_to_cells_to_visit(self.mapcell2robotcell(new_cell_to_visit))
        else:
            if ir_sensors.left >= threshold:
                self.map[curr_cell.y+1][curr_cell.x] = '-'
            else:
                self.map[curr_cell.y+1][curr_cell.x] = 'X'
                self.map[curr_cell.y+2][curr_cell.x] = 'X'
                new_cell_to_visit = Point(curr_cell.x, curr_cell.y+1)
                self.add_to_cells_to_visit(self.mapcell2robotcell(new_cell_to_visit))
            if ir_sensors.right >= threshold:
                self.map[curr_cell.y-1][curr_cell.x] = '-'
            else:
                self.map[curr_cell.y-1][curr_cell.x] = 'X'
                self.map[curr_cell.y-2][curr_cell.x] = 'X'
                new_cell_to_visit = Point(curr_cell.x, curr_cell.y-1)
                self.add_to_cells_to_visit(self.mapcell2robotcell(new_cell_to_visit))
            if ir_sensors.center >= threshold:
                self.map[curr_cell.y][curr_cell.x-1] = '|'
            else:
                self.map[curr_cell.y][curr_cell.x-1] = 'X'
                self.map[curr_cell.y][curr_cell.x-2] = 'X'
                new_cell_to_visit = Point(curr_cell.x-1, curr_cell.y)
                self.add_to_cells_to_visit(self.mapcell2robotcell(new_cell_to_visit))
            if ir_sensors.back >= threshold:
                self.map[curr_cell.y][curr_cell.x+1] = '|'
            else:
                self.map[curr_cell.y][curr_cell.x+1] = 'X'
                self.map[curr_cell.y][curr_cell.x+2] = 'X'
                new_cell_to_visit = Point(curr_cell.x+1, curr_cell.y)
                self.add_to_cells_to_visit(self.mapcell2robotcell(new_cell_to_visit))             
    
    def print_map_to_file(self):
        fout = open("map.txt", "w+")
        
        for row in range(len(self.map)):
            for col in range(len(self.map[row])):
                fout.write(self.map[row][col])
            fout.write("\n")

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
challenge = 1

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    elif (sys.argv[i] == "--challenge" or sys.argv[i] == "-c") and i != len(sys.argv) - 1:
        challenge = int(sys.argv[i+1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    if challenge == 1:
        rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host)
    else:
        rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
