
import sys
from croblink import *
from math import *
from robtools import *
from astar import astar, translate_map
from itertools import combinations
from simple_pid import PID
import xml.etree.ElementTree as ET
import time

# TODO check walls when know map node but not visited yet
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
        
        self.GroundStatus = Ground(self.nBeacons).GroundStatus

        self.alpha = 0.85
        self.prev_avg = 0
        
        if self.measures.gpsReady:
            self.gps_starting_spot = Point(self.measures.x, self.measures.y)    
            self.gps2cell_offset = (self.measures.x - self.map_starting_spot.y, self.measures.y + self.map_starting_spot.x)
        
        if challenge == 1:
            self.laps = 0
            self.visited_beacons = 0
            self.on_beacon = False
        elif challenge == 2 or challenge == 3 or challenge == 4:    
            self.map = [[' '] * self.map_width for i in range(self.map_height) ]
            self.visited_nodes = []
            self.nodes_to_visit = []
            # Mark known cell
            self.map[self.map_starting_spot.x][self.map_starting_spot.y] = 'X'
            self.visited_nodes.append(Point(0, 0))
            self.robot_state = RobotStates.MAPPING
            self.move_list = []
            if challenge == 3 or challenge == 4:
                self.beacon_location = {}
                self.beacon_location[0] = Point(self.map_starting_spot.y, self.map_starting_spot.x)
                self.possible_subpaths = list(combinations(list(range(self.nBeacons)), 2))
                self.subpaths_cost = {}
                self.possible_paths = generatePossiblePaths(self.nBeacons)
                self.path_cost = {}
                self.shortest_path = []
                self.shortest_path_index = 0
                if challenge == 4:
                    self.r_location = RobotLocation()
                    self.curr_cell = Point(0, 0)
                    self.prev_out_l = 0
                    self.prev_out_r = 0
                    self.prev_ang = 0
    
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
        elif cell in self.nodes_to_visit:
            self.nodes_to_visit.pop(self.nodes_to_visit.index(cell))
            self.nodes_to_visit.append(cell)
    
    def clean_cells_to_visit(self):
        aux = [i for i in self.nodes_to_visit if i not in self.visited_nodes]
        self.nodes_to_visit = aux

    def add_beacon_location(self, robot_location, ground):
        if challenge != 4:
            curr_cell = self.gps2mapcell(robot_location)
        else:
            curr_cell = self.robotcell2mapcell(robot_location)
        if curr_cell not in self.beacon_location.values():
            self.beacon_location[ground.status.value] = curr_cell

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
            ground = SetGroundSensorData(self.measures.ground, self.GroundStatus)
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
                    self.c1_brain(ir_sensors, ground)
                elif challenge == 2:
                    self.c2_brain(ir_sensors, robot_location)
                elif challenge == 3:
                    self.c3_brain(ir_sensors, ground, robot_location)
                elif challenge == 4:
                    self.c4_brain(ir_sensors, ground)
                else:
                    print("Not a valid challenge")
                    exit()
    
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

    def c1_brain(self, ir_sensors, ground):
        # Track info on laps in challenge 1

        if ground.status.value > 0:
            if not self.on_beacon:
                self.visited_beacons += 1
                self.on_beacon = True
                print("Visiting Beacon!")
        else:
            self.on_beacon = False

        if self.visited_beacons == self.nBeacons-1 and ground.status == self.GroundStatus.home:
            self.laps += 1
            print(f"Completed {self.laps} Lap! {10-self.laps} to go")
            self.visited_beacons = 0

        if self.laps == 10 or self.measures.time == self.totalTime-1:
            print(f"Final Lap Count: {self.laps}!")
            self.finish()
        
        self.c1_move(ir_sensors)

    def c2_brain(self, ir_sensors, robot_location):
        if self.robot_state == RobotStates.MAPPING:
            self.driveMotors(0,0)  
            # Mark walls on map, X on curr floor 
            self.mark_walls()
            # Prints to file, maybe only run this in the end due to performance
            self.print_map_to_file(filename)
            
            ## IF MAP IS FINISHED STOP, ELSE MOVE
            if not self.nodes_to_visit  or self.measures.time == self.totalTime-1:
                self.robot_state = RobotStates.FINISHED
            else:
                self.robot_state =  RobotStates.MOVING
        elif self.robot_state == RobotStates.MOVING:
            # Find candidates to move to, get path to it, execute said movements
            print("TO VISIT", self.nodes_to_visit)
            if self.c2_move(ir_sensors, robot_location):
                self.robot_state = RobotStates.MAPPING
            self.clean_cells_to_visit()
            if self.measures.time == self.totalTime-1:
                self.robot_state = RobotStates.FINISHED
        elif self.robot_state == RobotStates.FINISHED:
            self.map[self.map_starting_spot.x][self.map_starting_spot.y] = 'I'
            self.print_map_to_file(filename)
            print(f"Map printed to {filename}")
            self.finish()

    def c3_brain(self, ir_sensors, ground, robot_location):
        if self.robot_state == RobotStates.MAPPING:
            self.driveMotors(0,0)  
            # Mark walls on map, X on curr floor 
            self.mark_walls()
            # Save beacon location
            if ground.status.value > 0:
                self.add_beacon_location(robot_location, ground)
            
            ## IF COMPLETELY MAPPED STOP, IF ALL BEACONS FOUND GO PLAN, OTHERWISE MOVE
            if not self.nodes_to_visit  or self.measures.time == self.totalTime-1:
                self.robot_state = RobotStates.FINISHED
            elif len(self.beacon_location) == self.nBeacons:
                self.move_list = []
                self.nodes_to_visit = []
                self.robot_state = RobotStates.PLANNING
            else:
                self.robot_state =  RobotStates.MOVING
        elif self.robot_state == RobotStates.MOVING:
            # Find candidates to move to, get path to it, execute said movements
            print("TO VISIT", self.nodes_to_visit)
            if self.c2_move(ir_sensors, robot_location):
                self.robot_state = RobotStates.MAPPING
            self.clean_cells_to_visit()
            if self.measures.time == self.totalTime-1:
                self.robot_state = RobotStates.FINISHED
        elif self.robot_state == RobotStates.OPTIMIZING:
            # Check what subpath needs optimization
            optimal_subpath = [False]*len(self.shortest_path)
            subpath_to_optimize = None
            print("STATUS", ground.status.value)
            for i, subpath in enumerate(self.shortest_path):
                cost_known = 0
                cost_unk = 0
                start = reverse_point(self.beacon_location[subpath[0]])
                dest = reverse_point(self.beacon_location[subpath[1]])
                path_known = astar(self.map, start, dest, False)
                path_unk = astar(self.map, start, dest, True)
                cost_known += len(path_known)
                cost_unk += len(path_unk)
                if cost_known == cost_unk:
                    print(f"Subpath {subpath} optimized!")
                    optimal_subpath[i] = True
                elif subpath[0] == ground.status.value:
                    subpath_to_optimize = (i, subpath)
            
            if all(optimal_subpath) or self.measures.time >= self.totalTime - 1:
                print("All subpaths are optimal!")
                self.robot_state = RobotStates.PLANNING
                return 0
            
            if not subpath_to_optimize:
                to_optimize = [i for i, x in enumerate(optimal_subpath) if not x]
                if to_optimize:
                    index = to_optimize.pop()
                    subpath_to_optimize = (index, self.shortest_path[index])
            
            if ground.status.value != subpath_to_optimize[1][0]:
                self.nodes_to_visit.append(self.mapcell2robotcell(self.beacon_location[subpath_to_optimize[1][0]]))
                self.c2_move(ir_sensors, robot_location)
            else:
                while not optimal_subpath[subpath_to_optimize[0]]:
                    print("START OPTIMIZING")
                    ir_sensors, _, robot_location = self.readAndOrganizeSensors()
                    if self.c3_move(ir_sensors, robot_location, self.beacon_location[subpath_to_optimize[1][1]]):
                        print("PATH OPTIMIZED")
                        optimal_subpath[subpath_to_optimize[0]] = True
                    

        elif self.robot_state == RobotStates.PLANNING:
            self.calculate_path_costs()
            sorted_costs = [k for k, v in sorted(self.path_cost.items(), key=lambda item: item[1])]
            shortest_path_index = sorted_costs.pop(0)
            self.shortest_path_index = shortest_path_index
            self.shortest_path = self.possible_paths[shortest_path_index]

            if self.check_if_need_optimization(shortest_path_index):
                print("Shortest path can't be pinpointed!")
                print("Need to explore!")
                self.robot_state = RobotStates.OPTIMIZING
            else:
                print("Shortest path found!")
                self.robot_state = RobotStates.FINISHED
               
        elif self.robot_state == RobotStates.FINISHED:
            # Print map, path and distance to file, exit
            print("Printed map and plans to planning.out")
            print(f"Printed path to {filename}")

            for beacon, location in self.beacon_location.items():
                self.map[location.y][location.x] = str(beacon)
            self.print_path_to_file(filename)
            self.finish()
            exit()
    
    def c4_brain(self, ir_sensors, ground):
        if self.robot_state == RobotStates.MAPPING:
            self.driveMotors(0,0)  
            # Mark walls on map, X on curr floor 
            self.mark_walls()
            self.print_map_to_file(filename)

            # Save beacon location
            if ground.status.value > 0:
                self.add_beacon_location(self.curr_cell, ground)
            
            ## IF COMPLETELY MAPPED STOP, IF ALL BEACONS FOUND GO PLAN, OTHERWISE MOVE
            if not self.nodes_to_visit  or self.measures.time == self.totalTime-1:
                self.robot_state = RobotStates.FINISHED
            elif len(self.beacon_location) == self.nBeacons:
                self.move_list = []
                self.nodes_to_visit = []
                self.robot_state = RobotStates.PLANNING
            else:
                self.robot_state =  RobotStates.MOVING
        elif self.robot_state == RobotStates.MOVING:
            # Find candidates to move to, get path to it, execute said movements
            print("TO VISIT", self.nodes_to_visit)
            if self.c4_move_a():
                self.robot_state = RobotStates.MAPPING
            self.clean_cells_to_visit()
            if self.measures.time == self.totalTime-1:
                self.robot_state = RobotStates.FINISHED
        elif self.robot_state == RobotStates.OPTIMIZING:
            # Check what subpath needs optimization
            optimal_subpath = [False]*len(self.shortest_path)
            subpath_to_optimize = None
            print("STATUS", ground.status.value)
            for i, subpath in enumerate(self.shortest_path):
                cost_known = 0
                cost_unk = 0
                start = reverse_point(self.beacon_location[subpath[0]])
                dest = reverse_point(self.beacon_location[subpath[1]])
                path_known = astar(self.map, start, dest, False)
                path_unk = astar(self.map, start, dest, True)
                cost_known += len(path_known)
                cost_unk += len(path_unk)
                if cost_known == cost_unk:
                    print(f"Subpath {subpath} optimized!")
                    optimal_subpath[i] = True
                elif subpath[0] == ground.status.value:
                    subpath_to_optimize = (i, subpath)
            
            if all(optimal_subpath) or self.measures.time >= self.totalTime - 1:
                print("All subpaths are optimal!")
                self.robot_state = RobotStates.PLANNING
                return 0
            
            if not subpath_to_optimize:
                to_optimize = [i for i, x in enumerate(optimal_subpath) if not x]
                if to_optimize:
                    index = to_optimize.pop()
                    subpath_to_optimize = (index, self.shortest_path[index])
            
            if ground.status.value != subpath_to_optimize[1][0]:
                self.nodes_to_visit.append(self.mapcell2robotcell(self.beacon_location[subpath_to_optimize[1][0]]))
                self.c4_move_a()
            else:
                while not optimal_subpath[subpath_to_optimize[0]]:
                    print("START OPTIMIZING")
                    ir_sensors, _, robot_location = self.readAndOrganizeSensors()
                    if self.c4_move_b(ir_sensors, self.beacon_location[subpath_to_optimize[1][1]]):
                        print("PATH OPTIMIZED")
                        optimal_subpath[subpath_to_optimize[0]] = True
                    

        elif self.robot_state == RobotStates.PLANNING:
            self.calculate_path_costs()
            sorted_costs = [k for k, v in sorted(self.path_cost.items(), key=lambda item: item[1])]
            shortest_path_index = sorted_costs.pop(0)
            self.shortest_path_index = shortest_path_index
            self.shortest_path = self.possible_paths[shortest_path_index]

            if self.check_if_need_optimization(shortest_path_index):
                print("Shortest path can't be pinpointed!")
                print("Need to explore!")
                self.robot_state = RobotStates.OPTIMIZING
            else:
                print("Shortest path found!")
                self.robot_state = RobotStates.FINISHED
               
        elif self.robot_state == RobotStates.FINISHED:
            # Print map, path and distance to file, exit
            print(f"Printed map to {filename}.map")
            print(f"Printed path to {filename}.path")

            for beacon, location in self.beacon_location.items():
                self.map[location.y][location.x] = str(beacon)
            self.print_path_to_file(filename)
            self.print_map_to_file(filename)
            self.finish()
            exit()

    def c4_move_a(self):
        if not self.move_list:
            curr_cell = self.curr_cell
            dest_cell = self.nodes_to_visit.pop()
            
            dest_cell = Point(round_up_to_even(dest_cell.x), round_up_to_even(dest_cell.y))

            dest_map_cell = self.robotcell2mapcell(dest_cell)
            dest_map_cell = Point(dest_map_cell.y, dest_map_cell.x)

            curr_map_cell = Point(round_up_to_even(curr_cell.x), round_up_to_even(curr_cell.y))
            curr_map_cell = self.robotcell2mapcell(curr_map_cell)
            curr_map_cell = Point(curr_map_cell.y, curr_map_cell.x)

            print(f"MOVING FROM {curr_cell} -> {dest_cell}")
            
            path = astar(self.map, curr_map_cell, dest_map_cell)
            self.move_list = path_to_moves(path)
        #print("Path", path)
        #print("Move to path", path_to_moves(path))
        need_mapping = False
        while self.move_list:
            _ = self.move_list.pop(0)
            dest_cell, orientation = self.move_list.pop(0)
            dest_cell = self.mapcell2robotcell(dest_cell)

            if dest_cell not in self.visited_nodes:
                need_mapping = True
            
            self.rotate_c4(orientation.value)
            ir_sensors, _, _ = self.readAndOrganizeSensors()
            dest_cell = Point(round_up_to_even(dest_cell.x), round_up_to_even(dest_cell.y))
            self.move_forward_odometry(ir_sensors)
            self.add_to_visited_cells(dest_cell)

            self.curr_cell = dest_cell
            print("CURR", self.curr_cell)

            if need_mapping == True:
                print("CELL NEEDS MAPPING")
                return need_mapping
        return need_mapping

    def c4_move_b(self, ir_sensors, destination):
        if not self.move_list:
            curr_cell = self.curr_cell
            dest_cell = destination
            
            dest_map_cell = dest_cell
            dest_map_cell = Point(dest_map_cell.y, dest_map_cell.x)

            curr_map_cell = Point(round_up_to_even(curr_cell.x), round_up_to_even(curr_cell.y))
            curr_map_cell = self.robotcell2mapcell(curr_map_cell)
            curr_map_cell = Point(curr_map_cell.y, curr_map_cell.x)

            print(f"MOVING FROM {curr_cell} -> {dest_cell}")
            
            path = astar(self.map, curr_map_cell, dest_map_cell, True)
            self.move_list = path_to_moves(path)
            #print("Path", path)
            #print("Move to path", path_to_moves(path))
        need_mapping = False
        while self.move_list:
            _, prev_orientation = self.move_list.pop(0)
            dest_cell, orientation = self.move_list.pop(0) 
            dest_cell = self.mapcell2robotcell(dest_cell)

            if (dest_cell not in self.visited_nodes) and (dest_cell.x % 2 == 0 and dest_cell.y % 2 == 0):
                need_mapping = True
                        
            ir_sensors, _, _ = self.readAndOrganizeSensors()
            print(f"Going from {self.curr_cell} -> {dest_cell}")

            if self.check_if_next_is_possible(Orientation[degree_to_cardinal(self.measures.compass)], orientation, ir_sensors) and (prev_orientation == orientation):
                self.rotate_c4(orientation.value)
                ir_sensors, _, _ = self.readAndOrganizeSensors()
                print("CONTINUING TO MOVE")
                
                dest_cell = Point(round_up_to_even(dest_cell.x), round_up_to_even(dest_cell.y))
                self.move_forward_odometry(ir_sensors)
                self.curr_cell = dest_cell
                ir_sensors, _, _ = self.readAndOrganizeSensors()
                print("curr LOCATION", self.curr_cell)
                
                if need_mapping == True:
                    print("CELL NEEDS MAPPING")
                    self.driveMotors(0.0, 0.0)
                    self.mark_walls()
                    #self.print_map_to_file("planning.out")
            else:
                #self.print_map_to_file("planning.out")
                print("NOT POSSIBLE TO MOVE; RECALCULATING")
                self.driveMotors(0,0)
                self.move_list = []
                return False

        return True


    def calculate_deviation_odometry(self, ir_sensors):
        current_cardinal = degree_to_cardinal(self.measures.compass)
        expected_orientation = Orientation[current_cardinal]
        if self.measures.compass >= 0:
            orientation_deviation = radians(expected_orientation.value - self.measures.compass)
        elif expected_orientation == Orientation.S:
            orientation_deviation =  radians(abs(self.measures.compass) - expected_orientation.value)
        else: 
            orientation_deviation = radians(expected_orientation.value - self.measures.compass)
        
        return orientation_deviation

    def calculate_deviation_ir(self, ir_sensors, xt, yt, curr_orientation):
        x_dir = []
        y_dir = []
        if curr_orientation == Orientation.N:
            if ir_sensors.left > 2:
                y_dir.append(1/ir_sensors.left)
            if ir_sensors.right > 2:
                y_dir.append(1 - (1/ir_sensors.right))
            if ir_sensors.center > 2:
                x_dir.append(1/ir_sensors.center)
            #if ir_sensors.back > 2:
            #    x_dir.append(1 - (1/ir_sensors.back))
        elif curr_orientation == Orientation.W:
            if ir_sensors.left > 2:
                x_dir.append(1 - (1/ir_sensors.left))
            if ir_sensors.right > 2:
                x_dir.append(1/ir_sensors.right)
            if ir_sensors.center > 2:
                y_dir.append(1/ir_sensors.center)
            #if ir_sensors.back > 2:
            #    y_dir.append(1 - (1/ir_sensors.back))
        elif curr_orientation == Orientation.S:
            if ir_sensors.left > 2:
                y_dir.append(1 - (1/ir_sensors.left))
            if ir_sensors.right > 2:
                y_dir.append(1/ir_sensors.right)
            if ir_sensors.center > 2:
                x_dir.append(1 - (1/ir_sensors.center))
            #if ir_sensors.back > 2:
            #    x_dir.append(1/ir_sensors.back)
        else:
            if ir_sensors.left > 2:
                x_dir.append(1/ir_sensors.left)
            if ir_sensors.right > 2:
                x_dir.append(1 - (1/ir_sensors.right))
            if ir_sensors.center > 2:
                y_dir.append(1 - (1/ir_sensors.center))
            #if ir_sensors.back > 2:
            #    y_dir.append(1/ir_sensors.back)
        
        #if len(x_dir) == 0:
        #    x_dir = [0.5]
        #if len(y_dir) == 0:
        #    y_dir = [0.5]
        
        if curr_orientation == Orientation.N:
            if len(y_dir) == 0:
                y_dir = [0.5]
            if len(x_dir) > 0:
                corrections = [2.5 - sum(x_dir)/len(x_dir), 0.5 - sum(y_dir)/len(y_dir)]
            else:
                corrections = [xt, 0.5 - sum(y_dir)/len(y_dir)]
        elif curr_orientation == Orientation.S:
            if len(y_dir) == 0:
                y_dir = [0.5]
            if len(x_dir) > 0:
                corrections = [2.5 + sum(x_dir)/len(x_dir), 0.5 - sum(y_dir)/len(y_dir)]
            else:
                corrections = [xt, 0.5 - sum(y_dir)/len(y_dir)]
            
        elif curr_orientation == Orientation.W:
            if len(x_dir) == 0:
                x_dir = [0.5]
            if len(y_dir) > 0:
                corrections = [0.5 - sum(x_dir)/len(x_dir), 2.5 - sum(y_dir)/len(y_dir)]
            else:
                corrections = [0.5 - sum(x_dir)/len(x_dir), yt]
        else: 
            if len(x_dir) == 0:
                x_dir = [0.5]
            if len(y_dir) > 0:
                corrections = [0.5 - sum(x_dir)/len(x_dir), 2.5 + sum(y_dir)/len(y_dir)]
            else:
                corrections = [0.5 - sum(x_dir)/len(x_dir), yt]


        #print("correction", corrections)
        #if ir_sensors.center > 2:
        #    return 
        return corrections[0], corrections[1]
        pass

    def move_forward_odometry(self, ir_sensors):
        out_l = prev_out_l = out_r = prev_out_r = distance_covered = deg = prev_deg = 0
        curr_xt = curr_yt = prev_xt = prev_yt = 0
        deg = radians(self.measures.compass)
        curr_orientation = Orientation[degree_to_cardinal(self.measures.compass)]
        start_time = 0
        linear_pid = PID(0.30, 0, 0.02, setpoint=2, output_limits=(-0.14, 0.14))
        orientation_pid = PID(0.01, 0, 0.05, setpoint=0, output_limits=(-0.007, 0.007))

        while 2 - abs(distance_covered) > 0.02:
            #prev_out_l = out_l
            #prev_out_r = out_r
            prev_deg = deg
            prev_xt = curr_xt
            prev_yt = curr_yt

            if curr_orientation == Orientation.N:
                lin_pid = linear_pid(curr_xt)
                rot_pid = orientation_pid(0 + curr_yt)
            elif curr_orientation == Orientation.W:
                lin_pid = linear_pid(curr_yt)
                rot_pid = orientation_pid(0 - curr_xt)
            elif curr_orientation == Orientation.S:
                lin_pid = linear_pid(-curr_xt)
                rot_pid = orientation_pid(0 - curr_yt)
            else:
                lin_pid = linear_pid(-curr_yt)
                rot_pid = orientation_pid(0 + curr_xt)
            
            while time.time() - start_time < 0.05:
                pass

            l = lin_pid - rot_pid
            r = lin_pid + rot_pid

            self.driveMotors(l, r)
            start_time = time.time()
            temp_l = out_l
            temp_r = out_r
            out_l = out_t(l , prev_out_l)
            out_r = out_t(r, prev_out_r)
            prev_out_l = temp_l
            prev_out_r = temp_r

            deg = new_angle(out_l, out_r, prev_deg)

            curr_xt = xt(out_l, out_r, deg, prev_xt)
            curr_yt = yt(out_l, out_r, deg, prev_yt)
            #print("before", curr_yt)
            curr_xt, curr_yt = self.calculate_deviation_ir(ir_sensors, curr_xt, curr_yt, curr_orientation)
            #print("after", curr_yt)

            if curr_orientation == Orientation.N or curr_orientation == Orientation.S:
                distance_covered = curr_xt
            else:
                distance_covered = curr_yt
                
            ir_sensors, _, _ = self.readAndOrganizeSensors()
        
        if curr_orientation == Orientation.N:
            print("N - dist:", distance_covered)
            self.r_location.x += distance_covered
        elif curr_orientation == Orientation.W:
            print("W - dist:", distance_covered)
            self.r_location.y += distance_covered
        elif curr_orientation == Orientation.S:
            print("S - dist:", distance_covered)
            if distance_covered > 0:
                self.r_location.x += (1.02 - distance_covered)
            else:
                self.r_location.x += distance_covered
        else:
            print("E - dist:", distance_covered)
            if distance_covered > 0:
                self.r_location.y += (1.02 - distance_covered)
            else:
                self.r_location.y += distance_covered
        self.driveMotors(0, 0)
        #if self.measures.gpsReady:
            #gps_cell = self.gps2robotcell(Point(self.measures.x, self.measures.y))
            #export_loc_csv("lin030004rot001005lim0007.csv", self.measures.time, round(self.r_location.x, 3), round(self.r_location.y, 3), round(gps_cell.x, 3), round(gps_cell.y, 3))
        pass

    def rotate_c4(self, angle):
        rot_pid = PID(0.0035, 0, 0.000073, setpoint=angle, output_limits=(-0.15, 0.15))

        curr_deg = self.measures.compass

        while abs(curr_deg - rot_pid.setpoint) > 2:
            if angle == 180:
                curr_deg = abs(self.measures.compass)
            else:
                curr_deg = self.measures.compass
            rot = rot_pid(curr_deg)

            l = -rot
            r = rot

            self.driveMotors(l, r)

            self.readSensors()

    def move_test(self, ir_sensors):
        prevTime = self.measures.time
        while self.measures.time - prevTime < 20:
            err = self.calculate_deviation_odometry(ir_sensors)
            self.driveMotors(0.1 - err , 0.1 + err)
            ir_sensors, _, _ = self.readAndOrganizeSensors()

            if ir_sensors.center > 2:
                self.driveMotors(0.0, 0.0)
            
        self.driveMotors(0.0, 0.0)

    def calculate_path_costs(self):
        for i, p_path in enumerate(self.possible_paths):
            cost = 0
            cp_path = []
            for subpath in p_path:
                start = reverse_point(self.beacon_location[subpath[0]])
                dest = reverse_point(self.beacon_location[subpath[1]])
                path = astar(self.map, start, dest, True)
                cp_path = cp_path + path
                cost += len(path)
            self.path_cost[i] = cost - len(p_path)
    
    def check_if_need_optimization(self, shortest_path_index):
        cost_unknowns = self.path_cost[shortest_path_index]
        cost_known = 0
        for subpath in self.shortest_path:
            start = reverse_point(self.beacon_location[subpath[0]])
            dest = reverse_point(self.beacon_location[subpath[1]])
            path = astar(self.map, start, dest, False)
            cost_known += len(path)
        cost_known = cost_known - len(self.shortest_path)

        print("cost_know", cost_known, "cost_unk", cost_unknowns)

        if cost_known > cost_unknowns:
            return True
        else:
            return False

    def c3_move(self, ir_sensors, robot_location, destination):
        if not self.move_list:
            curr_cell = self.gps2robotcell(robot_location)
            dest_cell = destination
            
            dest_map_cell = dest_cell
            dest_map_cell = Point(dest_map_cell.y, dest_map_cell.x)

            curr_map_cell = Point(round_up_to_even(curr_cell.x), round_up_to_even(curr_cell.y))
            curr_map_cell = self.robotcell2mapcell(curr_map_cell)
            curr_map_cell = Point(curr_map_cell.y, curr_map_cell.x)

            print(f"MOVING FROM {curr_cell} -> {dest_cell}")
            
            path = astar(self.map, curr_map_cell, dest_map_cell, True)
            if not path:
                print("astar could not find a path, good luck!")
                self.desenrasca(ir_sensors, robot_location)
                return True
            self.move_list = path_to_moves(path)
            #print("Path", path)
            #print("Move to path", path_to_moves(path))
        need_mapping = False
        while self.move_list:
            _, prev_orientation = self.move_list.pop(0)
            dest_cell, orientation = self.move_list.pop(0) 
            dest_cell = self.mapcell2robotcell(dest_cell)

            if (dest_cell not in self.visited_nodes) and (dest_cell.x % 2 == 0 and dest_cell.y % 2 == 0):
                need_mapping = True
                        
            ir_sensors, _, robot_location = self.readAndOrganizeSensors()
            print(f"Going from {self.gps2robotcell(robot_location)} -> {dest_cell}")

            if self.check_if_next_is_possible(Orientation[degree_to_cardinal(self.measures.compass)], orientation, ir_sensors) and (prev_orientation == orientation):
                self.rotate_until(orientation.value)
                _, _, robot_location = self.readAndOrganizeSensors()
                print("CONTINUING TO MOVE")
                
                dest_cell = Point(round_up_to_even(dest_cell.x), round_up_to_even(dest_cell.y))
                self.move_forward(robot_location, dest_cell)
                _, _, robot_location = self.readAndOrganizeSensors()
                print("curr LOCATION", self.gps2robotcell(robot_location))
                
                if need_mapping == True:
                    print("CELL NEEDS MAPPING")
                    self.driveMotors(0.0, 0.0)
                    self.mark_walls()
                    #self.print_map_to_file("planning.out")
            else:
                #self.print_map_to_file("planning.out")
                print("NOT POSSIBLE TO MOVE; RECALCULATING")
                self.driveMotors(0,0)
                self.move_list = []
                return False

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
    
    def check_if_next_is_possible(self, curr_direction, next_direction,ir_sensors):
        threshold = 1.4
        
        if curr_direction == Orientation.N:
            if next_direction == Orientation.N:
                if ir_sensors.center >= threshold:
                    return False
            elif next_direction == Orientation.W:
                if ir_sensors.left >= threshold:
                    return False
            elif next_direction == Orientation.S:
                if ir_sensors.back >= threshold:
                    return False
            else:
                if ir_sensors.right >= threshold:
                    return False
        elif curr_direction == Orientation.W:
            if next_direction == Orientation.N:
                if ir_sensors.right >= threshold:
                    return False
            elif next_direction == Orientation.W:
                if ir_sensors.center >= threshold:
                    return False
            elif next_direction == Orientation.S:
                if ir_sensors.left >= threshold:
                    return False
            else:
                if ir_sensors.back >= threshold:
                    return False
        elif curr_direction == Orientation.S:
            if next_direction == Orientation.N:
                if ir_sensors.back >= threshold:
                    return False
            elif next_direction == Orientation.W:
                if ir_sensors.right >= threshold:
                    return False
            elif next_direction == Orientation.S:
                if ir_sensors.center >= threshold:
                    return False
            else:
                if ir_sensors.left >= threshold:
                    return False
        elif curr_direction == Orientation.E:
            if next_direction == Orientation.N:
                if ir_sensors.left >= threshold:
                    return False
            elif next_direction == Orientation.W:
                if ir_sensors.back >= threshold:
                    return False
            elif next_direction == Orientation.S:
                if ir_sensors.right >= threshold:
                    return False
            else:
                if ir_sensors.center >= threshold:
                    return False
        return True
    
    def c2_move(self, ir_sensors, robot_location):
        if not self.move_list:
            curr_cell = self.gps2robotcell(robot_location)
            dest_cell = self.nodes_to_visit.pop()
            
            dest_cell = Point(round_up_to_even(dest_cell.x), round_up_to_even(dest_cell.y))

            dest_map_cell = self.robotcell2mapcell(dest_cell)
            dest_map_cell = Point(dest_map_cell.y, dest_map_cell.x)

            curr_map_cell = Point(round_up_to_even(curr_cell.x), round_up_to_even(curr_cell.y))
            curr_map_cell = self.robotcell2mapcell(curr_map_cell)
            curr_map_cell = Point(curr_map_cell.y, curr_map_cell.x)

            print(f"MOVING FROM {curr_cell} -> {dest_cell}")
            
            path = astar(self.map, curr_map_cell, dest_map_cell)
            if not path:
                print("astar could not find a path, good luck!")
                self.desenrasca(ir_sensors, robot_location)
                return True
            self.move_list = path_to_moves(path)
        #print("Path", path)
        #print("Move to path", path_to_moves(path))
        need_mapping = False
        while self.move_list:
            _ = self.move_list.pop(0)
            dest_cell, orientation = self.move_list.pop(0)
            dest_cell = self.mapcell2robotcell(dest_cell)

            if dest_cell not in self.visited_nodes:
                need_mapping = True
            
            self.rotate_until(orientation.value)
            _, _, robot_location = self.readAndOrganizeSensors()
            dest_cell = Point(round_up_to_even(dest_cell.x), round_up_to_even(dest_cell.y))
            self.move_forward(robot_location, dest_cell)

            if need_mapping == True:
                print("CELL NEEDS MAPPING")
                return need_mapping
        return need_mapping

    def desenrasca(self, ir_sensors, robot_location):
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
        if challenge != 4:
            curr_cell = self.gps2mapcell(robot_location)
            threshold = 1.8
        else:
            curr_cell = self.robotcell2mapcell(self.curr_cell)
            threshold = 1.4
        print("MAPPING")
        
        if direction == 'N':
            if ir_sensors.back >= threshold:
                self.map[curr_cell.y][curr_cell.x-1] = '|'
            else:
                self.map[curr_cell.y][curr_cell.x-1] = 'X'
                self.map[curr_cell.y][curr_cell.x-2] = 'X'
                new_cell_to_visit = Point(curr_cell.x-2, curr_cell.y)
                self.add_to_cells_to_visit(self.mapcell2robotcell(new_cell_to_visit))

            if ir_sensors.center >= threshold:
                self.map[curr_cell.y][curr_cell.x+1] = '|'
            else:
                self.map[curr_cell.y][curr_cell.x+1] = 'X'
                self.map[curr_cell.y][curr_cell.x+2] = 'X'                
                new_cell_to_visit = Point(curr_cell.x+2, curr_cell.y)
                self.add_to_cells_to_visit(self.mapcell2robotcell(new_cell_to_visit))

            if ir_sensors.right >= threshold:
                self.map[curr_cell.y+1][curr_cell.x] = '-'
            else:
                self.map[curr_cell.y+1][curr_cell.x] = 'X'
                self.map[curr_cell.y+2][curr_cell.x] = 'X'
                new_cell_to_visit = Point(curr_cell.x, curr_cell.y+2)
                self.add_to_cells_to_visit(self.mapcell2robotcell(new_cell_to_visit))

            if ir_sensors.left >= threshold:
                self.map[curr_cell.y-1][curr_cell.x] = '-'
            else:
                self.map[curr_cell.y-1][curr_cell.x] = 'X'
                self.map[curr_cell.y-2][curr_cell.x] = 'X'
                new_cell_to_visit = Point(curr_cell.x, curr_cell.y-2)
                self.add_to_cells_to_visit(self.mapcell2robotcell(new_cell_to_visit))
            
        elif direction == "W":
            if ir_sensors.back >= threshold:
                self.map[curr_cell.y+1][curr_cell.x] = '-'
            else:
                self.map[curr_cell.y+1][curr_cell.x] = 'X'
                self.map[curr_cell.y+2][curr_cell.x] = 'X'
                new_cell_to_visit = Point(curr_cell.x, curr_cell.y+2)
                self.add_to_cells_to_visit(self.mapcell2robotcell(new_cell_to_visit))
            
            if ir_sensors.center >= threshold:
                self.map[curr_cell.y-1][curr_cell.x] = '-'
            else:
                self.map[curr_cell.y-1][curr_cell.x] = 'X'
                self.map[curr_cell.y-2][curr_cell.x] = 'X'
                new_cell_to_visit = Point(curr_cell.x, curr_cell.y-2)
                self.add_to_cells_to_visit(self.mapcell2robotcell(new_cell_to_visit))

            if ir_sensors.right >= threshold:
                self.map[curr_cell.y][curr_cell.x+1] = '|'
            else:
                self.map[curr_cell.y][curr_cell.x+1] = 'X'
                self.map[curr_cell.y][curr_cell.x+2] = 'X'
                new_cell_to_visit = Point(curr_cell.x+2, curr_cell.y)
                self.add_to_cells_to_visit(self.mapcell2robotcell(new_cell_to_visit))

            if ir_sensors.left >= threshold:
                self.map[curr_cell.y][curr_cell.x-1] = '|'
            else:
                self.map[curr_cell.y][curr_cell.x-1] = 'X'
                self.map[curr_cell.y][curr_cell.x-2] = 'X'
                new_cell_to_visit = Point(curr_cell.x-2, curr_cell.y)
                self.add_to_cells_to_visit(self.mapcell2robotcell(new_cell_to_visit))
        elif direction == "E":
            if ir_sensors.back >= threshold:
                self.map[curr_cell.y-1][curr_cell.x] = '-'
            else:
                self.map[curr_cell.y-1][curr_cell.x] = 'X'
                self.map[curr_cell.y-2][curr_cell.x] = 'X'
                new_cell_to_visit = Point(curr_cell.x, curr_cell.y-2)
                self.add_to_cells_to_visit(self.mapcell2robotcell(new_cell_to_visit))            

            if ir_sensors.center >= threshold:
                self.map[curr_cell.y+1][curr_cell.x] = '-'
            else:
                self.map[curr_cell.y+1][curr_cell.x] = 'X'
                self.map[curr_cell.y+2][curr_cell.x] = 'X'
                new_cell_to_visit = Point(curr_cell.x, curr_cell.y+2)
                self.add_to_cells_to_visit(self.mapcell2robotcell(new_cell_to_visit))
            

            if ir_sensors.right >= threshold:
                self.map[curr_cell.y][curr_cell.x-1] = '|'
            else:
                self.map[curr_cell.y][curr_cell.x-1] = 'X'
                self.map[curr_cell.y][curr_cell.x-2] = 'X'
                new_cell_to_visit = Point(curr_cell.x-2, curr_cell.y)
                self.add_to_cells_to_visit(self.mapcell2robotcell(new_cell_to_visit))
            
            if ir_sensors.left >= threshold:
                self.map[curr_cell.y][curr_cell.x+1] = '|'
            else:
                self.map[curr_cell.y][curr_cell.x+1] = 'X'
                self.map[curr_cell.y][curr_cell.x+2] = 'X'
                new_cell_to_visit = Point(curr_cell.x+2, curr_cell.y)
                self.add_to_cells_to_visit(self.mapcell2robotcell(new_cell_to_visit))
        else:
            if ir_sensors.back >= threshold:
                self.map[curr_cell.y][curr_cell.x+1] = '|'
            else:
                self.map[curr_cell.y][curr_cell.x+1] = 'X'
                self.map[curr_cell.y][curr_cell.x+2] = 'X'
                new_cell_to_visit = Point(curr_cell.x+2, curr_cell.y)
                self.add_to_cells_to_visit(self.mapcell2robotcell(new_cell_to_visit))  
            
            if ir_sensors.center >= threshold:
                self.map[curr_cell.y][curr_cell.x-1] = '|'
            else:
                self.map[curr_cell.y][curr_cell.x-1] = 'X'
                self.map[curr_cell.y][curr_cell.x-2] = 'X'
                new_cell_to_visit = Point(curr_cell.x-2, curr_cell.y)
                self.add_to_cells_to_visit(self.mapcell2robotcell(new_cell_to_visit))
            
            if ir_sensors.right >= threshold:
                self.map[curr_cell.y-1][curr_cell.x] = '-'
            else:
                self.map[curr_cell.y-1][curr_cell.x] = 'X'
                self.map[curr_cell.y-2][curr_cell.x] = 'X'
                new_cell_to_visit = Point(curr_cell.x, curr_cell.y-2)
                self.add_to_cells_to_visit(self.mapcell2robotcell(new_cell_to_visit))

            if ir_sensors.left >= threshold:
                self.map[curr_cell.y+1][curr_cell.x] = '-'
            else:
                self.map[curr_cell.y+1][curr_cell.x] = 'X'
                self.map[curr_cell.y+2][curr_cell.x] = 'X'
                new_cell_to_visit = Point(curr_cell.x, curr_cell.y+2)
                self.add_to_cells_to_visit(self.mapcell2robotcell(new_cell_to_visit))           
    
    def print_map_to_file(self, file_name="mapping"):
        fout = open(file_name+".map", "w+")
        
        for row in range(len(self.map)):
            for col in range(len(self.map[row])):
                fout.write(self.map[row][col])
            fout.write("\n")

    def print_path_info_to_file(self, file_name="planning"):
        fout = open(file_name, 'a')

        fout.write(' 0 ')
        for subpath in self.shortest_path[1:]:
            fout.write(str(subpath[0]) + ' ')
        fout.write('0')
        fout.write('\n')
        fout.write(str(self.path_cost[self.shortest_path_index]))

    def print_path_to_file(self, file_name="path.path"):
        fout = open(file_name+".path", "w+")
        path = []

        for subpath in self.shortest_path:
            start = reverse_point(self.beacon_location[subpath[0]])
            dest = reverse_point(self.beacon_location[subpath[1]])
            path = path + astar(self.map, start, dest, True)
            path.pop()
        
        for i, point in enumerate(path):
            if i % 2 == 0:
                if reverse_point(point) not in self.beacon_location.values():
                    robot_point = self.mapcell2robotcell(reverse_point(point))
                    fout.write(f"{robot_point.x} {robot_point.y}\n")
                else:
                    beacon = list(self.beacon_location.values()).index(reverse_point(point))
                    robot_point = self.mapcell2robotcell(reverse_point(point))
                    if beacon == 0:
                        fout.write(f"{robot_point.x} {robot_point.y}\n")
                    else:
                        fout.write(f"{robot_point.x} {robot_point.y} #{beacon}\n")
        fout.write("0 0")

        fout.close()

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


rob_name = "pyruetas"
host = "localhost"
pos = 1
mapc = None
challenge = 1
filename = "mapping.out"

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    elif (sys.argv[i] == "--challenge" or sys.argv[i] == "-c") and i != len(sys.argv) - 1:
        challenge = int(sys.argv[i+1])
    elif (sys.argv[i] == "--file" or sys.argv[i] == "-f") and i != len(sys.argv) - 1:
        filename = sys.argv[i+1]
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
