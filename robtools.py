import sys
import itertools

from typing import Any, NamedTuple
from collections import namedtuple
from enum import Enum

class IRSensorData(NamedTuple):
        center: float
        left: float
        right: float
        back: float

class Point(NamedTuple):
        x: float
        y: float
    
class Orientation(Enum):
    E = -90
    N = 0
    W = 90
    S = 180

class RobotStates(Enum):
    MOVING = 0
    MAPPING = 1
    FINISHED = 2
        
class Ground():

    def __init__(self, NBeacons):
        init_enum_pair = [('normal', -1), ('home', 0)]
        for beacon in range(1, NBeacons):
            beacon_string = 'beacon'+ str(beacon)
            init_enum_pair.append((beacon_string, beacon))
        self.GroundStatus = Enum('GroundStatus', init_enum_pair)
    
class GroundSensorData(NamedTuple):
    status: Any

def SetGroundSensorData(status, ground_enum):
    return GroundSensorData(ground_enum(status))

class PIDController():
    def __init__(self, Kp):
        self.u = 0
        self.e = 0
        self.max_u = 0.5

        self.Kp = Kp

        Ti = sys.float_info.max
        Td = 0.0
        h = 0.050
        self.delta = 0.05

        self.K0 = Kp * (1+h/Ti+Td/h)
        self.K1 = -Kp * (1+2*Td/h)
        self.K2 = Kp*Td/h 

        self.e_m1 = 0
        self.e_m2 = 0

        self.u_m1 = 0

    def control_action(self, set_point, feedback):
        self.e = set_point - feedback

        self.u = self.u_m1 + self.K0*self.e + self.K1*self.e_m1 + self.K2*self.e_m2

        self.e_m2 = self.e_m1
        self.e_m1 = self.e
        self.e_m1 = self.u

        if self.u_m1>self.max_u:
            self.u_m1 = self.max_u
        if self.u_m1<-self.max_u:
            self.u_m1 = -self.max_u
        
        return self.u

# Based on Gist by RobertSudwarts
def degree_to_cardinal(d):
    dirs = ["N", "W", "S", "E"]
    ix = round(d/(360. / len(dirs)))
    return dirs[ix % len(dirs)]

def round_up_to_even(f):
    if f >= 0:
        return int(f) if ((abs(int(f)) % 2) == 0) else abs(int(f)) + 1
    if f < 0:
        return int(f) if ((abs(int(f)) % 2) == 0) else int(f) - 1

# From itertools documentation 
def pairwise(iterable):
    a, b = itertools.tee(iterable)
    next(b, None)
    return zip(a, b)

def path_to_moves(path):
    if len(path) <= 2:
        return None 

    node_pairs = pairwise(path)
    move_list = []

    for node in node_pairs:
        if node[1][1] > node[0][1]:
            move_list.append((Point(node[1][1], node[1][0]), Orientation.N))
        elif node[1][1] < node[0][1]:
            move_list.append((Point(node[1][1], node[1][0]), Orientation.S))
        elif node[1][0] > node[0][0]:
            move_list.append((Point(node[1][1], node[1][0]), Orientation.E))
        elif node[1][0] < node[0][0]:
            move_list.append((Point(node[1][1], node[1][0]), Orientation.W))
    
    return move_list