import sys
import itertools
import csv

from typing import Any, NamedTuple
from collections import namedtuple
from enum import Enum
from math import cos, sin, radians

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
    PLANNING = 3
    OPTIMIZING = 4

class RobotLocation():
    def __init__(self):
        self.x = 0
        self.y = 0
        self.t = 0
        
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

def reverse_point(point):
    return Point(point.y, point.x)

def check_tuple_reverse(a, b):
    return list(a) == list(reversed(list(b)))

def remove_reversed_duplicates(iterable):
    # Create a set for already seen elements
    seen = set()
    for item in iterable:
        # Lists are mutable so we need tuples for the set-operations.
        tup = tuple(item)
        if tup not in seen:
            # If the tuple is not in the set append it in REVERSED order.
            seen.add(tup[::-1])
            # If you also want to remove normal duplicates uncomment the next line
            # seen.add(tup)
            yield item


def generatePossiblePaths(nBeacons):
    beacons = list(range(1, nBeacons))
    possible_variations = list(itertools.permutations(beacons, len(beacons)))
    possible_variations = list(remove_reversed_duplicates(possible_variations)) 
    possible_paths = []

    for variation in possible_variations:
        npath = [0]
        npath = npath + list(variation) + [0]
        possible_paths.append(list(pairwise(npath)))
    return possible_paths

# Mean noise is 1
def out_t(in_motor, prev_out_t):
    return (in_motor + prev_out_t) / 2

def lin(out_left, out_right):
    return (out_left + out_right) / 2

def xt(out_left, out_right, deg, prev_xt):
    return prev_xt + lin(out_left, out_right) * cos(deg)

def yt(out_left, out_right, deg, prev_yt):
    return prev_yt + lin(out_left, out_right) * sin(deg)   

def rot(out_left, out_right):
    return (out_left - out_right) / (2.0 * 0.5) 

def new_angle(out_left, out_right, prev_deg):
    return prev_deg + rot(out_left, out_right)

def exponential_mov_avg(sample, alpha, prev_avg):
    return (sample * alpha) + ((prev_avg) * (1-alpha));

def export_loc_csv(filename, time, x_est, y_est, x_real, y_real):
    data = [time, x_est, y_est, x_real, y_real]

    with open(filename, 'a', encoding='utf8') as f:
        writer = csv.writer(f)
        writer.writerow(data) 


class dict_to_obj(object):
    def __init__(self, dictionary):
        self.__dict__ = dictionary