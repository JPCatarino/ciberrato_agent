import sys

from typing import NamedTuple
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
        
class GroundStatus(Enum):
    normal = -1
    home = 0
    beacon1 = 1
    beacon2 = 2
    
class GroundSensorData(NamedTuple):
    status: GroundStatus

def SetGroundSensorData(status):
    return GroundSensorData(GroundStatus(status))

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
