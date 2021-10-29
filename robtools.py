from typing import NamedTuple
from enum import Enum

class IRSensorData(NamedTuple):
        center: float
        left: float
        right: float
        back: float

class RobotLocationData(NamedTuple):
        x: float
        y: float
    
class Orientation(Enum):
    south = -90
    east = 0
    north = 90
    west = 180
        
class GroundStatus(Enum):
    normal = -1
    home = 0
    beacon1 = 1
    beacon2 = 2
    
class GroundSensorData(NamedTuple):
    status: GroundStatus

def SetGroundSensorData(status):
    return GroundSensorData(GroundStatus(status))