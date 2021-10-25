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
        angle: float
        
class GroundStatus(Enum):
    normal = -1
    home = 0
    beacon1 = 1
    beacon2 = 2
    
class GroundSensorData(NamedTuple):
    status: GroundStatus

def SetGroundSensorData(status):
    return GroundSensorData(GroundStatus(status))