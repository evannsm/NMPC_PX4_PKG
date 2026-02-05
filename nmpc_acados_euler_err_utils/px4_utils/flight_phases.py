from enum import Enum, auto

class FlightPhase(Enum):
    HOVER  = auto()
    CUSTOM = auto()
    RETURN = auto()
    LAND   = auto()
