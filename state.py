from enum import Enum
class State(Enum):
    MOVE_TO_WALL=1
    TURN_RIGHT=2
    TRACK_WALL=3
    TURN_LEFT=4
    MOVE_TO_GOAL=5
    END=6
