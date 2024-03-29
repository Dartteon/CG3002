################
#VALUE CONSTANTS
################
SPEECH_DELAY = 0.5
STRIDE_LENGTH = 60.0 # Stride length in cm
TURN_INSTRUCTION_DELAY = 2.0
WALK_INSTRUCTION_DELAY = 4.0
ANGLE_OFFSET = 0 # Direct offset to every angle reading
ANGLE_TOLERANCE = 15

####################
# PRIORITY CONSTANTS
####################
HIGH_PRIORITY = 0
MED_PRIORITY = 1
LOW_PRIORITY = 2
LOWEST_PRIORITY = 3
USELESS = 9999

class PRIORITIES:
    numberOf = 4
    (
    # HIGHEST PRIORITY
    INFO,
    # INIT,
    # DESTINATION,
    # NODE,
    STEP_HIGH,
    TURN,
    STEP_LOW,
    # LOWEST PRIORITY
    ) = range(numberOf)


##################
#BOOLEAN CONSTANTS
##################
IS_DEBUG_MODE = False

#################
#STRING CONSTANTS
#################
BASE_URL = 'http://showmyway.comp.nus.edu.sg/getMapInfo.php?Building={building}&Level={level}'
