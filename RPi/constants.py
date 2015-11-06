
simulator_mode = False

use_exp2 = True

use_L0LL = True

bypass_exploration = False

use_064 = True

PORT = 12347


def dp(content):
    if simulator_mode:
        print content


"""
Below is a list of states that the robot undergoes
initial sequence: ready -> init_balance -> init_align
explore sequence: exp1 -> exp2 -> home
shortest sequence: shortest -> goal

A list of initiation commands are needed:

MCM_INIT_BALANCE: ready -> init_balance
MCM_INIT_ALIGN: init_balanced -> init_aligning
MCM_START_EXPLORE: init_aligned -> exp1_ing
MCM_START_SHORTEST: homed -> shortesting

In simulator mode, we don't go through ST_INIT_*, and simply step through things.
In physical mode, we wait for the 4 MCM_xxx commands to proceed

"""

NUM_SENSORS = 6

ST_READY          = "ready"
ST_INIT_BALANCING = "init_balancing"
ST_INIT_BALANCED  = 'init_balanced'
ST_INIT_ALIGNING  = 'init_aligning'
ST_INIT_ALIGNED   = 'init_aligned'
ST_EXP1_ING       = 'exp1_ing'
ST_EXP1_DONE      = 'exp1_done'
ST_EXP2_ING       = 'exp2_ing'
ST_EXP2_DONE      = 'exp2_done'
ST_HOMING         = 'homing'
ST_HOME_ALIGN     = 'home-aligning'
ST_HOMED          = 'homed'
ST_SHORTESTING    = 'shortesting'
ST_GOAL           = 'goal'

ST_ERROR_ARDUINO  = 'error_arduino'
ST_ERROR_RPI      = 'error_pi'
ST_ERROR_ABORT    = 'error_abort'


"""
Below is a list of commands.
1. Pi gives a command to arduino when necessary
2. arduino constantly report its "currently executing command"
CM_ABORT is used by pi to abort operation in arduino
CM_ERROR is used only by arduino to indicate arduino has somehow failed
"""

CM_UNKNOWN  = 0  # when connection between RPi and arduino not established
CM_FORWARD  = 1  # go forward 1 cell
CM_LEFT     = 2  # turn left
CM_RIGHT    = 3  # turn right
CM_BACK     = 4  # turn back
CM_INIT_BALANCE = 8  # initial 2-wheel balancing sequence
CM_INIT_ALIGN   = 9  # initial distance & orientation alignment sequence
CM_ALIGN    = 10  # perform distance and orientation alignment
CM_WAIT     = 11  # wait indefinitely for new commands, equivalent to RESET
CM_STAY     = 12  # stay for 0.25 seconds, then switch state to Next
CM_ABORT    = 16  # give up, just stop
CM_ERROR    = 17  # this is not a command but a status report by Arduino itself

CM_MULTI_FORWARD = 128  # go multiple steps ahead, num_step = command-128

"""
Map constants
"""
C = 0  # clean
U = 1  # unknown
O = 2  # obstacle

CU = 10  # used by 2-cell sensor, nearest C, further U
CO = 11  # used by 2-cell sensor, nearest C, further O
CC = 12  # used by 2-cell sensor, nearest C, further still C

INVALID = 32  # value of non-existing coordinate


"""
Monitor command constants
"""
MCM_REQUEST         = 0X10

MCM_INIT_BALANCE    = 0x20
MCM_INIT_ALIGN      = 0x30
MCM_ALIGN           = 0x31
MCM_START_EXPLORE   = 0x40
MCM_START_SHORTEST  = 0X50

MCM_FORWARD         = 0x61
MCM_BACK            = 0x62
MCM_LEFT            = 0x63
MCM_RIGHT           = 0x64
MCM_CM_INIT_ALIGN   = 0x65
MCM_CM_ALIGN        = 0x66
MCM_CM_INIT_BALANCE = 0x67

MCM_CM_MULTI_FORWARD= 0x80

MCM_ABORT           = 0xf0
MCM_CLOSE_ALL       = 0xff


# used by Dijkstra algorithm to indicate infinity
DISTANCE_INFINITY = 999


# algo's super-states, used to over-write arena.robot_state during the state machine choice
# insde step_algorithm()
SS_NONE         = 0
SS_ALIGN_LEFT   = 1
SS_ALIGN_RIGHT  = 2

"""
below are 2 LUTs for the sensor readings taken from the arduino sensor header file
we do translation on the RPi, not arduino so that we dont waste precious space on arduino.

TRANSFER_FUNCTION_LUT3V = [
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 79,
    76, 73, 71, 69, 67, 65, 63, 62, 60, 58, 57, 52, 50, 49, 48, 47,
    49, 48, 47, 46, 45, 44, 43, 43, 42, 41, 40, 40, 39, 38, 37, 37,
    36, 36, 35, 35, 34, 33, 33, 32, 32, 31, 31, 31, 30, 30, 29, 29,
    29, 28, 28, 27, 27, 27, 26, 26, 26, 25, 25, 25, 25, 24, 24, 24,
    23, 23, 23, 23, 22, 22, 22, 22, 22, 21, 21, 21, 21, 20, 20, 20,
    20, 20, 20, 19, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 18, 17,
    17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
    15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14,
    13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 12, 12,
    12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 11,
    11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10,
    10, 10, 10, 10, 10, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
]

TRANSFER_FUNCTION_LUT5V = [
    255, 127, 93, 77, 67, 60, 54, 50, 47, 44, 42, 40, 38, 36, 35, 34,
    32, 31, 30, 30, 29, 28, 27, 27, 26, 26, 25, 25, 24, 22, 20, 19,
    19, 18, 18, 17, 17, 17, 16, 16, 16, 15, 15, 15, 14, 14, 14, 13,
    13, 13, 13, 13, 12, 12, 12, 12, 12, 11, 11, 11, 11, 11, 11, 10,
    10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 9, 9,
    8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6,
    6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
    6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
    5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
]
"""