#ifndef ROBOT_GLOBAL_VARIABLES_DEFINED
#define ROBOT_GLOBAL_VARIABLES_DEFINED

#include "DualVNH5019MotorShield.h"

DualVNH5019MotorShield md; //motor shield object that is initialised for controlling the motors

const unsigned short
  CM_FORWARD=1,       // go forward 1 cell
  CM_LEFT=2,          // turn left
  CM_RIGHT=3,         // turn right
  CM_BACK=4,          // turn back
  CM_INIT_BALANCE=8,  // initial 2-wheel balancing sequence
  CM_INIT_ALIGN=9,    // initial distance & orientation alignment sequence
  CM_ALIGN=10,        // perform distance and orientation alignment
  CM_WAIT =11,        // wait indefinitely for new commands, equivalent to RESET
  CM_STAY =12,        // stay for 0.25 seconds, then switch state to Next
  CM_ABORT=16,        // give up, just stop
  CM_ERROR=17,        // this is not a command, but a status report by Arduino itself
  
  CM_MULTI_FORWARD = 128;

// A1 readCommand
/*
during the passage of a single cell's distance, there are 2 stages:
 ==============A----B

- During "=": robot executes commandNow
- At "A": commandTransit = commandNext
  - IF commandTransit = CM_FORWARD, increase targetLeft and targetRight, ignore B
  - ELSE enable brake detection, after braking, switch commandNow=commandTransit
- During "-": robot executes commandNow, prepare for commandTransit if it isn't forward
*/

static unsigned short commandID = 0;
static unsigned short commandNow  = CM_WAIT;
static unsigned short commandNext = CM_WAIT;
static unsigned short commandTransit=CM_WAIT;

static unsigned short commandSequence[32];
static unsigned short numCommandSequence = 0;
static unsigned short sequenceIndex = 0;
static unsigned short commandDone = false;

const unsigned int COMMAND_CUTOFF_POINT=200;
static unsigned short aPassed=0;
#define TICK_PER_CELL 288
#define TICK_PER_BACK 800
#define TICK_PER_L    400 // proper value is probably between 395-396
#define TICK_PER_R    400

// A2 readSensors
#ifdef USE_SENSOR_RING
#define SENSOR_LIST_SIZE 32
#define SENSOR_LIST_SIZE_EXP 5
static unsigned int ptr0=0,ptr1=0,ptr2=0;
static unsigned int sensorList0[SENSOR_LIST_SIZE];
static unsigned int sensorList1[SENSOR_LIST_SIZE];
static unsigned int sensorList2[SENSOR_LIST_SIZE];
#endif
static int dev0, dev1, dev2, dev3, dev4, dev5;
static unsigned int mean0=0,mean1=0,mean2=0, mean3=0, mean4=0, mean5=0;
static int align0 = 320, align1, align2 = 320, align3, align4, align5;

// A3 controlSpeed
static long timeBegin;   // micros() at the beginning of track
static unsigned short backwardsLeft = false; // if left wheel shall be rolling back
static unsigned short backwardsRight= false; // if right wheel shall be rolling back
#define STANDARD_SPEED_LEFT 150
#define STANDARD_SPEED_RIGHT 150
static float baseSpeedFloatLeft = STANDARD_SPEED_LEFT;
static float baseSpeedFloatRight= STANDARD_SPEED_RIGHT;
static int baseSpeedLeft = STANDARD_SPEED_LEFT;      // initial left speed
static int baseSpeedRight= STANDARD_SPEED_RIGHT;     // initial right speed
#define STANDARD_INTERVAL 1000
#define PWM_MULTI_FORWARD 350
static short highSpeedMode = false; // whether MULTI_FORWARD is running high-speed
static short CEntered = false;
static int timeObjective = STANDARD_INTERVAL;    // designed microseconds between each encoder tick
static unsigned long targetLeft = 0; // target count to be reached
static unsigned long targetRight= 0;
static unsigned long countLeft  = 0; // existing count already made, incremented by ISR
static unsigned long countRight = 0;
static int residualLeft = 0;
static int residualRight = 0;
#define RIGHT_RATIO 800.0/793.0


// we start braking when targetLeft - countLeft < BRAKE_PRIOR_LEFT
int brakeAheadLeft = 0;
int brakeAheadRight= 0;
// these 2 signals used to indicate that braking is in-progress, so that controlSpeed
// does not mistakenly remove the brake
static unsigned short brakingLeft = 0;
static unsigned short brakingRight= 0;
static unsigned int brakingLoopCount = 0;

static unsigned int stayingLoopCount = 0;

// A4 sendData


// B ISRs for tick counters

static short disableISR = false;

static long timeLeft = 0;
static long timeRight = 0;

static long diffL=0, diffR=0;

#define DIFF_LIST_SIZE 16
#define DIFF_LIST_SIZE_MASK 0xf
static int diffListLeft[DIFF_LIST_SIZE];
static int diffListRight[DIFF_LIST_SIZE];

static unsigned int diffPtrLeft=0;
static unsigned int diffPtrRight=0;

#endif
