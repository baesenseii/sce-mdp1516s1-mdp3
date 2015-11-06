#define DEFINED
#undef UNDEFINED

#define BRAKING_PAUSE 20

#define USE_RIGHT_RATIO
#define USE_PACKETS
#undef USE_CHECKLIST_AVOIDANCE
#undef USE_SENSOR_RING
#define USE_CORRECTION
#define USE_COMMAND_DETAILS

#define S_LEFT 2
#define S_MIDDLE 3
#define S_RIGHT 4
#define S_HEAD 5
#define S_LR_L 0
#define S_LR_R 1

#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"
#include "globalVariables.h"
#include "utility.h"
#define model 1080
#include "SharpIR.h"

 // SharpIR sharp(A3, 25, 93, model);


/***********************************************
 A1: readCommand
***********************************************/

void readCommand(long loopCount){ // 200Hz
  /*
  Most of the commands are COMMAND_BYTE + ZERO_TERMINATOR
  Whenever we receive a command, it is always immediately assigned to commandNext.
  However, when we receive an abort, it is immediately assigned to commandNow also
  */
  unsigned int length = Serial.available();
  while(length>=3){
    int data=Serial.read();
    if(data==-1){
      commandNow=CM_ERROR;
      return;
    }
    commandNext = data;
    
    // command sequence
    if(data == 0x64){
      while(true){
        length = Serial.available();
        if(length==0)
          continue;
        data = Serial.read();
        if(data != 0)
          commandSequence[sequenceIndex++] = data;
        else{
          numCommandSequence = sequenceIndex;
          sequenceIndex = 0;
          return;
        }
      }
    }
    else{
    
    // if we received ABORT, we apply it immediately
    if(commandNext==CM_ABORT)
      commandNow=CM_ABORT;
    
    commandID = Serial.read();
      
    data=Serial.read();
    if(data!=0){
      commandNow=CM_ERROR;
      return;
    }
    length -= 2;   
    } 
  }
}




/***********************************************
 A3: controlSpeed
***********************************************/

void controlSpeed(long loopCount){ // 200Hz

  long sumLeft = 0;
  long sumRight= 0;
  
  float timeDiffLeft, timeDiffRight;
  
  /*
  We initiate a new action under 2 circumstances:
  - state is waiting, and commandNow involves some sort of running
  - state is running, and is about to switch track
  */
  switch(commandNow){
    // During waiting, we constantly update commandNow to commandNext, so that
    // the moment when commandNext becomes non-waiting, it will also become commandNow
    case CM_WAIT:
      SET_BRKS(400);
      if (commandNext!=CM_WAIT){
        startNextCommand(commandNext);
      }
      return;
    case CM_STAY:
      stayingLoopCount++;
      if(stayingLoopCount>50){
        stayingLoopCount=0;
        startNextCommand(commandNext);
      }
      return;
    // if abort of error, brake and return
    case CM_ABORT:
    case CM_ERROR:
      SET_BRKL(400);
      SET_BRKR(400);
      return;
    case CM_INIT_BALANCE:
      // equilibrium speed adjustment      
      if(brakingLeft || brakingRight){
        if(brakingLeft && brakingRight){
          brakingLoopCount++;
          if(brakingLoopCount >= BRAKING_PAUSE)
            startNextCommand(CM_WAIT);
          return;
        }
        else
          return;
      }
      
      if(countLeft < 200)
        return;
      
      for(int i=0;i<DIFF_LIST_SIZE;i++){
        sumLeft += diffListLeft[i]; 
        sumRight+= diffListRight[i];
      }
      timeDiffLeft = float(sumLeft -(long int)DIFF_LIST_SIZE*timeObjective) / DIFF_LIST_SIZE / 100;
      baseSpeedFloatLeft += timeDiffLeft;
  
      timeDiffRight = float(sumRight -(long int)DIFF_LIST_SIZE*timeObjective) / DIFF_LIST_SIZE / 100;
      baseSpeedFloatRight += timeDiffRight;
      
      baseSpeedLeft = baseSpeedFloatLeft;
      baseSpeedRight= baseSpeedFloatRight;
      SET_PWML(baseSpeedLeft);
      SET_PWMR(baseSpeedRight);
      return;
    case CM_INIT_ALIGN:
      readSensors(0, true);
      align0 = mean0;
      align1 = mean1;
      align2 = mean2;
      align3 = mean3;
      align4 = mean4;
      align5 = mean5;
      startNextCommand(CM_WAIT);
      return;
    case CM_ALIGN:
      residualLeft = 0;
      residualRight= 0;
      performAlignment();
      startNextCommand(CM_WAIT);
      return;
    // if running, proceed to the remainder of the function
    case CM_FORWARD:
    case CM_LEFT:
    case CM_RIGHT:
    case CM_BACK:
    case CM_MULTI_FORWARD:
      break;
  }
  
  unsigned long time = micros();
  if(brakingLeft && brakingRight && time - timeLeft > 1000000/20 && time - timeRight > 1000000/20)
    startNextCommand(commandNext);
  
  // equilibrium-based pseudo-PID
  if(brakingLeft || brakingRight)
    return;
  
  // count diff control
  #ifdef USE_RIGHT_RATIO
  long countRightCorrected = float(countRight) * RIGHT_RATIO;
  #else
  long countRightCorrected = countRight;
  #endif
  long diff = countLeft - countRightCorrected;
  long diffSqr = sq(diff)*7/16;
  long adjustLeft = 0, adjustRight=0;
  if (diff < 0 )
    adjustLeft = diffSqr;
  else if(diff > 0)
    adjustRight = diffSqr;
  
  int speedLeft = baseSpeedLeft+adjustLeft;
  int speedRight= baseSpeedRight+adjustRight;
  if(highSpeedMode){
    int fastLeft = PWM_MULTI_FORWARD + adjustLeft;
    int fastRight= PWM_MULTI_FORWARD + adjustRight;
    /*
    ABBBBCC
    In A, use original speed
    In B, speed up to PWM_MULTI_FORWARD
    In C, down to original speed
    */
    // A
    if(countLeft < TICK_PER_CELL){
      float percentage = float(countLeft) / TICK_PER_CELL;
      speedLeft = percentage * fastLeft + (1-percentage) * speedLeft;
      speedRight= percentage * fastRight+ (1-percentage) * speedRight;
    }
    // C.3
    /*
    else if (targetLeft - countLeft < TICK_PER_CELL){
    }
    // C.1
    else if(targetLeft - countLeft < 16 * TICK_PER_CELL){
      if(diffL < (STANDARD_INTERVAL - 80) && (loopCount&0xf==0)){
        speedLeft = 0;
      }
      if(diffR < (STANDARD_INTERVAL - 80) && (loopCount&0xf==0)){
        speedRight= 0;
      }
    }
    */
    // B
    else {//if(targetLeft > (4*TICK_PER_CELL+100)){
      speedLeft = fastLeft;
      speedRight= fastRight;
    }
  }
  speedLeft = backwardsLeft? -speedLeft : speedLeft;
  speedRight= backwardsRight?-speedRight: speedRight;
  
  SET_PWML(speedLeft);
  SET_PWMR(speedRight);
  
}


/***********************************************
 A4: sendData
***********************************************/

void sendData(long loopCount){ // 50Hz
  // every one second or so, we send a synchronization sequence consisting of 8 consecutive 254's
  #ifdef USE_PACKETS
  if((loopCount&0xff)==0){
    for(int i=0;i<8;i++)
      Serial.write(254);
  }
  #endif
  /*
  When sending data, data has this format:
  PACKET_TYPE PACKET_CONTENT
    PACKET_TYPE: 
        single byte. 
        0x40: normal packet
        0x80: debug packet, see sendDebugPacket
    PACKET_CONTENT:
        normal packet: A byte with value of N, followed by N bytes 
        debug packet: 0-terminated string
  */
  
  readSensors(loopCount, false);
  
  
  #ifdef USE_PACKETS
  Serial.write(0x40); // PACKET_TYPE: normal packet
  Serial.write(28);   // length of content
  Serial.write(commandID);
  Serial.write(commandNow);
  Serial.write(commandNext);
  Serial.write(commandDone);
  Serial.write((const uint8_t*)&mean0, 2);
  Serial.write((const uint8_t*)&mean1, 2);
  Serial.write((const uint8_t*)&mean2, 2);
  Serial.write((const uint8_t*)&mean3, 2);
  Serial.write((const uint8_t*)&mean4, 2);
  Serial.write((const uint8_t*)&mean5, 2);
  Serial.write((const uint8_t*)&dev0, 2);
  Serial.write((const uint8_t*)&dev1, 2);
  Serial.write((const uint8_t*)&dev2, 2);
  Serial.write((const uint8_t*)&dev3, 2);
  Serial.write((const uint8_t*)&dev4, 2);
  Serial.write((const uint8_t*)&dev5, 2);
  #endif
  
}


/***********************************************
 B: ISRs for tick encoders
***********************************************/

void tickLeft(){
  if (disableISR) return;
  unsigned long time,diff;
  countLeft++;
  
  // time each tick
  time = micros();
  diff = time - timeLeft;
  diffL = diff;
  timeLeft = time;
  diffListLeft[diffPtrLeft++]=diff;
  diffPtrLeft &= DIFF_LIST_SIZE_MASK; 
  
  // when destination reached
  if(highSpeedMode && (targetLeft - countLeft <= 300)){
    SET_BRKL(100);
    brakingLeft = true;
  }
  
  if(int(targetLeft-countLeft) <= brakeAheadLeft){
    SET_BRKL(400);
    brakingLeft = true;
  }
}
void tickRight(){
  if (disableISR) return;
  unsigned long time, diff;
  countRight++;
  
  // time each tick
  time = micros ();
  diff = time - timeRight;
  diffR = diff;
  timeRight = time;
  diffListRight[diffPtrRight++]=diff;
  diffPtrRight &= DIFF_LIST_SIZE_MASK;
  
  // when destination reached
  #ifdef USE_RIGHT_RATIO
  int count = float(countRight)*RIGHT_RATIO;
  #else
  int count = countRight;
  #endif
  
  if(highSpeedMode && (targetRight - count <= 300)){
    SET_BRKR(100);
    brakingRight = true;
  }
  
  if(int(targetRight-count)<= brakeAheadRight){
    SET_BRKR(400);
    brakingRight=true;
  }
}



/***********************************************
 Loop
***********************************************/
void loop() {
  static long loopCount = 0;

  readCommand(loopCount);  // 200Hz
  // readSensors(loopCount);  // 200Hz
  controlSpeed(loopCount); // 200Hz
  if((loopCount&0x3)==0)   // 50Hz
    sendData(loopCount);
  
  loopCount+=1;
  // run the loop at 200Hz
  delay(5);
  
  #ifndef USE_PACKETS
  if((loopCount&0x1f)==0){ // 0x3f
    readSensors(loopCount, true);
    Serial.write(0x80);
    Serial.println();
    Serial.print("mean0 ");
    Serial.println(mean0);
    Serial.print("mean1 ");
    Serial.println(mean1);
    Serial.print("mean2 ");
    Serial.println(mean2);
    Serial.print("mean3 ");
    Serial.println(mean3);
    Serial.print("mean4 ");
    Serial.println(mean4);
    Serial.print("mean5 ");
    Serial.println(mean5);
    Serial.print("dev0 ");
    Serial.println(dev0);
    Serial.print("dev1 ");
    Serial.println(dev1);
    Serial.print("dev2 ");
    Serial.println(dev2);
    Serial.print("dev3 ");
    Serial.println(dev3);
    Serial.print("dev4 ");
    Serial.println(dev4);
    Serial.print("dev5 ");
    Serial.println(dev5);
    Serial.write(0);
  }
  #endif
}



/***********************************************
 Setup
***********************************************/

void setup() {
  // Serial operates at 115200 baud rate
  Serial.begin(115200);  
  
  // setup the motor encoder tick interrupts
  // currently pin 3 for right wheel, 11 for left
  pinMode(3, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  attachPinChangeInterrupt(3, tickLeft, RISING);
  attachPinChangeInterrupt(11,tickRight,RISING);
  
  
  // initialize motors
  md.init();
  
  // make sure the analog pins are not floating
  //pinMode(A0, OUTPUT); digitalWrite(A0, LOW);
  //pinMode(A1, OUTPUT); digitalWrite(A1, LOW);
  //pinMode(A2, OUTPUT); digitalWrite(A2, LOW);
  //pinMode(A3, OUTPUT); digitalWrite(A3, LOW);
  //pinMode(A4, OUTPUT); digitalWrite(A4, LOW);
  //pinMode(A5, OUTPUT); digitalWrite(A5, LOW);
}

