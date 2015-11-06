#ifndef UTILITY_DEFINED
#define UTILITY_DEFINED

#define SET_PWML(x) {md.setM1Speed(x);}
#define SET_PWMR(x) {md.setM2Speed(x);}
#define SET_PWMS(l,r) { SET_PWML(l); SET_PWMR(r);}

#define SET_BRKL(x) {md.setM1Brake(x);}
#define SET_BRKR(x) {md.setM2Brake(x);}
#define SET_BRKS(x) {SET_BRKL(x); SET_BRKR(x);}

void sendDebugPacket(const char* str);

/***********************************************
 some command handlers
***********************************************/

/***********************************************
 A2: readSensors
***********************************************/

void readSensors(long loopCount, int readMore=false){
  long sum0=0, sum1=0, sum2=0, sum3=0, sum4=0, sum5=0;
  long dsum0=0, dsum1=0, dsum2=0, dsum3=0, dsum4=0, dsum5=0;
  long old0, old1, old2, old3, old4, old5;
  
  old0 = analogRead(S_LEFT);
  old1 = analogRead(S_MIDDLE);
  old2 = analogRead(S_RIGHT);
  old3 = analogRead(S_HEAD);
  old4 = analogRead(S_LR_L);
  old5 = analogRead(S_LR_R);
  
  #define READ_MORE 512
  #define READ_LESS 64
  int read_times = readMore? READ_MORE : READ_LESS;
  for(int i = 0; i< read_times; i ++){
    int new0 = analogRead(S_LEFT);
    int new1 = analogRead(S_MIDDLE);
    int new2 = analogRead(S_RIGHT);
    int new3 = analogRead(S_HEAD);
    int new4 = analogRead(S_LR_L);
    int new5 = analogRead(S_LR_R);
    
    sum0 += new0;
    sum1 += new1;
    sum2 += new2;
    sum3 += new3;
    sum4 += new4;
    sum5 += new5;
    
    dsum0 += abs(new0 - old0);
    dsum1 += abs(new1 - old1);
    dsum2 += abs(new2 - old2);
    dsum3 += abs(new3 - old3);
    dsum4 += abs(new4 - old4);
    dsum5 += abs(new5 - old5);
    
    old0 = new0;
    old1 = new1;
    old2 = new2;
    old3 = new3;
    old4 = new4;
    old5 = new5;
  }
  
  if(readMore){
    mean0 = sum0/READ_MORE;
    mean1 = sum1/READ_MORE;
    mean2 = sum2/READ_MORE;
    mean3 = sum3/READ_MORE;
    mean4 = sum4/READ_MORE;
    mean5 = sum5/READ_MORE;
    
    dev0 = dsum0/READ_MORE;
    dev1 = dsum1/READ_MORE;
    dev2 = dsum2/READ_MORE;
    dev3 = dsum3/READ_MORE;
    dev4 = dsum4/READ_MORE;
    dev5 = dsum5/READ_MORE;
  }
  else{
    mean0 = sum0/READ_LESS;
    mean1 = sum1/READ_LESS;
    mean2 = sum2/READ_LESS;
    mean3 = sum3/READ_LESS;
    mean4 = sum4/READ_LESS;
    mean5 = sum5/READ_LESS;
    
    dev0 = dsum0/READ_LESS;
    dev1 = dsum1/READ_LESS;
    dev2 = dsum2/READ_LESS;
    dev3 = dsum3/READ_LESS;
    dev4 = dsum4/READ_LESS;
    dev5 = dsum5/READ_LESS;
  }
  
}



void performAlignment(){
  disableISR = true;
  
  while(true){
    readSensors(0, true);
    int d0 = mean0 - align0;
    int d1 = mean1 - align1;
    int d2 = mean2 - align2;
    
    // if diff too large, cannot calib.
    if(d0>150 || d2>150 || d0 < -130 || d2 < -130){
      sendDebugPacket("DIFF TOO BIG, GIVE UP");
      break;
    }
    // if too small, no need calib
    if(abs(d0) <= 1) d0=0;
    if(abs(d1) <= 1) d1=0;
    if(abs(d2) <= 1) d2=0;
    
    #define PAUSE_TIME_SHORT 20
    #define PAUSE_TIME_LONG 300
    #define LB 10
    
    // all aligned
    if(d0==0 && d2==0) // NOTE not checking middle one
      break;
    
    int speedHalf = baseSpeedLeft / 2;
    SET_PWML(d0 > 0 ? -speedHalf: speedHalf);
    SET_PWMR(d2 > 0 ? -speedHalf: speedHalf);
    if(d0 == 0) SET_PWML(0);
    if(d2 == 0) SET_PWMR(0);
    int sumTime = abs(d0) + abs(d2);
    int delayTime = 2 * sumTime;
    delayTime = constrain(delayTime, PAUSE_TIME_SHORT, PAUSE_TIME_LONG);
    delay(delayTime);
    SET_BRKS(400);
    
    delay(50);
  }
  disableISR = false;
}



/***********************************************
 simple utilities
***********************************************/

void sendDebugPacket(const char* str){
  Serial.write(0x80);
  Serial.println(str);
  Serial.write(0);
}

void resetISRCounters(){
  /*
  Reset the variables we use to track the interval between each tick
  timeLeft: time of previous left tick
  diffListLeft: a ring buffer of intervals, computed via microns()-timeLeft
  diffPtrLeft: a pointer to the head of the ring buffer
  */
  timeLeft   = timeRight   = micros();
  diffPtrLeft= diffPtrRight= 0;
  for(int i = 0; i<DIFF_LIST_SIZE; i++){
    diffListLeft[i]=0;
    diffListRight[i]=0;
  }
}

void setMotorsBackwards(unsigned left, unsigned right ){
  /*
  if(left != backwardsLeft)
    targetLeft -= 10;
  if(right!= backwardsRight)
    targetRight-= 10;
    */
  backwardsLeft = left;
  backwardsRight= right;
}

void startNextCommand(unsigned short next){
  
  int missedLeft = targetLeft - countLeft;
  int missedRight= targetRight- countRight; // float(countRight) * RIGHT_RATIO;

  switch(commandNow){
    case CM_FORWARD:
      residualLeft =  missedLeft;
      residualRight=  missedRight;
      break;
    case CM_LEFT:
      residualLeft = -missedLeft;
      residualRight=  missedRight;
      break;
    case CM_RIGHT:
      residualLeft =  missedLeft;
      residualRight= -missedRight;
      break;
    case CM_BACK:
      residualLeft = -missedLeft;
      residualRight=  missedRight;
      break;
  }
  
  #ifdef USE_COMMAND_DETAILS
  Serial.write(0x80);
  Serial.print("targetLeft: ");
  Serial.println(targetLeft);
  Serial.print("countLeft: ");
  Serial.println(countLeft);
  Serial.print("residualLeft: ");
  Serial.println(residualLeft);
  
  Serial.print("targetRight: ");
  Serial.println(targetRight);
  Serial.print("countRight: ");
  #ifdef USE_RIGHT_RATIO
  Serial.println(countRight*RIGHT_RATIO);
  #else
  Serial.println(countRight);
  #endif
  Serial.print("residualRight: ");
  Serial.println(residualRight);
  
  Serial.print("baseSpeedLeft: ");
  Serial.println(baseSpeedLeft);
  Serial.print("baseSpeedRight: ");
  Serial.println(baseSpeedRight);
  
  Serial.print("commandNow: ");
  Serial.println(commandNow);
  Serial.print("commandNext: ");
  Serial.println(commandNext);
  Serial.print("next: ");
  Serial.println(next);
  
  Serial.print("sequence length:");
  Serial.println(numCommandSequence);
  Serial.print("sequence index:");
  Serial.println(sequenceIndex);
  
  Serial.write(0x0);
  #endif
  
  
  /*
  1. clear things
    we need to clear the following things
    target, count, braking, brakingLoopCount, stayingLoopCount, aPassed, timeBegin
  2. set commandNow = next
  3. depending on commandNow, do something
  */
  
  targetLeft  = 0;
  targetRight = 0;
  countLeft   = 0;
  countRight  = 0;
  brakingLeft = 0;
  brakingRight= 0;
  brakingLoopCount=0;
  stayingLoopCount=0;
  aPassed = 0;
  timeBegin = micros();
  CEntered = false;
  brakeAheadLeft = 0;
  brakeAheadRight= 0;
  highSpeedMode = false;
  //pLeft = pRight = iLeft = iRight = dLeft = dRight = 0;
  //baseSpeedLeft = STANDARD_SPEED_LEFT;
  //baseSpeedRight= STANDARD_SPEED_RIGHT;
  
  #ifdef USE_CORRECTION
   // residuals already properly set
  #else
    residualLeft = 0;
    residualRight= 0;
  #endif
  
  commandNow = next;
  commandNext= CM_WAIT;
  commandTransit=CM_WAIT;
  
  if(sequenceIndex < numCommandSequence){
    commandNow = commandSequence[sequenceIndex];
    sequenceIndex += 1;
    if(sequenceIndex == numCommandSequence){
      commandDone = true;
    }
  }
  
  
  
  // stay: no need handle
  // wait: no need handle
  // abort: no need handle
  // error: no need handle
  // align, init_balance, init_align: not implemented
  
  // only handle FORWARD, LEFT, RIGHT, BACK. 
  // set targetLeft and targetRight
  int multiForwardStep = 0;
  if(commandNow >= CM_MULTI_FORWARD){
    multiForwardStep = commandNow - CM_MULTI_FORWARD;
    commandNow = CM_MULTI_FORWARD;
  }
  
  switch(commandNow){
    case CM_STAY:
    case CM_WAIT:
    case CM_ABORT:
    case CM_ERROR:
      SET_BRKS(400);
      break;
    case CM_INIT_BALANCE:
      targetLeft  = 8 * TICK_PER_CELL;
      targetRight = 8 * TICK_PER_CELL;
      setMotorsBackwards(false, false);
      //backwardsLeft = false;
      //backwardsRight= false;
      SET_PWMS(baseSpeedLeft,baseSpeedRight);
      break;
    case CM_INIT_ALIGN:
    case CM_ALIGN:
      break;
    case CM_MULTI_FORWARD:
      targetLeft  = ((TICK_PER_CELL+10) * multiForwardStep) -15;
      targetRight = ((TICK_PER_CELL+10) * multiForwardStep) -15;
      highSpeedMode = multiForwardStep > 2;
      setMotorsBackwards(false, false);
      //backwardsLeft = false;
      //backwardsRight= false;
      // 6V: 27,27
      brakeAheadLeft  = 110;  // 20,16 left-shake, 20,18 left-shake, 20,20 right-shake
      brakeAheadRight = 110;
      SET_PWMS(PWM_MULTI_FORWARD,PWM_MULTI_FORWARD);
      break;
      
    case CM_FORWARD:
      static int mm = 0;
      targetLeft  = 300 -1 + residualLeft;
      targetRight = 300 -2 + residualRight - mm;
      mm = !mm;
      setMotorsBackwards(false, false);
      //backwardsLeft = false;
      //backwardsRight= false;
      // for 5V: 32, 39
      brakeAheadLeft  = 42;  // 20,16 left-shake, 20,18 left-shake, 20,20 right-shake
      brakeAheadRight = 41; // 39 for 6V
      SET_PWMS(baseSpeedLeft,baseSpeedRight);
      break;
    case CM_LEFT:
      targetLeft  = TICK_PER_L-2 - residualLeft;
      targetRight = TICK_PER_R-2 + residualRight;
      setMotorsBackwards(true, false);
      //backwardsLeft = true;
      //backwardsRight= false;
      brakeAheadLeft  = 41; // 29 oversteering, 30 good enough, 31 slight understeering
      brakeAheadRight = 41;
      SET_PWMS(-baseSpeedLeft,baseSpeedRight);
      break;
    case CM_RIGHT:
      targetLeft  = TICK_PER_L +1 + residualLeft;
      targetRight = TICK_PER_R +1 - residualRight;
      setMotorsBackwards(false, true);
      //backwardsLeft = false;
      //backwardsRight= true;
      brakeAheadLeft  = 43;  // 27,26 good enough
      brakeAheadRight = 36;
      SET_PWMS(baseSpeedLeft,-baseSpeedRight);
      break;
    case CM_BACK:  
      targetLeft  = TICK_PER_BACK-4 - residualLeft;
      targetRight = TICK_PER_BACK-5 + residualRight;
      setMotorsBackwards(true, false);
      //backwardsLeft = true;
      //backwardsRight= false;
      brakeAheadLeft  = 42; // 31, oversteering
      brakeAheadRight = 34;
      SET_PWMS(-baseSpeedLeft,baseSpeedRight);
      break;
  }
}


#endif
