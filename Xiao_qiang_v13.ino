/*
  Left motor = M2, Right motor = M1
  Connections             Arduino Pin
  M2Encoder B output ---> pin 3
  M1Encoder B output ---> pin 5
  
  At software lab 3 arena
  m2_P = 1.08, m2_I = 0.0001, m2_D = 0.001;
  m1_P = 1.05 m1_I = 0.0001, m1_D = 0.001;
  
*/
#include <PinChangeInt.h>
#include <PID_v1.h>
#define samples 11

// initialization
int m1INA = 2;      // Motor1 Dir A
int m1INB = 4;        // Motor1 Dir B
int m1PWM = 9;       // M1PWM
int m2INA = 7;      // Motor2 Dir A
int m2INB = 8;        // Motor2 Dir B
int m2PWM = 10;       // M2PWM
int URPWM = 11; // PWM Output 0-25000US,Every 50US represent 1cm
int URCOMP= 13; // PWM trigger pin
unsigned long prev_ms = 0, sensor_cooldown_duration = 600;
unsigned long interval = 100 ; // time unit = ms
int command=0, sonarDist=0, avgCount=0, msgCount=0;
double m1_PID_input=0.0, m2_PID_input=0.0, m2DC = 0.0, m1DC = 0.0, m1SpeedAdjustment, m2SpeedAdjustment, targetSpeed = 0.0; // note: adjust target speed at the case statements
volatile int m2MovementCount=0, m1MovementCount=0, m2Ticks=0, m1Ticks=0;
double m2_P = 1.04, m2_I = 0.0001, m2_D = 0.001;
double m1_P = 1.08, m1_I = 0.0001, m1_D = 0.001;
double m2_SPF_P = 1.05, m1_SPF_P = 1.16;
String commandBuffer = "";
boolean sendSensorReading=false, explorationMode=false, startFlag=false;
int leftSideSensor[samples], leftDiagSensor[samples], rightSideSensor[samples], rightDiagSensor[samples], rightAdjustmentSensor[samples], leftAdjustmentSensor[samples];
int leftSideSensorMedian=0, leftDiagSensorMedian=0, rightDiagSensorMedian=0, rightSideSensorMedian=0, rightAdjustmentSensorMedian=0, leftAdjustmentSensorMedian;
int obstaclePositions[7];
static int lastCommand = 0; // testing
//Specify the links and initial tuning parameters
PID leftPID(&m2_PID_input, &m2SpeedAdjustment, &targetSpeed, m2_P, m2_I , m2_D, DIRECT);
PID rightPID(&m1_PID_input, &m1SpeedAdjustment, &targetSpeed, m1_P, m1_I, m1_D, DIRECT);


void setup() {
  // setting String buffer size 
  commandBuffer.reserve(400);
  
  //tell the PID to range between -800 and 800
  leftPID.SetOutputLimits(-800,800);
  rightPID.SetOutputLimits(-800,800);
  
  //turn the PID on
  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);
  
  // setup pin interrupts
  PCintPort::attachInterrupt(PIN3, &compute_m2_ticks, RISING);
  PCintPort::attachInterrupt(PIN5, &compute_m1_ticks, RISING);
  Serial.begin(9600); // set the baud rate
  
  // Establishing Communications
  while(!Serial);
  
  while(!startFlag){
    while(Serial.available()){
      char inChar = (char)Serial.read();
      if(inChar == '0'){
        explorationMode = true;
        sendSensorReading = true;
        startFlag = true;
      }
    }
  }
}

void loop(){
  switch(command){
    case 0:  // idle state, send readings once and check if the robot can auto-reposition itself
      if(sendSensorReading){
        obstacleIdentification();
        sendSensorReading = false;
        
        // if obstacle position is x1x1x, reposition robot
        if(obstaclePositions[1]==1 && obstaclePositions[3]==1){
          repositionRobotFront();
          if(obstaclePositions[2]==1){
            //if robot is too close to the wall, back away from the wall
            if(explorationMode)
              backAwayFromWall();
            else
              realignRobotCentre();
            obstacleIdentification();
            repositionRobotFront();
          }
        }
        // if right side of the robot is next to a wall
        if(obstaclePositions[4]==1 && obstaclePositions[5]==1){
          repositionRobotRightSide();
        } else if(obstaclePositions[0]==1 && obstaclePositions[6]==1){
          repositionRobotLeftSide();
        }   
      }
      // check for next command
      if(commandBuffer.length() > 1){
        command = commandBuffer.charAt(0)-48;
        sendSensorReading = true;
        commandBuffer = commandBuffer.substring(1);
      }
      else if(commandBuffer.length() == 1){
        command = commandBuffer.charAt(0)-48;
        sendSensorReading = true;
        commandBuffer = "";
      }
      else{
        command = 0;
      }
    break;
    
    case 1: // move 1 grid
      // Setting wheels to move robot forward
      digitalWrite(m1INB, HIGH);
      digitalWrite(m2INB, HIGH);
      digitalWrite(m1INA, LOW);
      digitalWrite(m2INA, LOW);
      leftPID.SetTunings(m2_P, m2_I, m2_D);
      rightPID.SetTunings(m1_P, m1_I, m1_D);
      targetSpeed = 600;
      for(m2MovementCount=0, m1MovementCount=0, m1Ticks=0, m2Ticks=0, avgCount=0; avgCount<223;){
        moveForward();
        avgCount = (m2MovementCount+m1MovementCount)/2;
      }
      // Setting wheels to brake
      digitalWrite(m1INB, LOW);
      digitalWrite(m2INB, LOW);
      digitalWrite(m1INA, LOW);
      digitalWrite(m2INA, LOW);
      analogWrite(m2PWM, 0.5*255);
      analogWrite(m1PWM, 0.5*255);
      lastCommand = command;
      msgCount = (msgCount+1)%5;
      command = 0;
      sendSensorReading = true;
    break;
    
    case 2: // rotate left 90
      rotateLeft90();
      lastCommand = command;
      msgCount = (msgCount+1)%5;
      command = 0;
      sendSensorReading = true;
    break; 
    
    case 3: // rotate right 90
      rotateRight90();
      lastCommand = command;
      msgCount = (msgCount+1)%5;
      command = 0;
      sendSensorReading = true;
    break;
    
    case 4: // left 180
      rotateLeft180();
      lastCommand = command;
      msgCount = (msgCount+1)%5;
      command = 0;
      sendSensorReading = true;
    break;
    
    case 5: // right 180
      rotateRight180();
      lastCommand = command;
      msgCount = (msgCount+1)%5;
      command = 0;
      sendSensorReading = true;
    break;
    
    case 6: // realign robot centre to be within grid, command = "A"
      repositionRobotFront();
      realignRobotCentre();
      repositionRobotFront();
      sendSensorReading = true;
      msgCount = (msgCount+1)%5;
      lastCommand = command;
      command = 0;
    break;
    
    case 7: // realign robot centre to make sure it is within 3x3 grid, command = "B"
      rotateRight90();
      repositionRobotFront();
      realignRobotCentre();
      repositionRobotFront();
      rotateLeft90();
      repositionRobotRightSide();
      sendSensorReading = true;
      msgCount = (msgCount+1)%5;
      lastCommand = command;
      command = 0;
    break;
    
    case 49: //  move 2 grid
      // Setting wheels to move robot forward
      digitalWrite(m1INB, HIGH);
      digitalWrite(m2INB, HIGH);
      digitalWrite(m1INA, LOW);
      digitalWrite(m2INA, LOW);
      targetSpeed = 600;
      leftPID.SetTunings(m2_SPF_P, m2_I, m2_D);
      rightPID.SetTunings(m1_SPF_P, m1_I, m1_D);
      for(m2MovementCount=0, m1MovementCount=0, m1Ticks=0, m2Ticks=0, avgCount=0; avgCount<500;){
        moveForward();
        avgCount = (m2MovementCount+m1MovementCount)/2;
      }
      // Setting wheels to brake
      digitalWrite(m1INB, LOW);
      digitalWrite(m2INB, LOW);
      digitalWrite(m1INA, LOW);
      digitalWrite(m2INA, LOW);
      analogWrite(m2PWM, 0.5*255);
      analogWrite(m1PWM, 0.5*255);
      command = 0;
      sendSensorReading = true;
    break;
    
    case 50: // move 3 grid
      // Setting wheels to move robot forward
      digitalWrite(m1INB, HIGH);
      digitalWrite(m2INB, HIGH);
      digitalWrite(m1INA, LOW);
      digitalWrite(m2INA, LOW);
      leftPID.SetTunings(m2_SPF_P, m2_I, m2_D);
      rightPID.SetTunings(m1_SPF_P, m1_I, m1_D);
      targetSpeed = 600;
      for(m2MovementCount=0, m1MovementCount=0, m1Ticks=0, m2Ticks=0, avgCount=0; avgCount<765;){
        moveForward();
        avgCount = (m2MovementCount+m1MovementCount)/2;
      }
      // Setting wheels to brake
      digitalWrite(m1INB, LOW);
      digitalWrite(m2INB, LOW);
      digitalWrite(m1INA, LOW);
      digitalWrite(m2INA, LOW);
      analogWrite(m2PWM, 0.5*255);
      analogWrite(m1PWM, 0.5*255);
      command = 0;
      sendSensorReading = true;
    break;
    
    case 51: // move 4 grid
      // Setting wheels to move robot forward
      digitalWrite(m1INB, HIGH);
      digitalWrite(m2INB, HIGH);
      digitalWrite(m1INA, LOW);
      digitalWrite(m2INA, LOW);
      
      targetSpeed = 600;
      leftPID.SetTunings(m2_SPF_P, m2_I, m2_D);
      rightPID.SetTunings(m1_SPF_P, m1_I, m1_D);
      for(m2MovementCount=0, m1MovementCount=0, m1Ticks=0, m2Ticks=0, avgCount=0; avgCount<1045;){
        moveForward();
        avgCount = (m2MovementCount+m1MovementCount)/2;
      }
      // Setting wheels to brake
      digitalWrite(m1INB, LOW);
      digitalWrite(m2INB, LOW);
      digitalWrite(m1INA, LOW);
      digitalWrite(m2INA, LOW);
      analogWrite(m2PWM, 0.5*255);
      analogWrite(m1PWM, 0.5*255);
      command = 0;
      sendSensorReading = true;
    break;
    
    case 52: // move 5 grid
      // Setting wheels to move robot forward
      digitalWrite(m1INB, HIGH);
      digitalWrite(m2INB, HIGH);
      digitalWrite(m1INA, LOW);
      digitalWrite(m2INA, LOW);
      
      targetSpeed = 600;
      leftPID.SetTunings(m2_SPF_P, m2_I, m2_D);
      rightPID.SetTunings(m1_SPF_P, m1_I, m1_D);
      for(m2MovementCount=0, m1MovementCount=0, m1Ticks=0, m2Ticks=0, avgCount=0; avgCount<1385;){
        moveForward();
        avgCount = (m2MovementCount+m1MovementCount)/2;
      }
      // Setting wheels to brake
      digitalWrite(m1INB, LOW);
      digitalWrite(m2INB, LOW);
      digitalWrite(m1INA, LOW);
      digitalWrite(m2INA, LOW);
      analogWrite(m2PWM, 0.5*255);
      analogWrite(m1PWM, 0.5*255);
      command = 0;
      sendSensorReading = true;
    break;
    
    case 53: // move 6 grid
      // Setting wheels to move robot forward
      digitalWrite(m1INB, HIGH);
      digitalWrite(m2INB, HIGH);
      digitalWrite(m1INA, LOW);
      digitalWrite(m2INA, LOW);
      
      targetSpeed = 600;
      leftPID.SetTunings(m2_SPF_P, m2_I, m2_D);
      rightPID.SetTunings(m1_SPF_P, m1_I, m1_D);
      for(m2MovementCount=0, m1MovementCount=0, m1Ticks=0, m2Ticks=0, avgCount=0; avgCount<1665;){
        moveForward();
        avgCount = (m2MovementCount+m1MovementCount)/2;
      }
      // Setting wheels to brake
      digitalWrite(m1INB, LOW);
      digitalWrite(m2INB, LOW);
      digitalWrite(m1INA, LOW);
      digitalWrite(m2INA, LOW);
      analogWrite(m2PWM, 0.5*255);
      analogWrite(m1PWM, 0.5*255);
      command = 0;
      sendSensorReading = true;
    break;
    
    case 54: // move 7 grid
      // Setting wheels to move robot forward
      digitalWrite(m1INB, HIGH);
      digitalWrite(m2INB, HIGH);
      digitalWrite(m1INA, LOW);
      digitalWrite(m2INA, LOW);
      
      targetSpeed = 600;
      leftPID.SetTunings(m2_SPF_P, m2_I, m2_D);
      rightPID.SetTunings(m1_SPF_P, m1_I, m1_D);
      for(m2MovementCount=0, m1MovementCount=0, m1Ticks=0, m2Ticks=0, avgCount=0; avgCount<1965;){
        moveForward();
        avgCount = (m2MovementCount+m1MovementCount)/2;
      }
      // Setting wheels to brake
      digitalWrite(m1INB, LOW);
      digitalWrite(m2INB, LOW);
      digitalWrite(m1INA, LOW);
      digitalWrite(m2INA, LOW);
      analogWrite(m2PWM, 0.5*255);
      analogWrite(m1PWM, 0.5*255);
      command = 0;
      sendSensorReading = true;
    break;
    
    case 55: // move 8 grid
      // Setting wheels to move robot forward
      digitalWrite(m1INB, HIGH);
      digitalWrite(m2INB, HIGH);
      digitalWrite(m1INA, LOW);
      digitalWrite(m2INA, LOW);
      
      targetSpeed = 600;
      leftPID.SetTunings(m2_SPF_P, m2_I, m2_D);
      rightPID.SetTunings(m1_SPF_P, m1_I, m1_D);
      for(m2MovementCount=0, m1MovementCount=0, m1Ticks=0, m2Ticks=0, avgCount=0; avgCount<2265;){
        moveForward();
        avgCount = (m2MovementCount+m1MovementCount)/2;
      }
      // Setting wheels to brake
      digitalWrite(m1INB, LOW);
      digitalWrite(m2INB, LOW);
      digitalWrite(m1INA, LOW);
      digitalWrite(m2INA, LOW);
      analogWrite(m2PWM, 0.5*255);
      analogWrite(m1PWM, 0.5*255);
      command = 0;
      sendSensorReading = true;
    break;
    
    case 56: // move 9 grid
      // Setting wheels to move robot forward
      digitalWrite(m1INB, HIGH);
      digitalWrite(m2INB, HIGH);
      digitalWrite(m1INA, LOW);
      digitalWrite(m2INA, LOW);
      
      targetSpeed = 600;
      leftPID.SetTunings(m2_SPF_P, m2_I, m2_D);
      rightPID.SetTunings(m1_SPF_P, m1_I, m1_D);
      for(m2MovementCount=0, m1MovementCount=0, m1Ticks=0, m2Ticks=0, avgCount=0; avgCount<2585;){
        moveForward();
        avgCount = (m2MovementCount+m1MovementCount)/2;
      }
      // Setting wheels to brake
      digitalWrite(m1INB, LOW);
      digitalWrite(m2INB, LOW);
      digitalWrite(m1INA, LOW);
      digitalWrite(m2INA, LOW);
      analogWrite(m2PWM, 0.5*255);
      analogWrite(m1PWM, 0.5*255);
      command = 0;
      sendSensorReading = true;
    break;
    
    case 57: // move 10 grid
      // Setting wheels to move robot forward
      digitalWrite(m1INB, HIGH);
      digitalWrite(m2INB, HIGH);
      digitalWrite(m1INA, LOW);
      digitalWrite(m2INA, LOW);
      
      targetSpeed = 600;
      leftPID.SetTunings(m2_SPF_P, m2_I, m2_D);
      rightPID.SetTunings(m1_SPF_P, m1_I, m1_D);
      for(m2MovementCount=0, m1MovementCount=0, m1Ticks=0, m2Ticks=0, avgCount=0; avgCount<2865;){
        moveForward();
        avgCount = (m2MovementCount+m1MovementCount)/2;
      }
      // Setting wheels to brake
      digitalWrite(m1INB, LOW);
      digitalWrite(m2INB, LOW);
      digitalWrite(m1INA, LOW);
      digitalWrite(m2INA, LOW);
      analogWrite(m2PWM, 0.5*255);
      analogWrite(m1PWM, 0.5*255);
      command = 0;
      sendSensorReading = true;
    break;
    
    default: command = 0; // go to idle state by default
  }
}

void serialEvent(){
  // get data from serial buffer
  while(Serial.available()){
    char inChar = (char)Serial.read();
    if(inChar == 's'){ // s indicate start of shortest path run, sensor readings will not be sent
      explorationMode = false;
    }
    else if(inChar != 's'){   
      commandBuffer += inChar;
    }
  }
  // remove whitespaces
  commandBuffer.trim();
  
}

void obstacleIdentification(){
  for(long startTime=millis();(millis()-startTime)<sensor_cooldown_duration;){
    computeMedian();
    sonarReading();
  }
  // check left side distance
  if(leftSideSensorMedian>=17 && leftSideSensorMedian<=25)
    obstaclePositions[0] = 3;
  else if(leftSideSensorMedian>=7 && leftSideSensorMedian<=16)
    obstaclePositions[0] = 2;
  else if(leftSideSensorMedian<7 && leftSideSensorMedian>0)
    obstaclePositions[0] = 1;
  else 
    obstaclePositions[0] = 0;
    
  // check left diag distance
  if(leftDiagSensorMedian>=18 && leftDiagSensorMedian<=27)
    obstaclePositions[1] = 3;
  else if(leftDiagSensorMedian>=11 && leftDiagSensorMedian<=17)
    obstaclePositions[1] = 2;
  else if(leftDiagSensorMedian<11 && leftDiagSensorMedian>0)
    obstaclePositions[1] = 1;
  else 
    obstaclePositions[1] = 0;
    
  // check right diag distance
  if(rightDiagSensorMedian>=19 && rightDiagSensorMedian<=31)
    obstaclePositions[3] = 3;
  else if(rightDiagSensorMedian>=11 && rightDiagSensorMedian<=18)
    obstaclePositions[3] = 2;
  else if(rightDiagSensorMedian<11 && rightDiagSensorMedian>0)
    obstaclePositions[3] = 1;
  else
    obstaclePositions[3] = 0;
  
  // check ultrasonic distance
  if(sonarDist>16 && sonarDist<=30)
    obstaclePositions[2] = 3;
  else if(sonarDist>=8 && sonarDist<=16)
    obstaclePositions[2] = 2;
  else if(sonarDist<=7)
    obstaclePositions[2] = 1;
  else 
    obstaclePositions[2] = 0;
    
  // check right side distance
  if(rightSideSensorMedian>=21 && rightSideSensorMedian<=24)
    obstaclePositions[4] = 3;
  else if(rightSideSensorMedian>=7 && rightSideSensorMedian<=20)
    obstaclePositions[4] = 2;
  else if(rightSideSensorMedian<7 && rightSideSensorMedian>0)
    obstaclePositions[4] = 1;
  else 
    obstaclePositions[4] = 0;
    
  // check right adjustment sensor distance
  if(rightAdjustmentSensorMedian>=17 && rightAdjustmentSensorMedian<=20)
    obstaclePositions[5] = 3;
  else if(rightAdjustmentSensorMedian>=9 && rightAdjustmentSensorMedian<=16)
    obstaclePositions[5] = 2;
  else if(rightAdjustmentSensorMedian<9 && rightAdjustmentSensorMedian>0)
    obstaclePositions[5] = 1;
  else 
    obstaclePositions[5] = 0;
    
  // check left adjustment sensor distance
  if(leftAdjustmentSensorMedian>=17 && leftAdjustmentSensorMedian<=20)
    obstaclePositions[6] = 3;
  else if(leftAdjustmentSensorMedian>=9 && leftAdjustmentSensorMedian<=16)
    obstaclePositions[6] = 2;
  else if(leftAdjustmentSensorMedian<9 && leftAdjustmentSensorMedian>0)
    obstaclePositions[6] = 1;
  else 
    obstaclePositions[6] = 0;
    
  if(sendSensorReading&&explorationMode){
    Serial.print("pc:");
    Serial.print(obstaclePositions[0]);
    Serial.print(obstaclePositions[1]);
    Serial.print(obstaclePositions[2]);
    Serial.print(obstaclePositions[3]);
    Serial.print(obstaclePositions[4]);
    Serial.print(obstaclePositions[5]);
    Serial.print(msgCount);
    Serial.println(lastCommand);
  }
}

int leftSideSensorReading(){
 return ((5000/(analogRead(4)-10))-3); 
}

int leftDiagSensorReading(){
 return ((5200/(analogRead(2)-10))-3); 
}

int rightDiagSensorReading(){
 return ((5210/(analogRead(3)-10))-3); 
}

int rightSideSensorReading(){
 return ((5000/(analogRead(5)-10))-3); 
}

int rightAdjustmentSensorReading(){
 return ((4400/(analogRead(0)+5))-3); 
}

int leftAdjustmentSensorReading(){
 return ((4600/(analogRead(1)+5))-3); 
}

void computeMedian(){
  static int sideIndex=0;
  static int diagIndex=0;
  // get reading
  leftSideSensor[sideIndex] = leftSideSensorReading();
  leftDiagSensor[diagIndex] = leftDiagSensorReading();
  rightSideSensor[sideIndex] = rightSideSensorReading();
  rightDiagSensor[diagIndex] = rightDiagSensorReading();
  rightAdjustmentSensor[diagIndex] = rightAdjustmentSensorReading();
  leftAdjustmentSensor[diagIndex] = leftAdjustmentSensorReading();
  // sort data     
  insertionSort();
  
  leftSideSensorMedian = leftSideSensor[samples/2];
  leftDiagSensorMedian = leftDiagSensor[samples/2];
  rightSideSensorMedian = rightSideSensor[samples/2];
  rightDiagSensorMedian = rightDiagSensor[samples/2];
  rightAdjustmentSensorMedian = rightAdjustmentSensor[samples/2];
  leftAdjustmentSensorMedian = leftAdjustmentSensor[samples/2];
  
  sideIndex = (sideIndex+1)%samples;
  diagIndex = (diagIndex+1)%samples;
}

void computeLeftAdjustmentMedian(){
  static int diagIndex=0;
  // get reading
  leftSideSensor[diagIndex] = leftSideSensorReading();
  leftAdjustmentSensor[diagIndex] = leftAdjustmentSensorReading();
  // sort data     
  leftAdjustmentInsertionSort();
  
  leftSideSensorMedian = leftSideSensor[samples/2];
  leftAdjustmentSensorMedian = leftAdjustmentSensor[samples/2];
  
  diagIndex = (diagIndex+1)%samples;
}

void leftAdjustmentInsertionSort(){
  for(int i=0; i<samples; i++){
    for(int j=i; j>0;j--){
      if(leftSideSensor[j]<leftSideSensor[j-1]){
        int temp = leftSideSensor[j];
        leftSideSensor[j] = leftSideSensor[j-1];
        leftSideSensor[j-1] = temp;
      }
      if(leftAdjustmentSensor[j]<leftAdjustmentSensor[j-1]){
        int temp = leftAdjustmentSensor[j];
        leftAdjustmentSensor[j] = leftAdjustmentSensor[j-1];
        leftAdjustmentSensor[j-1] = temp;
      }
    }
  } 
}

void insertionSort(){
  for(int i=0; i<samples; i++){
    for(int j=i; j>0;j--){
      if(leftSideSensor[j]<leftSideSensor[j-1]){
        int temp = leftSideSensor[j];
        leftSideSensor[j] = leftSideSensor[j-1];
        leftSideSensor[j-1] = temp;
      }
      if(leftDiagSensor[j]<leftDiagSensor[j-1]){
        int temp = leftDiagSensor[j];
        leftDiagSensor[j] = leftDiagSensor[j-1];
        leftDiagSensor[j-1] = temp;
      }
      if(rightSideSensor[j]<rightSideSensor[j-1]){
        int temp = rightSideSensor[j];
        rightSideSensor[j] = rightSideSensor[j-1];
        rightSideSensor[j-1] = temp;
      }
      if(rightDiagSensor[j]<rightDiagSensor[j-1]){
        int temp = rightDiagSensor[j];
        rightDiagSensor[j] = rightDiagSensor[j-1];
        rightDiagSensor[j-1] = temp;
      }
      if(rightAdjustmentSensor[j]<rightAdjustmentSensor[j-1]){
        int temp = rightAdjustmentSensor[j];
        rightAdjustmentSensor[j] = rightAdjustmentSensor[j-1];
        rightAdjustmentSensor[j-1] = temp;
      }
      if(leftAdjustmentSensor[j]<leftAdjustmentSensor[j-1]){
        int temp = leftAdjustmentSensor[j];
        leftAdjustmentSensor[j] = leftAdjustmentSensor[j-1];
        leftAdjustmentSensor[j-1] = temp;
      }
    }
  } 
}

void sonarReading(){
  unsigned long pulseDuration = (pulseIn(URPWM, LOW, 12000));
  if(pulseDuration<=50000 && pulseDuration>0)
    sonarDist = pulseDuration/50; // 1cm for every 50us
}

void moveForward(){
  // Setting wheels to move
  analogWrite(m1PWM, m1DC*255);
  analogWrite(m2PWM, m2DC*255);
  
  unsigned long current_ms = millis();
  
  if((current_ms - prev_ms) > interval){
    m2_PID_input = m2Ticks;
    m1_PID_input = m1Ticks;
    leftPID.Compute();
    rightPID.Compute();
    m2DC = (m2_PID_input+m2SpeedAdjustment)/1290;
    m1DC = (m1_PID_input+m1SpeedAdjustment)/1230;
    
    m2Ticks = 0;
    m1Ticks = 0;
    prev_ms = current_ms;
  }
}

void compute_m2_ticks(){
  m2Ticks++;
  m2MovementCount++;
}

void compute_m1_ticks(){
  m1Ticks++;
  m1MovementCount++;
}

void rotateLeft90(){
  m2MovementCount=0;
  m1MovementCount=0;
  avgCount=0;
  /// setting wheel direction to rotate robot left
  digitalWrite(m1INB, HIGH);
  digitalWrite(m2INB, LOW);
  digitalWrite(m1INA, LOW);
  digitalWrite(m2INA, HIGH);
  
  analogWrite(m2PWM,0.5*255);
  analogWrite(m1PWM,0.5*255);
  delay(300);
  analogWrite(m2PWM,0.2*255);
  analogWrite(m1PWM,0.2*255);
  while(avgCount<415){
    avgCount = (m2MovementCount+m1MovementCount)/2;
  }
  analogWrite(m2PWM, 0*255);
  analogWrite(m1PWM, 0*255);
}

void rotateLeft180(){
  m2MovementCount=0;
  m1MovementCount=0;
  avgCount=0;
  /// setting wheel direction to rotate robot left
  digitalWrite(m1INB, HIGH);
  digitalWrite(m2INB, LOW);
  digitalWrite(m1INA, LOW);
  digitalWrite(m2INA, HIGH);
  
  analogWrite(m2PWM,0.5*255);
  analogWrite(m1PWM,0.5*255);
  delay(900);
  analogWrite(m2PWM,0.2*255);
  analogWrite(m1PWM,0.2*255);
  while(avgCount<835){
    avgCount = (m2MovementCount+m1MovementCount)/2;
  }
  analogWrite(m2PWM, 0*255);
  analogWrite(m1PWM, 0*255);
}

void rotateRight180(){
  m2MovementCount=0;
  m1MovementCount=0;
  avgCount=0;
  /// setting wheel direction to rotate robot left
  digitalWrite(m1INB, LOW);
  digitalWrite(m2INB, HIGH);
  digitalWrite(m1INA, HIGH);
  digitalWrite(m2INA, LOW);
  
  analogWrite(m2PWM,0.5*255);
  analogWrite(m1PWM,0.5*255);
  delay(900);
  analogWrite(m2PWM,0.2*255);
  analogWrite(m1PWM,0.2*255);
  while(avgCount<845){
    avgCount = (m2MovementCount+m1MovementCount)/2;
  }
  analogWrite(m2PWM, 0*255);
  analogWrite(m1PWM, 0*255);
}

void rotateRight90(){
  m2MovementCount=0;
  m1MovementCount=0;
  avgCount=0;
  /// setting wheel direction to rotate robot left
  digitalWrite(m1INB, LOW);
  digitalWrite(m2INB, HIGH);
  digitalWrite(m1INA, HIGH);
  digitalWrite(m2INA, LOW);
  
  analogWrite(m2PWM,0.5*255);
  analogWrite(m1PWM,0.5*255);
  delay(300);
  analogWrite(m2PWM,0.2*255);
  analogWrite(m1PWM,0.2*255);
  while(avgCount<413){
    avgCount = (m2MovementCount+m1MovementCount)/2;
  }
  analogWrite(m2PWM, 0*255);
  analogWrite(m1PWM, 0*255);
}

void repositionRobotFront(){
  // compute median sensor readings to know if robot is facing straight
//  for(long start_time = millis(); (millis()-start_time)<sensor_cooldown_duration;)
//    computeMedian();
  if(leftDiagSensorMedian>rightDiagSensorMedian){  
    // set wheels to rotate right
    digitalWrite(m1INB, LOW);
    digitalWrite(m2INB, HIGH);
    digitalWrite(m1INA, HIGH);
    digitalWrite(m2INA, LOW);
    
    // start rotating
    analogWrite(m2PWM, 0.15*255);
    analogWrite(m1PWM, 0.15*255);
    
    while(leftDiagSensorMedian>rightDiagSensorMedian)
      computeMedian();
  } else if(leftDiagSensorMedian<rightDiagSensorMedian){
    // set wheels to rotate left
    digitalWrite(m1INB, HIGH);
    digitalWrite(m2INB, LOW);
    digitalWrite(m1INA, LOW);
    digitalWrite(m2INA, HIGH);
    // start rotating
    analogWrite(m2PWM, 0.16*255);
    analogWrite(m1PWM, 0.16*255);
    
    while(leftDiagSensorMedian<rightDiagSensorMedian)
      computeMedian();
  }
  analogWrite(m2PWM, 0);
  analogWrite(m1PWM, 0);
}

void repositionRobotLeftSide(){
  if(leftSideSensorMedian>leftAdjustmentSensorMedian){
    // set wheels to rotate right
    digitalWrite(m1INB, LOW);
    digitalWrite(m2INB, HIGH);
    digitalWrite(m1INA, HIGH);
    digitalWrite(m2INA, LOW);
    // start rotating
    analogWrite(m2PWM, 0.16*255);
    analogWrite(m1PWM, 0.16*255);
    while(leftSideSensorMedian>leftAdjustmentSensorMedian){
      computeLeftAdjustmentMedian();
    }
  } else if(leftSideSensorMedian<leftAdjustmentSensorMedian){
    // set wheels to rotate left
    digitalWrite(m1INB, HIGH);
    digitalWrite(m2INB, LOW);
    digitalWrite(m1INA, LOW);
    digitalWrite(m2INA, HIGH);
    // start rotating
    analogWrite(m2PWM, 0.16*255);
    analogWrite(m1PWM, 0.16*255);
    
    while(leftSideSensorMedian<leftAdjustmentSensorMedian)
        computeLeftAdjustmentMedian();
  
  }
  analogWrite(m2PWM, 0);
  analogWrite(m1PWM, 0);
}

void repositionRobotRightSide(){
  // compute median sensor readings to know if robot is facing straight
//  for(long start_time = millis(); (millis()-start_time)<(sensor_cooldown_duration+200);)
//    computeMedian();
  if(rightSideSensorMedian>rightAdjustmentSensorMedian){  
    // set wheels to rotate left
    digitalWrite(m1INB, HIGH);
    digitalWrite(m2INB, LOW);
    digitalWrite(m1INA, LOW);
    digitalWrite(m2INA, HIGH);
    // start rotating
    analogWrite(m2PWM, 0.2*255);
    analogWrite(m1PWM, 0.2*255);
    
    while(rightSideSensorMedian>rightAdjustmentSensorMedian)
      computeMedian();
  } else if(rightSideSensorMedian<rightAdjustmentSensorMedian){
    // set wheels to rotate right
    digitalWrite(m1INB, LOW);
    digitalWrite(m2INB, HIGH);
    digitalWrite(m1INA, HIGH);
    digitalWrite(m2INA, LOW);
    // start rotating
    analogWrite(m2PWM, 0.2*255);
    analogWrite(m1PWM, 0.2*255);
    
    while(rightSideSensorMedian<rightAdjustmentSensorMedian)
      computeMedian();
  } 
  analogWrite(m2PWM, 0);
  analogWrite(m1PWM, 0);
}
void backAwayFromWall(){
  // get sonar dist
  sonarReading();
  if(sonarDist<5){
    // Setting wheels to move robot backward
    digitalWrite(m1INB, LOW);
    digitalWrite(m2INB, LOW);
    digitalWrite(m1INA, HIGH);
    digitalWrite(m2INA, HIGH);
  
    analogWrite(m2PWM, 0.2*255);
    analogWrite(m1PWM, 0.2*255);
    while(sonarDist<5)
      sonarReading();
    analogWrite(m2PWM, 0);
    analogWrite(m1PWM, 0);
  }
}
void realignRobotCentre(){  // might need to use IR instead of UR if too close to the wall unless we calibrate to never move more then 1 grid
  // get sonar dist
  sonarReading();
  if(sonarDist!=4){
    if(sonarDist>4){
      // Setting wheels to move robot forward
      digitalWrite(m1INB, HIGH);
      digitalWrite(m2INB, HIGH);
      digitalWrite(m1INA, LOW);
      digitalWrite(m2INA, LOW);
    }
    else if(sonarDist<4){
      // Setting wheels to move robot backward
      digitalWrite(m1INB, LOW);
      digitalWrite(m2INB, LOW);
      digitalWrite(m1INA, HIGH);
      digitalWrite(m2INA, HIGH);
    }
    analogWrite(m2PWM, 0.2*255);
    analogWrite(m1PWM, 0.2*255);
    while(sonarDist!=4)
      sonarReading();
    analogWrite(m2PWM, 0);
    analogWrite(m1PWM, 0);
  }
}
