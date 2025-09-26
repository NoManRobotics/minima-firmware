/*
 * Copyright 2025 Noman Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "Wire.h"
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>

// macro to enable ROS
#define ENABLE_ROS 0

#if ENABLE_ROS
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/JointState.h>
#endif

#define MIN_PULSE_WIDTH       700
#define MAX_PULSE_WIDTH       2300
#define FREQUENCY             50
#define VACUUM_OFF      150   // pump off pulse
#define VACUUM_ON       600   // pump on pulse

// version check
#define FIRMWARE_VERSION "1.0.0"

/* ---------------------------------------------------------------------------------------- Init */
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Preferences preferences;

const int numServos = 4; // number of robot arm joints, excluding end effector
const int potPins[5] = {25, 26, 27, 14, 13}; // pin numbers for potentiometers (include end effector potentiometer)
const int mtrPins[numServos] = {15, 14, 13, 12}; // pin numbers for servos (pca9685) (only 4 joints)
const int gripperPin = 11; // pin number for gripper servo (pca9685)
const int homePositions[numServos] = {0, 0, 360, 0};
const float jointLimits[numServos][2] = {
    {-90.0f, 90.0f},   // base
    {0.0f, 170.0f},    // shoulder
    {240.0f, 360.0f},  // elbow
    {-90.0f, 90.0f}    // wrist
};
const float gear_ratio4 = 24.0f / 19.0f; // Gear ratio for joint 4 (motor teeth / output teeth = 24 / 19)

struct EOAT_Type {
    uint8_t type;          // 0: GRIPPER, 1: PEN_HOLDER, 2: VACUUM_PUMP
    uint8_t pins[4];       // multiple IO pins that the tool may use
    uint8_t pin_count;     // actual number of pins used
    int state;             // tool state (gripper: 90-180 degrees, pump: 0-1)
};
EOAT_Type currentEOAT;

// calibration offset for pulse max and min
float calibrationOffsets[numServos][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}};
float gripperOffset[2] = {370.0f, -390.0f}; // Gripper offset

const int potRange = 2705; // potentiometer range
const int servoRange = 180; // servo range

// Average Filter hyperparameters
const float EMA_a = 0.05; // initialization of EMA alpha
int EMA_W = 0;
int EMA_E = 0;
int EMA_S = 0;
int EMA_B = 0;

// Control parameters
bool isExecute = false; 
bool isRecordingOnceJoints = false;
bool isRecordingOnceTool = false;
bool isRecording = false;
bool isMicroStep = false;
bool isMoveTool = false;
bool potentiometersEnabled = false;
bool potentiometersConnected = false;

// M280 command tracking
int targetToolState = 0;

float targetAngles[numServos];
float currentAngles[numServos];  
float tmpAngles[numServos];
short rosAngles[numServos];

int time_delay = 0;
int time_elapse = 0;

// speed factors for each motor
float jointSpeedFactors[numServos] = {1.0f, 1.0f, 1.0f, 1.0f};
const unsigned long BASE_MOTION_DURATION = 1500;

// motion state of joint
struct MotionState {
    float startPos;
    float targetPos;
    unsigned long startTime;
    unsigned long duration;
    bool isMoving;
};
MotionState jointStates[numServos];

// global motion management
struct GlobalMotionState {
    bool isNewMotion;           // whether it is a new motion command
    unsigned long globalDuration;   // global unified motion time
    int activeJoints;          // number of active joints
    bool syncMode;             // sync mode (true=sync, false=independent)
} globalMotion;

int currentSpeedProfile = 1;  // by default, use NORMAL speed
const float MIN_STEP = 1.5f;     // minimum step degree

int mode = 0; // 0 for Controller mode, 1 for ROS mode

#if ENABLE_ROS
ros::NodeHandle nh;
std_msgs::Int16MultiArray str_msg2;
ros::Publisher chatter("servoarm", &str_msg2);

void servo_cb(const sensor_msgs::JointState& cmd_msg)
{
  // 直接将弧度转换为角度
  double angles[numServos+1];  // +1 for gripper
  angles[0] = radiansToDegrees(cmd_msg.position[0]);
  angles[1] = radiansToDegrees(cmd_msg.position[1]); 
  angles[2] = radiansToDegrees(cmd_msg.position[2]);
  angles[3] = radiansToDegrees(cmd_msg.position[3]);
  // 将夹爪距离(米)映射到角度(90-180度)
  angles[4] = gripperDistanceToAngle(cmd_msg.position[4]);

  // 存储角度用于发布回ROS
  for(int i = 0; i < numServos; i++) {
    rosAngles[i] = (short)angles[i];
    setServoPosition(mtrPins[i], angles[i], jointLimits[i][0], jointLimits[i][1], calibrationOffsets[i][0], calibrationOffsets[i][1]);
  }
  
  // 处理夹爪
  if(currentEOAT.type == 0) {
    currentEOAT.state = controlGripper(angles[4], false);
  }
}

ros::Subscriber<sensor_msgs::JointState> sub("joint_states", servo_cb);
#endif

/* ------------------------------------------------------------------------------ Helper Functions */

bool arePotentiometersConnected() {
  int potVals[numServos];
  potVals[3] = analogRead(potPins[3]);
  potVals[2] = analogRead(potPins[2]);
  potVals[1] = analogRead(potPins[1]);
  potVals[0] = analogRead(potPins[0]);
  
  // Check if potentiometer values are in a reasonable range
  return (potVals[3] > 1800 && potVals[3] < 2000) && 
         (potVals[2] > 300 && potVals[2] < 540) && 
         (potVals[1] > 3100 && potVals[1] < 3300) && 
         (potVals[0] > 1700 && potVals[0] < 2100);
}

// Convert radians to degreees
double radiansToDegrees(float position_radians)
{
  position_radians = position_radians * 57.2958;
  return position_radians;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setServoPosition(int pinNum, float position, float lowerLimit, float upperLimit, float minPulseOffset, float maxPulseOffset) {
  // joint4 inverse direction and gear ratio compensation
  float actualLower = lowerLimit;
  float actualUpper = upperLimit;
  
  if (pinNum == 12) {
    position = -position * gear_ratio4;
    // 调整映射范围以匹配变换后的角度范围
    actualLower = -upperLimit * gear_ratio4;  // 注意：上下限交换因为有负号
    actualUpper = -lowerLimit * gear_ratio4;
  }
  
  float pulse_wide = mapFloat(position, actualLower, actualUpper, 
                      MIN_PULSE_WIDTH + minPulseOffset, MAX_PULSE_WIDTH + maxPulseOffset);
  
  // 添加脉宽边界检查
  pulse_wide = constrain(pulse_wide, MIN_PULSE_WIDTH - 200, MAX_PULSE_WIDTH + 200);
  
  int pulse_width = int(pulse_wide / 1000000 * FREQUENCY * 4096);
  
  // 添加PWM值边界检查
  pulse_width = constrain(pulse_width, 0, 4095);
  
  pwm.setPWM(pinNum, 0, pulse_width);
}

// sin² interpolation
float smoothInterpolation(float start, float target, float progress) {
    if (progress <= 0.0f) return start;
    if (progress >= 1.0f) return target;
    
    float smoothProgress = sin(progress * PI / 2.0f);
    smoothProgress = smoothProgress * smoothProgress;
    
    // calculate angle difference (considering shortest path)
    float angleDiff = target - start;
    while (angleDiff > 180) angleDiff -= 360;
    while (angleDiff < -180) angleDiff += 360;
    
    return start + angleDiff * smoothProgress;
}

unsigned long calculateMotionDuration(float angleChange, int jointId) {
    float absChange = abs(angleChange);
    // calculate the basic time based on the angle change, then apply the speed factor of the joint
    unsigned long duration = (unsigned long)(BASE_MOTION_DURATION * (absChange / 90.0f));
    if (jointId >= 0 && jointId < numServos) {
        return (unsigned long)(duration / jointSpeedFactors[jointId]);
    }
    return duration;
}

// calculate the maximum motion time of all active joints
unsigned long calculateGlobalMotionDuration(float targetAngles[], float currentAngles[]) {
    unsigned long maxDuration = 0;
    int activeCount = 0;
    
    for (int i = 0; i < numServos; i++) {
        float angleChange = abs(targetAngles[i] - currentAngles[i]);
        if (angleChange >= MIN_STEP) {
            activeCount++;
            unsigned long duration = (unsigned long)(BASE_MOTION_DURATION * (angleChange / 90.0f));
            duration = (unsigned long)(duration / jointSpeedFactors[i]);
            if (duration > maxDuration) {
                maxDuration = duration;
            }
        }
    }
    
    globalMotion.activeJoints = activeCount;
    globalMotion.syncMode = (activeCount > 1); // multi-joint sync, single-joint independent
    
    return maxDuration;
}

float moveMotor(float targetDegree, float curDegree, int id, bool fake) {
    if(id >= 0 && id < numServos) {
        
        if(abs(targetDegree - curDegree) < MIN_STEP) {
            setServoPosition(mtrPins[id], targetDegree, jointLimits[id][0], jointLimits[id][1], calibrationOffsets[id][0], calibrationOffsets[id][1]);
            jointStates[id].isMoving = false;
            return targetDegree;
        }
        
        if(!jointStates[id].isMoving) {
            jointStates[id].startPos = curDegree;
            jointStates[id].targetPos = targetDegree;
            jointStates[id].startTime = millis();
            
            // use global unified time or independent time
            if (globalMotion.syncMode && globalMotion.globalDuration > 0) {
                jointStates[id].duration = globalMotion.globalDuration;
            } else {
                float angleChange = abs(targetDegree - curDegree);
                jointStates[id].duration = (unsigned long)(BASE_MOTION_DURATION * (angleChange / 90.0f) / jointSpeedFactors[id]);
            }
            
            jointStates[id].isMoving = true;
        }
        
        unsigned long currentTime = millis();
        unsigned long elapsedTime = currentTime - jointStates[id].startTime;
        float progress = constrain((float)elapsedTime / jointStates[id].duration, 0.0f, 1.0f);
        
        float newPosition = smoothInterpolation(
            jointStates[id].startPos,
            jointStates[id].targetPos,
            progress
        );
        
        if (!fake) {
            setServoPosition(mtrPins[id], newPosition, jointLimits[id][0], jointLimits[id][1], calibrationOffsets[id][0], calibrationOffsets[id][1]);
        }
        
        if(progress >= 1.0f) {
            jointStates[id].isMoving = false;
        }
        
        return newPosition;
    }
    return curDegree;  // if id is invalid, return current position
}

int moveMotorPot(int pinNum) {
  int pulseWide, pulseWidth, potVal;

  // read potentiometer value
  potVal = analogRead(potPins[pinNum]);

  // apply filter
  switch (potPins[pinNum]) {
    case 14: EMA_W = (EMA_a * potVal) + ((1 - EMA_a) * EMA_W); potVal = EMA_W; break;
    case 27: EMA_E = (EMA_a * potVal) + ((1 - EMA_a) * EMA_E); potVal = EMA_E; break;
    case 26: EMA_S = (EMA_a * potVal) + ((1 - EMA_a) * EMA_S); potVal = EMA_S; break;
    case 25: EMA_B = (EMA_a * potVal) + ((1 - EMA_a) * EMA_B); potVal = EMA_B; break;
  }

  // map the potentiometer position to the motor with calibration offsets applied
  if (mtrPins[pinNum] == 13) {
    pulseWide = map(potVal, 3300, 500, MIN_PULSE_WIDTH + calibrationOffsets[pinNum][0], MAX_PULSE_WIDTH + calibrationOffsets[pinNum][1]);
  } else {
    pulseWide = map(potVal, 3300, 595, MIN_PULSE_WIDTH + calibrationOffsets[pinNum][0], MAX_PULSE_WIDTH + calibrationOffsets[pinNum][1]);
  }
  
  // 添加脉宽边界检查
  pulseWide = constrain(pulseWide, MIN_PULSE_WIDTH - 200, MAX_PULSE_WIDTH + 200);
  
  pulseWidth = int(float(pulseWide) / 1000000 * FREQUENCY * 4096);
  
  // 添加PWM值边界检查
  pulseWidth = constrain(pulseWidth, 0, 4095);
  
  pwm.setPWM(mtrPins[pinNum], 0, pulseWidth);
  return pulseWidth;
}

int controlGripper(float state, bool fake) {
    float angle = constrain(state, 90.0f, 135.0f);
    int io = currentEOAT.pins[0];
    if (!fake) {
      setServoPosition(io, angle, 90.0f, 135.0f, gripperOffset[0], gripperOffset[1]);
    }

    return (int)angle;
}

// Convert gripper distance to motor angle for M280 command
int gripperDistanceToAngle(float distance) {
    return (int)(90 + (distance / 0.008f) * 45);
}

// Convert gripper motor angle back to original distance value for M280 command
float gripperAngleToDistance(int motorAngle) {
    return ((float)(motorAngle - 90) / 45.0f) * 0.008f;
}

int controlPump(int state, bool fake) {
    // pin[0] is suction channel, pin[1] is release channel
    uint8_t suctionPin = currentEOAT.pins[0];
    uint8_t releasePin = currentEOAT.pins[1];
    
    if (currentEOAT.state == state) {
      return state;
    }

    if (!fake) {
      if (state == 1) {
          // suction mode
          setServoPosition(suctionPin, 180.0f, 0.0f, 180.0f, 0.0f, 0.0f);
          setServoPosition(releasePin, 0.0f, 0.0f, 180.0f, 0.0f, 0.0f);
          
      } else if (state == 0) {
          setServoPosition(suctionPin, 0.0f, 0.0f, 180.0f, 0.0f, 0.0f);
          setServoPosition(releasePin, 180.0f, 0.0f, 180.0f, 0.0f, 0.0f);
          delay(50);
          setServoPosition(releasePin, 0.0f, 0.0f, 180.0f, 0.0f, 0.0f);
      }
    }

    return state;
}

// save calibration offsets
void saveCalibrationOffsets() {
  preferences.begin("robot-arm", false); // open namespace
  for (int i = 0; i < numServos; i++) {
    char keyMin[16], keyMax[16];
    sprintf(keyMin, "offset_%d_min", i);
    sprintf(keyMax, "offset_%d_max", i);
    preferences.putFloat(keyMin, calibrationOffsets[i][0]);
    preferences.putFloat(keyMax, calibrationOffsets[i][1]);
  }
  preferences.end();
}

void loadCalibrationOffsets() {
  preferences.begin("robot-arm", true); // open in read-only mode
  for (int i = 0; i < numServos; i++) {
    char keyMin[16], keyMax[16];
    sprintf(keyMin, "offset_%d_min", i);
    sprintf(keyMax, "offset_%d_max", i);
    calibrationOffsets[i][0] = preferences.getFloat(keyMin, 0.0f); // default value is 0
    calibrationOffsets[i][1] = preferences.getFloat(keyMax, 0.0f); // default value is 0
  }
  preferences.end();
}

// set calibration offset
void setCalibrationOffset(int joint, float minOffset, float maxOffset) {
  if (joint >= 0 && joint < numServos) {
    calibrationOffsets[joint][0] = minOffset;
    calibrationOffsets[joint][1] = maxOffset;
    // save to EEPROM
    saveCalibrationOffsets();
  }
}

// get current pulse width for a joint
int getCurrentPulseWidth(int joint) {
  if (joint >= 0 && joint < numServos) {
    float position = currentAngles[joint];
    float lowerLimit = jointLimits[joint][0];
    float upperLimit = jointLimits[joint][1];
    
    // joint4 inverse direction and gear ratio compensation
    if (mtrPins[joint] == 12) {
      position = -position * gear_ratio4;
      // 调整映射范围以匹配变换后的角度范围
      float actualLower = -upperLimit * gear_ratio4;  
      float actualUpper = -lowerLimit * gear_ratio4;
      lowerLimit = actualLower;
      upperLimit = actualUpper;
    }
    
    float pulse_wide = mapFloat(position, lowerLimit, upperLimit, 
                        MIN_PULSE_WIDTH + calibrationOffsets[joint][0], 
                        MAX_PULSE_WIDTH + calibrationOffsets[joint][1]);
    
    // 添加边界检查
    pulse_wide = constrain(pulse_wide, MIN_PULSE_WIDTH - 200, MAX_PULSE_WIDTH + 200);
    
    return int(pulse_wide);
  }
  return 0;
}

/* ---------------------------------------------------------------------------------------- Main */

void setup() {
  delay(2000);
  
  Serial.begin(115200);
  
  // Setup PWM Controller object
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);

  delay(1000);

  // load calibration offsets from EEPROM
  loadCalibrationOffsets();

  EMA_W = analogRead(potPins[3]);
  EMA_E = analogRead(potPins[2]);
  EMA_S = analogRead(potPins[1]);
  EMA_B = analogRead(potPins[0]);
  
  // Setup end effector potentiometer pin
  pinMode(potPins[4], INPUT_PULLUP);
  
  // initialize end effector (default configuration is gripper)
  currentEOAT.type = 0;
  currentEOAT.pins[0] = gripperPin;
  currentEOAT.pin_count = 1;
  targetToolState = 90;
  currentEOAT.state = controlGripper(targetToolState, false);

  potentiometersConnected = arePotentiometersConnected();
  
  for(int i = 0; i < numServos; i++) {
    currentAngles[i] = homePositions[i];
    targetAngles[i] = homePositions[i];
    tmpAngles[i] = homePositions[i];
    rosAngles[i] = homePositions[i];
    jointStates[i].startPos = homePositions[i];
    jointStates[i].targetPos = homePositions[i];
    jointStates[i].isMoving = false;
    setServoPosition(mtrPins[i], homePositions[i], jointLimits[i][0], jointLimits[i][1], calibrationOffsets[i][0], calibrationOffsets[i][1]);
  }
  
  mode = 0; // Start in Controller mode
  
  // initialize global motion state
  globalMotion.isNewMotion = false;
  globalMotion.globalDuration = 0;
  globalMotion.activeJoints = 0;
  globalMotion.syncMode = true;

  #if ENABLE_ROS
  str_msg2.data_length = numServos+1;
  str_msg2.data = (int16_t *)malloc(sizeof(int16_t) * (numServos+1));
  #endif
}
 
void loop() {
  String command;
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    if (command == "VERC") {
      String chipModel = ESP.getChipModel();
      uint32_t freeHeap = ESP.getFreeHeap() / 1024.0;
      uint32_t cpuFreq = ESP.getCpuFreqMHz();
      
      Serial.println("INFOS");
      Serial.println("VER," + String(FIRMWARE_VERSION));
      Serial.println("Chip Model: " + chipModel);
      Serial.println("Free Mem: " + String(freeHeap) + " KB");
      Serial.println("CPU Freq: " + String(cpuFreq) + " MHz");
      Serial.println("INFOE");
    } else if (command.startsWith("CALIBRATE,")) {
      // format: CALIBRATE,joint,minOffset,maxOffset
      int firstComma = command.indexOf(',');
      int secondComma = command.indexOf(',', firstComma + 1);
      int thirdComma = command.indexOf(',', secondComma + 1);
      
      if (firstComma != -1 && secondComma != -1 && thirdComma != -1) {
        int joint = command.substring(firstComma + 1, secondComma).toInt();
        float minOffset = command.substring(secondComma + 1, thirdComma).toFloat();
        float maxOffset = command.substring(thirdComma + 1).toFloat();
        
        setCalibrationOffset(joint, minOffset, maxOffset);
        
        for (int i = 0; i < numServos; i++) {
          setServoPosition(mtrPins[i], targetAngles[i], jointLimits[i][0], jointLimits[i][1], calibrationOffsets[i][0], calibrationOffsets[i][1]);
          currentAngles[i] = targetAngles[i];
        }
      }
    } else if (command.startsWith("DELAY,")) {
      String param = command.substring(6);
      if (param.startsWith("S")) { 
          float seconds = param.substring(1).toFloat();
          time_delay = int(seconds * 1000);
      } else if (param.startsWith("MS")) {
          time_delay = param.substring(2).toInt();
      }
      delay(time_delay);
    } else if (command == "RECSTART") {
      isRecording = true;
    } else if (command == "RECSTOP") {
      isRecording = false;
      isRecordingOnceJoints = false;
      isRecordingOnceTool = false;
      
      // reset targetAngles to current angles to prevent accidental movement
      for (int i = 0; i < numServos; i++) {
        currentAngles[i] = targetAngles[i];
      }
    } else if (command.startsWith("REP,")) {
      int startIndex = command.indexOf(',') + 1;
      for (int i = 0; i < numServos; i++) {
        int nextIndex = command.indexOf(',', startIndex);
        if (nextIndex == -1 && i < numServos - 1) {
            return; // incorrect data format
        }
        String angleStr = (nextIndex == -1) ? command.substring(startIndex) : command.substring(startIndex, nextIndex);
        targetAngles[i] = angleStr.toInt();
        startIndex = nextIndex + 1;
      }
      
      isMicroStep = true;
    } else if (command == "POTOFF") {
      potentiometersEnabled = false;
      Serial.println("POT0");
    } else if (command == "POTON") {
      potentiometersEnabled = true;
      Serial.println("POT1");
    } else if (command == "TOROS") {
      mode = 1;

      #if ENABLE_ROS
      // initialise ROS
      nh.getHardware()->setBaud(115200);
      nh.initNode();
      delay(2000);
      nh.subscribe(sub);
      nh.advertise(chatter);
      delay(2000);
      #endif
    } else if (command == "TOCTR") {
      mode = 0;
    } else if (command.startsWith("TOOL[GRIPPER]")) {
      currentEOAT.type = 0;
      currentEOAT.pin_count = 1;
      currentEOAT.state = 90;

      int startIndex = command.indexOf(',') + 1;  // skip TOOL_GRIPPER,
      String io_str = command.substring(startIndex);
      if(io_str.startsWith("IO")) {
          int pin = io_str.substring(2).toInt();
          currentEOAT.pins[0] = pin;
          currentEOAT.pin_count = 1;
      }
      
      Serial.println("CP2");
    } else if (command.startsWith("TOOL[PEN]")) {  // pen holder does not need IO
      currentEOAT.type = 1;
      currentEOAT.pin_count = 0;
      currentEOAT.state = 0;
      
      Serial.println("CP2");
    } else if (command.startsWith("TOOL[PUMP]")) {
      currentEOAT.type = 2;
      currentEOAT.pin_count = 0;
      currentEOAT.state = 0;

      int startIndex = command.indexOf(',') + 1;  // skip TOOL_VACUUM_PUMP,
      while (startIndex < command.length()) {
          int endIndex = command.indexOf(',', startIndex);
          if (endIndex == -1) endIndex = command.length();
          
          String io_str = command.substring(startIndex, endIndex);
          if(io_str.startsWith("IO")) {
              int pin = io_str.substring(2).toInt();
              currentEOAT.pins[currentEOAT.pin_count++] = pin;  // use actual GPIO pin number
          }
          
          startIndex = endIndex + 1;
      }
      
      Serial.println("CP2");
    } else if (command.startsWith("M280")) {
      // format: M280,<value1>[,<value2>] where value1 and value2 are optional
      int firstComma = command.indexOf(',');
      if (firstComma != -1) {
        int secondComma = command.indexOf(',', firstComma + 1);
        
        if (secondComma != -1) {
          // there are two values: M280,value1,value2
          float value1 = command.substring(firstComma + 1, secondComma).toFloat();
          float value2 = command.substring(secondComma + 1).toFloat();
          
          if (currentEOAT.type == 0) { // gripper mode
            // for gripper, use the first value (usually the same) and convert to motor angle
            targetToolState = gripperDistanceToAngle(value1);
          } else if (currentEOAT.type == 2) { // pump mode
            // for pump, use the first value as the state
            targetToolState = (int)value1;
          }
        } else {
          // only one value case: M280,value
          float value = command.substring(firstComma + 1).toFloat();
          
          if (currentEOAT.type == 0) { // gripper mode
            // for gripper, convert the slider length to motor angle
            targetToolState = gripperDistanceToAngle(value);
          } else if (currentEOAT.type == 2) { // pump mode
            // for pump, use the value as the state
            targetToolState = (int)value;
          } else {
            // other mode, use the value directly
            targetToolState = (int)value;
          }
        }
        
        isMoveTool = true;
      }
    } else if (command == "EXEC" && !potentiometersEnabled) {
      String command = Serial.readStringUntil('\n');
      // Process the incoming command as comma-separated PWM values
      int startIndex = 0;
      for (int i = 0; i < numServos; i++) {
        int nextIndex = command.indexOf(',', startIndex);
        if (nextIndex == -1 && i < numServos - 1) {
            return; // Improper formatting of incoming data
        }
        String inputStr = (nextIndex == -1) ? command.substring(startIndex) : command.substring(startIndex, nextIndex);

        targetAngles[i] = inputStr.toFloat();
        startIndex = nextIndex + 1;
      }
      
      // calculate global motion parameters
      globalMotion.globalDuration = calculateGlobalMotionDuration(targetAngles, currentAngles);
      globalMotion.isNewMotion = true;
      
      isExecute = true;

    } else if (command == "RECONCEJ") {
      isRecordingOnceJoints = true;
    } else if (command == "RECONCET") {
      isRecordingOnceTool = true;
    } else if (command.startsWith("SPD,")) {
        // format: SPD,J1:0.2,J2:0.3,J4:0.1 to set speed factors for specific joints
        String paramStr = command.substring(4); // skip "SPD," prefix
        
        // parse comma-separated joint speed settings
        int startIndex = 0;
        while (startIndex < paramStr.length()) {
            int endIndex = paramStr.indexOf(',', startIndex);
            if (endIndex == -1) endIndex = paramStr.length();
            
            String jointSpeedStr = paramStr.substring(startIndex, endIndex);
            jointSpeedStr.trim(); // remove spaces
            
            // parse format like "J1:0.2"
            int colonIndex = jointSpeedStr.indexOf(':');
            if (colonIndex != -1) {
                String jointStr = jointSpeedStr.substring(0, colonIndex);
                String speedStr = jointSpeedStr.substring(colonIndex + 1);
                
                // extract joint number (J1 -> 0, J2 -> 1, etc.)
                if (jointStr.startsWith("J") && jointStr.length() > 1) {
                    int jointId = jointStr.substring(1).toInt() - 1; // J1=0, J2=1, etc.
                    float speed = speedStr.toFloat();
                    
                    // verify joint ID and speed value
                    if (jointId >= 0 && jointId < numServos && speed > 0.0f) {
                        jointSpeedFactors[jointId] = speed;
                    }
                }
            }
            
            startIndex = endIndex + 1;
        }
    } else if (command.startsWith("GPULSE,")) {
      // format: GPULSE,J1 to get the current pulse value of joint 1
      String jointStr = command.substring(7); // skip "GPULSE," prefix
      if (jointStr.startsWith("J") && jointStr.length() > 1) {
        int jointId = jointStr.substring(1).toInt() - 1; // J1=0, J2=1, etc.
        if (jointId >= 0 && jointId < numServos) {
          int pulseWidth = getCurrentPulseWidth(jointId);
          // send a single command response
          Serial.println("PULSE,J" + String(jointId + 1) + "," + String(pulseWidth));
        }
      }
    }
  }

  if (mode == 0) {
    if (potentiometersEnabled && potentiometersConnected) {
      if (isMicroStep) {
        for (int i = 0; i < numServos; i++) {
          pwm.setPWM(mtrPins[i], 0, targetAngles[i]);
          currentAngles[i] = targetAngles[i];
        }
        Serial.println("CP1");
        isMicroStep = false;
      } else {
        for (int i = 0; i < numServos; i++) {
          currentAngles[i] = moveMotorPot(i);
        }

        // Handle end effector potentiometer separately (index 4)
        int pushButton = digitalRead(potPins[4]);
        int previousToolState = currentEOAT.state; // record previous state
        
        if (currentEOAT.type == 0) { // gripper
          if (pushButton == LOW) {
            currentEOAT.state = controlGripper(135, false);
          } else {
            currentEOAT.state = controlGripper(90, false);
          }
        } else if (currentEOAT.type == 2) { // vacuum pump
          if (pushButton == LOW) {
            currentEOAT.state = controlPump(1, false); // turn on suction
          } else {
            currentEOAT.state = controlPump(0, false); // turn off suction
          }
        }
        
        // check if tool state changed (for potentiometer mode recording)
        bool toolStateChanged = (previousToolState != currentEOAT.state);

        // recording callbacks
        if (isRecording) {
          Serial.print("REC,");  // add REC verification
          for (int i = 0; i < numServos; i++) {
            int angle = map(currentAngles[i], MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, int(jointLimits[i][0]), int(jointLimits[i][1]));
            Serial.print(angle);
            if (i < numServos - 1) {
              Serial.print(",");
            } else {
              Serial.println();
            }
          }

          if (toolStateChanged) {
            Serial.print("M280,");
            if (currentEOAT.type == 0) { // gripper - convert back to distance
              Serial.println(gripperAngleToDistance(currentEOAT.state));
            } else {
              Serial.println(currentEOAT.state);
            }
          }
        }
      }
    } else {
      bool allServosDone = true;
      bool isMoveToolDone = false;
      
      // switch case
      if (isMicroStep) {                            // REP
        for (int i = 0; i < numServos; i++) {
          setServoPosition(mtrPins[i], targetAngles[i], jointLimits[i][0], jointLimits[i][1], calibrationOffsets[i][0], calibrationOffsets[i][1]);
          currentAngles[i] = targetAngles[i]; // update current angle
        }
        Serial.println("CP1");
        isMicroStep = false;
      } else if (isExecute) {                             // EXEC
        if (isRecording) {
          // during recording, use tmpAngles
          for (int i = 0; i < numServos; i++) {
            tmpAngles[i] = moveMotor(targetAngles[i],tmpAngles[i],i,false);
          }
        } else {
          // normal execution mode
          for (int i = 0; i < numServos; i++) {
            currentAngles[i] = moveMotor(targetAngles[i],currentAngles[i],i,false);
          }
        }
      } else if (isMoveTool) {                            // M280
        if (currentEOAT.type == 0) {
          currentEOAT.state = controlGripper(targetToolState, false);
        } else if (currentEOAT.type == 2) {
          currentEOAT.state = controlPump(targetToolState, false);
        }
      } else if (isRecordingOnceJoints) {                 // RECONCEJ
        for (int i = 0; i < numServos; i++) {
          currentAngles[i] = moveMotor(targetAngles[i],currentAngles[i],i,true);
        }
      } else if (isRecordingOnceTool) {                   // RECONCET
        if (currentEOAT.type == 0) {
          currentEOAT.state = controlGripper(targetToolState, true);
        } else if (currentEOAT.type == 2) {
          currentEOAT.state = controlPump(targetToolState, true);
        }
      }

      // check if all joint motors have reached target angles
      for (int i = 0; i < numServos; i++) {
        if (jointStates[i].isMoving) {
          allServosDone = false;
          break;
        }
      }

      // check if tool has reached target state
      if (currentEOAT.state == targetToolState) {
        isMoveToolDone = true;
        if (isMoveTool) {
          Serial.println("TP0");
          isMoveTool = false;
        }
      }

      // callbacks
      if (allServosDone) {
        if (isExecute) {
          Serial.println("CP0");
          isExecute = false;
        }
      }

      // recording callbacks
      if (isRecording) {
        if (!allServosDone && isRecordingOnceJoints) {
          Serial.print("REC,");  // add REC verification
          for (int i = 0; i < numServos; i++) {
              Serial.print(currentAngles[i]);
              if (i < numServos - 1) {
                  Serial.print(",");
              } else {
                  Serial.println();
              }
          }
        } else if (allServosDone && isRecordingOnceJoints) {
          Serial.println("CP0");
          isRecordingOnceJoints = false;
        }
        
        if (isMoveToolDone && isRecordingOnceTool) {
          Serial.print("M280,");
          if (currentEOAT.type == 0) { // gripper - convert back to distance
            Serial.println(gripperAngleToDistance(currentEOAT.state));
          } else {
            Serial.println(currentEOAT.state);
          }
          Serial.println("TP0");
          isRecordingOnceTool = false;
          
        }
      }
      
      delay(5); // Adjust based on your needs
    }
    
  } else if (mode == 1) {
    #if ENABLE_ROS
    // ROS mode operation
    for(int i = 0; i < numServos; i++) {
        str_msg2.data[i] = rosAngles[i];
    }
    str_msg2.data[numServos] = currentEOAT.state;
    str_msg2.data_length = numServos+1;
    delay(2);
    chatter.publish(&str_msg2);
    nh.spinOnce();
    #endif
  }
  
}
