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

#include <AccelStepper.h>
#include <MultiStepper.h>

// 定义步进电机引脚 (TMC2209)
//x
#define J0_PUL_PIN        A0
#define J0_DIR_PIN        A1
#define J0_ENABLE_PIN     38

//y
#define J1_PUL_PIN        A6
#define J1_DIR_PIN        A7
#define J1_ENABLE_PIN     A2

//z
#define J2_PUL_PIN        46
#define J2_DIR_PIN        48
#define J2_ENABLE_PIN     A8 

// 定义扩展TB6600步进电机引脚
#define J3_PUL_PIN        47
#define J3_DIR_PIN        45
#define J3_ENABLE_PIN     28

#define J4_PUL_PIN        43
#define J4_DIR_PIN        41
#define J4_ENABLE_PIN     30

#define J5_PUL_PIN        39
#define J5_DIR_PIN        37
#define J5_ENABLE_PIN     32

// 定义各电机减速比和步进/角度比率
#define J5_GEAR_RATIO     6.4
#define J4_GEAR_RATIO     20
#define J3_GEAR_RATIO     18
#define J2_GEAR_RATIO     4
#define J1_GEAR_RATIO     4
#define J0_GEAR_RATIO     5

// 电机步进与角度转换系数 (根据电机型号调整)
#define STEPS_PER_DEGREE_J5  17.78 * J5_GEAR_RATIO
#define STEPS_PER_DEGREE_J4  17.78 * J4_GEAR_RATIO
#define STEPS_PER_DEGREE_J3  17.78 * J3_GEAR_RATIO
#define STEPS_PER_DEGREE_J2  17.78 * J2_GEAR_RATIO
#define STEPS_PER_DEGREE_J1  17.78 * J1_GEAR_RATIO
#define STEPS_PER_DEGREE_J0  17.78 * J0_GEAR_RATIO

// 末端执行器IO定义
#define VACUUM_PIN        9  // 真空泵控制引脚
#define GRIPPER_PIN       10 // 夹爪控制引脚

// 定义基础速度和加速度参数
#define BASE_MAX_SPEED    2000
#define BASE_ACCELERATION 1000
#define BASE_SPEED        1000
#define MIN_SPEED         200

// 版本信息
#define FIRMWARE_VERSION "1.0.0"

// macro to enable ROS
#define ENABLE_ROS 0

// 为每个电机单独定义最大速度和加速度
int motorMaxSpeeds[numJoints] = {2000, 2000, 2000, 2000, 2000, 2000};     // 每个电机的最大速度
int motorAccelerations[numJoints] = {1000, 1000, 1000, 1000, 1000, 1000}; // 每个电机的加速度

const float MIN_STEP = 1.5f;  // 最小步进角度

// 创建6个AccelStepper对象
AccelStepper stepper1(AccelStepper::DRIVER, J5_PUL_PIN, J5_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, J4_PUL_PIN, J4_DIR_PIN);
AccelStepper stepper3(AccelStepper::DRIVER, J3_PUL_PIN, J3_DIR_PIN);
AccelStepper stepper4(AccelStepper::DRIVER, J2_PUL_PIN, J2_DIR_PIN);
AccelStepper stepper5(AccelStepper::DRIVER, J1_PUL_PIN, J1_DIR_PIN);
AccelStepper stepper6(AccelStepper::DRIVER, J0_PUL_PIN, J0_DIR_PIN);

// 创建MultiStepper对象用于同步控制
MultiStepper steppers;

const int numSteppers = 6;
AccelStepper* stepperArray[numSteppers] = {&stepper1, &stepper2, &stepper3, &stepper4, &stepper5, &stepper6};
long positions[numSteppers]; // 目标位置数组

const int numJoints = 6;
const float jointLimits[numJoints][2] = {
    {-90.0f, 90.0f},   // base
    {-49.0f, 90.0f},    // shoulder
    {-89.0f, 71.0f},  // elbow
    {-90.0f, 90.0f},   // wrist
    {-90.0f, 90.0f},   // 工具(夹爪角度或真空泵状态)
    {-90.0f, 90.0f}     // 扩展轴
};

const char* joint_names[] = {
    "Revolute1",
    "Revolute2",
    "Revolute3",
    "Revolute4",
    "Slider5",
    "Slider6"
};

// 校准偏移量
float calibrationOffsets[numJoints] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// 工具类型定义
struct EOAT_Type {
    uint8_t type;          // 0: GRIPPER, 1: PEN_HOLDER, 2: VACUUM_PUMP
    uint8_t pins[4];       // 工具可能使用的多个IO引脚
    uint8_t pin_count;     // 实际使用的引脚数量
    bool state;            // 工具状态
};
EOAT_Type currentEOAT;

// 运行状态标志
bool isExecute = false;
bool isMovingOnce = false;
bool isRecordingOnce = false;
bool isDelayed = false;
bool isRecording = false;
bool isReplaying = false;
bool isPaused = false;
bool potentiometersEnabled = false; // 步进电机版本不使用电位器

// 当前和目标角度
float currentAngles[numJoints];
float targetAngles[numJoints];
float tmpAngles[numJoints];
float homePositions[numJoints] = {0, -49, 69, 0, 0, 0}; // 机械臂初始位置

// 延时相关参数
int time_delay = 0;
int time_elapse = 0;

// 运动状态
struct MotionState {
    float startPos;         // 起始角度
    float targetPos;        // 目标角度
    long startSteps;        // 起始步数
    long targetSteps;       // 目标步数
    unsigned long startTime; // 起始时间
    unsigned long duration;  // 持续时间
    bool isMoving;          // 是否在运动
};
MotionState jointStates[numJoints];

int mode = 0; // 0表示控制器模式，1表示ROS模式

// 电机使能状态和上次活动时间
bool stepperEnabled[numSteppers] = {false, false, false, false, false, false};
unsigned long lastStepperActivity[numSteppers] = {0, 0, 0, 0, 0, 0};
const uint8_t enablePins[numSteppers] = {J5_ENABLE_PIN, J4_ENABLE_PIN, J3_ENABLE_PIN, J2_ENABLE_PIN, J1_ENABLE_PIN, J0_ENABLE_PIN};

// 工具函数
float degreesToSteps(float degrees, int jointIndex) {
    switch (jointIndex) {
        case 0: return degrees * STEPS_PER_DEGREE_J5;
        case 1: return degrees * STEPS_PER_DEGREE_J4;
        case 2: return degrees * STEPS_PER_DEGREE_J3;
        case 3: return degrees * STEPS_PER_DEGREE_J2;
        case 4: return degrees * STEPS_PER_DEGREE_J1;
        case 5: return degrees * STEPS_PER_DEGREE_J0;
        default: return degrees * STEPS_PER_DEGREE_J5;
    }
}

// 平滑插值函数
float smoothInterpolation(float start, float target, float progress) {
    if (progress <= 0.0f) return start;
    if (progress >= 1.0f) return target;
    
    // 使用 sin²(t) 插值
    float smoothProgress = sin(progress * PI / 2.0f);
    smoothProgress = smoothProgress * smoothProgress;
    
    // 计算角度差（考虑最短路径）
    float angleDiff = target - start;
    while (angleDiff > 180) angleDiff -= 360;
    while (angleDiff < -180) angleDiff += 360;
    
    return start + angleDiff * smoothProgress;
}

// 添加浮点映射函数
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// 计算运动持续时间
unsigned long calculateMotionDuration(float angleChange, int jointId) {
    float absChange = abs(angleChange);
    // 根据电机配置和角度变化计算合理的运动时间
    unsigned long baseDuration = 1000;  // 基础时间1秒
    
    // 根据电机速度调整时间
    if (jointId >= 0 && jointId < numJoints) {
        // 角度变化越大，时间越长
        baseDuration += (unsigned long)((absChange / 90.0f) * 2000);
        
        // 考虑电机速度（速度越高，时间越短）
        float speedRatio = (float)motorMaxSpeeds[jointId] / BASE_MAX_SPEED;
        
        if (speedRatio > 0) {
            return baseDuration / speedRatio;
        }
    }
    
    return baseDuration;
}

// 设置校准偏移
void setCalibrationOffset(int joint, float offset) {
    if (joint >= 0 && joint < numJoints) {
        calibrationOffsets[joint] = offset;
    }
}

// 控制夹爪
void controlGripper(float angle) {
    float clampedAngle = constrain(angle, 90.0f, 180.0f);
    // 将夹爪角度映射到PWM脉冲宽度
    int pulseWidth = map(clampedAngle, 90, 180, 1000, 2000);
    
    // 如果有专用引脚控制夹爪舵机，可以在这里使用
    // 例如: servoPWM.setPWM(GRIPPER_PIN, 0, pulseWidth);
    
    // 更新当前工具状态
    currentEOAT.state = (angle > 135) ? true : false;
}

// 控制真空泵
void controlPump(int state) {
    // 真空泵控制可能是直接通过数字引脚
    uint8_t suctionPin = currentEOAT.pins[0];
    
    if (currentEOAT.state == state) {
        return;
    }

    if (state == 1) {
        // 打开真空泵
        digitalWrite(suctionPin, HIGH);
    } else {
        // 关闭真空泵
        digitalWrite(suctionPin, LOW);
    }
    
    currentEOAT.state = state;
}

// 启用或禁用单个步进电机
void enableStepper(int index, bool enable) {
    if (index < 0 || index >= numSteppers) return;
    
    // 步进电机启用/禁用，低电平有效
    uint8_t level = enable ? LOW : HIGH;
    digitalWrite(enablePins[index], level);
    stepperEnabled[index] = enable;
    
    if (enable) {
        lastStepperActivity[index] = millis();
    }
}

void enableSteppers(bool enabled) {
    // 启用/禁用所有步进电机
    for (int i = 0; i < numSteppers; i++) {
        enableStepper(i, enabled);
    }
}

// 更新电机的上次活动时间
void updateStepperActivity(int index) {
    if (index < 0 || index >= numSteppers) return;
    lastStepperActivity[index] = millis();
    
    // 确保电机已启用
    if (!stepperEnabled[index]) {
        enableStepper(index, true);
    }
}

// 移动电机到目标角度
float moveMotor(float targetDegree, float curDegree, int id, bool fake) {
    if(id >= 0 && id < numJoints) {
        // 检查是否是工具移动
        if(id == 4) { // 工具ID为4
            if(abs(targetDegree - curDegree) >= MIN_STEP) {
                jointStates[id].isMoving = true;
                if(!fake) {
                    if(currentEOAT.type == 0) { // 夹爪
                        controlGripper(int(targetDegree));
                    } else if(currentEOAT.type == 2) { // 真空泵
                        controlPump(int(targetDegree));
                    }
                }
            } else {
                jointStates[id].isMoving = false;
            }
            return targetDegree;
        }
        
        if(abs(targetDegree - curDegree) < MIN_STEP) {
            jointStates[id].isMoving = false;
            return targetDegree;
        }
        
        if(!jointStates[id].isMoving) {
            jointStates[id].startPos = curDegree;
            jointStates[id].targetPos = targetDegree;
            
            // 转换为步数
            jointStates[id].startSteps = stepperArray[id]->currentPosition();
            jointStates[id].targetSteps = degreesToSteps(targetDegree, id);
            
            jointStates[id].startTime = millis();
            jointStates[id].duration = calculateMotionDuration(targetDegree - jointStates[id].startPos, id);
            jointStates[id].isMoving = true;
            
            // 设置新的目标位置
            if(!fake && id < numSteppers) {
                // 设置步进电机速度和加速度
                updateStepperActivity(id);
                
                stepperArray[id]->setMaxSpeed(motorMaxSpeeds[id]);
                stepperArray[id]->setAcceleration(motorAccelerations[id]);
                stepperArray[id]->moveTo(jointStates[id].targetSteps);
            }
        }
        
        unsigned long currentTime = millis();
        unsigned long elapsedTime = currentTime - jointStates[id].startTime;
        float progress = constrain((float)elapsedTime / jointStates[id].duration, 0.0f, 1.0f);
        
        // 由于AccelStepper库会处理实际的电机运动，这里我们只需要更新虚拟角度值
        float newAngle;
        if (fake) {
            // 如果是虚拟运动，手动计算插值
            newAngle = smoothInterpolation(
                jointStates[id].startPos,
                jointStates[id].targetPos,
                progress
            );
        } else {
            // 如果是真实运动，用步进电机的实际位置计算当前角度
            if (id < numSteppers) {
                long currentSteps = stepperArray[id]->currentPosition();
                newAngle = mapFloat(currentSteps, jointStates[id].startSteps, jointStates[id].targetSteps, 
                             jointStates[id].startPos, jointStates[id].targetPos);
            } else {
                newAngle = curDegree;
            }
        }
        
        if(progress >= 1.0f) {
            jointStates[id].isMoving = false;
            
            // 更新最后活动时间
            if (id < numSteppers) {
                updateStepperActivity(id);
            }
            
            return targetDegree;
        }
        
        // 更新电机活动时间
        if (id < numSteppers && !fake) {
            updateStepperActivity(id);
        }
        
        return newAngle;
    }
    return curDegree;  // 如果ID无效，返回当前位置
}

void setup() {
    // 初始化串口通信
    Serial.begin(115200);
    delay(2000);
    
    // 设置使能引脚
    pinMode(J5_ENABLE_PIN, OUTPUT);
    pinMode(J4_ENABLE_PIN, OUTPUT);
    pinMode(J3_ENABLE_PIN, OUTPUT);
    pinMode(J2_ENABLE_PIN, OUTPUT);
    pinMode(J1_ENABLE_PIN, OUTPUT);
    pinMode(J0_ENABLE_PIN, OUTPUT);
    
    // 初始化工具引脚
    pinMode(VACUUM_PIN, OUTPUT);
    pinMode(GRIPPER_PIN, OUTPUT);
    
    // 禁用所有电机（高电平禁用）
    enableSteppers(false);
    delay(100);
    // 所有电机先启用以设置初始位置
    enableSteppers(true);

    // 配置每个电机的最大速度和加速度
    stepper1.setMaxSpeed(motorMaxSpeeds[0]);
    stepper1.setAcceleration(motorAccelerations[0]);
    
    stepper2.setMaxSpeed(motorMaxSpeeds[1]);
    stepper2.setAcceleration(motorAccelerations[1]);
    
    stepper3.setMaxSpeed(motorMaxSpeeds[2]);
    stepper3.setAcceleration(motorAccelerations[2]);
    
    stepper4.setMaxSpeed(motorMaxSpeeds[3]);
    stepper4.setAcceleration(motorAccelerations[3]);
    
    stepper5.setMaxSpeed(motorMaxSpeeds[4]);
    stepper5.setAcceleration(motorAccelerations[4]);
    
    stepper6.setMaxSpeed(motorMaxSpeeds[5]);
    stepper6.setAcceleration(motorAccelerations[5]);
    
    // 添加到MultiStepper对象
    steppers.addStepper(stepper1);
    steppers.addStepper(stepper2);
    steppers.addStepper(stepper3);
    steppers.addStepper(stepper4);
    steppers.addStepper(stepper5);
    steppers.addStepper(stepper6);
    
    // 初始化工具配置
    currentEOAT.type = 0;           // 默认为夹爪
    currentEOAT.pins[0] = GRIPPER_PIN; // 默认使用夹爪引脚
    currentEOAT.pin_count = 1;
    currentEOAT.state = 90;
    
    // 初始化关节状态
    for(int i = 0; i < numJoints; i++) {
        currentAngles[i] = homePositions[i];
        targetAngles[i] = homePositions[i];
        tmpAngles[i] = homePositions[i];
        
        // 设置初始位置
        long steps = degreesToSteps(homePositions[i], i);
        if(i < numSteppers) {
            stepperArray[i]->setCurrentPosition(steps);
        }
        
        jointStates[i].startPos = homePositions[i];
        jointStates[i].targetPos = homePositions[i];
        jointStates[i].isMoving = false;
    }
    
    // 设置初始模式
    mode = 0;
}

void loop() {
    // 检查串口命令
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        
        if (command == "VERC") {
            // 返回固件版本和系统信息
            Serial.println("INFOS");
            Serial.println("VER," + String(FIRMWARE_VERSION));
            Serial.println("INFOE");
        } else if (command.startsWith("CALIBRATE,")) {
            // 校准格式: CALIBRATE,关节,偏移量
            int firstComma = command.indexOf(',');
            int secondComma = command.indexOf(',', firstComma + 1);
            
            int joint = command.substring(firstComma + 1, secondComma).toInt();
            float offset = command.substring(secondComma + 1).toFloat();
            
            setCalibrationOffset(joint, offset);
            Serial.println("Calibration offset set");
        } else if (command == "RECSTART") {
            isRecording = true;
            isReplaying = false;
        } else if (command == "RECSTOP") {
            isRecording = false;
            isMovingOnce = false;
            isRecordingOnce = false;
        } else if (command == "REPSTART") {
            isReplaying = true;
            isRecording = false;
            isRecordingOnce = false;
            isMovingOnce = false;
            isPaused = false;
        } else if (command == "REPPAUSE") {
            if (isReplaying) {
                isPaused = true;
                // 暂停所有步进电机
                for (int i = 0; i < numSteppers; i++) {
                    stepperArray[i]->stop();
                }
            }
        } else if (command == "REPSTOP") {
            isReplaying = false;
            // 停止所有步进电机
            for (int i = 0; i < numSteppers; i++) {
                stepperArray[i]->stop();
            }
            delay(5);
        } else if (command == "POTOFF") {
            // 步进电机版本不使用电位器，但保留命令兼容性
            potentiometersEnabled = false;
            Serial.println("POT0");
        } else if (command == "POTON") {
            potentiometersEnabled = false;  // 始终返回禁用
            Serial.println("POT0");
        } else if (command == "TOROS") {
            mode = 1;
        } else if (command == "TOCTR") {
            mode = 0;
        } else if (command.startsWith("TOOL[GRIPPER],")) {
            currentEOAT.type = 0;
            currentEOAT.pin_count = 1;
            currentEOAT.state = 90;
            
            // 解析逗号分隔的IO指令
            int startIndex = command.indexOf(',') + 1;
            String io_str = command.substring(startIndex);
            if(io_str.startsWith("IO")) {
                int pin = io_str.substring(2).toInt();
                currentEOAT.pins[0] = pin;
                currentEOAT.pin_count = 1;
            }
            
            Serial.println("CP2");
        } else if (command.startsWith("TOOL[PEN_HOLDER],")) {
            currentEOAT.type = 1;
            currentEOAT.pin_count = 0;
            currentEOAT.state = false;
            
            Serial.println("CP2");
        } else if (command.startsWith("TOOL[VACUUM_PUMP],")) {
            currentEOAT.type = 2;
            currentEOAT.pin_count = 0;
            currentEOAT.state = 0;
            
            // 解析逗号分隔的多个IO指令
            int startIndex = command.indexOf(',') + 1;
            while (startIndex < command.length()) {
                int endIndex = command.indexOf(',', startIndex);
                if (endIndex == -1) endIndex = command.length();
                
                String io_str = command.substring(startIndex, endIndex);
                if(io_str.startsWith("IO")) {
                    int pin = io_str.substring(2).toInt();
                    currentEOAT.pins[currentEOAT.pin_count++] = pin;
                }
                
                startIndex = endIndex + 1;
            }
            
            Serial.println("CP2");
        } else if (command == "EXEC" && !potentiometersEnabled) {
            String cmd = Serial.readStringUntil('\n');
            // 处理逗号分隔的角度值
            int startIndex = 0;
            for (int i = 0; i < numJoints; i++) {
                int nextIndex = cmd.indexOf(',', startIndex);
                if (nextIndex == -1 && i < numJoints - 1) {
                    return; // 数据格式不正确
                }
                String inputStr = (nextIndex == -1) ? cmd.substring(startIndex) : cmd.substring(startIndex, nextIndex);

                targetAngles[i] = inputStr.toFloat();
                startIndex = nextIndex + 1;
            }
            isExecute = true;
        } else if (command == "MOVEONCE" && !potentiometersEnabled) {
            String cmd = Serial.readStringUntil('\n');
            // 处理MOVE_ONCE命令
            int startIndex = 0;
            for (int i = 0; i < numJoints; i++) {
                int nextIndex = cmd.indexOf(',', startIndex);
                if (nextIndex == -1 && i < numJoints - 1) {
                    return; // 数据格式不正确
                }
                String inputStr = (nextIndex == -1) ? cmd.substring(startIndex) : cmd.substring(startIndex, nextIndex);
                
                targetAngles[i] = inputStr.toFloat();
                startIndex = nextIndex + 1;
            }
            
            isMovingOnce = true;
            isRecordingOnce = false;
        } else if (command == "RECONCE" && !potentiometersEnabled) {
            String cmd = Serial.readStringUntil('\n');
            time_delay = cmd.toInt();
            isRecordingOnce = true;
            isMovingOnce = false;
            isDelayed = false;  // 重置延迟标志
        } else if (command.startsWith("SPD,")) {
            // 格式: SPD,speed1,speed2,speed3,speed4,speed5,speed6
            int startIndex = command.indexOf(',') + 1;
            for (int i = 0; i < numJoints; i++) {
                int nextIndex = command.indexOf(',', startIndex);
                if (nextIndex == -1 && i < numJoints - 1) {
                    break; // 不完整数据也接受部分设置
                }
                String speedStr = (nextIndex == -1) ? command.substring(startIndex) : command.substring(startIndex, nextIndex);
                int speed = speedStr.toInt();
                if (speed > 0) {
                    motorMaxSpeeds[i] = speed;
                    // 立即更新电机速度
                    if (i < numSteppers) {
                        stepperArray[i]->setMaxSpeed(motorMaxSpeeds[i]);
                    }
                }
                if (nextIndex == -1) break;
                startIndex = nextIndex + 1;
            }
            Serial.println("SPD_SET");
        } else if (command.startsWith("ACC,")) {
            // 格式: ACC,accel1,accel2,accel3,accel4,accel5,accel6
            int startIndex = command.indexOf(',') + 1;
            for (int i = 0; i < numJoints; i++) {
                int nextIndex = command.indexOf(',', startIndex);
                if (nextIndex == -1 && i < numJoints - 1) {
                    break; // 不完整数据也接受部分设置
                }
                String accelStr = (nextIndex == -1) ? command.substring(startIndex) : command.substring(startIndex, nextIndex);
                int accel = accelStr.toInt();
                if (accel > 0) {
                    motorAccelerations[i] = accel;
                    // 立即更新电机加速度
                    if (i < numSteppers) {
                        stepperArray[i]->setAcceleration(motorAccelerations[i]);
                    }
                }
                if (nextIndex == -1) break;
                startIndex = nextIndex + 1;
            }
            Serial.println("ACC_SET");
        }
    }
    
    if (mode == 0) { // 控制器模式
        bool allSteppersDone = true;
        
        if (!isReplaying) {
            // 控制步进电机运动
            if (isMovingOnce) {
                for (int i = 0; i < numJoints; i++) {
                    tmpAngles[i] = moveMotor(targetAngles[i], tmpAngles[i], i, false);
                }
            } else {
                for (int i = 0; i < numJoints; i++) {
                    if (!isRecordingOnce && !isMovingOnce) {
                        currentAngles[i] = moveMotor(targetAngles[i], currentAngles[i], i, false);
                    } else {
                        currentAngles[i] = moveMotor(targetAngles[i], currentAngles[i], i, true);
                    }
                }
            }
            
            // 运行步进电机
            for (int i = 0; i < numSteppers; i++) {
                // 只对已启用的电机运行步进
                if (stepperEnabled[i]) {
                    bool running = stepperArray[i]->run();
                    if (running) {
                        allSteppersDone = false;
                        updateStepperActivity(i);
                    }
                } else if (jointStates[i].isMoving) {
                    allSteppersDone = false;
                }
            }
        } else if (isReplaying) {
            // 回放模式
            if (!isPaused) {
                int angles[numJoints];
                int startIndex = 0;
                
                // 需要从Serial读取命令
                String replayCommand = Serial.readStringUntil('\n');
                
                // 解析命令字符串中的角度值
                for (int i = 0; i < numJoints; i++) {
                    int nextIndex = replayCommand.indexOf(',', startIndex);
                    if (nextIndex == -1 && i < numJoints - 1) {
                        return; // 数据格式不正确
                    }
                    String angleStr = (nextIndex == -1) ? replayCommand.substring(startIndex) : replayCommand.substring(startIndex, nextIndex);
                    angles[i] = angleStr.toInt();
                    startIndex = nextIndex + 1;
                    
                    if (i == 4) {
                        if (currentEOAT.type == 0) {
                            controlGripper(angles[i]);
                        } else if (currentEOAT.type == 2) {
                            controlPump(angles[i]);
                        }
                    } else if (i < numSteppers) {
                        
                        // 设置电机目标位置
                        long steps = degreesToSteps(angles[i], i);
                        stepperArray[i]->moveTo(steps);
                        updateStepperActivity(i);
                    }
                }
                
                // 同步运行所有电机
                for (int i = 0; i < numSteppers; i++) {
                    if (stepperEnabled[i]) {
                        bool running = stepperArray[i]->run();
                        if (running) {
                            allSteppersDone = false;
                            updateStepperActivity(i);
                        }
                    }
                }
            }
        }
        
        // 检查所有关节是否完成运动
        for (int i = 0; i < numJoints; i++) {
            if (jointStates[i].isMoving) {
                allSteppersDone = false;
                break;
            }
        }
        
        if (allSteppersDone && isExecute) {
            Serial.println("CP0");
            isExecute = false;
        }
        
        if (isRecording) {
            if (!allSteppersDone && isRecordingOnce) {
                Serial.print("REC,");
                for (int i = 0; i < numJoints; i++) {
                    Serial.print(currentAngles[i]);
                    if (i < numJoints - 1) {
                        Serial.print(",");
                    } else {
                        Serial.println();
                    }
                }
            } else if (allSteppersDone && isRecordingOnce && !isDelayed) {
                // 只发送一次延迟命令
                Serial.print("REC,D");
                Serial.println(time_delay);
                isDelayed = true;
            }
        }
    } else if (mode == 1) {
        // ROS模式，暂不实现
    }
    
    delay(5);
}
