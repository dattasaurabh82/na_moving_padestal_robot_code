// ==========================================================================
// ==                         Robot Controller Code                        ==
// == Description: Arduino code for an autonomous robot with obstacle      ==
// == avoidance, and a timed active/pause cycle.                           ==
// ==========================================================================

/*
* Total Pins used:
* - 6 Digital Pins for motors (10, 16, 8, 6, 7, 9)
* - 1 Digital pin for the buzzer (5)
* - 2 Digital Pins for for IMU (2, 3 as I2C) - IMU primarily for debug/future
* - 2 Analog Pins (A0 for motorBatteryPin , A1 for logicBatteryPin)
* - 2 Analog Pins (A2 for Random Seed, A3 for Distance sensor)
* - 1 Digital from Cap Touch sensor (Used to cmd the robot to start/stop moving)

* Pins un-used:
* - 0 & 1 (RX-TX Serial COMM) RESERVED

** handlePanicReversing, handleAutoSpinningLeft, handleAutoSpinningRight removed

*/

// ========================================
// ==             Libraries              ==
// ========================================
#include <Arduino.h>
#include <Wire.h>            // For I2C communication (IMU)
#include <FreeSixIMU.h>      // IMU library
#include <FIMU_ADXL345.h>    // Accelerometer part of FreeSixIMU
#include <FIMU_ITG3200.h>    // Gyroscope part of FreeSixIMU
#include <SharpIR.h>         // Distance sensor library
#include "RobotTones.h"      // RobotTones header file
#include "BatteryMonitor.h"  // For battery monitor functions

// ========================================
// ==        Debugging Switches          ==
// ========================================
// #define DEBUG_STATE_CHANGES // Uncomment to see state transitions
// #define DEBUG_IMU           // Uncomment for IMU yaw readings (IMU not used for control)
// #define DEBUG_DISTANCE      // Uncomment for distance sensor readings
// #define DEBUG_OBSTACLE      // Uncomment for obstacle detection logic
// #define DEBUG_TURN          // Uncomment for turning logic details
// #define DEBUG_CAP_SENSE     // Uncomment to read capsense toggle state
// #define DEBUG_BATTERIES     // Uncomment for battery debug prints

// ========================================
// ==      Robot Behavior Constants      ==
// ========================================
const int COLLISION_THRESHOLD_DISTANCE = 22;        // cm, robot stops if obstacle is closer
const unsigned long STOP_DURATION_OBSTACLE = 500;   // ms, time to stop after detecting obstacle
const unsigned long REVERSE_DURATION = 2000;        // ms, time to move backward (was 5000)
const unsigned long POST_TURN_STOP_DURATION = 500;  // ms, time to stop after completing a turn
const unsigned long STATE_TRANSITION_DELAY = 50;    // ms, small delay when transitioning states

// -- Auto Roam Cycle Timeouts --- //
const unsigned long ACTIVE_ROAM_DURATION_MS = 10000;  // 10 seconds
const unsigned long AUTO_PAUSE_DURATION_MS = 60000;   // 1 min in milliseconds (1x60x1000)

// -- Cap Sense Debounce --- //
const unsigned long CAP_SENSE_DEBOUNCE_DELAY = 50;

// ========================================
// ==         Motor Speed Settings       ==
// ========================================
const int FORWARD_SPEED = 125;                               // PWM value (0-255), adjust as needed
const int REVERSE_SPEED = 100;                               // PWM value
const int TURN_SPEED = 100;                                  // PWM value for sharp turns (was 100)
const unsigned long TIMED_OBSTACLE_TURN_DURATION_MS = 1000;  // 1 seconds for a timed turn (was 1000)

// ========================================
// ==        Motor Pin Definitions       ==
// ========================================
const int M1_PWM_PIN = 10;
const int M1_IN_A_PIN = 16;
const int M1_IN_B_PIN = 8;
const int M2_PWM_PIN = 6;
const int M2_IN_A_PIN = 7;
const int M2_IN_B_PIN = 9;

// ========================================
// ==     Distance Sensor Setup          ==
// ========================================
const int DIST_SENSOR_PIN = A3;
const int DIST_SENSOR_MODEL = SharpIR::GP2Y0A02YK0F;
SharpIR distSensor(DIST_SENSOR_MODEL, DIST_SENSOR_PIN);
const int DIST_SAMPLE_SIZE = 5;
int distanceSamples[DIST_SAMPLE_SIZE];
int currentDistanceSampleIndex = 0;
int medianDistance = 0;

// ========================================
// ==        Buzzer & Tone Setup         ==
// ========================================
const int BUZZER_PIN = 5;

// ========================================
// ==          CapSense Setup            ==
// ========================================
const int CAP_SENSE_PIN = 4;
bool INIT_ROBOT = false;  // Global flag: true = robot active, false = robot stopped/idle
bool playStartToneOnNextActivation = true;

// Variables for Automatic Roam Cycle
bool autoCycleEngaged = false;
bool currentlyInAutoPausePhase = false;
unsigned long currentAutoPhaseStartTime = 0;

// For CapSense debouncing (global static for simplicity here)
static bool lastDebouncedCapState = false;
static unsigned long lastCapSenseDebounceTime = 0;
static bool lastRawCapSenseState = false;

// ========================================
// ==            IMU Setup               ==
// ========================================
FreeSixIMU imu = FreeSixIMU();
float ypr[3];             // Yaw, Pitch, Roll
float currentYaw = 0.0f;  // Still read for potential debug or future use

// ========================================
// ==          State Machine             ==
// ========================================
enum RobotState {
  IDLE,
  MOVING_FORWARD,
  OBSTACLE_DETECTED_STOP,
  REVERSING,
  DECIDING_TURN_DIRECTION,
  SELECTING_TURN_ANGLE,
  TURNING_LEFT,
  TURNING_RIGHT,
  POST_TURN_STOP
};
RobotState currentState = IDLE;
RobotState previousState = IDLE;
unsigned long stateEntryTime = 0;

// ========================================
// ==    Battery management related      ==
// ========================================
int internalCheckDuration = 5000;
int infoPrintDuration = 5000;
float motorBatteryLowThres = 8.0;
float arduinoBatteryLowThres = 3.0;
bool debugExtraBattInfo = false;  // Set by DEBUG_BATTERIES
bool buzzerAlert = true;          // Enable buzzer for low battery

// ========================================
// ==     Function Declarations          ==
// ========================================
void readCapSensePad();
void readDistanceSensor();
void readIMU();
void setMotorSpeed(int motorNum, int speed, bool forward);
void moveForwardCmd(int speed);
void moveBackwardCmd(int speed);
void turnLeftSharpCmd(int speed);
void turnRightSharpCmd(int speed);
void stopMotors();
void handleIdle();
void handleMovingForward();
void handleObstacleDetectedStop();
void handleReversing();
void handleDecidingTurnDirection();
void handleSelectingTurnAngle();
void handleTurningLeft();
void handleTurningRight();
void handlePostTurnStop();
// void handlePanicReversing();      // Removed
// void handleAutoSpinningLeft();    // Removed
// void handleAutoSpinningRight();   // Removed
void changeState(RobotState newState);
const char *getStateName(RobotState state);
void manageAutomaticRoamCycle();

// ==========================================================================
// ==                            SETUP FUNCTION                            ==
// ==========================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000)
    ;
  Serial.println(F("Robot Controller Initializing (Simplified)..."));

#ifdef DEBUG_BATTERIES
  debugExtraBattInfo = true;
#endif
  Serial.println(F("Initializing Battery Monitor..."));
  batteryMonitorInit(A0, A1, 6.5f, 3.0f, debugExtraBattInfo);
  // After batteryMonitorInit, analogReference is INTERNAL. Switch back for other sensors.
  analogReference(DEFAULT);
  // [Optional] customize settings
  setBatteryCheckInterval(internalCheckDuration);                      // Check every 10 seconds
  setBatteryThresholds(motorBatteryLowThres, arduinoBatteryLowThres);  // Set volt warning thresholds
  Serial.println(F("Battery Monitor initialized. Analog reference set to DEFAULT."));

  // --- Initialize DO from Cap sense ---
  pinMode(CAP_SENSE_PIN, INPUT);
  Serial.println(F("Cap Sense pin (To Start or Stop the Robot) initialized."));

  // --- Initialize Motors ---
  pinMode(M1_PWM_PIN, OUTPUT);
  pinMode(M1_IN_A_PIN, OUTPUT);
  pinMode(M1_IN_B_PIN, OUTPUT);
  pinMode(M2_PWM_PIN, OUTPUT);
  pinMode(M2_IN_A_PIN, OUTPUT);
  pinMode(M2_IN_B_PIN, OUTPUT);
  Serial.println(F("Motor pins initialized."));
  stopMotors();
  Serial.println(F("Motors stopped."));

  // --- Initialize IMU ---
  Wire.begin();
  delay(50);  // Allow I2C to settle
  imu.init(); // Initialize the IMU
  delay(50);  // Allow IMU to stabilize after init
  // We will rely on readIMU() to see if we get valid data later.
  // If init fails catastrophically, the program might hang or behave erratically here,
  // which would be an indication of an I2C or IMU hardware problem.
  Serial.println(F("IMU init called. Check sensor readings."));
  for (int i = 0; i < DIST_SAMPLE_SIZE; ++i) { distanceSamples[i] = 0; }
  Serial.println(F("Distance sensor initialized."));

  // --- Initialize Random Seed ---
  // Use an unconnected analog pin for a somewhat random seed
  randomSeed(analogRead(A2)); // Make sure A2 is not used by other components
  Serial.println(F("Random seed initialized."));

  // --- Initial State ---
  readIMU(); // Get an initial yaw reading (just for debugging)

  Serial.println(F("System Initialization Complete."));
  
  Serial.println(F("System Initialization Complete."));
  changeState(IDLE); // Start in IDLE, will transition to MOVING_FORWARD

  // --- Initialize Buzzer Pin ---
  pinMode(BUZZER_PIN, OUTPUT);
  Serial.println(F("Buzzer pin initialized."));

  // --- Before entering main loop ... --- //
  //  Give people warning that we are about to start
  playStartupSequence(BUZZER_PIN);
  delay(1000);
}

// ==========================================================================
// ==                             MAIN LOOP                                ==
// ==========================================================================
void loop() {
  analogReference(INTERNAL);
  bool batteriesJustChecked = updateBatteryStatus();
  if (batteriesJustChecked) {
#ifdef DEBUG_BATTERIES  // Changed from DEBUG_STATE_CHANGES
    printBatteryStatus();
#endif
    if (checkMotorBatteryLowOnce()) {
      Serial.println(F("ALERT: Motor battery LOW!"));
      if (buzzerAlert) playLowBatteryTone(BUZZER_PIN);
      if (motorBatteryVoltage < 7.0) {  // CRITICAL threshold (example)
        if (INIT_ROBOT) {
          INIT_ROBOT = false;
          playStartToneOnNextActivation = true;
          autoCycleEngaged = false;
          currentlyInAutoPausePhase = false;
          Serial.println(F("CRITICAL: Motor battery very low. Robot STOPPED."));
          if (currentState != IDLE) changeState(IDLE);
        }
      }
    }
    if (checkLogicBatteryLowOnce()) {
      Serial.println(F("ALERT: Logic battery LOW!"));
      if (buzzerAlert) playLowBatteryTone(BUZZER_PIN);
      if (logicBatteryVoltage < 2.8) {  // CRITICAL threshold
        if (INIT_ROBOT) {
          INIT_ROBOT = false;
          playStartToneOnNextActivation = true;
          autoCycleEngaged = false;
          currentlyInAutoPausePhase = false;
          Serial.println(F("CRITICAL: Logic battery very low. Robot STOPPED."));
          if (currentState != IDLE) changeState(IDLE);
        }
      }
    }
  }
  analogReference(DEFAULT);

  readCapSensePad();
  manageAutomaticRoamCycle();
  readDistanceSensor();
  readIMU();

  if (currentState != previousState) {
#ifdef DEBUG_STATE_CHANGES
    Serial.print(millis());
    Serial.print(F(" State Change: "));
    Serial.print(getStateName(previousState));
    Serial.print(F(" -> "));
    Serial.println(getStateName(currentState));
#endif
    previousState = currentState;
  }

  switch (currentState) {
    case IDLE:
      handleIdle();
      break;
    case MOVING_FORWARD:
      if (INIT_ROBOT) handleMovingForward();
      else changeState(IDLE);
      break;
    case OBSTACLE_DETECTED_STOP:
      if (INIT_ROBOT) handleObstacleDetectedStop();
      else changeState(IDLE);
      break;
    case REVERSING:
      if (INIT_ROBOT) handleReversing();
      else changeState(IDLE);
      break;
    case DECIDING_TURN_DIRECTION:
      if (INIT_ROBOT) handleDecidingTurnDirection();
      else changeState(IDLE);
      break;
    case SELECTING_TURN_ANGLE:
      if (INIT_ROBOT) handleSelectingTurnAngle();
      else changeState(IDLE);
      break;
    case TURNING_LEFT:
      if (INIT_ROBOT) handleTurningLeft();
      else changeState(IDLE);
      break;
    case TURNING_RIGHT:
      if (INIT_ROBOT) handleTurningRight();
      else changeState(IDLE);
      break;
    case POST_TURN_STOP:
      if (INIT_ROBOT) handlePostTurnStop();
      else changeState(IDLE);
      break;
    // PANIC_REVERSING, AUTO_SPINNING_LEFT/RIGHT cases are removed
    default:
      Serial.println(F("Error: Unknown state!"));
      changeState(IDLE);
      break;
  }
  delay(10);
}

// ==========================================================================
// ==                  HELPER FUNCTION IMPLEMENTATIONS                     ==
// ==========================================================================
void changeState(RobotState newState) {
  currentState = newState;
  stateEntryTime = millis();
#ifdef DEBUG_STATE_CHANGES
  Serial.print(millis());
  Serial.print(F(" Transitioning to: "));
  Serial.println(getStateName(currentState));
#endif
}

const char *getStateName(RobotState state) {
  switch (state) {
    case IDLE: return "IDLE";
    case MOVING_FORWARD: return "MOVING_FORWARD";
    case OBSTACLE_DETECTED_STOP: return "OBSTACLE_DETECTED_STOP";
    case REVERSING: return "REVERSING";
    case DECIDING_TURN_DIRECTION: return "DECIDING_TURN_DIRECTION";
    case SELECTING_TURN_ANGLE: return "SELECTING_TURN_ANGLE";
    case TURNING_LEFT: return "TURNING_LEFT";
    case TURNING_RIGHT: return "TURNING_RIGHT";
    case POST_TURN_STOP: return "POST_TURN_STOP";
    // PANIC_REVERSING, AUTO_SPINNING_LEFT/RIGHT cases are removed
    default: return "UNKNOWN_STATE";
  }
}

// ==========================================================================
// ==                 MOTOR CONTROL FUNCTION IMPLEMENTATIONS               ==
// ==========================================================================
void setMotorSpeed(int motorNum, int speedVal, bool forwardDir) {
  int actualSpeed = constrain(speedVal, 0, 255);
  if (motorNum == 1) {
    analogWrite(M1_PWM_PIN, actualSpeed);
    digitalWrite(M1_IN_A_PIN, forwardDir ? HIGH : LOW);
    digitalWrite(M1_IN_B_PIN, forwardDir ? LOW : HIGH);
  } else if (motorNum == 2) {
    analogWrite(M2_PWM_PIN, actualSpeed);
    digitalWrite(M2_IN_A_PIN, forwardDir ? HIGH : LOW);
    digitalWrite(M2_IN_B_PIN, forwardDir ? LOW : HIGH);
  }
}
void moveForwardCmd(int speed) {
  setMotorSpeed(1, speed, true);
  setMotorSpeed(2, speed, true);
}
void moveBackwardCmd(int speed) {
  setMotorSpeed(1, speed, false);
  setMotorSpeed(2, speed, false);
}
void turnLeftSharpCmd(int speed) {
  setMotorSpeed(1, speed, false);
  setMotorSpeed(2, speed, true);
}
void turnRightSharpCmd(int speed) {
  setMotorSpeed(1, speed, true);
  setMotorSpeed(2, speed, false);
}
void stopMotors() {
  analogWrite(M1_PWM_PIN, 0);
  analogWrite(M2_PWM_PIN, 0);
  digitalWrite(M1_IN_A_PIN, LOW);
  digitalWrite(M1_IN_B_PIN, LOW);
  digitalWrite(M2_IN_A_PIN, LOW);
  digitalWrite(M2_IN_B_PIN, LOW);
}

// ==========================================================================
// ==                   SENSOR FUNCTION IMPLEMENTATIONS                    ==
// ==========================================================================
void readCapSensePad() {
  static bool firstRun = true;
  if (firstRun) {
    lastRawCapSenseState = digitalRead(CAP_SENSE_PIN);
    lastDebouncedCapState = lastRawCapSenseState;
    if (INIT_ROBOT != lastDebouncedCapState) {
      INIT_ROBOT = lastDebouncedCapState;
#ifdef DEBUG_CAP_SENSE
      Serial.print(millis());
      Serial.print(F(" Initial Cap Sense State -> INIT_ROBOT: "));
      Serial.println(INIT_ROBOT ? "ON (Active)" : "OFF (Inactive)");
#endif
    }
    if (INIT_ROBOT) {  // If robot starts ON due to cap sensor state
      autoCycleEngaged = true;
      currentlyInAutoPausePhase = false;
      currentAutoPhaseStartTime = millis();
      // nextAutoWakeUpShouldBeSpin = true; // Removed
    }
    firstRun = false;
  }

  bool currentRawCapSenseState = digitalRead(CAP_SENSE_PIN);

  if (currentRawCapSenseState != lastRawCapSenseState) {
    lastCapSenseDebounceTime = millis();
  }

  if ((millis() - lastCapSenseDebounceTime) > CAP_SENSE_DEBOUNCE_DELAY) {
    if (currentRawCapSenseState != lastDebouncedCapState) {
      lastDebouncedCapState = currentRawCapSenseState;
      INIT_ROBOT = lastDebouncedCapState;

      Serial.print(millis());
      Serial.print(F(" Cap Sense Toggle: INIT_ROBOT is now -> "));
      Serial.println(INIT_ROBOT ? "ON (Run Commanded)" : "OFF (Stop Commanded)");

      if (!INIT_ROBOT) {  // Just Toggled to OFF (MANUAL STOP)
        stopMotors();
        playStopMovingTone(BUZZER_PIN);
        playStartToneOnNextActivation = true;
        autoCycleEngaged = false;
        currentlyInAutoPausePhase = false;
        // nextAutoWakeUpShouldBeSpin = true; // Removed
        if (currentState != IDLE) {
          changeState(IDLE);
        }
      } else {  // Just Toggled to ON (MANUAL START)
        playStartToneOnNextActivation = true;
        autoCycleEngaged = true;
        currentlyInAutoPausePhase = false;
        currentAutoPhaseStartTime = millis();
        // nextAutoWakeUpShouldBeSpin = true; // Removed
        if (currentState != IDLE) {
          changeState(IDLE);
        }
      }
    }
  }
  lastRawCapSenseState = currentRawCapSenseState;

#ifdef DEBUG_CAP_SENSE
  static unsigned long lastContinuousDebugPrint = 0;
  if (millis() - lastContinuousDebugPrint > 2000) {
    Serial.print(millis());  // Added millis to debug print
    Serial.print(F("  [CS Debug] INIT_ROBOT: "));
    Serial.print(INIT_ROBOT ? "ON" : "OFF");
    Serial.print(F(", autoCycle: "));
    Serial.print(autoCycleEngaged ? "ENGAGED" : "OFF");
    Serial.print(F(", autoPause: "));
    Serial.println(currentlyInAutoPausePhase ? "YES" : "NO");
    lastContinuousDebugPrint = millis();
  }
#endif
}

void readDistanceSensor() {
  distanceSamples[currentDistanceSampleIndex] = distSensor.getDistance();
  currentDistanceSampleIndex = (currentDistanceSampleIndex + 1) % DIST_SAMPLE_SIZE;
  int sortedSamples[DIST_SAMPLE_SIZE];
  for (int i = 0; i < DIST_SAMPLE_SIZE; ++i) { sortedSamples[i] = distanceSamples[i]; }
  for (int i = 0; i < DIST_SAMPLE_SIZE - 1; ++i) {
    for (int j = 0; j < DIST_SAMPLE_SIZE - i - 1; ++j) {
      if (sortedSamples[j] > sortedSamples[j + 1]) {
        int temp = sortedSamples[j];
        sortedSamples[j] = sortedSamples[j + 1];
        sortedSamples[j + 1] = temp;
      }
    }
  }
  if (DIST_SAMPLE_SIZE % 2 == 0) {
    medianDistance = (sortedSamples[DIST_SAMPLE_SIZE / 2 - 1] + sortedSamples[DIST_SAMPLE_SIZE / 2]) / 2;
  } else {
    medianDistance = sortedSamples[DIST_SAMPLE_SIZE / 2];
  }
  if (medianDistance <= 0 || medianDistance > 150) { medianDistance = 999; }
#ifdef DEBUG_DISTANCE
  Serial.print(millis());  // Added millis to debug print
  Serial.print(F(" Median Dist: "));
  Serial.print(medianDistance);
  Serial.println(F(" cm"));
#endif
}

void readIMU() {
  imu.getEuler(ypr);
  currentYaw = ypr[0];
#ifdef DEBUG_IMU
  Serial.print(millis());  // Added millis to debug print
  Serial.print(F(" IMU Yaw: "));
  Serial.print(currentYaw);
  Serial.println(F(" deg"));
#endif
}

// ==========================================================================
// ==                 STATE HANDLER FUNCTION IMPLEMENTATIONS               ==
// ==========================================================================
void handleIdle() {
  stopMotors();
  if (INIT_ROBOT) {
    if (playStartToneOnNextActivation) {
      playStartMovingTone(BUZZER_PIN);
      playStartToneOnNextActivation = false;
      stateEntryTime = millis();
    }
    if (!playStartToneOnNextActivation && (millis() - stateEntryTime > 500)) {
      changeState(MOVING_FORWARD);
    }
  } else {
    playStartToneOnNextActivation = true;
#ifdef DEBUG_STATE_CHANGES
    static unsigned long lastIdleNotifyTime = 0;
    if (millis() - lastIdleNotifyTime > 2000) {
      Serial.print(millis());  // Added millis to debug print
      Serial.println(F(" In IDLE, waiting for INIT_ROBOT=true (CapSense)."));
      lastIdleNotifyTime = millis();
    }
#endif
  }
}

void handleMovingForward() {
  moveForwardCmd(FORWARD_SPEED);
  if (medianDistance < COLLISION_THRESHOLD_DISTANCE) {
#ifdef DEBUG_OBSTACLE
    Serial.print(millis());  // Added millis to debug print
    Serial.print(F(" Obstacle @ "));
    Serial.print(medianDistance);
    Serial.println(F(" cm"));
#endif
    stopMotors();
    changeState(OBSTACLE_DETECTED_STOP);
    return;
  }
}

void handleObstacleDetectedStop() {
  stopMotors();
  if (millis() - stateEntryTime >= STOP_DURATION_OBSTACLE) {
    playBackwardTone(BUZZER_PIN);  // Tone before reversing
    changeState(REVERSING);
  }
}

void handleReversing() {
  moveBackwardCmd(REVERSE_SPEED);
  if (millis() - stateEntryTime >= REVERSE_DURATION) {
    stopMotors();
    delay(STATE_TRANSITION_DELAY);
    changeState(DECIDING_TURN_DIRECTION);
  }
}

void handleDecidingTurnDirection() {
  stopMotors();
  changeState(SELECTING_TURN_ANGLE);
}

void handleSelectingTurnAngle() {
  stopMotors();
  int randomDirection = random(0, 2);
  if (randomDirection == 0) {
#ifdef DEBUG_TURN
    Serial.print(millis());  // Added millis to debug print
    Serial.println(F(" Selected LEFT timed turn."));
#endif
    changeState(TURNING_LEFT);
  } else {
#ifdef DEBUG_TURN
    Serial.print(millis());  // Added millis to debug print
    Serial.println(F(" Selected RIGHT timed turn."));
#endif
    changeState(TURNING_RIGHT);
  }
}

void handleTurningLeft() {
  turnLeftSharpCmd(TURN_SPEED);
#ifdef DEBUG_TURN
  static unsigned long lastTurnPrint = 0;
  if (millis() - lastTurnPrint > 500) {
    Serial.print(millis());  // Added millis to debug print
    Serial.println(F(" Executing timed LEFT turn..."));
    lastTurnPrint = millis();
  }
#endif
  if (millis() - stateEntryTime >= TIMED_OBSTACLE_TURN_DURATION_MS) {
#ifdef DEBUG_TURN
    Serial.print(millis());  // Added millis to debug print
    Serial.println(F(" Timed LEFT turn completed."));
#endif
    stopMotors();
    delay(STATE_TRANSITION_DELAY);
    changeState(POST_TURN_STOP);
  }
}

void handleTurningRight() {
  turnRightSharpCmd(TURN_SPEED);
#ifdef DEBUG_TURN
  static unsigned long lastTurnPrint = 0;
  if (millis() - lastTurnPrint > 500) {
    Serial.print(millis());  // Added millis to debug print
    Serial.println(F(" Executing timed RIGHT turn..."));
    lastTurnPrint = millis();
  }
#endif
  if (millis() - stateEntryTime >= TIMED_OBSTACLE_TURN_DURATION_MS) {
#ifdef DEBUG_TURN
    Serial.print(millis());  // Added millis to debug print
    Serial.println(F(" Timed RIGHT turn completed."));
#endif
    stopMotors();
    delay(STATE_TRANSITION_DELAY);
    changeState(POST_TURN_STOP);
  }
}

void handlePostTurnStop() {
  stopMotors();
  if (millis() - stateEntryTime >= POST_TURN_STOP_DURATION) {
    // forwardReferenceYaw and its debug print removed
    changeState(MOVING_FORWARD);
  }
}

// ==========================================================================
// ==              AUTOMATIC ROAM CYCLE MANAGEMENT FUNCTION              ==
// ==========================================================================
void manageAutomaticRoamCycle() {
  if (!autoCycleEngaged) {
    return;
  }

  if (!currentlyInAutoPausePhase) {
    if (INIT_ROBOT && (millis() - currentAutoPhaseStartTime >= ACTIVE_ROAM_DURATION_MS)) {
      Serial.print(millis());  // Added millis to debug print
      Serial.println(F(" Auto-Cycle: Active roam ended. Initiating auto-pause (silent)."));
      stopMotors();
      INIT_ROBOT = false;
      playStartToneOnNextActivation = true;
      currentlyInAutoPausePhase = true;
      currentAutoPhaseStartTime = millis();
      if (currentState != IDLE) {
        changeState(IDLE);
      }
    }
  } else {  // currentlyInAutoPausePhase is true
    if (!INIT_ROBOT && (millis() - currentAutoPhaseStartTime >= AUTO_PAUSE_DURATION_MS)) {
      Serial.print(millis());  // Added millis to debug print
      Serial.println(F(" Auto-Cycle: Auto-pause ended. Resuming normal roam (silent via IDLE)."));
      INIT_ROBOT = true;
      playStartToneOnNextActivation = false;  // SILENT auto-wake-up
      currentlyInAutoPausePhase = false;
      currentAutoPhaseStartTime = millis();
      if (currentState != IDLE) {
        changeState(IDLE);
      } else {
        stateEntryTime = millis();
      }
    }
  }
}