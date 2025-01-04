//motion node
#include <WiFi.h>
#include <WiFiClient.h>
#include <BluetoothSerial.h>

// Create BluetoothSerial instance
BluetoothSerial SerialBT;

// Motor pins (TB6600)
#define MOTOR_L_STEP 25
#define MOTOR_L_DIR  26
#define MOTOR_L_EN   27
#define MOTOR_R_STEP 32
#define MOTOR_R_DIR  33
#define MOTOR_R_EN   14

// IR Sensors (10 sensors)
const int IR_PINS[10] = {36, 39, 34, 35, 15, 4, 16, 17, 18, 19};

// States
enum RobotState {IDLE, FINDING_LINE, FOLLOWING_LINE, AT_PATIENT, WAITING};
enum Command {FORWARD, BACKWARD, LEFT, RIGHT, STOP, FOLLOW_LINE, EMERGENCY};

// Global variables
RobotState currentState = IDLE;
Command currentCommand = STOP;
int currentPatient = 0;
int targetPatient = 3;  // Maximum patients
int baseSpeed = 50;

// PID constants and variables
float Kp = 25, Ki = 0, Kd = 15;
float error = 0, P = 0, I = 0, D = 0;
float previousError = 0;

// Function declarations
void enableMotors();
void disableMotors();
void moveRobot(int leftSpeed, int rightSpeed);
bool checkAllSensors();
float calculatePID();
void handleMotion();
void processCommand(String cmd);

void setup() {
  // Motor setup
  for(int i = 0; i < 6; i++) {
    pinMode(MOTOR_L_STEP + i, OUTPUT);
  }

  // IR setup
  for(int i = 0; i < 10; i++) {
    pinMode(IR_PINS[i], INPUT);
  }

  SerialBT.begin("MedicalRobot");
}

void processCommand(String cmd) {
  if(cmd == "AUX1" && currentState == IDLE) {
    currentState = FINDING_LINE;
    currentCommand = FOLLOW_LINE;
    enableMotors();
  }
  else if(cmd == "Accelerate") {
    currentCommand = FORWARD;
    currentState = IDLE;
  }
  else if(cmd == "Brake") {
    currentCommand = STOP;
    currentState = IDLE;
  }
  else if(cmd == "Steer left") {
    currentCommand = LEFT;
    currentState = IDLE;
  }
  else if(cmd == "Steer right") {
    currentCommand = RIGHT;
    currentState = IDLE;
  }
  else if(cmd == "Emergency") {
    currentCommand = EMERGENCY;
    currentState = IDLE;
    disableMotors();
  }
}

void enableMotors() {
  digitalWrite(MOTOR_L_EN, LOW);
  digitalWrite(MOTOR_R_EN, LOW);
}

void disableMotors() {
  digitalWrite(MOTOR_L_EN, HIGH);
  digitalWrite(MOTOR_R_EN, HIGH);
}

bool checkAllSensors() {
  int blackCount = 0;
  for(int i = 0; i < 10; i++) {
    if(digitalRead(IR_PINS[i])) blackCount++;
  }
  return blackCount >= 8;  // Allow for some sensor error
}

float calculatePID() {
  int position = 0;
  int sensorCount = 0;

  for(int i = 0; i < 10; i++) {
    if(digitalRead(IR_PINS[i])) {
      position += i * 1000;
      sensorCount++;
    }
  }

  if(sensorCount == 0) return previousError;

  position = position / sensorCount;
  error = position - 4500;  // Center point (9000/2)

  P = error;
  I += error;
  D = error - previousError;
  previousError = error;

  return (Kp * P) + (Ki * I) + (Kd * D);
}

void moveRobot(int leftSpeed, int rightSpeed) {
  digitalWrite(MOTOR_L_DIR, leftSpeed > 0);
  digitalWrite(MOTOR_R_DIR, rightSpeed > 0);

  for(int i = 0; i < abs(max(leftSpeed, rightSpeed))/10; i++) {
    digitalWrite(MOTOR_L_STEP, HIGH);
    digitalWrite(MOTOR_R_STEP, HIGH);
    delayMicroseconds(500);
    digitalWrite(MOTOR_L_STEP, LOW);
    digitalWrite(MOTOR_R_STEP, LOW);
    delayMicroseconds(500);
  }
}

void handleMotion() {
  if(currentCommand == FOLLOW_LINE) {
    switch(currentState) {
      case FINDING_LINE:
        if(checkAllSensors()) {
          currentState = FOLLOWING_LINE;
        } else {
          moveRobot(baseSpeed, -baseSpeed);  // Rotate to find line
        }
        break;

      case FOLLOWING_LINE:
        if(checkAllSensors()) {
          currentState = AT_PATIENT;
          currentPatient++;
          moveRobot(0, 0);
        } else {
          float pidValue = calculatePID();
          int leftSpeed = baseSpeed - pidValue;
          int rightSpeed = baseSpeed + pidValue;
          moveRobot(leftSpeed, rightSpeed);
        }
        break;

      case AT_PATIENT:
        if(currentPatient < targetPatient) {
          currentState = FOLLOWING_LINE;
        } else {
          currentState = WAITING;
        }
        break;

      case WAITING:
        moveRobot(0, 0);
        break;
    }
  } else {
    // Manual control
    switch(currentCommand) {
      case FORWARD:
        moveRobot(baseSpeed, baseSpeed);
        break;
      case BACKWARD:
        moveRobot(-baseSpeed, -baseSpeed);
        break;
      case LEFT:
        moveRobot(-baseSpeed/2, baseSpeed/2);
        break;
      case RIGHT:
        moveRobot(baseSpeed/2, -baseSpeed/2);
        break;
      case STOP:
      case EMERGENCY:
        moveRobot(0, 0);
        break;
    }
  }
}

void loop() {
  if(SerialBT.available()) {
    String command = SerialBT.readStringUntil('\n');
    processCommand(command);
  }

  handleMotion();
  delay(10);
}
