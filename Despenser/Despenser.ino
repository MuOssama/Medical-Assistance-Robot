//despenser node
#include <ros.h>
#include <Servo.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32MultiArray.h>

// Pin Definitions
const int WATER_PUMP_PIN = 2;
const int SYRUP1_PUMP_PIN = 3;
const int SYRUP2_PUMP_PIN = 4;
const int TABLET1_SERVO_PIN = 5;
const int TABLET2_SERVO_PIN = 6;
const int DANGER_LED_PIN = 13;

// IR Sensors
const int IR_WATER_PIN = 7;
const int IR_SYRUP_PIN = 8;
const int IR_TABLET_PIN = 9;

// Timing Constants
const int PUMP_DURATION = 3000;  // 3 seconds for pumps
const int SERVO_DURATION = 1000; // 1 second for servo movement
const int DANGER_LED_DURATION = 1000; // 1 second for danger LED

// Servo positions
const int SERVO_START = 0;
const int SERVO_END = 180;

// Global variables
Servo tablet1Servo;
Servo tablet2Servo;
bool isDispensing = false;
unsigned long dispensingStartTime = 0;
int currentDispenseStep = 0;
int medications[5] = {0, 0, 0, 0, 0}; // [water, syrup1, syrup2, tablet1, tablet2]

// ROS setup
ros::NodeHandle nh;

void medicationCallback(const std_msgs::Int32MultiArray& msg) {
  if (isDispensing) return; // Don't start new dispense if currently dispensing
  
  // Copy medication array
  for (int i = 0; i < 5; i++) {
    medications[i] = msg.data[i];
  }
  
  // Start dispensing process
  isDispensing = true;
  currentDispenseStep = 0;
  dispensingStartTime = millis();
}

void dangerCallback(const std_msgs::Bool& msg) {
  if (msg.data) {
    digitalWrite(DANGER_LED_PIN, HIGH);
    delay(DANGER_LED_DURATION);
    digitalWrite(DANGER_LED_PIN, LOW);
  }
}

ros::Subscriber<std_msgs::Int32MultiArray> medSub("medication_array", medicationCallback);
ros::Subscriber<std_msgs::Bool> dangerSub("patient_danger", dangerCallback);

void setup() {
  // Initialize pins
  pinMode(WATER_PUMP_PIN, OUTPUT);
  pinMode(SYRUP1_PUMP_PIN, OUTPUT);
  pinMode(SYRUP2_PUMP_PIN, OUTPUT);
  pinMode(DANGER_LED_PIN, OUTPUT);
  pinMode(IR_WATER_PIN, INPUT);
  pinMode(IR_SYRUP_PIN, INPUT);
  pinMode(IR_TABLET_PIN, INPUT);
  
  // Initialize servos
  tablet1Servo.attach(TABLET1_SERVO_PIN);
  tablet2Servo.attach(TABLET2_SERVO_PIN);
  tablet1Servo.write(SERVO_START);
  tablet2Servo.write(SERVO_START);
  
  // Initialize ROS
  nh.initNode();
  nh.subscribe(medSub);
  nh.subscribe(dangerSub);
}

bool checkIRSensor(int sensorPin) {
  return digitalRead(sensorPin) == LOW; // LOW means object detected
}

void dispenseWater() {
  if (!checkIRSensor(IR_WATER_PIN)) return;
  digitalWrite(WATER_PUMP_PIN, HIGH);
  delay(PUMP_DURATION);
  digitalWrite(WATER_PUMP_PIN, LOW);
}

void dispenseSyrup(int pumpPin) {
  if (!checkIRSensor(IR_SYRUP_PIN)) return;
  digitalWrite(pumpPin, HIGH);
  delay(PUMP_DURATION);
  digitalWrite(pumpPin, LOW);
}

void dispenseTablet(Servo& servo) {
  if (!checkIRSensor(IR_TABLET_PIN)) return;
  servo.write(SERVO_END);
  delay(SERVO_DURATION);
  servo.write(SERVO_START);
}

void handleDispensing() {
  if (!isDispensing) return;
  
  unsigned long currentTime = millis();
  
  // State machine for dispensing
  switch (currentDispenseStep) {
    case 0: // Water
      if (medications[0]) dispenseWater();
      currentDispenseStep++;
      break;
      
    case 1: // Syrup 1
      if (medications[1]) dispenseSyrup(SYRUP1_PUMP_PIN);
      currentDispenseStep++;
      break;
      
    case 2: // Syrup 2
      if (medications[2]) dispenseSyrup(SYRUP2_PUMP_PIN);
      currentDispenseStep++;
      break;
      
    case 3: // Tablet 1
      if (medications[3]) dispenseTablet(tablet1Servo);
      currentDispenseStep++;
      break;
      
    case 4: // Tablet 2
      if (medications[4]) dispenseTablet(tablet2Servo);
      currentDispenseStep++;
      break;
      
    default:
      // Reset dispensing state
      isDispensing = false;
      currentDispenseStep = 0;
      break;
  }
}

void loop() {
  nh.spinOnce();
  handleDispensing();
  delay(100);
}
