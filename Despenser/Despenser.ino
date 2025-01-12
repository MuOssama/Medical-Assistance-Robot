#include <ros.h>
#include <Servo.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>

// ROS node handle
ros::NodeHandle nh;

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

// Status LEDs
const int STATUS_LED_DISPENSING = 10;
const int STATUS_LED_READY = 11;
const int STATUS_LED_ERROR = 12;

// Timing Constants
const int PUMP_DURATION = 3000;    // 3 seconds for pumps
const int SERVO_DURATION = 1000;   // 1 second for servo movement
const int DANGER_LED_DURATION = 1000;
const int ERROR_TIMEOUT = 10000;   // 10 second timeout for dispensing
const int SENSOR_CHECK_DELAY = 100; // 100ms between sensor checks

// Servo positions
const int SERVO_START = 0;
const int SERVO_END = 180;

// Global variables
Servo tablet1Servo;
Servo tablet2Servo;
bool isDispensing = false;
bool isPatientPresent = false;
unsigned long dispensingStartTime = 0;
unsigned long lastSensorCheckTime = 0;
int currentDispenseStep = 0;
int medications[5] = {0, 0, 0, 0, 0}; // [water, syrup1, syrup2, tablet1, tablet2]
int currentPatientId = 0;
bool systemHalted = false;
int dispensingErrors = 0;
const int MAX_DISPENSING_RETRIES = 3;

// Publishers
std_msgs::Bool halt_msg;
ros::Publisher halt_pub("halt_system", &halt_msg);

// Status message publisher
std_msgs::Int32 status_msg;
ros::Publisher status_pub("dispenser_status", &status_msg);

// Function declarations
void initializeDispenser();
bool checkSensor(int sensorPin);
void setStatusLEDs(bool dispensing, bool ready, bool error);
bool dispenseWithRetry(void (*dispenseFunc)(), int sensorPin);
void handleDispenseError();
void resetDispenser();

// Callback for patient reached message
void reachedCallback(const std_msgs::Bool& msg) {
    isPatientPresent = msg.data;
    if(isPatientPresent && !isDispensing) {
        // Start dispensing process
        isDispensing = true;
        currentDispenseStep = 0;
        dispensingStartTime = millis();
        dispensingErrors = 0;
        
        // Halt the robot
        halt_msg.data = true;
        halt_pub.publish(&halt_msg);
        
        // Set status LEDs
        setStatusLEDs(true, false, false);
        
        // Publish status
        status_msg.data = 1; // 1 = Started dispensing
        status_pub.publish(&status_msg);
    }
}

// Callback for current patient ID
void patientCallback(const std_msgs::Int32& msg) {
    currentPatientId = msg.data;
}

// Callback for medication array
void medicationCallback(const std_msgs::Int32MultiArray& msg) {
    if(msg.data.size() == 5) {
        for(int i = 0; i < 5; i++) {
            medications[i] = msg.data[i];
        }
    }
}

// ROS subscribers
ros::Subscriber<std_msgs::Bool> reached_sub("patient_reached", &reachedCallback);
ros::Subscriber<std_msgs::Int32> patient_sub("current_patient", &patientCallback);
ros::Subscriber<std_msgs::Int32MultiArray> med_sub("medication_array", &medicationCallback);

void setup() {
    // Initialize ROS
    nh.initNode();
    nh.subscribe(reached_sub);
    nh.subscribe(patient_sub);
    nh.subscribe(med_sub);
    nh.advertise(halt_pub);
    nh.advertise(status_pub);
    
    initializeDispenser();
}

void initializeDispenser() {
    // Initialize pins
    pinMode(WATER_PUMP_PIN, OUTPUT);
    pinMode(SYRUP1_PUMP_PIN, OUTPUT);
    pinMode(SYRUP2_PUMP_PIN, OUTPUT);
    pinMode(DANGER_LED_PIN, OUTPUT);
    pinMode(STATUS_LED_DISPENSING, OUTPUT);
    pinMode(STATUS_LED_READY, OUTPUT);
    pinMode(STATUS_LED_ERROR, OUTPUT);
    pinMode(IR_WATER_PIN, INPUT);
    pinMode(IR_SYRUP_PIN, INPUT);
    pinMode(IR_TABLET_PIN, INPUT);
    
    // Initialize servos
    tablet1Servo.attach(TABLET1_SERVO_PIN);
    tablet2Servo.attach(TABLET2_SERVO_PIN);
    tablet1Servo.write(SERVO_START);
    tablet2Servo.write(SERVO_START);
    
    // Set initial status
    setStatusLEDs(false, true, false);
}

void setStatusLEDs(bool dispensing, bool ready, bool error) {
    digitalWrite(STATUS_LED_DISPENSING, dispensing);
    digitalWrite(STATUS_LED_READY, ready);
    digitalWrite(STATUS_LED_ERROR, error);
}

bool checkSensor(int sensorPin) {
    int readings = 0;
    for(int i = 0; i < 5; i++) {  // Take 5 readings
        readings += digitalRead(sensorPin);
        delay(10);
    }
    return readings >= 3;  // Return true if majority of readings detect object
}

void dispenseWater() {
    digitalWrite(WATER_PUMP_PIN, HIGH);
    delay(PUMP_DURATION);
    digitalWrite(WATER_PUMP_PIN, LOW);
}

void dispenseSyrup(int pumpPin) {
    digitalWrite(pumpPin, HIGH);
    delay(PUMP_DURATION);
    digitalWrite(pumpPin, LOW);
}

void dispenseTablet(Servo& servo) {
    servo.write(SERVO_END);
    delay(SERVO_DURATION);
    servo.write(SERVO_START);
    delay(SERVO_DURATION);  // Wait for tablet to fall
}

bool dispenseWithRetry(void (*dispenseFunc)(), int sensorPin) {
    for(int retry = 0; retry < MAX_DISPENSING_RETRIES; retry++) {
        if(!checkSensor(sensorPin)) {
            dispensingErrors++;
            delay(1000);  // Wait before retry
            continue;
        }
        
        dispenseFunc();
        delay(500);  // Wait for dispensing to complete
        
        if(checkSensor(sensorPin)) {
            return true;
        }
        
        dispensingErrors++;
        delay(1000);  // Wait before retry
    }
    return false;
}

void handleDispenseError() {
    setStatusLEDs(false, false, true);
    status_msg.data = -1;  // -1 = Error
    status_pub.publish(&status_msg);
    
    // Reset the dispenser
    resetDispenser();
}

void resetDispenser() {
    isDispensing = false;
    currentDispenseStep = 0;
    dispensingErrors = 0;
    
    // Turn off all pumps
    digitalWrite(WATER_PUMP_PIN, LOW);
    digitalWrite(SYRUP1_PUMP_PIN, LOW);
    digitalWrite(SYRUP2_PUMP_PIN, LOW);
    
    // Reset servos
    tablet1Servo.write(SERVO_START);
    tablet2Servo.write(SERVO_START);
    
    // Release halt
    halt_msg.data = false;
    halt_pub.publish(&halt_msg);
    
    // Reset status
    setStatusLEDs(false, true, false);
}

void handleDispensing() {
    if(!isDispensing) return;
    
    // Check for timeout
    if(millis() - dispensingStartTime > ERROR_TIMEOUT) {
        handleDispenseError();
        return;
    }
    
    // Check for too many errors
    if(dispensingErrors > MAX_DISPENSING_RETRIES * 2) {
        handleDispenseError();
        return;
    }
    
    switch(currentDispenseStep) {
        case 0:  // Water
            if(medications[0]) {
                if(dispenseWithRetry(dispenseWater, IR_WATER_PIN)) {
                    currentDispenseStep++;
                }
            } else {
                currentDispenseStep++;
            }
            break;
            
        case 1:  // Syrup 1
            if(medications[1]) {
                if(dispenseWithRetry([]() { dispenseSyrup(SYRUP1_PUMP_PIN); }, IR_SYRUP_PIN)) {
                    currentDispenseStep++;
                }
            } else {
                currentDispenseStep++;
            }
            break;
            
        case 2:  // Syrup 2
            if(medications[2]) {
                if(dispenseWithRetry([]() { dispenseSyrup(SYRUP2_PUMP_PIN); }, IR_SYRUP_PIN)) {
                    currentDispenseStep++;
                }
            } else {
                currentDispenseStep++;
            }
            break;
            
        case 3:  // Tablet 1
            if(medications[3]) {
                if(dispenseWithRetry([]() { dispenseTablet(tablet1Servo); }, IR_TABLET_PIN)) {
                    currentDispenseStep++;
                }
            } else {
                currentDispenseStep++;
            }
            break;
            
        case 4:  // Tablet 2
            if(medications[4]) {
                if(dispenseWithRetry([]() { dispenseTablet(tablet2Servo); }, IR_TABLET_PIN)) {
                    currentDispenseStep++;
                }
            } else {
                currentDispenseStep++;
            }
            break;
            
        default:
            // Dispensing complete
            status_msg.data = 2;  // 2 = Completed successfully
            status_pub.publish(&status_msg);
            
            // Reset and release halt
            resetDispenser();
            break;
    }
}

void loop() {
    nh.spinOnce();
    
    // Only check sensors periodically to avoid overwhelming the system
    if(millis() - lastSensorCheckTime >= SENSOR_CHECK_DELAY) {
        handleDispensing();
        lastSensorCheckTime = millis();
    }
    
    delay(10);
}
