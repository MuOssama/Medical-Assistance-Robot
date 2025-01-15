#include <string.h>
#include <ros.h>
#include <Servo.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>

// ROS node handle
ros::NodeHandle nh;

// Use byte type instead of int for boolean flags and small numbers
static const byte WATER_PUMP_PIN = 2;
static const byte SYRUP1_PUMP_PIN = 3;
static const byte SYRUP2_PUMP_PIN = 4;
static const byte TABLET1_SERVO_PIN = 5;
static const byte TABLET2_SERVO_PIN = 6;
static const byte DANGER_LED_PIN = 13;
static const byte IR_WATER_PIN = 7;
static const byte IR_SYRUP_PIN = 8;
static const byte IR_TABLET_PIN = 9;
static const byte STATUS_LED_DISPENSING = 10;
static const byte STATUS_LED_READY = 11;
static const byte STATUS_LED_ERROR = 12;

// Combine boolean flags into a single byte using bitfields
struct StatusFlags {
    unsigned isDispensing : 1;
    unsigned isPatientPresent : 1;
    unsigned systemHalted : 1;
} flags;

// Use uint16_t instead of unsigned long where possible
static const uint16_t PUMP_DURATION = 3000;
static const uint16_t SERVO_DURATION = 1000;
static const uint16_t DANGER_LED_DURATION = 1000;
static const uint16_t ERROR_TIMEOUT = 10000;
static const uint16_t SENSOR_CHECK_DELAY = 100;

static const byte SERVO_START = 0;
static const byte SERVO_END = 180;
static const byte MAX_DISPENSING_RETRIES = 3;

// Combine related variables into structs to optimize memory alignment
struct DispenserState {
    Servo tablet1Servo;
    Servo tablet2Servo;
    uint32_t dispensingStartTime;
    uint32_t lastSensorCheckTime;
    int8_t currentDispenseStep;
    int8_t dispensingErrors;
    int8_t medications[5];
    int16_t currentPatientId;
} state;

// Publishers
std_msgs::Bool dispensing_complete_msg;
ros::Publisher dispensing_complete_pub("dispensing_complete", &dispensing_complete_msg);

// Function declarations
void dispenseWater();
void dispenseSyrup(byte pumpPin);
void dispenseTablet(Servo& servo);
void handleDispenseError();
void resetDispenser();
inline void setStatusLEDs(bool dispensing, bool ready, bool error);
bool checkSensor(byte sensorPin);

// Updated callback implementations
void dispensingStateCallback(const std_msgs::Bool& msg) {
    if(msg.data && !flags.isDispensing) {
        flags.isDispensing = true;
        state.currentDispenseStep = 0;
        state.dispensingStartTime = millis();
        state.dispensingErrors = 0;
        setStatusLEDs(true, false, false);
    }
}

void patientCallback(const std_msgs::Int32& msg) {
    state.currentPatientId = msg.data;
}

void medicationCallback(const std_msgs::Int32MultiArray& msg) {
    for(byte i = 0; i < min(msg.data_length, (unsigned int)5); i++) {
        state.medications[i] = msg.data[i];
    }
}

// Subscribers
ros::Subscriber<std_msgs::Bool> dispensing_state_sub("dispensing_state", &dispensingStateCallback);
ros::Subscriber<std_msgs::Int32> patient_sub("current_patient", &patientCallback);
ros::Subscriber<std_msgs::Int32MultiArray> med_sub("medication_array", &medicationCallback);

// Helper functions from original code
inline void setStatusLEDs(bool dispensing, bool ready, bool error) {
    digitalWrite(STATUS_LED_DISPENSING, dispensing);
    digitalWrite(STATUS_LED_READY, ready);
    digitalWrite(STATUS_LED_ERROR, error);
}

bool checkSensor(byte sensorPin) {
    byte readings = 0;
    for(byte i = 0; i < 5; i++) {
        readings += digitalRead(sensorPin);
        delay(10);
    }
    return readings >= 3;
}

void dispenseWater() {
    digitalWrite(WATER_PUMP_PIN, HIGH);
    delay(PUMP_DURATION);
    digitalWrite(WATER_PUMP_PIN, LOW);
}

void dispenseSyrup(byte pumpPin) {
    digitalWrite(pumpPin, HIGH);
    delay(PUMP_DURATION);
    digitalWrite(pumpPin, LOW);
}

void dispenseTablet(Servo& servo) {
    servo.write(SERVO_END);
    delay(SERVO_DURATION);
    servo.write(SERVO_START);
    delay(SERVO_DURATION);
}

bool dispenseWithRetry(void (*dispenseFunc)(), byte sensorPin) {
    for(byte retry = 0; retry < MAX_DISPENSING_RETRIES; retry++) {
        if(!checkSensor(sensorPin)) {
            state.dispensingErrors++;
            delay(1000);
            continue;
        }
        
        dispenseFunc();
        delay(500);
        
        if(checkSensor(sensorPin)) {
            return true;
        }
        
        state.dispensingErrors++;
        delay(1000);
    }
    return false;
}

void handleDispenseError() {
    setStatusLEDs(false, false, true);
    resetDispenser();
}

void resetDispenser() {
    flags.isDispensing = false;
    state.currentDispenseStep = 0;
    state.dispensingErrors = 0;
    
    digitalWrite(WATER_PUMP_PIN, LOW);
    digitalWrite(SYRUP1_PUMP_PIN, LOW);
    digitalWrite(SYRUP2_PUMP_PIN, LOW);
    
    state.tablet1Servo.write(SERVO_START);
    state.tablet2Servo.write(SERVO_START);
    
    setStatusLEDs(false, true, false);
}

void handleDispensing() {
    if(!flags.isDispensing) return;
    
    if(millis() - state.dispensingStartTime > ERROR_TIMEOUT) {
        handleDispenseError();
        return;
    }
    
    if(state.dispensingErrors > MAX_DISPENSING_RETRIES * 2) {
        handleDispenseError();
        return;
    }
    
    switch(state.currentDispenseStep) {
        case 0:
            if(state.medications[0] && dispenseWithRetry(dispenseWater, IR_WATER_PIN)) {
                state.currentDispenseStep++;
            }
            if(!state.medications[0]) state.currentDispenseStep++;
            break;
            
        case 1:
            if(state.medications[1] && dispenseWithRetry([]() { dispenseSyrup(SYRUP1_PUMP_PIN); }, IR_SYRUP_PIN)) {
                state.currentDispenseStep++;
            }
            if(!state.medications[1]) state.currentDispenseStep++;
            break;
            
        case 2:
            if(state.medications[2] && dispenseWithRetry([]() { dispenseSyrup(SYRUP2_PUMP_PIN); }, IR_SYRUP_PIN)) {
                state.currentDispenseStep++;
            }
            if(!state.medications[2]) state.currentDispenseStep++;
            break;
            
        case 3:
            if(state.medications[3] && dispenseWithRetry([]() { dispenseTablet(state.tablet1Servo); }, IR_TABLET_PIN)) {
                state.currentDispenseStep++;
            }
            if(!state.medications[3]) state.currentDispenseStep++;
            break;
            
        case 4:
            if(state.medications[4] && dispenseWithRetry([]() { dispenseTablet(state.tablet2Servo); }, IR_TABLET_PIN)) {
                state.currentDispenseStep++;
            }
            if(!state.medications[4]) state.currentDispenseStep++;
            break;
            
        default:
            // Publish completion message
            dispensing_complete_msg.data = true;
            dispensing_complete_pub.publish(&dispensing_complete_msg);
            resetDispenser();
            break;
    }
}

void setup() {
    nh.initNode();
    nh.subscribe(dispensing_state_sub);
    nh.subscribe(patient_sub);
    nh.subscribe(med_sub);
    nh.advertise(dispensing_complete_pub);
    
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
    
    state.tablet1Servo.attach(TABLET1_SERVO_PIN);
    state.tablet2Servo.attach(TABLET2_SERVO_PIN);
    state.tablet1Servo.write(SERVO_START);
    state.tablet2Servo.write(SERVO_START);
    
    setStatusLEDs(false, true, false);
}

void loop() {
    nh.spinOnce();
    
    if(millis() - state.lastSensorCheckTime >= SENSOR_CHECK_DELAY) {
        handleDispensing();
        state.lastSensorCheckTime = millis();
    }
    
    delay(10);
}
