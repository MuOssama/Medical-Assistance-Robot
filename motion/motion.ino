#include <WiFi.h>
#include <HTTPClient.h>
#include <BluetoothSerial.h>
#include <ArduinoJson.h>

// Config
const char* ssid = "your_ssid";
const char* password = "your_password";
const char* serverUrl = "http://your_server_ip:5000";

// Pins
const byte MOTOR_PINS[] = {25, 26, 27, 32, 33, 14}; // STEP, DIR, EN for L & R
const byte IR_PINS[] = {36, 39, 34, 35, 15, 4, 16, 17, 18, 19};

// State machine
byte currentState = 0;  // 0:IDLE, 1:FINDING, 2:FOLLOWING, 3:AT_PATIENT, 4:WAITING
byte currentCommand = 0; // 0:STOP, 1:FORWARD, 2:LINE_FOLLOW
byte currentPatient = 0;
const byte targetPatient = 3;
const byte baseSpeed = 50;

BluetoothSerial SerialBT;

void setMotors(bool enable) {
    digitalWrite(MOTOR_PINS[2], !enable);  // L_EN
    digitalWrite(MOTOR_PINS[5], !enable);  // R_EN
}

bool updateServer(byte id, bool state) {
    if(WiFi.status() != WL_CONNECTED) return false;
    
    HTTPClient http;
    String url = String(serverUrl) + "/api/patients/" + String(id);
    
    StaticJsonDocument<64> doc;
    doc["dispensing_state"] = state;
    
    String payload;
    serializeJson(doc, payload);
    
    http.begin(url);
    http.addHeader("Content-Type", "application/json");
    int code = http.PATCH(payload);
    http.end();
    
    return (code == 200);
}

// Simplified line detection - returns true if robot is on a line intersection
bool checkLine() {
    byte count = 0;
    for(byte i = 0; i < 10; i++) {
        if(digitalRead(IR_PINS[i])) count++;
    }
    return count >= 8;
}

// Simplified line position calculation
int8_t getLinePosition() {
    byte leftSum = 0, rightSum = 0;
    
    // Simple left/right balance detection
    for(byte i = 0; i < 5; i++) {
        if(digitalRead(IR_PINS[i])) leftSum++;
        if(digitalRead(IR_PINS[i + 5])) rightSum++;
    }
    
    if(leftSum > rightSum) return -1;
    if(rightSum > leftSum) return 1;
    return 0;
}

void move(int16_t left, int16_t right) {
    left = constrain(left, -100, 100);
    right = constrain(right, -100, 100);
    
    digitalWrite(MOTOR_PINS[1], left > 0);   // L_DIR
    digitalWrite(MOTOR_PINS[4], right > 0);  // R_DIR
    
    byte steps = abs(max(left, right));
    while(steps--) {
        if(abs(left)) digitalWrite(MOTOR_PINS[0], HIGH);   // L_STEP
        if(abs(right)) digitalWrite(MOTOR_PINS[3], HIGH);  // R_STEP
        delayMicroseconds(500);
        digitalWrite(MOTOR_PINS[0], LOW);
        digitalWrite(MOTOR_PINS[3], LOW);
        delayMicroseconds(500);
    }
}

void handleCommand(const String& cmd) {
    if(cmd == "AUX1" && !currentState) {
        currentState = 1;
        currentCommand = 2;
        setMotors(true);
        currentPatient = 0;
    }
    else if(cmd == "STOP" || cmd == "Emergency") {
        currentCommand = currentState = 0;
        setMotors(false);
    }
    else if(!currentState) {
        setMotors(true);
        if(cmd == "Forward") move(baseSpeed, baseSpeed);
        else if(cmd == "Backward") move(-baseSpeed, -baseSpeed);
        else if(cmd == "Left") move(-baseSpeed/2, baseSpeed/2);
        else if(cmd == "Right") move(baseSpeed/2, -baseSpeed/2);
        else if(cmd == "Stop") move(0, 0);
    }
}

void followLine() {
    if(currentCommand != 2) return;
    
    switch(currentState) {
        case 1:  // FINDING
            if(checkLine()) {
                currentState = 2;
            } else {
                move(baseSpeed/2, -baseSpeed/2);  // Rotate to find line
            }
            break;

        case 2:  // FOLLOWING
            if(checkLine()) {
                currentState = 3;
                currentPatient++;
                move(0, 0);
                updateServer(currentPatient, true);
            } else {
                // Simple line following using left/right balance
                int8_t pos = getLinePosition();
                if(pos < 0) {
                    move(baseSpeed/2, baseSpeed);      // Turn left
                } else if(pos > 0) {
                    move(baseSpeed, baseSpeed/2);      // Turn right
                } else {
                    move(baseSpeed, baseSpeed);        // Go straight
                }
            }
            break;

        case 3:  // AT_PATIENT
            if(currentPatient < targetPatient) {
                delay(5000);
                currentState = 2;
            } else {
                currentState = 4;
                setMotors(false);
            }
            break;
    }
}

void setup() {
    WiFi.begin(ssid, password);
    
    for(byte i = 0; i < 6; i++) {
        pinMode(MOTOR_PINS[i], OUTPUT);
    }
    for(byte i = 0; i < 10; i++) {
        pinMode(IR_PINS[i], INPUT);
    }
    
    SerialBT.begin("MedRobot");
    setMotors(false);
}

void loop() {
    if(SerialBT.available()) {
        handleCommand(SerialBT.readStringUntil('\n'));
    }
    followLine();
    delay(10);
}
