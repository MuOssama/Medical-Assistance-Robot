#include <WiFi.h>
#include <HTTPClient.h>
#include <BluetoothSerial.h>
#include <Preferences.h>
#include <Stepper.h>


#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b)) // Returns the larger of a and b

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Config storage
Preferences preferences;
String ssid;
String password;
String serverIP;
String serverUrl;
bool isConfigMode = false;

// Pin Definitions
const uint16_t LEFT_MOTOR[] = {25, 26, 27};   // STEP, DIR, EN for left TB6600
const uint16_t RIGHT_MOTOR[] = {32, 33, 35};  // STEP, DIR, EN for right TB6600
const uint16_t PINS_IR[] = {18, 19, 21, 22, 23};  // IR sensors from left to right

// Motion Control Constants
const uint16_t STEPS_PER_REVOLUTION = 200;
const uint16_t MICROSTEPS = 1;
const uint16_t MAX_SPEED = 1500;
const uint16_t TURN_SPEED = 1500;
const unsigned long MOVE_DURATION = 3000; // ms
// Instead of just step/dir pins, add steps per revolution
Stepper leftStepper(STEPS_PER_REVOLUTION, LEFT_MOTOR[0], LEFT_MOTOR[1]); // Add enable pin
Stepper rightStepper(STEPS_PER_REVOLUTION, RIGHT_MOTOR[0], RIGHT_MOTOR[1]);
// Movement State
struct MovementState {
    unsigned long moveStartTime;
    bool isMoving;
    int16_t leftSpeed;
    int16_t rightSpeed;
    uint16_t state;     // 0:IDLE, 1:FINDING, 2:FOLLOWING, 3:AT_PATIENT, 4:WAITING
    uint16_t command;   // 0:STOP, 1:MANUAL, 2:LINE_FOLLOW
    uint16_t patient;
} movement = {0, false, 0, 0, 0, 0, 0};

const uint16_t TARGET_PATIENT = 1;
BluetoothSerial SerialBT;

// WiFi and Server Functions
void updateServerUrl() {
    serverUrl = "http://" + serverIP + ":5000/api/patients/";
}

void loadSettings() {
    preferences.begin("settings", false);
    ssid = preferences.getString("ssid", "");
    password = preferences.getString("password", "");
    serverIP = preferences.getString("ip", "192.168.1.100");
    
    Serial.println("\nLoaded settings:");
    Serial.print("SSID length: ");
    Serial.println(ssid.length());
    Serial.print("Password length: ");
    Serial.println(password.length());
    Serial.print("Server IP: ");
    Serial.println(serverIP);
    
    updateServerUrl();
    preferences.end();
}
void saveSettings() {
    preferences.begin("settings", false);
    preferences.putString("ssid", ssid);
    preferences.putString("password", password);
    preferences.putString("ip", serverIP);
    preferences.end();
}

bool connectWiFi() {
    if (ssid.length() == 0) {
        Serial.println("No SSID configured");
        return false;
    }
    
    Serial.println("Attempting to connect to WiFi");
    Serial.print("SSID: ");
    Serial.println(ssid);
    Serial.print("Password length: ");
    Serial.println(password.length());
    
    WiFi.begin(ssid.c_str(), password.c_str());
    uint16_t attempts = 0;
    const uint16_t MAX_ATTEMPTS = 40; // Increased from 20 to 40 attempts
    
    while (WiFi.status() != WL_CONNECTED && attempts < MAX_ATTEMPTS) {
        delay(500);
        Serial.print(".");
        attempts++;
        
        // Print WiFi status for debugging
        switch(WiFi.status()) {
            case WL_NO_SHIELD: Serial.println("No WiFi shield"); break;
            case WL_IDLE_STATUS: Serial.println("Idle"); break;
            case WL_NO_SSID_AVAIL: Serial.println("No SSID available"); break;
            case WL_SCAN_COMPLETED: Serial.println("Scan completed"); break;
            case WL_CONNECT_FAILED: Serial.println("Connection failed"); break;
            case WL_CONNECTION_LOST: Serial.println("Connection lost"); break;
            case WL_DISCONNECTED: Serial.println("Disconnected"); break;
        }
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConnected successfully!");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        return true;
    } else {
        Serial.println("\nFailed to connect! Final status: ");
        Serial.println(WiFi.status());
        return false;
    }
}

void setMotors(bool enable) {
    digitalWrite(LEFT_MOTOR[2], !enable);   // Enable pin is typically active low
    digitalWrite(RIGHT_MOTOR[2], !enable);
}
void stepMotor(const uint16_t* motor_pins, bool direction, uint16_t steps) {
    digitalWrite(motor_pins[1], direction);  // Set direction
    
    // Add small delay after direction change
    delayMicroseconds(100);
    
    for(uint16_t i = 0; i < steps; i++) {
        digitalWrite(motor_pins[0], HIGH);
        delayMicroseconds(1);  // Increased pulse width
        digitalWrite(motor_pins[0], LOW);
        delayMicroseconds(1);  // Increased delay between pulses
    }
}

void moveRobot(int16_t leftSpeed, int16_t rightSpeed) {
    // Convert speed to RPM 
    int leftRPM = map(abs(leftSpeed), 0, MAX_SPEED, 0, 1500);
    int rightRPM = map(abs(rightSpeed), 0, MAX_SPEED, 0, 1500);
    
    leftStepper.setSpeed(leftRPM);
    rightStepper.setSpeed(rightRPM);
    
    // Use step() with number of steps
    leftStepper.step(leftSpeed > 0 ? 100 : -100);
    rightStepper.step(rightSpeed > 0 ? 100 : -100);
}

// Line Following Functions
bool checkLine() {
    uint16_t count = 0;
    for(uint16_t i = 0; i < 5; i++) {
        count += digitalRead(PINS_IR[i]);
    }
    return count >= 4;
}

int16_t getLinePos() {
    int16_t position = 0;
    uint16_t sensors = 0;
    
    for(uint16_t i = 0; i < 5; i++) {
        if(digitalRead(PINS_IR[i])) {
            position += (i - 2) * 100;
            sensors++;
        }
    }
    
    if(sensors == 0) return 0;
    position /= sensors;
    
    if(abs(position) < 50) return 0;
    return position < 0 ? -1 : 1;
}

// Server Communication
bool updateServer(uint16_t id, bool isDispensing) {
    if(WiFi.status() != WL_CONNECTED) return false;
    
    HTTPClient http;
    String fullUrl = serverUrl + String(id);
    http.begin(fullUrl);
    http.addHeader("Content-Type", "application/json");
    
    String payload = "{\"is_dispensing\":" + String(isDispensing ? "true" : "false") + "}";
    int code = http.POST(payload);
    http.end();
    
    return code == 200;
}

// Movement Control
void startMovement(int16_t leftSpeed, int16_t rightSpeed) {
    movement.moveStartTime = millis();
    movement.isMoving = true;
    movement.leftSpeed = leftSpeed;
    movement.rightSpeed = rightSpeed;
    
    // Set stepper speeds
    leftStepper.setSpeed(abs(leftSpeed));
    rightStepper.setSpeed(abs(rightSpeed));
    
    // Enable motors by setting initial movement
    leftStepper.step(leftSpeed > 0 ? 1 : -1);
    rightStepper.step(rightSpeed > 0 ? 1 : -1);
}
void stopMovement() {
    movement.isMoving = false;
    movement.leftSpeed = 0;
    movement.rightSpeed = 0;
    setMotors(false);
}

void updateMovement() {
    if (movement.isMoving) {
        if (millis() - movement.moveStartTime >= MOVE_DURATION) {
            stopMovement();
        } else {
            moveRobot(movement.leftSpeed, movement.rightSpeed);
        }
    }
}

// Command Handling
void handleCommand(String cmd) {
    if(cmd == "X" && !movement.state) {
        movement.state = 1;
        movement.command = 2;
        setMotors(true);
        movement.patient = 0;
        Serial.println("Follow Line");
    }
    else if(cmd == "STOP" || cmd == "Emergency") {
        movement.command = movement.state = 0;
        stopMovement();
    }
    else if(!movement.state && !movement.isMoving) {
        if(cmd == "U") {
            startMovement(MAX_SPEED, MAX_SPEED);
            Serial.println("forward");
        }
        else if(cmd == "r") {
            startMovement(-MAX_SPEED, -MAX_SPEED);
            Serial.println("backward");
        }
        else if(cmd == "L") {
            startMovement(TURN_SPEED,-TURN_SPEED);  // Left motor backward, Right motor forward
            Serial.println("left");
        }
        else if(cmd == "R") {
            startMovement(-TURN_SPEED, TURN_SPEED);  // Left motor forward, Right motor backward
            Serial.println("right");
        }
    }
}

void followLine() {
if(movement.command != 2 || movement.isMoving) return;
    
    static float lastError = 0;
    static float integral = 0;
    const float KP = 1.5, KI = 0.01, KD = 0.5;
    
    switch(movement.state) {
        case 1: // FINDING
            if(checkLine()) {
                movement.state = 2;
            } else {
                static int searchDirection = 1;
                static unsigned long searchStartTime = millis();
                
                if (millis() - searchStartTime > 2000) {
                    searchDirection *= -1;
                    searchStartTime = millis();
                }
                
                // Convert to RPM and use step()
                int searchRPM = 1500;  // Slower search speed
                leftStepper.setSpeed(searchRPM);
                rightStepper.setSpeed(searchRPM);
                leftStepper.step(5 * searchDirection);
                rightStepper.step(-5 * searchDirection);
            }
            break;

        case 2: // FOLLOWING
            if(checkLine()) {
                movement.state = 3;
                movement.patient++;
                leftStepper.step(0);
                rightStepper.step(0);
                updateServer(movement.patient, true);
            } else {
                float error = getLinePos();
                integral = constrain(integral + error, -50, 50);
                float derivative = error - lastError;
                
                float correction = KP * error + KI * integral + KD * derivative;
                lastError = error;
                
                int baseRPM = 1500;  // Base RPM 
                int leftRPM = baseRPM - correction;
                int rightRPM = baseRPM + correction;
                
                // Constrain RPM
                leftRPM = constrain(leftRPM, 0, 120);
                rightRPM = constrain(rightRPM, 0, 120);
                
                leftStepper.setSpeed(leftRPM);
                rightStepper.setSpeed(rightRPM);
                leftStepper.step(error > 0 ? 50 : -50);
                rightStepper.step(error > 0 ? -50 : 50);
            }
            break;
        case 3: // AT_PATIENT
            if(movement.patient < TARGET_PATIENT) {
                delay(5000);
                updateServer(movement.patient, false);
                movement.state = 2;
            } else {
                movement.state = 4;
                setMotors(false);
            }
            break;
    }
}

void handleConfig() {
    Serial.println("\n=== Configuration Mode ===");
    Serial.println("Current settings:");
    Serial.println("SSID: " + ssid);
    Serial.println("Password: " + password);  // Added line to show password
    Serial.println("Server IP: " + serverIP); // Added line to show current server IP
    Serial.println("\n1. Set WiFi credentials");
    Serial.println("2. Set server IP");
    Serial.println("3. Save and exit");
    Serial.println("Please enter your choice (1-3):");
    
    while (true) {
        if (Serial.available()) {
            char choice = Serial.read();
            
            switch (choice) {
        case '1':
            Serial.println("\nWiFi Setup:");
            Serial.println("Enter SSID, wait for password prompt");
            
            // Clear any existing serial data
            while(Serial.available()) {
                Serial.read();
            }
            
            // Wait for SSID input
            while(!Serial.available()) {
                delay(100);
            }
            delay(100); // Give time for full input
            ssid = Serial.readStringUntil('\n');
            ssid.trim();
            
            Serial.println("SSID received: " + ssid);
            Serial.println("Now enter password:");
            
            // Clear buffer again
            while(Serial.available()) {
                Serial.read();
            }
            
            // Wait for password input
            while(!Serial.available()) {
                delay(100);
            }
            delay(100); // Give time for full input
            password = Serial.readStringUntil('\n');
            password.trim();
            
            Serial.println("Password received.");
            Serial.println("WiFi credentials updated!");
            delay(1000);
            handleConfig();
            break;
                    
            case '2':
                Serial.println("\nServer Setup:");
                Serial.println("Enter server IP (e.g., 192.168.1.100):");
                
                // Clear any existing serial data
                while(Serial.available()) {
                    Serial.read();
                }
                
                // Wait for IP input
                while(!Serial.available()) {
                    delay(100);
                }
                delay(100); // Give time for full input
                serverIP = Serial.readStringUntil('\n');
                serverIP.trim();
                
                Serial.println("Server IP received: " + serverIP);
                updateServerUrl();
                
                Serial.println("Server IP updated!");
                delay(1000);
                handleConfig();
                break;
                    
                case '3':
                    saveSettings();
                    Serial.println("\nSettings saved!");
                    if (connectWiFi()) {
                        Serial.println("Successfully connected to WiFi!");
                        Serial.print("IP address: ");
                        Serial.println(WiFi.localIP());
                        isConfigMode = false;
                        return;
                    } else {
                        Serial.println("Failed to connect to WiFi. Please check credentials.");
                        handleConfig();
                    }
                    break;
                    
                default:
                    Serial.println("Invalid choice. Please try again.");
                    handleConfig();
                    break;
            }
        }
        delay(100);
    }
}

void setup() {
    Serial.begin(115200);
    
    // Initialize pins
    for(uint16_t i = 0; i < 3; i++) {
        pinMode(LEFT_MOTOR[i], OUTPUT);
        pinMode(RIGHT_MOTOR[i], OUTPUT);
    }
    for(uint16_t i = 0; i < 5; i++) {
        pinMode(PINS_IR[i], INPUT);
    }
    
    setMotors(false);
    
    // Initialize WiFi
    loadSettings();
    
    // Check for configuration request
    Serial.println("Press 'c' within 5 seconds to enter configuration mode...");
    unsigned long startTime = millis();
    while (millis() - startTime < 5000) {
        if (Serial.available() && Serial.read() == 'c') {
            isConfigMode = true;
            break;
        }
        delay(100);
    }
    
    if (isConfigMode) {
        handleConfig();
    } else if (!connectWiFi()) {
        Serial.println("Failed to connect to WiFi. Starting in config mode...");
        isConfigMode = true;
        handleConfig();
    }
    
    // Initialize Bluetooth
    SerialBT.begin("MedRobot");
    Serial.println("The device started, now you can pair it with bluetooth!");
}

void loop() {
    if (isConfigMode) {
        if (Serial.available() && Serial.read() == 'c') {
            handleConfig();
        }
    } else {
        if (SerialBT.available()) {
            String cmd = SerialBT.readStringUntil('\n');
            cmd.trim();
            handleCommand(cmd);
        }
        
        updateMovement();
        if (!movement.isMoving) {
            followLine();
        }
    }
    
    delay(5);
}
