#include <WiFi.h>
#include <HTTPClient.h>
#include <BluetoothSerial.h>
#include <Preferences.h>

// Config storage
Preferences preferences;
String ssid;
String password;
String serverIP;
String serverUrl;
bool isConfigMode = false;

// Pin Definitions
const uint8_t PINS_MOTOR[] = {25, 26, 27, 32, 33, 14}; // STEP, DIR, EN for L & R
const uint8_t PINS_IR[] = {36, 39, 34, 35, 15, 4, 16, 17, 18, 19};

// Motion Constants
const uint16_t STEPS_PER_REV = 200;
const uint8_t MICROSTEPS = 1;
uint8_t currentRPM = 5;

// State Machine
uint8_t state = 0;     // 0:IDLE, 1:FINDING, 2:FOLLOWING, 3:AT_PATIENT, 4:WAITING
uint8_t command = 0;   // 0:STOP, 1:FORWARD, 2:LINE_FOLLOW
uint8_t patient = 0;
const uint8_t TARGET_PATIENT = 1;

BluetoothSerial SerialBT;

// Configuration Functions
void updateServerUrl() {
    serverUrl = "http://" + serverIP + ":5000/api/patients/";
}

void loadSettings() {
    preferences.begin("settings", false);
    ssid = preferences.getString("ssid", "");
    password = preferences.getString("password", "");
    serverIP = preferences.getString("ip", "192.168.1.100");
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
    if (ssid.length() == 0) return false;
    
    Serial.print("Connecting to WiFi");
    WiFi.begin(ssid.c_str(), password.c_str());
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    Serial.println();
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("Connected to WiFi!");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        return true;
    }
    Serial.println("Connection failed!");
    return false;
}

// Serial Configuration Interface
void printHelp() {
    Serial.println("\n=== Available Commands ===");
    Serial.println("CONFIG     - Enter configuration mode");
    Serial.println("SHOW      - Show current settings");
    Serial.println("CLEAR     - Clear all settings");
    Serial.println("SSID:xxx  - Set WiFi name");
    Serial.println("PASS:xxx  - Set WiFi password");
    Serial.println("IP:xxx    - Set server IP (e.g., IP:192.168.1.100)");
    Serial.println("SAVE      - Save all settings");
    Serial.println("HELP      - Show this help");
    Serial.println("=====================");
}

void handleSerialConfig() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        
        if (cmd.startsWith("SSID:")) {
            ssid = cmd.substring(5);
            ssid.trim();
            Serial.println("SSID set to: " + ssid);
            Serial.println("Type SAVE to store permanently");
        }
        else if (cmd.startsWith("PASS:")) {
            password = cmd.substring(5);
            password.trim();
            Serial.println("Password set");
            Serial.println("Type SAVE to store permanently");
        }
        else if (cmd.startsWith("IP:")) {
            serverIP = cmd.substring(3);
            serverIP.trim();
            updateServerUrl();
            Serial.println("Server IP set to: " + serverIP);
            Serial.println("Full server URL: " + serverUrl);
            Serial.println("Type SAVE to store permanently");
        }
        else if (cmd == "SAVE") {
            saveSettings();
            Serial.println("Settings saved!");
            if (connectWiFi()) {
                isConfigMode = false;
            }
        }
        else if (cmd == "CONFIG") {
            isConfigMode = true;
            Serial.println("\n=== Configuration Mode ===");
            printHelp();
        }
        else if (cmd == "SHOW") {
            Serial.println("\nCurrent Settings:");
            Serial.println("SSID: " + ssid);
            Serial.println("Password: " + String(password.length()) + " characters");
            Serial.println("Server IP: " + serverIP);
            Serial.println("Full Server URL: " + serverUrl);
            Serial.println("WiFi Status: " + String(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected"));
            if (WiFi.status() == WL_CONNECTED) {
                Serial.println("IP: " + WiFi.localIP().toString());
            }
        }
        else if (cmd == "CLEAR") {
            preferences.begin("settings", false);
            preferences.clear();
            preferences.end();
            ssid = "";
            password = "";
            serverIP = "192.168.1.100";
            updateServerUrl();
            Serial.println("All settings cleared");
            isConfigMode = true;
        }
        else if (cmd == "HELP") {
            printHelp();
        }
    }
}

// Robot Control Functions
void setMotors(bool enable) {
    digitalWrite(PINS_MOTOR[2], !enable);
    digitalWrite(PINS_MOTOR[5], !enable);
}

bool updateServer(uint8_t id, bool isDispensing) {
    if(WiFi.status() != WL_CONNECTED) return false;
    
    HTTPClient http;
    String fullUrl = serverUrl + String(id);
    http.begin(fullUrl);
    http.addHeader("Content-Type", "application/json");
    
    char payload[32];
    snprintf(payload, sizeof(payload), "{\"is_dispensing\":%s}", isDispensing ? "true" : "false");
    
    int code = http.POST(payload);
    http.end();
    return (code == 200);
}

bool checkLine() {
    uint8_t count = 0;
    for(uint8_t i = 0; i < 10; i++) {
        if(digitalRead(PINS_IR[i])) count++;
    }
    return count >= 8;
}

int8_t getLinePos() {
    uint8_t left = 0, right = 0;
    
    for(uint8_t i = 0; i < 5; i++) {
        if(digitalRead(PINS_IR[i])) left++;
        if(digitalRead(PINS_IR[i + 5])) right++;
    }
    
    return (right > left) ? 1 : ((left > right) ? -1 : 0);
}

void step(int8_t left, int8_t right) {
    static uint32_t stepDelay = (60L * 1000000L) / (STEPS_PER_REV * MICROSTEPS * currentRPM) / 2;
    
    digitalWrite(PINS_MOTOR[1], left > 0);
    digitalWrite(PINS_MOTOR[4], right > 0);
    
    if(abs(left) || abs(right)) {
        digitalWrite(PINS_MOTOR[0], HIGH);
        digitalWrite(PINS_MOTOR[3], HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(PINS_MOTOR[0], LOW);
        digitalWrite(PINS_MOTOR[3], LOW);
        delayMicroseconds(stepDelay);
    }
}

void handleCmd() {
    if(!SerialBT.available()) return;
    
    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();
    Serial.println(cmd);
    
    if(cmd == F("AUX1") && !state) {
        state = 1;
        command = 2;
        setMotors(true);
        patient = 0;
        Serial.println("AUX1");
    }
    else if(cmd == F("STOP") || cmd == F("Emergency")) {
        command = state = 0;
        setMotors(false);
        Serial.println("Stop");
    }
    else if(cmd.startsWith(F("RPM:"))) {
        uint8_t rpm = cmd.substring(4).toInt();
        if(rpm > 0 && rpm <= 10) currentRPM = rpm;
    }
    else if(!state) {
        setMotors(true);
        if(cmd == F("U")){ step(5, 5); Serial.println("Forward");}
        else if(cmd == F("r")) {step(-5, -5);Serial.println("Backward");}
        else if(cmd == F("L")) {step(-3, 3);Serial.println("Left");}
        else if(cmd == F("R")) {step(3, -3);Serial.println("Right");}
        else if(cmd == F("B")) {step(0, 0);Serial.println("Stop");}
    }
}

void followLine() {
    if(command != 2) return;
    
    switch(state) {
        case 1: // FINDING
            if(checkLine()) {
                state = 2;
            } else {
                step(3, -3);
            }
            break;

        case 2: // FOLLOWING
            if(checkLine()) {
                state = 3;
                patient++;
                step(0, 0);
                updateServer(patient, true);
            } else {
                int8_t pos = getLinePos();
                if(!pos) step(5, 5);
                else if(pos < 0) step(3, 5);
                else step(5, 3);
            }
            break;

        case 3: // AT_PATIENT
            if(patient < TARGET_PATIENT) {
                delay(5000);
                updateServer(patient, false);
                state = 2;
            } else {
                state = 4;
                setMotors(false);
            }
            break;
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    loadSettings();
    if (!connectWiFi()) {
        isConfigMode = true;
        Serial.println("\nWiFi not configured or connection failed");
        printHelp();
    }
    
    for(uint8_t i = 0; i < 6; i++) {
        pinMode(PINS_MOTOR[i], OUTPUT);
    }
    for(uint8_t i = 0; i < 10; i++) {
        pinMode(PINS_IR[i], INPUT);
    }
    
    SerialBT.begin("MedRobot");
    setMotors(false);
}

void loop() {
    handleSerialConfig();
    
    if (!isConfigMode) {
        handleCmd();
        followLine();
    }
    delay(5);
}
