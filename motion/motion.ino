#include <WiFi.h>
#include <HTTPClient.h>
#include <BluetoothSerial.h>
#include <Preferences.h>

// Config
Preferences preferences;
String ssid;
String password;
String serverIP;
String serverUrl;
bool isConfigMode = false;

// Pins (using direct port manipulation for speed and size)
const uint8_t PINS_MOTOR[] = {25, 26, 27, 32, 33, 14}; // STEP, DIR, EN for L & R
const uint8_t PINS_IR[] = {36, 39, 34, 35, 15, 4, 16, 17, 18, 19};

// Constants
const uint16_t STEPS_PER_REV = 200;
const uint8_t MICROSTEPS = 1;
uint8_t currentRPM = 5; // Store as integer, divide when needed

// State machine (use smaller data types)
uint8_t state = 0;     // 0:IDLE, 1:FINDING, 2:FOLLOWING, 3:AT_PATIENT, 4:WAITING
uint8_t command = 0;   // 0:STOP, 1:FORWARD, 2:LINE_FOLLOW
uint8_t patient = 0;
const uint8_t TARGET_PATIENT = 1;

BluetoothSerial SerialBT;

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
            String newSSID = cmd.substring(5);
            newSSID.trim();
            ssid = newSSID;
            Serial.println("SSID set to: " + ssid);
            Serial.println("Type SAVE to store permanently");
        }
        else if (cmd.startsWith("PASS:")) {
            String newPassword = cmd.substring(5);
            newPassword.trim();
            password = newPassword;
            Serial.println("Password set");
            Serial.println("Type SAVE to store permanently");
        }
        else if (cmd.startsWith("IP:")) {
            String newIP = cmd.substring(3);
            newIP.trim();
            serverIP = newIP;
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

// [Rest of the original functions remain the same: setMotors, checkLine, getLinePos, step, handleCmd, followLine]

void setup() {
    Serial.begin(115200);
    delay(1000);  // Give time for Serial Monitor to connect
    
    // Load saved settings and try to connect
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
    handleSerialConfig();  // Always check for serial commands
    
    if (!isConfigMode) {
        handleCmd();  // Original Bluetooth commands
        followLine();
    }
    delay(5);
}
