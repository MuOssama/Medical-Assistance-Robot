#include <WiFi.h>
#include <HTTPClient.h>
#include <BluetoothSerial.h>
#include <Preferences.h>

// Config
Preferences preferences;
String ssid;
String password;
const char* serverUrl = "http://192.168.1.9:5000/api/patients/";
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

void loadWiFiCredentials() {
    preferences.begin("wifi-config", false);
    ssid = preferences.getString("ssid", "");
    password = preferences.getString("password", "");
    preferences.end();
}

void saveWiFiCredentials(const String& newSSID, const String& newPassword) {
    preferences.begin("wifi-config", false);
    preferences.putString("ssid", newSSID);
    preferences.putString("password", newPassword);
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

void handleSerialConfig() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        
        if (cmd.startsWith("SSID:")) {
            String newSSID = cmd.substring(5);
            newSSID.trim();
            ssid = newSSID;
            Serial.println("SSID set to: " + ssid);
            Serial.println("Now enter password (PASS:your_password)");
        }
        else if (cmd.startsWith("PASS:")) {
            String newPassword = cmd.substring(5);
            newPassword.trim();
            password = newPassword;
            Serial.println("Password set");
            
            // Save and attempt connection
            saveWiFiCredentials(ssid, password);
            Serial.println("Saved credentials, attempting to connect...");
            if (connectWiFi()) {
                isConfigMode = false;
            } else {
                Serial.println("Please check credentials and try again");
                Serial.println("Enter new SSID (SSID:your_ssid)");
            }
        }
        else if (cmd == "CONFIG") {
            isConfigMode = true;
            Serial.println("\n=== WiFi Configuration Mode ===");
            Serial.println("Enter new WiFi credentials:");
            Serial.println("Format: SSID:your_ssid");
            Serial.println("Format: PASS:your_password");
        }
        else if (cmd == "SHOW") {
            Serial.println("\nCurrent WiFi Settings:");
            Serial.println("SSID: " + ssid);
            Serial.println("Password: " + String(password.length()) + " characters");
            Serial.println("WiFi Status: " + String(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected"));
            if (WiFi.status() == WL_CONNECTED) {
                Serial.println("IP: " + WiFi.localIP().toString());
            }
        }
        else if (cmd == "CLEAR") {
            saveWiFiCredentials("", "");
            ssid = "";
            password = "";
            Serial.println("WiFi credentials cleared");
            isConfigMode = true;
        }
    }
}

// [Previous functions remain the same: setMotors, updateServer, checkLine, getLinePos, step]

void setup() {
    Serial.begin(115200);
    delay(1000);  // Give time for Serial Monitor to connect
    
    // Load saved WiFi credentials and try to connect
    loadWiFiCredentials();
    if (!connectWiFi()) {
        isConfigMode = true;
        Serial.println("\nWiFi not configured or connection failed");
        Serial.println("Available commands:");
        Serial.println("CONFIG - Enter configuration mode");
        Serial.println("SHOW - Show current WiFi settings");
        Serial.println("CLEAR - Clear saved credentials");
    }
    
    for(uint8_t i = 0; i < 6; i++) {
        pinMode(PINS_MOTOR[i], OUTPUT);
    }
    for(uint8_t i = 0; i < 10; i++) {
        pinMode(PINS_IR[i], INPUT);
    }
    
    SerialBT.begin(F("MedRobot"));
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
