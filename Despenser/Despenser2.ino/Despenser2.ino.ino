#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <Servo.h>
#include <EEPROM.h>

// Structure to store configuration
struct Config {
  char ssid[32];
  char password[32];
  char serverIP[16];
  int serverPort;
  bool isConfigured;
};

// Pin definitions
const int WATER_PUMP_PIN = D1;
const int SYRUP1_PUMP_PIN = D2;
const int SYRUP2_PUMP_PIN = D3;
const int TABLET_SERVO_PIN = D4;

// Timing constants
const int PUMP_DURATION = 3000;       // 3 seconds
const int SERVO_DURATION = 1000;      // 1 second
const int REQUEST_TIMEOUT = 5000;     // 5 seconds timeout for HTTP requests
const int RETRY_DELAY = 5000;         // 5 seconds between retries
const int MAX_RETRIES = 3;            // Maximum number of retry attempts
const int WIFI_RECONNECT_DELAY = 10000; // 10 seconds between WiFi reconnection attempts
const int patientId = 1;

// Global variables
Config config;
Servo tabletServo;
WiFiClient wifiClient;
HTTPClient http;

// Function declarations
void loadConfiguration();
void saveConfiguration();
bool getSerialConfiguration();
bool connectWiFi();
void dispenseMedication(JsonArray medicals);
void operatePump(int pin, int duration);
void operateServo(int angle);
void updateDispensingState(bool isDispensing);
bool checkDispensingState();
bool makeHttpRequest(String url, String& response, String method = "GET", String payload = "");

void setup() {
  Serial.begin(115200);
  EEPROM.begin(512);
  
  Serial.println("\n=== Automated Medication Dispenser Starting ===");
  
  // Initialize pins
  pinMode(WATER_PUMP_PIN, OUTPUT);
  pinMode(SYRUP1_PUMP_PIN, OUTPUT);
  pinMode(SYRUP2_PUMP_PIN, OUTPUT);
  digitalWrite(WATER_PUMP_PIN, LOW);
  digitalWrite(SYRUP1_PUMP_PIN, LOW);
  digitalWrite(SYRUP2_PUMP_PIN, LOW);
  
  // Initialize servo
  tabletServo.attach(TABLET_SERVO_PIN);
  tabletServo.write(0);
  
  // Load or get configuration
  loadConfiguration();
  if (!config.isConfigured) {
    while (!getSerialConfiguration()) {
      Serial.println("Configuration failed. Please try again.");
      delay(1000);
    }
    saveConfiguration();
  }
  
  // Initial WiFi connection
  while (!connectWiFi()) {
    Serial.println("Initial WiFi connection failed. Retrying...");
    delay(WIFI_RECONNECT_DELAY);
  }
}

void loadConfiguration() {
  EEPROM.get(0, config);
  if (config.isConfigured) {
    Serial.println("Loaded configuration:");
    Serial.print("SSID: ");
    Serial.println(config.ssid);
    Serial.print("Server IP: ");
    Serial.println(config.serverIP);
    Serial.print("Server Port: ");
    Serial.println(config.serverPort);
  } else {
    Serial.println("No configuration found. Please configure through serial.");
  }
}

void saveConfiguration() {
  EEPROM.put(0, config);
  EEPROM.commit();
  Serial.println("Configuration saved to EEPROM");
}

bool getSerialConfiguration() {
  Serial.println("\n=== Device Configuration ===");
  Serial.println("Please enter the following information:");
  
  Serial.print("WiFi SSID: ");
  while (!Serial.available()) delay(100);
  String ssid = Serial.readStringUntil('\n');
  ssid.trim();
  
  Serial.print("WiFi Password: ");
  while (!Serial.available()) delay(100);
  String password = Serial.readStringUntil('\n');
  password.trim();
  
  Serial.print("Server IP (e.g., 192.168.1.100): ");
  while (!Serial.available()) delay(100);
  String serverIP = Serial.readStringUntil('\n');
  serverIP.trim();
  
  Serial.print("Server Port (e.g., 5000): ");
  while (!Serial.available()) delay(100);
  String portStr = Serial.readStringUntil('\n');
  portStr.trim();
  int port = portStr.toInt();
  
  if (ssid.length() == 0 || password.length() == 0 || serverIP.length() == 0 || port == 0) {
    Serial.println("Invalid input. Please try again.");
    return false;
  }
  
  // Store configuration
  ssid.toCharArray(config.ssid, sizeof(config.ssid));
  password.toCharArray(config.password, sizeof(config.password));
  serverIP.toCharArray(config.serverIP, sizeof(config.serverIP));
  config.serverPort = port;
  config.isConfigured = true;
  
  Serial.println("\nConfiguration complete!");
  return true;
}

void loop() {
  static unsigned long lastWiFiCheck = 0;
  const unsigned long WiFiCheckInterval = 30000; // Check WiFi every 30 seconds
  
  // Check for serial command to reconfigure
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command == "config") {
      if (getSerialConfiguration()) {
        saveConfiguration();
        ESP.restart();
      }
    }
  }
  
  // Regular WiFi connection check
  if (millis() - lastWiFiCheck >= WiFiCheckInterval) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi connection lost - Attempting to reconnect...");
      if (!connectWiFi()) {
        delay(WIFI_RECONNECT_DELAY);
        return;
      }
    }
    lastWiFiCheck = millis();
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n--- Checking dispensing state ---");
    bool isDispensing = checkDispensingState();
    
    if (isDispensing) {
      Serial.println("Dispensing state is TRUE - Starting medication dispensing process");
      String url = "http://" + String(config.serverIP) + ":" + String(config.serverPort) + "/api/patients/" + String(patientId);
      String response;
      
      if (makeHttpRequest(url, response)) {
        DynamicJsonDocument doc(1024);
        DeserializationError error = deserializeJson(doc, response);
        
        if (!error) {
          JsonArray medicals = doc["medicals"];
          dispenseMedication(medicals);
          updateDispensingState(false);
        } else {
          Serial.print("JSON parsing failed: ");
          Serial.println(error.c_str());
        }
      }
    } else {
      Serial.println("Dispensing state is FALSE - No action needed");
    }
  }
  
  delay(1000);
}

bool connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }

  Serial.print("Connecting to WiFi network: ");
  Serial.println(config.ssid);
  
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(config.ssid, config.password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected successfully");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    return true;
  } else {
    Serial.println("\nWiFi connection failed!");
    return false;
  }
}

// The rest of the functions remain the same as in the original code
void dispenseMedication(JsonArray medicals) {
  Serial.println("\n=== Starting Medication Dispensing Sequence ===");
  
  for (int i = 0; i < medicals.size(); i++) {
    if (medicals[i] == 1) {
      Serial.print("\n");
      Serial.print(i + 1);
      Serial.println(". Dispensing medication:");
      
      switch (i) {
        case 0: // Water
          operatePump(WATER_PUMP_PIN, PUMP_DURATION);
          break;
        case 1: // Syrup 1
          operatePump(SYRUP1_PUMP_PIN, PUMP_DURATION);
          break;
        case 2: // Syrup 2
          operatePump(SYRUP2_PUMP_PIN, PUMP_DURATION);
          break;
        case 3: // Tablet 1
          operateServo(90);
          delay(SERVO_DURATION);
          operateServo(0);
          break;
        case 4: // Tablet 2
          operateServo(180);
          delay(SERVO_DURATION);
          operateServo(0);
          break;
      }
      delay(500);
    }
  }
  
  Serial.println("\n=== Medication Dispensing Sequence Completed ===");
}

void operatePump(int pin, int duration) {
  Serial.println("   - Pump activated");
  digitalWrite(pin, HIGH);
  delay(duration);
  digitalWrite(pin, LOW);
  Serial.println("   - Pump deactivated");
}

void operateServo(int angle) {
  Serial.print("   - Moving servo to ");
  Serial.print(angle);
  Serial.println("°");
  tabletServo.write(angle);
  delay(500);
  Serial.println("   - Servo movement completed");
}

bool makeHttpRequest(String url, String& response, String method, String payload) {
  int retryCount = 0;
  bool success = false;
  
  while (!success && retryCount < MAX_RETRIES) {
    http.setTimeout(REQUEST_TIMEOUT);
    
    Serial.print("Making HTTP ");
    Serial.print(method);
    Serial.print(" request to: ");
    Serial.println(url);
    Serial.print("Attempt: ");
    Serial.println(retryCount + 1);
    
    if (!http.begin(wifiClient, url)) {
      Serial.println("HTTP setup failed");
      retryCount++;
      delay(RETRY_DELAY);
      continue;
    }
    
    int httpCode;
    if (method == "POST") {
      http.addHeader("Content-Type", "application/json");
      httpCode = http.POST(payload);
    } else {
      httpCode = http.GET();
    }
    
    if (httpCode == HTTP_CODE_OK) {
      response = http.getString();
      success = true;
      Serial.println("HTTP request successful");
    } else {
      Serial.print("HTTP request failed, error code: ");
      Serial.println(httpCode);
      Serial.print("Error message: ");
      Serial.println(http.errorToString(httpCode).c_str());
      
      retryCount++;
      if (retryCount < MAX_RETRIES) {
        Serial.println("Retrying...");
        delay(RETRY_DELAY);
      }
    }
    
    http.end();
  }
  
  if (!success) {
    Serial.println("All retry attempts failed");
  }
  
  return success;
}

bool checkDispensingState() {
  String url = "http://" + String(config.serverIP) + ":" + String(config.serverPort) + "/api/patients/" + String(patientId);
  String response;
  
  if (makeHttpRequest(url, response)) {
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, response);
    
    if (!error && doc.containsKey("is_dispensing")) {
      bool isDispensing = doc["is_dispensing"].as<bool>();
      Serial.print("Current dispensing state: ");
      Serial.println(isDispensing ? "TRUE" : "FALSE");
      return isDispensing;
    }
  }
  
  return false;
}

void updateDispensingState(bool isDispensing) {
  Serial.println("Updating dispensing state on server...");
  String url = "http://" + String(config.serverIP) + ":" + String(config.serverPort) + "/api/patients/" + String(patientId);
  
  StaticJsonDocument<200> doc;
  doc["is_dispensing"] = isDispensing;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  String response;
  if (makeHttpRequest(url, response, "POST", jsonString)) {
    Serial.println("Dispensing state updated successfully");
  } else {
    Serial.println("Failed to update dispensing state");
  }
}
