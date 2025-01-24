#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <Servo.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

// Config storage
struct DeviceConfig {
  char ssid[32];
  char password[32];
  char serverIP[16];
} config;

// Server details
const int serverPort = 5000;
const int patientId = 1;

// Pin definitions
const int WATER_PUMP_PIN = D1;
const int SYRUP1_PUMP_PIN = D2;
const int SYRUP2_PUMP_PIN = D3;
const int TABLET_SERVO_PIN = D5;

// Timing constants
const int PUMP_DURATION = 3000;       // 3 seconds
const int SERVO_DURATION = 1000;      // 1 second
const int REQUEST_TIMEOUT = 5000;     // 5 seconds timeout for HTTP requests
const int RETRY_DELAY = 5000;         // 5 seconds between retries
const int MAX_RETRIES = 3;            // Maximum number of retry attempts
const int WIFI_RECONNECT_DELAY = 10000; // 10 seconds between WiFi reconnection attempts

const int SIM800_RX_PIN = D6;  // Adjust based on your wiring
const int SIM800_TX_PIN = D7;  // Adjust based on your wiring
const char* EMERGENCY_PHONE = "+201063384995";

SoftwareSerial sim800lSerial(SIM800_RX_PIN, SIM800_TX_PIN);

void setupSim800l() {
  sim800lSerial.begin(9600);
  delay(1000);
  
  // Initialize Sim800l module
  sim800lSerial.println("AT");
  delay(1000);
  sim800lSerial.println("AT+CMGF=1"); // Set SMS mode to text
  delay(1000);
}

void sendEmergencySMS(String message) {
  Serial.println("Sending Emergency SMS...");
  
  sim800lSerial.print("AT+CMGS=\"");
  sim800lSerial.print(EMERGENCY_PHONE);
  sim800lSerial.println("\"");
  delay(100);
  
  sim800lSerial.println(message);
  delay(100);
  sim800lSerial.write(26);  // Ctrl+Z to send
  
  delay(1000);
  Serial.println("SMS Sent");
}

void makeEmergencyCall() {
  Serial.println("Making Emergency Call...");
  
  sim800lSerial.print("ATD");
  sim800lSerial.print(EMERGENCY_PHONE);
  sim800lSerial.println(";");
  
  delay(1000);
  Serial.println("Call Initiated");
}


void sendHealthAlert(bool isInDanger) {
  static bool flag = false;
  if (isInDanger && !flag) {
    String alertMessage = "EMERGENCY: Patient in danger! Immediate medical attention required.";
    sendEmergencySMS(alertMessage);
    makeEmergencyCall();
    flag = true;
    
  }
}

// Create objects
Servo tabletServo;
WiFiClient wifiClient;
HTTPClient http;
bool isConfigMode = false;

void loadConfig() {
  EEPROM.get(0, config);
}

void saveConfig() {
  EEPROM.put(0, config);
  EEPROM.commit();
}

void handleSerialConfig() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    switch (input[0]) {
      case '1':
        Serial.println("Enter WiFi SSID:");
        while (!Serial.available()) delay(100);
        input = Serial.readStringUntil('\n');
        input.trim();
        input.toCharArray(config.ssid, sizeof(config.ssid));
        Serial.println("SSID set to: " + String(config.ssid));
        break;
        
      case '2':
        Serial.println("Enter WiFi Password:");
        while (!Serial.available()) delay(100);
        input = Serial.readStringUntil('\n');
        input.trim();
        input.toCharArray(config.password, sizeof(config.password));
        Serial.println("Password set");
        break;
        
      case '3':
        Serial.println("Enter Server IP:");
        while (!Serial.available()) delay(100);
        input = Serial.readStringUntil('\n');
        input.trim();
        input.toCharArray(config.serverIP, sizeof(config.serverIP));
        Serial.println("Server IP set to: " + String(config.serverIP));
        break;
        
      case '4':
        Serial.println("\nCurrent Configuration:");
        Serial.println("SSID: " + String(config.ssid));
        Serial.println("Password: [hidden]");
        Serial.println("Server IP: " + String(config.serverIP));
        break;
        
      case '5':
        Serial.println("Saving configuration and restarting...");
        saveConfig();
        delay(1000);
        ESP.restart();
        break;
        
      default:
        Serial.println("\n=== Configuration Mode ===");
        Serial.println("Enter commands:");
        Serial.println("1: Set WiFi SSID");
        Serial.println("2: Set WiFi Password");
        Serial.println("3: Set Server IP");
        Serial.println("4: Show current config");
        Serial.println("5: Save and restart");
        break;
    }
  }
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
      delay(500); // Short delay between medications
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

bool checkDispensingState() {
  String url = "http://" + String(config.serverIP) + ":" + String(serverPort) + "/api/patients/" + String(patientId);
  String response;
  
  // Added missing arguments for method and payload
  if (makeHttpRequest(url, response, "GET", "")) {
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

bool checkPatientHealthState() {
  String url = "http://" + String(config.serverIP) + ":" + String(serverPort) + "/api/patients/" + String(patientId);
  String response;
  
  if (makeHttpRequest(url, response, "GET", "")) {
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, response);
    
    if (!error) {
      String healthState = doc["health_state"].as<String>();
      Serial.print("Patient Health State: ");
      Serial.println(healthState);
      
      return (healthState == "in danger");
    }
  }
  
  return false;
}
void updateDispensingState(bool isDispensing) {
  Serial.println("Updating dispensing state on server...");
  String url = "http://" + String(config.serverIP) + ":" + String(serverPort) + "/api/patients/" + String(patientId);
  
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

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Automated Medication Dispenser Starting ===");
  
  // Initialize EEPROM
EEPROM.begin(sizeof(DeviceConfig));
  
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
  
  // Load configuration
  loadConfig();
  
  // Check if configuration exists
  if (strlen(config.ssid) == 0) {
    Serial.println("No configuration found - entering config mode");
    isConfigMode = true;
  } else {
    // Try to connect to WiFi
    if (!connectWiFi()) {
      Serial.println("WiFi connection failed - entering config mode");
      isConfigMode = true;
    }
  }

  if (isConfigMode) {
    Serial.println("\n=== Configuration Mode ===");
    Serial.println("Enter commands:");
    Serial.println("1: Set WiFi SSID");
    Serial.println("2: Set WiFi Password");
    Serial.println("3: Set Server IP");
    Serial.println("4: Show current config");
    Serial.println("5: Save and restart");
  }
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
  bool isInDanger = checkPatientHealthState();
  if (isInDanger) {
    sendHealthAlert(isInDanger);
  }
}
  if (isConfigMode) {
    handleSerialConfig();
    return;
  }

  static unsigned long lastWiFiCheck = 0;
  const unsigned long WiFiCheckInterval = 30000;
  
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
      String url = "http://" + String(config.serverIP) + ":" + String(serverPort) + "/api/patients/" + String(patientId);
      String response;
      
      // Added missing arguments for method and payload
      if (makeHttpRequest(url, response, "GET", "")) {
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
