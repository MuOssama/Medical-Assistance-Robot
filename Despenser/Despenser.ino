#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <Servo.h>

// WiFi credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// Server details
const char* serverHost = "YOUR_SERVER_IP";
const int serverPort = 5000;
const int patientId = 1;  // Change this to match the patient ID you want to monitor

// Pin definitions
const int WATER_PUMP_PIN = D1;        // Water pump
const int SYRUP1_PUMP_PIN = D2;       // Syrup medical 1 pump
const int SYRUP2_PUMP_PIN = D3;       // Syrup medical 2 pump
const int TABLET_SERVO_PIN = D4;      // Servo for tablets

// Timing constants
const int PUMP_DURATION = 3000;       // Duration for pump operation (3 seconds)
const int SERVO_DURATION = 1000;      // Duration for servo movement (1 second)

// Create objects
Servo tabletServo;
WiFiClient wifiClient;
HTTPClient http;

// Function declarations
void dispenseMedication(JsonArray medicals);
void operatePump(int pin, int duration);
void operateServo(int angle);
void updateDispensingState(bool isDispensing);
bool checkDispensingState();

void setup() {
  Serial.begin(115200);
  
  // Initialize pins
  pinMode(WATER_PUMP_PIN, OUTPUT);
  pinMode(SYRUP1_PUMP_PIN, OUTPUT);
  pinMode(SYRUP2_PUMP_PIN, OUTPUT);
  
  // Initialize servo
  tabletServo.attach(TABLET_SERVO_PIN);
  tabletServo.write(0);  // Initial position
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    bool isDispensing = checkDispensingState();
    
    if (isDispensing) {
      // Get medications array
      String url = String("http://") + serverHost + ":" + serverPort + "/api/patients/" + patientId;
      
      http.begin(wifiClient, url);
      int httpCode = http.GET();
      
      if (httpCode == HTTP_CODE_OK) {
        String payload = http.getString();
        
        // Parse JSON response
        DynamicJsonDocument doc(1024);
        DeserializationError error = deserializeJson(doc, payload);
        
        if (!error) {
          // Get medications array
          JsonArray medicals = doc["medicals"];
          
          // Dispense medications
          dispenseMedication(medicals);
          
          // Update dispensing state to false (ready)
          updateDispensingState(false);
        } else {
          Serial.print("JSON parsing failed: ");
          Serial.println(error.c_str());
        }
      } else {
        Serial.print("HTTP GET failed, error: ");
        Serial.println(http.errorToString(httpCode).c_str());
      }
      
      http.end();
    }
  }
  
  delay(1000);  // Check every second
}

void dispenseMedication(JsonArray medicals) {
  // Water pump
  if (medicals[0] == 1) {
    Serial.println("Dispensing water");
    operatePump(WATER_PUMP_PIN, PUMP_DURATION);
  }
  
  // Syrup medical 1
  if (medicals[1] == 1) {
    Serial.println("Dispensing syrup 1");
    operatePump(SYRUP1_PUMP_PIN, PUMP_DURATION);
  }
  
  // Syrup medical 2
  if (medicals[2] == 1) {
    Serial.println("Dispensing syrup 2");
    operatePump(SYRUP2_PUMP_PIN, PUMP_DURATION);
  }
  
  // Tablet medical 1
  if (medicals[3] == 1) {
    Serial.println("Dispensing tablet 1");
    operateServo(90);  // Move to 90 degrees to dispense tablet 1
    delay(SERVO_DURATION);
    operateServo(0);   // Return to initial position
  }
  
  // Tablet medical 2
  if (medicals[4] == 1) {
    Serial.println("Dispensing tablet 2");
    operateServo(0);   // Move to 0 degrees to dispense tablet 2
    delay(SERVO_DURATION);
    operateServo(0);   // Return to initial position
  }
}

void operatePump(int pin, int duration) {
  digitalWrite(pin, HIGH);
  delay(duration);
  digitalWrite(pin, LOW);
  delay(500);  // Small delay between operations
}

void operateServo(int angle) {
  tabletServo.write(angle);
  delay(500);  // Give servo time to reach position
}

bool checkDispensingState() {
  String url = String("http://") + serverHost + ":" + serverPort + "/api/patients/" + patientId;
  
  http.begin(wifiClient, url);
  int httpCode = http.GET();
  
  bool isDispensing = false;
  
  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, payload);
    
    if (!error) {
      isDispensing = doc["is_dispensing"];
    } else {
      Serial.print("JSON parsing failed: ");
      Serial.println(error.c_str());
    }
  } else {
    Serial.print("HTTP GET failed, error: ");
    Serial.println(http.errorToString(httpCode).c_str());
  }
  
  http.end();
  return isDispensing;
}

void updateDispensingState(bool isDispensing) {
  String url = String("http://") + serverHost + ":" + serverPort + "/api/patients/" + patientId;
  
  // Create JSON payload
  DynamicJsonDocument doc(1024);
  doc["is_dispensing"] = isDispensing;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  http.begin(wifiClient, url);
  http.addHeader("Content-Type", "application/json");
  int httpCode = http.POST(jsonString);
  
  if (httpCode == HTTP_CODE_OK) {
    Serial.println("Dispensing state updated successfully");
  } else {
    Serial.print("Failed to update dispensing state, error: ");
    Serial.println(http.errorToString(httpCode).c_str());
  }
  
  http.end();
}
