#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>

// Display configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Pin definitions
#define THERMISTOR_PIN A0
#define BUZZER_PIN D8

// Network configuration
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
const char* serverUrl = "http://your_server:5000/api/patients/1";

// Timing constants
#define REPORTING_PERIOD_MS 3000
#define SERVER_UPDATE_PERIOD_MS 10000

// Health thresholds (as defined in server.py)
const float TEMP_MIN = 36.5;
const float TEMP_MAX = 37.5;
const float SPO2_MIN = 95.0;
const float HR_MIN = 60.0;
const float HR_MAX = 100.0;
const float GLUCOSE_MIN = 70.0;
const float GLUCOSE_MAX = 140.0;

// Global variables
PulseOximeter pox;
uint32_t tsLastReport = 0;
uint32_t tsLastServerUpdate = 0;
bool isDangerState = false;
float lastValidHeartRate = 0;
float lastValidSpO2 = 0;
float lastValidTemp = 0;
float lastValidGlucose = 0;

void setup() {
  Serial.begin(9600);
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Initialize WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.display();

  // Initialize MAX30100
  if (!pox.begin()) {
    Serial.println("Failed to initialize pulse oximeter");
    while (1);
  }
  pox.setIRLedCurrent(MAX30100_LED_CURR_33_8MA);

  // Show initial message
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Smart Medical Watch");
  display.println("Initialized!");
  display.display();
  delay(2000);
}

float calculateTemperature() {
  int rawValue = analogRead(THERMISTOR_PIN);
  float resistance = 10000.0 * (1023.0 / rawValue - 1.0);
  float steinhart = log(resistance / 10000.0) / 3950.0 + 1.0 / 298.15;
  return 1.0 / steinhart - 273.15;
}

float predictGlucose(float heartRate, float spO2) {
  if (heartRate > 0 && spO2 > 0) {
    return 70.0 + (heartRate - 60) * 0.4 + (spO2 - 95) * 2;
  }
  return 0;
}

bool checkDangerCondition(float heartRate, float spO2, float temperature, float glucose) {
  if (heartRate == 0 || spO2 == 0 || temperature == 0 || glucose == 0) {
    return false;  // Invalid readings, don't trigger danger
  }
  
  return (heartRate < HR_MIN || heartRate > HR_MAX ||
          spO2 < SPO2_MIN ||
          temperature < TEMP_MIN || temperature > TEMP_MAX ||
          glucose < GLUCOSE_MIN || glucose > GLUCOSE_MAX);
}

void updateServer(float heartRate, float spO2, float temperature, float glucose) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected");
    return;
  }

  HTTPClient http;
  http.begin(serverUrl);
  http.addHeader("Content-Type", "application/json");

  StaticJsonDocument<300> doc;
  
  // Add all required fields for the server
  doc["heart_rate"] = heartRate;
  doc["spo2"] = spO2;
  doc["temperature"] = temperature;
  doc["glucose_level"] = glucose;
  doc["medical_schedule"] = 1;
  doc["patient_order"] = 1;
  
  // Create medicals array (5 elements as required by server)
  JsonArray medicals = doc.createNestedArray("medicals");
  for(int i = 0; i < 5; i++) {
    medicals.add(0);
  }

  String jsonString;
  serializeJson(doc, jsonString);

  int httpResponseCode = http.POST(jsonString);
  
  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.printf("Server updated successfully, code: %d\n", httpResponseCode);
  } else {
    Serial.printf("Error updating server: %d\n", httpResponseCode);
  }
  
  http.end();
}

void manageBuzzer(bool isDanger) {
  if (isDanger) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }
}

void updateDisplay(float heartRate, float spO2, float temperature, float glucose, bool isDanger) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  
  if (heartRate > 0 && spO2 > 0) {
    display.println("Patient Vitals:");
    display.println("--------------");
    display.printf("HR  : %.1f bpm\n", heartRate);
    display.printf("SpO2: %.1f%%\n", spO2);
    display.printf("Temp: %.1fC\n", temperature);
    display.printf("Gluc: %.1f mg/dL\n", glucose);
    
    if (isDanger) {
      display.setTextSize(1);
      display.println("\n** DANGER **");
    } else {
      display.println("\nStatus: Normal");
    }
  } else {
    display.setTextSize(2);
    display.println("Place");
    display.println("finger");
    display.println("on");
    display.println("sensor");
  }
  
  display.display();
}

void loop() {
  pox.update();

  if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
    float heartRate = pox.getHeartRate();
    float spO2 = pox.getSpO2();
    float temperature = calculateTemperature();
    
    // Validate and store readings
    if (heartRate > 20 && heartRate < 200 && spO2 > 50 && spO2 < 100) {
      lastValidHeartRate = heartRate;
      lastValidSpO2 = spO2;
      lastValidTemp = temperature;
      lastValidGlucose = predictGlucose(heartRate, spO2);
      
      // Check danger condition
      bool isDanger = checkDangerCondition(lastValidHeartRate, lastValidSpO2, 
                                         lastValidTemp, lastValidGlucose);
      
      // Update display
      updateDisplay(lastValidHeartRate, lastValidSpO2, lastValidTemp, 
                   lastValidGlucose, isDanger);
      
      // Manage buzzer
      manageBuzzer(isDanger);
      
      // Update server periodically
      if (millis() - tsLastServerUpdate > SERVER_UPDATE_PERIOD_MS) {
        updateServer(lastValidHeartRate, lastValidSpO2, lastValidTemp, 
                    lastValidGlucose);
        tsLastServerUpdate = millis();
      }
      
      // Debug output
      Serial.printf("HR: %.1f, SpO2: %.1f, Temp: %.1f, Glucose: %.1f\n",
                   lastValidHeartRate, lastValidSpO2, lastValidTemp, lastValidGlucose);
    } else {
      updateDisplay(0, 0, 0, 0, false);  // Show "Place finger" message
    }
    
    tsLastReport = millis();
  }
}
