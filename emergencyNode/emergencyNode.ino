#include <SoftwareSerial.h>

// Define pins for SIM800L communication
#define SIM800L_RX 3 // SIM800L TX -> ESP8266 RX (SoftwareSerial)
#define SIM800L_TX 1 // SIM800L RX -> ESP8266 TX (SoftwareSerial)
#define DETECTION_PIN 2 // GPIO to detect high signal

// Define the patient ID and phone number
const int patientID = 1;
const char phoneNumber[] = "+1155508075";

SoftwareSerial sim800l(SIM800L_RX, SIM800L_TX);

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  sim800l.begin(9600);

  // Set up the detection pin
  pinMode(DETECTION_PIN, INPUT);

  // Test SIM800L initialization
  Serial.println("Initializing SIM800L...");
  sim800l.println("AT"); // Test command
  delay(1000);
  sim800l.println("AT+CMGF=1"); // Set SMS mode to text
  delay(1000);
  sim800l.println("AT+CSCS=\"GSM\""); // Set character set
  delay(1000);
  Serial.println("SIM800L Initialized");
}

void loop() {
  if (digitalRead(DETECTION_PIN) == HIGH) {
    // Generate the message
    String message = "Patient " + String(patientID) + " is in danger";

    // Send the SMS
    sendSMS(phoneNumber, message);

    // Make a call
    makeCall(phoneNumber);

    // Halt for 2 seconds
    delay(2000);
  }
}

void sendSMS(const char* number, String message) {
  sim800l.println("AT+CMGS=\"" + String(number) + "\"");
  delay(1000);
  sim800l.println(message); // The SMS content
  delay(100);
  sim800l.write(26); // End of message (CTRL+Z)
  delay(2000);
  Serial.println("SMS sent");
}

void makeCall(const char* number) {
  sim800l.println("ATD" + String(number) + ";");
  delay(10000); // Call duration (10 seconds)
  sim800l.println("ATH"); // Hang up
  delay(1000);
  Serial.println("Call made and ended");
}
