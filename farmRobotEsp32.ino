// Project: Mini farming robot
// Code Author: Dhrubo Roy Partho
// Date: 26/11/2024
// Version: 1.0.0

#include <WiFi.h>
#include <WiFiUdp.h>

// Wi-Fi credentials
const char *ssid = "*";
const char *password = "hello-lol";

// UDP setup
WiFiUDP udp;
const int localPort = 8888;
char packetBuffer[255];

// Motor control pins
const int leftMotorPin = 4;
const int rightMotorPin = 5;

// PID variables
float currentError = 0;
float futureError = 0;
float previousError = 0;
float integral = 0;

// PID constants
float Kp = 2.0;  // Proportional gain
float Ki = 0.5;  // Integral gain
float Kd = 1.0;  // Derivative gain

// U-turn threshold
const int uTurnThreshold = 120; // Need to adjust with real world

String decision = "";

void setup() {
    Serial.begin(115200);

    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // Start UDP
    udp.begin(localPort);
    Serial.println("UDP server started");

    // Motor pins
    pinMode(leftMotorPin, OUTPUT);
    pinMode(rightMotorPin, OUTPUT);
}

void loop() {
    // Receive data from Android
    int packetSize = udp.parsePacket();
    if (packetSize) {
        int len = udp.read(packetBuffer, 255);
        if (len > 0) {
        packetBuffer[len] = 0;  // Null-terminate the string
        }
        parseData(packetBuffer);

        if(decision != "" && decision != "No Decision"){
            // Calculating PID value
            float pidOutput = calculatePID(currentError);

            // Control motors
            controlMotors(pidOutput, futureError);
        }
    }

}

// Function to parse incoming data
void parseData(const char *data) {
    String receivedData = String(data);

    // Extract current and future distances
    String future = getValue(receivedData, "future");
    String current = getValue(receivedData, "current");
    decision = getValue(receivedData, "decision");

    currentError = current.toFloat(); // Error for PID
    futureError = future.toFloat();   // U-turn decision

    Serial.print("Current Distance: ");
    Serial.println(currentError);
    Serial.print("Future Distance: ");
    Serial.println(futureError);
}

// Function for extract values
String getValue(String data, String key) {
    int startIndex = data.indexOf(key + ":") + key.length() + 1;
    int endIndex = data.indexOf(",", startIndex);
    if (endIndex == -1) {
        endIndex = data.length();  // Handle the last key
    }
    return data.substring(startIndex, endIndex);
}

// Function for PID value calculation
float calculatePID(float error) {
    float derivative = error - previousError;
    integral += error;

    // PID formula
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // Limit integral windup
    integral = constrain(integral, -100, 100);

    previousError = error;

    // Debugging PID Output
    Serial.print("PID Output: ");
    Serial.println(output);

    return output;
}

// Function for motor control
void controlMotors(float pidOutput, float futureDistance) {
    // U-turn logic
    if (abs(futureDistance) > uTurnThreshold) {
        Serial.println("Performing U-Turn");
        performUTurn();
        return;
    }

    // Wrap PID output to motor range
    pidOutput = constrain(pidOutput, -255, 255);

    // Calculate motor speeds
    int leftMotorSpeed = constrain(255 - pidOutput, 0, 255);
    int rightMotorSpeed = constrain(255 + pidOutput, 0, 255);

    // Debugging motor speeds
    Serial.print("Left Motor Speed: ");
    Serial.println(leftMotorSpeed);
    Serial.print("Right Motor Speed: ");
    Serial.println(rightMotorSpeed);

    // Apply motor speeds
    analogWrite(leftMotorPin, leftMotorSpeed);
    analogWrite(rightMotorPin, rightMotorSpeed);
}


// Perform U-turn
void performUTurn() {
    // Stop motors
    analogWrite(leftMotorPin, 0);
    analogWrite(rightMotorPin, 0);
    delay(500);

    // Rotate in place
    analogWrite(leftMotorPin, 255);
    analogWrite(rightMotorPin, 0);
    delay(1500);  // Adjust turning time

    // Resume forward motion
    analogWrite(leftMotorPin, 255);
    analogWrite(rightMotorPin, 255);
    delay(500);
}
