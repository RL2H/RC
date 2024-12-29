#include <Wire.h>

// Pin Definitions
#define in1 A3
#define in2 A2
#define in3 A1
#define in4 A0
#define enA 3
#define enB 11
#define bluetooth_transmitter 1
#define bluetooth_receiver 0

const int trigPin = 5;
const int echoPin = 4;

// Configuration Constants
const int safeDistance = 5;   // Minimum safe distance in cm
const unsigned long orderTimeout = 5000; // Timeout for commands in milliseconds

// Motor Speeds
const int forwardSpeed = 80;
const int turnSpeed = 200;
const int reverseSpeed = 80;

// Variables
unsigned long lastOrderTime = 0;
char currentOrder = 'S'; // Default to stopped

void setup() {
    // Pin Setup
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(bluetooth_transmitter, OUTPUT);
    pinMode(bluetooth_receiver, INPUT);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    Serial.begin(9600);
    Serial.println("Car initialized. Awaiting orders...");
}

void loop() {
    // Check for Bluetooth Commands
    if (Serial.available()) {
        char command = Serial.read();
        processOrder(command);
    }

    // Check for Command Timeout
    if (millis() - lastOrderTime > orderTimeout) {
        stopCar();
        Serial.println("Order timeout. Stopping the car.");
    }

    // Measure Distance
    int distance = getDistance();
    if (distance > 0 && distance < safeDistance) {
        Serial.println("Obstacle detected. Reversing...");
        avoidObstacle(); // Reverse until the path is clear
        // Resume the previous command after clearing the obstacle
        processOrder(currentOrder);
    }

  

}

void processOrder(char order) {
    currentOrder = order;
    lastOrderTime = millis();

    switch (order) {
        case 'F':
            goForward();
            Serial.println("Forward");
            break;
        case 'L':
            turnLeft();
            Serial.println("Turn Left");
            break;
        case 'R':
            turnRight();
            Serial.println("Turn Right");
            break;
        case 'S':
            stopCar();
            Serial.println("Stop");
            break;
        case 'B':
            goBackward();
            Serial.println("Reverse");
            break;
        case 'E':
            curveRight();
            Serial.println("Curve right");
            break;
        case 'Q':
            curveLeft();
            Serial.println("Curve left");
            break;
        case 'C':
            reverseRight();
            Serial.println("Reverse right");
            break;
        case 'Z':
            reverseLeft();
            Serial.println("Reverse left");
            break;
        default:
            Serial.println("Invalid order Received");
            break;
    }
}

void goForward() {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enA, forwardSpeed);
    analogWrite(enB, forwardSpeed);
}

void turnLeft() {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enA, turnSpeed);
    analogWrite(enB, turnSpeed);
}

void curveLeft(){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, 125);
  analogWrite(enB, forwardSpeed);
}

void turnRight() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enA, turnSpeed);
    analogWrite(enB, turnSpeed);
}

void curveRight(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, forwardSpeed);
  analogWrite(enB, 125);
}

void goBackward() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enA, reverseSpeed);
    analogWrite(enB, reverseSpeed);
}

void reverseLeft() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enA, 125);
    analogWrite(enB, reverseSpeed);
}

void reverseRight() {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enA, reverseSpeed);
    analogWrite(enB, 125);
}

void stopCar() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
}


int getDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 30000); // Timeout after 30ms
    if (duration == 0) {
        Serial.println("Ultrasonic sensor timeout");
        return -1; // Indicates no reading
    }
    return duration * 0.034 / 2; // Convert time to distance in cm
}

void avoidObstacle() {
    reverseLeft();
    delay(500); // Small movement
    if (getDistance() > safeDistance) return;

    reverseRight();
    delay(500); // Small movement
    if (getDistance() > safeDistance) return;

    reverseUntilSafe();
}

void reverseUntilSafe() {
    // Keep reversing until the distance is greater than the safe distance
    while (true) {
        int distance = getDistance();
        if (distance > safeDistance) {
            Serial.println("Path cleared. Stopping reverse.");
            stopCar();
            break;
        }
        goBackward(); // Continue reversing
        delay(600);   // Small delay to allow movement
    }
}

void display(const char* message) {
    Serial.println(message);
}
