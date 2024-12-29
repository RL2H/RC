#include <LiquidCrystal.h>
#include <PinChangeInterrupt.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>
// Pin configuration for the LCD
const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 13, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Right Motor connections
int enA = 3;
int in1 = A3;
int in2 = A2;
// Left Motor connections
int enB = 11;
int in3 = A1;
int in4 = A0;
// Left sensor
int leftSensor = 12;
// Right sensor
int rightSensor = 6;

// Rotary encoder pins (single signal)
int leftEncA = 10;  // Left encoder signal (Interrupt pin)
int rightEncA = 2; // Right encoder signal (Interrupt pin)
// Variables for encoder counts
volatile unsigned long leftEncoderCount = 0;
volatile unsigned long rightEncoderCount = 0;
// Wheel parameters
const float wheelCircumference = 21.031;
// Encoder pulses per revolution (adjust to your encoder's specs)
const int pulsesPerRevolution = 20; 
// Interrupt service routines for encoder signals
void leftEncoderInterrupt() {
  leftEncoderCount++;
}
void rightEncoderInterrupt() {
  rightEncoderCount++;
}

Adafruit_MPU6050 mpu;
float roll = 0;

int i = 1;

float gyroZ;

unsigned long previousTime;

float angleZ = 0; // Total angle of rotation in degrees

float distance = 0;

float time = 0;

int currentState = 1;


void setup() {
  lcd.begin(16, 2); // Initialize the LCD with 16 columns and 2 rows

  pinMode(leftSensor, INPUT);
  pinMode(rightSensor, INPUT);

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Encoder pins
  pinMode(leftEncA, INPUT_PULLUP);
  pinMode(rightEncA, INPUT_PULLUP);

  // Attach interrupts
  attachPCINT(digitalPinToPCINT(leftEncA), leftEncoderInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncA), rightEncoderInterrupt, RISING);

  if (!mpu.begin()) {
    while (1); // Halt if MPU6050 initialization fails
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  previousTime = millis();

}

void followLine(int speed){
  int leftState = digitalRead(leftSensor);
  int rightState = digitalRead(rightSensor);

  // Move forward
  if (leftState == 1 && rightState == 1) {
    analogWrite(enA, speed);
    analogWrite(enB, speed);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  // Turn left
  else if (leftState == 0 && rightState == 1) {
    analogWrite(enA, 255);
    analogWrite(enB, 255);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  // Turn right
  else if (leftState == 1 && rightState == 0) {
    analogWrite(enA, 255);
    analogWrite(enB, 255);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  // Stop
  else {
    if (i == 1){
       analogWrite(enA, 60);
       analogWrite(enB, 60);
       digitalWrite(in1, HIGH);
       digitalWrite(in2, LOW);
       digitalWrite(in3, HIGH);
       digitalWrite(in4, LOW);
    }
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }
}

void stopMotors(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void turn360(){
  angleZ = 0;
  analogWrite(enA, 175);
  analogWrite(enB, 175);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  while (angleZ < 360) {  // Keep turning until angleZ reaches approximately 360
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp); 
    // Calculate the time step
    unsigned long currentTime = millis();
    float dt = (currentTime - previousTime) / 1000.0; // Convert time to seconds
    previousTime = currentTime;
    // Integrate the gyro Z value to calculate the angle change

    angleZ += g.gyro.z * dt * 57.2958; // Convert rad/s to deg/s
    // Handle overflow
    if (angleZ >= 360.0) break;
  }
}

void calculateDistance() {
  float leftDistance = (leftEncoderCount / (float)pulsesPerRevolution) * wheelCircumference;
  float rightDistance = (rightEncoderCount / (float)pulsesPerRevolution) * wheelCircumference;
  distance = (leftDistance + rightDistance) / 2.0;
}
void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  roll = (atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI) ;
  time = millis()/1000;
  calculateDistance();
 
  switch (currentState) {
    case 1:
      followLine(70);
      if (roll > 2){
        currentState = 2;
      }
      else if (roll < -2){
        currentState = 5;
      }
      break;

    case 2:
      followLine(200);
      if (roll > 20){
        stopMotors();
        delay(3000);
        currentState = 3;
      }
      
      break;

    case 3:
      followLine(200);
      if (roll > -1 && roll < 1){
        stopMotors();
        delay(4000);
        currentState = 4;
      }
      break;

    case 4:
      turn360();
      currentState = 1;
      i=i+1;
      break;
    
    case 5:
      followLine(40);
      if (roll > -1 && roll < 1){
        currentState = 6;
        i=i-1;
      }
      break;

    case 6:
      stopMotors();
      delay(2500);
      currentState = 7;
      break;

    case 7:
      leftEncoderCount = 0;
      rightEncoderCount = 0;
      followLine(69);
      calculateDistance();
      if (distance > 258 && distance < 261){
        stopMotors();
        delay(3000);
        currentState = 8;
      }
      break;

    case 8:
    followLine(70);
    if (digitalRead(leftSensor) == 0 && digitalRead(rightSensor) == 0) {  // Both sensors off line
        stopMotors();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Track completed");
    }
    else if (roll > 2 || roll < -2) {  // New slope detected
        currentState = 1;
    }
    break;

    default:
      followLine(70);
      break;
  }

  lcd.setCursor(0, 0);
  lcd.print("Deg: ");
  lcd.print(roll, 2);
  lcd.setCursor(11, 0);
  lcd.print("T:");
  lcd.print(time);
  lcd.print("s");
  lcd.setCursor(0, 1);
  lcd.print("Distance:");
  lcd.print(distance, 1);
  lcd.print("cm");

}
 

  

  

  
