#include <Wire.h>
#include "Adafruit_VL6180X.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SparkFun_TB6612.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C

// Motor pins
#define AIN1 17
#define BIN1 18
#define AIN2 16
#define BIN2 19
#define PWMA 4
#define PWMB 23
#define STBY 13

// Front IR sensor pin
#define FRONT_IR_ANALOG 39

// Motor setup
const int offsetA = 1;
const int offsetB = 1;
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

// Constants for wall following
const int TARGET_DISTANCE = 110;  // Target distance from wall in mm
const int DISTANCE_TOLERANCE = 5; // Reduced tolerance for smoother control
const int BASE_SPEED = 65;        // Slightly increased base speed
const int MAX_ADJUST = 20;        // Reduced max adjustment for finer control
const int FRONT_THRESHOLD = 800;  // Analog reading threshold for front obstacle

Adafruit_VL6180X vl = Adafruit_VL6180X();
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Serial.begin(115200);

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  display.clearDisplay();

  // Initialize TOF sensor
  if (!vl.begin()) {
    Serial.println("Failed to find sensor");
    while (1);
  }

  // Initialize front IR sensor
  pinMode(FRONT_IR_ANALOG, INPUT);

  delay(10); // Give sensors time to initialize
}

void wallFollow() {
  // Read sensors
  uint8_t range = vl.readRange();
  uint8_t status = vl.readRangeStatus();
  int frontIR = analogRead(FRONT_IR_ANALOG);

  if (frontIR < FRONT_THRESHOLD) { // Obstacle in front
    // Stop and turn based on wall-following direction
    motor1.drive(0);
    motor2.drive(0);
    delay(50);
    if (range < 150) { // Turn right if close to wall
      turnRight();
    } else {                        // Turn left if far from wall
      turnLeft();
    }
    return;
  }

  // Only adjust if TOF reading is valid
  if (status == VL6180X_ERROR_NONE) {
    int error = range - TARGET_DISTANCE;
    int speedAdjust = 0;

    // Calculate speed adjustment based on error
    if (abs(error) > DISTANCE_TOLERANCE) {
      speedAdjust = map(error, -25, 25, -MAX_ADJUST, MAX_ADJUST);
      speedAdjust = constrain(speedAdjust, -MAX_ADJUST, MAX_ADJUST);
    }

    // Adjust motor speeds
    int leftSpeed = BASE_SPEED - speedAdjust;
    int rightSpeed = BASE_SPEED + speedAdjust;

    // Constrain speeds to valid range
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    // Drive motors
    motor1.drive(leftSpeed);
    motor2.drive(rightSpeed);
  } else {
    // If sensor reading is invalid, drive straight
    motor1.drive(BASE_SPEED);
    motor2.drive(BASE_SPEED);
  }
}

void turnLeft() {
  motor1.drive(-BASE_SPEED);  // Left motor backward
  motor2.drive(BASE_SPEED);   // Right motor forward
  delay(350);                 // Shortened delay for a quicker turn
  motor1.drive(0);
  motor2.drive(0);
}

void turnRight() {
  motor1.drive(BASE_SPEED);   // Left motor forward
  motor2.drive(-BASE_SPEED);  // Right motor backward
  delay(350);                 // Shortened delay for a quicker turn
  motor1.drive(0);
  motor2.drive(0);
}

void loop() {
  wallFollow();
  delay(10); // Small delay to prevent sensor overread
}
