#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SparkFun_TB6612.h>
#include <ESP32Servo.h>


// OLED display setup
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Motor driver pins
#define AIN1 17
#define BIN1 18
#define AIN2 16
#define BIN2 19
#define PWMA 4
#define PWMB 23
#define STBY 13

// Front IR sensor pins
#define FRONT_IR_DIGITAL 26
#define FRONT_IR_ANALOG 39

// Multiplexer select pins
const int selectPins[3] = {25, 33, 32};
const int muxOutputPin = 35;
const uint8_t sensorCount = 8;

// IR sensor pins
const int LIR = 36;
const int RIR = 27;

// Motor setup
const int offsetA = 1;
const int offsetB = 1;
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

// PID control parameters
int lastError = 0;
int integral = 0;
float Kp = 0.013;
float Ki = 0.0;
float Kd = 0.008;

// Motor speed parameters
int maxSpeed = 110;
int baseSpeed = 75;

// Calibrated threshold values
uint16_t threshold[sensorCount] = {3000,3000, 3000, 3000, 3000, 3000, 3000, 3000};
uint16_t LIRThreshold = 2460;
uint16_t RIRThreshold = 2495;

// Sensor values
bool sensorDigital[sensorCount];
bool LIRDetected, RIRDetected;
volatile bool objectDetected = false;
bool interruptAttached = true;  // Track if interrupt is attached
bool Allblack = false;
const unsigned long debounceDelay = 100; // debounce period in milliseconds
unsigned long allblackDebounceStart = 0;
bool allblackConfirmed = false;
bool isTurning = false;
const unsigned long frontSensorDebounceDelay = 100; // Adjust this debounce delay as needed
unsigned long frontSensorDebounceStart = 0;
bool frontSensorsOnLineConfirmed = false;


unsigned long allblackStartTime = 0;
bool timerActive = false;

Servo servo1;
Servo servo2;

enum RobotState {
  LINE_FOLLOW,
  PICK,
  PLACE
  // Add more states here as needed
};

RobotState currentState = LINE_FOLLOW;

// ISR for front IR sensor
void IRAM_ATTR frontIRInterrupt() {
  if (interruptAttached) {
    objectDetected = true;
    detachInterrupt(digitalPinToInterrupt(FRONT_IR_DIGITAL));  // Permanently detach interrupt
    interruptAttached = false;  // Mark interrupt as detached
  }
}

void setup() {
  Serial.begin(115200);

  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(20, 20);
  display.println(F("VOYEGERS"));
  display.setTextSize(1);
  display.display();

  // Set up the select pins as outputs
  for (int i = 0; i < 3; i++) {
    pinMode(selectPins[i], OUTPUT);
  }
  pinMode(muxOutputPin, INPUT);
  pinMode(LIR, INPUT); 
  pinMode(RIR, INPUT);
  
  // Setup front IR sensor
  pinMode(FRONT_IR_DIGITAL, INPUT_PULLUP);
  pinMode(FRONT_IR_ANALOG, INPUT);
  attachInterrupt(digitalPinToInterrupt(FRONT_IR_DIGITAL), frontIRInterrupt, FALLING);
  interruptAttached = true;  // Mark interrupt as attached initially

  Serial.println("Robot State Monitor Initialized");

  digitalWrite(STBY, HIGH);  // Enable 
  servo1.attach(13);  //Arm
  servo2.attach(14);  //Base
  servo1.write(140);  // Initial position
  servo2.write(120);
}

void loop() {
  readSensors();
  updateState();
  executeState();
  displaySensorsOnOLED();
  delay(10);  // Small delay to stabilize the loop
}

void readSensors() {
  for (uint8_t i = 0; i < sensorCount; i++) {
    selectMuxPin(i);
    delayMicroseconds(2);
    uint16_t sensorValue = analogRead(muxOutputPin);
    sensorDigital[i] = (sensorValue > threshold[i]);
  }

  LIRDetected = analogRead(LIR) < LIRThreshold;
  RIRDetected = analogRead(RIR) < RIRThreshold;
}

void executeLineFollow() {
  long position = 0;
  long total = 0;

  for (uint8_t i = 0; i < sensorCount; i++) {
    position += sensorDigital[i] * (i * 1000);
    total += sensorDigital[i];
  }

  int error = (total > 0) ? (position / total) - 3500 : 0;
  int P = error;
  integral += error;
  int D = error - lastError;
  int adjust = Kp * P + Ki * integral + Kd * D;
  lastError = error;

  int rightMotorSpeed = constrain(baseSpeed + adjust, -maxSpeed, maxSpeed);
  int leftMotorSpeed = constrain(baseSpeed - adjust, -maxSpeed, maxSpeed);

  bool frontSensorsOnLine = (total > 0);
  // Only perform turning actions if we are in LINE_FOLLOW state
  if (currentState == LINE_FOLLOW) {
    if (LIRDetected && !frontSensorsOnLine && LIRDetected) {
      turnLeft90();
    } else if (RIRDetected && !frontSensorsOnLine) {
      turnRight90();
    } else if (LIRDetected && frontSensorsOnLine) {
      turnLeftT();
    } else {
      motor1.drive(leftMotorSpeed);
      motor2.drive(rightMotorSpeed);
    }
  } else {
    // If not in LINE_FOLLOW state, don't turn, just stop the motors or keep the current speed
    motor1.drive(leftMotorSpeed);
    motor2.drive(rightMotorSpeed);
  }
}
void turnLeft90() {
  isTurning = true;
  motor1.drive(-baseSpeed);  // Stop left motor
  motor2.drive(baseSpeed);   // Move right motor forward
  delayUntilMiddleSensorsOnLine(false);
  isTurning = false;  // Not a T-turn
}
void turnRight90() {
  isTurning = true;
  motor1.drive(baseSpeed);   // Move left motor forward
  motor2.drive(-baseSpeed);  // Stop right motor
  delayUntilMiddleSensorsOnLine(false);
  isTurning = false;  // Not a T-turn
}
void turnLeftT() {// This is a T-turn
  isTurning = true;
  motor1.drive(-baseSpeed);  // Stop left motor
  motor2.drive(baseSpeed);   // Move right motor forward
  delayUntilMiddleSensorsOnLine(true);
  isTurning = false; 
}
void turnLeft180() {
  isTurning = true;
  motor1.drive(baseSpeed);  // Stop left motor
  motor2.drive(-baseSpeed);   // Move right motor forward
  
  // Wait until we see the line with the rightmost sensor
  unsigned long startTime = millis();
  const unsigned long maxWaitTime = 1500;  // Increased time for 180-degree turn
  
  while (true) {
    readSensors();
    
    // Look for the line with the rightmost sensor
    if (sensorDigital[0] == 1) {
      break;
    }
    
    if (millis() - startTime > maxWaitTime) {
      break;
    }
    
    delay(10);
  }
  
  // Stop the motors after completing the turn
  motor1.drive(0);
  motor2.drive(0);
  delay(100);  // Longer stabilization delay
  motor1.drive(-baseSpeed);
  motor2.drive(-baseSpeed);
  delay(100); 
  motor1.drive(0);
  motor2.drive(0);
  delay(10);
  isTurning = false;
}
void executePick() {
  // First stop motors for 1 second
  motor1.brake();
  motor2.brake();
  delay(500);
  
  delay(100);
  servo1.write(50);
  servo2.write(5);
  delay(600);
  servo1.write(145);
  delay(500);
  servo2.write(90);
  delay(1000);

  // Perform 180-degree turn
  turnLeft180();
  
  // Add a small delay to stabilize
  delay(200);
  
  // Clear any sensor flags
  objectDetected = false;
  LIRDetected = false;
  RIRDetected = false;
  motor1.drive(baseSpeed);
  motor2.drive(baseSpeed);
  delay(10);  // Adjust this delay based on your robot's speed
  currentState = LINE_FOLLOW;
}
void executePlace() {
  // First stop motors for 1 second
  motor1.drive(0);
  motor2.drive(0);
  delay(500);
  motor1.drive(-baseSpeed);
  motor2.drive(-baseSpeed);
  delay(650);
  
  motor1.brake();
  motor2.brake();
  delay(300); 
  
  servo2.write(5);
  delay(600);
  servo1.write(70);
  delay(600);
  servo2.write(120);
  delay(500);
  servo1.write(140);
  delay(500);


  motor1.brake();
  motor2.brake();
  delay(100); 

  turnLeft180();
  
  // Add a small delay to stabilize
  delay(500);
  
  // Clear any sensor flags
 
  // Drive forward slightly to clear the T-junction
  motor1.drive(baseSpeed);
  motor2.drive(baseSpeed);
  delay(300);  // Adjust this delay based on your robot's speed
  
  currentState = LINE_FOLLOW;
}

void updateState() {
  bool frontSensorsOnLine = false;
  for (uint8_t i = 0; i < sensorCount; i++) {
    if (sensorDigital[i]) {
      frontSensorsOnLine = true;
      break;
    }
  }

  // Debounce front sensors
  if (frontSensorsOnLine && !frontSensorsOnLineConfirmed) {
    if (millis() - frontSensorDebounceStart >= frontSensorDebounceDelay) {
      frontSensorsOnLineConfirmed = true; // Confirmed as true after debounce delay
    }
  } else if (!frontSensorsOnLine) {
    frontSensorsOnLineConfirmed = false;
    frontSensorDebounceStart = millis(); // Reset debounce timer
  }

  // Debounce the Allblack condition
  bool allblackDetected = (sensorDigital[0] == 1 && sensorDigital[1] == 1 && sensorDigital[2] == 1 && 
                           sensorDigital[3] == 1 && sensorDigital[4] == 1 && sensorDigital[5] == 1 && 
                           sensorDigital[6] == 1 && sensorDigital[7] == 1);
  
  if (allblackDetected && !allblackConfirmed) {
    if (millis() - allblackDebounceStart >= debounceDelay) {
      allblackConfirmed = true;  // Confirmed as true after debounce delay
    }
  } else if (!allblackDetected) {
    allblackConfirmed = false;
    allblackDebounceStart = millis();  // Reset debounce timer
  }

  // Start a non-blocking timer if debounced Allblack is detected
  if (allblackConfirmed && !timerActive && !isTurning) {
    allblackStartTime = millis();
    timerActive = true;
  }

  // Check if 1000ms has passed and LIR/RIR are not detected
  if (timerActive && millis() - allblackStartTime >= 300 && !LIRDetected && !RIRDetected && !isTurning) {
    currentState = PLACE;
    timerActive = false;  // Reset timer after changing state
  }

  // Store previous state for change detection
  RobotState previousState = currentState;

  switch (currentState) {
    case LINE_FOLLOW:
      if (objectDetected && frontSensorsOnLineConfirmed) {
        currentState = PICK;
        objectDetected = false;  // Reset the flag
      }
      break;

    case PICK:
      // State transition is handled in executePick()
      break;

    case PLACE:
      // No transition in PLACE; add if needed
      break;
  }

  // Print state change to Serial monitor
  if (previousState != currentState) {
    Serial.print("State changed from: ");
    switch (previousState) {
      case LINE_FOLLOW:
        Serial.print("LINE_FOLLOW");
        break;
      case PICK:
        Serial.print("PICK");
        break;
      case PLACE:
        Serial.print("PLACE");
        break;
    }
    Serial.print(" to: ");
    switch (currentState) {
      case LINE_FOLLOW:
        Serial.println("LINE_FOLLOW");
        break;
      case PICK:
        Serial.println("PICK");
        break;
      case PLACE:
        Serial.println("PLACE");
        break;
    }
  }
}

void executeState() {
  switch (currentState) {
    case LINE_FOLLOW:
      executeLineFollow();
      break;
      
    case PICK:
      executePick();
      break;

    case PLACE:
      executePlace();
      break;

  }
}
void selectMuxPin(byte pin) {
  for (int i = 0; i < 3; i++) {
    digitalWrite(selectPins[i], (pin & (1 << i)) ? HIGH : LOW);
  }
}
void displaySensorsOnOLED() {
  display.clearDisplay();      // Clear previous content from the screen
  display.setTextSize(2);      // Set text size
  display.setTextColor(WHITE); // Set text color
  // Display Allblack status
  display.setCursor(0, 20);  // Move cursor to next line
  display.print("B: ");
  display.println(Allblack ? "1" : "0");
  display.setCursor(0, 40);
  display.setTextSize(1);      // Set text size
  switch (currentState) {
    case LINE_FOLLOW:
      display.println("LINE_FOLLOW");
      break;
    case PICK:
      display.println("PICK");
      break;
    case PLACE:
      display.println("PLACE");
      break;
  }

  // Update display with the new content
  display.display();
}
void delayUntilMiddleSensorsOnLine(bool isTTurn) {
  unsigned long startTime = millis();        // Start time to avoid indefinite looping
  const unsigned long maxWaitTime = 1000;    // Maximum wait time in milliseconds

  while (true) {
    readSensors();  // Continuously read sensor values

    // For T-turns, only stop if the leftmost sensor detects the line
    if (isTTurn && sensorDigital[7] == 1) {
      break;  // Stop when [7] detects the line
    }
    
    // For regular turns, stop if either of the corner sensors detects the line
    if (!isTTurn && (sensorDigital[0] == 1 || sensorDigital[7] == 1)) {
      break;  // Stop when [0] or [7] detects the line
    }

    // Check if the waiting time exceeds the maximum allowed duration
    if (millis() - startTime > maxWaitTime) {
      break;  // Break the loop if we've waited too long, avoiding getting stuck
    }

    delay(10);  // Small delay to prevent rapid sampling
  }

  // Stop the motors after detecting the line to stabilize the turn
  motor1.drive(0);
  motor2.drive(0);
  delay(50);  // Brief delay to stabilize after stopping
}

