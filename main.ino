#include <Servo.h>

// Pin definitions
// Color sensor pins
#define S0 7
#define S1 5
#define S2 11
#define S3 12
#define sensorOut 10
#define OE_PIN 4  // Fixed syntax

// Motor control pins
#define IN1 A5
#define IN2 A4
#define IN3 A3
#define IN4 A2
#define ENA 3  // PWM speed control for motor A
#define ENB 6  // PWM speed control for motor B

// Servo pin
#define SERVO_PIN 11

// Ultrasonic sensor pins
#define TRIG_PIN 13
#define ECHO_PIN 12

// LED pin
#define LED_PIN A2

// Movement constants
#define MOTOR_SPEED 200  // Added missing constant
#define TURN_SPEED 180   // Added missing constant
#define TURN_DELAY 750   // Added missing constant
#define OBSTACLE_DISTANCE 20.0

// Color calibration values based on provided readings
struct ColorValue {
    int red;
    int green;
    int blue;
};

const ColorValue BLUE_CAL = {253, 61, 0};
const ColorValue GREEN_CAL = {255, 82, 119};
const ColorValue RED_CAL = {28, 227, 151};

// Tolerance for color matching
const int COLOR_TOLERANCE = 30;

Servo clawServo;
bool flagDropped = false;

void setup() {
  // Setup color sensor pins
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OE_PIN, OUTPUT);
  pinMode(sensorOut, INPUT);
  
  // Set frequency scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  digitalWrite(OE_PIN, LOW);  // Active LOW
  
  // Setup motor control pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  // Setup LED pin
  pinMode(LED_PIN, OUTPUT);
  
  // Setup ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Attach servo
  clawServo.attach(SERVO_PIN);
  
  // Initialize serial for debugging
  Serial.begin(9600);
}

// Color detection functions
ColorValue readColor() {
  ColorValue current;
  
  // Read Red value
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  current.red = pulseIn(sensorOut, LOW);
  
  // Read Green value
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  current.green = pulseIn(sensorOut, LOW);
  
  // Read Blue value
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  current.blue = pulseIn(sensorOut, LOW);
  
  return current;
}

enum Color {
  RED,
  GREEN,
  BLUE,
  BLACK,
  UNKNOWN
};

Color detectColor() {
  ColorValue current = readColor();
  
  // Print raw color values for debugging
  Serial.print("Raw RGB: ");
  Serial.print(current.red);
  Serial.print(", ");
  Serial.print(current.green);
  Serial.print(", ");
  Serial.println(current.blue);
  
  // Compare with calibration values using tolerance
  if (abs(current.red - RED_CAL.red) < COLOR_TOLERANCE &&
      abs(current.green - RED_CAL.green) < COLOR_TOLERANCE &&
      abs(current.blue - RED_CAL.blue) < COLOR_TOLERANCE) {
    return RED;
  }
  else if (abs(current.red - GREEN_CAL.red) < COLOR_TOLERANCE &&
           abs(current.green - GREEN_CAL.green) < COLOR_TOLERANCE &&
           abs(current.blue - GREEN_CAL.blue) < COLOR_TOLERANCE) {
    return GREEN;
  }
  else if (abs(current.red - BLUE_CAL.red) < COLOR_TOLERANCE &&
           abs(current.green - BLUE_CAL.green) < COLOR_TOLERANCE &&
           abs(current.blue - BLUE_CAL.blue) < COLOR_TOLERANCE) {
    return BLUE;
  }
  else if (current.red > 200 && current.green > 200 && current.blue > 200) {
    return BLACK;
  }
  
  return UNKNOWN;
}

// Movement functions
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, MOTOR_SPEED);
  analogWrite(ENB, MOTOR_SPEED);
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, MOTOR_SPEED);
  analogWrite(ENB, MOTOR_SPEED);
}

void turnLeft90() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, TURN_SPEED);
  delay(TURN_DELAY);
  stopMotors();
}

void turnRight90() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, TURN_SPEED);
  delay(TURN_DELAY);
  stopMotors();
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// Ultrasonic distance measurement
float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance = duration * 0.034 / 2;  // Convert to cm
  
  // Print the ultrasonic sensor reading
  Serial.print("Distance (cm): ");
  Serial.println(distance);
  
  return distance;  // Fixed missing semicolon
}

// Flag control
void dropFlag() {
  if (!flagDropped) {
    clawServo.write(90);  // Adjust angle as needed
    delay(500);
    flagDropped = true;
  }
}

// Part 1: Color detection and flag placement
const int PART1_PATTERN_LENGTH = 5;
const Color PART1_PATTERN[PART1_PATTERN_LENGTH] = {RED, GREEN, BLUE, RED, GREEN};
int part1PatternIndex = 0;

void challengePart1() {
  Color currentColor = detectColor();
  float distance = getDistance();
  static bool foundFirstColor = false;
  static unsigned long lastColorChange = 0;
  const unsigned long COLOR_TIMEOUT = 2000; // 2 seconds timeout for color detection
  
  // Print debug info
  Serial.print("Current Color: ");
  Serial.print(currentColor);
  Serial.print(" Distance: ");
  Serial.println(distance);
  
  if (currentColor == BLACK && !foundFirstColor) {
    moveForward();
    return;
  }
  
  if (currentColor != BLACK && !foundFirstColor) {
    foundFirstColor = true;
    stopMotors();
    delay(500);
  }
  
  if (foundFirstColor) {
    if (currentColor == PART1_PATTERN[part1PatternIndex]) {
      if (millis() - lastColorChange > COLOR_TIMEOUT) {
        Serial.print("Found correct color: ");
        Serial.println(part1PatternIndex);
        part1PatternIndex++;
        lastColorChange = millis();
        moveForward();
        delay(500);
      }
    } else if (currentColor != BLACK) {
      turnRight90();
    }
    
    if (part1PatternIndex >= PART1_PATTERN_LENGTH) {
      if (distance <= OBSTACLE_DISTANCE && distance > 0) {
        stopMotors();
        dropFlag();
        Serial.println("Flag dropped!");
        while(1);
      } else {
        moveForward();
      }
    }
  }
}

// Part 2: Maze Navigation
void challengePart2() {
  Color currentColor = detectColor();
  float distance = getDistance();
  
  if (distance < OBSTACLE_DISTANCE) {
    switch(currentColor) {
      case RED:
        turnRight90();
        break;
      case GREEN:
        turnLeft90();
        break;
      case BLUE:
        moveBackward();
        delay(1000);
        turnRight90();
        break;
    }
  } else {
    moveForward();
  }
}

// Part 3: Color Pattern Detection
const int MAX_PATTERN = 5;
const Color TARGET_PATTERN[MAX_PATTERN] = {RED, GREEN, BLUE, GREEN, BLUE};
int patternIndex = 0;

// Track visited color positions
const int MAZE_SIZE = 15;  // 1.5m maze with 10cm grid
bool visitedGreen[MAZE_SIZE][MAZE_SIZE] = {false};
bool visitedBlue[MAZE_SIZE][MAZE_SIZE] = {false};
int currentX = MAZE_SIZE/2;  // Start in middle
int currentY = MAZE_SIZE/2;
int currentDir = 0;  // 0=North, 1=East, 2=South, 3=West

void updatePosition() {
  // Update position based on current direction
  switch(currentDir) {
    case 0: currentY--; break;  // North
    case 1: currentX++; break;  // East
    case 2: currentY++; break;  // South
    case 3: currentX--; break;  // West
  }
}

bool isPositionValid() {
  return currentX >= 0 && currentX < MAZE_SIZE && 
         currentY >= 0 && currentY < MAZE_SIZE;
}

void challengePart3() {
  Color currentColor = detectColor();
  float distance = getDistance();
  static unsigned long lastColorChange = 0;
  const unsigned long COLOR_TIMEOUT = 1000;
  
  // Print debug info
  Serial.print("Pattern Index: ");
  Serial.print(patternIndex);
  Serial.print(" Current Color: ");
  Serial.println(currentColor);
  
  // Check if we found a target color
  if (currentColor == TARGET_PATTERN[patternIndex] && 
      millis() - lastColorChange > COLOR_TIMEOUT) {
    
    // For repeated colors (Green and Blue), check if position was visited
    bool validColor = true;
    if (currentColor == GREEN && patternIndex > 0) {
      validColor = !visitedGreen[currentX][currentY];
      if (validColor) visitedGreen[currentX][currentY] = true;
    }
    else if (currentColor == BLUE && patternIndex > 0) {
      validColor = !visitedBlue[currentX][currentY];
      if (validColor) visitedBlue[currentX][currentY] = true;
    }
    
    if (validColor) {
      // Blink LED to indicate pattern match
      digitalWrite(LED_PIN, HIGH);
      delay(500);
      digitalWrite(LED_PIN, LOW);
      patternIndex++;
      lastColorChange = millis();
      Serial.println("Pattern color found!");
    }
  }
  
  // Navigation logic
  if (distance < OBSTACLE_DISTANCE) {  // Wall detected
    stopMotors();
    delay(100);
    turnRight90();
    currentDir = (currentDir + 1) % 4;
  } else {
    moveForward();
    updatePosition();
    
    // If position invalid, turn around
    if (!isPositionValid()) {
      stopMotors();
      turnRight90();
      turnRight90();
      currentDir = (currentDir + 2) % 4;
      updatePosition();  // Move back to valid position
    }
  }
  
  // Victory condition
  if (patternIndex >= MAX_PATTERN) {
    stopMotors();
    while(1) {  // Victory flash
      digitalWrite(LED_PIN, HIGH);
      delay(200);
      digitalWrite(LED_PIN, LOW);
      delay(200);
    }
  }
}

void loop() {
  // Uncomment the challenge part you want to run
  // challengePart1();
  // challengePart2();
  challengePart3();
  
  delay(100);
}
