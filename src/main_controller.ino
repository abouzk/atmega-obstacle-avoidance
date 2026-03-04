/**
 * Basic mobile robot controller
 * Uses encoders for movement tracking and an ultrasonic sensor for obstacle checks
 * 
 * Hardware:
 * - 2x DC motors with quadrature encoders
 * - HC-SR04 ultrasonic sensor for obstacle detection
 * - Arduino with AVR interrupts for encoder counting
 */

#include <avr/interrupt.h>

// ===== MOTOR PINS =====
#define IN1 12  // Right motor direction control (HIGH = forward)
#define IN2 13  // Right motor direction control (LOW = forward)
#define IN3 7   // Left motor direction control (HIGH = forward)
#define IN4 8   // Left motor direction control (LOW = forward)
#define ENA 11  // Right motor PWM speed (0-255)
#define ENB 6   // Left motor PWM speed (0-255)

// ===== ENCODER PINS =====
// Encoder pins used for wheel tick counting
#define ENCA1 2      // Right encoder channel A (interrupt pin)
#define ENCB1 4      // Right encoder channel B
#define ENCA2 3      // Left encoder channel A (interrupt pin)
#define ENCB2 5      // Left encoder channel B

// ===== ULTRASONIC SENSOR PINS =====
const int echoPin = 9;   // Receives echo pulse from HC-SR04
const int trigPin = 10;  // Triggers distance measurement on HC-SR04

// ===== BUTTON PIN =====
#define BUTTON 22  // Button input (pulled high, LOW when pressed)

// ===== TUNING VALUES =====
// Change these based on your robot
volatile int r_pos = 0;  // Right encoder position (ticks)
volatile int l_pos = 0;  // Left encoder position (ticks)
const unsigned int encoder_ticks_per_meter = 9000;  // Encoder ticks per meter traveled
const int fullturndegrees = 2900;  // Encoder ticks for a full 360-degree rotation

void setup() {
  Serial.begin(9600);
  
  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Encoder pins
  pinMode(ENCA1, INPUT);
  pinMode(ENCB1, INPUT);
  pinMode(ENCA2, INPUT);
  pinMode(ENCB2, INPUT);

  // Use interrupts to count encoder ticks
  // CHANGE means it triggers on both rising and falling edges
  attachInterrupt(digitalPinToInterrupt(ENCA1), rightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA2), leftEncoder, CHANGE);

  // Ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Button with internal pull-up (LOW when pressed)
  pinMode(BUTTON, INPUT_PULLUP);
}

void loop() {
  // Read button (LOW when pressed)
  bool buttonState = digitalRead(BUTTON);
  
  // TODO: Implement button-triggered modes (autonomous, manual, etc.)
  // if (buttonState == LOW) {
  //   Serial.println("Button Pressed");
  // }
  
  // Print sensor + encoder values
  Serial.print("Right: ");
  Serial.print(r_pos);
  Serial.print("  |  Left: ");
  Serial.print(l_pos);
  Serial.print("  |  Distance: ");
  Serial.print(measureDistance());
  Serial.println(" cm");
  
  // Run one waypoint-style demo sequence once, then idle
  static bool ranSequence = false;
  if (!ranSequence) {
    driveForward(1.0);
    turn(90);
    driveForward(0.5);
    turn(180);
    ranSequence = true;
    Serial.println("---SEQUENCE COMPLETE---");
  }

  delay(300);
}

/**
 * Right encoder interrupt
 * Updates right wheel tick count
 */
void rightEncoder() {
  // Same state = one direction, different = the other direction
  if (digitalRead(ENCA1) == digitalRead(ENCB1)) {
    r_pos++;  // Forward rotation
  } else {
    r_pos--;  // Backward rotation
  }
}

/**
 * Left encoder interrupt
 * Updates left wheel tick count
 */
void leftEncoder() {
  if (digitalRead(ENCA2) == digitalRead(ENCB2)) {
    l_pos++;  // Forward rotation
  } else {
    l_pos--;  // Backward rotation
  }
}

/**
 * Drive forward for a set distance (meters)
 * Uses a simple P controller to keep left/right wheels close
 */
void driveForward(float distance) {
  // Convert meters to encoder ticks
  long target_ticks = distance * encoder_ticks_per_meter; 
  
  // Reset ticks for this move
  noInterrupts();
  r_pos = 0; 
  l_pos = 0;
  interrupts();

  // Basic P controller settings
  const float Kp = 5.0;       // Bigger Kp = stronger correction
  const int baseSpeed = 200;  // Base PWM speed (0-255)

  // Keep driving until target is reached
  while (true) {
    int rightTicks;
    int leftTicks;
    noInterrupts();
    rightTicks = r_pos;
    leftTicks = l_pos;
    interrupts();

    if ((abs(rightTicks) + abs(leftTicks)) / 2 >= target_ticks) {
      break;
    }
    
    // If blocked, stop and wait until clear
    if(checkObstacle()) {
      stopMotors();
      Serial.println("Obstacle detected. Waiting...");
      while (checkObstacle()) {
        delay(200);
      }
      Serial.println("Path clear. Continuing forward.");
    }

    // Wheel sync error
    int error = abs(rightTicks) - abs(leftTicks); 

    // Adjust motor speed based on error
    int rightSpeed = baseSpeed - (Kp * error);
    int leftSpeed = baseSpeed + (Kp * error);

    // Keep PWM in range
    rightSpeed = constrain(rightSpeed, 0, 255);
    leftSpeed = constrain(leftSpeed, 0, 255);

    // Drive both motors forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, rightSpeed);

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, leftSpeed);

    // Debug print
    Serial.print("Target: "); Serial.print(target_ticks);
    Serial.print(" | R: "); Serial.print(rightTicks);
    Serial.print(" | L: "); Serial.print(leftTicks);
    Serial.print(" | Error: "); Serial.println(error);
  }

  stopMotors();
}

/**
 * Turn in place by a fixed angle
 * Supports: 90, -90, 180, 360
 */
void turn(int degrees) {
  Serial.print("TURN: ");
  Serial.print(degrees);
  Serial.println(" degrees");
  delay(1000);  // Small pause before turn
  
  reset_encoders();
  
  // Set motor directions and tick scaling for each turn
  int r_fac, l_fac;
  
  if (degrees == 90) {
    // Right turn: right motor backward, left motor forward
    digitalWrite(IN1, LOW);   // Backward right
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, 255);
    r_fac = -4;

    digitalWrite(IN3, HIGH);  // Forward left
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 255);
    l_fac = 4;
    
  } else if (degrees == -90) {
    // Left turn: right motor forward, left motor backward
    digitalWrite(IN1, HIGH);  // Forward right
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 255);
    r_fac = 4;

    digitalWrite(IN3, LOW);   // Backward left
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, 255);
    l_fac = -4;
    
  } else if (degrees == 180) {
    // 180° turn: right backward, left forward
    digitalWrite(IN1, LOW);   // Backward right
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, 255);
    r_fac = -2;

    digitalWrite(IN3, HIGH);  // Forward left
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 255);
    l_fac = 2;
    
  } else if (degrees == 360) {
    // 360° circle: right backward, left forward
    digitalWrite(IN1, LOW);   // Backward right
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, 250);
    r_fac = -1;

    digitalWrite(IN3, HIGH);  // Forward left
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 250);
    l_fac = 1;
    
  } else {
    Serial.println("ERROR: Unsupported turn angle. Supported: ±90, 180, 360");
    return;
  }

  // Print target ticks
  Serial.print("Target ticks - Right: ");
  Serial.print(fullturndegrees / r_fac);
  Serial.print(" | Left: ");
  Serial.println(fullturndegrees / l_fac);

  // Keep turning until both wheels hit target
  while (abs(r_pos) < abs(fullturndegrees / r_fac) || abs(l_pos) < abs(fullturndegrees / l_fac)) {
    // Stop right motor if it reached target
    if (abs(r_pos) >= abs(fullturndegrees / r_fac)) {
      analogWrite(ENA, 0);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
    }
    // Stop left motor if it reached target
    if (abs(l_pos) >= abs(fullturndegrees / l_fac)) {
      analogWrite(ENB, 0);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
    }
    
    // Debug print
    Serial.print("Right: ");
    Serial.print(r_pos);
    Serial.print("  |  Left: ");
    Serial.println(l_pos);
  }

  stopMotors();
  Serial.println("Turn complete.");
}

/**
 * Measure distance with HC-SR04
 * Returns cm, or 0 if timeout
 */
float measureDistance() {
  // Start from LOW
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Send trigger pulse
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure echo pulse width (with timeout)
  long duration = pulseIn(echoPin, HIGH, 30000);

  // Convert time to distance in cm
  float distanceCm = (duration / 2.0) * 0.0343;

  // Timeout gives distance 0
  if (duration == 0) {
    distanceCm = 0;
  }

  return distanceCm;
}

/**
 * Check if something is too close in front
 * Returns true if obstacle is too close, false otherwise.
 */
bool checkObstacle() {
  float distanceCm = measureDistance();
  const int OBSTACLE_THRESHOLD = 20;  // Stop if closer than this
  
  Serial.print("Distance: ");
  Serial.print(distanceCm);
  Serial.println(" cm");
  
  // Obstacle found
  if ((distanceCm < OBSTACLE_THRESHOLD) && (distanceCm > 0)) {
    return true;
  }
  return false;
}

/**
 * Reset encoder counters back to 0
 */
void reset_encoders() {
  // Turn off interrupts while resetting
  detachInterrupt(digitalPinToInterrupt(ENCA1));
  detachInterrupt(digitalPinToInterrupt(ENCA2));
  
  r_pos = 0;
  l_pos = 0;
  
  // Turn interrupts back on
  attachInterrupt(digitalPinToInterrupt(ENCA1), rightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA2), leftEncoder, CHANGE);
  
  Serial.println("Encoders reset to 0");
}

/**
 * Stop both motors
 */
void stopMotors() {
  Serial.println("STOPPING MOTORS");
  
  // PWM to 0
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  
  // Set direction pins low
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  
  delay(300);  // Small settle delay
}
