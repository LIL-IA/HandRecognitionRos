/*
 * Arduino Slave Node for TurtleBot Motor Control
 * 
 * This code runs on Arduino and:
 * 1. Receives motor commands from ESP32 via Serial
 * 2. Controls left and right motors using PWM
 * 3. Implements differential drive kinematics
 * 
 * Hardware:
 * - Arduino Uno/Nano
 * - Motor Driver (L298N, TB6612FNG, or DRV8833)
 * - 2x DC Motors with encoders (optional)
 * 
 * Wiring Example (L298N):
 * - D5: Left motor IN1
 * - D6: Left motor IN2
 * - D9: Left motor ENA (PWM)
 * - D10: Right motor IN3
 * - D11: Right motor IN4
 * - D3: Right motor ENB (PWM)
 */

// ========== Pin Definitions ==========
// Left Motor Pins
const int LEFT_MOTOR_IN1 = 5;   // Forward
const int LEFT_MOTOR_IN2 = 6;   // Backward
const int LEFT_MOTOR_PWM = 9;   // Speed control (PWM)

// Right Motor Pins
const int RIGHT_MOTOR_IN1 = 10;  // Forward
const int RIGHT_MOTOR_IN2 = 11;  // Backward
const int RIGHT_MOTOR_PWM = 3;   // Speed control (PWM)

// ========== Robot Parameters ==========
// Adjust these based on your TurtleBot model
const float WHEEL_RADIUS = 0.033;      // Wheel radius in meters (TurtleBot3 Burger: 0.033m)
const float WHEEL_BASE = 0.160;         // Distance between wheels in meters (TurtleBot3 Burger: 0.160m)
const float MAX_LINEAR_VEL = 0.22;      // Maximum linear velocity (m/s)
const float MAX_ANGULAR_VEL = 2.84;     // Maximum angular velocity (rad/s)

// Motor control parameters
const int MIN_PWM = 0;                  // Minimum PWM value
const int MAX_PWM = 255;                // Maximum PWM value
const int DEADBAND_PWM = 30;            // Minimum PWM to overcome friction

// ========== Global Variables ==========
String serialBuffer = "";
bool newData = false;
float linear_vel = 0.0;
float angular_vel = 0.0;

unsigned long last_cmd_time = 0;
const unsigned long CMD_TIMEOUT_MS = 1000; // Stop if no command for 1 second

// ========== Function Declarations ==========
void setup();
void loop();
void readSerial();
void parseCommand(String data);
void setMotorSpeed(int motor, float speed);
void stopMotors();
float mapVelocityToPWM(float velocity);

// ========== Setup ==========
void setup() {
  // Initialize Serial communication with ESP32
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  
  // Configure motor pins as outputs
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  
  // Initialize motors to stopped
  stopMotors();
  
  // Initialize timing
  last_cmd_time = millis();
  
  Serial.println("Arduino TurtleBot Motor Controller Ready");
  Serial.println("Waiting for commands from ESP32...");
}

// ========== Main Loop ==========
void loop() {
  // Read serial data from ESP32
  readSerial();
  
  // Process command if new data received
  if (newData) {
    parseCommand(serialBuffer);
    newData = false;
    serialBuffer = "";
    last_cmd_time = millis();
  }
  
  // Safety: Stop motors if no command received for timeout period
  if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
    stopMotors();
    last_cmd_time = millis(); // Reset to prevent continuous checking
  }
  
  // Apply motor control
  controlMotors();
  
  delay(10); // Small delay for stability
}

// ========== Read Serial Data ==========
void readSerial() {
  while (Serial.available() > 0) {
    char inChar = Serial.read();
    
    if (inChar == '\n') {
      newData = true;
      break;
    } else if (inChar != '\r') {
      serialBuffer += inChar;
    }
  }
}

// ========== Parse Command ==========
void parseCommand(String data) {
  // Expected format: "linear_vel,angular_vel"
  // Example: "0.15,-0.5"
  
  int commaIndex = data.indexOf(',');
  
  if (commaIndex > 0) {
    String linearStr = data.substring(0, commaIndex);
    String angularStr = data.substring(commaIndex + 1);
    
    linear_vel = linearStr.toFloat();
    angular_vel = angularStr.toFloat();
    
    // Clamp values to safe limits
    linear_vel = constrain(linear_vel, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
    angular_vel = constrain(angular_vel, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);
    
    // Debug output (optional, comment out for production)
    // Serial.print("Received: linear=");
    // Serial.print(linear_vel);
    // Serial.print(", angular=");
    // Serial.println(angular_vel);
  }
}

// ========== Control Motors ==========
void controlMotors() {
  // Differential drive kinematics
  // v_left = linear_vel - (angular_vel * wheel_base / 2)
  // v_right = linear_vel + (angular_vel * wheel_base / 2)
  
  float v_left = linear_vel - (angular_vel * WHEEL_BASE / 2.0);
  float v_right = linear_vel + (angular_vel * WHEEL_BASE / 2.0);
  
  // Convert velocities to wheel speeds (rad/s)
  float omega_left = v_left / WHEEL_RADIUS;
  float omega_right = v_right / WHEEL_RADIUS;
  
  // Convert to motor speeds (normalized -1 to 1)
  float left_speed = omega_left * WHEEL_RADIUS / MAX_LINEAR_VEL;
  float right_speed = omega_right * WHEEL_RADIUS / MAX_LINEAR_VEL;
  
  // Clamp to [-1, 1]
  left_speed = constrain(left_speed, -1.0, 1.0);
  right_speed = constrain(right_speed, -1.0, 1.0);
  
  // Set motor speeds
  setMotorSpeed(0, left_speed);   // 0 = left motor
  setMotorSpeed(1, right_speed);  // 1 = right motor
}

// ========== Set Motor Speed ==========
void setMotorSpeed(int motor, float speed) {
  // speed: -1.0 (full reverse) to +1.0 (full forward)
  
  int pwmValue = mapVelocityToPWM(abs(speed));
  pwmValue = constrain(pwmValue, MIN_PWM, MAX_PWM);
  
  if (motor == 0) { // Left motor
    if (speed > 0.01) {
      // Forward
      digitalWrite(LEFT_MOTOR_IN1, HIGH);
      digitalWrite(LEFT_MOTOR_IN2, LOW);
      analogWrite(LEFT_MOTOR_PWM, pwmValue);
    } else if (speed < -0.01) {
      // Backward
      digitalWrite(LEFT_MOTOR_IN1, LOW);
      digitalWrite(LEFT_MOTOR_IN2, HIGH);
      analogWrite(LEFT_MOTOR_PWM, pwmValue);
    } else {
      // Stop
      digitalWrite(LEFT_MOTOR_IN1, LOW);
      digitalWrite(LEFT_MOTOR_IN2, LOW);
      analogWrite(LEFT_MOTOR_PWM, 0);
    }
  } else if (motor == 1) { // Right motor
    if (speed > 0.01) {
      // Forward
      digitalWrite(RIGHT_MOTOR_IN1, HIGH);
      digitalWrite(RIGHT_MOTOR_IN2, LOW);
      analogWrite(RIGHT_MOTOR_PWM, pwmValue);
    } else if (speed < -0.01) {
      // Backward
      digitalWrite(RIGHT_MOTOR_IN1, LOW);
      digitalWrite(RIGHT_MOTOR_IN2, HIGH);
      analogWrite(RIGHT_MOTOR_PWM, pwmValue);
    } else {
      // Stop
      digitalWrite(RIGHT_MOTOR_IN1, LOW);
      digitalWrite(RIGHT_MOTOR_IN2, LOW);
      analogWrite(RIGHT_MOTOR_PWM, 0);
    }
  }
}

// ========== Map Velocity to PWM ==========
float mapVelocityToPWM(float velocity) {
  // velocity: 0.0 to 1.0
  // Apply deadband to overcome friction
  if (velocity < 0.01) {
    return 0;
  }
  
  // Map from [0, 1] to [DEADBAND_PWM, MAX_PWM]
  float pwm = DEADBAND_PWM + (velocity * (MAX_PWM - DEADBAND_PWM));
  return (int)pwm;
}

// ========== Stop All Motors ==========
void stopMotors() {
  linear_vel = 0.0;
  angular_vel = 0.0;
  
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  analogWrite(LEFT_MOTOR_PWM, 0);
  
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
  analogWrite(RIGHT_MOTOR_PWM, 0);
}

