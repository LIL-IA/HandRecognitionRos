/*
 * ESP32 Master Node for TurtleBot Control
 * 
 * This code runs on ESP32 and:
 * 1. Connects to micro-ROS agent (via Serial or WiFi)
 * 2. Subscribes to /cmd_vel topic
 * 3. Sends motor commands to Arduino via Serial
 * 
 * Hardware:
 * - ESP32 Development Board
 * - Serial2 connection to Arduino:
 *   - ESP32 GPIO 17 (TX2) → Arduino RX
 *   - ESP32 GPIO 16 (RX2) → Arduino TX (optional, if Arduino sends data back)
 *   - GND → GND
 * 
 * Dependencies:
 * - micro-ROS library for ESP32
 * - WiFi library (if using WiFi transport)
 */

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/bool.h>

// ========== Configuration ==========
// WiFi Configuration (if using WiFi transport)
#define WIFI_SSID "your_wifi_ssid"
#define WIFI_PASSWORD "your_wifi_password"

// Serial Configuration
#define SERIAL_BAUD 115200

// Hardware Serial for Arduino communication (Serial2 uses GPIO 16/17)
// If using Serial for micro-ROS, use Serial2 for Arduino
// If using WiFi for micro-ROS, can use Serial for Arduino
#define ARDUINO_SERIAL Serial2  // Use Serial2 for Arduino communication

// micro-ROS Configuration
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// ========== Global Variables ==========
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Last command time for timeout detection
unsigned long last_cmd_time = 0;
const unsigned long CMD_TIMEOUT_MS = 1000; // Stop motors if no command for 1 second

// ========== Function Declarations ==========
void error_loop();
void subscription_callback(const void *msgin);
void setup();
void loop();

// ========== Error Handler ==========
void error_loop() {
  while(1) {
    delay(100);
  }
}

// ========== Subscription Callback ==========
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *twist_msg = (const geometry_msgs__msg__Twist *)msgin;
  
  // Update last command time
  last_cmd_time = millis();
  
  // Extract linear and angular velocities
  float linear_vel = twist_msg->linear.x;
  float angular_vel = twist_msg->angular.z;
  
  // Send to Arduino via Serial2 (Hardware Serial)
  // Format: "<linear_vel>,<angular_vel>\n"
  ARDUINO_SERIAL.print(linear_vel, 3);
  ARDUINO_SERIAL.print(",");
  ARDUINO_SERIAL.print(angular_vel, 3);
  ARDUINO_SERIAL.print("\n");
  
  // Debug output (optional)
  // Serial.printf("Received: linear=%.3f, angular=%.3f\n", linear_vel, angular_vel);
}

// ========== Setup ==========
void setup() {
  // Initialize Serial2 for Arduino communication (GPIO 16=RX, 17=TX)
  ARDUINO_SERIAL.begin(SERIAL_BAUD);
  
  // Initialize Serial for micro-ROS (USB Serial)
  Serial.begin(SERIAL_BAUD);
  
  // Set micro-ROS transport
  // Option 1: Serial transport (USB) - uses Serial
  set_microros_transports();
  
  // Option 2: WiFi transport (uncomment if using WiFi) - then can use Serial for Arduino
  // set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, "192.168.1.100", 8888);
  // If using WiFi, change ARDUINO_SERIAL to Serial above
  
  delay(2000);
  
  // Allocate memory
  allocator = rcl_get_default_allocator();
  
  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  // Create node
  RCCHECK(rclc_node_init_default(&node, "esp32_turtlebot_controller", "", &support));
  
  // Create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"));
  
  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  
  // Initialize message
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  
  // Initialize last command time
  last_cmd_time = millis();
  
  Serial.println("ESP32 TurtleBot Controller initialized");
  Serial.println("Waiting for micro-ROS agent...");
}

// ========== Main Loop ==========
void loop() {
  // Spin executor to process incoming messages
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  
  // Safety: Stop motors if no command received for timeout period
  if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
    // Send stop command to Arduino
    ARDUINO_SERIAL.print("0.0,0.0\n");
    last_cmd_time = millis(); // Reset to prevent continuous sending
  }
  
  delay(10);
}

