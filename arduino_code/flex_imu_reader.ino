#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>

#include <Wire.h>
#include "SparkFun_ISM330DHCX.h"

// --- 1. PIN DEFINITIONS (Previously missing) ---
const int flexPins[] = {36, 34, 35, 32, 33}; 

// --- 2. SENSOR OBJECTS ---
SparkFun_ISM330DHCX myISM;

// --- 3. FILTER SETTINGS ---
#define WINDOW_SIZE 10
float rollBuffer[WINDOW_SIZE];
float pitchBuffer[WINDOW_SIZE];
int bufferIdx = 0;

float movingAverage(float newValue, float* buffer) {
  buffer[bufferIdx] = newValue;
  float sum = 0;
  for(int i=0; i<WINDOW_SIZE; i++) sum += buffer[i];
  return sum / WINDOW_SIZE;
}

// --- 4. micro-ROS OBJECTS ---
rcl_publisher_t publisher;
std_msgs__msg__Float32MultiArray msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while(1) {
    delay(100);
  }
}

void setup() {
  // Use Serial Transport for local USB connection
  set_microros_transports();
  
  analogReadResolution(10);
  Wire.begin(21, 22);

  // Initialize ISM330 IMU
  if (!myISM.begin()) error_loop();
  myISM.deviceReset();
  delay(100);
  myISM.setDeviceConfig();
  myISM.setAccelDataRate(ISM_XL_ODR_104Hz);

  // Initialize Filter Buffers
  for(int i=0; i<WINDOW_SIZE; i++) {
    rollBuffer[i] = 0; 
    pitchBuffer[i] = 0;
  }

  delay(2000); // Wait for micro-ROS agent

  allocator = rcl_get_default_allocator();

  // Initialize micro-ROS lifecycle
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "glove_node", "", &support));
  RCCHECK(rclc_publisher_init_default(
    &publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "glove_data"));

  // Pre-allocate message memory (8 values: 5 flex + 3 IMU)
  static float data_memory[8];
  msg.data.capacity = 8;
  msg.data.size = 8;
  msg.data.data = data_memory;
}

void loop() {
  // 1. Read Flex Sensors with Exponential Smoothing
  static float smoothFlex[5] = {0};
  float alpha_flex = 0.2; 
  
  for (int i = 0; i < 5; i++) {
    int raw = analogRead(flexPins[i]);
    float currentVal = (float)constrain(map(raw, 700, 1000, 0, 100), 0, 100) / 100.0;
    smoothFlex[i] = (alpha_flex * currentVal) + ((1.0 - alpha_flex) * smoothFlex[i]);
    msg.data.data[i] = smoothFlex[i];
  }

  // 2. Read IMU and Apply Moving Average Filter
  sfe_ism_data_t acc;
  myISM.getAccel(&acc);
  
  float rawRoll  = atan2((float)acc.yData, (float)acc.zData);
  float rawPitch = atan2(-(float)acc.xData, sqrt((float)acc.yData * acc.yData + (float)acc.zData * acc.zData));
  
  msg.data.data[5] = movingAverage(rawRoll, rollBuffer);
  msg.data.data[6] = movingAverage(rawPitch, pitchBuffer);
  msg.data.data[7] = 0.0; // Yaw
  
  bufferIdx = (bufferIdx + 1) % WINDOW_SIZE;

  // 3. Publish to ROS 2
  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

  delay(15); 
}